#include "wifi_telemetry.h"

#include "config.h"
#include "telemetry.h"
#include "status_led.h"

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "mdns.h"

#define TELEMETRY_PORT 4000

static const char *TAG = "wifi_telemetry";
static esp_netif_t *s_netif;
static TaskHandle_t s_reconnect_task;
static volatile bool s_initialized;
static volatile bool s_wifi_started;
static volatile bool s_connected;
static volatile uint32_t s_reconnect_delay_s = 1;
static volatile int s_listen_fd = -1;
static volatile int s_client_fd = -1;
static volatile bool s_error;
static esp_ip4_addr_t s_ip;
static bool s_mdns_initialized;
static bool s_mdns_service_active;

static void close_socket(volatile int *fd)
{
    int value = *fd;
    *fd = -1;
    if (value >= 0) {
        shutdown(value, SHUT_RDWR);
        close(value);
    }
}

static void stop_mdns_service(void)
{
    if (s_mdns_initialized && s_mdns_service_active) {
        (void)mdns_service_remove("_smd-telemetry", "_tcp");
        s_mdns_service_active = false;
    }
}

static void start_mdns_service(void)
{
    if (!s_mdns_initialized) {
        if (mdns_init() != ESP_OK) {
            ESP_LOGW(TAG, "mDNS init failed");
            return;
        }
        s_mdns_initialized = true;
        char hostname[24];
        snprintf(hostname, sizeof(hostname), "smd-%s", telemetry_board_id());
        (void)mdns_hostname_set(hostname);
        (void)mdns_instance_name_set("SteppingMotorDriver Telemetry");
    }
    if (!s_mdns_service_active) {
        mdns_txt_item_t txt[] = {
            {"board_id", telemetry_board_id()},
        };
        if (mdns_service_add("SteppingMotorDriver Telemetry",
                             "_smd-telemetry", "_tcp", TELEMETRY_PORT,
                             txt, 1) == ESP_OK) {
            s_mdns_service_active = true;
        }
    }
}

static esp_err_t apply_station_config(void)
{
    wifi_config_t wifi_config = {0};
    size_t ssid_len = strlen(config_get_wifi_ssid());
    size_t password_len = strlen(config_get_wifi_password());
    if (ssid_len == 0 || ssid_len > sizeof(wifi_config.sta.ssid)) {
        return ESP_ERR_INVALID_ARG;
    }
    memcpy(wifi_config.sta.ssid, config_get_wifi_ssid(), ssid_len);
    memcpy(wifi_config.sta.password, config_get_wifi_password(), password_len);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;
    return esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
}

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *event_data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        s_wifi_started = true;
        if (config_get_wifi_enable()) (void)esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *event = event_data;
        s_connected = false;
        if (config_get_wifi_enable()) {
            s_error = true;
            ESP_LOGW(TAG, "station disconnected reason=%u", event->reason);
        }
        memset(&s_ip, 0, sizeof(s_ip));
        close_socket(&s_client_fd);
        close_socket(&s_listen_fd);
        stop_mdns_service();
        if (config_get_wifi_enable() && s_reconnect_task) {
            xTaskNotifyGive(s_reconnect_task);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = event_data;
        s_ip = event->ip_info.ip;
        s_connected = true;
        s_error = false;
        s_reconnect_delay_s = 1;
        start_mdns_service();
        ESP_LOGI(TAG, "station connected " IPSTR, IP2STR(&s_ip));
    }
}

static void reconnect_task(void *arg)
{
    (void)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (!config_get_wifi_enable()) continue;
        uint32_t delay_s = s_reconnect_delay_s;
        vTaskDelay(pdMS_TO_TICKS(delay_s * 1000U));
        if (!config_get_wifi_enable() || s_connected || !s_wifi_started) continue;
        (void)esp_wifi_connect();
        if (s_reconnect_delay_s < 60) {
            uint32_t next = s_reconnect_delay_s * 2U;
            s_reconnect_delay_s = next > 60 ? 60 : next;
        }
    }
}

static bool send_all(int fd, const char *data, size_t length)
{
    while (length > 0) {
        int sent = send(fd, data, length, 0);
        if (sent <= 0) return false;
        data += sent;
        length -= (size_t)sent;
    }
    return true;
}

static bool send_frame(int fd, const char *type,
                       bool (*formatter)(char *, size_t))
{
    char payload[320];
    char frame[384];
    if (!formatter(payload, sizeof(payload))) return false;
    int n = snprintf(frame, sizeof(frame),
                     "{\"type\":\"%s\",\"data\":%s}\n", type, payload);
    return n > 0 && (size_t)n < sizeof(frame) &&
           send_all(fd, frame, (size_t)n);
}

static bool send_telemetry_cycle(int fd)
{
    return send_frame(fd, "axis_status", telemetry_format_axis_status) &&
           send_frame(fd, "power", telemetry_format_power) &&
           send_frame(fd, "fault", telemetry_format_fault) &&
           send_frame(fd, "gear_angle", telemetry_format_gear_angle);
}

static int create_server_socket(void)
{
    int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (fd < 0) return -1;

    int reuse = 1;
    (void)setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    struct sockaddr_in address = {
        .sin_family = AF_INET,
        .sin_port = htons(TELEMETRY_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(fd, (struct sockaddr *)&address, sizeof(address)) != 0 ||
        listen(fd, 1) != 0) {
        close(fd);
        return -1;
    }
    int flags = fcntl(fd, F_GETFL, 0);
    (void)fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    return fd;
}

static void telemetry_server_task(void *arg)
{
    (void)arg;
    uint8_t discarded[256];
    size_t received_total = 0;

    for (;;) {
        if (!s_connected) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        if (s_listen_fd < 0) {
            s_listen_fd = create_server_socket();
            if (s_listen_fd < 0) {
                s_error = true;
                ESP_LOGW(TAG, "TCP server create failed errno=%d", errno);
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            ESP_LOGI(TAG, "TCP telemetry listening on port %d", TELEMETRY_PORT);
        }

        if (s_client_fd < 0) {
            int client = accept(s_listen_fd, NULL, NULL);
            if (client < 0) {
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
            s_client_fd = client;
            s_error = false;
            received_total = 0;
        }

        /* Refuse any additional client while the single slot is occupied. */
        int extra = accept(s_listen_fd, NULL, NULL);
        if (extra >= 0) close(extra);

        int received = recv(s_client_fd, discarded, sizeof(discarded),
                            MSG_DONTWAIT);
        if (received > 0) {
            received_total += (size_t)received;
            if (received_total > 4096) {
                close_socket(&s_client_fd);
                continue;
            }
        } else if (received == 0) {
            close_socket(&s_client_fd);
            continue;
        } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
            close_socket(&s_client_fd);
            continue;
        }

        if (!send_telemetry_cycle(s_client_fd)) {
            s_error = true;
            close_socket(&s_client_fd);
            continue;
        }
        s_error = false;
        status_led_note_wireless_activity();
        uint8_t rate = config_get_wifi_telemetry_rate();
        vTaskDelay(pdMS_TO_TICKS(1000U / (rate ? rate : 10U)));
    }
}

esp_err_t wifi_telemetry_init(void)
{
    if (s_initialized) return ESP_OK;
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        s_error = true;
        return err;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        s_error = true;
        return err;
    }

    s_netif = esp_netif_create_default_wifi_sta();
    if (!s_netif) {
        s_error = true;
        return ESP_ERR_NO_MEM;
    }
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&init);
    if (err != ESP_OK) {
        s_error = true;
        return err;
    }
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               wifi_event_handler, NULL));
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        s_error = true;
        return err;
    }

    if (xTaskCreate(reconnect_task, "WifiReconnect", 3072, NULL, 8,
                    &s_reconnect_task) != pdPASS ||
        xTaskCreate(telemetry_server_task, "WifiTelemetryTask", 5120, NULL, 8,
                    NULL) != pdPASS) {
        s_error = true;
        return ESP_ERR_NO_MEM;
    }
    s_initialized = true;

    if (config_get_wifi_enable()) {
        err = apply_station_config();
        if (err == ESP_OK) err = esp_wifi_start();
        s_error = err != ESP_OK;
        return err;
    }
    return ESP_OK;
}

esp_err_t wifi_telemetry_set_enabled(bool enabled)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (enabled) {
        esp_err_t err = apply_station_config();
        if (err == ESP_OK) {
            if (!s_wifi_started) {
                err = esp_wifi_start();
            } else {
                err = esp_wifi_connect();
            }
        }
        s_error = err != ESP_OK;
        return err;
    }

    close_socket(&s_client_fd);
    close_socket(&s_listen_fd);
    stop_mdns_service();
    s_connected = false;
    s_error = false;
    memset(&s_ip, 0, sizeof(s_ip));
    if (s_wifi_started) {
        (void)esp_wifi_disconnect();
        esp_err_t err = esp_wifi_stop();
        s_wifi_started = false;
        return err;
    }
    return ESP_OK;
}

esp_err_t wifi_telemetry_reconfigure(void)
{
    if (!s_initialized || !config_get_wifi_enable()) return ESP_OK;
    (void)esp_wifi_disconnect();
    esp_err_t err = apply_station_config();
    if (err == ESP_OK && s_wifi_started) err = esp_wifi_connect();
    s_error = err != ESP_OK;
    return err;
}

wifi_telemetry_status_t wifi_telemetry_get_status(void)
{
    if (!config_get_wifi_enable()) return WIFI_TELEMETRY_DISABLED;
    return s_connected ? WIFI_TELEMETRY_CONNECTED
                       : WIFI_TELEMETRY_DISCONNECTED;
}

void wifi_telemetry_format_status(char *buf, size_t size)
{
    wifi_telemetry_status_t status = wifi_telemetry_get_status();
    if (status == WIFI_TELEMETRY_DISABLED) {
        snprintf(buf, size, "DISABLED");
    } else if (status == WIFI_TELEMETRY_DISCONNECTED) {
        snprintf(buf, size, "DISCONNECTED");
    } else {
        wifi_ap_record_t ap;
        int rssi = 0;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) rssi = ap.rssi;
        snprintf(buf, size, "CONNECTED " IPSTR " RSSI=%d",
                 IP2STR(&s_ip), rssi);
    }
}

bool wifi_telemetry_has_client(void)
{
    return s_client_fd >= 0;
}

bool wifi_telemetry_has_error(void)
{
    return s_error;
}
