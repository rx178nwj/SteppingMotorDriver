#include "ble_telemetry.h"

#include "config.h"
#include "gear_monitor.h"
#include "telemetry.h"
#include "status_led.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_att.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "os/os_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

/* Public UUIDs, canonical form:
 * service 7c9e1000-6a3d-4b89-a712-5f4e8d2c0100
 * chars   7c9e1001..1005-6a3d-4b89-a712-5f4e8d2c0100
 */
#define SMD_UUID(value) BLE_UUID128_INIT(                              \
    0x00, 0x01, 0x2c, 0x8d, 0x4e, 0x5f, 0x12, 0xa7,                  \
    0x89, 0x4b, 0x3d, 0x6a, (uint8_t)((value) & 0xff),                \
    (uint8_t)(((value) >> 8) & 0xff), 0x9e, 0x7c)

enum telemetry_characteristic {
    CHR_DEVICE_INFO,
    CHR_AXIS_STATUS,
    CHR_POWER,
    CHR_FAULT,
    CHR_GEAR_ANGLE,
};

static const char *TAG = "ble_telemetry";
static const ble_uuid128_t s_service_uuid = SMD_UUID(0x1000);
static const ble_uuid128_t s_device_info_uuid = SMD_UUID(0x1001);
static const ble_uuid128_t s_axis_status_uuid = SMD_UUID(0x1002);
static const ble_uuid128_t s_power_uuid = SMD_UUID(0x1003);
static const ble_uuid128_t s_fault_uuid = SMD_UUID(0x1004);
static const ble_uuid128_t s_gear_uuid = SMD_UUID(0x1005);

static uint16_t s_device_info_handle;
static uint16_t s_axis_status_handle;
static uint16_t s_power_handle;
static uint16_t s_fault_handle;
static uint16_t s_gear_handle;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool s_notify_axis;
static bool s_notify_power;
static bool s_notify_fault;
static bool s_notify_gear;
static bool s_synced;
static bool s_encrypted;
static bool s_initialized;
static ble_telemetry_status_t s_status = BLE_TELEMETRY_DISABLED;
static char s_device_name[18];
static char s_device_name_short[9];

void ble_store_config_init(void);

static int gap_event(struct ble_gap_event *event, void *arg);
static void start_advertising(void);

static void delete_peer_bond(uint16_t conn_handle, const char *reason)
{
    struct ble_gap_conn_desc desc;
    int rc = ble_gap_conn_find(conn_handle, &desc);
    if (rc != 0) {
        ESP_LOGW(TAG, "cannot inspect peer bond (%s), conn=%u rc=%d",
                 reason, conn_handle, rc);
        return;
    }

    rc = ble_store_util_delete_peer(&desc.peer_id_addr);
    if (rc == 0) {
        ESP_LOGW(TAG, "deleted stale peer bond (%s), conn=%u",
                 reason, conn_handle);
    } else {
        ESP_LOGW(TAG, "peer bond delete failed (%s), conn=%u rc=%d",
                 reason, conn_handle, rc);
    }
}

static bool format_characteristic(enum telemetry_characteristic chr,
                                  char *buf, size_t size)
{
    switch (chr) {
    case CHR_DEVICE_INFO: return telemetry_format_device_info(buf, size);
    case CHR_AXIS_STATUS: return telemetry_format_axis_status(buf, size);
    case CHR_POWER:       return telemetry_format_power(buf, size);
    case CHR_FAULT:       return telemetry_format_fault(buf, size);
    case CHR_GEAR_ANGLE:  return telemetry_format_gear_angle(buf, size);
    default:              return false;
    }
}

static int characteristic_access(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }

    char json[320];
    if (!format_characteristic((enum telemetry_characteristic)(intptr_t)arg,
                               json, sizeof(json))) {
        ESP_LOGE(TAG, "GATT read format failed, conn=%u attr=%u chr=%d",
                 conn_handle, attr_handle, (int)(intptr_t)arg);
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    size_t length = strlen(json);
    int rc = os_mbuf_append(ctxt->om, json, length);
    ESP_LOGI(TAG, "GATT read, conn=%u attr=%u chr=%d len=%u rc=%d",
             conn_handle, attr_handle, (int)(intptr_t)arg,
             (unsigned)length, rc);
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

#define READ_ENCRYPTED (BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC)
#define READ_NOTIFY_ENCRYPTED (READ_ENCRYPTED | BLE_GATT_CHR_F_NOTIFY)

static const struct ble_gatt_svc_def s_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &s_device_info_uuid.u,
                .access_cb = characteristic_access,
                .arg = (void *)(intptr_t)CHR_DEVICE_INFO,
                .val_handle = &s_device_info_handle,
                .flags = READ_ENCRYPTED,
            },
            {
                .uuid = &s_axis_status_uuid.u,
                .access_cb = characteristic_access,
                .arg = (void *)(intptr_t)CHR_AXIS_STATUS,
                .val_handle = &s_axis_status_handle,
                .flags = READ_NOTIFY_ENCRYPTED,
            },
            {
                .uuid = &s_power_uuid.u,
                .access_cb = characteristic_access,
                .arg = (void *)(intptr_t)CHR_POWER,
                .val_handle = &s_power_handle,
                .flags = READ_NOTIFY_ENCRYPTED,
            },
            {
                .uuid = &s_fault_uuid.u,
                .access_cb = characteristic_access,
                .arg = (void *)(intptr_t)CHR_FAULT,
                .val_handle = &s_fault_handle,
                .flags = READ_NOTIFY_ENCRYPTED,
            },
            {
                .uuid = &s_gear_uuid.u,
                .access_cb = characteristic_access,
                .arg = (void *)(intptr_t)CHR_GEAR_ANGLE,
                .val_handle = &s_gear_handle,
                .flags = READ_NOTIFY_ENCRYPTED,
            },
            {0},
        },
    },
    {0},
};

static void notify_json(uint16_t handle, enum telemetry_characteristic chr)
{
    char json[320];
    if (!format_characteristic(chr, json, sizeof(json))) return;
    struct os_mbuf *om = ble_hs_mbuf_from_flat(json, strlen(json));
    if (!om) return;
    int rc = ble_gatts_notify_custom(s_conn_handle, handle, om);
    if (rc != 0 && rc != BLE_HS_ENOTCONN) {
        ESP_LOGD(TAG, "notify handle=%u failed rc=%d", handle, rc);
    } else if (rc == 0) {
        status_led_note_wireless_activity();
    }
}

static void notification_task(void *arg)
{
    (void)arg;
    char last_fault[128] = "";
    unsigned fault_keepalive_ticks = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (s_status != BLE_TELEMETRY_CONNECTED || !s_encrypted) continue;

        if (s_notify_axis) notify_json(s_axis_status_handle, CHR_AXIS_STATUS);
        if (s_notify_power) notify_json(s_power_handle, CHR_POWER);
        if (s_notify_gear && gear_monitor_get_state() == GEAR_STATE_READY) {
            notify_json(s_gear_handle, CHR_GEAR_ANGLE);
        }

        char fault[128];
        if (telemetry_format_fault(fault, sizeof(fault))) {
            bool changed = strcmp(fault, last_fault) != 0;
            if (s_notify_fault && (changed || fault_keepalive_ticks >= 10)) {
                notify_json(s_fault_handle, CHR_FAULT);
                fault_keepalive_ticks = 0;
            } else {
                fault_keepalive_ticks++;
            }
            if (changed) {
                memcpy(last_fault, fault, strlen(fault) + 1);
            }
        }
    }
}

static void start_advertising(void)
{
    if (!s_synced || !config_get_ble_enable() ||
        s_conn_handle != BLE_HS_CONN_HANDLE_NONE || ble_gap_adv_active()) {
        return;
    }

    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = (ble_uuid128_t *)&s_service_uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;
    /* Windows' Settings > Bluetooth "Add device" list often scans
     * passively and never fetches the scan response, so a device with
     * no name in the primary PDU shows up unlabeled and is easy to miss
     * or fails to appear at all. A shortened name (fits the leftover
     * ~10 bytes after flags + 128-bit service UUID in the 31-byte
     * legacy PDU) keeps it identifiable; the full name still goes in
     * the scan response below.
     */
    fields.name = (uint8_t *)s_device_name_short;
    fields.name_len = strlen(s_device_name_short);
    fields.name_is_complete = 0;
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "advertising data failed rc=%d", rc);
        s_status = BLE_TELEMETRY_ERROR;
        return;
    }

    struct ble_hs_adv_fields response = {0};
    response.name = (uint8_t *)s_device_name;
    response.name_len = strlen(s_device_name);
    response.name_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&response);
    if (rc != 0) {
        ESP_LOGE(TAG, "scan response failed rc=%d", rc);
        s_status = BLE_TELEMETRY_ERROR;
        return;
    }

    uint8_t own_addr_type;
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        s_status = BLE_TELEMETRY_ERROR;
        return;
    }

    struct ble_gap_adv_params params = {0};
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    params.itvl_min = 160; /* 100 ms */
    params.itvl_max = 320; /* 200 ms */
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &params, gap_event, NULL);
    if (rc == 0) {
        s_status = BLE_TELEMETRY_ADVERTISING;
    } else {
        ESP_LOGE(TAG, "advertising start failed rc=%d", rc);
        s_status = BLE_TELEMETRY_ERROR;
    }
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status != 0) {
            start_advertising();
            break;
        }
        s_conn_handle = event->connect.conn_handle;
        s_status = BLE_TELEMETRY_CONNECTED;
        s_encrypted = false;
        {
            struct ble_gap_upd_params params = {
                .itvl_min = 24, .itvl_max = 40, /* 30-50 ms */
                .latency = 0, .supervision_timeout = 400,
                .min_ce_len = 0, .max_ce_len = 0,
            };
            (void)ble_gap_update_params(s_conn_handle, &params);
        }
        {
            int rc = ble_gap_security_initiate(s_conn_handle);
            if (rc == 0) {
                ESP_LOGI(TAG, "BLE security initiated, conn=%u",
                         s_conn_handle);
            } else if (rc == BLE_HS_EALREADY) {
                ESP_LOGI(TAG, "BLE security already in progress, conn=%u",
                         s_conn_handle);
            } else {
                ESP_LOGE(TAG, "BLE security initiation failed, conn=%u rc=%d",
                         s_conn_handle, rc);
                delete_peer_bond(s_conn_handle, "security initiation failed");
                (void)ble_gap_terminate(s_conn_handle,
                                        BLE_ERR_REM_USER_CONN_TERM);
            }
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_encrypted = false;
        s_notify_axis = s_notify_power = s_notify_fault = s_notify_gear = false;
        s_status = config_get_ble_enable()
                       ? BLE_TELEMETRY_ADVERTISING : BLE_TELEMETRY_DISABLED;
        start_advertising();
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        start_advertising();
        break;

    case BLE_GAP_EVENT_ENC_CHANGE:
        s_encrypted = event->enc_change.status == 0;
        if (s_encrypted) {
            struct ble_gap_conn_desc desc;
            int rc = ble_gap_conn_find(s_conn_handle, &desc);
            if (rc == 0) {
                ESP_LOGI(TAG,
                         "BLE security state, conn=%u encrypted=%u "
                         "authenticated=%u bonded=%u key_size=%u",
                         s_conn_handle, desc.sec_state.encrypted,
                         desc.sec_state.authenticated, desc.sec_state.bonded,
                         desc.sec_state.key_size);
            } else {
                ESP_LOGW(TAG, "BLE link encrypted but state lookup failed, "
                         "conn=%u rc=%d", s_conn_handle, rc);
            }
        } else {
            ESP_LOGE(TAG, "BLE encryption failed, conn=%u status=%d",
                     s_conn_handle, event->enc_change.status);
            delete_peer_bond(s_conn_handle, "encryption failed");
            (void)ble_gap_terminate(s_conn_handle,
                                    BLE_ERR_REM_USER_CONN_TERM);
        }
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_axis_status_handle) {
            s_notify_axis = event->subscribe.cur_notify;
        } else if (event->subscribe.attr_handle == s_power_handle) {
            s_notify_power = event->subscribe.cur_notify;
        } else if (event->subscribe.attr_handle == s_fault_handle) {
            s_notify_fault = event->subscribe.cur_notify;
        } else if (event->subscribe.attr_handle == s_gear_handle) {
            s_notify_gear = event->subscribe.cur_notify;
        }
        break;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
        ESP_LOGW(TAG, "repeat pairing requested, conn=%u",
                 event->repeat_pairing.conn_handle);
        delete_peer_bond(event->repeat_pairing.conn_handle,
                         "repeat pairing requested");
        return BLE_GAP_REPEAT_PAIRING_RETRY;

    default:
        break;
    }
    return 0;
}

static void on_sync(void)
{
    s_synced = true;
    start_advertising();
}

static void on_reset(int reason)
{
    ESP_LOGE(TAG, "NimBLE host reset reason=%d", reason);
    s_synced = false;
    s_status = BLE_TELEMETRY_ERROR;
}

static void host_task(void *arg)
{
    (void)arg;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t ble_telemetry_init(void)
{
    if (s_initialized) return ESP_OK;

    snprintf(s_device_name, sizeof(s_device_name), "SMD-%s",
             telemetry_board_id());
    {
        const char *board_id = telemetry_board_id();
        size_t len = strlen(board_id);
        const char *suffix = len > 4 ? board_id + (len - 4) : board_id;
        snprintf(s_device_name_short, sizeof(s_device_name_short), "SMD-%s", suffix);
    }
    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        s_status = BLE_TELEMETRY_ERROR;
        return err;
    }

    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;
    /*
     * Just Works has no MITM authentication.  NimBLE's SC-only mode promotes
     * every encrypted GATT attribute to security level 4, which would reject
     * this valid encrypted-but-unauthenticated pairing.  Keep Secure
     * Connections enabled, but allow the level-2 Just Works link required by
     * this headless board.
     */
    ble_hs_cfg.sm_sc_only = 0;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC |
                                 BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC |
                                   BLE_SM_PAIR_KEY_DIST_ID;
    ble_store_config_init();

    ble_svc_gap_init();
    ble_svc_gatt_init();
    int rc = ble_gatts_count_cfg(s_services);
    if (rc == 0) rc = ble_gatts_add_svcs(s_services);
    if (rc == 0) rc = ble_svc_gap_device_name_set(s_device_name);
    if (rc == 0) rc = ble_att_set_preferred_mtu(185);
    if (rc != 0) {
        ESP_LOGE(TAG, "GATT initialization failed rc=%d", rc);
        s_status = BLE_TELEMETRY_ERROR;
        return ESP_FAIL;
    }

    s_status = config_get_ble_enable()
                   ? BLE_TELEMETRY_ADVERTISING : BLE_TELEMETRY_DISABLED;
    s_initialized = true;
    nimble_port_freertos_init(host_task);
    if (xTaskCreate(notification_task, "BleTask", 4096, NULL, 8, NULL) != pdPASS) {
        s_status = BLE_TELEMETRY_ERROR;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t ble_telemetry_set_enabled(bool enabled)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (!enabled) {
        if (ble_gap_adv_active()) (void)ble_gap_adv_stop();
        if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            (void)ble_gap_terminate(s_conn_handle,
                                    BLE_ERR_REM_USER_CONN_TERM);
        }
        s_status = BLE_TELEMETRY_DISABLED;
    } else {
        start_advertising();
    }
    return ESP_OK;
}

ble_telemetry_status_t ble_telemetry_get_status(void)
{
    return s_status;
}
