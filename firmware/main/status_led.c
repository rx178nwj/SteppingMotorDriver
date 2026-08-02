#include "status_led.h"

#include "ble_telemetry.h"
#include "gpio_config.h"
#include "motor_ctrl.h"
#include "wifi_telemetry.h"

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LED_TASK_PERIOD_MS   10U
#define LED_ACTIVITY_HOLD_MS 500U
#define LED_SLOW_HALF_MS     250U
#define LED_FAST_HALF_US     62500ULL

static volatile TickType_t s_usb_activity_tick;
static volatile TickType_t s_wireless_activity_tick;
static volatile bool s_usb_activity_seen;
static volatile bool s_wireless_activity_seen;

static bool any_axis_faulted(void)
{
    for (uint8_t axis = 0; axis < NUM_AXES; ++axis) {
        axis_status_t status;
        motor_get_status(axis, &status);
        if (status.state == AXIS_FAULT) return true;
    }
    return false;
}

static bool activity_is_recent(bool seen, TickType_t activity_tick,
                               TickType_t now)
{
    return seen &&
           (now - activity_tick) < pdMS_TO_TICKS(LED_ACTIVITY_HOLD_MS);
}

static bool slow_blink_level(TickType_t now)
{
    const TickType_t half_period = pdMS_TO_TICKS(LED_SLOW_HALF_MS);
    return half_period != 0 && ((now / half_period) & 1U) == 0;
}

static bool fast_blink_level(int64_t now_us)
{
    return (((uint64_t)now_us / LED_FAST_HALF_US) & 1ULL) == 0;
}

static void status_led_task(void *arg)
{
    (void)arg;
    for (;;) {
        TickType_t now = xTaskGetTickCount();
        int64_t now_us = esp_timer_get_time();

        bool status1;
        if (any_axis_faulted()) {
            status1 = fast_blink_level(now_us);
        } else if (activity_is_recent(s_usb_activity_seen,
                                      s_usb_activity_tick, now)) {
            status1 = slow_blink_level(now);
        } else {
            status1 = usb_serial_jtag_is_connected();
        }

        bool wireless_error =
            ble_telemetry_get_status() == BLE_TELEMETRY_ERROR ||
            wifi_telemetry_has_error();
        bool wireless_active =
            activity_is_recent(s_wireless_activity_seen,
                               s_wireless_activity_tick, now);
        bool wireless_connected =
            ble_telemetry_get_status() == BLE_TELEMETRY_CONNECTED ||
            wifi_telemetry_has_client();

        bool status2 = wireless_error ? fast_blink_level(now_us) :
                       wireless_active ? slow_blink_level(now) :
                       wireless_connected;

        gpio_set_level(GPIO_STATUS1_LED, status1);
        gpio_set_level(GPIO_STATUS2_LED, status2);
        vTaskDelay(pdMS_TO_TICKS(LED_TASK_PERIOD_MS));
    }
}

esp_err_t status_led_init(void)
{
    gpio_set_level(GPIO_STATUS1_LED, 0);
    gpio_set_level(GPIO_STATUS2_LED, 0);
    return xTaskCreate(status_led_task, "StatusLedTask", 3072, NULL, 5, NULL) ==
                   pdPASS
               ? ESP_OK
               : ESP_ERR_NO_MEM;
}

void status_led_note_usb_activity(void)
{
    s_usb_activity_tick = xTaskGetTickCount();
    s_usb_activity_seen = true;
}

void status_led_note_wireless_activity(void)
{
    s_wireless_activity_tick = xTaskGetTickCount();
    s_wireless_activity_seen = true;
}
