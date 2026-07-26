#pragma once

#include <stdbool.h>
#include "esp_err.h"

typedef enum {
    BLE_TELEMETRY_DISABLED = 0,
    BLE_TELEMETRY_ADVERTISING,
    BLE_TELEMETRY_CONNECTED,
    BLE_TELEMETRY_ERROR,
} ble_telemetry_status_t;

esp_err_t ble_telemetry_init(void);
esp_err_t ble_telemetry_set_enabled(bool enabled);
ble_telemetry_status_t ble_telemetry_get_status(void);

