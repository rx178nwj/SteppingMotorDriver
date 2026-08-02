#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

typedef enum {
    WIFI_TELEMETRY_DISABLED = 0,
    WIFI_TELEMETRY_DISCONNECTED,
    WIFI_TELEMETRY_CONNECTED,
} wifi_telemetry_status_t;

esp_err_t wifi_telemetry_init(void);
esp_err_t wifi_telemetry_set_enabled(bool enabled);
esp_err_t wifi_telemetry_reconfigure(void);
wifi_telemetry_status_t wifi_telemetry_get_status(void);
void wifi_telemetry_format_status(char *buf, size_t size);
bool wifi_telemetry_has_client(void);
bool wifi_telemetry_has_error(void);
