#pragma once

#include <stdbool.h>
#include <stddef.h>

/* Shared JSON serializers for USB, BLE and WiFi telemetry paths. */
const char *telemetry_board_id(void);
bool telemetry_format_device_info(char *buf, size_t size);
bool telemetry_format_axis_status(char *buf, size_t size);
bool telemetry_format_power(char *buf, size_t size);
bool telemetry_format_fault(char *buf, size_t size);
bool telemetry_format_gear_angle(char *buf, size_t size);

