#pragma once

#include "esp_err.h"

esp_err_t status_led_init(void);

/* Task-context APIs. Activity is held for one 2 Hz cycle so it is visible. */
void status_led_note_usb_activity(void);
void status_led_note_wireless_activity(void);
