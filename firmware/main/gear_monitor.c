#include "gear_monitor.h"

#include "comm.h"
#include "config.h"
#include "motor_ctrl.h"
#include "motor_specs.h"

#include <math.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "gear_monitor";

#define GEAR_I2C_PORT              I2C_NUM_0
#define GEAR_BRIDGE_ADDR           0x42
#define GEAR_I2C_FREQ_HZ           400000
#define GEAR_I2C_TIMEOUT_MS        5
#define GEAR_READY_TIMEOUT_MS      200
#define GEAR_READY_POLL_MS         10
#define GEAR_MONITOR_PERIOD_MS     10
#define GEAR_REACQUIRE_MS          30000
#define GEAR_DIAGNOSTIC_PERIOD_MS  1000
#define GEAR_DEGRADED_FAIL_COUNT   5
#define GEAR_UNAVAILABLE_FAIL_COUNT 20

#define REG_WHO_AM_I   0x00
#define REG_VERSION    0x01
#define REG_FAULT      0x04
#define REG_ANGLE_BASE 0x10
#define REG_CONFIG     0x40
#define REG_CH_PRESENT 0x46
#define REG_CMD        0x50

#define WHO_AM_I_VALUE 0xB6
#define EXPECTED_MAJOR 1
#define CMD_CLEAR_FAULT 0x01

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_bridge;
static bool                    s_i2c_initialized;

static SemaphoreHandle_t s_data_mutex;
static gear_state_t s_state = GEAR_STATE_INIT;
static gear_axis_status_t s_axes[NUM_AXES];
static int32_t s_home_step_pos[NUM_AXES];
static bool s_axis_seen[NUM_AXES];
static bool s_deviation_warned[NUM_AXES];
static uint8_t s_comm_fail_count;
static uint8_t s_last_sample_lo;
static bool s_sample_seen;
static int64_t s_next_reacquire_us;
static int64_t s_next_diagnostic_us;
static gear_event_cb_t s_event_cb;

static float normalize_deg(float deg)
{
    deg = fmodf(deg, 360.0f);
    if (deg < 0.0f) deg += 360.0f;
    return deg;
}

static float shortest_delta_deg(float actual, float expected)
{
    float delta = normalize_deg(actual - expected);
    if (delta > 180.0f) delta -= 360.0f;
    return delta;
}

static void emit_event(gear_event_t event, uint8_t axis, float value)
{
    gear_event_cb_t cb = s_event_cb;
    if (cb) cb(event, axis, value);
}

static esp_err_t bridge_read_reg(uint8_t reg, uint8_t *buf, size_t n)
{
    if (!s_bridge || !buf || n == 0) return ESP_ERR_INVALID_STATE;
    return i2c_master_transmit_receive(s_bridge, &reg, 1, buf, n,
                                       GEAR_I2C_TIMEOUT_MS);
}

static esp_err_t bridge_write_reg(uint8_t reg, const uint8_t *data, size_t n)
{
    if (!s_bridge || !data || n == 0 || n > 16) return ESP_ERR_INVALID_ARG;
    uint8_t frame[17];
    frame[0] = reg;
    memcpy(&frame[1], data, n);
    return i2c_master_transmit(s_bridge, frame, n + 1,
                               GEAR_I2C_TIMEOUT_MS);
}

static void gear_i2c_deinit(void)
{
    if (s_bridge && s_bus) {
        i2c_master_bus_rm_device(s_bridge);
    }
    s_bridge = NULL;
    if (s_bus) {
        i2c_del_master_bus(s_bus);
    }
    s_bus = NULL;
    s_i2c_initialized = false;
}

static esp_err_t gear_i2c_init(void)
{
    gear_i2c_deinit();

    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = GEAR_I2C_PORT,
        .sda_io_num = GPIO_I2C_SDA,
        .scl_io_num = GPIO_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (err != ESP_OK) return err;

    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = GEAR_BRIDGE_ADDR,
        .scl_speed_hz = GEAR_I2C_FREQ_HZ,
    };
    err = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_bridge);
    if (err != ESP_OK) {
        gear_i2c_deinit();
        return err;
    }
    s_i2c_initialized = true;
    return ESP_OK;
}

static bool gear_wait_ready(void)
{
    int64_t deadline_us = esp_timer_get_time() + GEAR_READY_TIMEOUT_MS * 1000LL;
    do {
        uint8_t who_am_i = 0;
        if (bridge_read_reg(REG_WHO_AM_I, &who_am_i, 1) == ESP_OK &&
            who_am_i == WHO_AM_I_VALUE) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(GEAR_READY_POLL_MS));
    } while (esp_timer_get_time() < deadline_us);
    return false;
}

static void mark_all_axes_unavailable(void)
{
    bool notify[NUM_AXES] = {false};
    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        s_axes[i].enabled = config_get_gear_enable(i);
        if (s_axes[i].enabled && s_axes[i].ok) notify[i] = true;
        s_axes[i].ok = false;
    }
    xSemaphoreGive(s_data_mutex);

    for (uint8_t i = 0; i < NUM_AXES; i++) {
        if (notify[i]) emit_event(GEAR_EVENT_DEGRADED, i, 0.0f);
    }
}

static void enter_unavailable(bool version_mismatch)
{
    gear_state_t previous;
    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    previous = s_state;
    s_state = version_mismatch ? GEAR_STATE_VERSION_MISMATCH
                               : GEAR_STATE_UNAVAILABLE;
    xSemaphoreGive(s_data_mutex);

    mark_all_axes_unavailable();
    s_next_reacquire_us = esp_timer_get_time() + GEAR_REACQUIRE_MS * 1000LL;
    if (previous == GEAR_STATE_READY || previous == GEAR_STATE_INIT) {
        emit_event(GEAR_EVENT_UNAVAILABLE, 0, 0.0f);
    }
}

static bool configure_ready_bridge(void)
{
    uint8_t version = 0;
    if (bridge_read_reg(REG_VERSION, &version, 1) != ESP_OK) return false;
    if (((version >> 4) & 0x0F) != EXPECTED_MAJOR) {
        ESP_LOGE(TAG, "E014 GEAR_VERSION_MISMATCH: bridge=0x%02x expected major=%u",
                 version, EXPECTED_MAJOR);
        enter_unavailable(true);
        return false;
    }

    uint8_t present = 0;
    if (bridge_read_reg(REG_CH_PRESENT, &present, 1) != ESP_OK) return false;
    if ((present & ((1U << NUM_AXES) - 1U)) != ((1U << NUM_AXES) - 1U)) {
        ESP_LOGW(TAG, "Expected CH0..CH2, CH_PRESENT=0x%02x", present);
    }

    const uint8_t raw_angle_config = 0x00;
    if (bridge_write_reg(REG_CONFIG, &raw_angle_config, 1) != ESP_OK) return false;

    gear_state_t previous;
    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    previous = s_state;
    s_state = GEAR_STATE_READY;
    xSemaphoreGive(s_data_mutex);

    s_comm_fail_count = 0;
    s_sample_seen = false;
    s_next_diagnostic_us = esp_timer_get_time() + GEAR_DIAGNOSTIC_PERIOD_MS * 1000LL;
    ESP_LOGI(TAG, "Bridge ready: version=%u.%u CH_PRESENT=0x%02x",
             (version >> 4) & 0x0F, version & 0x0F, present);
    if (previous == GEAR_STATE_UNAVAILABLE ||
        previous == GEAR_STATE_VERSION_MISMATCH) {
        emit_event(GEAR_EVENT_AVAILABLE, 0, 0.0f);
    }
    return true;
}

static bool acquire_bridge(void)
{
    esp_err_t err = gear_i2c_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "I2C initialization failed: %s", esp_err_to_name(err));
        enter_unavailable(false);
        return false;
    }
    if (!gear_wait_ready()) {
        ESP_LOGW(TAG, "Bridge ready timeout");
        enter_unavailable(false);
        return false;
    }
    if (!configure_ready_bridge()) {
        if (gear_monitor_get_state() != GEAR_STATE_VERSION_MISMATCH) {
            ESP_LOGW(TAG, "Bridge configuration failed");
            enter_unavailable(false);
        }
        return false;
    }
    return true;
}

static void update_deviation(uint8_t axis, gear_axis_status_t *status,
                             const axis_status_t *motor_status,
                             bool motor_status_valid,
                             bool *warn, float *warn_value)
{
    if (!status->home_angle_valid || !status->ok) {
        status->deviation_deg = 0.0f;
        s_deviation_warned[axis] = false;
        return;
    }

    const motor_profile_t *profile = config_get_motor_profile_data(axis);
    float ratio = config_get_gear_ratio(axis);
    if (!motor_status_valid || !profile ||
        profile->full_steps_per_rev == 0 || ratio <= 0.0f) {
        status->deviation_deg =
            shortest_delta_deg(status->angle_deg, status->home_angle_deg);
        return;
    }

    float steps_per_rev =
        (float)profile->full_steps_per_rev * (float)config_get_microstep();
    float expected = status->home_angle_deg +
        ((float)(motor_status->pos - s_home_step_pos[axis]) * 360.0f) /
        (steps_per_rev * ratio);
    status->deviation_deg =
        shortest_delta_deg(status->angle_deg, normalize_deg(expected));

    bool over = config_get_gear_abs_capable(axis) &&
                fabsf(status->deviation_deg) > config_get_gear_deviation_warn();
    if (over && !s_deviation_warned[axis]) {
        s_deviation_warned[axis] = true;
        *warn = true;
        *warn_value = status->deviation_deg;
    } else if (!over) {
        s_deviation_warned[axis] = false;
    }
}

static void update_channels(const uint8_t buf[15])
{
    bool degraded[NUM_AXES] = {false};
    bool recovered[NUM_AXES] = {false};
    bool warn[NUM_AXES] = {false};
    float warn_value[NUM_AXES] = {0.0f};
    axis_status_t motor_status[NUM_AXES];
    bool motor_status_valid[NUM_AXES];
    uint8_t status_lo = buf[12];
    uint8_t sample_lo = buf[14];

    for (uint8_t axis = 0; axis < NUM_AXES; axis++) {
        motor_status_valid[axis] = motor_get_status(axis, &motor_status[axis]);
    }

    if (s_sample_seen && sample_lo == s_last_sample_lo) {
        ESP_LOGD(TAG, "Snapshot not updated (sample=%u)", sample_lo);
    }
    s_last_sample_lo = sample_lo;
    s_sample_seen = true;

    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    for (uint8_t axis = 0; axis < NUM_AXES; axis++) {
        gear_axis_status_t *status = &s_axes[axis];
        bool was_ok = status->ok;
        bool enabled = config_get_gear_enable(axis);
        bool raw_invalid = buf[2 * axis] == 0xFF &&
                           buf[2 * axis + 1] == 0xFF;
        bool now_ok = enabled && !raw_invalid &&
                      ((status_lo & (1U << axis)) != 0);

        status->enabled = enabled;
        status->sample_lo = sample_lo;
        status->ok = now_ok;

        if (enabled && (!s_axis_seen[axis] || was_ok != now_ok)) {
            if (now_ok && s_axis_seen[axis]) recovered[axis] = true;
            if (!now_ok) degraded[axis] = true;
        }
        s_axis_seen[axis] = enabled;

        if (!now_ok) continue;

        uint16_t raw = (uint16_t)buf[2 * axis] |
                       ((uint16_t)buf[2 * axis + 1] << 8);
        raw &= 0x0FFF;
        float raw_deg = (float)raw * 360.0f / 4096.0f;
        float angle = normalize_deg(raw_deg * (float)config_get_gear_dir(axis) -
                                    config_get_gear_offset(axis));

        status->raw_count = raw;
        status->raw_angle_deg = raw_deg;
        status->angle_deg = angle;
        status->motor_equiv_deg = angle * config_get_gear_ratio(axis);
        if (!status->boot_angle_valid) {
            status->boot_angle_deg = angle;
            status->boot_angle_valid = true;
            ESP_LOGI(TAG, "Axis %u boot angle %.2f deg", axis, (double)angle);
        }
        update_deviation(axis, status, &motor_status[axis],
                         motor_status_valid[axis],
                         &warn[axis], &warn_value[axis]);
    }
    xSemaphoreGive(s_data_mutex);

    for (uint8_t axis = 0; axis < NUM_AXES; axis++) {
        if (degraded[axis]) emit_event(GEAR_EVENT_DEGRADED, axis, 0.0f);
        if (recovered[axis]) emit_event(GEAR_EVENT_RECOVERED, axis, 0.0f);
        if (warn[axis]) {
            emit_event(GEAR_EVENT_DEVIATION_WARN, axis, warn_value[axis]);
        }
    }
}

static void handle_comm_failure(void)
{
    if (s_comm_fail_count < UINT8_MAX) s_comm_fail_count++;
    if (s_comm_fail_count == GEAR_DEGRADED_FAIL_COUNT) {
        ESP_LOGW(TAG, "Bridge communication degraded (%u consecutive failures)",
                 s_comm_fail_count);
        mark_all_axes_unavailable();
    }
    if (s_comm_fail_count >= GEAR_UNAVAILABLE_FAIL_COUNT) {
        ESP_LOGE(TAG, "Bridge unavailable (%u consecutive failures)",
                 s_comm_fail_count);
        enter_unavailable(false);
        gear_i2c_deinit();
    }
}

static void read_diagnostics(void)
{
    uint8_t faults[2] = {0};
    if (bridge_read_reg(REG_FAULT, faults, sizeof(faults)) != ESP_OK) return;
    if (faults[0] == 0 && faults[1] == 0) return;

    ESP_LOGW(TAG, "Bridge fault=0x%02x ch_fault=0x%02x",
             faults[0], faults[1]);
    const uint8_t clear = CMD_CLEAR_FAULT;
    if (bridge_write_reg(REG_CMD, &clear, 1) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to clear bridge fault latch");
    }
}

static void gear_monitor_task(void *arg)
{
    (void)arg;
    acquire_bridge();
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        gear_state_t state = gear_monitor_get_state();
        if (state == GEAR_STATE_READY) {
            uint8_t buf[15];
            if (bridge_read_reg(REG_ANGLE_BASE, buf, sizeof(buf)) == ESP_OK) {
                s_comm_fail_count = 0;
                update_channels(buf);
            } else {
                handle_comm_failure();
            }

            int64_t now = esp_timer_get_time();
            if (gear_monitor_get_state() == GEAR_STATE_READY &&
                now >= s_next_diagnostic_us) {
                read_diagnostics();
                s_next_diagnostic_us = now + GEAR_DIAGNOSTIC_PERIOD_MS * 1000LL;
            }
        } else if (esp_timer_get_time() >= s_next_reacquire_us) {
            acquire_bridge();
            last_wake = xTaskGetTickCount();
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(GEAR_MONITOR_PERIOD_MS));
    }
}

void gear_monitor_init(void)
{
    s_data_mutex = xSemaphoreCreateMutex();
    if (!s_data_mutex) {
        ESP_LOGE(TAG, "Failed to create gear data mutex");
        s_state = GEAR_STATE_UNAVAILABLE;
        return;
    }

    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    memset(s_axes, 0, sizeof(s_axes));
    memset(s_home_step_pos, 0, sizeof(s_home_step_pos));
    memset(s_axis_seen, 0, sizeof(s_axis_seen));
    memset(s_deviation_warned, 0, sizeof(s_deviation_warned));
    s_state = GEAR_STATE_INIT;
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        s_axes[i].enabled = config_get_gear_enable(i);
    }
    xSemaphoreGive(s_data_mutex);

    BaseType_t created = xTaskCreate(gear_monitor_task, "GearMonitorTask",
                                     3072, NULL, 15, NULL);
    if (created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create GearMonitorTask");
        enter_unavailable(false);
    }
}

void gear_monitor_register_event_callback(gear_event_cb_t cb)
{
    s_event_cb = cb;
}

gear_state_t gear_monitor_get_state(void)
{
    if (!s_data_mutex) return s_state;
    gear_state_t state;
    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    state = s_state;
    xSemaphoreGive(s_data_mutex);
    return state;
}

const char *gear_monitor_state_name(gear_state_t state)
{
    switch (state) {
    case GEAR_STATE_INIT:             return "INIT";
    case GEAR_STATE_READY:            return "READY";
    case GEAR_STATE_UNAVAILABLE:      return "UNAVAILABLE";
    case GEAR_STATE_VERSION_MISMATCH: return "VERSION_MISMATCH";
    default:                          return "UNKNOWN";
    }
}

bool gear_monitor_get_axis_status(uint8_t axis, gear_axis_status_t *out)
{
    if (axis >= NUM_AXES || !out || !s_data_mutex) return false;
    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    *out = s_axes[axis];
    xSemaphoreGive(s_data_mutex);
    return true;
}

void gear_monitor_mark_home(uint8_t axis)
{
    if (axis >= NUM_AXES || !s_data_mutex) return;

    axis_status_t motor_status;
    if (!motor_get_status(axis, &motor_status)) return;

    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    if (s_state == GEAR_STATE_READY && s_axes[axis].ok) {
        s_axes[axis].home_angle_deg = s_axes[axis].angle_deg;
        s_axes[axis].home_angle_valid = true;
        s_axes[axis].deviation_deg = 0.0f;
        s_home_step_pos[axis] = motor_status.pos;
        s_deviation_warned[axis] = false;
    }
    xSemaphoreGive(s_data_mutex);
}
