#include "input.h"

#include <limits.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs.h"

/* Configuration macros are centralized in configs.h. */

typedef enum {
    AXIS_ROLL = 0,
    AXIS_PITCH,
    AXIS_THROTTLE,
    AXIS_YAW,
    AXIS_COUNT
} axis_id_t;

typedef struct {
    gpio_num_t gpio;
    adc_channel_t channel;
    int min_raw;
    int max_raw;
    int invert;
} axis_cfg_t;

typedef struct {
    gpio_num_t gpio;
} switch_cfg_t;

static const char *TAG = "input";

static const axis_cfg_t s_axis_cfg[AXIS_COUNT] = {
    { FPV_ADC_ROLL_GPIO, FPV_ADC_ROLL_CH, FPV_ADC_ROLL_MIN, FPV_ADC_ROLL_MAX, FPV_ROLL_INVERT },
    { FPV_ADC_PITCH_GPIO, FPV_ADC_PITCH_CH, FPV_ADC_PITCH_MIN, FPV_ADC_PITCH_MAX, FPV_PITCH_INVERT },
    { FPV_ADC_THROTTLE_GPIO, FPV_ADC_THROTTLE_CH, FPV_ADC_THROTTLE_MIN, FPV_ADC_THROTTLE_MAX, FPV_THROTTLE_INVERT },
    { FPV_ADC_YAW_GPIO, FPV_ADC_YAW_CH, FPV_ADC_YAW_MIN, FPV_ADC_YAW_MAX, FPV_YAW_INVERT },
};

static const switch_cfg_t s_switch_cfg[4] = {
    { FPV_SWITCH_AUX1_GPIO },
    { FPV_SWITCH_AUX2_GPIO },
    { FPV_SWITCH_AUX3_GPIO },
    { FPV_SWITCH_AUX4_GPIO },
};

static adc_oneshot_unit_handle_t s_adc_handle;
static bool s_initialized;
static bool s_filter_initialized;
static int s_last_raw[AXIS_COUNT];
static float s_last_filtered[AXIS_COUNT];
static uint16_t s_last_axis_crsf[AXIS_COUNT];
static uint16_t s_last_channels[CRSF_NUM_CHANNELS];
static int s_calib_min[AXIS_COUNT];
static int s_calib_max[AXIS_COUNT];
static int s_calib_center[AXIS_COUNT];
static int s_calib_invert[AXIS_COUNT];
static int s_calib_center_noise[AXIS_COUNT];
static bool s_calib_loaded;
/* Axis names used in logs and ASCII prompts */

static const char *s_axis_names[AXIS_COUNT] = {
    "ROLL",
    "PITCH",
    "THROTTLE",
    "YAW"
};

static const char *s_calib_keys[AXIS_COUNT][3] = {
    { "roll_min", "roll_max", "roll_ctr" },
    { "pitch_min", "pitch_max", "pitch_ctr" },
    { "throt_min", "throt_max", "throt_ctr" },
    { "yaw_min", "yaw_max", "yaw_ctr" },
};

static const char *s_calib_inv_keys[AXIS_COUNT] = {
    "roll_inv",
    "pitch_inv",
    "throt_inv",
    "yaw_inv",
};
/* Per-axis inversion saved in NVS (0/1). */

typedef enum {
    CAL_DIR_UP = 0,
    CAL_DIR_DOWN,
    CAL_DIR_LEFT,
    CAL_DIR_RIGHT,
    CAL_DIR_CENTER
} cal_dir_t;

typedef enum {
    CAL_KIND_MIN = 0,
    CAL_KIND_MAX,
    CAL_KIND_CENTER
} cal_kind_t;

typedef struct {
    const char *label;
    cal_dir_t dir;
    cal_kind_t kind;
} calib_step_t;

typedef struct {
    const calib_step_t *steps;
    size_t count;
} calib_sequence_t;

static const calib_step_t s_steps_vertical[] = {
    { "Up", CAL_DIR_UP, CAL_KIND_MAX },
    { "Down", CAL_DIR_DOWN, CAL_KIND_MIN },
    { "Center", CAL_DIR_CENTER, CAL_KIND_CENTER },
};

static const calib_step_t s_steps_horizontal[] = {
    { "Right", CAL_DIR_RIGHT, CAL_KIND_MAX },
    { "Left", CAL_DIR_LEFT, CAL_KIND_MIN },
    { "Center", CAL_DIR_CENTER, CAL_KIND_CENTER },
};

static const calib_sequence_t s_axis_sequence[AXIS_COUNT] = {
    [AXIS_ROLL] = { s_steps_horizontal, sizeof(s_steps_horizontal) / sizeof(s_steps_horizontal[0]) },
    [AXIS_PITCH] = { s_steps_vertical, sizeof(s_steps_vertical) / sizeof(s_steps_vertical[0]) },
    [AXIS_THROTTLE] = { s_steps_vertical, sizeof(s_steps_vertical) / sizeof(s_steps_vertical[0]) },
    [AXIS_YAW] = { s_steps_horizontal, sizeof(s_steps_horizontal) / sizeof(s_steps_horizontal[0]) },
};

static const axis_id_t s_calib_order[] = {
    AXIS_YAW,
    AXIS_THROTTLE,
    AXIS_PITCH,
    AXIS_ROLL,
};

/* Clamp integer to [low, high]. Used for ADC and CRSF normalization. */
static int clamp_int(int value, int low, int high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

/* Map raw ADC range to CRSF [172..1811] linearly. */
static uint16_t map_axis_to_crsf(int value, int min_raw, int max_raw, int invert)
{
    int bounded = clamp_int(value, min_raw, max_raw);
    if (invert) {
        bounded = max_raw - (bounded - min_raw);
    }

    const int span = max_raw - min_raw;
    if (span <= 0) {
        return CRSF_CH_MID;
    }

    const int32_t numerator = (int32_t)(bounded - min_raw) * (int32_t)(CRSF_CH_MAX - CRSF_CH_MIN);
    const int32_t mapped = (int32_t)CRSF_CH_MIN + (numerator / span);
    return (uint16_t)clamp_int((int)mapped, (int)CRSF_CH_MIN, (int)CRSF_CH_MAX);
}

/* Map raw ADC to CRSF using separate ranges for below/above center.
 * This preserves symmetric behavior around the calibrated center. */
static uint16_t map_axis_to_crsf_centered(int value, int min_raw, int max_raw, int center_raw, int invert)
{
    int bounded = clamp_int(value, min_raw, max_raw);
    if (invert) {
        bounded = max_raw - (bounded - min_raw);
        center_raw = max_raw - (center_raw - min_raw);
    }

    if ((center_raw <= min_raw) || (center_raw >= max_raw)) {
        return map_axis_to_crsf(bounded, min_raw, max_raw, 0);
    }

    if (bounded <= center_raw) {
        const int span = center_raw - min_raw;
        if (span <= 0) {
            return CRSF_CH_MID;
        }
        const int32_t numerator = (int32_t)(bounded - min_raw) * (int32_t)(CRSF_CH_MID - CRSF_CH_MIN);
        const int32_t mapped = (int32_t)CRSF_CH_MIN + (numerator / span);
        return (uint16_t)clamp_int((int)mapped, (int)CRSF_CH_MIN, (int)CRSF_CH_MID);
    } else {
        const int span = max_raw - center_raw;
        if (span <= 0) {
            return CRSF_CH_MID;
        }
        const int32_t numerator = (int32_t)(bounded - center_raw) * (int32_t)(CRSF_CH_MAX - CRSF_CH_MID);
        const int32_t mapped = (int32_t)CRSF_CH_MID + (numerator / span);
        return (uint16_t)clamp_int((int)mapped, (int)CRSF_CH_MID, (int)CRSF_CH_MAX);
    }
}

/* Configure buzzer GPIO. */
static void buzzer_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << FPV_BUZZER_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(FPV_BUZZER_GPIO, !FPV_BUZZER_ACTIVE_LEVEL);
}

/* Short confirmation beep for each calibration step. */
static void buzzer_beep(void)
{
    gpio_set_level(FPV_BUZZER_GPIO, FPV_BUZZER_ACTIVE_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(FPV_BUZZER_BEEP_MS));
    gpio_set_level(FPV_BUZZER_GPIO, !FPV_BUZZER_ACTIVE_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(FPV_BUZZER_GAP_MS));
}

/* Startup melody (simple on/off pattern, no PWM tones). */
static void buzzer_startup_melody(void)
{
#if FPV_BUZZER_STARTUP_ENABLE
    const int melody_ms[] = {
        FPV_BUZZER_STARTUP_BEEP1_MS,
        FPV_BUZZER_STARTUP_BEEP2_MS,
        FPV_BUZZER_STARTUP_BEEP3_MS
    };

    for (size_t i = 0; i < sizeof(melody_ms) / sizeof(melody_ms[0]); ++i) {
        gpio_set_level(FPV_BUZZER_GPIO, FPV_BUZZER_ACTIVE_LEVEL);
        vTaskDelay(pdMS_TO_TICKS(melody_ms[i]));
        gpio_set_level(FPV_BUZZER_GPIO, !FPV_BUZZER_ACTIVE_LEVEL);
        vTaskDelay(pdMS_TO_TICKS(FPV_BUZZER_STARTUP_GAP_MS));
    }
#endif
}

/* Default calibration from build-time macros (fallback when NVS is empty). */
static void set_default_calibration(void)
{
    s_calib_min[AXIS_ROLL] = FPV_ADC_ROLL_MIN;
    s_calib_max[AXIS_ROLL] = FPV_ADC_ROLL_MAX;
    s_calib_center[AXIS_ROLL] = (FPV_ADC_ROLL_MIN + FPV_ADC_ROLL_MAX) / 2;
    s_calib_invert[AXIS_ROLL] = 0;
    s_calib_center_noise[AXIS_ROLL] = 0;

    s_calib_min[AXIS_PITCH] = FPV_ADC_PITCH_MIN;
    s_calib_max[AXIS_PITCH] = FPV_ADC_PITCH_MAX;
    s_calib_center[AXIS_PITCH] = (FPV_ADC_PITCH_MIN + FPV_ADC_PITCH_MAX) / 2;
    s_calib_invert[AXIS_PITCH] = 0;
    s_calib_center_noise[AXIS_PITCH] = 0;

    s_calib_min[AXIS_THROTTLE] = FPV_ADC_THROTTLE_MIN;
    s_calib_max[AXIS_THROTTLE] = FPV_ADC_THROTTLE_MAX;
    s_calib_center[AXIS_THROTTLE] = (FPV_ADC_THROTTLE_MIN + FPV_ADC_THROTTLE_MAX) / 2;
    s_calib_invert[AXIS_THROTTLE] = 0;
    s_calib_center_noise[AXIS_THROTTLE] = 0;

    s_calib_min[AXIS_YAW] = FPV_ADC_YAW_MIN;
    s_calib_max[AXIS_YAW] = FPV_ADC_YAW_MAX;
    s_calib_center[AXIS_YAW] = (FPV_ADC_YAW_MIN + FPV_ADC_YAW_MAX) / 2;
    s_calib_invert[AXIS_YAW] = 0;
    s_calib_center_noise[AXIS_YAW] = 0;
}

#if USING_CALIBRATE
static esp_err_t load_calibration_from_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(FPV_CALIB_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS open failed, using defaults: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t done = 0;
    if (nvs_get_u8(handle, FPV_CALIB_DONE_KEY, &done) != ESP_OK || done != 1) {
        nvs_close(handle);
        ESP_LOGW(TAG, "Calibration not found in NVS");
        return ESP_ERR_NOT_FOUND;
    }

    for (int axis = 0; axis < AXIS_COUNT; ++axis) {
        int32_t vmin = 0;
        int32_t vmax = 0;
        int32_t vctr = 0;
        int32_t vinv = 0;

        err = nvs_get_i32(handle, s_calib_keys[axis][0], &vmin);
        if (err != ESP_OK) {
            continue;
        }
        err = nvs_get_i32(handle, s_calib_keys[axis][1], &vmax);
        if (err != ESP_OK) {
            continue;
        }
        err = nvs_get_i32(handle, s_calib_keys[axis][2], &vctr);
        if (err != ESP_OK) {
            continue;
        }

        s_calib_min[axis] = (int)vmin;
        s_calib_max[axis] = (int)vmax;
        s_calib_center[axis] = (int)vctr;

        if (nvs_get_i32(handle, s_calib_inv_keys[axis], &vinv) == ESP_OK) {
            s_calib_invert[axis] = (vinv != 0);
        }
    }

    nvs_close(handle);
    s_calib_loaded = true;
    ESP_LOGI(TAG, "Calibration loaded from NVS");
    return ESP_OK;
}
#endif

/* Save calibration data + "calib_done" flag to NVS. */
static esp_err_t save_calibration_to_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(FPV_CALIB_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return err;
    }

    for (int axis = 0; axis < AXIS_COUNT; ++axis) {
        ESP_RETURN_ON_ERROR(nvs_set_i32(handle, s_calib_keys[axis][0], s_calib_min[axis]),
                            TAG, "nvs_set min failed axis=%d", axis);
        ESP_RETURN_ON_ERROR(nvs_set_i32(handle, s_calib_keys[axis][1], s_calib_max[axis]),
                            TAG, "nvs_set max failed axis=%d", axis);
        ESP_RETURN_ON_ERROR(nvs_set_i32(handle, s_calib_keys[axis][2], s_calib_center[axis]),
                            TAG, "nvs_set ctr failed axis=%d", axis);
        ESP_RETURN_ON_ERROR(nvs_set_i32(handle, s_calib_inv_keys[axis], s_calib_invert[axis]),
                            TAG, "nvs_set inv failed axis=%d", axis);
    }

    ESP_RETURN_ON_ERROR(nvs_set_u8(handle, FPV_CALIB_DONE_KEY, 1),
                        TAG, "nvs_set calib_done failed");

    err = nvs_commit(handle);
    nvs_close(handle);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Calibration saved to NVS");
    }
    return err;
}

/* Read single ADC sample for the given axis. */
static int read_axis_raw(axis_id_t axis)
{
    int raw = 0;
    if (adc_oneshot_read(s_adc_handle, s_axis_cfg[axis].channel, &raw) != ESP_OK) {
        return 0;
    }
    return raw;
}

/* Read a window of samples and return avg/min/max for stability checks. */
static void read_axis_window(axis_id_t axis, int samples, int delay_ms, int *avg_out, int *min_out, int *max_out)
{
    int min_v = INT_MAX;
    int max_v = INT_MIN;
    int64_t sum = 0;

    for (int i = 0; i < samples; ++i) {
        int v = read_axis_raw(axis);
        if (v < min_v) {
            min_v = v;
        }
        if (v > max_v) {
            max_v = v;
        }
        sum += v;
        if (delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
    }

    if (avg_out != NULL) {
        *avg_out = (samples > 0) ? (int)(sum / samples) : 0;
    }
    if (min_out != NULL) {
        *min_out = (samples > 0) ? min_v : 0;
    }
    if (max_out != NULL) {
        *max_out = (samples > 0) ? max_v : 0;
    }
}

/* Step 1: wait until all axes are in center range and stable for 3 seconds. */
static void wait_for_center_all_axes(void)
{
    ESP_LOGW(TAG, "Установите стики в центр");

    const int tick_ms = 100;
    const int needed_ms = 3000;
    int stable_ms = 0;
    int log_ms = 0;
    int avg_arr[AXIS_COUNT] = {0};
    int min_arr[AXIS_COUNT] = {0};
    int max_arr[AXIS_COUNT] = {0};

    while (true) {
        bool all_stable = true;
        for (int axis = 0; axis < AXIS_COUNT; ++axis) {
            int avg = 0;
            int min_v = 0;
            int max_v = 0;
            read_axis_window((axis_id_t)axis,
                             FPV_CALIB_SAMPLE_COUNT,
                             FPV_CALIB_SAMPLE_DELAY_MS,
                             &avg,
                             &min_v,
                             &max_v);
            avg_arr[axis] = avg;
            min_arr[axis] = min_v;
            max_arr[axis] = max_v;
            s_calib_center_noise[axis] = max_v - min_v;

            if ((avg < FPV_CALIB_CENTER_MIN) || (avg > FPV_CALIB_CENTER_MAX)) {
                all_stable = false;
            }
            if ((max_v - min_v) > FPV_CALIB_CENTER_STABLE_DELTA) {
                all_stable = false;
            }

            s_calib_center[axis] = avg;
        }

        if (all_stable) {
            stable_ms += tick_ms;
        } else {
            stable_ms = 0;
        }

        if (stable_ms >= needed_ms) {
            ESP_LOGW(TAG, "Center captured for all axes");
            buzzer_beep();
            break;
        }

        log_ms += tick_ms;
        if (log_ms >= FPV_CALIB_LOG_EVERY_MS) {
            for (int axis = 0; axis < AXIS_COUNT; ++axis) {
                ESP_LOGI(TAG, "CENTER %s: avg=%d min=%d max=%d",
                         s_axis_names[axis], avg_arr[axis], min_arr[axis], max_arr[axis]);
            }
            log_ms = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(tick_ms));
    }
}

/* Center step for a single axis. Requires stability for ~1 second. */
static int wait_for_axis_center(axis_id_t axis)
{
    ESP_LOGI(TAG, "Center %s and hold...", s_axis_names[axis]);
    const int tick_ms = 100;
    const int needed_ms = 1000;
    int stable_ms = 0;
    int log_ms = 0;
    while (true) {
        int avg = 0;
        int min_v = 0;
        int max_v = 0;
        read_axis_window(axis,
                         FPV_CALIB_SAMPLE_COUNT,
                         FPV_CALIB_SAMPLE_DELAY_MS,
                         &avg,
                         &min_v,
                         &max_v);
        if ((avg >= FPV_CALIB_CENTER_MIN) && (avg <= FPV_CALIB_CENTER_MAX) &&
            (max_v - min_v) <= FPV_CALIB_CENTER_STABLE_DELTA) {
            stable_ms += tick_ms;
            if (stable_ms >= needed_ms) {
                return avg;
            }
        } else {
            stable_ms = 0;
        }
        log_ms += tick_ms;
        if (log_ms >= FPV_CALIB_LOG_EVERY_MS) {
            ESP_LOGI(TAG, "CENTER %s: avg=%d min=%d max=%d",
                     s_axis_names[axis], avg, min_v, max_v);
            log_ms = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(tick_ms));
    }
}

/* Extreme step (min/max). Waits until ADC is inside the target range and stable. */
static int wait_for_axis_extreme(axis_id_t axis, int baseline, int required_side, bool *above_out)
{
    (void)baseline;
    int last_avg = 0;
    bool detected_above = false;
    int log_ms = 0;
    const int tick_ms = 100;
    int stable_ms = 0;

    while (true) {
        int avg = 0;
        int min_v = 0;
        int max_v = 0;
        read_axis_window(axis,
                         FPV_CALIB_SAMPLE_COUNT,
                         FPV_CALIB_SAMPLE_DELAY_MS,
                         &avg,
                         &min_v,
                         &max_v);

        bool in_min = (avg >= FPV_CALIB_MIN_MIN) && (avg <= FPV_CALIB_MIN_MAX);
        bool in_max = (avg >= FPV_CALIB_MAX_MIN) && (avg <= FPV_CALIB_MAX_MAX);

        if (required_side == 0) {
            /* Accept either min or max range on first step */
            if (!in_min && !in_max) {
                stable_ms = 0;
                vTaskDelay(pdMS_TO_TICKS(60));
                continue;
            }
            detected_above = in_max;
        } else {
            /* required_side >0 => expect max range, <0 => expect min range */
            if (required_side > 0 && !in_max) {
                stable_ms = 0;
                vTaskDelay(pdMS_TO_TICKS(60));
                continue;
            }
            if (required_side < 0 && !in_min) {
                stable_ms = 0;
                vTaskDelay(pdMS_TO_TICKS(60));
                continue;
            }
            detected_above = (required_side > 0);
        }

        if ((max_v - min_v) <= FPV_CALIB_EXTREME_STABLE_DELTA) {
            stable_ms += tick_ms;
            last_avg = avg;
            if (stable_ms >= FPV_CALIB_EXTREME_STABLE_MS) {
                if (above_out != NULL) {
                    *above_out = detected_above;
                }
                return last_avg;
            }
        } else {
            stable_ms = 0;
        }

        log_ms += tick_ms;
        if (log_ms >= FPV_CALIB_LOG_EVERY_MS) {
            ESP_LOGI(TAG, "STEP %s: avg=%d min=%d max=%d in_min=%d in_max=%d",
                     s_axis_names[axis], avg, min_v, max_v, (int)in_min, (int)in_max);
            log_ms = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ASCII-art prompt for user guidance during calibration. */
static void print_joystick_ascii(const char *axis, const calib_step_t *step)
{
    const char *arrow = "o";
    switch (step->dir) {
    case CAL_DIR_UP:
        arrow = "^";
        break;
    case CAL_DIR_DOWN:
        arrow = "v";
        break;
    case CAL_DIR_LEFT:
        arrow = "<";
        break;
    case CAL_DIR_RIGHT:
        arrow = ">";
        break;
    case CAL_DIR_CENTER:
    default:
        arrow = "o";
        break;
    }

    printf("\n[%s] %s\n", axis, step->label);
    printf("  +-----+\n");
    printf("  |  %s  |\n", arrow);
    printf("  |  o  |\n");
    printf("  +-----+\n");
}

/* Calibrate a single axis (min/max/center + auto inversion). */
static esp_err_t calibrate_axis(axis_id_t axis)
{
    int min_val = INT_MAX;
    int max_val = INT_MIN;
    int center_val = 0;
    const int baseline = s_calib_center[axis];
    bool first_above = false;

    ESP_LOGI(TAG, "Calibrating %s", s_axis_names[axis]);

    const calib_sequence_t seq = s_axis_sequence[axis];
    for (size_t i = 0; i < seq.count; ++i) {
        const calib_step_t *step = &seq.steps[i];
        print_joystick_ascii(s_axis_names[axis], step);

        int sample = 0;
        if (step->kind == CAL_KIND_CENTER) {
            sample = wait_for_axis_center(axis);
            center_val = sample;
        } else {
            if (i == 0) {
                ESP_LOGI(TAG, "Move %s %s and hold...", s_axis_names[axis], step->label);
                sample = wait_for_axis_extreme(axis, baseline, 0, &first_above);
                s_calib_invert[axis] = first_above ? 0 : 1;
                if (sample < min_val) {
                    min_val = sample;
                }
                if (sample > max_val) {
                    max_val = sample;
                }
            } else {
                const int required_side = first_above ? -1 : 1;
                ESP_LOGI(TAG, "Move %s %s and hold...", s_axis_names[axis], step->label);
                sample = wait_for_axis_extreme(axis, baseline, required_side, NULL);
                if (sample < min_val) {
                    min_val = sample;
                }
                if (sample > max_val) {
                    max_val = sample;
                }
            }
        }

        ESP_LOGI(TAG, "%s %s = %d", s_axis_names[axis], step->label, sample);
        buzzer_beep();
    }

    if (min_val == INT_MAX || max_val == INT_MIN) {
        ESP_LOGE(TAG, "Calibration failed for %s", s_axis_names[axis]);
        return ESP_FAIL;
    }

    if (min_val > max_val) {
        int tmp = min_val;
        min_val = max_val;
        max_val = tmp;
    }

    s_calib_min[axis] = min_val;
    s_calib_max[axis] = max_val;
    s_calib_center[axis] = center_val;

    ESP_LOGI(TAG, "%s calib: min=%d max=%d center=%d",
             s_axis_names[axis], min_val, max_val, center_val);
    return ESP_OK;
}

/* Initialize ADC1 oneshot channels for all axes. */
static esp_err_t init_adc(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&init_cfg, &s_adc_handle), TAG, "adc unit init failed");

    for (int i = 0; i < AXIS_COUNT; ++i) {
        adc_oneshot_chan_cfg_t chan_cfg = {
            .atten = FPV_ADC_ATTEN,
            .bitwidth = FPV_ADC_BITWIDTH,
        };
        ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc_handle, s_axis_cfg[i].channel, &chan_cfg),
                            TAG, "adc channel config failed idx=%d", i);
    }

    return ESP_OK;
}

/* Configure AUX switches with internal pull-ups. */
static esp_err_t init_switches(void)
{
    for (size_t i = 0; i < sizeof(s_switch_cfg) / sizeof(s_switch_cfg[0]); ++i) {
        gpio_config_t cfg = {
            .pin_bit_mask = (1ULL << s_switch_cfg[i].gpio),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "switch gpio config failed idx=%u", (unsigned)i);
    }
    return ESP_OK;
}

esp_err_t input_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(init_adc(), TAG, "adc init failed");
    ESP_RETURN_ON_ERROR(init_switches(), TAG, "switch init failed");

    memset(s_last_raw, 0, sizeof(s_last_raw));
    memset(s_last_filtered, 0, sizeof(s_last_filtered));
    memset(s_last_axis_crsf, 0, sizeof(s_last_axis_crsf));
    for (size_t i = 0; i < CRSF_NUM_CHANNELS; ++i) {
        s_last_channels[i] = CRSF_CH_MID;
    }

    /* Load defaults, then optionally override from NVS if enabled. */
    set_default_calibration();
    s_calib_loaded = false;
#if USING_CALIBRATE
    if (load_calibration_from_nvs() != ESP_OK) {
        ESP_LOGW(TAG, "Using default calibration (NVS missing or invalid)");
    }
#endif

    s_filter_initialized = false;
    s_initialized = true;

    /* Buzzer is optional; used only for calibration feedback. */
    buzzer_init();
    buzzer_startup_melody();
    ESP_LOGI(TAG, "Input init done (ADC1: GPIO32/33/34/35, switches: GPIO21/22/23/25)");
    return ESP_OK;
}

esp_err_t input_read_channels(uint16_t ch_out[CRSF_NUM_CHANNELS])
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (ch_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Read all axes, apply filtering, then map to CRSF values. */
    for (int axis = 0; axis < AXIS_COUNT; ++axis) {
        int raw = 0;
        ESP_RETURN_ON_ERROR(adc_oneshot_read(s_adc_handle, s_axis_cfg[axis].channel, &raw),
                            TAG, "adc read failed axis=%d", axis);

        s_last_raw[axis] = raw;

        if (!s_filter_initialized) {
            s_last_filtered[axis] = (float)raw;
        } else {
            s_last_filtered[axis] =
                (FPV_ADC_EMA_ALPHA * (float)raw) + ((1.0f - FPV_ADC_EMA_ALPHA) * s_last_filtered[axis]);
        }

        const int filtered_int = (int)(s_last_filtered[axis] + 0.5f);
        /* Use calibrated mapping only if flag is enabled and NVS has valid data. */
        uint16_t mapped = 0;
        if (USING_CALIBRATE && s_calib_loaded) {
            const int invert = s_axis_cfg[axis].invert ^ s_calib_invert[axis];
            mapped = map_axis_to_crsf_centered(filtered_int,
                                               s_calib_min[axis],
                                               s_calib_max[axis],
                                               s_calib_center[axis],
                                               invert);
        } else {
            mapped = map_axis_to_crsf(filtered_int,
                                      s_axis_cfg[axis].min_raw,
                                      s_axis_cfg[axis].max_raw,
                                      s_axis_cfg[axis].invert);
        }

        if (FPV_CENTER_SNAP_ENABLE &&
            (axis == AXIS_ROLL || axis == AXIS_PITCH || axis == AXIS_YAW)) {
            int delta = (int)mapped - (int)CRSF_CH_MID;
            if (delta < 0) {
                delta = -delta;
            }
            if (delta <= FPV_CENTER_SNAP_DELTA) {
                mapped = CRSF_CH_MID;
            }
        }

        if (FPV_CRSF_DEADBAND > 0 && s_filter_initialized) {
            int diff = (int)mapped - (int)s_last_axis_crsf[axis];
            if (diff < 0) {
                diff = -diff;
            }
            if (diff < FPV_CRSF_DEADBAND) {
                mapped = s_last_axis_crsf[axis];
            }
        }
        s_last_axis_crsf[axis] = mapped;
    }
    s_filter_initialized = true;

    s_last_channels[0] = s_last_axis_crsf[AXIS_ROLL];
    s_last_channels[1] = s_last_axis_crsf[AXIS_PITCH];
    s_last_channels[2] = s_last_axis_crsf[AXIS_THROTTLE];
    s_last_channels[3] = s_last_axis_crsf[AXIS_YAW];

    for (size_t i = 0; i < 4; ++i) {
        const int level = gpio_get_level(s_switch_cfg[i].gpio);
        s_last_channels[4 + i] = (level == FPV_SWITCH_ON_LEVEL) ? FPV_AUX_ON_VALUE : FPV_AUX_OFF_VALUE;
    }

    /* CH9..CH16 stay centered. */
    for (size_t i = 8; i < CRSF_NUM_CHANNELS; ++i) {
        s_last_channels[i] = CRSF_CH_MID;
    }

    memcpy(ch_out, s_last_channels, sizeof(s_last_channels));
    return ESP_OK;
}

esp_err_t input_run_calibration(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGW(TAG, "Joystick calibration mode enabled");

    /* Global center capture before per-axis steps. */
    wait_for_center_all_axes();

    /* Axis order: Yaw, Throttle, Pitch, Roll. */
    for (size_t i = 0; i < sizeof(s_calib_order) / sizeof(s_calib_order[0]); ++i) {
        ESP_RETURN_ON_ERROR(calibrate_axis(s_calib_order[i]), TAG, "axis calibration failed");
    }

    ESP_RETURN_ON_ERROR(save_calibration_to_nvs(), TAG, "save calibration failed");
    s_filter_initialized = false;
    ESP_LOGI(TAG, "Calibration complete");
    return ESP_OK;
}

void input_log_debug_snapshot(void)
{
#if FPV_VERBOSE_DEBUG
    printf("-----------------------------------------------------------------------------------------------------\n");
    ESP_LOGI(TAG, "ADC raw     : roll=\t%d\tpitch=\t%d\tthrottle=\t%d\tyaw=\t%d",
             s_last_raw[AXIS_ROLL], s_last_raw[AXIS_PITCH],
             s_last_raw[AXIS_THROTTLE], s_last_raw[AXIS_YAW]);
    // ESP_LOGI(TAG, "ADC filtered: roll=\t%.1f\tpitch=\t%.1f\tthrottle=\t%.1f\tyaw=\t%.1f",
    //          s_last_filtered[AXIS_ROLL], s_last_filtered[AXIS_PITCH],
    //          s_last_filtered[AXIS_THROTTLE], s_last_filtered[AXIS_YAW]);
    ESP_LOGI(TAG, "Axis CRSF   : roll=\t%u\tpitch=\t%u\tthrottle=\t%u\tyaw=\t%u",
             s_last_axis_crsf[AXIS_ROLL], s_last_axis_crsf[AXIS_PITCH],
             s_last_axis_crsf[AXIS_THROTTLE], s_last_axis_crsf[AXIS_YAW]);
    ESP_LOGI(TAG, "CRSF ch1..8 : %u %u %u %u %u %u %u %u",
             s_last_channels[0], s_last_channels[1], s_last_channels[2], s_last_channels[3],
             s_last_channels[4], s_last_channels[5], s_last_channels[6], s_last_channels[7]);
    // ESP_LOGI(TAG, "CRSF ch9..16: %u %u %u %u %u %u %u %u",
    //          s_last_channels[8], s_last_channels[9], s_last_channels[10], s_last_channels[11],
    //          s_last_channels[12], s_last_channels[13], s_last_channels[14], s_last_channels[15]);
    printf("-----------------------------------------------------------------------------------------------------\n");
    
#else
    (void)0;
#endif
}

void input_log_pretty(void)
{
    const uint16_t roll = s_last_channels[0];
    const uint16_t pitch = s_last_channels[1];
    const uint16_t throttle = s_last_channels[2];
    const uint16_t yaw = s_last_channels[3];

    const uint16_t aux1 = s_last_channels[4];
    const uint16_t aux2 = s_last_channels[5];
    const uint16_t aux3 = s_last_channels[6];
    const uint16_t aux4 = s_last_channels[7];

    const char *aux1_state = (aux1 > CRSF_CH_MID) ? "ON" : "OFF";
    const char *aux2_state = (aux2 > CRSF_CH_MID) ? "ON" : "OFF";
    const char *aux3_state = (aux3 > CRSF_CH_MID) ? "ON" : "OFF";
    const char *aux4_state = (aux4 > CRSF_CH_MID) ? "ON" : "OFF";
    
    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, "ROLL:-\t%4u", roll);
    ESP_LOGI(TAG, "PITCH:-\t%4u", pitch);
    ESP_LOGI(TAG, "THROTTLE:-\t%4u", throttle);
    ESP_LOGI(TAG, "YAW:-\t\t%4u", yaw);
    ESP_LOGI(TAG, "SWITCH_AUX1: %s", aux1_state);
    ESP_LOGI(TAG, "SWITCH_AUX2: %s", aux2_state);
    ESP_LOGI(TAG, "SWITCH_AUX3: %s", aux3_state);
    ESP_LOGI(TAG, "SWITCH_AUX4: %s", aux4_state);
    ESP_LOGI(TAG, "==================================================\n");
}
