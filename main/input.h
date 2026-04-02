#ifndef FPV_INPUT_H
#define FPV_INPUT_H

#include <stdint.h>

#include "esp_err.h"

#include "configs.h"

/*
 * Input module:
 * - Reads 4 analog axes from ADC1 via adc_oneshot API.
 * - Reads 4 AUX switches with internal pull-up, active-low logic.
 * - Applies EMA filtering and fixed linear calibration to CRSF range.
 * - Builds CRSF channel array [16]:
 *   CH1 Roll, CH2 Pitch, CH3 Throttle, CH4 Yaw,
 *   CH5..CH8 AUX1..AUX4, CH9..CH16 center.
 */

esp_err_t input_init(void);
esp_err_t input_read_channels(uint16_t ch_out[CRSF_NUM_CHANNELS]);
esp_err_t input_run_calibration(void);
void input_log_debug_snapshot(void);
void input_log_pretty(void);

#endif /* FPV_INPUT_H */
