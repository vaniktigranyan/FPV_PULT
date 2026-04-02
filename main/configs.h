#ifndef FPV_CONFIGS_H
#define FPV_CONFIGS_H

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"

#include "crsf.h"

/* -------- System / Modes -------- */
#define calibration_joystick 0      // 1 = run calibration wizard on boot
#define USING_CALIBRATE 1           // 1 = use saved calibration from NVS
#define FPV_VERBOSE_DEBUG 1         // 1 = verbose debug logs (ADC/CRSF)
#define FPV_CRSF_DEADBAND 7         // deadband in CRSF units for CH1..CH4
#define FPV_CENTER_SNAP_ENABLE 1    // 1 = snap Roll/Pitch/Yaw to center
#define FPV_CENTER_SNAP_DELTA 15    // center snap threshold in CRSF units

/* -------- UART / CRSF -------- */
#define FPV_UART_PORT UART_NUM_2        // UART port for CRSF
#define FPV_UART_TX_GPIO GPIO_NUM_17    // ESP32 TX -> FC RX
#define FPV_UART_RX_GPIO GPIO_NUM_16    // ESP32 RX <- FC TX
#define FPV_UART_BAUD_RATE 416666       // CRSF baud rate
#define FPV_UART_RX_BUFFER_SIZE 1024    // UART RX ring buffer size
#define FC_RX_ENABLE 0                  // 1 = enable telemetry RX parsing

/* -------- RC TX timing -------- */
#define FPV_RC_SEND_RATE_HZ 200  // RC packet send rate
#define FPV_RC_SEND_PERIOD_US (1000000 / FPV_RC_SEND_RATE_HZ) // period in microseconds

/* -------- Tasks / Logging -------- */
#define FPV_TASK_STACK_SIZE 4096 // task stack size
#define FPV_TASK_PRIORITY_TX 8   // TX task priority
#define FPV_TASK_PRIORITY_RX 7   // RX task priority
#define FPV_VERBOSE_LOG_EVERY_N_PACKETS 25U // verbose log throttle
#define FPV_INFO_LOG_EVERY_N_PACKETS 150U   // info log throttle

/* -------- ADC / Axes -------- */
#define FPV_ADC_ROLL_GPIO GPIO_NUM_32       // Roll axis analog input
#define FPV_ADC_PITCH_GPIO GPIO_NUM_33      // Pitch axis analog input
#define FPV_ADC_THROTTLE_GPIO GPIO_NUM_34   // Throttle axis analog input
#define FPV_ADC_YAW_GPIO GPIO_NUM_35        // Yaw axis analog input

#define FPV_ADC_ROLL_CH ADC_CHANNEL_4       // ADC1 channel for Roll
#define FPV_ADC_PITCH_CH ADC_CHANNEL_5      // ADC1 channel for Pitch
#define FPV_ADC_THROTTLE_CH ADC_CHANNEL_6   // ADC1 channel for Throttle
#define FPV_ADC_YAW_CH ADC_CHANNEL_7        // ADC1 channel for Yaw

#define FPV_ADC_ATTEN ADC_ATTEN_DB_12           // ADC attenuation
#define FPV_ADC_BITWIDTH ADC_BITWIDTH_DEFAULT   // ADC resolution

#define FPV_ADC_ROLL_MIN 200        // default Roll min raw
#define FPV_ADC_ROLL_MAX 3900       // default Roll max raw
#define FPV_ADC_PITCH_MIN 200       // default Pitch min raw
#define FPV_ADC_PITCH_MAX 3900      // default Pitch max raw
#define FPV_ADC_THROTTLE_MIN 200    // default Throttle min raw
#define FPV_ADC_THROTTLE_MAX 3900   // default Throttle max raw
#define FPV_ADC_YAW_MIN 200         // default Yaw min raw
#define FPV_ADC_YAW_MAX 3900        // default Yaw max raw

#define FPV_ROLL_INVERT 0       // 1 = invert Roll axis
#define FPV_PITCH_INVERT 0      // 1 = invert Pitch axis
#define FPV_THROTTLE_INVERT 0   // 1 = invert Throttle axis
#define FPV_YAW_INVERT 0        // 1 = invert Yaw axis

#define FPV_ADC_EMA_ALPHA 0.20f // EMA filter coefficient

/* -------- Switches / AUX -------- */
#define FPV_SWITCH_AUX1_GPIO GPIO_NUM_21    // AUX1 switch input
#define FPV_SWITCH_AUX2_GPIO GPIO_NUM_22    // AUX2 switch input
#define FPV_SWITCH_AUX3_GPIO GPIO_NUM_23    // AUX3 switch input
#define FPV_SWITCH_AUX4_GPIO GPIO_NUM_19    // AUX4 switch input
#define FPV_SWITCH_ON_LEVEL 0               // ON = LOW (pull-up)

#define FPV_AUX_ON_VALUE CRSF_CH_MAX    // CRSF value when switch is ON
#define FPV_AUX_OFF_VALUE CRSF_CH_MIN   // CRSF value when switch is OFF

/* -------- Buzzer -------- */
#define FPV_BUZZER_GPIO GPIO_NUM_5  // buzzer GPIO
#define FPV_BUZZER_ACTIVE_LEVEL 1   // active level for buzzer
#define FPV_BUZZER_BEEP_MS 80       // beep duration
#define FPV_BUZZER_GAP_MS 60        // gap between beeps
#define FPV_BUZZER_STARTUP_ENABLE 1 // 1 = play startup melody
#define FPV_BUZZER_STARTUP_GAP_MS 80 // gap between melody beeps
#define FPV_BUZZER_STARTUP_BEEP1_MS 60 // melody beep 1 duration
#define FPV_BUZZER_STARTUP_BEEP2_MS 90 // melody beep 2 duration
#define FPV_BUZZER_STARTUP_BEEP3_MS 130 // melody beep 3 duration

/* -------- Calibration (NVS + thresholds) -------- */
#define FPV_CALIB_NAMESPACE "fpv_calib"     // NVS namespace
#define FPV_CALIB_DONE_KEY "calib_done"     // NVS flag key
#define FPV_CALIB_MOVE_THRESHOLD 60         // min change to consider "moved"
#define FPV_CALIB_STABLE_DELTA 8            // max spread for stability window
#define FPV_CALIB_STABLE_SAMPLES 12         // samples required for stability
#define FPV_CALIB_SAMPLE_COUNT 8            // samples per measurement window
#define FPV_CALIB_SAMPLE_DELAY_MS 5         // delay between samples
#define FPV_CALIB_CENTER_STABLE_DELTA 50    // center stability threshold
#define FPV_CALIB_CENTER_STABLE_SAMPLES 15  // stable windows for center

#define FPV_CALIB_CENTER_MIN 1600           // allowed center min
#define FPV_CALIB_CENTER_MAX 2100           // allowed center max
#define FPV_CALIB_MIN_MIN 0                 // allowed min range start
#define FPV_CALIB_MIN_MAX 200               // allowed min range end
#define FPV_CALIB_MAX_MIN 3600              // allowed max range start
#define FPV_CALIB_MAX_MAX 4096              // allowed max range end
#define FPV_CALIB_LOG_EVERY_MS 500          // log interval during calibration
#define FPV_CALIB_EXTREME_STABLE_DELTA 120  // extreme stability threshold
#define FPV_CALIB_EXTREME_STABLE_MS 800     // time in extreme position

#endif /* FPV_CONFIGS_H */
