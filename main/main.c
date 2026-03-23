#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "crsf.h"
#include "crsf_parser.h"
#include "input.h"
#include "telemetry.h"

/* UART CRSF defaults */
#define FPV_UART_PORT                    UART_NUM_2
#define FPV_UART_TX_GPIO                 GPIO_NUM_17
#define FPV_UART_RX_GPIO                 GPIO_NUM_16
#define FPV_UART_BAUD_RATE               416666
#define FPV_UART_RX_BUFFER_SIZE          1024

/* Enable/disable telemetry RX parsing (CRSF RX). */
#ifndef FC_RX_ENABLE
#define FC_RX_ENABLE                     0
#endif

/* RC frame transmit timing */
#define FPV_RC_SEND_RATE_HZ              150
#define FPV_RC_SEND_PERIOD_US            (1000000 / FPV_RC_SEND_RATE_HZ)

/* Task settings */
#define FPV_TASK_STACK_SIZE              4096
#define FPV_TASK_PRIORITY_TX             8
#define FPV_TASK_PRIORITY_RX             7

/* Debug log throttling */
#define FPV_VERBOSE_LOG_EVERY_N_PACKETS  25U
#define FPV_INFO_LOG_EVERY_N_PACKETS     150U

static const char *TAG = "fpv_main";
static TaskHandle_t s_rc_tx_task_handle;
static esp_timer_handle_t s_rc_timer;

static void rc_timer_callback(void *arg)
{
    TaskHandle_t tx_task = (TaskHandle_t)arg;
    if (tx_task != NULL) {
        xTaskNotifyGive(tx_task);
    }
}

static esp_err_t uart_crsf_init(void)
{
    const uart_config_t uart_cfg = {
        .baud_rate = FPV_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(uart_driver_install(FPV_UART_PORT, FPV_UART_RX_BUFFER_SIZE, 0, 0, NULL, 0),
                        TAG, "uart_driver_install failed");
    ESP_RETURN_ON_ERROR(uart_param_config(FPV_UART_PORT, &uart_cfg), TAG, "uart_param_config failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(FPV_UART_PORT,
                                     FPV_UART_TX_GPIO,
                                     FPV_UART_RX_GPIO,
                                     UART_PIN_NO_CHANGE,
                                     UART_PIN_NO_CHANGE),
                        TAG, "uart_set_pin failed");

#if !FC_RX_ENABLE
    ESP_RETURN_ON_ERROR(uart_disable_rx_intr(FPV_UART_PORT), TAG, "uart_disable_rx_intr failed");
#endif

    ESP_LOGI(TAG, "CRSF UART configured: UART2 TX=%d RX=%d baud=%d",
             FPV_UART_TX_GPIO, FPV_UART_RX_GPIO, FPV_UART_BAUD_RATE);
    return ESP_OK;
}

static void rc_tx_task(void *arg)
{
    (void)arg;

    uint16_t channels[CRSF_NUM_CHANNELS] = {0};
    uint8_t packet[CRSF_RC_PACKET_SIZE] = {0};
    size_t packet_len = 0;
    uint32_t packet_counter = 0;
    s_rc_tx_task_handle = xTaskGetCurrentTaskHandle();

    const esp_timer_create_args_t timer_args = {
        .callback = rc_timer_callback,
        .arg = s_rc_tx_task_handle,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "rc_tx_150hz",
        .skip_unhandled_events = true,
    };

    esp_err_t err = esp_timer_create(&timer_args, &s_rc_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_timer_create failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    err = esp_timer_start_periodic(s_rc_timer, FPV_RC_SEND_PERIOD_US);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_timer_start_periodic failed: %s", esp_err_to_name(err));
        (void)esp_timer_delete(s_rc_timer);
        s_rc_timer = NULL;
        vTaskDelete(NULL);
        return;
    }

    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        err = input_read_channels(channels);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "input_read_channels failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (!crsf_build_rc_channels_packet(channels, packet, sizeof(packet), &packet_len)) {
            ESP_LOGE(TAG, "Failed to build CRSF RC packet");
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        const int written = uart_write_bytes(FPV_UART_PORT, (const char *)packet, packet_len);
        if (written < 0) {
            ESP_LOGE(TAG, "uart_write_bytes failed");
            continue;
        }

        if ((packet_counter % FPV_INFO_LOG_EVERY_N_PACKETS) == 0U) {
            //input_log_pretty();
        }

#if FPV_VERBOSE_DEBUG
        if ((packet_counter % FPV_VERBOSE_LOG_EVERY_N_PACKETS) == 0U) {
            input_log_debug_snapshot();
            ESP_LOGI(TAG, "CRSF TX packet len=%u ch1..8=%u %u %u %u %u %u %u %u",
                     (unsigned)packet_len,
                     channels[0], channels[1], channels[2], channels[3],
                     channels[4], channels[5], channels[6], channels[7]);
        }
#endif
        packet_counter++;
    }
}

static void crsf_rx_task(void *arg)
{
    (void)arg;

    crsf_parser_t parser;
    crsf_frame_t frame;
    uint8_t rx_buf[128];

    crsf_parser_init(&parser);

    for (;;) {
        const int len = uart_read_bytes(FPV_UART_PORT, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(20));
        if (len < 0) {
            ESP_LOGE(TAG, "uart_read_bytes failed");
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        for (int i = 0; i < len; ++i) {
            if (crsf_parser_process_byte(&parser, rx_buf[i], &frame)) {
                telemetry_handle_frame(&frame);
            }
        }
    }
}

void app_main(void)
{
    /* Wiring:
     * ESP32 TX (GPIO17) -> FC RX
     * ESP32 RX (GPIO16) -> FC TX
     * GND -> GND
     */
    ESP_LOGI(TAG, "Starting wired CRSF controller");

    esp_err_t err = nvs_flash_init();
    if ((err == ESP_ERR_NVS_NO_FREE_PAGES) || (err == ESP_ERR_NVS_NEW_VERSION_FOUND)) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(err);
    }

    ESP_ERROR_CHECK(uart_crsf_init());
    ESP_ERROR_CHECK(input_init());

#if calibration_joystick
    ESP_ERROR_CHECK(input_run_calibration());
#endif

    BaseType_t ok_tx = xTaskCreate(rc_tx_task,
                                   "rc_tx_task",
                                   FPV_TASK_STACK_SIZE,
                                   NULL,
                                   FPV_TASK_PRIORITY_TX,
                                   NULL);
#if FC_RX_ENABLE
    BaseType_t ok_rx = xTaskCreate(crsf_rx_task,
                                   "crsf_rx_task",
                                   FPV_TASK_STACK_SIZE,
                                   NULL,
                                   FPV_TASK_PRIORITY_RX,
                                   NULL);
#else
    BaseType_t ok_rx = pdPASS;
#endif

    if ((ok_tx != pdPASS) || (ok_rx != pdPASS)) {
        ESP_LOGE(TAG, "Task creation failed (tx=%ld rx=%ld)", (long)ok_tx, (long)ok_rx);
        return;
    }

#if FC_RX_ENABLE
    ESP_LOGI(TAG, "Tasks started: RC TX @ %d Hz, telemetry RX enabled", FPV_RC_SEND_RATE_HZ);
#else
    ESP_LOGI(TAG, "Tasks started: RC TX @ %d Hz, telemetry RX disabled", FPV_RC_SEND_RATE_HZ);
#endif
}
