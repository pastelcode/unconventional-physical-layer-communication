#include <stdbool.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Infrared transmitter / receiver */
#define IR_TX_GPIO          GPIO_NUM_18
#define IR_RX_GPIO          GPIO_NUM_26

/* Demodulating receivers (TSOP/VS1838/etc.) expect a 38 kHz carrier. */
#define IR_CARRIER_HZ       38000
#define IR_CARRIER_RES      LEDC_TIMER_10_BIT
#define IR_CARRIER_DUTY     ((1 << 10) / 2)     /* 50 % of the 10-bit range */

/* One cycle: carrier on for IR_ON_MS, then off for IR_OFF_MS. */
#define IR_ON_MS            1000
#define IR_OFF_MS           1000

/* Receiver sampling period while the transmitter is active. */
#define IR_RX_POLL_US       2000

static void ir_tx_init(void)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = IR_CARRIER_RES,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = IR_CARRIER_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t chan_conf = {
        .gpio_num = IR_TX_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,                                  /* start silent */
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&chan_conf));
}

static void ir_tx_set(bool on)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0,
                                  on ? IR_CARRIER_DUTY : 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

static void ir_rx_init(void)
{
    gpio_config_t rx_conf = {
        .pin_bit_mask = 1ULL << IR_RX_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&rx_conf));
}

/*
 * Sample the receiver for duration_us and report whether its level ever
 * changed. Checking for an edge instead of a fixed level makes this work
 * with both active-low demodulating modules and raw phototransistors.
 */
static bool ir_rx_detect(uint32_t duration_us)
{
    const int64_t start = esp_timer_get_time();
    int last_level = gpio_get_level(IR_RX_GPIO);
    bool detected = false;

    while ((esp_timer_get_time() - start) < (int64_t)duration_us) {
        int level = gpio_get_level(IR_RX_GPIO);
        if (level != last_level) {
            detected = true;
            last_level = level;
        }
        esp_rom_delay_us(IR_RX_POLL_US);
    }

    return detected;
}

void app_main(void)
{
    ir_tx_init();
    ir_rx_init();

    printf("Physical Layer Project\n");
    printf("IR transmitter on GPIO%d (%d Hz carrier), receiver on GPIO%d\n",
           IR_TX_GPIO, IR_CARRIER_HZ, IR_RX_GPIO);

    uint32_t cycle = 0;
    while (1) {
        ir_tx_set(true);
        bool detected = ir_rx_detect(IR_ON_MS * 1000);
        ir_tx_set(false);

        printf("cycle %lu: transmitter ON -> receiver %s\n",
               (unsigned long)++cycle,
               detected ? "DETECTED a signal" : "saw nothing");

        vTaskDelay(pdMS_TO_TICKS(IR_OFF_MS));
    }
}
