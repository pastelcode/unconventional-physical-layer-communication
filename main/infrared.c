#include "infrared.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_attr.h"

#include "diagnostics.h"
#include "protocol_config.h"

#if PHY_IR_DEMODULATING
#define IR_CARRIER_RES  LEDC_TIMER_10_BIT
#define IR_CARRIER_DUTY ((1 << 10) / 2) /* 50 % duty */
static volatile uint32_t s_carrier_hz = IR_CARRIER_HZ;
#endif

esp_err_t ir_init(void)
{
#if PHY_IR_DEMODULATING
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = IR_CARRIER_RES,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = IR_CARRIER_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        return err;
    }

    ledc_channel_config_t chan_conf = {
        .gpio_num = GPIO_IR_TX,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0, /* start silent */
        .hpoint = 0,
    };
    err = ledc_channel_config(&chan_conf);
    if (err != ESP_OK) {
        return err;
    }
#else
    gpio_config_t tx_conf = {
        .pin_bit_mask = 1ULL << GPIO_IR_TX,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&tx_conf);
    if (err != ESP_OK) {
        return err;
    }
    gpio_set_level(GPIO_IR_TX, 0);
#endif

    gpio_config_t rx_conf = {
        .pin_bit_mask = 1ULL << GPIO_IR_RX,
        .mode = GPIO_MODE_INPUT,
        /* A demodulating module drives the line push-pull; a raw detector
         * may need the pull-up, which is harmless either way. */
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&rx_conf);
}

esp_err_t ir_set_carrier_hz(uint32_t hz)
{
    if (hz < IR_CARRIER_MIN_HZ || hz > IR_CARRIER_MAX_HZ) {
        return ESP_ERR_INVALID_ARG;
    }
#if PHY_IR_DEMODULATING
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = IR_CARRIER_RES,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err == ESP_OK) {
        s_carrier_hz = hz;
    }
    return err;
#else
    /* No carrier: the IR line carries the logical level directly. */
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

uint32_t ir_carrier_hz(void)
{
#if PHY_IR_DEMODULATING
    return s_carrier_hz;
#else
    return 0;
#endif
}

void ir_tx_set_logical_bit(bool bit)
{
#if PHY_IR_DEMODULATING
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, bit ? IR_CARRIER_DUTY : 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
#else
    gpio_set_level(GPIO_IR_TX, bit ? 1 : 0);
#endif
}

void ir_tx_idle(void)
{
    ir_tx_set_logical_bit(false);
}

bool IRAM_ATTR ir_rx_raw_level(void)
{
    return gpio_get_level(GPIO_IR_RX) != 0;
}

bool IRAM_ATTR ir_rx_sample_logical_bit(void)
{
    const bool raw = ir_rx_raw_level();
#if PHY_IR_RX_ACTIVE_LOW
    return !raw;
#else
    return raw;
#endif
}
