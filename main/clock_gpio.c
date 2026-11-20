#include "clock_gpio.h"

#include <inttypes.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

#include "diagnostics.h"
#include "infrared.h"
#include "protocol_config.h"

static QueueHandle_t s_edge_queue;
static volatile uint32_t s_edge_count;

/* Runtime pulse widths; written by the console task, read per pulse. */
static volatile uint32_t s_high_us = CLK_HIGH_US;
static volatile uint32_t s_low_us = CLK_LOW_MIN_US;

static void IRAM_ATTR clock_rx_isr(void *arg)
{
    (void)arg;

    clk_edge_t edge = {
        .ts_us = esp_timer_get_time(),
        .clk_level = (uint8_t)gpio_get_level(GPIO_CLK_RX),
        .ir_bit = (uint8_t)(ir_rx_sample_logical_bit() ? 1u : 0u),
    };

    s_edge_count++;

    if (s_edge_queue == NULL) {
        return;
    }
    BaseType_t woken = pdFALSE;
    if (xQueueSendFromISR(s_edge_queue, &edge, &woken) != pdTRUE) {
        /* Queue full: count it and let the frame timeout clean up. */
        g_phy_stats.clk_rx_overflow++;
    }
    if (woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

esp_err_t clock_gpio_init(QueueHandle_t edge_queue)
{
    s_edge_queue = edge_queue;
    s_edge_count = 0;

    gpio_config_t tx_conf = {
        .pin_bit_mask = 1ULL << GPIO_CLK_TX,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&tx_conf);
    if (err != ESP_OK) {
        return err;
    }
    gpio_set_level(GPIO_CLK_TX, CLK_IDLE_LEVEL);

    gpio_config_t rx_conf = {
        .pin_bit_mask = 1ULL << GPIO_CLK_RX,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE, /* define the level if unwired */
        .intr_type = GPIO_INTR_POSEDGE,
    };
    err = gpio_config(&rx_conf);
    if (err != ESP_OK) {
        return err;
    }

    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    return gpio_isr_handler_add(GPIO_CLK_RX, clock_rx_isr, NULL);
}

void clock_tx_low(void)
{
    gpio_set_level(GPIO_CLK_TX, 0);
}

void clock_tx_pulse(void)
{
    const uint32_t high_us = s_high_us;
    const uint32_t low_us = s_low_us;

    gpio_set_level(GPIO_CLK_TX, 1);
    esp_rom_delay_us(high_us);
    gpio_set_level(GPIO_CLK_TX, 0);
    esp_rom_delay_us(low_us);
}

esp_err_t clock_set_rate_hz(uint32_t hz)
{
    if (hz < CLK_RATE_MIN_HZ || hz > CLK_RATE_MAX_HZ) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t period_us = 1000000u / hz;
    uint32_t high = period_us / 2;
    uint32_t low = period_us - high;
    if (high < 100) {
        high = 100; /* never produce an unreadable stub pulse */
    }
    if (low < 100) {
        low = 100;
    }

    s_high_us = high;
    s_low_us = low;
    return ESP_OK;
}

uint32_t clock_rate_hz(void)
{
    const uint32_t period = s_high_us + s_low_us;
    return period == 0 ? 0 : 1000000u / period;
}

uint32_t clock_high_us(void)
{
    return s_high_us;
}

uint32_t clock_low_us(void)
{
    return s_low_us;
}

uint32_t clock_rx_edge_count(void)
{
    return s_edge_count;
}
