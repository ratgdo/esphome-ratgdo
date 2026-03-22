#include "ratgdo_uart.h"

#ifdef USE_ESP32

#include "esphome/core/log.h"
#include <driver/uart.h>

#include <driver/rmt_tx.h>
#include <esp_private/rmt.h>

#include <driver/gpio.h>
#include <esp_rom_gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <soc/gpio_sig_map.h>

namespace esphome::ratgdo {

static const char* const TAG = "ratgdo_uart";

static constexpr size_t UART_RX_BUFFER_SIZE = 512;

// Security+ 2.0 preamble timing (microseconds, at 1MHz RMT resolution = ticks)
static constexpr uint16_t PREAMBLE_DURATION_US = 1300;
static constexpr uint16_t PREAMBLE_MARK_US = 130;
static constexpr uint8_t SIGNAL_SETTLE_US = 5;

RatgdoUART::RatgdoUART() { }

RatgdoUART::~RatgdoUART()
{
    if (this->is_initialized_) {
        uart_driver_delete((uart_port_t)this->uart_num_);
        if (this->rmt_copy_encoder_) {
            rmt_del_encoder(this->rmt_copy_encoder_);
            this->rmt_copy_encoder_ = nullptr;
        }
        if (this->rmt_chan_handle_) {
            rmt_disable(this->rmt_chan_handle_);
            rmt_del_channel(this->rmt_chan_handle_);
            this->rmt_chan_handle_ = nullptr;
        }
    }
}

void RatgdoUART::begin(int baud, RatgdoUARTConfig config, int rx_pin,
    int tx_pin, bool invert)
{
    this->tx_pin_ = tx_pin;
    this->rx_pin_ = rx_pin;
    this->baud_ = baud;
    this->inverted_ = invert;

    this->uart_num_ = UART_NUM_1;

    uart_config_t uart_config = { };
    uart_config.baud_rate = baud;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = (config == RATGDO_UART_8E1) ? UART_PARITY_EVEN : UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_APB;

    ESP_ERROR_CHECK(
        uart_driver_install((uart_port_t)this->uart_num_, UART_RX_BUFFER_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config((uart_port_t)this->uart_num_, &uart_config));

    rmt_tx_channel_config_t tx_chan_config = { };
    tx_chan_config.gpio_num = (gpio_num_t)tx_pin;
    tx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;
    tx_chan_config.resolution_hz = 1000000;
    tx_chan_config.mem_block_symbols = 64;
    tx_chan_config.trans_queue_depth = 4;
    tx_chan_config.flags.invert_out = 0;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &this->rmt_chan_handle_));

    rmt_copy_encoder_config_t copy_encoder_config = { };
    ESP_ERROR_CHECK(
        rmt_new_copy_encoder(&copy_encoder_config, &this->rmt_copy_encoder_));

    ESP_ERROR_CHECK(rmt_enable(this->rmt_chan_handle_));

    ESP_ERROR_CHECK(uart_set_pin((uart_port_t)this->uart_num_, tx_pin, rx_pin,
        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    if (invert) {
        uart_set_line_inverse((uart_port_t)this->uart_num_,
            UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV);
    }

    this->is_initialized_ = true;
    ESP_LOGD(TAG, "Hardware UART and RMT initialized on TX=%d RX=%d", tx_pin,
        rx_pin);
}

void RatgdoUART::transmit_secplus2_preamble()
{
    if (!this->is_initialized_)
        return;

    // Get the actual channel ID allocated by the driver
    int channel_id = 0;
    ESP_ERROR_CHECK(rmt_get_channel_id(this->rmt_chan_handle_, &channel_id));

    // Switch GPIO matrix from UART TX to RMT output
    esp_rom_gpio_connect_out_signal(this->tx_pin_, RMT_SIG_OUT0_IDX + channel_id,
        false, false);

    esp_rom_delay_us(SIGNAL_SETTLE_US);

    // Indicate the start of a frame by pulling the 12V line low for at least
    // 1 byte followed by one STOP bit, which indicates to the receiving end
    // that the start of the message follows.
    // The output pin controls a transistor, so the logic is inverted:
    // RMT level 1 (HIGH) pulls the wire low, level 0 (LOW) lets it float high.
    rmt_symbol_word_t symbols[1];
    symbols[0].duration0 = PREAMBLE_DURATION_US;
    symbols[0].level0 = 1;
    symbols[0].duration1 = PREAMBLE_MARK_US;
    symbols[0].level1 = 0;

    rmt_transmit_config_t transmit_config = { };
    transmit_config.loop_count = 0;
    rmt_transmit(this->rmt_chan_handle_, this->rmt_copy_encoder_, symbols,
        sizeof(symbols), &transmit_config);
    rmt_tx_wait_all_done(this->rmt_chan_handle_, -1);

    // Switch GPIO matrix back to UART TX
    esp_rom_gpio_connect_out_signal(this->tx_pin_, U1TXD_OUT_IDX, false, false);
    esp_rom_delay_us(SIGNAL_SETTLE_US);
}

void RatgdoUART::write(const uint8_t* data, size_t len)
{
    if (this->is_initialized_) {
        uart_write_bytes((uart_port_t)this->uart_num_, (const char*)data, len);
        uart_wait_tx_done((uart_port_t)this->uart_num_, portMAX_DELAY);
    }
}

void RatgdoUART::write(uint8_t data) { write(&data, 1); }

int RatgdoUART::available()
{
    if (!this->is_initialized_)
        return 0;
    size_t length = 0;
    uart_get_buffered_data_len((uart_port_t)this->uart_num_, &length);
    return length;
}

int RatgdoUART::read()
{
    if (!this->is_initialized_)
        return -1;
    uint8_t data = 0;
    int len = uart_read_bytes((uart_port_t)this->uart_num_, &data, 1, 0);
    if (len > 0) {
        return data;
    }
    return -1;
}

void RatgdoUART::on_shutdown()
{
    if (this->is_initialized_) {
        // Unmap the matrix output signal so that UART peripheral resets do not
        // pull the hardware line dominant.
        esp_rom_gpio_connect_out_signal(this->tx_pin_, SIG_GPIO_OUT_IDX, false, false);
        gpio_set_direction((gpio_num_t)this->tx_pin_, GPIO_MODE_INPUT);
        gpio_set_direction((gpio_num_t)this->rx_pin_, GPIO_MODE_INPUT);
    }
}

} // namespace esphome::ratgdo

#endif
