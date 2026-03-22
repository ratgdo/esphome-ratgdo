#include "ratgdo_uart.h"

#ifdef USE_ESP32

#include "esphome/core/log.h"
#include <driver/uart.h>
#include <esp_idf_version.h>

#include <driver/gpio.h>
#include <esp_rom_gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <soc/gpio_sig_map.h>

namespace esphome::ratgdo {

static const char* const TAG = "ratgdo_uart";

RatgdoUART::RatgdoUART() { }

RatgdoUART::~RatgdoUART()
{
    if (is_initialized_) {
        uart_driver_delete((uart_port_t)uart_num_);
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
        uart_driver_install((uart_port_t)uart_num_, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config((uart_port_t)uart_num_, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin((uart_port_t)uart_num_, tx_pin, rx_pin,
        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    if (invert) {
        uart_set_line_inverse((uart_port_t)uart_num_,
            UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV);
    }

    is_initialized_ = true;
    ESP_LOGD(TAG, "Hardware UART initialized on TX=%d RX=%d", tx_pin, rx_pin);
}

void RatgdoUART::transmit_secplus2_preamble()
{
    if (!is_initialized_)
        return;

    // Disconnect UART from the pin by routing the generic GPIO matrix
    esp_rom_gpio_connect_out_signal(tx_pin_, SIG_GPIO_OUT_IDX, false, false);
    gpio_set_direction((gpio_num_t)tx_pin_, GPIO_MODE_OUTPUT);

    // Preamble is ~1.3ms low, followed by 130us high on the physical wire
    uint32_t first_level = inverted_ ? 1 : 0;
    uint32_t second_level = inverted_ ? 0 : 1;

    gpio_set_level((gpio_num_t)tx_pin_, first_level);
    esp_rom_delay_us(1300);

    gpio_set_level((gpio_num_t)tx_pin_, second_level);
    esp_rom_delay_us(130);

    // Reattach Hardware UART to the physical pin dynamically
    ESP_ERROR_CHECK(uart_set_pin((uart_port_t)uart_num_, tx_pin_, rx_pin_,
        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    if (inverted_) {
        uart_set_line_inverse((uart_port_t)uart_num_,
            UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV);
    }
}

void RatgdoUART::write(const uint8_t* data, size_t len)
{
    if (is_initialized_) {
        uart_write_bytes((uart_port_t)uart_num_, (const char*)data, len);
        uart_wait_tx_done((uart_port_t)uart_num_, portMAX_DELAY);
    }
}

void RatgdoUART::write(uint8_t data) { write(&data, 1); }

int RatgdoUART::available()
{
    if (!is_initialized_)
        return 0;
    size_t length = 0;
    uart_get_buffered_data_len((uart_port_t)uart_num_, &length);
    return length;
}

int RatgdoUART::read()
{
    if (!is_initialized_)
        return -1;
    uint8_t data = 0;
    int len = uart_read_bytes((uart_port_t)uart_num_, &data, 1, 0);
    if (len > 0) {
        return data;
    }
    return -1;
}

void RatgdoUART::enableIntTx(bool enable) { }
void RatgdoUART::enableAutoBaud(bool enable) { }
int RatgdoUART::baudRate() { return baud_; }

void RatgdoUART::on_shutdown()
{
    if (is_initialized_) {
        // Unmap the matrix output signal so that UART peripheral resets do not pull
        // the hardware line dominant.
        esp_rom_gpio_connect_out_signal(tx_pin_, SIG_GPIO_OUT_IDX, false, false);
        gpio_set_direction((gpio_num_t)tx_pin_, GPIO_MODE_INPUT);
        gpio_set_direction((gpio_num_t)rx_pin_, GPIO_MODE_INPUT);
    }
}

} // namespace esphome::ratgdo

#endif
