#include "ratgdo_uart.h"

#ifdef USE_ESP32

#include "esphome/core/log.h"
#include <driver/uart.h>
#include <esp_idf_version.h>

#include <driver/rmt_tx.h>

#include <driver/gpio.h>
#include <esp_rom_gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <soc/gpio_sig_map.h>

namespace esphome::ratgdo {

static const char *const TAG = "ratgdo_uart";

rmt_channel_handle_t rmt_chan_handle_ = NULL;
rmt_encoder_handle_t rmt_copy_encoder_ = NULL;

RatgdoUART::RatgdoUART() {}

RatgdoUART::~RatgdoUART() {
  if (is_initialized_) {
    uart_driver_delete((uart_port_t)uart_num_);
    if (rmt_copy_encoder_)
      rmt_del_encoder(rmt_copy_encoder_);
    if (rmt_chan_handle_) {
      rmt_disable(rmt_chan_handle_);
      rmt_del_channel(rmt_chan_handle_);
    }
  }
}

void RatgdoUART::begin(int baud, RatgdoUARTConfig config, int rx_pin,
                       int tx_pin, bool invert) {
  this->tx_pin_ = tx_pin;
  this->rx_pin_ = rx_pin;
  this->baud_ = baud;
  this->inverted_ = invert;

  this->uart_num_ = UART_NUM_1;

  uart_config_t uart_config = {};
  uart_config.baud_rate = baud;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity =
      (config == RATGDO_UART_8E1) ? UART_PARITY_EVEN : UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uart_config.source_clk = UART_SCLK_APB;

  ESP_ERROR_CHECK(
      uart_driver_install((uart_port_t)uart_num_, 256, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config((uart_port_t)uart_num_, &uart_config));

  rmt_tx_channel_config_t tx_chan_config = {};
  tx_chan_config.gpio_num = (gpio_num_t)tx_pin;
  tx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;
  tx_chan_config.resolution_hz = 1000000;
  tx_chan_config.mem_block_symbols = 64;
  tx_chan_config.trans_queue_depth = 4;
  tx_chan_config.flags.invert_out =
      0; // The true inversion is handled by UART hardware manually or software
         // when we attach the pin.
  ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &rmt_chan_handle_));

  rmt_copy_encoder_config_t copy_encoder_config = {};
  ESP_ERROR_CHECK(
      rmt_new_copy_encoder(&copy_encoder_config, &rmt_copy_encoder_));

  ESP_ERROR_CHECK(rmt_enable(rmt_chan_handle_));

  ESP_ERROR_CHECK(uart_set_pin((uart_port_t)uart_num_, tx_pin, rx_pin,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  if (invert) {
    uart_set_line_inverse((uart_port_t)uart_num_,
                          UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV);
  }

  esp_rom_gpio_connect_out_signal(tx_pin, U1TXD_OUT_IDX, invert, false);

  is_initialized_ = true;
  ESP_LOGD(TAG, "Hardware UART and RMT initialized on TX=%d RX=%d", tx_pin,
           rx_pin);
}

void RatgdoUART::transmit_secplus2_preamble() {
  if (!is_initialized_)
    return;

  // Map RMT signal
  int channel_idx = 0;
  esp_rom_gpio_connect_out_signal(tx_pin_, RMT_SIG_OUT0_IDX + channel_idx,
                                  false, false);

  esp_rom_delay_us(5);

  uint16_t first_level = 1;
  uint16_t second_level = 0;

  rmt_symbol_word_t symbols[1];
  symbols[0].duration0 = 1300;
  symbols[0].level0 = first_level;
  symbols[0].duration1 = 130;
  symbols[0].level1 = second_level;

  rmt_transmit_config_t transmit_config = {};
  transmit_config.loop_count = 0;
  rmt_transmit(rmt_chan_handle_, rmt_copy_encoder_, symbols, sizeof(symbols),
               &transmit_config);
  rmt_tx_wait_all_done(rmt_chan_handle_, -1);

  esp_rom_gpio_connect_out_signal(tx_pin_, U1TXD_OUT_IDX, inverted_, false);
  esp_rom_delay_us(5);
}

void RatgdoUART::write(const uint8_t *data, size_t len) {
  if (is_initialized_) {
    uart_write_bytes((uart_port_t)uart_num_, (const char *)data, len);
    uart_wait_tx_done((uart_port_t)uart_num_, portMAX_DELAY);
  }
}

void RatgdoUART::write(uint8_t data) { write(&data, 1); }

int RatgdoUART::available() {
  if (!is_initialized_)
    return 0;
  size_t length = 0;
  uart_get_buffered_data_len((uart_port_t)uart_num_, &length);
  return length;
}

int RatgdoUART::read() {
  if (!is_initialized_)
    return -1;
  uint8_t data = 0;
  int len = uart_read_bytes((uart_port_t)uart_num_, &data, 1, 0);
  if (len > 0) {
    return data;
  }
  return -1;
}

void RatgdoUART::enableIntTx(bool enable) {}
void RatgdoUART::enableAutoBaud(bool enable) {}
int RatgdoUART::baudRate() { return baud_; }

} // namespace esphome::ratgdo

#endif
