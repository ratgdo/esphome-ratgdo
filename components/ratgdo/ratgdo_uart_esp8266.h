#pragma once

#include "esphome/core/defines.h"

#ifdef USE_ESP8266

#include "SoftwareSerial.h"

namespace esphome::ratgdo {

// Security+ 2.0 preamble timing (microseconds)
static constexpr uint16_t PREAMBLE_DURATION_US = 1300;
static constexpr uint16_t PREAMBLE_MARK_US = 130;

enum RatgdoUARTConfig {
    RATGDO_UART_8N1 = SWSERIAL_8N1,
    RATGDO_UART_8E1 = SWSERIAL_8E1,
};

// On ESP8266, simply fall back to SoftwareSerial
class RatgdoUART : public SoftwareSerial {
public:
    void begin(uint32_t baud, RatgdoUARTConfig config, int8_t rxPin, int8_t txPin,
        bool invert)
    {
        this->rx_pin_ = rxPin;
        this->tx_pin_ = txPin;
        SoftwareSerial::begin(baud, static_cast<Config>(config), rxPin, txPin, invert);
    }

    void on_shutdown()
    {
        if (this->tx_pin_ >= 0) {
            ::pinMode(this->tx_pin_, INPUT);
        }
        if (this->rx_pin_ >= 0) {
            ::pinMode(this->rx_pin_, INPUT);
        }
    }

    // Fallback for SECPLUS2 preamble on ESP8266
    void transmit_secplus2_preamble()
    {
        if (this->tx_pin_ < 0)
            return;
        ::digitalWrite(this->tx_pin_, HIGH);
        ::delayMicroseconds(PREAMBLE_DURATION_US);
        ::digitalWrite(this->tx_pin_, LOW);
        ::delayMicroseconds(PREAMBLE_MARK_US);
    }

private:
    int8_t rx_pin_ { -1 };
    int8_t tx_pin_ { -1 };
};

} // namespace esphome::ratgdo

#endif
