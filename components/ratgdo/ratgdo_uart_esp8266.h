#pragma once

#include "esphome/core/defines.h"

#ifdef USE_ESP8266

#include "SoftwareSerial.h"

namespace esphome::ratgdo {

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
        SoftwareSerial::begin(baud, static_cast<Config>(config), rxPin, txPin, invert);
    }

    void on_shutdown()
    {
        if (m_txPin >= 0) {
            ::pinMode(m_txPin, INPUT);
        }
        if (m_rxPin >= 0) {
            ::pinMode(m_rxPin, INPUT);
        }
    }

    // Fallback for SECPLUS2 preamble on ESP8266
    void transmit_secplus2_preamble()
    {
        if (m_txPin < 0)
            return;
        // On ESP8266 SoftwareSerial bit-bangs anyway, so we just use standard
        // methods. The calling code previously used `tx_pin_->digital_write()`. We
        // can toggle it natively.
        ::digitalWrite(m_txPin, HIGH);
        ::delayMicroseconds(1300);
        ::digitalWrite(m_txPin, LOW);
        ::delayMicroseconds(130);
    }
};

} // namespace esphome::ratgdo

#endif
