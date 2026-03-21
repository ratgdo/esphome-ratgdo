#pragma once

#include "esphome/core/defines.h"

#ifdef USE_ESP32

#include <stddef.h>
#include <stdint.h>

namespace esphome {
namespace ratgdo {

    enum RatgdoUARTConfig {
        RATGDO_UART_8N1,
        RATGDO_UART_8E1,
    };

    class RatgdoUART {
    public:
        RatgdoUART();
        ~RatgdoUART();

        void begin(int baud, RatgdoUARTConfig config, int rx_pin, int tx_pin, bool invert);
        void write(const uint8_t* data, size_t len);
        void write(uint8_t data);
        int available();
        int read();
        void enableIntTx(bool enable);
        void enableAutoBaud(bool enable);
        int baudRate();

        // Sends the SecPlus 2.0 preamble using RMT
        void transmit_secplus2_preamble();

    private:
        int tx_pin_ { -1 };
        int rx_pin_ { -1 };
        int baud_ { 9600 };
        bool inverted_ { true };
        int uart_num_ { -1 };
        int rmt_channel_ { -1 };
        bool is_initialized_ { false };
    };

} // namespace ratgdo
} // namespace esphome

#else // USE_ESP8266

#include "SoftwareSerial.h"

namespace esphome::ratgdo

    // On ESP8266, simply fall back to SoftwareSerial
    class RatgdoUART : public SoftwareSerial {
public:
    void begin(uint32_t baud, Config config, int8_t rxPin, int8_t txPin, bool invert)
    {
        SoftwareSerial::begin(baud, config, rxPin, txPin, invert);
    }

    // Fallback for SECPLUS2 preamble on ESP8266
    void transmit_secplus2_preamble()
    {
        if (m_txPin < 0)
            return;
        // On ESP8266 SoftwareSerial bit-bangs anyway, so we just use standard methods.
        // The calling code previously used `tx_pin_->digital_write()`. We can toggle it natively.
        ::digitalWrite(m_txPin, HIGH);
        ::delayMicroseconds(1300);
        ::digitalWrite(m_txPin, LOW);
        ::delayMicroseconds(130);
    }
};

enum RatgdoUARTConfig {
    RATGDO_UART_8N1 = SWSERIAL_8N1,
    RATGDO_UART_8E1 = SWSERIAL_8E1,
};

} // namespace ratgdo
} // namespace esphome

#endif
