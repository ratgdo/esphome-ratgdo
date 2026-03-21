#pragma once

#include "esphome/core/defines.h"

#ifdef USE_ESP32

#include <stddef.h>
#include <stdint.h>

namespace esphome::ratgdo {

enum RatgdoUARTConfig {
    RATGDO_UART_8N1,
    RATGDO_UART_8E1,
};

class RatgdoUART {
public:
    RatgdoUART();
    ~RatgdoUART();

    void begin(int baud, RatgdoUARTConfig config, int rx_pin, int tx_pin,
        bool invert);
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
    bool is_initialized_ { false };
};

} // namespace esphome::ratgdo

#endif
