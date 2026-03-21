#pragma once

#include "esphome/core/defines.h"

#ifdef USE_ESP32
#include "ratgdo_uart_esp32.h"
#elif defined(USE_ESP8266)
#include "ratgdo_uart_esp8266.h"
#endif
