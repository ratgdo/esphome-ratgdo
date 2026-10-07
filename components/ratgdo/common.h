#pragma once

#include "esphome/core/log.h"

#define ESP_LOG1 ESP_LOGV
#define ESP_LOG2 ESP_LOGV

// ESPHome 2026.10 and later keep log tags in flash on ESP8266; older releases lack the macro
#ifndef ESPHOME_LOG_TAG
#define ESPHOME_LOG_TAG(name, tag) static const char* const name = tag
#endif
