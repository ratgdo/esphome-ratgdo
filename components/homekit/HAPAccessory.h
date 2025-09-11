#pragma once
#include <esp_log.h>
#include <esphome/core/log.h>
#include <esphome/core/defines.h>
#include <esphome/core/component.h>
#include "const.h"
#ifdef USE_HOMEKEY
#include <nvs.h> // For nvs_handle
#include "../pn532/pn532.h" // For pn532::PN532
#endif
#ifdef USE_LIGHT
#include "light.hpp"
#endif
#ifdef USE_LOCK
#include "lock.h"
#endif
#ifdef USE_FAN
#include "fan.hpp"
#endif
#ifdef USE_SWITCH
#include "switch.hpp"
#endif
#ifdef USE_SENSOR
#include "sensor.hpp"
#endif
#ifdef USE_CLIMATE
#include "climate.hpp"
#endif
#ifdef USE_COVER
#include "cover.hpp"
#endif
namespace esphome
{
  namespace homekit
  {
    class HAPAccessory : public Component
    {
    public:
      const char* TAG = "HAPAccessory";
      HAPAccessory();
      float get_setup_priority() const override { return setup_priority::AFTER_WIFI + 1; }
      void setup() override;
      void dump_config() override;
      #ifdef USE_HOMEKEY
      pn532::PN532* nfc = nullptr;
      HKFinish hkHwFinish = SILVER;
      #endif
      #ifdef USE_LIGHT
      std::vector<LightEntity*> lights;
      LightEntity* add_light(light::LightState* lightPtr);
      #endif
      #ifdef USE_LOCK
      std::vector<LockEntity*> locks;
      LockEntity* add_lock(lock::Lock* lockPtr);
      #endif
      #ifdef USE_HOMEKEY
      void set_nfc_ctx(pn532::PN532* nfcCtx);
      void set_hk_hw_finish(HKFinish color);
      #endif
      #ifdef USE_FAN
      std::vector<FanEntity*> fans;
      FanEntity* add_fan(fan::Fan* fanPtr);
      #endif
      #ifdef USE_SWITCH
      std::vector<SwitchEntity*> switches;
      SwitchEntity* add_switch(switch_::Switch* switchPtr);
      #endif
      #ifdef USE_SENSOR
      std::vector<SensorEntity*> sensors;
      SensorEntity* add_sensor(sensor::Sensor* sensorPtr, TemperatureUnits units);
      #endif
      #ifdef USE_CLIMATE
      std::vector<ClimateEntity*> climates;
      ClimateEntity* add_climate(climate::Climate* sensorPtr);
      #endif
      #ifdef USE_COVER
      std::vector<CoverEntity*> covers;
      CoverEntity* add_cover(cover::Cover* coverPtr);
      #endif
    };
  }
}