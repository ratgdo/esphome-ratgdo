#include "HAPAccessory.h"
namespace esphome {
namespace homekit {
    HAPAccessory::HAPAccessory()
    {
    }
    void HAPAccessory::setup()
    {
#ifdef USE_LIGHT
        for (const auto v : lights) {
            v->setup();
        }
#endif
#ifdef USE_LOCK
        for (const auto& v : locks) {
#ifdef USE_HOMEKEY
            if (nfc) {
                v->set_hk_hw_finish(hkHwFinish);
                v->set_nfc_ctx(nfc);
            }
#endif
            v->setup();
        }
#endif

#ifdef USE_SWITCH
        for (const auto v : switches) {
            v->setup();
        }
#endif
#ifdef USE_SENSOR
        for (const auto v : sensors) {
            v->setup();
        }
#endif

#ifdef USE_COVER
        for (const auto v : covers) {
            v->setup();
        }
#endif
    }
    void HAPAccessory::dump_config()
    {
#ifdef USE_LOCK
        ESP_LOGCONFIG(TAG, "Lock HK Entities: %d", locks.size());
#endif
#ifdef USE_LIGHT
        ESP_LOGCONFIG(TAG, "Light HK Entities: %d", lights.size());
#endif
#ifdef USE_SENSOR
        ESP_LOGCONFIG(TAG, "Sensor HK Entities: %d", sensors.size());
#endif

#ifdef USE_SWITCH
        ESP_LOGCONFIG(TAG, "Switch HK Entities: %d", switches.size());
#endif

#ifdef USE_COVER
        ESP_LOGCONFIG(TAG, "Cover HK Entities: %d", covers.size());
#endif
    }
#ifdef USE_LIGHT
    LightEntity* HAPAccessory::add_light(light::LightState* lightPtr)
    {
        lights.push_back(new LightEntity(lightPtr));
        return lights.back();
    }
#endif
#ifdef USE_LOCK
    LockEntity* HAPAccessory::add_lock(lock::Lock* lockPtr)
    {
        locks.push_back(new LockEntity(lockPtr));
        return locks.back();
    }
#endif
#ifdef USE_HOMEKEY
    void HAPAccessory::set_nfc_ctx(pn532::PN532* nfcCtx)
    {
        nfc = nfcCtx;
    }
    void HAPAccessory::set_hk_hw_finish(HKFinish color)
    {
        hkHwFinish = color;
    }
#endif

#ifdef USE_SWITCH
    SwitchEntity* HAPAccessory::add_switch(switch_::Switch* switchPtr)
    {
        switches.push_back(new SwitchEntity(switchPtr));
        return switches.back();
    }
#endif
#ifdef USE_SENSOR
    SensorEntity* HAPAccessory::add_sensor(sensor::Sensor* sensorPtr, TemperatureUnits units)
    {
        sensors.push_back(new SensorEntity(sensorPtr, units));
        return sensors.back();
    }
#endif

#ifdef USE_COVER
    CoverEntity* HAPAccessory::add_cover(cover::Cover* coverPtr)
    {
        covers.push_back(new CoverEntity(coverPtr));
        return covers.back();
    }
#endif
}
}
