#pragma once
#include <esphome/core/defines.h>
#ifdef USE_SENSOR
#include "hap_entity.h"
#include <esphome/core/application.h>
#include <esphome/components/sensor/sensor.h>
#include <hap.h>
#include <hap_apple_chars.h>
#include <hap_apple_servs.h>

namespace esphome {
namespace homekit {
    class SensorEntity : public HAPEntity {
    private:
        static constexpr const char* TAG = "SensorEntity";
        sensor::Sensor* sensorPtr;
        TemperatureUnits units;

        static int acc_identify(hap_acc_t* ha)
        {
            ESP_LOGI(TAG, "Accessory identified");
            return HAP_SUCCESS;
        }

    public:
        SensorEntity(sensor::Sensor* sensorPtr, TemperatureUnits units = CELSIUS)
            : HAPEntity({ { MODEL, "HAP-SENSOR" } })
            , sensorPtr(sensorPtr)
            , units(units)
        {
        }

        void setup()
        {
            hap_acc_cfg_t acc_cfg = {
                .model = (char*)accessory_info[MODEL],
                .manufacturer = (char*)accessory_info[MANUFACTURER],
                .fw_rev = (char*)accessory_info[FW_REV],
                .hw_rev = (char*)"1.0",
                .pv = (char*)"1.1.0",
                .cid = HAP_CID_BRIDGE,
                .identify_routine = acc_identify,
                .name = nullptr,
                .serial_num = nullptr,
                .hw_finish = HAP_HW_FINISH_OTHER
            };
            hap_acc_t* accessory = nullptr;
            hap_serv_t* service = nullptr;
            std::string accessory_name = sensorPtr->get_name();
            
            if (accessory_info[NAME] == NULL) {
                acc_cfg.name = (char*)accessory_name.c_str();
            } else {
                acc_cfg.name = (char*)accessory_info[NAME];
            }
            if (accessory_info[SN] == NULL) {
                acc_cfg.serial_num = (char*)std::to_string(sensorPtr->get_object_id_hash()).c_str();
            } else {
                acc_cfg.serial_num = (char*)accessory_info[SN];
            }
            
            accessory = hap_acc_create(&acc_cfg);
            service = hap_serv_temperature_sensor_create(sensorPtr->get_state());
            hap_serv_add_char(service, hap_char_name_create(accessory_name.data()));
            
            ESP_LOGD(TAG, "ID HASH: %lu", sensorPtr->get_object_id_hash());
            hap_serv_set_priv(service, sensorPtr);
            
            /* Add the Sensor Service to the Accessory Object */
            hap_acc_add_serv(accessory, service);
            
            /* Add the Accessory to the HomeKit Database */
            hap_add_bridged_accessory(accessory, hap_get_unique_aid(std::to_string(sensorPtr->get_object_id_hash()).c_str()));
            
            ESP_LOGI(TAG, "Sensor '%s' linked to HomeKit", accessory_name.c_str());
        }
    };
}
}
#endif