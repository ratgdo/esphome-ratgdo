#include <esphome/core/defines.h>
#ifdef USE_LIGHT
#pragma once
#include "hap_entity.h"
#include <esphome/core/application.h>
#include <hap.h>
#include <hap_apple_chars.h>
#include <hap_apple_servs.h>

namespace esphome {
namespace homekit {
    class LightEntity : public HAPEntity {
    private:
        static constexpr const char* TAG = "LightEntity";
        light::LightState* lightPtr;
        static int light_write(hap_write_data_t write_data[], int count, void* serv_priv, void* write_priv)
        {
            light::LightState* lightPtr = (light::LightState*)serv_priv;
            ESP_LOGD(TAG, "Write called for Accessory %s (%s)", std::to_string(lightPtr->get_object_id_hash()).c_str(), lightPtr->get_name().c_str());
            int i, ret = HAP_SUCCESS;
            hap_write_data_t* write;
            for (i = 0; i < count; i++) {
                write = &write_data[i];
                if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_ON)) {
                    ESP_LOGD(TAG, "Received Write for Light '%s' state: %s", lightPtr->get_name().c_str(), write->val.b ? "On" : "Off");
                    ESP_LOGD(TAG, "[STATE] CURRENT STATE: %d", (int)(lightPtr->current_values.get_state() * 100));
                    write->val.b ? lightPtr->turn_on().set_save(true).perform() : lightPtr->turn_off().set_save(true).perform();
                    hap_char_update_val(write->hc, &(write->val));
                    *(write->status) = HAP_STATUS_SUCCESS;
                }
                if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_BRIGHTNESS)) {
                    ESP_LOGD(TAG, "Received Write for Light '%s' Level: %d", lightPtr->get_name().c_str(), write->val.i);
                    ESP_LOGD(TAG, "[LEVEL] CURRENT BRIGHTNESS: %d", (int)(lightPtr->current_values.get_brightness() * 100));
                    ESP_LOGD(TAG, "TARGET BRIGHTNESS: %d", (int)write->val.i);
                    lightPtr->make_call().set_save(true).set_brightness((float)(write->val.i) / 100).perform();
                    hap_char_update_val(write->hc, &(write->val));
                    *(write->status) = HAP_STATUS_SUCCESS;
                }
                if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_HUE)) {
                    ESP_LOGD(TAG, "Received Write for Light '%s' Hue: %.2f", lightPtr->get_name().c_str(), write->val.f);
                    int hue = 0;
                    float saturation = 0;
                    float colorValue = 0;
                    rgb_to_hsv(lightPtr->remote_values.get_red(), lightPtr->remote_values.get_green(), lightPtr->remote_values.get_blue(), hue, saturation, colorValue);
                    ESP_LOGD(TAG, "[HUE] CURRENT Hue: %d, Saturation: %.2f, Value: %.2f", hue, saturation, colorValue);
                    ESP_LOGD(TAG, "TARGET HUE: %.2f", write->val.f);
                    float tR = 0;
                    float tG = 0;
                    float tB = 0;
                    hsv_to_rgb(write->val.f, saturation, colorValue, tR, tG, tB);
                    ESP_LOGD(TAG, "TARGET RGB: %.2f %.2f %.2f", tR, tG, tB);
                    lightPtr->make_call().set_rgb(tR, tG, tB).set_save(true).perform();
                    hap_char_update_val(write->hc, &(write->val));
                    *(write->status) = HAP_STATUS_SUCCESS;
                }
                if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_SATURATION)) {
                    ESP_LOGD(TAG, "Received Write for Light '%s' Saturation: %.2f", lightPtr->get_name().c_str(), write->val.f);
                    int hue = 0;
                    float saturation = 0;
                    float colorValue = 0;
                    rgb_to_hsv(lightPtr->remote_values.get_red(), lightPtr->remote_values.get_green(), lightPtr->remote_values.get_blue(), hue, saturation, colorValue);
                    ESP_LOGD(TAG, "[SATURATION] CURRENT Hue: %d, Saturation: %.2f, Value: %.2f", hue, saturation, colorValue);
                    ESP_LOGD(TAG, "TARGET SATURATION: %.2f", write->val.f);
                    float tR = 0;
                    float tG = 0;
                    float tB = 0;
                    hsv_to_rgb(hue, write->val.f / 100, colorValue, tR, tG, tB);
                    ESP_LOGD(TAG, "TARGET RGB: %.2f %.2f %.2f", tR, tG, tB);
                    lightPtr->make_call().set_rgb(tR, tG, tB).set_save(true).perform();
                    hap_char_update_val(write->hc, &(write->val));
                    *(write->status) = HAP_STATUS_SUCCESS;
                }
                if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_COLOR_TEMPERATURE)) {
                    ESP_LOGD(TAG, "Received Write for Light '%s' Level: %d", lightPtr->get_name().c_str(), write->val.i);
                    ESP_LOGD(TAG, "[LEVEL] CURRENT COLOR TEMPERATURE(mired): %.2f", lightPtr->current_values.get_color_temperature());
                    ESP_LOGD(TAG, "TARGET COLOR TEMPERATURE(mired): %lu", write->val.u);
                    lightPtr->make_call().set_color_temperature(write->val.u).set_save(true).perform();
                    hap_char_update_val(write->hc, &(write->val));
                    *(write->status) = HAP_STATUS_SUCCESS;
                } else {
                    *(write->status) = HAP_STATUS_RES_ABSENT;
                }
            }
            return ret;
        }
        static void on_light_update(light::LightState* obj)
        {
            bool rgb = obj->current_values.get_color_mode() & light::ColorCapability::RGB;
            bool level = obj->get_traits().supports_color_capability(light::ColorCapability::BRIGHTNESS);
            bool temperature = obj->current_values.get_color_mode() & (light::ColorCapability::COLOR_TEMPERATURE | light::ColorCapability::COLD_WARM_WHITE);
            if (rgb) {
                ESP_LOGD(TAG, "%s RED: %.2f, GREEN: %.2f, BLUE: %.2f", obj->get_name().c_str(), obj->current_values.get_red(), obj->current_values.get_green(), obj->current_values.get_blue());
            }
            ESP_LOGD(TAG, "%s state: %d brightness: %d", obj->get_name().c_str(), (int)(obj->current_values.get_state() * 100), (int)(obj->current_values.get_brightness() * 100));
            hap_acc_t* acc = hap_acc_get_by_aid(hap_get_unique_aid(std::to_string(obj->get_object_id_hash()).c_str()));
            if (acc) {
                hap_serv_t* hs = hap_acc_get_serv_by_uuid(acc, HAP_SERV_UUID_LIGHTBULB);
                hap_char_t* on_char = hap_serv_get_char_by_uuid(hs, HAP_CHAR_UUID_ON);
                hap_val_t state;
                state.b = obj->current_values.get_state();
                hap_char_update_val(on_char, &state);
                if (level) {
                    hap_char_t* level_char = hap_serv_get_char_by_uuid(hs, HAP_CHAR_UUID_BRIGHTNESS);
                    hap_val_t level;
                    level.i = (int)(obj->current_values.get_brightness() * 100);
                    hap_char_update_val(level_char, &level);
                }
                if (rgb) {
                    hap_char_t* hue_char = hap_serv_get_char_by_uuid(hs, HAP_CHAR_UUID_HUE);
                    hap_char_t* saturation_char = hap_serv_get_char_by_uuid(hs, HAP_CHAR_UUID_SATURATION);
                    hap_val_t hue;
                    hap_val_t saturation;
                    int cHue = 0;
                    float cSaturation = 0;
                    float colorValue = 0;
                    rgb_to_hsv(obj->current_values.get_red(), obj->current_values.get_green(), obj->current_values.get_blue(), cHue, cSaturation, colorValue);
                    hue.f = cHue;
                    saturation.f = cSaturation * 100;
                    hap_char_update_val(hue_char, &hue);
                    hap_char_update_val(saturation_char, &saturation);
                }
                if (temperature) {
                    hap_char_t* temp_char = hap_serv_get_char_by_uuid(hs, HAP_CHAR_UUID_COLOR_TEMPERATURE);
                    hap_val_t temp;
                    temp.u = obj->current_values.get_color_temperature();
                    hap_char_update_val(temp_char, &temp);
                }
            }
        }
        static int acc_identify(hap_acc_t* ha)
        {
            ESP_LOGI(TAG, "Accessory identified");
            return HAP_SUCCESS;
        }

    public:
        LightEntity(light::LightState* lightPtr)
            : HAPEntity({ { MODEL, "HAP-LIGHT" } })
            , lightPtr(lightPtr)
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
            std::string accessory_name = lightPtr->get_name();
            if (accessory_info[NAME] == NULL) {
                acc_cfg.name = (char*)accessory_name.c_str();
            } else {
                acc_cfg.name = (char*)accessory_info[NAME];
            }
            if (accessory_info[SN] == NULL) {
                acc_cfg.serial_num = (char*)std::to_string(lightPtr->get_object_id_hash()).c_str();
            } else {
                acc_cfg.serial_num = (char*)accessory_info[SN];
            }
            accessory = hap_acc_create(&acc_cfg);
            int hue = 0;
            float saturation = 0;
            float colorValue = 0;
            rgb_to_hsv(lightPtr->current_values.get_red(), lightPtr->current_values.get_green(), lightPtr->current_values.get_blue(), hue, saturation, colorValue);
            service = hap_serv_lightbulb_create(lightPtr->current_values.get_state());
            hap_serv_add_char(service, hap_char_name_create(accessory_name.data()));
            if (lightPtr->get_traits().supports_color_capability(light::ColorCapability::BRIGHTNESS)) {
                hap_serv_add_char(service, hap_char_brightness_create(lightPtr->current_values.get_brightness() * 100));
            }
            if (lightPtr->get_traits().supports_color_capability(light::ColorCapability::RGB)) {
                hap_serv_add_char(service, hap_char_hue_create(hue));
                hap_serv_add_char(service, hap_char_saturation_create(saturation * 100));
            }
            if (lightPtr->get_traits().supports_color_capability(light::ColorCapability::COLOR_TEMPERATURE) || lightPtr->get_traits().supports_color_capability(light::ColorCapability::COLD_WARM_WHITE)) {
                hap_serv_add_char(service, hap_char_color_temperature_create(lightPtr->current_values.get_color_temperature()));
            }
            ESP_LOGD(TAG, "ID HASH: %lu", lightPtr->get_object_id_hash());
            hap_serv_set_priv(service, lightPtr);

            /* Set the write callback for the service */
            hap_serv_set_write_cb(service, light_write);

            /* Add the Light Service to the Accessory Object */
            hap_acc_add_serv(accessory, service);

            /* Add the Accessory to the HomeKit Database */
            hap_add_bridged_accessory(accessory, hap_get_unique_aid(std::to_string(lightPtr->get_object_id_hash()).c_str()));
            if (!lightPtr->is_internal())
                lightPtr->add_new_target_state_reached_callback([this]() { LightEntity::on_light_update(lightPtr); });

            ESP_LOGI(TAG, "Light '%s' linked to HomeKit", accessory_name.c_str());
        }
    };
}
}
#endif
