#pragma once
#include <esphome/core/defines.h>
#ifdef USE_COVER
#include "hap_entity.h"
#include <esphome/core/application.h>
#include <hap.h>
#include <hap_apple_chars.h>
#include <hap_apple_servs.h>

namespace esphome {
namespace homekit {
    class CoverEntity : public HAPEntity {
    private:
        static constexpr const char* TAG = "CoverEntity";
        static constexpr float POSITION_THRESHOLD = 0.02f; // 2% threshold for position comparisons
        cover::Cover* coverPtr;

        static int cover_write(hap_write_data_t write_data[], int count, void* serv_priv, void* write_priv)
        {
            cover::Cover* coverPtr = (cover::Cover*)serv_priv;
            ESP_LOGD(TAG, "Write called for Accessory %s (%s)", std::to_string(coverPtr->get_object_id_hash()).c_str(), coverPtr->get_name().c_str());
            int i, ret = HAP_SUCCESS;
            hap_write_data_t* write;
            for (i = 0; i < count; i++) {
                write = &write_data[i];

                // Check for target door state characteristic (0x32)
                if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_TARGET_DOOR_STATE)) {
                    // Validate TargetDoorState value (only 0=Open or 1=Closed are valid)
                    if (write->val.i != 0 && write->val.i != 1) {
                        ESP_LOGW(TAG, "Invalid TargetDoorState value %d for garage door '%s'. Only 0 (Open) or 1 (Closed) are valid.", write->val.i, coverPtr->get_name().c_str());
                        *(write->status) = HAP_STATUS_VAL_INVALID;
                    } else {
                        ESP_LOGD(TAG, "Received Write for garage door '%s' -> %s", coverPtr->get_name().c_str(), write->val.i == 0 ? "Open" : "Close");
                        if (write->val.i == 0) {
                            // Open - Use new ESPHome API
                            auto call = coverPtr->make_call();
                            call.set_command_open();
                            call.perform();
                        } else {
                            // Close - Use new ESPHome API
                            auto call = coverPtr->make_call();
                            call.set_command_close();
                            call.perform();
                        }
                        hap_char_update_val(write->hc, &(write->val));
                        *(write->status) = HAP_STATUS_SUCCESS;
                    }
                } else {
                    *(write->status) = HAP_STATUS_RES_ABSENT;
                }
            }
            return ret;
        }

        static void on_cover_update(cover::Cover* obj)
        {
            ESP_LOGD(TAG, "%s state: %s", obj->get_name().c_str(), cover_operation_to_str(obj->current_operation));
            hap_acc_t* acc = hap_acc_get_by_aid(hap_get_unique_aid(std::to_string(obj->get_object_id_hash()).c_str()));
            if (acc) {
                hap_serv_t* hs = hap_acc_get_serv_by_uuid(acc, HAP_SERV_UUID_GARAGE_DOOR_OPENER); // Garage Door Opener

                if (hs) {
                    hap_char_t* current_state = hap_serv_get_char_by_uuid(hs, HAP_CHAR_UUID_CURRENT_DOOR_STATE); // Current Door State
                    hap_char_t* target_state = hap_serv_get_char_by_uuid(hs, HAP_CHAR_UUID_TARGET_DOOR_STATE); // Target Door State
                    hap_char_t* obstruction_detected = hap_serv_get_char_by_uuid(hs, HAP_CHAR_UUID_OBSTRUCTION_DETECTED); // Obstruction Detected

                    if (current_state && target_state && obstruction_detected) {
                        hap_val_t c, t, obstruction;

                        // Read current target_state value to preserve it when stopped - Fix function signature
                        const hap_val_t* target_val = hap_char_get_val(target_state);
                        if (target_val) {
                            t = *target_val;
                        } else {
                            t.i = 1; // Default to closed
                        }

                        // Initialize obstruction as false by default
                        obstruction.b = false;

                        // Map ESPHome cover states to HomeKit garage door states
                        switch (obj->current_operation) {
                        case cover::COVER_OPERATION_IDLE:
                            if (obj->position >= (1.0f - POSITION_THRESHOLD)) {
                                c.i = 0; // Open
                                t.i = 0; // Target Open
                            } else if (obj->position <= POSITION_THRESHOLD) {
                                c.i = 1; // Closed
                                t.i = 1; // Target Closed
                            } else {
                                c.i = 4; // Stopped
                                // Detect potential obstruction: cover stopped in an intermediate position
                                // This could indicate an obstruction was encountered
                                obstruction.b = true;
                                ESP_LOGD(TAG, "Garage door '%s' stopped at intermediate position (%.2f), potential obstruction detected", obj->get_name().c_str(), obj->position);
                            }
                            break;
                        case cover::COVER_OPERATION_OPENING:
                            c.i = 2; // Opening
                            t.i = 0; // Target Open
                            break;
                        case cover::COVER_OPERATION_CLOSING:
                            c.i = 3; // Closing
                            t.i = 1; // Target Closed
                            break;
                        }

                        // Only update characteristics if values have actually changed - Fix function signature
                        const hap_val_t* current_val = hap_char_get_val(current_state);
                        if (current_val && current_val->i != c.i) {
                            hap_char_update_val(current_state, &c);
                        }

                        const hap_val_t* target_val_check = hap_char_get_val(target_state);
                        if (target_val_check && target_val_check->i != t.i) {
                            hap_char_update_val(target_state, &t);
                        }

                        const hap_val_t* obstruction_val = hap_char_get_val(obstruction_detected);
                        if (obstruction_val && obstruction_val->b != obstruction.b) {
                            hap_char_update_val(obstruction_detected, &obstruction);
                            ESP_LOGD(TAG, "Garage door '%s' obstruction status updated to: %s", obj->get_name().c_str(), obstruction.b ? "true" : "false");
                        }
                    }
                }
            }
        }

        static int acc_identify(hap_acc_t* ha)
        {
            ESP_LOGI(TAG, "Accessory identified");
            return HAP_SUCCESS;
        }

    public:
        CoverEntity(cover::Cover* coverPtr)
            : HAPEntity({ { MODEL, "HAP-GARAGE-DOOR" } })
            , coverPtr(coverPtr)
        {
        }

        void setup()
        {
            hap_acc_cfg_t acc_cfg = {
                .name = nullptr,
                .model = (char*)accessory_info[MODEL],
                .manufacturer = (char*)accessory_info[MANUFACTURER],
                .serial_num = nullptr,
                .fw_rev = (char*)accessory_info[FW_REV],
                .hw_rev = (char*)"1.0",
                .pv = (char*)"1.1.0",
                .cid = HAP_CID_BRIDGE,
                .identify_routine = acc_identify,
                .hw_finish = HAP_HW_FINISH_OTHER
            };
            hap_acc_t* accessory = nullptr;
            hap_serv_t* service = nullptr;
            std::string accessory_name = coverPtr->get_name();

            if (accessory_info[NAME] == NULL) {
                acc_cfg.name = (char*)accessory_name.c_str();
            } else {
                acc_cfg.name = (char*)accessory_info[NAME];
            }
            if (accessory_info[SN] == NULL) {
                acc_cfg.serial_num = (char*)std::to_string(coverPtr->get_object_id_hash()).c_str();
            } else {
                acc_cfg.serial_num = (char*)accessory_info[SN];
            }

            /* Create accessory object */
            accessory = hap_acc_create(&acc_cfg);

            /* Create the garage door opener Service. */
            // Initialize with current states
            int current_state = 1; // Default to closed
            int target_state = 1; // Default to closed

            if (coverPtr->position >= (1.0f - POSITION_THRESHOLD)) {
                current_state = 0; // Open
                target_state = 0;
            } else if (coverPtr->position <= POSITION_THRESHOLD) {
                current_state = 1; // Closed
                target_state = 1;
            }

// Prefer typed creator if available (guard for older SDKs)
#ifdef HAP_SERV_GARAGE_DOOR_OPENER_CREATE
            service = hap_serv_garage_door_opener_create(current_state, target_state, false /* obstruction_detected */);
#else
            service = nullptr;
#endif
            if (!service) {
                // Fallback: manual service + typed characteristic creators
                service = hap_serv_create(HAP_SERV_UUID_GARAGE_DOOR_OPENER);
                if (service) {
                    hap_serv_add_char(service, hap_char_current_door_state_create(current_state));
                    hap_serv_add_char(service, hap_char_target_door_state_create(target_state));
                    // Fix function name typo
                    hap_serv_add_char(service, hap_char_obstruction_detect_create(false));
                }
            }

            if (!service) {
                ESP_LOGE(TAG, "Failed to create garage door service");
                return;
            }

            ESP_LOGD(TAG, "ID HASH: %lu", coverPtr->get_object_id_hash());
            hap_serv_set_priv(service, coverPtr);

            /* Set the write callback for the service */
            hap_serv_set_write_cb(service, cover_write);

            /* Add the Garage Door Service to the Accessory Object */
            hap_acc_add_serv(accessory, service);

            /* Add the Accessory to the HomeKit Database */
            hap_add_bridged_accessory(accessory, hap_get_unique_aid(std::to_string(coverPtr->get_object_id_hash()).c_str()));

            if (!coverPtr->is_internal())
                coverPtr->add_on_state_callback([this]() { CoverEntity::on_cover_update(coverPtr); });

            ESP_LOGI(TAG, "Garage Door '%s' linked to HomeKit", accessory_name.c_str());
        }
    };
}
}
#endif
