#ifdef USE_LOCK
#include "lock.h"

namespace esphome {
namespace homekit {
#ifdef USE_HOMEKEY
    readerData_t LockEntity::readerData;
    nvs_handle LockEntity::savedHKdata;
    pn532::PN532* LockEntity::nfc_ctx;
#endif

    void LockEntity::on_lock_update(lock::Lock* obj)
    {
        ESP_LOGD(TAG, "%s state: %s", obj->get_name().c_str(),
            lock_state_to_string(obj->state));
        hap_acc_t* acc = hap_acc_get_by_aid(
            hap_get_unique_aid(std::to_string(obj->get_object_id_hash()).c_str()));
        if (acc) {
            hap_serv_t* hs = hap_acc_get_serv_by_uuid(acc, HAP_SERV_UUID_LOCK_MECHANISM);
            if (hs) {
                hap_char_t* current_state = hap_serv_get_char_by_uuid(hs, HAP_CHAR_UUID_LOCK_CURRENT_STATE);
                hap_char_t* target_state = hap_serv_get_char_by_uuid(hs, HAP_CHAR_UUID_LOCK_TARGET_STATE);
                if (current_state && target_state) {
                    hap_val_t c;
                    hap_val_t t;
                    if (obj->state == lock::LockState::LOCK_STATE_LOCKED || obj->state == lock::LockState::LOCK_STATE_UNLOCKED) {
                        c.i = obj->state % 2;
                        t.i = obj->state % 2;
                        hap_char_update_val(current_state, &c);
                        hap_char_update_val(target_state, &t);
                    } else if (obj->state == lock::LockState::LOCK_STATE_LOCKING || obj->state == lock::LockState::LOCK_STATE_UNLOCKING) {
                        t.i = (obj->state % 5) % 3;
                        hap_char_update_val(target_state, &t);
                    } else if (obj->state == lock::LockState::LOCK_STATE_JAMMED) {
                        c.i = obj->state;
                        hap_char_update_val(current_state, &c);
                    }
                }
            }
        }
    }

    int LockEntity::lock_write(hap_write_data_t write_data[], int count,
        void* serv_priv, void* write_priv)
    {
        lock::Lock* lockPtr = (lock::Lock*)serv_priv;
        ESP_LOGD(TAG, "Write called for Accessory '%s'(%s)",
            lockPtr->get_name().c_str(),
            std::to_string(lockPtr->get_object_id_hash()).c_str());
        int i, ret = HAP_SUCCESS;
        hap_write_data_t* write;
        for (i = 0; i < count; i++) {
            write = &write_data[i];
            if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_LOCK_TARGET_STATE)) {
                ESP_LOGD(TAG, "Target State req: %d", write->val.i);
                write->val.i ? lockPtr->lock() : lockPtr->unlock();
                hap_char_update_val(write->hc, &(write->val));
                *(write->status) = HAP_STATUS_SUCCESS;
            } else {
                *(write->status) = HAP_STATUS_RES_ABSENT;
            }
        }
        return ret;
    }

    int LockEntity::acc_identify(hap_acc_t* ha)
    {
        ESP_LOGI(TAG, "Accessory identified");
        return HAP_SUCCESS;
    }

    LockEntity::LockEntity(lock::Lock* lockPtr)
        : HAPEntity({ { MODEL, "HAP-LOCK" } })
        , ptrToLock(lockPtr)
    {
    }

#ifdef USE_HOMEKEY
    void LockEntity::register_onhk_trigger(HKAuthTrigger* trig)
    {
        triggers_onhk_.push_back(trig);
    }
    void LockEntity::register_onhkfail_trigger(HKFailTrigger* trig)
    {
        triggers_onhk_fail_.push_back(trig);
    }
    void LockEntity::set_hk_hw_finish(HKFinish color)
    {
        // Simplified implementation
    }
    void LockEntity::set_nfc_ctx(pn532::PN532* ctx)
    {
        nfc_ctx = ctx;
    }
#endif

    void LockEntity::setup()
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
        hap_serv_t* lockMechanism = nullptr;
        std::string accessory_name = ptrToLock->get_name();

        if (accessory_info[NAME] == NULL) {
            acc_cfg.name = (char*)accessory_name.c_str();
        } else {
            acc_cfg.name = (char*)accessory_info[NAME];
        }
        if (accessory_info[SN] == NULL) {
            acc_cfg.serial_num = (char*)std::to_string(ptrToLock->get_object_id_hash()).c_str();
        } else {
            acc_cfg.serial_num = (char*)accessory_info[SN];
        }

        accessory = hap_acc_create(&acc_cfg);
        lockMechanism = hap_serv_lock_mechanism_create(ptrToLock->state, ptrToLock->state);

        ESP_LOGD(TAG, "ID HASH: %lu", ptrToLock->get_object_id_hash());
        hap_serv_set_priv(lockMechanism, ptrToLock);

        /* Set the write callback for the service */
        hap_serv_set_write_cb(lockMechanism, lock_write);

        /* Add the Lock Service to the Accessory Object */
        hap_acc_add_serv(accessory, lockMechanism);

        /* Add the Accessory to the HomeKit Database */
        hap_add_bridged_accessory(
            accessory, hap_get_unique_aid(std::to_string(ptrToLock->get_object_id_hash()).c_str()));

        if (!ptrToLock->is_internal())
            ptrToLock->add_on_state_callback(
                [this]() { LockEntity::on_lock_update(ptrToLock); });

        ESP_LOGI(TAG, "Lock '%s' linked to HomeKit", accessory_name.c_str());
    }

} // namespace homekit
} // namespace esphome
#endif
