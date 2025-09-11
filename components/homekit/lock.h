#pragma once
#include <esphome/core/defines.h>
#ifdef USE_LOCK
#include "hap_entity.h"
#include <esphome/core/application.h>
#include <hap.h>
#include <hap_apple_chars.h>
#include <hap_apple_servs.h>
#ifdef USE_HOMEKEY
#include "automation.h"
#include "const.h"
#include <esphome/components/pn532/pn532.h>
#include <esphome/core/base_automation.h>
#include <nvs.h>
#endif

namespace esphome {
namespace homekit {
    class LockEntity : public HAPEntity {
    private:
        static constexpr const char* TAG = "LockEntity";
        lock::Lock* ptrToLock;
#ifdef USE_HOMEKEY
        static nvs_handle savedHKdata;
        static pn532::PN532* nfc_ctx;
        std::vector<HKAuthTrigger*> triggers_onhk_;
        std::vector<HKFailTrigger*> triggers_onhk_fail_;
#endif
        static void on_lock_update(lock::Lock* obj);
        static int lock_write(hap_write_data_t write_data[], int count, void* serv_priv, void* write_priv);
        static int acc_identify(hap_acc_t* ha);

    public:
        LockEntity(lock::Lock* lockPtr);
        void setup();
#ifdef USE_HOMEKEY
        void set_nfc_ctx(pn532::PN532* ctx);
        void set_hk_hw_finish(HKFinish color);
        void register_onhk_trigger(HKAuthTrigger* trig);
        void register_onhkfail_trigger(HKFailTrigger* trig);
#endif
    };
}
}
#endif
