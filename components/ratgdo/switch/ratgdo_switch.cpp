#include "ratgdo_switch.h"
#include "../ratgdo_state.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ratgdo {

    static const char* const TAG = "ratgdo.switch";

    void RATGDOSwitch::dump_config()
    {
        LOG_SWITCH("", "RATGDO Switch", this);
        ESP_LOGCONFIG(TAG, "  Type: Lock");
    }

    void RATGDOSwitch::on_lock_state(LockState state)
    {
        this->state(state == LockState::LOCK_STATE_LOCKED) this->publish_state();
    }
    void RATGDOSwitch::turn_on()
    {
        ESP_LOGD(TAG, "name: %s this->type_:%d ON", this->get_name(), this->switch_type_);
        this->parent_->lock(value);
        this->publish_state(value);
    }
    void RATGDOSwitch::turn_off()
    {
        ESP_LOGD(TAG, "name: %s this->type_:%d OFF", this->get_name(), this->switch_type_);
        this->parent_->unlock(value);
        this->publish_state(value);
    }

} // namespace ratgdo
} // namespace esphome
