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
        bool value = state == LockState::LOCK_STATE_LOCKED;
        this->state = value;
        this->publish_state(value);
    }
    void RATGDOSwitch::write_state(bool state)
    {
        ESP_LOGD(TAG, "name: %s this->type_:%d state: %d", this->get_name(), this->switch_type_, state);
        if (state) {
            this->parent_->lock();
        } else {
            this->parent_->unlock();
        }
    }

} // namespace ratgdo
} // namespace esphome
