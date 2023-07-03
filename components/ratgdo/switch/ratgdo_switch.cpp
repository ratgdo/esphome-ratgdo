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

    void RATGDOSwitch::setup()
    {
        this->parent_->subscribe_lock_state([=](LockState state) {
            this->on_lock_state(state);
        });
    }

    void RATGDOSwitch::on_lock_state(LockState state)
    {
        bool value = state == LockState::LOCKED;
        this->state = value;
        this->publish_state(value);
    }
    void RATGDOSwitch::write_state(bool state)
    {
        if (state) {
            this->parent_->lock();
        } else {
            this->parent_->unlock();
        }
    }

} // namespace ratgdo
} // namespace esphome
