#include "ratgdo_switch.h"
#include "../ratgdo_state.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ratgdo {

    static const char* const TAG = "ratgdo.switch";

    void RATGDOSwitch::dump_config()
    {
        LOG_SWITCH("", "RATGDO Switch", this);
        if (this->switch_type_ == SwitchType::RATGDO_LOCK) {
            ESP_LOGCONFIG(TAG, "  Type: Lock");
        } else if (this->switch_type_ == SwitchType::RATGDO_HOLDOPEN) {
            ESP_LOGCONFIG(TAG, "  Type: Hold Open");
        }        
    }

    void RATGDOSwitch::setup()
    {
        if (this->switch_type_ == SwitchType::RATGDO_LOCK) {
            this->parent_->subscribe_lock_state([=](LockState state) {
                this->on_lock_state(state);
            });            
        } else if (this->switch_type_ == SwitchType::RATGDO_HOLDOPEN) {
            this->parent_->subscribe_hold_state([=](HoldState state) {
                this->on_hold_state(state);
            });               
        } 
    }

    void RATGDOSwitch::on_lock_state(LockState state)
    {
        bool value = state == LockState::LOCKED;
        this->state = value;
        this->publish_state(value);
    }
    void RATGDOSwitch::on_hold_state(HoldState state)
    {
        bool value = state == HoldState::HOLD_ENABLED;
        this->state = value;
        this->publish_state(value);
    }    

    void RATGDOSwitch::write_state(bool state)
    {
        if (this->switch_type_ == SwitchType::RATGDO_LOCK) {
            if (state) {
                this->parent_->lock();
            } else {
                this->parent_->unlock();
            }
        } else if (this->switch_type_ == SwitchType::RATGDO_HOLDOPEN) {
            if (state) {
                this->parent_->hold_enable();
            } else {
                this->parent_->hold_disable();
            }
        }         
    }

} // namespace ratgdo
} // namespace esphome
