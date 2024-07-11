#include "ratgdo_switch.h"
#include "../ratgdo_state.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ratgdo {
    using protocol::SetEnableEmulationMode;

    static const char* const TAG = "ratgdo.switch";

    void RATGDOSwitch::dump_config()
    {
        LOG_SWITCH("", "RATGDO Switch", this);
        if (this->switch_type_ == SwitchType::RATGDO_LEARN) {
            ESP_LOGCONFIG(TAG, "  Type: Learn");
        }
    }

    void RATGDOSwitch::setup()
    {
        if (this->switch_type_ == SwitchType::RATGDO_LEARN) {
            this->parent_->subscribe_learn_state([=](LearnState state) {
                this->publish_state(state == LearnState::ACTIVE);
            });
        } else if (this->switch_type_ == SwitchType::RATGDO_EMULATION) {
            bool value = false;
            this->pref_ = global_preferences->make_preference<float>(this->get_object_id_hash());
            this->pref_.load(&value);
            this->state = value;
            this->parent_->emulation_state = value ? EmulationState::ACTIVE : EmulationState::INACTIVE;

            this->parent_->subscribe_emulation_state([=](EmulationState state) {
                this->publish_state(state == EmulationState::ACTIVE);
            });

            this->parent_->call_protocol(SetEnableEmulationMode { static_cast<bool>(this->state) });
        }
    }

    void RATGDOSwitch::write_state(bool state)
    {
        if (this->switch_type_ == SwitchType::RATGDO_LEARN) {
            if (state) {
                this->parent_->activate_learn();
            } else {
                this->parent_->inactivate_learn();
            }
        } else if (this->switch_type_ == SwitchType::RATGDO_EMULATION) {
            this->state = state;
            this->parent_->emulation_state = state ? EmulationState::ACTIVE : EmulationState::INACTIVE;
            this->parent_->call_protocol(SetEnableEmulationMode { static_cast<bool>(state) });

            this->pref_.save(&state);
        }
    }

} // namespace ratgdo
} // namespace esphome
