#include "ratgdo_switch.h"
#include "../ratgdo_state.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ratgdo {

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
        } else if (this->switch_type_ == SwitchType::RATGDO_LED) {
            this->pin_->setup();
            this->parent_->subscribe_vehicle_arriving_state([=](VehicleArrivingState state) {
                this->write_state(state == VehicleArrivingState::YES);
            });
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
        } else if (this->switch_type_ == SwitchType::RATGDO_LED) {
            this->pin_->digital_write(state);
            this->publish_state(state);
        }
    }

} // namespace ratgdo
} // namespace esphome
