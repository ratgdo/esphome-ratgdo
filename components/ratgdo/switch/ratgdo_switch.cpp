#include "ratgdo_switch.h"
#include "../ratgdo_state.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ratgdo {

    static const char* const TAG = "ratgdo.switch";

    void RATGDOSwitch::dump_config()
    {
        LOG_SWITCH("", "RATGDO Switch", this);
        switch (this->switch_type_) {
        case SwitchType::RATGDO_LEARN:
            ESP_LOGCONFIG(TAG, "  Type: Learn");
            break;
        case SwitchType::RATGDO_LED:
            ESP_LOGCONFIG(TAG, "  Type: LED");
            break;
        case SwitchType::RATGDO_BEEP_ON_ARRIVAL:
            ESP_LOGCONFIG(TAG, "  Type: Beep on arrival");
            break;
        default:
            break;
        }
    }

    void RATGDOSwitch::setup()
    {
        switch (this->switch_type_) {
        case SwitchType::RATGDO_LEARN:
            this->parent_->subscribe_learn_state([this](LearnState state) {
                this->publish_state(state == LearnState::ACTIVE);
            });
            break;
        case SwitchType::RATGDO_LED:
            this->pin_->setup();
#ifdef RATGDO_USE_VEHICLE_SENSORS
            this->parent_->subscribe_vehicle_arriving_state([this](VehicleArrivingState state) {
                this->write_state(state == VehicleArrivingState::YES);
            });
#endif
            break;
        case SwitchType::RATGDO_BEEP_ON_ARRIVAL: {
            bool value;
            this->pref_ = this->make_entity_preference<bool>();
            if (!this->pref_.load(&value)) {
                value = true;
            }
            this->parent_->set_beep_on_arrival(value);
            this->publish_state(value);
            this->parent_->subscribe_beep_on_arrival([this](bool state) {
                this->publish_state(state);
            });
            break;
        }
        default:
            break;
        }
    }

    void RATGDOSwitch::write_state(bool state)
    {
        switch (this->switch_type_) {
        case SwitchType::RATGDO_LEARN:
            if (state) {
                this->parent_->activate_learn();
            } else {
                this->parent_->inactivate_learn();
            }
            break;
        case SwitchType::RATGDO_LED:
            this->pin_->digital_write(state);
            this->publish_state(state);
            break;
        case SwitchType::RATGDO_BEEP_ON_ARRIVAL:
            this->parent_->set_beep_on_arrival(state);
            this->pref_.save(&state);
            this->publish_state(state);
            break;
        default:
            break;
        }
    }

} // namespace ratgdo
} // namespace esphome
