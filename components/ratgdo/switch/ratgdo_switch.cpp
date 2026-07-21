#include "ratgdo_switch.h"
#include "../ratgdo_state.h"
#include "esphome/core/log.h"

namespace esphome::ratgdo {

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
    case SwitchType::RATGDO_REVERSE_ENCODER:
        ESP_LOGCONFIG(TAG, "  Type: Reverse Encoder");
        break;
    case SwitchType::RATGDO_AUTO_CLOSE:
        ESP_LOGCONFIG(TAG, "  Type: Auto Close");
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
#ifdef RATGDO_USE_ENCODER
    case SwitchType::RATGDO_REVERSE_ENCODER:
        this->pref_ = global_preferences->make_preference<bool>(fnv1_hash("ratgdo_reverse_encoder"));
        {
            bool stored = false;
            if (!this->pref_.load(&stored)) {
                ESP_LOGW(TAG, "Failed to load reverse_encoder preference. Defaulting to false.");
            }
            this->parent_->set_reverse_encoder(stored);
            this->publish_state(stored);
        }
        break;
#endif
    case SwitchType::RATGDO_AUTO_CLOSE:
        this->parent_->subscribe_ttc_state([this](TtcState state) {
            this->publish_state(ttc_is_counting(state));
        });
        break;
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
#ifdef RATGDO_USE_ENCODER
    case SwitchType::RATGDO_REVERSE_ENCODER:
        if (!this->pref_.save(&state)) {
            ESP_LOGW(TAG, "Failed to save reverse_encoder preference.");
            return;
        }
        this->parent_->set_reverse_encoder(state);
        this->parent_->recalculate_encoder_state();
        this->publish_state(state);
        break;
#endif
    case SwitchType::RATGDO_AUTO_CLOSE:
        // NOTE: No publish_state() call here: The auto_close switch's
        // published state comes entirely from the subscribe_ttc_state()
        // callback, not from write_state(). ttc_toggle_hold() guards
        // against being called while ttc_state is UNKNOWN itself, so
        // that doesn't need checking here either.
        if (state != ttc_is_counting(*this->parent_->ttc_state)) {
            // The user interface is a switch, but the message protocol
            // is toggle. So only send a ttc toggle message if the state
            // really needs to change.
            this->parent_->ttc_toggle_hold();
        }
        break;
    default:
        break;
    }
}

} // namespace esphome::ratgdo
