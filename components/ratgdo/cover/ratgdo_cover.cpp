#include "ratgdo_cover.h"
#include "../ratgdo_state.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ratgdo {

    using namespace esphome::cover;

    static const char* const TAG = "ratgdo.cover";

    void RATGDOCover::dump_config()
    {
        LOG_COVER("", "RATGDO Cover", this);
    }

    void RATGDOCover::setup()
    {
        auto state = this->restore_state_();
        if (state.has_value()) {
            this->parent_->set_door_position(state.value().position);
        }
        this->parent_->subscribe_door_state([this](DoorState state, float position) {
            this->on_door_state(state, position);
        });
    }

    void RATGDOCover::on_door_state(DoorState state, float position)
    {
        // ESP_LOGD("ON_DOOR_STATE", "%s %f", DoorState_to_string(state), position);
        bool save_to_flash = true;
        switch (state) {
        case DoorState::OPEN:
            this->position = COVER_OPEN;
            this->current_operation = COVER_OPERATION_IDLE;
            break;
        case DoorState::CLOSED:
            this->position = COVER_CLOSED;
            this->current_operation = COVER_OPERATION_IDLE;
            break;
        case DoorState::OPENING:
            this->current_operation = COVER_OPERATION_OPENING;
            this->position = position;
            save_to_flash = false;
            break;
        case DoorState::CLOSING:
            this->current_operation = COVER_OPERATION_CLOSING;
            this->position = position;
            save_to_flash = false;
            break;
        case DoorState::STOPPED:
            this->current_operation = COVER_OPERATION_IDLE;
            this->position = position;
        case DoorState::UNKNOWN:
        default:
            this->current_operation = COVER_OPERATION_IDLE;
            this->position = position;

            break;
        }

        this->publish_state(save_to_flash);
    }

    CoverTraits RATGDOCover::get_traits()
    {
        auto traits = CoverTraits();
        traits.set_supports_stop(true);
        traits.set_supports_toggle(true);
        traits.set_supports_position(true);
        return traits;
    }

    void RATGDOCover::control(const CoverCall& call)
    {
        if (call.get_stop()) {
            this->parent_->door_stop();
        }
        if (call.get_toggle()) {
            this->parent_->door_toggle();
        }
        if (call.get_position().has_value()) {
            auto pos = *call.get_position();
            if (pos == COVER_OPEN) {
                this->parent_->door_open();
            } else if (pos == COVER_CLOSED) {
                this->parent_->door_close();
            } else {
                this->parent_->door_move_to_position(pos);
            }
        }
    }

} // namespace ratgdo
} // namespace esphome
