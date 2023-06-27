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
        this->parent_->subscribe_door_state([=](DoorState state, float position) {
            this->on_door_state(state, position);
        });
    }

    void RATGDOCover::on_door_state(DoorState state, float position)
    {
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
            break;
        case DoorState::CLOSING:
            this->current_operation = COVER_OPERATION_CLOSING;
            this->position = position;
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

        this->publish_state();
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
            this->parent_->stop_door();
        }
        if (call.get_toggle()) {
            this->parent_->toggle_door();
        }
        if (call.get_position().has_value()) {
            auto pos = *call.get_position();
            if (pos == COVER_OPEN) {
                this->parent_->open_door();
            } else if (pos == COVER_CLOSED) {
                this->parent_->close_door();
            } else {
                this->parent_->door_move_to_position(pos);
            }
        }
    }

} // namespace ratgdo
} // namespace esphome
