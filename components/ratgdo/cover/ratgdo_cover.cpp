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
    void RATGDOCover::on_motion_state(esphome::ratgdo::MotionState state) { }
    void RATGDOCover::on_obstruction_state(esphome::ratgdo::ObstructionState state) { }
    void RATGDOCover::on_door_state(esphome::ratgdo::DoorState state)
    {
        switch (state) {
        case esphome::ratgdo::DoorState::DOOR_STATE_OPEN:
            this->position = COVER_OPEN;
            this->current_operation = COVER_OPERATION_IDLE;
        case esphome::ratgdo::DoorState::DOOR_STATE_CLOSED:
            this->position = COVER_CLOSED;
            this->current_operation = COVER_OPERATION_IDLE;
        case esphome::ratgdo::DoorState::DOOR_STATE_OPENING:
            this->current_operation = COVER_OPERATION_OPENING;
        case esphome::ratgdo::DoorState::DOOR_STATE_CLOSING:
            this->current_operation = COVER_OPERATION_CLOSING;
        case esphome::ratgdo::DoorState::DOOR_STATE_STOPPED:
        default:
            this->current_operation = COVER_OPERATION_IDLE;

            break;
        }

        this->publish_state();
    }
    void RATGDOCover::on_light_state(esphome::ratgdo::LightState state) { }
    void RATGDOCover::on_lock_state(esphome::ratgdo::LockState state) { }

    CoverTraits RATGDOCover::get_traits()
    {
        auto traits = CoverTraits();
        traits.set_supports_stop(true);
        traits.set_supports_position(false);
        traits.set_supports_tilt(false);
        traits.set_is_assumed_state(false);
        return traits;
    }
    void RATGDOCover::control(const CoverCall& call) { }

} // namespace ratgdo
} // namespace esphome
