#include "esphome/core/helpers.h"

#include "ratgdo.h"
#include "ratgdo_state.h"

namespace esphome {
namespace ratgdo {

    void RATGDOClient::on_door_state(DoorState state) {};
    void RATGDOClient::on_light_state(LightState state) {};
    void RATGDOClient::on_lock_state(LockState state) {};
    void RATGDOClient::on_motion_state(MotionState state) {};
    void RATGDOClient::on_obstruction_state(ObstructionState state) {};

} // namespace ratgdo
} // namespace esphome
