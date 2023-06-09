#pragma once

#include "esphome/core/helpers.h"

#include "ratgdo.h"
#include "ratgdo_state.h"

namespace esphome {
namespace ratgdo {

    // Forward declare RATGDOComponent
    class RATGDOComponent;

    class RATGDOClient : public Parented<RATGDOComponent> {
    public:
        virtual void on_door_state(DoorState state);
        virtual void on_light_state(LightState state);
        virtual void on_lock_state(LockState state);
        virtual void on_motion_state(MotionState state);
        virtual void on_obstruction_state(ObstructionState state);
        virtual void on_motor_state(MotorState state);
        virtual void on_rolling_code_change(uint32_t rollingCodeCounter);
        virtual void on_openings_change(uint32_t openings);
        virtual void on_button_state(ButtonState state);

    protected:
        friend RATGDOComponent;
    };

} // namespace ratgdo
} // namespace esphome
