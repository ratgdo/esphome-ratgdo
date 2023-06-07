#pragma once

#include "esphome/core/helpers.h"

#include "ratgdo.h"
namespace esphome {
namespace ratgdo {

    // Forward declare RATGDOComponent
    class RATGDOComponent;

    class RATGDOClient : public Parented<RATGDOComponent> {
    public:
        virtual void on_door_state(DoorState state) = 0;
        virtual void on_light_state(LightState state) = 0;
        virtual void on_lock_state(LockState state) = 0;
        virtual void on_motion_state(MotionState state) = 0;
        virtual void on_obstruction_state(ObstructionState state) = 0;

    protected:
        friend RATGDOComponent;
        virtual std::string describe() = 0;
    };

} // namespace ratgdo
} // namespace esphome
