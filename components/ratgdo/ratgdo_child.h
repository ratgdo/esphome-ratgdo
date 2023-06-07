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

    protected:
        friend RATGDOComponent;
    };

} // namespace ratgdo
} // namespace esphome
