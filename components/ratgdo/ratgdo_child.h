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
        void on_door_state(esphome::ratgdo::DoorState state) {};
        void on_light_state(esphome::ratgdo::LightState state) {};
        void on_lock_state(esphome::ratgdo::LockState state) {};
        void on_motion_state(esphome::ratgdo::MotionState state) {};
        void on_obstruction_state(esphome::ratgdo::ObstructionState state) {};

    protected:
        friend RATGDOComponent;
    };

} // namespace ratgdo
} // namespace esphome
