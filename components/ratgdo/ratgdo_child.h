#pragma once

#include "esphome/core/helpers.h"

#include "ratgdo.h"
namespace esphome {
namespace ratgdo {

    // Forward declare RATGDOComponent
    class RATGDOComponent;


    class RATGDOClient : public Parented<RATGDOComponent> {
    public:
        virtual void on_door_state(RATGDOComponent::DoorState state) = 0;
        virtual void on_light_state(RATGDOComponent::LightState state) = 0;
        virtual void on_lock_state(RATGDOComponent::LockState state) = 0;
        virtual void on_motion_state(RATGDOComponent::MotionState state) = 0;
        virtual void on_obstruction_state(RATGDOComponent::ObstructionState state) = 0;

    protected:
        friend RATGDOComponent;
        virtual std::string describe() = 0;
    };

} // namespace ratgdo
} // namespace esphome
