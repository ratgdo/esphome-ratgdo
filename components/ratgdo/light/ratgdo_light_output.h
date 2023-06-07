#pragma once

#include "../ratgdo.h"
#include "../ratgdo_child.h"
#include "../ratgdo_state.h"
#include "esphome/components/light/light_output.h"
#include "esphome/core/component.h"

namespace esphome {
namespace ratgdo {

    class RATGDOLightOutput : public light::LightOutput, public RATGDOClient, public Component {
    public:
        void dump_config() override;
        light::LightTraits get_traits() override;
        void setup_state(light::LightState* state) override { this->state_ = state; }
        LightState* get_state() { return this->state_; }

        void on_motion_state(esphome::ratgdo::MotionState state) override;
        void on_obstruction_state(esphome::ratgdo::ObstructionState state) override;
        void on_door_state(esphome::ratgdo::DoorState state) override;
        void on_light_state(esphome::ratgdo::LightState state) override;
        void on_lock_state(esphome::ratgdo::LockState state) override;

    protected:
        bool _is_on;
        light::LightState* state_;
    };

} // namespace ratgdo
} // namespace esphome
