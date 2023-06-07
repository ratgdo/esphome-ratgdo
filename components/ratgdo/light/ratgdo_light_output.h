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
        void write_state(light::LightState* state) override;
        void setup_state(light::LightState* state) override { this->light_state_ = state; }
        light::LightState* get_state() { return this->light_state_; }

        void on_light_state(LightState state) override;

    protected:
        bool _is_on;
        light::LightState* light_state_;
    };

} // namespace ratgdo
} // namespace esphome
