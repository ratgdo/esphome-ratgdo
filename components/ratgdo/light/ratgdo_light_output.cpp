#include "ratgdo_light_output.h"
#include "../ratgdo_state.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ratgdo {

    using namespace esphome::light;

    static const char* const TAG = "ratgdo.light";

    void RATGDOLightOutput::dump_config()
    {
        ESP_LOGCONFIG("", "RATGDO Light");
    }

    void RATGDOLightOutput::setup()
    {
        this->parent_->subscribe_light_state([=](LightState s) {
            this->on_light_state(s);
        });
    }

    void RATGDOLightOutput::on_light_state(esphome::ratgdo::LightState state)
    {
        if (this->light_state_) {
            this->has_initial_state_ = true;
            set_state(state);
        }
    }
    void RATGDOLightOutput::set_state(esphome::ratgdo::LightState state)
    {

        bool is_on = state == LightState::LIGHT_STATE_ON;
        this->light_state_->current_values.set_state(is_on);
        this->light_state_->remote_values.set_state(is_on);
        this->light_state_->publish_state();
    }
    void RATGDOLightOutput::setup_state(light::LightState* light_state)
    {
        esphome::ratgdo::LightState state = this->parent_->getLightState();
        this->light_state_ = light_state;
        this->set_state(state);
    }
    LightTraits RATGDOLightOutput::get_traits()
    {
        auto traits = LightTraits();
        traits.set_supported_color_modes({ light::ColorMode::ON_OFF });
        return traits;
    }

    void RATGDOLightOutput::write_state(light::LightState* state)
    {
        if (!this->has_initial_state_)
            return;
        bool binary;
        state->current_values_as_binary(&binary);
        if (binary) {
            this->parent_->lightOn();
        } else {
            this->parent_->lightOff();
        }
    }

} // namespace ratgdo
} // namespace esphome
