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
    void RATGDOLightOutput::on_motion_state(esphome::ratgdo::MotionState state) { }
    void RATGDOLightOutput::on_obstruction_state(esphome::ratgdo::ObstructionState state) { }
    void RATGDOLightOutput::on_door_state(esphome::ratgdo::DoorState state) { }
    void RATGDOLightOutput::on_light_state(esphome::ratgdo::LightState state) { }
    void RATGDOLightOutput::on_lock_state(esphome::ratgdo::LockState state) { }

    LightTraits RATGDOLightOutput::get_traits()
    {
        auto traits = LightTraits();
        traits.set_supported_color_modes({ light::ColorMode::ON_OFF });
        return traits;
    }

    void RATGDOLightOutput::write_state(light::LightState* state)
    {
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
