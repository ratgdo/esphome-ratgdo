#include "../ratgdo_state.h"
#include "esphome/core/log.h"
#include "ratgdo_cover.h"

namespace esphome {
namespace ratgdo {

    using namespace esphome::cover;

    static const char* const TAG = "ratgdo.light";

    void RATGDOLightOutput::dump_config()
    {
        LOG_CONFIG("", "RATGDO Light");
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
    LightState* RATGDOLightOutput::get_state() { return this->state_; }

    void RATGDOLightOutput::write_state(light::LightState* state)
    {
        bool binary;
        state->current_values_as_binary(&binary);
        if (binary) {
            this->parent_->turnOnLight();
        } else {
            this->parent_->turnOffLight();
        }
    }

} // namespace ratgdo
} // namespace esphome
