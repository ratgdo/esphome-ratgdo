#pragma once

#include "../ratgdo.h"
#include "../ratgdo_state.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/component.h"

namespace esphome {
namespace ratgdo {

    enum SwitchType {
        RATGDO_LEARN,
        RATGDO_EMULATION
    };

    class RATGDOSwitch : public switch_::Switch, public RATGDOClient, public Component {
    public:
        void dump_config() override;
        void setup() override;
        void set_switch_type(SwitchType switch_type_) { this->switch_type_ = switch_type_; }

        void write_state(bool state) override;

    protected:
        SwitchType switch_type_;
        ESPPreferenceObject pref_;
    };

} // namespace ratgdo
} // namespace esphome
