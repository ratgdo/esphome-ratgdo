#pragma once

#include "../ratgdo.h"
#include "../ratgdo_state.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"

namespace esphome {
namespace ratgdo {

    enum SwitchType {
        RATGDO_LEARN,
        RATGDO_LED
    };

    class RATGDOSwitch : public switch_::Switch, public RATGDOClient, public Component {
    public:
        void dump_config() override;
        void setup() override;
        void set_switch_type(SwitchType switch_type_) { this->switch_type_ = switch_type_; }

        void write_state(bool state) override;
        void set_pin(GPIOPin* pin) { pin_ = pin; }

    protected:
        SwitchType switch_type_;
        GPIOPin* pin_;
    };

} // namespace ratgdo
} // namespace esphome
