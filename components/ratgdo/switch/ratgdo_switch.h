#pragma once

#include "../ratgdo.h"
#include "../ratgdo_state.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/component.h"

namespace esphome {
namespace ratgdo {

    enum SwitchType {
        RATGDO_LEARN
    };

    class RATGDOSwitch : public switch_::Switch, public RATGDOClient, public Component {
    public:
        void dump_config() override;
        void setup() override;
        void set_switch_type(SwitchType switch_type_) { this->switch_type_ = switch_type_; }

        void on_learn_state(LearnState state);
        void write_state(bool state) override;

    protected:
        SwitchType switch_type_;
    };

} // namespace ratgdo
} // namespace esphome
