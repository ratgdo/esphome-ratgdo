#pragma once

#include "../ratgdo.h"
#include "../ratgdo_child.h"
#include "../ratgdo_state.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/component.h"

namespace esphome {
namespace ratgdo {

    enum SwitchType {
        RATGDO_LOCK
    };

    class RATGDOSwitch : public switch ::Switch, public RATGDOClient, public Component {
    public:
        void dump_config() override;
        void set_switch_type(SwitchType switch_type_) { this->switch_type_ = switch_type_; }

        void on_lock_state(LockState state) override;
        void turn_off() override;
        void turn_on() override;

    protected:
        SwitchType switch_type_;
    };

} // namespace ratgdo
} // namespace esphome
