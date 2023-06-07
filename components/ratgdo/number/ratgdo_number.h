#pragma once

#include "../ratgdo.h"
#include "../ratgdo_child.h"
#include "../ratgdo_state.h"
#include "esphome/components/number/number.h"
#include "esphome/core/component.h"

namespace esphome {
namespace ratgdo {

    enum NumberType {
        RATGDO_ROLLING_CODE_COUNTER
    };

    class RATGDONumber : public number::Number, public RATGDOClient, public Component {
    public:
        void dump_config() override;
        void set_number_type(NumberType number_type_) { this->number_type_ = number_type_; }

        void on_rolling_code_change(uint32_t rollingCodeCounter) override;
        void RATGDONumber::control(float value) override;

    protected:
        NumberType number_type_;
    };

} // namespace ratgdo
} // namespace esphome
