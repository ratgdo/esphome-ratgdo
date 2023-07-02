#pragma once

#include "../ratgdo.h"
#include "../ratgdo_state.h"
#include "esphome/components/number/number.h"
#include "esphome/core/component.h"

namespace esphome {
namespace ratgdo {

    enum NumberType {
        RATGDO_ROLLING_CODE_COUNTER,
        RATGDO_OPENING_DURATION,
        RATGDO_CLOSING_DURATION,
    };

    class RATGDONumber : public number::Number, public RATGDOClient, public Component {
    public:
        void dump_config() override;
        void setup() override;
        void set_number_type(NumberType number_type);

        void update_state(float value);
        void control(float value) override;

    protected:
        NumberType number_type_;
        ESPPreferenceObject pref_;
    };

} // namespace ratgdo
} // namespace esphome
