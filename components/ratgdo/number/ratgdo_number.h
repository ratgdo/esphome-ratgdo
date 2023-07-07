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
        // other esphome components that persist state in the flash have HARDWARE priority
        // ensure we get initialized before them, so that the state doesn't get invalidated
        // by components that might be added in the future
        float get_setup_priority() const override { return setup_priority::HARDWARE + 1; }

        void update_state(float value);
        void control(float value) override;

    protected:
        NumberType number_type_;
        ESPPreferenceObject pref_;
    };

} // namespace ratgdo
} // namespace esphome
