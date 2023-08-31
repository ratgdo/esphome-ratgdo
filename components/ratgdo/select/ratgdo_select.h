#pragma once

#include "../ratgdo.h"
#include "../ratgdo_state.h"
#include "esphome/components/select/select.h"
#include "esphome/core/component.h"

namespace esphome {
namespace ratgdo {

    enum RATGDOSelectType {
        RATGDO_TTC
    };

    class RATGDOSelect : public select::Select, public RATGDOClient, public Component {
    public:
        void dump_config() override;
        void setup() override;
        void set_ratgdo_select_type(RATGDOSelectType ratgdo_select_type_) { this->ratgdo_select_type_ = ratgdo_select_type_; }

    protected:
        RATGDOSelectType ratgdo_select_type_;
        void control(const std::string &value);
    };

} // namespace ratgdo
} // namespace esphome
