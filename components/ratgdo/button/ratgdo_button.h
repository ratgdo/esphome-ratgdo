#pragma once

#include "../ratgdo.h"
#include "../ratgdo_child.h"
#include "../ratgdo_state.h"
#include "esphome/components/button/button.h"
#include "esphome/core/component.h"

namespace esphome {
namespace ratgdo {

    enum ButtonType {
        RATGDO_SYNC,
        RATGDO_QUERY
    };

    class RATGDOButton : public button::Button, public RATGDOClient, public Component {
    public:
        void dump_config() override;
        void set_button_type(ButtonType button_type_) { this->button_type_ = button_type_; }

        void press_action() override;

    protected:
        ButtonType button_type_;
    };

} // namespace ratgdo
} // namespace esphome
