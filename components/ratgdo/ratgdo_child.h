#pragma once

#include "esphome/core/helpers.h"

namespace esphome {
namespace ratgdo {

    // Forward declare RATGDOComponent
    class RATGDOComponent;

    class RATGDOClient : public Parented<RATGDOComponent> {
    public:
        virtual void on_status() = 0;
        virtual void on_ratgdo_state(bool is_ready) = 0;

    protected:
        friend RATGDOComponent;
        virtual std::string describe() = 0;
    };

} // namespace ratgdo
} // namespace esphome
