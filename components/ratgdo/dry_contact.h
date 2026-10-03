#pragma once

#ifdef PROTOCOL_DRYCONTACT

#include "esphome/core/defines.h"

#include "esphome/core/gpio.h"
#include "esphome/core/optional.h"

#include "callbacks.h"
#include "observable.h"
#include "protocol.h"
#include "ratgdo_state.h"

namespace esphome {

class Scheduler;
class InternalGPIOPin;

} // namespace esphome

namespace esphome::ratgdo {
namespace dry_contact {

    using namespace esphome::ratgdo::protocol;
    using namespace esphome::gpio;

    class DryContact : public Protocol {
    public:
        void setup(RATGDOComponent* ratgdo, Scheduler* scheduler,
            InternalGPIOPin* rx_pin, InternalGPIOPin* tx_pin);
        void loop();
        void dump_config();

        void sync();

        void light_action(LightAction action);
        void lock_action(LockAction action);
        void door_action(DoorAction action);
        void set_open_limit(bool state);
        void set_close_limit(bool state);
        void send_door_state();

        void set_discrete_open_pin(InternalGPIOPin* pin)
        {
            this->discrete_open_pin_ = pin;
            this->discrete_open_pin_->setup();
            this->discrete_open_pin_->pin_mode(gpio::FLAG_OUTPUT);
        }

        void set_discrete_close_pin(InternalGPIOPin* pin)
        {
            this->discrete_close_pin_ = pin;
            this->discrete_close_pin_->setup();
            this->discrete_close_pin_->pin_mode(gpio::FLAG_OUTPUT);
        }

        Result call(Args args);

        void set_toggle_behavior(DryContactBehavior while_opening, DryContactBehavior while_closing,
            DryContactBehavior while_stopped);
        void set_obstruction_behavior(DryContactBehavior while_opening, DryContactBehavior while_closing);

        const Traits& traits() const { return this->traits_; }

    protected:
        // Pointers first (4-byte aligned)
        InternalGPIOPin* tx_pin_;
        InternalGPIOPin* rx_pin_;
        InternalGPIOPin* discrete_open_pin_ { nullptr };
        InternalGPIOPin* discrete_close_pin_ { nullptr };
        RATGDOComponent* ratgdo_;
        Scheduler* scheduler_;

        // Traits (likely aligned structure)
        Traits traits_;

        // Toggle sequence, see TOGGLE SEQUENCE in dry_contact.cpp
        static constexpr uint32_t MIN_TOGGLE_INTERVAL_MS = 600; // press is held 500 ms
        static constexpr uint8_t MAX_TOGGLES = 4; // worst case needs 3
        uint32_t next_toggle_ms_ { 0 };
        DryContactBehavior toggle_while_opening_ { DryContactBehavior::UNSET };
        DryContactBehavior toggle_while_closing_ { DryContactBehavior::UNSET };
        DryContactBehavior toggle_while_stopped_ { DryContactBehavior::UNSET };
        DryContactBehavior obstruction_while_opening_ { DryContactBehavior::UNSET };
        DryContactBehavior obstruction_while_closing_ { DryContactBehavior::UNSET };
        DoorAction request_ { DoorAction::UNKNOWN }; // UNKNOWN when idle
        DoorState expected_state_ { DoorState::UNKNOWN };
        DoorState last_direction_ { DoorState::UNKNOWN };
        uint8_t toggle_count_ { 0 };
        bool toggle_pending_ { false };
        bool delayed_press_ { false };

        void report(DoorState state);
        bool toggle_configured() const;
        bool sequence_action(DoorAction action);
        void press(DoorAction action);
        bool request_reached(DoorAction action, DoorState state) const;
        DoorState state_after(DryContactBehavior behavior, DoorState moving) const;
        DoorState state_after_toggle(DoorState state) const;
        void step();
        void send_toggle(DoorState expected);
        void cancel_request();
        void cancel_step();
        void on_resolved(DoorState state);
        void obstructed();

        // Small members grouped at the end
        DoorState door_state_;
        struct {
            uint8_t open_limit_reached : 1;
            uint8_t last_open_limit : 1;
            uint8_t close_limit_reached : 1;
            uint8_t last_close_limit : 1;
            uint8_t reserved : 4; // Reserved for future use
        } limits_;
    };

} // namespace dry_contact
} // namespace esphome::ratgdo

#endif // PROTOCOL_DRYCONTACT
