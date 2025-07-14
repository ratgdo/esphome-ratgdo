#pragma once

#include "esphome/core/defines.h"

#include "SoftwareSerial.h" // Using espsoftwareserial https://github.com/plerup/espsoftwareserial
#include "esphome/core/gpio.h"
#include "esphome/core/optional.h"

#include "callbacks.h"
#include "observable.h"
#include "protocol.h"
#include "ratgdo_state.h"

namespace esphome {

class Scheduler;
class InternalGPIOPin;

namespace ratgdo {
    namespace dry_contact {

        using namespace esphome::ratgdo::protocol;
        using namespace esphome::gpio;

        class DryContact : public Protocol {
        public:
            void setup(RATGDOComponent* ratgdo, Scheduler* scheduler, InternalGPIOPin* rx_pin, InternalGPIOPin* tx_pin);
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

            const Traits& traits() const { return this->traits_; }

        protected:
            // Pointers first (4-byte aligned)
            InternalGPIOPin* tx_pin_;
            InternalGPIOPin* rx_pin_;
            InternalGPIOPin* discrete_open_pin_;
            InternalGPIOPin* discrete_close_pin_;
            RATGDOComponent* ratgdo_;
            Scheduler* scheduler_;

            // Traits (likely aligned structure)
            Traits traits_;

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
} // namespace ratgdo
} // namespace esphome
