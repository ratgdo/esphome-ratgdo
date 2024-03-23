#pragma once

#include "SoftwareSerial.h" // Using espsoftwareserial https://github.com/plerup/espsoftwareserial
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

            Result call(Args args);

            const Traits& traits() const { return this->traits_; }

        protected:
            Traits traits_;

            InternalGPIOPin* tx_pin_;
            InternalGPIOPin* rx_pin_;

            RATGDOComponent* ratgdo_;
            Scheduler* scheduler_;

            bool open_limit_reached_;
            bool last_open_limit_;
            bool close_limit_reached_;
            bool last_close_limit_;
        };

    } // namespace secplus1
} // namespace ratgdo
} // namespace esphome
