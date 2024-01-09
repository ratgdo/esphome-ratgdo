#pragma once

#include "ratgdo_state.h"
#include "common.h"

namespace esphome {

class Scheduler;
class InternalGPIOPin;

namespace ratgdo {

    class RATGDOComponent;

    class Protocol {
    public:
        virtual void setup(RATGDOComponent* ratgdo, Scheduler* scheduler, InternalGPIOPin* rx_pin, InternalGPIOPin* tx_pin);
        virtual void loop();
        virtual void dump_config();

        virtual void sync();

        virtual void light_action(LightAction action);
        virtual void lock_action(LockAction action);
        virtual void door_action(DoorAction action);

        virtual protocol::Result call(protocol::Args args);
    };
    
} // namespace ratgdo
} // namespace esphome
