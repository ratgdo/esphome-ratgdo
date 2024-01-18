#pragma once

#include "common.h"
#include "ratgdo_state.h"

namespace esphome {

class Scheduler;
class InternalGPIOPin;

namespace ratgdo {

    class RATGDOComponent;

    const uint32_t HAS_DOOR_OPEN = 1 << 0; // has idempotent open door command
    const uint32_t HAS_DOOR_CLOSE = 1 << 1; // has idempotent close door command
    const uint32_t HAS_DOOR_STOP = 1 << 2; // has idempotent stop door command
    const uint32_t HAS_DOOR_STATUS = 1 << 3;

    const uint32_t HAS_LIGHT_TOGGLE = 1 << 10; // some protocols might not support this

    const uint32_t HAS_LOCK_TOGGLE = 1 << 20;

    class ProtocolTraits {
        uint32_t value;

    public:
        ProtocolTraits()
            : value(0)
        {
        }

        bool has_door_open() const { return this->value & HAS_DOOR_OPEN; }
        bool has_door_close() const { return this->value & HAS_DOOR_CLOSE; }
        bool has_door_stop() const { return this->value & HAS_DOOR_STOP; }
        bool has_door_status() const { return this->value & HAS_DOOR_STATUS; }

        bool has_light_toggle() const { return this->value & HAS_LIGHT_TOGGLE; }

        bool has_lock_toggle() const { return this->value & HAS_LOCK_TOGGLE; }

        void set_features(uint32_t feature) { this->value |= feature; }
        void clear_features(uint32_t feature) { this->value &= ~feature; }

        static uint32_t all()
        {
            return HAS_DOOR_CLOSE | HAS_DOOR_OPEN | HAS_DOOR_STOP | HAS_DOOR_STATUS | HAS_LIGHT_TOGGLE | HAS_LOCK_TOGGLE;
        }
    };

    class Protocol {
    public:
        virtual void setup(RATGDOComponent* ratgdo, Scheduler* scheduler, InternalGPIOPin* rx_pin, InternalGPIOPin* tx_pin);
        virtual void loop();
        virtual void dump_config();

        virtual void sync();

        virtual const ProtocolTraits& traits() const;

        virtual void light_action(LightAction action);
        virtual void lock_action(LockAction action);
        virtual void door_action(DoorAction action);

        virtual protocol::Result call(protocol::Args args);
    };

} // namespace ratgdo
} // namespace esphome
