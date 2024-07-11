#pragma once

#include "common.h"
#include "ratgdo_state.h"

namespace esphome {

class Scheduler;
class InternalGPIOPin;

namespace ratgdo {

    class RATGDOComponent;

    namespace protocol {

        const uint32_t HAS_DOOR_OPEN = 1 << 0; // has idempotent open door command
        const uint32_t HAS_DOOR_CLOSE = 1 << 1; // has idempotent close door command
        const uint32_t HAS_DOOR_STOP = 1 << 2; // has idempotent stop door command
        const uint32_t HAS_DOOR_STATUS = 1 << 3;

        const uint32_t HAS_LIGHT_TOGGLE = 1 << 10; // some protocols might not support this

        const uint32_t HAS_LOCK_TOGGLE = 1 << 20;

        class Traits {
            uint32_t value;

        public:
            Traits()
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

        struct SetRollingCodeCounter {
            uint32_t counter;
        };
        struct GetRollingCodeCounter {
        };
        struct SetClientID {
            uint64_t client_id;
        };
        struct SetEnableEmulationMode {
            bool enable_emulation_mode;
        };
        struct QueryStatus {
        };
        struct QueryOpenings {
        };
        struct ActivateLearn {
        };
        struct InactivateLearn {
        };
        struct QueryPairedDevices {
            PairedDevice kind;
        };
        struct QueryPairedDevicesAll {
        };
        struct ClearPairedDevices {
            PairedDevice kind;
        };

        // a poor man's sum-type, because C++
        SUM_TYPE(Args,
            (SetRollingCodeCounter, set_rolling_code_counter),
            (GetRollingCodeCounter, get_rolling_code_counter),
            (SetClientID, set_client_id),
            (SetEnableEmulationMode, set_enable_emulation_mode),
            (QueryStatus, query_status),
            (QueryOpenings, query_openings),
            (ActivateLearn, activate_learn),
            (InactivateLearn, inactivate_learn),
            (QueryPairedDevices, query_paired_devices),
            (QueryPairedDevicesAll, query_paired_devices_all),
            (ClearPairedDevices, clear_paired_devices), )

        struct RollingCodeCounter {
            observable<uint32_t>* value;
        };

        SUM_TYPE(Result,
            (RollingCodeCounter, rolling_code_counter), )

        class Protocol {
        public:
            virtual void setup(RATGDOComponent* ratgdo, Scheduler* scheduler, InternalGPIOPin* rx_pin, InternalGPIOPin* tx_pin);
            virtual void loop();
            virtual void dump_config();

            virtual void sync();

            // dry contact methods
            virtual void set_open_limit(bool);
            virtual void set_close_limit(bool);
            virtual void set_discrete_open_pin(InternalGPIOPin* pin);
            virtual void set_discrete_close_pin(InternalGPIOPin* pin);

            virtual const Traits& traits() const;

            virtual void light_action(LightAction action);
            virtual void lock_action(LockAction action);
            virtual void door_action(DoorAction action);

            virtual protocol::Result call(protocol::Args args);
        };

    }
} // namespace ratgdo
} // namespace esphome
