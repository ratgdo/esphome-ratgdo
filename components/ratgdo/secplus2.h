#pragma once

#ifdef PROTOCOL_SECPLUSV2

#include "SoftwareSerial.h" // Using espsoftwareserial https://github.com/plerup/espsoftwareserial
#include "esphome/core/optional.h"

#include "callbacks.h"
#include "common.h"
#include "observable.h"
#include "protocol.h"
#include "ratgdo_state.h"

namespace esphome {

class Scheduler;
class InternalGPIOPin;

namespace ratgdo {
    class RATGDOComponent;

    namespace secplus2 {

        using namespace esphome::ratgdo::protocol;

        static const uint8_t PACKET_LENGTH = 19;
        typedef uint8_t WirePacket[PACKET_LENGTH];

        ENUM_SPARSE(CommandType, uint16_t,
            (UNKNOWN, 0x000),
            (GET_STATUS, 0x080),
            (STATUS, 0x081),
            (OBST_1, 0x084), // sent when an obstruction happens?
            (OBST_2, 0x085), // sent when an obstruction happens?
            (BATTERY_STATUS, 0x09d),
            (PAIR_3, 0x0a0),
            (PAIR_3_RESP, 0x0a1),

            (LEARN, 0x181),
            (LOCK, 0x18c),
            (DOOR_ACTION, 0x280),
            (LIGHT, 0x281),
            (MOTOR_ON, 0x284),
            (MOTION, 0x285),

            (GET_PAIRED_DEVICES, 0x307), // nibble 0 for total, 1 wireless, 2 keypads, 3 wall, 4 accessories.
            (PAIRED_DEVICES, 0x308), // byte2 holds number of paired devices
            (CLEAR_PAIRED_DEVICES, 0x30D), // nibble 0 to clear remotes, 1 keypads, 2 wall, 3 accessories (offset from above)

            (LEARN_1, 0x391),
            (PING, 0x392),
            (PING_RESP, 0x393),

            (PAIR_2, 0x400),
            (PAIR_2_RESP, 0x401),
            (SET_TTC, 0x402), // ttc_in_seconds = (byte1<<8)+byte2
            (CANCEL_TTC, 0x408), // ?
            (TTC, 0x40a), // Time to close
            (GET_OPENINGS, 0x48b),
            (OPENINGS, 0x48c), // openings = (byte1<<8)+byte2
        )

        inline bool operator==(const uint16_t cmd_i, const CommandType& cmd_e) { return cmd_i == static_cast<uint16_t>(cmd_e); }
        inline bool operator==(const CommandType& cmd_e, const uint16_t cmd_i) { return cmd_i == static_cast<uint16_t>(cmd_e); }

        enum class IncrementRollingCode {
            NO,
            YES,
        };

        struct Command {
            CommandType type;
            uint8_t nibble;
            uint8_t byte1;
            uint8_t byte2;

            Command()
                : type(CommandType::UNKNOWN)
            {
            }
            Command(CommandType type_, uint8_t nibble_ = 0, uint8_t byte1_ = 0, uint8_t byte2_ = 0)
                : type(type_)
                , nibble(nibble_)
                , byte1(byte1_)
                , byte2(byte2_)
            {
            }
        };

        class Secplus2 : public Protocol {
        public:
            void setup(RATGDOComponent* ratgdo, Scheduler* scheduler, InternalGPIOPin* rx_pin, InternalGPIOPin* tx_pin);
            void loop();
            void dump_config();

            void sync();

            void light_action(LightAction action);
            void lock_action(LockAction action);
            void door_action(DoorAction action);

            Result call(Args args);

            const Traits& traits() const { return this->traits_; }

            // methods not used by secplus2
            void set_open_limit(bool state) { }
            void set_close_limit(bool state) { }
            void set_discrete_open_pin(InternalGPIOPin* pin) { }
            void set_discrete_close_pin(InternalGPIOPin* pin) { }

        protected:
            void increment_rolling_code_counter(int delta = 1);
            void set_rolling_code_counter(uint32_t counter);
            void set_client_id(uint64_t client_id);

            optional<Command> read_command();
            void handle_command(const Command& cmd);

            void send_command(Command cmd, IncrementRollingCode increment = IncrementRollingCode::YES);
            void send_command(Command cmd, IncrementRollingCode increment, std::function<void()>&& on_sent);
            void encode_packet(Command cmd, WirePacket& packet);
            bool transmit_packet();

            void door_command(DoorAction action);

            void query_status();
            void query_openings();
            void query_paired_devices();
            void query_paired_devices(PairedDevice kind);
            void clear_paired_devices(PairedDevice kind);
            void activate_learn();
            void inactivate_learn();

            void print_packet(const esphome::LogString* prefix, const WirePacket& packet) const;
            optional<Command> decode_packet(const WirePacket& packet) const;

            void sync_helper(uint32_t start, uint32_t delay, uint8_t tries);

            // 8-byte member first (may require 8-byte alignment on some 32-bit systems)
            uint64_t client_id_ { 0x539 };

            // Pointers (4-byte aligned)
            InternalGPIOPin* tx_pin_;
            InternalGPIOPin* rx_pin_;
            RATGDOComponent* ratgdo_;
            Scheduler* scheduler_;

            // 4-byte members
            uint32_t transmit_pending_start_ { 0 };

            // Larger structures
            single_observable<uint32_t> rolling_code_counter_ { 0 };
            OnceCallbacks<void()> on_command_sent_;
            Traits traits_;
            SoftwareSerial sw_serial_;

            // 19-byte array
            WirePacket tx_packet_;

            // Small members at the end
            LearnState learn_state_ { LearnState::UNKNOWN };
            struct {
                uint8_t transmit_pending : 1;
                uint8_t reserved : 7; // Reserved for future use
            } flags_ { 0 };
        };
    } // namespace secplus2
} // namespace ratgdo
} // namespace esphome

#endif // PROTOCOL_SECPLUSV2
