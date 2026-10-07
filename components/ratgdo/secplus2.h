#pragma once

#ifdef PROTOCOL_SECPLUSV2

#include "esphome/core/optional.h"
#include "ratgdo_uart.h"

#include "callbacks.h"
#include "common.h"
#include "observable.h"
#include "protocol.h"
#include "ratgdo_state.h"

namespace esphome {

class Scheduler;
class InternalGPIOPin;

} // namespace esphome

namespace esphome::ratgdo {
class RATGDOComponent;

namespace secplus2 {

    using namespace esphome::ratgdo::protocol;

    static const uint8_t PACKET_LENGTH = 19;
    static constexpr uint32_t STATUS_WATCHDOG_TIMEOUT = 360000; // 6 min. status updates are normally every ~5 min
    static constexpr uint32_t STATUS_WATCHDOG_POLL = 60000; // check once per minute
    typedef uint8_t WirePacket[PACKET_LENGTH];

    ENUM_SPARSE(CommandType, uint16_t,
        (UNKNOWN, 0x000),
        (GET_STATUS, 0x080),
        (STATUS, 0x081),
        (OBST_1, 0x084), // sent when an obstruction happens?
        (OBST_2, 0x085), // sent when an obstruction happens?
        (BATTERY_STATUS, 0x09d),

        (TTC_GET_STATE, 0x0a0),
        (TTC_STATE, 0x0a1),

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

        (TTC_GET_LIMIT, 0x400), // get current autoclose time limit
        (TTC_LIMIT, 0x401), // current TTC limit in seconds => (byte1<<8)+byte2; same as TTC_SET_LIMIT
        (TTC_SET_LIMIT, 0x402), // command to set TTC in seconds => (byte1<<8)+byte2
        (TTC_ACTION, 0x408), // byte1 = TtcActionCode (TOGGLE or DISABLE)
        (TTC_GET_COUNTDOWN, 0x409), // query current countdown; response is a normal TTC_COUNTDOWN broadcast - confirmed via live test
        (TTC_COUNTDOWN, 0x40a), // Periodic countdown broadcast message (sent every 60 seconds) while TTC counting down

        (GET_OPENINGS, 0x48b),
        (OPENINGS, 0x48c), // openings = (byte1<<8)+byte2
    )

    inline bool operator==(const uint16_t cmd_i, const CommandType& cmd_e) { return cmd_i == static_cast<uint16_t>(cmd_e); }
    inline bool operator==(const CommandType& cmd_e, const uint16_t cmd_i) { return cmd_i == static_cast<uint16_t>(cmd_e); }

    // Named values for TTC_ACTION message's byte1 - Found two that do something.
    // Other values seem to be silently ignored by the GDO.
    //
    // Values are prefixed with TTC_ (unlike other enums in this file) because
    // plain names can collide with #define macros in vendored ESP32 core
    // headers on boards using framework: arduino - enum class scoping
    // doesn't protect against the preprocessor, which substitutes any
    // matching macro name wherever it appears as a token, including inside
    // this ENUM_SPARSE() invocation.
    ENUM_SPARSE(TtcActionCode, uint8_t,
        (TTC_TOGGLE, 0x04), // pause/resume the countdown (HOLD <-> COUNTING/READY)
        (TTC_DISABLE, 0x05)) // disable TTC entirely

    inline bool operator==(const uint8_t val, const TtcActionCode& code) { return val == static_cast<uint8_t>(code); }
    inline bool operator==(const TtcActionCode& code, const uint8_t val) { return val == static_cast<uint8_t>(code); }

    // Named values for TTC_STATE message's byte1. (All are sent by GDO, except for 1.)
    // See TtcActionCode above for why these are TTC_-prefixed.
    ENUM_SPARSE(TtcStateCode, uint8_t,
        (TTC_UNKNOWN, 0),
        (TTC_WALL_CONTROL_ACK, 0x01), // sent by a wall control acknowledging a TTC_STATE broadcast
        (TTC_ENABLED_COUNTING, 0x02), // TTC is configured and counting down
        (TTC_DISABLED, 0x09), // TTC is disabled (limit is set to 0)
        (TTC_ENABLED_HOLDING, 0x0a), // TTC is configured, but on hold
        (TTC_CLOSING_ALERT, 0x0b), // Countdown ended, light-flash/beeper warning pre-close period
        (TTC_ENABLED_READY, 0x0c), // TTC is configured, but not running
        (TTC_INITIALIZING_ENABLED, 0x0d), // TTC starting up, will end up enabled. Transitions to ENABLED_* later.
        (TTC_INITIALIZING_DISABLED, 0x0e)) // Same as TTC_INITIALIZING_ENABLED, but transitions to TTC_DISABLED later.

    inline bool operator==(const uint8_t val, const TtcStateCode& code) { return val == static_cast<uint8_t>(code); }
    inline bool operator==(const TtcStateCode& code, const uint8_t val) { return val == static_cast<uint8_t>(code); }

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
        void on_shutdown() override;

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
        template <typename F>
        void send_command(Command cmd, IncrementRollingCode increment, F&& on_sent)
        {
            // Only register the callback if the command will be accepted.
            // If transmit_pending is set the command will be dropped, and
            // a stale callback would fire when the previous pending packet
            // transmits -- executing logic (e.g. the second phase of a
            // door_command) at the wrong time.
            //
            // Register before send_command() because transmit_packet() may
            // succeed immediately and call on_command_sent_.trigger() inline.
            if (this->flags_.transmit_pending) {
                return;
            }
            this->on_command_sent_(std::forward<F>(on_sent));
            this->send_command(cmd, increment);
        }
        void encode_packet(Command cmd, WirePacket& packet);
        bool transmit_packet();

        void door_command(DoorAction action);

        void query_status();
        void query_openings();
        void send_ttc_action(TtcActionCode action);
        void query_ttc_state();
        void query_ttc_limit();
        void set_ttc_limit(uint16_t seconds);
        void query_ttc_countdown();
        void query_paired_devices();
        void query_paired_devices(PairedDevice kind);
        void clear_paired_devices(PairedDevice kind);
        void activate_learn();
        void inactivate_learn();

        void print_packet(const esphome::LogString* prefix, const WirePacket& packet) const;
        optional<Command> decode_packet(const WirePacket& packet) const;

        void sync_helper(uint32_t start, uint32_t delay, uint8_t tries);
        void start_status_watchdog();

        // 8-byte member first (may require 8-byte alignment on some 32-bit systems)
        uint64_t client_id_ { 0x539 };

        // Pointers (4-byte aligned)
        InternalGPIOPin* tx_pin_;
        InternalGPIOPin* rx_pin_;
        RATGDOComponent* ratgdo_;
        Scheduler* scheduler_;

        // 4-byte members
        uint32_t transmit_pending_start_ { 0 };
        uint32_t rx_msg_start_ { 0 };
        uint32_t rx_last_read_ { 0 };
        uint32_t last_status_ms_ { 0 };

        // Larger structures
        single_observable<uint32_t> rolling_code_counter_ { 0 };
        OnceCallbacks<void()> on_command_sent_;
        Traits traits_;
        RatgdoUART uart_;

        // 19-byte arrays
        WirePacket tx_packet_;
        WirePacket rx_packet_;

        // Small members at the end
        uint16_t rx_byte_count_ { 0 };
        LearnState learn_state_ { LearnState::UNKNOWN };
        struct {
            uint8_t transmit_pending : 1;
            uint8_t rx_reading_msg : 1;
            uint8_t reserved : 6; // Reserved for future use
        } flags_ { 0 };
    };
} // namespace secplus2
} // namespace esphome::ratgdo

#endif // PROTOCOL_SECPLUSV2
