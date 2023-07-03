/************************************
 * Rage
 * Against
 * The
 * Garage
 * Door
 * Opener
 *
 * Copyright (C) 2022  Paul Wieland
 *
 * GNU GENERAL PUBLIC LICENSE
 ************************************/

#pragma once
#include "SoftwareSerial.h" // Using espsoftwareserial https://github.com/plerup/espsoftwareserial
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/log.h"
#include "esphome/core/preferences.h"
#include "observable.h"

extern "C" {
#include "secplus.h"
}

#include "ratgdo_state.h"

namespace esphome {
namespace ratgdo {

    class RATGDOComponent;
    typedef Parented<RATGDOComponent> RATGDOClient;

    static const uint8_t PACKET_LENGTH = 19;
    typedef uint8_t WirePacket[PACKET_LENGTH];

    const float DOOR_POSITION_UNKNOWN = -1.0;

    /*
    from: https://github.com/argilo/secplus/blob/f98c3220356c27717a25102c0b35815ebbd26ccc/secplus.py#L540
    _WIRELINE_COMMANDS = {
        # sent by opener
        0x081: "status",
        0x084: "unknown_1",
        0x085: "unknown_2",
        0x0a1: "pair_3_resp",
        0x284: "motor_on",
        0x393: "learn_3_resp",
        0x401: "pair_2_resp",
        0x48c: "openings",

        # sent by switch
        0x080: "get_status",
        0x0a0: "pair_3",
        0x181: "learn_2",
        0x18c: "lock",
        0x280: "open",
        0x281: "light",
        0x285: "motion",
        0x391: "learn_1",
        0x392: "learn_3",
        0x400: "pair_2",
        0x48b: "get_openings",
    }
    */

    namespace data {
        const uint32_t LIGHT_OFF = 0;
        const uint32_t LIGHT_ON = 1;
        const uint32_t LIGHT_TOGGLE = 2;
        const uint32_t LIGHT_TOGGLE2 = 3;

        const uint32_t LOCK_OFF = 0;
        const uint32_t LOCK_ON = 1;
        const uint32_t LOCK_TOGGLE = 2;

        const uint32_t DOOR_CLOSE = 0;
        const uint32_t DOOR_OPEN = 1;
        const uint32_t DOOR_TOGGLE = 2;
        const uint32_t DOOR_STOP = 3;
    }

    namespace command {

        enum cmd : uint64_t {
            GET_STATUS = 0x080,
            STATUS = 0x081,
            OBST_1 = 0x084, // sent when an obstruction happens?
            OBST_2 = 0x085, // sent when an obstruction happens?
            PAIR_3 = 0x0a0,
            PAIR_3_RESP = 0x0a1,

            LEARN_2 = 0x181,
            LOCK = 0x18c,

            OPEN = 0x280,
            LIGHT = 0x281,
            MOTOR_ON = 0x284,
            MOTION = 0x285,

            LEARN_1 = 0x391,
            LEARN_3 = 0x392,
            LEARN_3_RESP = 0x393,

            PAIR_2 = 0x400,
            PAIR_2_RESP = 0x401,
            TTC = 0x40a, // Time to close
            GET_OPENINGS = 0x48b,
            OPENINGS = 0x48c,

        };
    }

    struct RATGDOStore {
        ISRInternalGPIOPin input_obst;

        int obstruction_low_count = 0; // count obstruction low pulses
        long last_obstruction_high = 0; // count time between high pulses from the obst ISR

        static void IRAM_ATTR HOT isr_obstruction(RATGDOStore* arg);
    };

    class RATGDOComponent : public Component {
    public:
        void setup() override;
        void loop() override;
        void dump_config() override;

        EspSoftwareSerial::UART sw_serial;

        observable<uint32_t> rolling_code_counter { 0 };

        float start_opening { -1 };
        observable<float> opening_duration { 0 };
        float start_closing { -1 };
        observable<float> closing_duration { 0 };

        observable<uint16_t> openings { 0 }; // number of times the door has been opened

        observable<DoorState> door_state { DoorState::DOOR_STATE_UNKNOWN };
        observable<float> door_position { DOOR_POSITION_UNKNOWN };
        bool moving_to_position { false };

        observable<LightState> light_state { LightState::LIGHT_STATE_UNKNOWN };
        observable<LockState> lock_state { LockState::LOCK_STATE_UNKNOWN };
        observable<ObstructionState> obstruction_state { ObstructionState::OBSTRUCTION_STATE_UNKNOWN };
        observable<MotorState> motor_state { MotorState::MOTOR_STATE_UNKNOWN };
        observable<ButtonState> button_state { ButtonState::BUTTON_STATE_UNKNOWN };
        observable<MotionState> motion_state { MotionState::MOTION_STATE_UNKNOWN };

        void set_output_gdo_pin(InternalGPIOPin* pin) { this->output_gdo_pin_ = pin; }
        void set_input_gdo_pin(InternalGPIOPin* pin) { this->input_gdo_pin_ = pin; }
        void set_input_obst_pin(InternalGPIOPin* pin) { this->input_obst_pin_ = pin; }
        void set_remote_id(uint64_t remote_id) { this->remote_id_ = remote_id & 0xffffff; } // not sure how large remote_id can be, assuming not more than 24 bits

        void gdo_state_loop();
        uint16_t decode_packet(const WirePacket& packet);
        void obstruction_loop();
        void transmit(command::cmd command, uint32_t data = 0, bool increment = true);
        void encode_packet(command::cmd command, uint32_t data, bool increment, WirePacket& packet);
        void print_packet(const WirePacket& packet) const;

        void increment_rolling_code_counter(int delta = 1);
        void set_rolling_code_counter(uint32_t code);
        void save_rolling_code_counter();

        // door
        void door_command(uint32_t data);
        void toggle_door();
        void open_door();
        void close_door();
        void stop_door();
        void door_move_to_position(float position);
        void position_sync_while_opening(float delta, float update_period = 500);
        void position_sync_while_closing(float delta, float update_period = 500);
        void cancel_position_sync_callbacks();
        void set_opening_duration(float duration);
        void set_closing_duration(float duration);

        // light
        void toggle_light();
        void light_on();
        void light_off();
        LightState get_light_state() const;

        // lock
        void toggle_lock();
        void lock();
        void unlock();

        // button functionality
        void query_status();
        void query_openings();
        void sync();

        // children subscriptions
        void subscribe_rolling_code_counter(std::function<void(uint32_t)>&& f);
        void subscribe_opening_duration(std::function<void(float)>&& f);
        void subscribe_closing_duration(std::function<void(float)>&& f);
        void subscribe_openings(std::function<void(uint16_t)>&& f);
        void subscribe_door_state(std::function<void(DoorState, float)>&& f);
        void subscribe_light_state(std::function<void(LightState)>&& f);
        void subscribe_lock_state(std::function<void(LockState)>&& f);
        void subscribe_obstruction_state(std::function<void(ObstructionState)>&& f);
        void subscribe_motor_state(std::function<void(MotorState)>&& f);
        void subscribe_button_state(std::function<void(ButtonState)>&& f);
        void subscribe_motion_state(std::function<void(MotionState)>&& f);

    protected:
        ESPPreferenceObject rolling_code_counter_pref_;
        ESPPreferenceObject opening_duration_pref_;
        ESPPreferenceObject closing_duration_pref_;
        RATGDOStore isr_store_ {};

        InternalGPIOPin* output_gdo_pin_;
        InternalGPIOPin* input_gdo_pin_;
        InternalGPIOPin* input_obst_pin_;
        uint64_t remote_id_;

    }; // RATGDOComponent

} // namespace ratgdo
} // namespace esphome