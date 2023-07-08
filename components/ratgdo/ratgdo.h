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
#include "enum.h"
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

    ENUM(Command, uint16_t,
        (UNKNOWN, 0x000),
        (GET_STATUS, 0x080),
        (STATUS, 0x081),
        (OBST_1, 0x084), // sent when an obstruction happens?
        (OBST_2, 0x085), // sent when an obstruction happens?
        (PAIR_3, 0x0a0),
        (PAIR_3_RESP, 0x0a1),

        (LEARN_2, 0x181),
        (LOCK, 0x18c),
        (OPEN, 0x280),
        (LIGHT, 0x281),
        (MOTOR_ON, 0x284),
        (MOTION, 0x285),

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

    inline bool operator==(const uint16_t cmd_i, const Command& cmd_e) { return cmd_i == static_cast<uint16_t>(cmd_e); }
    inline bool operator==(const Command& cmd_e, const uint16_t cmd_i) { return cmd_i == static_cast<uint16_t>(cmd_e); }

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

        observable<uint32_t> rolling_code_counter { 0 };

        float start_opening { -1 };
        observable<float> opening_duration { 0 };
        float start_closing { -1 };
        observable<float> closing_duration { 0 };

        observable<uint16_t> openings { 0 }; // number of times the door has been opened

        observable<DoorState> door_state { DoorState::UNKNOWN };
        observable<float> door_position { DOOR_POSITION_UNKNOWN };
        bool moving_to_position { false };

        observable<LightState> light_state { LightState::UNKNOWN };
        observable<LockState> lock_state { LockState::UNKNOWN };
        observable<ObstructionState> obstruction_state { ObstructionState::UNKNOWN };
        observable<MotorState> motor_state { MotorState::UNKNOWN };
        observable<ButtonState> button_state { ButtonState::UNKNOWN };
        observable<MotionState> motion_state { MotionState::UNKNOWN };

        observable<bool> sync_failed { false };

        void set_output_gdo_pin(InternalGPIOPin* pin) { this->output_gdo_pin_ = pin; }
        void set_input_gdo_pin(InternalGPIOPin* pin) { this->input_gdo_pin_ = pin; }
        void set_input_obst_pin(InternalGPIOPin* pin) { this->input_obst_pin_ = pin; }
        void set_remote_id(uint64_t remote_id) { this->remote_id_ = remote_id & 0xffffff; } // not sure how large remote_id can be, assuming not more than 24 bits
        uint64_t get_remote_id() { return this->remote_id_; }

        void gdo_state_loop();
        uint16_t decode_packet(const WirePacket& packet);
        void obstruction_loop();
        void send_command(Command command, uint32_t data = 0, bool increment = true);
        bool transmit_packet();
        void encode_packet(Command command, uint32_t data, bool increment, WirePacket& packet);
        void print_packet(const WirePacket& packet) const;

        void increment_rolling_code_counter(int delta = 1);
        void set_rolling_code_counter(uint32_t code);

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
        void set_door_position(float door_position) { this->door_position = door_position; }
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
        void subscribe_sync_failed(std::function<void(bool)>&& f);

    protected:
        // tx data
        bool transmit_pending_ {false};
        WirePacket tx_packet_;

        RATGDOStore isr_store_ {};
        SoftwareSerial sw_serial_;

        InternalGPIOPin* output_gdo_pin_;
        InternalGPIOPin* input_gdo_pin_;
        InternalGPIOPin* input_obst_pin_;
        uint64_t remote_id_;

    }; // RATGDOComponent

} // namespace ratgdo
} // namespace esphome