#pragma once

#include "SoftwareSerial.h" // Using espsoftwareserial https://github.com/plerup/espsoftwareserial
#include "esphome/core/optional.h"

#include "ratgdo_state.h"
#include "protocol.h"
#include "callbacks.h"
#include "observable.h"

namespace esphome {

    class Scheduler;
    class InternalGPIOPin;

namespace ratgdo {
namespace secplus1 {

    using namespace esphome::ratgdo::protocol;

    static const uint8_t RX_LENGTH = 2;
    typedef uint8_t RxPacket[RX_LENGTH];

    static const uint8_t TX_LENGTH = 2;
    typedef uint8_t TxPacket[TX_LENGTH];

    static const TxPacket toggle_door = {0x30, 0x31};
    static const TxPacket toggle_light = {0x32, 0x33};
    static const TxPacket toggle_lock = {0x34, 0x35};

    static const uint8_t secplus1_states[] = {0x35,0x35,0x35,0x35,0x33,0x33,0x53,0x53,0x38,0x3A,0x3A,0x3A,0x39,0x38,0x3A, 0x38,0x3A,0x39,0x3A};

    ENUM(CommandType, uint16_t,
        (TOGGLE_DOOR_REQ, 0x30),
        (TOGGLE_DOOR_COMMIT, 0x31),
        (TOGGLE_LIGHT_REQ, 0x32),
        (TOGGLE_LIGHT_COMMIT, 0x33),
        (TOGGLE_LOCK_REQ, 0x34),
        (TOGGLE_LOCK_COMMIT, 0x35),
        (DOOR_STATUS_37, 0x37),
        (DOOR_STATUS, 0x38),
        (OBSTRUCTION, 0x39),
        (OTHER_STATUS, 0x3A),
        (UNKNOWN, 0xFF),
    )

    struct Command {
        CommandType type;
        uint8_t value;

        Command(): type(CommandType::UNKNOWN) {}
        Command(CommandType type_, uint8_t value_ = 0) : type(type_), value(value_) {}
    };

    enum class WallPanelEmulationState {
        WAITING,
        RUNNING,
    };

    class Secplus1 : public Protocol {
    public:
        void setup(RATGDOComponent* ratgdo, Scheduler* scheduler, InternalGPIOPin* rx_pin, InternalGPIOPin* tx_pin);
        void loop();
        void dump_config();

        void sync();

        void light_action(LightAction action);
        void lock_action(LockAction action);
        void door_action(DoorAction action);

        Result call(Args args);

    protected:
        void wall_panel_emulation(size_t index = 0);

        optional<Command> read_command();
        void handle_command(const Command& cmd);

        void print_rx_packet(const RxPacket& packet) const;
        void print_tx_packet(const TxPacket& packet) const;
        optional<Command> decode_packet(const RxPacket& packet) const;

        void transmit_packet(const TxPacket& packet, bool first_byte = false);

        LightState light_state { LightState::UNKNOWN };
        LockState lock_state { LockState::UNKNOWN };
        DoorState door_state { DoorState::UNKNOWN };
        DoorState prev_door_state { DoorState::UNKNOWN };

        bool wall_panel_starting_ { false };
        uint32_t wall_panel_emulation_start_ { 0 };
        WallPanelEmulationState wall_panel_emulation_state_ { WallPanelEmulationState::WAITING };

        bool is_0x37_panel_ { false };

        // bool transmit_pending_ { false };
        // uint32_t transmit_pending_start_ { 0 };
        TxPacket tx_packet_;

        uint32_t last_rx_ { 0 };

        SoftwareSerial sw_serial_;

        InternalGPIOPin* tx_pin_;
        InternalGPIOPin* rx_pin_;

        RATGDOComponent* ratgdo_;
        Scheduler* scheduler_;
    };

} // namespace secplus1
} // namespace ratgdo
} // namespace esphome
