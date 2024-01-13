#pragma once

#include <queue>

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
        (TOGGLE_DOOR_PRESS, 0x30),
        (TOGGLE_DOOR_RELEASE, 0x31),
        (TOGGLE_LIGHT_PRESS, 0x32),
        (TOGGLE_LIGHT_RELEASE, 0x33),
        (TOGGLE_LOCK_PRESS, 0x34),
        (TOGGLE_LOCK_RELEASE, 0x35),
        (DOOR_STATUS_0x37, 0x37),
        (DOOR_STATUS, 0x38),
        (OBSTRUCTION, 0x39),
        (OTHER_STATUS, 0x3A),
        (UNKNOWN, 0xFF),
    )

    struct RxCommand {
        CommandType req;
        uint8_t resp;

        RxCommand(): req(CommandType::UNKNOWN), resp(0) {}
        RxCommand(CommandType req_): req(req_), resp(0) {}
        RxCommand(CommandType req_, uint8_t resp_ = 0) : req(req_), resp(resp_) {}
    };

    struct TxCommand {
        CommandType request;
        uint32_t time;
    };

    struct FirstToSend {
        bool operator()(const TxCommand l, const TxCommand r) const { return l.time > r.time; }
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

        optional<RxCommand> read_command();
        void handle_command(const RxCommand& cmd);

        void print_rx_packet(const RxPacket& packet) const;
        void print_tx_packet(const TxPacket& packet) const;
        optional<RxCommand> decode_packet(const RxPacket& packet) const;

        void enqueue_transmit(CommandType cmd, uint32_t time = 0);
        optional<CommandType> pending_tx();
        bool do_transmit_if_pending();
        void enqueue_command_pair(CommandType cmd);
        void transmit_byte(uint32_t value, bool enable_rx = false);

        void toggle_light();
        void toggle_lock();
        void toggle_door();
        void query_status();

        LightState light_state { LightState::UNKNOWN };
        LockState lock_state { LockState::UNKNOWN };
        DoorState door_state { DoorState::UNKNOWN };

        bool wall_panel_starting_ { false };
        uint32_t wall_panel_emulation_start_ { 0 };
        WallPanelEmulationState wall_panel_emulation_state_ { WallPanelEmulationState::WAITING };

        bool is_0x37_panel_ { false };
        std::priority_queue<TxCommand, std::vector<TxCommand>, FirstToSend> pending_tx_;
        uint32_t last_rx_ { 0 };
        uint32_t last_status_query_ { 0 };

        SoftwareSerial sw_serial_;

        InternalGPIOPin* tx_pin_;
        InternalGPIOPin* rx_pin_;

        RATGDOComponent* ratgdo_;
        Scheduler* scheduler_;
    };

} // namespace secplus1
} // namespace ratgdo
} // namespace esphome
