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

extern "C" {
#include "secplus.h"
}

#include "ratgdo_child.h"
#include "ratgdo_state.h"

namespace esphome {
namespace ratgdo {

    // Forward declare RATGDOClient
    class RATGDOClient;

    static const uint8_t CODE_LENGTH = 19;

/*
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
        enum CmdData: uint32_t {
            OFF = 0,
            CLOSE = 0,
            ON = 1,
            OPEN = 1,
            TOGGLE = 2,
        };
    }

    namespace command {

        enum cmd: uint64_t {
            GET_STATUS      = 0x080,
            STATUS          = 0x081,
            PAIR            = 0x0a0,
            LEARN           = 0x392,
            DOOR            = 0x280,
            LIGHT           = 0x281,
            MOTOR_ON        = 0x284,
            MOTION          = 0x285,
            LOCK            = 0x18c,
            GET_OPENINGS    = 0x48b,
            OPENINGS        = 0x48c,
        };
    }

    class RATGDOComponent : public Component {
    public:
        void setup() override;
        void loop() override;
        void dump_config() override;

        EspSoftwareSerial::UART swSerial;

        uint32_t rollingCodeCounter { 0 };
        uint32_t lastSyncedRollingCodeCounter { 0 };

        uint16_t previousOpenings { 0 }; // number of times the door has been opened
        uint16_t openings { 0 }; // number of times the door has been opened

        uint8_t txRollingCode[CODE_LENGTH];
        uint8_t rxRollingCode[CODE_LENGTH];

        uint8_t previousDoorState { DoorState::DOOR_STATE_UNKNOWN };
        uint8_t previousLightState { LightState::LIGHT_STATE_UNKNOWN };
        uint8_t previousLockState { LockState::LOCK_STATE_UNKNOWN };
        uint8_t previousObstructionState { ObstructionState::OBSTRUCTION_STATE_UNKNOWN };
        uint8_t previousMotorState { MotorState::MOTOR_STATE_UNKNOWN };
        uint8_t previousButtonState { ButtonState::BUTTON_STATE_UNKNOWN };
        uint8_t previousMotionState { MotionState::MOTION_STATE_UNKNOWN };

        uint8_t doorState { DoorState::DOOR_STATE_UNKNOWN };
        uint8_t lightState { LightState::LIGHT_STATE_UNKNOWN };
        uint8_t lockState { LockState::LOCK_STATE_UNKNOWN };
        uint8_t obstructionState { ObstructionState::OBSTRUCTION_STATE_UNKNOWN };
        uint8_t motorState { MotorState::MOTOR_STATE_UNKNOWN };
        uint8_t buttonState { ButtonState::BUTTON_STATE_UNKNOWN };
        uint8_t motionState { MotionState::MOTION_STATE_UNKNOWN };

        void set_output_gdo_pin(InternalGPIOPin* pin) { this->output_gdo_pin_ = pin; };
        void set_input_gdo_pin(InternalGPIOPin* pin) { this->input_gdo_pin_ = pin; };
        void set_remote_id(uint64_t remote_id) { this->remote_id = remote_id & 0xffffff; };

        /********************************** FUNCTION DECLARATION
         * *****************************************/
        void transmit(command::cmd command, uint32_t data = 0, bool increment = true);
        void sync();

        void gdoStateLoop();
        void obstructionLoop();
        void statusUpdateLoop();

        void saveCounter();
        
        void doorCommand(uint32_t data);
        void toggleDoor();
        void openDoor();
        void closeDoor();
        void stopDoor();
        void toggleLight();
        void lightOn();
        void lightOff();
        void toggleLock();
        void lock();
        void unlock();
        void query_status();
        void query_openings();

        void printRollingCode();
        void getRollingCode(command::cmd command, uint32_t data, bool increment);
        uint16_t readRollingCode();
        void incrementRollingCodeCounter();
        void sendRollingCodeChanged();
        void setRollingCodeCounter(uint32_t counter);
        LightState getLightState();
        /** Register a child component. */
        void register_child(RATGDOClient* obj);

    protected:
        ESPPreferenceObject pref_;
        std::vector<RATGDOClient*> children_;
        bool rollingCodeUpdatesEnabled_ { true };
        bool forceUpdate_ { false };

        InternalGPIOPin* output_gdo_pin_;
        InternalGPIOPin* input_gdo_pin_;
        uint64_t remote_id;

    }; // RATGDOComponent

} // namespace ratgdo
} // namespace esphome