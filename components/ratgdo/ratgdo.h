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

    struct cmd {
        uint64_t fixed;
        uint32_t data;
        inline bool operator!=(cmd const& other) const
        {
            return (fixed != other.fixed || data != other.data);
        }
    };

    typedef struct {
        cmd REBOOT1;
        cmd REBOOT2;
        cmd REBOOT3;
        cmd REBOOT4;
        cmd REBOOT5;
        cmd REBOOT6;
        cmd DOOR1;
        cmd DOOR2;
        cmd LIGHT;
        cmd LOCK;
        cmd DOOR_BEEP1;
        cmd DOOR_BEEP2;
        cmd DOOR_BEEP3;
    } cmds;

    const cmds Command = {
        .REBOOT1 = (cmd) { 0x400000000, 0x0000618b },
        .REBOOT2 = (cmd) { 0, 0x01009080 },
        .REBOOT3 = (cmd) { 0, 0x0000b1a0 },
        .REBOOT4 = (cmd) { 0, 0x01009080 },
        .REBOOT5 = (cmd) { 0x300000000, 0x00008092 },
        .REBOOT6 = (cmd) { 0x300000000, 0x00008092 },
        .DOOR1 = (cmd) { 0x200000000, 0x01018280 },
        .DOOR2 = (cmd) { 0x200000000, 0x01009280 },
        .LIGHT = (cmd) { 0x200000000, 0x00009281 },
        .LOCK = (cmd) { 0x0100000000, 0x0000728c },
        .DOOR_BEEP1 = (cmd) {  0x400000000, 0x0000f10a },
        .DOOR_BEEP2 = (cmd) { 0, 0x000b11a1 },
        .DOOR_BEEP3 = (cmd) { 0, 0x000c61a1 },
        // At auto close
        // [18:09:45][V][ratgdo:098]: command: cmd=040a nibble=01 byte1=00 byte2=00 fixed=c4a3d2c00a data=0000f10a  Timer == 0
        // [18:09:45][V][ratgdo:098]: command: cmd=040a nibble=01 byte1=00 byte2=00 fixed=c4a3d2c00a data=0000f10a  Timer == 0
        // [18:09:45][V][ratgdo:098]: command: cmd=00a1 nibble=01 byte1=0b byte2=00 fixed=c0a3d2c00a data=000b11a1  close1?
        // [18:09:53][V][ratgdo:098]: command: cmd=00a1 nibble=01 byte1=0c byte2=00 fixed=c0a3d2c00a data=000c61a1  close2?
        // [18:09:54][V][ratgdo:098]: command: cmd=0081 nibble=05 byte1=60 byte2=42 fixed=c0a3d2c00a data=4260c581  status
        // command: cmd=040a nibble=01 byte1=01 byte2=e0 fixed=c4a3d2c00a data=e001010a
        // time = (byte1 << 8) | byte2;
        // .AUTO_CLOSE = (cmd) { 0x0400000000, 0x0000010a },
    };
    struct RATGDOStore {
        ISRInternalGPIOPin input_obst;

        int obstructionLowCount = 0; // count obstruction low pulses
        long lastObstructionHigh = 0; // count time between high pulses from the obst ISR

        static void IRAM_ATTR HOT isrObstruction(RATGDOStore* arg);
    };

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
        void set_input_obst_pin(InternalGPIOPin* pin) { this->input_obst_pin_ = pin; };

        /********************************** FUNCTION DECLARATION
         * *****************************************/
        void transmit(cmd command);
        void sync();

        void gdoStateLoop();
        void obstructionLoop();
        void statusUpdateLoop();

        void sendCommandAndSaveCounter(cmd command);
        void toggleDoor();
        void openDoor();
        void closeDoor();
        void stopDoor();
        void toggleLight();
        void lightOn();
        void lightOff();
        bool isLightOn();
        void toggleLock();
        void closeBeep();
        void lock();
        void unlock();
        void query();

        void printRollingCode();
        void getRollingCode(cmd command);
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
        RATGDOStore store_ {};

        InternalGPIOPin* output_gdo_pin_;
        InternalGPIOPin* input_gdo_pin_;
        InternalGPIOPin* input_obst_pin_;

    }; // RATGDOComponent

} // namespace ratgdo
} // namespace esphome