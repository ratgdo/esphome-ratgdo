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
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/log.h"
#include "esphome/core/preferences.h"

extern "C" {
#include "secplus.h"
}

#include "ratgdo_child.h"
#include "ratgdo_state.h"

#define CODE_LENGTH 19

namespace esphome {
namespace ratgdo {

    // Forward declare RATGDOClient
    class RATGDOClient;

    typedef struct {
        uint64_t fixed;
        uint32_t data;
        inline bool operator!=(command a)
        {
            return (fixed != a.fixed || data != a.data);
        }        
    } command;

    typedef struct {
        command REBOOT1;
        command REBOOT2;
        command REBOOT3;
        command REBOOT4;
        command REBOOT5;
        command REBOOT6;
        command DOOR1;
        command DOOR2;
        command LIGHT;
        command LOCK;
    } commands;

    const commands Command = {
        .REBOOT1 = (command) { 0x400000000, 0x0000618b },
        .REBOOT2 = (command) { 0, 0x01009080 },
        .REBOOT3 = (command) { 0, 0x0000b1a0 },
        .REBOOT4 = (command) { 0, 0x01009080 },
        .REBOOT5 = (command) { 0x300000000, 0x00008092 },
        .REBOOT6 = (command) { 0x300000000, 0x00008092 },
        .DOOR1 = (command) { 0x200000000, 0x01018280 },
        .DOOR2 = (command) { 0x200000000, 0x01009280 },
        .LIGHT = (command) { 0x200000000, 0x00009281 },
        .LOCK = (command) { 0x0100000000, 0x0000728c },
    };
    struct RATGDOStore {
        ISRInternalGPIOPin input_obst;

        int obstructionLowCount = 0; // count obstruction low pulses
        long lastObstructionHigh = 0; // count time between high pulses from the obst ISR

        static void IRAM_ATTR isrDoorOpen(RATGDOStore* arg);
        static void IRAM_ATTR isrDoorClose(RATGDOStore* arg);
        static void IRAM_ATTR isrLight(RATGDOStore* arg);
        static void IRAM_ATTR isrObstruction(RATGDOStore* arg);
    };

    class RATGDOComponent : public uart::UARTDevice, public Component {
    public:
        void setup() override;
        void loop() override;
        void dump_config() override;
        /********************************** GLOBAL VARS
         * *****************************************/
        uint32_t rollingCodeCounter;
        uint8_t txRollingCode[CODE_LENGTH];
        uint8_t rxRollingCode[CODE_LENGTH];

        uint8_t previousDoorState { DoorState::DOOR_STATE_UNKNOWN };
        uint8_t previousLightState { LightState::LIGHT_STATE_UNKNOWN };
        uint8_t previousLockState { LockState::LOCK_STATE_UNKNOWN };
        uint8_t previousObstructionState { ObstructionState::OBSTRUCTION_STATE_UNKNOWN };
        uint8_t previousMotorState { MotorState::MOTOR_STATE_UNKNOWN };

        uint8_t obstructionState { ObstructionState::OBSTRUCTION_STATE_UNKNOWN };
        uint8_t motionState { MotionState::MOTION_STATE_CLEAR };
        uint8_t motorState { MotorState::MOTOR_STATE_UNKNOWN };
        uint8_t lockState { LockState::LOCK_STATE_UNKNOWN };
        uint8_t lightState { LightState::LIGHT_STATE_UNKNOWN };
        uint8_t doorState { DoorState::DOOR_STATE_UNKNOWN };

        void set_output_gdo_pin(InternalGPIOPin* pin) { this->output_gdo_pin_ = pin; };
        void set_input_gdo_pin(InternalGPIOPin* pin) { this->input_gdo_pin_ = pin; };
        void set_input_obst_pin(InternalGPIOPin* pin) { this->input_obst_pin_ = pin; };

        void set_status_door_pin(InternalGPIOPin* pin) { this->status_door_pin_ = pin; };
        void set_status_obst_pin(InternalGPIOPin* pin) { this->status_obst_pin_ = pin; };

        /********************************** FUNCTION DECLARATION
         * *****************************************/
        void transmit(command command);
        void sync();

        void obstructionLoop();
        void sendObstructionStatus();

        void toggleDoor();
        void openDoor();
        void closeDoor();
        void stopDoor();
        void sendDoorStatus();

        void toggleLight();
        void lightOn();
        void lightOff();
        bool isLightOn();
        void sendLightStatus();

        void toggleLock();
        void lock();
        void unlock();
        void sendLockStatus();

        void sendMotionStatus();
        void sendMotorStatus();
        void query();
        void doorStateLoop();
        void printRollingCode();
        void getRollingCode(command command);
        void gdoStateLoop();
        void statusUpdateLoop();
        void readRollingCode(bool& isStatus, uint8_t& door, uint8_t& light, uint8_t& lock, uint8_t& motion, uint8_t& obstruction, uint8_t& motor);
        void incrementRollingCodeCounter();
        void sendRollingCodeChanged();
        void setRollingCodeCounter(uint32_t counter);
        void sendCommandAndSaveCounter(command command);
        /** Register a child component. */
        void register_child(RATGDOClient* obj);

    protected:
        ESPPreferenceObject pref_;
        std::vector<RATGDOClient*> children_;
        bool forceUpdate_ { false };
        RATGDOStore store_ {};

        InternalGPIOPin* output_gdo_pin_;

        InternalGPIOPin* input_gdo_pin_;
        InternalGPIOPin* input_obst_pin_;

        InternalGPIOPin* status_door_pin_;
        InternalGPIOPin* status_obst_pin_;

    }; // RATGDOComponent

} // namespace ratgdo
} // namespace esphome
