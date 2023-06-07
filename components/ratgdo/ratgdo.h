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

    enum Commands {
        REBOOT1,
        REBOOT2,
        REBOOT3,
        REBOOT4,
        REBOOT5,
        REBOOT6,
        DOOR1,
        DOOR2,
        LIGHT,
        LOCK,
    };
    struct RATGDOStore {
        ISRInternalGPIOPin input_obst;

        ISRInternalGPIOPin trigger_open;
        ISRInternalGPIOPin trigger_close;
        ISRInternalGPIOPin trigger_light;

        bool dryContactDoorOpen { false };
        bool dryContactDoorClose { false };
        bool dryContactToggleLight { false };

        int obstructionLowCount = 0; // count obstruction low pulses
        long lastObstructionHigh = 0; // count time between high pulses from the obst ISR

        uint8_t obstructionState { ObstructionState::OBSTRUCTION_STATE_UNKNOWN };
        uint8_t motionState { MotionState::MOTION_STATE_CLEAR };
        uint8_t lockState { LockState::LOCK_STATE_UNKNOWN };
        uint8_t lightState { LightState::LIGHT_STATE_UNKNOWN };
        uint8_t doorState { DoorState::DOOR_STATE_UNKNOWN };

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

        void set_output_gdo_pin(InternalGPIOPin* pin) { this->output_gdo_pin_ = pin; };
        void set_input_gdo_pin(InternalGPIOPin* pin) { this->input_gdo_pin_ = pin; };
        void set_input_obst_pin(InternalGPIOPin* pin) { this->input_obst_pin_ = pin; };

        void set_trigger_open_pin(InternalGPIOPin* pin) { this->trigger_open_pin_ = pin; };
        void set_trigger_close_pin(InternalGPIOPin* pin) { this->trigger_close_pin_ = pin; };
        void set_trigger_light_pin(InternalGPIOPin* pin) { this->trigger_light_pin_ = pin; };

        void set_status_door_pin(InternalGPIOPin* pin) { this->status_door_pin_ = pin; };
        void set_status_obst_pin(InternalGPIOPin* pin) { this->status_obst_pin_ = pin; };

        /********************************** FUNCTION DECLARATION
         * *****************************************/
        void transmit(Commands command);
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
        void sendLightStatus();

        void toggleLock();
        void lock();
        void unlock();
        void sendLockStatus();

        void sendMotionStatus();

        void doorStateLoop();
        void dryContactLoop();
        void printRollingCode();
        void getRollingCode(Commands command);
        void gdoStateLoop();
        void statusUpdateLoop();
        void readRollingCode(uint8_t& door, uint8_t& light, uint8_t& lock, uint8_t& motion, uint8_t& obstruction);
        void sendCommand(Commands command);
        /** Register a child component. */
        void register_child(RATGDOClient* obj);

    protected:
        ESPPreferenceObject pref_;
        std::vector<RATGDOClient*> children_;
        RATGDOStore store_ {};

        InternalGPIOPin* output_gdo_pin_;

        InternalGPIOPin* input_gdo_pin_;
        InternalGPIOPin* input_obst_pin_;

        InternalGPIOPin* trigger_open_pin_;
        InternalGPIOPin* trigger_close_pin_;
        InternalGPIOPin* trigger_light_pin_;

        InternalGPIOPin* status_door_pin_;
        InternalGPIOPin* status_obst_pin_;

    }; // RATGDOComponent

} // namespace ratgdo
} // namespace esphome