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
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/preferences.h"

#include "SoftwareSerial.h"
extern "C" {
#include "secplus.h"
}

#define CODE_LENGTH 19

namespace esphome {
namespace ratgdo {

    class RATGDOComponent;

    struct RATGDOStore {
        ISRInternalGPIOPin output_gdo;

        ISRInternalGPIOPin input_gdo;
        ISRInternalGPIOPin input_obst;

        ISRInternalGPIOPin trigger_open;
        ISRInternalGPIOPin trigger_close;
        ISRInternalGPIOPin trigger_light;

        ISRInternalGPIOPin status_door;
        ISRInternalGPIOPin status_obst;

        unsigned long lastOpenDoorTime { 0 };
        unsigned long lastCloseDoorTime { 0 };
        unsigned long lastToggleLightTime { 0 };
        unsigned long lastPulse { 0 };
        volatile int doorPositionCounter { 0 }; // calculate the door's movement and position
        bool rpm1Pulsed { false }; // did rpm1 get a pulse or not - eliminates an issue when the
                                   // sensor is parked on a high pulse which fires rpm2 isr

        int obstructionLowCount = 0; // count obstruction low pulses
        long lastObstructionHigh = 0; // count time between high pulses from the obst ISR

        uint8_t obstructionState = 2;
        uint8_t motionState = 0;
        uint8_t lockState = 2;
        uint8_t lightState = 2;
        uint8_t doorState = 0;

        bool dryContactDoorOpen { false };
        bool dryContactDoorClose { false };
        bool dryContactToggleLight { false };

        static void IRAM_ATTR isrDoorOpen(RATGDOStore* arg);
        static void IRAM_ATTR isrDoorClose(RATGDOStore* arg);
        static void IRAM_ATTR isrLight(RATGDOStore* arg);
        static void IRAM_ATTR isrObstruction(RATGDOStore* arg);
    };

    class RATGDOComponent : public Component {
    public:
        void setup() override;
        void loop() override;
        /********************************** GLOBAL VARS
         * *****************************************/
        unsigned int rollingCodeCounter;
        SoftwareSerial swSerial;
        uint8_t txRollingCode[CODE_LENGTH];
        uint8_t rxRollingCode[CODE_LENGTH];
        String doorStates[6] = {"unknown","open","closed","stopped","opening","closing"};
        String lightStates[3] = {"off","on","unknown"};
        String lockStates[3] = {"unlocked","locked","unknown"};
        String motionStates[2] = {"clear","detected"};
        String obstructionStates[3] = {"obstructed","clear","unknown"};

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
        void set_rolling_codes(bool useRollingCodes);
        void transmit(const uint8_t*);
        void sync();
        void openDoor();
        void closeDoor();
        void toggleLight();
        void toggleDoor();
        void sendSyncCodes();

        void obstructionLoop();
        void obstructionDetected();
        void obstructionCleared();

        void sendDoorStatus();
        void doorStateLoop();
        void dryContactLoop();
        void printRollingCode();
        void getRollingCode(const char* command);
        void gdoStateLoop();
        void statusUpdateLoop();

    protected:
        ESPPreferenceObject pref_;
        bool useRollingCodes_ { true };
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