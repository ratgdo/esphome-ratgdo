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
#include "esphome/core/preferences.h"

#include "SoftwareSerial.h"
extern "C" {
#include "secplus.h"
}

static const uint8_t D0 = 16;
static const uint8_t D1 = 5;
static const uint8_t D2 = 4;
static const uint8_t D3 = 0;
static const uint8_t D4 = 2;
static const uint8_t D5 = 14;
static const uint8_t D6 = 12;
static const uint8_t D7 = 13;
static const uint8_t D8 = 15;
static const uint8_t D9 = 3;
static const uint8_t D10 = 1;

#define CODE_LENGTH 19 // the length of each command sent to the door.
/********************************** PIN DEFINITIONS
 * *****************************************/
#define OUTPUT_GDO \
    D4 // red control terminal / GarageDoorOpener (UART1 TX) pin is D4 on D1 Mini
#define TRIGGER_OPEN D5 // dry contact for opening door
#define TRIGGER_CLOSE D6 // dry contact for closing door
#define TRIGGER_LIGHT \
    D3 // dry contact for triggering light (no discrete light commands, so toggle
       // only)
#define STATUS_DOOR D0 // output door status, HIGH for open, LOW for closed
#define STATUS_OBST \
    D8 // output for obstruction status, HIGH for obstructed, LOW for clear
#define INPUT_RPM1 \
    D1 // RPM1 rotary encoder input OR reed switch if not soldering to the door
       // opener logic board
#define INPUT_RPM2 \
    D2 // RPM2 rotary encoder input OR not used if using reed switch
#define INPUT_OBST D7 // black obstruction sensor terminal

namespace esphome {
namespace ratgdo {

    class RATGDOComponent : public Component {
    public:
        void setup() override;
        void loop() override;
        /********************************** GLOBAL VARS
         * *****************************************/
        unsigned int rollingCodeCounter;
        SoftwareSerial swSerial;
        byte rollingCode[CODE_LENGTH];
        String doorState = "unknown"; // will be
                                      // [online|offline|opening|open|closing|closed|obstructed|clear|reed_open|reed_closed]

        unsigned int obstructionLowCount = 0; // count obstruction low pulses
        unsigned long lastObstructionHigh = 0; // count time between high pulses from the obst ISR

        bool useRollingCodes = true; // use rolling codes or not
        bool doorIsObstructed = false;
        bool dryContactDoorOpen = false;
        bool dryContactDoorClose = false;
        bool dryContactToggleLight = false;
        int doorPositionCounter = 0; // calculate the door's movement and position
        bool rpm1Pulsed = false; // did rpm1 get a pulse or not - eliminates an issue when the
                                 // sensor is parked on a high pulse which fires rpm2 isr

        void set_output_gdo_pin(InternalGPIOPin* pin) { this->output_gdo_pin = pin; };
        void set_trigger_open_pin(InternalGPIOPin* pin) { this->trigger_open_pin = pin; };
        void set_trigger_close_pin(InternalGPIOPin* pin) { this->trigger_close_pin = pin; };
        void set_trigger_light_pin(InternalGPIOPin* pin) { this->trigger_light_pin = pin; };
        void set_status_door_pin(InternalGPIOPin* pin) { this->status_door_pin = pin; };
        void set_status_obst_pin(InternalGPIOPin* pin) { this->status_obst_pin = pin; };
        void set_input_rpm1_pin(InternalGPIOPin* pin) { this->input_rpm1_pin = pin; };
        void set_input_rpm2_pin(InternalGPIOPin* pin) { this->input_rpm2_pin = pin; };
        void set_input_obst_pin(InternalGPIOPin* pin) { this->input_obst_pin = pin; };

        /********************************** FUNCTION DECLARATION
         * *****************************************/
        void set_rolling_codes(bool useRollingCodes);
        void transmit(const unsigned char*, unsigned int);
        void sync();
        void openDoor();
        void closeDoor();
        void toggleLight();
        void sendSyncCodes();

        void obstructionLoop();
        void obstructionDetected();
        void obstructionCleared();

        void sendDoorStatus();
        void doorStateLoop();
        void dryContactLoop();
        void printRollingCode();
        void getRollingCode(const char* command);

        /********************************** INTERRUPT SERVICE ROUTINES
         * ***********************************/
        void IRAM_ATTR isrDebounce(const char* type);
        void IRAM_ATTR isrDoorOpen();
        void IRAM_ATTR isrDoorClose();
        void IRAM_ATTR isrLight();
        void IRAM_ATTR isrObstruction();
        void IRAM_ATTR isrRPM1();
        void IRAM_ATTR isrRPM2();

    protected:
        ESPPreferenceObject pref_;
        bool useRollingCodes_;
        InternalGPIOPin* output_gdo_pin;
        InternalGPIOPin* trigger_open_pin;
        InternalGPIOPin* trigger_close_pin;
        InternalGPIOPin* trigger_light_pin;
        InternalGPIOPin* status_door_pin;
        InternalGPIOPin* status_obst_pin;
        InternalGPIOPin* input_rpm1_pin;
        InternalGPIOPin* input_rpm2_pin;
        InternalGPIOPin* input_obst_pin;

    }; // RATGDOComponent

} // namespace ratgdo
} // namespace esphome