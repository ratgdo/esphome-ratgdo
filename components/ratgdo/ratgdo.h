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

        float openingDuration { 0 };
        float closingDuration { 0 };

        uint8_t txRollingCode[CODE_LENGTH];
        uint8_t rxRollingCode[CODE_LENGTH];

        uint16_t previousOpenings { 0 }; // number of times the door has been opened
        uint16_t openings { 0 }; // number of times the door has been opened

        DoorState previousDoorState { DoorState::DOOR_STATE_UNKNOWN };
        DoorState doorState { DoorState::DOOR_STATE_UNKNOWN };

        float doorPosition { DOOR_POSITION_UNKNOWN };
        float previousDoorPosition { DOOR_POSITION_UNKNOWN };
        bool movingToPosition { false };

        LightState previousLightState { LightState::LIGHT_STATE_UNKNOWN };
        LightState lightState { LightState::LIGHT_STATE_UNKNOWN };

        LockState previousLockState { LockState::LOCK_STATE_UNKNOWN };
        LockState lockState { LockState::LOCK_STATE_UNKNOWN };

        ObstructionState previousObstructionState { ObstructionState::OBSTRUCTION_STATE_UNKNOWN };
        ObstructionState obstructionState { ObstructionState::OBSTRUCTION_STATE_UNKNOWN };

        MotorState previousMotorState { MotorState::MOTOR_STATE_UNKNOWN };
        MotorState motorState { MotorState::MOTOR_STATE_UNKNOWN };

        ButtonState previousButtonState { ButtonState::BUTTON_STATE_UNKNOWN };
        ButtonState buttonState { ButtonState::BUTTON_STATE_UNKNOWN };

        MotionState previousMotionState { MotionState::MOTION_STATE_UNKNOWN };
        MotionState motionState { MotionState::MOTION_STATE_UNKNOWN };

        void set_output_gdo_pin(InternalGPIOPin* pin) { this->output_gdo_pin_ = pin; };
        void set_input_gdo_pin(InternalGPIOPin* pin) { this->input_gdo_pin_ = pin; };
        void set_input_obst_pin(InternalGPIOPin* pin) { this->input_obst_pin_ = pin; };
        void set_remote_id(uint64_t remote_id) { this->remote_id = remote_id & 0xffffff; }; // not sure how large remote_id can be, assuming not more than 24 bits

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
        void setDoorPosition(float position);
        void positionSyncWhileOpening(float delta, float update_period = 500);
        void positionSyncWhileClosing(float delta, float update_period = 500);
        void cancelPositionSyncCallbacks();

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

        void setOpeningDuration(float duration);
        void sendOpeningDuration();
        void setClosingDuration(float duration);
        void sendClosingDuration();

        LightState getLightState();
        /** Register a child component. */
        void register_child(RATGDOClient* obj);

    protected:
        ESPPreferenceObject rollingCodePref_;
        ESPPreferenceObject openingDurationPref_;
        ESPPreferenceObject closingDurationPref_;
        std::vector<RATGDOClient*> children_;
        bool rollingCodeUpdatesEnabled_ { true };
        bool forceUpdate_ { false };
        RATGDOStore store_ {};

        InternalGPIOPin* output_gdo_pin_;
        InternalGPIOPin* input_gdo_pin_;
        InternalGPIOPin* input_obst_pin_;
        uint64_t remote_id;

    }; // RATGDOComponent

} // namespace ratgdo
} // namespace esphome