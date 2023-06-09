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

#include "ratgdo.h"
#include "ratgdo_child.h"
#include "ratgdo_state.h"

#include "esphome/core/log.h"

namespace esphome {
namespace ratgdo {

    static const char* const TAG = "ratgdo";
    static const int STARTUP_DELAY = 2000; // delay before enabling interrupts
    static const uint64_t REMOTE_ID = 0x539;

    void IRAM_ATTR HOT RATGDOStore::isrObstruction(RATGDOStore* arg)
    {
        if (arg->input_obst.digital_read()) {
            arg->lastObstructionHigh = millis();
        } else {
            arg->obstructionLowCount++;
        }
    }

    void RATGDOComponent::setup()
    {
        this->pref_ = global_preferences->make_preference<int>(734874333U);
        if (!this->pref_.load(&this->rollingCodeCounter)) {
            this->rollingCodeCounter = 0;
        }

        this->output_gdo_pin_->setup();
        this->input_gdo_pin_->setup();
        this->input_obst_pin_->setup();

        this->store_.input_obst = this->input_obst_pin_->to_isr();

        this->output_gdo_pin_->pin_mode(gpio::FLAG_OUTPUT);
        this->input_gdo_pin_->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLUP);
        this->input_obst_pin_->pin_mode(gpio::FLAG_INPUT);

        this->check_uart_settings(9600, 1, esphome::uart::UART_CONFIG_PARITY_NONE, 8);

        this->input_obst_pin_->attach_interrupt(RATGDOStore::isrObstruction, &this->store_, gpio::INTERRUPT_ANY_EDGE);

        ESP_LOGD(TAG, "Syncing rolling code counter after reboot...");
        sync(); // reboot/sync to the opener on startup
    }

    void RATGDOComponent::loop()
    {
        obstructionLoop();
        gdoStateLoop();
        statusUpdateLoop();
    }

    void RATGDOComponent::dump_config()
    {
        ESP_LOGCONFIG(TAG, "Setting up RATGDO...");
        LOG_PIN("  Output GDO Pin: ", this->output_gdo_pin_);
        LOG_PIN("  Input GDO Pin: ", this->input_gdo_pin_);
        LOG_PIN("  Input Obstruction Pin: ", this->input_obst_pin_);
        ESP_LOGCONFIG(TAG, "  Rolling Code Counter: %d", this->rollingCodeCounter);
    }

    void RATGDOComponent::readRollingCode(bool& isStatus, uint8_t& door, uint8_t& light, uint8_t& lock, uint8_t& motion, uint8_t& obstruction, uint8_t& motor, uint16_t& openings)
    {
        uint32_t rolling = 0;
        uint64_t fixed = 0;
        uint32_t data = 0;

        uint16_t cmd = 0;
        uint8_t nibble = 0;
        uint8_t byte1 = 0;
        uint8_t byte2 = 0;

        decode_wireline(this->rxRollingCode, &rolling, &fixed, &data);

        cmd = ((fixed >> 24) & 0xf00) | (data & 0xff);

        nibble = (data >> 8) & 0xf;
        byte1 = (data >> 16) & 0xff;
        byte2 = (data >> 24) & 0xff;

        if (cmd == 0x81) {

            door = nibble;
            light = (byte2 >> 1) & 1;
            lock = byte2 & 1;
            motion = 0; // when the status message is read, reset motion state to 0|clear
            motor = 0; // when the status message is read, reset motor state to 0|off
            // obstruction = (byte1 >> 6) & 1; // unreliable due to the time it takes to register an obstruction
            ESP_LOGD(TAG, "Door: %d Light: %d Lock: %d Motion: %d Obstruction: %d", door, light, lock, motion, obstruction);
            isStatus = true;

        } else if (cmd == 0x281) {
            light ^= 1; // toggle bit
            ESP_LOGD(TAG, "Light: %d (toggle)", light);
        } else if (cmd == 0x84) {
            ESP_LOGD(TAG, "Unknown 0x84");
        } else if (cmd == 0x284) {
            motor = 1;
        } else if (cmd == 0x280) {
            ESP_LOGD(TAG, "Pressed: %s", byte1 == 1 ? "pressed" : "released");
        } else if (cmd == 0x48c) {
            openings = (byte1 << 8) | byte2;
            ESP_LOGD(TAG, "Openings: %d", (byte1 << 8) | byte2);
        } else if (cmd == 0x285) {
            motion = 1; // toggle bit
            ESP_LOGD(TAG, "Motion: %d (toggle)", motion);
        } else {
            ESP_LOGD(TAG, "Unknown command: %04x", cmd);
        }
    }

    void RATGDOComponent::getRollingCode(cmd command)
    {
        uint64_t fixed = command.fixed | REMOTE_ID;
        encode_wireline(this->rollingCodeCounter, fixed, command.data, this->txRollingCode);
        printRollingCode();
        if (command != Command.DOOR1) { // door2 is created with same counter and should always be called after door1
            incrementRollingCodeCounter();
        }
    }

    void RATGDOComponent::setRollingCodeCounter(uint32_t counter)
    {
        ESP_LOGD(TAG, "Set rolling code counter to %d", counter);
        this->rollingCodeCounter = counter;
        this->pref_.save(&this->rollingCodeCounter);
        sendRollingCodeChanged();
    }

    void RATGDOComponent::incrementRollingCodeCounter()
    {
        ESP_LOGD(TAG, "Incrementing rolling code counter");
        this->rollingCodeCounter = (this->rollingCodeCounter + 1) & 0xfffffff;
        sendRollingCodeChanged();
    }

    void RATGDOComponent::sendRollingCodeChanged()
    {
        for (auto* child : this->children_) {
            child->on_rolling_code_change(this->rollingCodeCounter);
        }
    }

    void RATGDOComponent::printRollingCode()
    {
        ESP_LOGD(TAG, "Counter: %d Send code: [%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X]",
            this->rollingCodeCounter,
            this->txRollingCode[0],
            this->txRollingCode[1],
            this->txRollingCode[2],
            this->txRollingCode[3],
            this->txRollingCode[4],
            this->txRollingCode[5],
            this->txRollingCode[6],
            this->txRollingCode[7],
            this->txRollingCode[8],
            this->txRollingCode[9],
            this->txRollingCode[10],
            this->txRollingCode[11],
            this->txRollingCode[12],
            this->txRollingCode[13],
            this->txRollingCode[14],
            this->txRollingCode[15],
            this->txRollingCode[16],
            this->txRollingCode[17],
            this->txRollingCode[18]);
    }

    /*************************** OBSTRUCTION DETECTION ***************************/

    void RATGDOComponent::obstructionLoop()
    {
        long currentMillis = millis();
        static unsigned long lastMillis = 0;

        // the obstruction sensor has 3 states: clear (HIGH with LOW pulse every 7ms), obstructed (HIGH), asleep (LOW)
        // the transitions between awake and asleep are tricky because the voltage drops slowly when falling asleep
        // and is high without pulses when waking up

        // If at least 3 low pulses are counted within 50ms, the door is awake, not obstructed and we don't have to check anything else

        // Every 50ms
        if (currentMillis - lastMillis > 50) {
            // check to see if we got between 3 and 8 low pulses on the line
            if (this->store_.obstructionLowCount >= 3 && this->store_.obstructionLowCount <= 8) {
                // obstructionCleared();
                this->obstructionState = ObstructionState::OBSTRUCTION_STATE_CLEAR;

                // if there have been no pulses the line is steady high or low
            } else if (this->store_.obstructionLowCount == 0) {
                // if the line is high and the last high pulse was more than 70ms ago, then there is an obstruction present
                if (this->input_obst_pin_->digital_read() && currentMillis - this->store_.lastObstructionHigh > 70) {
                    this->obstructionState = ObstructionState::OBSTRUCTION_STATE_OBSTRUCTED;
                    // obstructionDetected();
                } else {
                    // asleep
                }
            }

            lastMillis = currentMillis;
            this->store_.obstructionLowCount = 0;
        }
    }

    void RATGDOComponent::gdoStateLoop()
    {
        static uint32_t msgStart;
        static bool reading = false;
        static uint16_t byteCount = 0;
        static bool isStatus = false;

        while (this->available()) {
            // ESP_LOGD(TAG, "No data available input:%d output:%d", this->input_gdo_pin_->get_pin(), this->output_gdo_pin_->get_pin());
            uint8_t serData;
            if (!this->read_byte(&serData)) {
                ESP_LOGD(TAG, "Failed to read byte");
                return;
            }
            if (!reading) {
                // shift serial byte onto msg start
                msgStart <<= 8;
                msgStart |= serData;

                // truncate to 3 bytes
                msgStart &= 0x00FFFFFF;

                // if we are at the start of a message, capture the next 16 bytes
                if (msgStart == 0x550100) {
                    byteCount = 3;
                    rxRollingCode[0] = 0x55;
                    rxRollingCode[1] = 0x01;
                    rxRollingCode[2] = 0x00;

                    reading = true;
                    return;
                }
            }
            if (reading) {
                this->rxRollingCode[byteCount] = serData;
                byteCount++;

                if (byteCount == CODE_LENGTH) {
                    reading = false;
                    msgStart = 0;
                    byteCount = 0;
                    isStatus = false;

                    readRollingCode(isStatus, this->doorState, this->lightState, this->lockState, this->motionState, this->obstructionState, this->motorState, this->openings);
                    if (isStatus && this->forceUpdate_) {
                        this->forceUpdate_ = false;
                        this->previousDoorState = DoorState::DOOR_STATE_UNKNOWN;
                        this->previousLightState = LightState::LIGHT_STATE_UNKNOWN;
                        this->previousLockState = LockState::LOCK_STATE_UNKNOWN;
                    }
                }
            }
        }
    }

    void RATGDOComponent::statusUpdateLoop()
    {
        if (this->doorState != this->previousDoorState)
            sendDoorStatus();
        this->previousDoorState = this->doorState;
        if (this->lightState != this->previousLightState)
            sendLightStatus();
        this->previousLightState = this->lightState;
        if (this->lockState != this->previousLockState)
            sendLockStatus();
        this->previousLockState = this->lockState;
        if (this->obstructionState != this->previousObstructionState)
            sendObstructionStatus();
        this->previousObstructionState = this->obstructionState;
        if (this->motorState != this->previousMotorState) {
            sendMotorStatus();
            this->previousMotorState = this->motorState;
        }
        if (this->motionState == MotionState::MOTION_STATE_DETECTED) {
            sendMotionStatus();
            this->motionState = MotionState::MOTION_STATE_CLEAR;
        }
        if (this->openings != this->previousOpenings) {
            sendOpenings();
            this->previousOpenings = this->openings;
        }
    }

    void RATGDOComponent::query()
    {
        this->forceUpdate_ = true;
        sendCommandAndSaveCounter(Command.REBOOT2);
    }

    void RATGDOComponent::sendOpenings()
    {
        ESP_LOGD(TAG, "Openings: %d", this->openings);
        for (auto* child : this->children_) {
            child->on_openings_change(this->openings);
        }
    }

    void RATGDOComponent::sendDoorStatus()
    {
        DoorState val = static_cast<DoorState>(this->doorState);
        ESP_LOGD(TAG, "Door state: %s", door_state_to_string(val));
        for (auto* child : this->children_) {
            child->on_door_state(val);
        }
    }

    void RATGDOComponent::sendLightStatus()
    {
        LightState val = static_cast<LightState>(this->lightState);
        ESP_LOGD(TAG, "Light state %s (%d)", light_state_to_string(val), this->lightState);
        for (auto* child : this->children_) {
            child->on_light_state(val);
        }
    }

    void RATGDOComponent::sendLockStatus()
    {
        LockState val = static_cast<LockState>(this->lockState);
        ESP_LOGD(TAG, "Lock state %s", lock_state_to_string(val));
        for (auto* child : this->children_) {
            child->on_lock_state(val);
        }
    }

    void RATGDOComponent::sendMotionStatus()
    {
        MotionState val = static_cast<MotionState>(this->motionState);
        ESP_LOGD(TAG, "Motion state %s", motion_state_to_string(val));
        for (auto* child : this->children_) {
            child->on_motion_state(val);
        }
    }

    void RATGDOComponent::sendMotorStatus()
    {
        MotorState val = static_cast<MotorState>(this->motorState);
        ESP_LOGD(TAG, "Motor state %s", motor_state_to_string(val));
        for (auto* child : this->children_) {
            child->on_motor_state(val);
        }
    }

    void RATGDOComponent::sendObstructionStatus()
    {
        ObstructionState val = static_cast<ObstructionState>(this->obstructionState);
        ESP_LOGD(TAG, "Obstruction state %s", obstruction_state_to_string(val));
        for (auto* child : this->children_) {
            child->on_obstruction_state(val);
        }
    }

    /************************* DOOR COMMUNICATION *************************/
    /*
     * Transmit a message to the door opener over uart1
     * The TX1 pin is controlling a transistor, so the logic is inverted
     * A HIGH state on TX1 will pull the 12v line LOW
     *
     * The opener requires a specific duration low/high pulse before it will accept
     * a message
     */
    void RATGDOComponent::transmit(cmd command)
    {
        getRollingCode(command);
        this->output_gdo_pin_->digital_write(true); // pull the line high for 1305 micros so the
                                                    // door opener responds to the message
        delayMicroseconds(1305);
        this->output_gdo_pin_->digital_write(false); // bring the line low

        delayMicroseconds(1260); // "LOW" pulse duration before the message start
        this->write_array(this->txRollingCode, CODE_LENGTH);
    }

    void RATGDOComponent::sync()
    {
        transmit(Command.REBOOT1);
        delay(65);
        transmit(Command.REBOOT2);
        delay(65);
        transmit(Command.REBOOT3);
        delay(65);
        transmit(Command.REBOOT4);
        delay(65);
        transmit(Command.REBOOT5);
        delay(65);
        sendCommandAndSaveCounter(Command.REBOOT6);
        delay(65);
    }

    void RATGDOComponent::openDoor()
    {
        if (this->doorState == DoorState::DOOR_STATE_OPEN || this->doorState == DoorState::DOOR_STATE_OPENING) {
            ESP_LOGD(TAG, "The door is already %s", door_state_to_string(static_cast<DoorState>(this->doorState)));
            return;
        }
        toggleDoor();
    }

    void RATGDOComponent::closeDoor()
    {
        if (this->doorState == DoorState::DOOR_STATE_CLOSED || this->doorState == DoorState::DOOR_STATE_CLOSING) {
            ESP_LOGD(TAG, "The door is already %s", door_state_to_string(static_cast<DoorState>(this->doorState)));
            return;
        }
        toggleDoor();
    }

    void RATGDOComponent::stopDoor()
    {
        if (this->doorState != DoorState::DOOR_STATE_OPENING && this->doorState != DoorState::DOOR_STATE_CLOSING) {
            ESP_LOGD(TAG, "The door is not moving.");
            return;
        }
        toggleDoor();
    }

    void RATGDOComponent::toggleDoor()
    {
        transmit(Command.DOOR1);
        delay(40);
        sendCommandAndSaveCounter(Command.DOOR2);
    }

    bool RATGDOComponent::isLightOn()
    {
        return this->lightState == LightState::LIGHT_STATE_ON;
    }

    void RATGDOComponent::lightOn()
    {
        if (this->lightState == LightState::LIGHT_STATE_ON) {
            ESP_LOGD(TAG, "The light is already on");
            return;
        }
        toggleLight();
        // We don't always get the state back so be optimistic
        this->previousLightState = this->lightState;
        this->lightState = LightState::LIGHT_STATE_ON;
    }

    void RATGDOComponent::lightOff()
    {
        if (this->lightState == LightState::LIGHT_STATE_OFF) {
            ESP_LOGD(TAG, "The light is already off");
            return;
        }
        toggleLight();
        // We don't always get the state back so be optimistic
        this->previousLightState = this->lightState;
        this->lightState = LightState::LIGHT_STATE_OFF;
    }

    void RATGDOComponent::toggleLight()
    {
        sendCommandAndSaveCounter(Command.LIGHT);
    }

    // Lock functions
    void RATGDOComponent::lock()
    {
        if (this->lockState == LockState::LOCK_STATE_LOCKED) {
            ESP_LOGD(TAG, "already locked");
            return;
        }
        toggleLock();
    }

    void RATGDOComponent::unlock()
    {
        if (this->lockState == LockState::LOCK_STATE_UNLOCKED) {
            ESP_LOGD(TAG, "already unlocked");
            return;
        }
        toggleLock();
    }

    void RATGDOComponent::toggleLock()
    {
        sendCommandAndSaveCounter(Command.LOCK);
    }

    void RATGDOComponent::sendCommandAndSaveCounter(cmd command)
    {
        transmit(command);
        this->pref_.save(&this->rollingCodeCounter);
        global_preferences->sync();
    }

    void RATGDOComponent::register_child(RATGDOClient* obj)
    {
        this->children_.push_back(obj);
        obj->set_parent(this);
    }
    LightState RATGDOComponent::getLightState()
    {
        return static_cast<LightState>(this->lightState);
    }

} // namespace ratgdo
} // namespace esphome
