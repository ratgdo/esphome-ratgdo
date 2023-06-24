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
#include <ctime>

#include "esphome/core/log.h"

namespace esphome {
namespace ratgdo {

    static const char* const TAG = "ratgdo";
    static const int STARTUP_DELAY = 2000; // delay before enabling interrupts
    static const uint64_t REMOTE_ID = 0x539;
    static const uint16_t STATUS_CMD = 0x81;
    static const uint8_t MAX_CODES_WITHOUT_FLASH_WRITE = 3;

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

        this->swSerial.begin(9600, SWSERIAL_8N1, this->input_gdo_pin_->get_pin(), this->output_gdo_pin_->get_pin(), true);

        this->input_obst_pin_->attach_interrupt(RATGDOStore::isrObstruction, &this->store_, gpio::INTERRUPT_ANY_EDGE);

        ESP_LOGV(TAG, "Syncing rolling code counter after reboot...");
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

    uint16_t RATGDOComponent::readRollingCode()
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

        if (cmd == STATUS_CMD) {
            this->doorState = nibble;
            this->lightState = (byte2 >> 1) & 1;
            this->lockState = byte2 & 1;
            this->motionState = MotionState::MOTION_STATE_CLEAR; // when the status message is read, reset motion state to 0|clear
            this->motorState = MotorState::MOTOR_STATE_OFF; // when the status message is read, reset motor state to 0|off
            // obstruction = (byte1 >> 6) & 1; // unreliable due to the time it takes to register an obstruction
            ESP_LOGV(TAG, "Door: %d Light: %d Lock: %d", this->doorState, this->lightState, this->lockState);

        } else if (cmd == 0x281) {
            if (this->lightState == LightState::LIGHT_STATE_ON) {
                this->lightState = LightState::LIGHT_STATE_OFF;
            } else {
                this->lightState = LightState::LIGHT_STATE_ON;
            }
            ESP_LOGV(TAG, "Light: %d (toggle)", this->lightState);
        } else if (cmd == 0x284) {
            this->motorState = MotorState::MOTOR_STATE_ON;
        } else if (cmd == 0x280) {
            this->buttonState = byte1 == 1 ? ButtonState::BUTTON_STATE_PRESSED : ButtonState::BUTTON_STATE_RELEASED;
            ESP_LOGV(TAG, "Pressed: %d", this->buttonState);
        } else if (cmd == 0x48c) {
            this->openings = (byte1 << 8) | byte2;
            ESP_LOGV(TAG, "Openings: %d", this->openings);
        } else if (cmd == 0x285) {
            this->motionState = MotionState::MOTION_STATE_DETECTED; // toggle bit
            ESP_LOGV(TAG, "Motion: %d (toggle)", this->motionState);
        } else if (cmd == 0x40a) {
            time_t newAutoCloseTime = std::time(0) + ((byte1 << 8) | byte2);
            // The time will wobble a bit and since TTC close times are measured in minutes
            // we only update if the time is off by more than 30 seconds
            if (newAutoCloseTime + 30 != this->autoCloseTime && newAutoCloseTime - 30 != this->autoCloseTime) {
                this->autoCloseTime = newAutoCloseTime;
                ESP_LOGV(TAG, "Auto close time: %d", this->autoCloseTime);
            }
        } else {
            // 0x84 -- is it used?
            ESP_LOGV(TAG, "Unknown command: cmd=%04x nibble=%02x byte1=%02x byte2=%02x fixed=%010" PRIx64 " data=%08" PRIx32, cmd, nibble, byte1, byte2, fixed, data);
        }
        return cmd;
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
        ESP_LOGV(TAG, "Set rolling code counter to %d", counter);
        this->rollingCodeCounter = counter;
        this->pref_.save(&this->rollingCodeCounter);
        sendRollingCodeChanged();
    }

    void RATGDOComponent::incrementRollingCodeCounter()
    {
        this->rollingCodeCounter = (this->rollingCodeCounter + 1) & 0xfffffff;
        sendRollingCodeChanged();
    }

    void RATGDOComponent::sendRollingCodeChanged()
    {
        if (!this->rollingCodeUpdatesEnabled_) {
            return;
        }
        for (auto* child : this->children_) {
            child->on_rolling_code_change(this->rollingCodeCounter);
        }
    }

    void RATGDOComponent::printRollingCode()
    {
        ESP_LOGV(TAG, "Counter: %d Send code: [%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X]",
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
        static bool reading_msg = false;
        static uint32_t msg_start = 0;
        static uint16_t byte_count = 0;

        if (!reading_msg) {
            while (this->swSerial.available()) {
                uint8_t ser_byte = this->swSerial.read();
                if (ser_byte != 0x55 && ser_byte != 0x01 && ser_byte != 0x00) {
                    byte_count = 0;
                    continue;
                }
                msg_start = ((msg_start << 8) | ser_byte) & 0xffffff;
                byte_count++;

                // if we are at the start of a message, capture the next 16 bytes
                if (msg_start == 0x550100) {
                    this->rxRollingCode[0] = 0x55;
                    this->rxRollingCode[1] = 0x01;
                    this->rxRollingCode[2] = 0x00;

                    reading_msg = true;
                    break;
                }
            }
        }
        if (reading_msg) {
            while (this->swSerial.available()) {
                uint8_t ser_byte = this->swSerial.read();
                this->rxRollingCode[byte_count] = ser_byte;
                byte_count++;

                if (byte_count == CODE_LENGTH) {
                    reading_msg = false;
                    byte_count = 0;
                    if (readRollingCode() == STATUS_CMD && this->forceUpdate_) {
                        this->forceUpdate_ = false;
                        this->previousDoorState = DoorState::DOOR_STATE_UNKNOWN;
                        this->previousLightState = LightState::LIGHT_STATE_UNKNOWN;
                        this->previousLockState = LockState::LOCK_STATE_UNKNOWN;
                    }
                    return;
                }
            }
        }
    }

    void RATGDOComponent::statusUpdateLoop()
    {
        if (this->doorState != this->previousDoorState) {
            DoorState val = static_cast<DoorState>(this->doorState);
            ESP_LOGV(TAG, "Door state: %s", door_state_to_string(val));
            for (auto* child : this->children_) {
                child->on_door_state(val);
            }
            this->previousDoorState = this->doorState;
        }
        if (this->lightState != this->previousLightState) {
            LightState val = static_cast<LightState>(this->lightState);
            ESP_LOGV(TAG, "Light state %s (%d)", light_state_to_string(val), this->lightState);
            for (auto* child : this->children_) {
                child->on_light_state(val);
            }
            this->previousLightState = this->lightState;
        }
        if (this->lockState != this->previousLockState) {
            LockState val = static_cast<LockState>(this->lockState);
            ESP_LOGV(TAG, "Lock state %s", lock_state_to_string(val));
            for (auto* child : this->children_) {
                child->on_lock_state(val);
            }
            this->previousLockState = this->lockState;
        }
        if (this->obstructionState != this->previousObstructionState) {
            ObstructionState val = static_cast<ObstructionState>(this->obstructionState);
            ESP_LOGV(TAG, "Obstruction state %s", obstruction_state_to_string(val));
            for (auto* child : this->children_) {
                child->on_obstruction_state(val);
            }
            this->previousObstructionState = this->obstructionState;
        }
        if (this->motorState != this->previousMotorState) {
            MotorState val = static_cast<MotorState>(this->motorState);
            ESP_LOGV(TAG, "Motor state %s", motor_state_to_string(val));
            for (auto* child : this->children_) {
                child->on_motor_state(val);
            }
            this->previousMotorState = this->motorState;
        }
        if (this->motionState != this->previousMotionState) {
            MotionState val = static_cast<MotionState>(this->motionState);
            ESP_LOGV(TAG, "Motion state %s", motion_state_to_string(val));
            for (auto* child : this->children_) {
                child->on_motion_state(val);
            }
            this->previousMotionState = this->motionState;
        }
        if (this->buttonState != this->previousButtonState) {
            ButtonState val = static_cast<ButtonState>(this->buttonState);
            ESP_LOGV(TAG, "Button state %s", button_state_to_string(val));
            for (auto* child : this->children_) {
                child->on_button_state(val);
            }
            this->previousButtonState = this->buttonState;
        }
        if (this->openings != this->previousOpenings) {
            ESP_LOGV(TAG, "Openings: %d", this->openings);
            for (auto* child : this->children_) {
                child->on_openings_change(this->openings);
            }
            this->previousOpenings = this->openings;
        }
        if (this->autoCloseTime != this->previousAutoCloseTime) {
            ESP_LOGV(TAG, "Auto close time: %d", this->autoCloseTime);
            for (auto* child : this->children_) {
                child->on_auto_close_time_change(this->autoCloseTime);
            }
            this->previousAutoCloseTime = this->autoCloseTime;
        }
    }

    void RATGDOComponent::query()
    {
        this->forceUpdate_ = true;
        sendCommandAndSaveCounter(Command.REBOOT2);
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
        this->swSerial.write(this->txRollingCode, CODE_LENGTH);
    }

    void RATGDOComponent::sync()
    {
        this->rollingCodeUpdatesEnabled_ = false;
        for (int i = 0; i <= MAX_CODES_WITHOUT_FLASH_WRITE; i++) {
            transmit(Command.REBOOT1); // get openings
            delay(65);
        }
        transmit(Command.REBOOT2); // get state
        delay(65);
        transmit(Command.REBOOT3);
        delay(65);
        transmit(Command.REBOOT4);
        delay(65);
        transmit(Command.REBOOT5);
        delay(65);
        this->rollingCodeUpdatesEnabled_ = true;
        sendCommandAndSaveCounter(Command.REBOOT6);
        delay(65);
    }

    void RATGDOComponent::openDoor()
    {
        if (this->doorState == DoorState::DOOR_STATE_OPEN || this->doorState == DoorState::DOOR_STATE_OPENING) {
            ESP_LOGV(TAG, "The door is already %s", door_state_to_string(static_cast<DoorState>(this->doorState)));
            return;
        }
        toggleDoor();
    }

    void RATGDOComponent::closeDoor()
    {
        if (this->doorState == DoorState::DOOR_STATE_CLOSED || this->doorState == DoorState::DOOR_STATE_CLOSING) {
            ESP_LOGV(TAG, "The door is already %s", door_state_to_string(static_cast<DoorState>(this->doorState)));
            return;
        }
        toggleDoor();
    }

    void RATGDOComponent::stopDoor()
    {
        if (this->doorState != DoorState::DOOR_STATE_OPENING && this->doorState != DoorState::DOOR_STATE_CLOSING) {
            ESP_LOGV(TAG, "The door is not moving.");
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
            ESP_LOGV(TAG, "The light is already on");
            return;
        }
        toggleLight();
    }

    void RATGDOComponent::lightOff()
    {
        if (this->lightState == LightState::LIGHT_STATE_OFF) {
            ESP_LOGV(TAG, "The light is already off");
            return;
        }
        toggleLight();
    }

    void RATGDOComponent::toggleLight()
    {
        sendCommandAndSaveCounter(Command.LIGHT);
    }

    // Lock functions
    void RATGDOComponent::lock()
    {
        if (this->lockState == LockState::LOCK_STATE_LOCKED) {
            ESP_LOGV(TAG, "already locked");
            return;
        }
        toggleLock();
    }

    void RATGDOComponent::unlock()
    {
        if (this->lockState == LockState::LOCK_STATE_UNLOCKED) {
            ESP_LOGV(TAG, "already unlocked");
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
        if (!this->lastSyncedRollingCodeCounter || this->rollingCodeCounter - this->lastSyncedRollingCodeCounter >= MAX_CODES_WITHOUT_FLASH_WRITE) {
            this->lastSyncedRollingCodeCounter = this->rollingCodeCounter;
            global_preferences->sync();
        }
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
