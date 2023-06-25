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
    static const int SYNC_DELAY = 2000;
    //
    // MAX_CODES_WITHOUT_FLASH_WRITE is a bit of a guess
    // since we write the flash at most every every 5s
    //
    // We want the rolling counter to be high enough that the
    // GDO will accept the command after an unexpected reboot
    // that did not save the counter to flash in time which
    // results in the rolling counter being behind what the GDO
    // expects.
    //
    static const uint8_t MAX_CODES_WITHOUT_FLASH_WRITE = 5;
    static const uint32_t FLASH_WRITE_INTERVAL = 10000;

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

        // many things happening at startup, use some delay for sync
        set_timeout(SYNC_DELAY, std::bind(&RATGDOComponent::sync, this));
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
        ESP_LOGCONFIG(TAG, "  Remote ID: %d", this->remote_id);
    }

    const char* cmd_name(uint16_t cmd)
    {
        // from: https://github.com/argilo/secplus/blob/f98c3220356c27717a25102c0b35815ebbd26ccc/secplus.py#L540
        switch (cmd) {
        // sent by opener (motor)
        case 0x081:
            return "status";
        case 0x084:
            return "unknown_1";
        case 0x085:
            return "unknown_2";
        case 0x0a1:
            return "pair_3_resp";
        case 0x284:
            return "motor_on";
        case 0x393:
            return "learn_3_resp";
        case 0x401:
            return "pair_2_resp";
        case 0x48c:
            return "openings";

        // sent by switch
        case 0x080:
            return "get_status";
        case 0x0a0:
            return "pair_3";
        case 0x181:
            return "learn_2";
        case 0x18c:
            return "lock";
        case 0x280:
            return "open";
        case 0x281:
            return "light";
        case 0x285:
            return "motion";
        case 0x391:
            return "learn_1";
        case 0x392:
            return "learn_3";
        case 0x400:
            return "pair_2";
        case 0x48b:
            return "get_openings";
        default:
            return "unknown";
        }
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
        data &= ~0xf000; // clear parity nibble

        if ((fixed & 0xfff) == this->remote_id) { // my commands
            ESP_LOGV(TAG, "[%ld] received mine: rolling=%07" PRIx32 " fixed=%010" PRIx64 " data=%08" PRIx32, millis(), rolling, fixed, data);
            return 0;
        } else {
            ESP_LOGV(TAG, "[%ld] received rolling=%07" PRIx32 " fixed=%010" PRIx64 " data=%08" PRIx32, millis(), rolling, fixed, data);
        }

        nibble = (data >> 8) & 0xff;
        byte1 = (data >> 16) & 0xff;
        byte2 = (data >> 24) & 0xff;

        ESP_LOGV(TAG, "cmd=%03x (%s) byte2=%02x byte1=%02x nibble=%01x", cmd, cmd_name(cmd), byte2, byte1, nibble);

        if (cmd == command::STATUS) {
            auto doorState = static_cast<DoorState>(nibble);
            if (doorState == DoorState::DOOR_STATE_CLOSED && this->doorState != doorState) {
                transmit(command::GET_OPENINGS);
            }

            this->doorState = doorState;
            this->lightState = static_cast<LightState>((byte2 >> 1) & 1);
            this->lockState = static_cast<LockState>(byte2 & 1);
            this->motionState = MotionState::MOTION_STATE_CLEAR; // when the status message is read, reset motion state to 0|clear
            this->motorState = MotorState::MOTOR_STATE_OFF; // when the status message is read, reset motor state to 0|off
            // this->obstructionState = static_cast<ObstructionState>((byte1 >> 6) & 1);
            ESP_LOGV(TAG, "Status: door=%s light=%s lock=%s",
                door_state_to_string(this->doorState),
                light_state_to_string(this->lightState),
                lock_state_to_string(this->lockState));
        } else if (cmd == command::LIGHT) {
            if (nibble == 0) {
                this->lightState = LightState::LIGHT_STATE_OFF;
            } else if (nibble == 1) {
                this->lightState = LightState::LIGHT_STATE_ON;
            } else if (nibble == 2) { // toggle
                this->lightState = light_state_toggle(this->lightState);
            }
            ESP_LOGV(TAG, "Light: action=%s state=%s",
                nibble == 0 ? "OFF" : nibble == 1 ? "ON"
                                                  : "TOGGLE",
                light_state_to_string(this->lightState));
        } else if (cmd == command::MOTOR_ON) {
            this->motorState = MotorState::MOTOR_STATE_ON;
            ESP_LOGV(TAG, "Motor: state=%s", motor_state_to_string(this->motorState));
        } else if (cmd == command::OPEN) {
            this->buttonState = (byte1 & 1) == 1 ? ButtonState::BUTTON_STATE_PRESSED : ButtonState::BUTTON_STATE_RELEASED;
            ESP_LOGV(TAG, "Open: button=%s", button_state_to_string(this->buttonState));
        } else if (cmd == command::OPENINGS) {
            this->openings = (byte1 << 8) | byte2;
            ESP_LOGV(TAG, "Openings: %d", this->openings);
        } else if (cmd == command::MOTION) {
            this->motionState = MotionState::MOTION_STATE_DETECTED;
            if (this->lightState == LightState::LIGHT_STATE_OFF) {
                transmit(command::GET_STATUS);
            }
            ESP_LOGV(TAG, "Motion: %s", motion_state_to_string(this->motionState));
        } else if (cmd == 0x40a) {
            uint32_t secondsUntilClose = ((byte1 << 8) | byte2);
            if (secondsUntilClose) {
                time_t newAutoCloseTime = std::time(nullptr) + secondsUntilClose;
                // The time will wobble a bit and since TTC close times are measured in minutes
                // we only update if the time is off by more than 30 seconds
                if (newAutoCloseTime + 30 < this->autoCloseTime || newAutoCloseTime - 30 > this->autoCloseTime) {
                    this->autoCloseTime = newAutoCloseTime;
                }
            }
        } else {
            ESP_LOGV(TAG, "Unhandled command: cmd=%03x nibble=%02x byte1=%02x byte2=%02x fixed=%010" PRIx64 " data=%08" PRIx32, cmd, nibble, byte1, byte2, fixed, data);
        }
        return cmd;
    }

    void RATGDOComponent::getRollingCode(command::cmd command, uint32_t data, bool increment)
    {
        uint64_t fixed = ((command & ~0xff) << 24) | this->remote_id;
        uint32_t send_data = (data << 8) | (command & 0xff);

        ESP_LOGV(TAG, "[%ld] Encode for transmit rolling=%07" PRIx32 " fixed=%010" PRIx64 " data=%08" PRIx32, millis(), this->rollingCodeCounter, fixed, send_data);
        encode_wireline(this->rollingCodeCounter, fixed, send_data, this->txRollingCode);

        printRollingCode();
        if (increment) {
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
                    if (readRollingCode() == command::STATUS && this->forceUpdate_) {
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
            ESP_LOGV(TAG, "Door state: %s", door_state_to_string(this->doorState));
            for (auto* child : this->children_) {
                child->on_door_state(this->doorState);
            }
            this->previousDoorState = this->doorState;
        }
        if (this->lightState != this->previousLightState) {
            ESP_LOGV(TAG, "Light state %s (%d)", light_state_to_string(this->lightState), this->lightState);
            for (auto* child : this->children_) {
                child->on_light_state(this->lightState);
            }
            this->previousLightState = this->lightState;
        }
        if (this->lockState != this->previousLockState) {
            ESP_LOGV(TAG, "Lock state %s", lock_state_to_string(this->lockState));
            for (auto* child : this->children_) {
                child->on_lock_state(this->lockState);
            }
            this->previousLockState = this->lockState;
        }
        if (this->obstructionState != this->previousObstructionState) {
            ESP_LOGV(TAG, "Obstruction state %s", obstruction_state_to_string(this->obstructionState));
            for (auto* child : this->children_) {
                child->on_obstruction_state(this->obstructionState);
            }
            this->previousObstructionState = this->obstructionState;
        }
        if (this->motorState != this->previousMotorState) {
            ESP_LOGV(TAG, "Motor state %s", motor_state_to_string(this->motorState));
            for (auto* child : this->children_) {
                child->on_motor_state(this->motorState);
            }
            this->previousMotorState = this->motorState;
        }
        if (this->motionState != this->previousMotionState) {
            ESP_LOGV(TAG, "Motion state %s", motion_state_to_string(this->motionState));
            for (auto* child : this->children_) {
                child->on_motion_state(this->motionState);
            }
            this->previousMotionState = this->motionState;
        }
        if (this->buttonState != this->previousButtonState) {
            ESP_LOGV(TAG, "Button state %s", button_state_to_string(this->buttonState));
            for (auto* child : this->children_) {
                child->on_button_state(this->buttonState);
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

    void RATGDOComponent::query_status()
    {
        this->forceUpdate_ = true;
        transmit(command::GET_STATUS);
    }

    void RATGDOComponent::query_openings()
    {
        transmit(command::GET_OPENINGS);
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
    void RATGDOComponent::transmit(command::cmd command, uint32_t data, bool increment)
    {
        getRollingCode(command, data, increment);
        this->output_gdo_pin_->digital_write(true); // pull the line high for 1305 micros so the
                                                    // door opener responds to the message
        delayMicroseconds(1305);
        this->output_gdo_pin_->digital_write(false); // bring the line low

        delayMicroseconds(1260); // "LOW" pulse duration before the message start
        this->swSerial.write(this->txRollingCode, CODE_LENGTH);

        saveCounter();
    }

    void RATGDOComponent::sync()
    {
        if (this->rollingCodeCounter == 0) { // first time use
            this->rollingCodeCounter = 1;
            // the opener only sends a reply when the rolling code > previous rolling code for a given remote id
            // when used the first time there is no previous rolling code, so first command is ignored
            set_timeout(100, [=] {
                transmit(command::GET_STATUS);
            });
            // send it twice since manual says it can take 3 button presses for door to open on first use
            set_timeout(200, [=] {
                transmit(command::GET_STATUS);
            });
        }
        for (int i = 0; i <= MAX_CODES_WITHOUT_FLASH_WRITE; i++) {
            set_timeout(300 + i * 100, [=] {
                transmit(command::GET_STATUS);
            });
        }
        set_timeout(400 + 100 * MAX_CODES_WITHOUT_FLASH_WRITE, [=] {
            transmit(command::GET_OPENINGS);
        });
    }

    void RATGDOComponent::openDoor()
    {
        doorCommand(data::DOOR_OPEN);
    }

    void RATGDOComponent::closeDoor()
    {
        doorCommand(data::DOOR_CLOSE);
    }

    void RATGDOComponent::stopDoor()
    {
        if (this->doorState != DoorState::DOOR_STATE_OPENING && this->doorState != DoorState::DOOR_STATE_CLOSING) {
            ESP_LOGV(TAG, "The door is not moving.");
            return;
        }
        doorCommand(data::DOOR_STOP);
    }

    void RATGDOComponent::toggleDoor()
    {
        doorCommand(data::DOOR_TOGGLE);
    }

    void RATGDOComponent::doorCommand(uint32_t data)
    {
        data |= (1 << 16); // button 1 ?
        data |= (1 << 8); // button press
        transmit(command::OPEN, data, false);
        set_timeout(100, [=] {
            auto data2 = data & ~(1 << 8); // button release
            transmit(command::OPEN, data2);
        });
    }

    void RATGDOComponent::lightOn()
    {
        this->lightState = LightState::LIGHT_STATE_ON;
        transmit(command::LIGHT, data::LIGHT_ON);
    }

    void RATGDOComponent::lightOff()
    {
        this->lightState = LightState::LIGHT_STATE_OFF;
        transmit(command::LIGHT, data::LIGHT_OFF);
    }

    void RATGDOComponent::toggleLight()
    {
        this->lightState = light_state_toggle(this->lightState);
        transmit(command::LIGHT, data::LIGHT_TOGGLE);
    }

    // Lock functions
    void RATGDOComponent::lock()
    {
        this->lockState = LockState::LOCK_STATE_LOCKED;
        transmit(command::LOCK, data::LOCK_ON);
    }

    void RATGDOComponent::unlock()
    {
        transmit(command::LOCK, data::LOCK_OFF);
    }

    void RATGDOComponent::toggleLock()
    {
        this->lockState = lock_state_toggle(this->lockState);
        transmit(command::LOCK, data::LOCK_TOGGLE);
    }

    void RATGDOComponent::saveCounter()
    {
        this->pref_.save(&this->rollingCodeCounter);
        // Forcing a sync results in a soft reset if there are too many
        // writes to flash in a short period of time. To avoid this,
        // we have configured preferences to write every 5s
    }

    void RATGDOComponent::register_child(RATGDOClient* obj)
    {
        this->children_.push_back(obj);
        obj->set_parent(this);
    }
    LightState RATGDOComponent::getLightState()
    {
        return this->lightState;
    }

} // namespace ratgdo
} // namespace esphome
