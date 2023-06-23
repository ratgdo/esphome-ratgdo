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

    void RATGDOComponent::setup()
    {
        this->pref_ = global_preferences->make_preference<int>(734874333U);
        if (!this->pref_.load(&this->rollingCodeCounter)) {
            this->rollingCodeCounter = 0;
        }

        this->output_gdo_pin_->setup();
        this->input_gdo_pin_->setup();

        this->output_gdo_pin_->pin_mode(gpio::FLAG_OUTPUT);
        this->input_gdo_pin_->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLUP);

        this->swSerial.begin(9600, SWSERIAL_8N1, this->input_gdo_pin_->get_pin(), this->output_gdo_pin_->get_pin(), true);

        // save counter to flash every 10s if it changed
        set_interval(10000, std::bind(&RATGDOComponent::saveCounter, this));

        // sync state on startup
        sync();
    }

    void RATGDOComponent::loop()
    {
        gdoStateLoop();
        statusUpdateLoop();
    }

    void RATGDOComponent::dump_config()
    {
        ESP_LOGCONFIG(TAG, "Setting up RATGDO...");
        LOG_PIN("  Output GDO Pin: ", this->output_gdo_pin_);
        LOG_PIN("  Input GDO Pin: ", this->input_gdo_pin_);
        ESP_LOGCONFIG(TAG, "  Rolling Code Counter: %d", this->rollingCodeCounter);
        ESP_LOGCONFIG(TAG, "  Remote ID: %d", this->remote_id);
    }

    const char* cmd_name(uint16_t cmd) {
        // from: https://github.com/argilo/secplus/blob/f98c3220356c27717a25102c0b35815ebbd26ccc/secplus.py#L540
        switch (cmd) {
            // sent by opener (motor)
            case 0x081: return "status";
            case 0x084: return "unknown_1";
            case 0x085: return "unknown_2";
            case 0x0a1: return "pair_3_resp";
            case 0x284: return "motor_on";
            case 0x393: return "learn_3_resp";
            case 0x401: return "pair_2_resp";
            case 0x48c: return "openings";

            // sent by switch
            case 0x080: return "get_status";
            case 0x0a0: return "pair_3";
            case 0x181: return "learn_2";
            case 0x18c: return "lock";
            case 0x280: return "open";
            case 0x281: return "light";
            case 0x285: return "motion";
            case 0x391: return "learn_1";
            case 0x392: return "learn_3";
            case 0x400: return "pair_2";
            case 0x48b: return "get_openings";
            default: return "unknown";
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
            ESP_LOGD(TAG, "[%ld] received mine: rolling=%07" PRIx32 " fixed=%010" PRIx64 " data=%08" PRIx32, millis(), rolling, fixed, data);
            return 0;
        } else {
            ESP_LOGD(TAG, "[%ld] received rolling=%07" PRIx32 " fixed=%010" PRIx64 " data=%08" PRIx32, millis(), rolling, fixed, data);
        }

        ESP_LOGD(TAG, "cmd=%03x (%s)", cmd, cmd_name(cmd));

        nibble = (data >> 8) & 0xff; 
        byte1 = (data >> 16) & 0xff;
        byte2 = (data >> 24) & 0xff;


        if (cmd == command::STATUS) {
            auto doorState = nibble;
            if (doorState == DoorState::DOOR_STATE_CLOSED && this->doorState != doorState) {
                transmit(command::GET_OPENINGS);
            }

            this->doorState = doorState;
            this->lightState = (byte2 >> 1) & 1;
            this->lockState = byte2 & 1;
            this->motionState = MotionState::MOTION_STATE_CLEAR; // when the status message is read, reset motion state to 0|clear
            this->motorState = MotorState::MOTOR_STATE_OFF; // when the status message is read, reset motor state to 0|off
            this->obstructionState = (byte1 >> 6) & 1;
            // obstruction = (byte1 >> 6) & 1; // unreliable due to the time it takes to register an obstruction
            ESP_LOGD(TAG, "Status cmd: Door: %d Light: %d Lock: %d", this->doorState, this->lightState, this->lockState);

        } 
        else if (cmd == command::LIGHT) {
            ESP_LOGD(TAG, "Light cmd: nibble=%02d byte1=%02d byte2=%02d", nibble, byte1, byte2);
            if (nibble == 0) {
                this->lightState = LightState::LIGHT_STATE_OFF;
            }
            else if (nibble == 1) {
                this->lightState = LightState::LIGHT_STATE_ON;
            }
            else if (nibble == 2) { // toggle
                if (this->lightState == LightState::LIGHT_STATE_ON) {
                    this->lightState = LightState::LIGHT_STATE_OFF;
                } else {
                    this->lightState = LightState::LIGHT_STATE_ON;
                }
            }
            ESP_LOGV(TAG, "Light: %d (toggle)", this->lightState);
        } 
        else if (cmd == command::MOTOR_ON) {
            ESP_LOGD(TAG, "Motor on: nibble=%02d byte1=%02d byte2=%02d", nibble, byte1, byte2);
            this->motorState = MotorState::MOTOR_STATE_ON;
        } 
        else if (cmd == command::OPEN) {
            ESP_LOGD(TAG, "Door open: nibble=%02d byte1=%02d byte2=%02d", nibble, byte1, byte2);
            this->buttonState = (byte1 & 1) == 1 ? ButtonState::BUTTON_STATE_PRESSED : ButtonState::BUTTON_STATE_RELEASED;
            ESP_LOGV(TAG, "Pressed: %d", this->buttonState);
        } 
        else if (cmd == command::OPENINGS) {
            ESP_LOGD(TAG, "Openings: nibble=%02d byte1=%02d byte2=%02d", nibble, byte1, byte2);
            this->openings = (byte1 << 8) | byte2;
            ESP_LOGV(TAG, "Openings: %d", this->openings);
        } 
        else if (cmd == command::MOTION) {
            ESP_LOGD(TAG, "Motion: nibble=%02d byte1=%02d byte2=%02d", nibble, byte1, byte2);
            this->motionState = MotionState::MOTION_STATE_DETECTED;
            if (this->lightState == LightState::LIGHT_STATE_OFF) {
                transmit(command::GET_STATUS);
            }
            ESP_LOGV(TAG, "Motion: %d (toggle)", this->motionState);
        } 
        else {
            // 0x84 -- is it used?
            // ESP_LOGD(TAG, "Unknown command: cmd=%04x nibble=%02d byte1=%02d byte2=%02d", cmd, nibble, byte1, byte2);
            ESP_LOGD(TAG, "Unhandled command: cmd=%04x nibble=%02x byte1=%02x byte2=%02x fixed=%010" PRIx64 " data=%08" PRIx32, cmd, nibble, byte1, byte2, fixed, data);
        }
        return cmd;
    }

    void RATGDOComponent::getRollingCode(command::cmd command, uint32_t data, bool increment)
    {
        uint64_t fixed = ((command & ~0xff) << 24) | this->remote_id;
        uint32_t send_data = (data << 8) | (command & 0xff);

        ESP_LOGD(TAG, "[%ld] Encode for transmit rolling=%07" PRIx32 " fixed=%010" PRIx64 " data=%08" PRIx32, millis(), this->rollingCodeCounter, fixed, send_data);
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

    void RATGDOComponent::sync()
    {
        if (this->rollingCodeCounter == 0) { // first time use
            this->rollingCodeCounter = 1;
            // the opener only sends a reply when the rolling code > previous rolling code for a given remote id
            // when used the first time there is no previous rolling code, so first command is ignored
            set_timeout(30, [=] {
                transmit(command::GET_STATUS);
            });
        }
        set_timeout(100, [=] {
            transmit(command::GET_STATUS);
        });
        set_timeout(200, [=] {
            transmit(command::GET_OPENINGS);
        });
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
    }

    void RATGDOComponent::openDoor()
    {
        doorCommand(data::OPEN);
    }

    void RATGDOComponent::closeDoor()
    {
        doorCommand(data::CLOSE);
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
        doorCommand(data::TOGGLE);
    }

    void RATGDOComponent::doorCommand(uint32_t data)
    {
        data |= (1<<16); // button 1 ?
        data |= (1<<8); // button press
        transmit(command::OPEN, data, false); 
        set_timeout(100, [=] {
            auto data2  = data & ~(1<<8); // button release
            transmit(command::OPEN, data2);
        });
    }


    void RATGDOComponent::lightOn()
    {
        this->lightState = LightState::LIGHT_STATE_ON;
        transmit(command::LIGHT, data::ON);
    }

    void RATGDOComponent::lightOff()
    {
        this->lightState = LightState::LIGHT_STATE_OFF;
        transmit(command::LIGHT, data::OFF);
    }

    void RATGDOComponent::toggleLight()
    {
        if (this->lightState == LightState::LIGHT_STATE_ON) {
            this->lightState = LightState::LIGHT_STATE_OFF;
        } else {
            this->lightState = LightState::LIGHT_STATE_ON;
        }
        transmit(command::LIGHT, data::TOGGLE);
    }

    // Lock functions
    void RATGDOComponent::lock()
    {
        transmit(command::LOCK, data::ON);
    }

    void RATGDOComponent::unlock()
    {
        transmit(command::LOCK, data::OFF);
    }

    void RATGDOComponent::toggleLock()
    {
        if (this->lockState == LockState::LOCK_STATE_LOCKED) {
            this->lockState = LockState::LOCK_STATE_UNLOCKED;
        } else {
            this->lockState = LockState::LOCK_STATE_LOCKED;
        }
        transmit(command::LOCK, data::TOGGLE);
    }

    void RATGDOComponent::saveCounter()
    {
        this->pref_.save(&this->rollingCodeCounter);
        if (!this->lastSyncedRollingCodeCounter || this->rollingCodeCounter > this->lastSyncedRollingCodeCounter) {
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
