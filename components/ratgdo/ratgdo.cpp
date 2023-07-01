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
#include "ratgdo_state.h"

#include "esphome/core/log.h"

namespace esphome {
namespace ratgdo {

    static const char* const TAG = "ratgdo";
    static const int SYNC_DELAY = 1000;
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
    static const uint8_t MAX_CODES_WITHOUT_FLASH_WRITE = 10;

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
        this->rollingCodePref_ = global_preferences->make_preference<int>(734874333U);
        uint32_t rolling_code_counter = 0;
        this->rollingCodePref_.load(&rolling_code_counter);
        this->rollingCodeCounter = rolling_code_counter;
        // observers are subscribed in the setup() of children defer notify until after setup()
        defer([=] { this->rollingCodeCounter.notify(); });

        this->openingDurationPref_ = global_preferences->make_preference<float>(734874334U);
        float opening_duration = 0;
        this->openingDurationPref_.load(&opening_duration);
        this->setOpeningDuration(opening_duration);
        defer([=] { this->openingDuration.notify(); });

        this->closingDurationPref_ = global_preferences->make_preference<float>(734874335U);
        float closing_duration = 0;
        this->closingDurationPref_.load(&closing_duration);
        this->setClosingDuration(closing_duration);
        defer([=] { this->closingDuration.notify(); });

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
        set_timeout(SYNC_DELAY, [=] { this->sync(); });
    }

    void RATGDOComponent::loop()
    {
        obstructionLoop();
        gdoStateLoop();
    }

    void RATGDOComponent::dump_config()
    {
        ESP_LOGCONFIG(TAG, "Setting up RATGDO...");
        LOG_PIN("  Output GDO Pin: ", this->output_gdo_pin_);
        LOG_PIN("  Input GDO Pin: ", this->input_gdo_pin_);
        LOG_PIN("  Input Obstruction Pin: ", this->input_obst_pin_);
        ESP_LOGCONFIG(TAG, "  Rolling Code Counter: %d", *this->rollingCodeCounter);
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
        case 0x40a:
            return "ttc"; // Time to close
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

            if (doorState == DoorState::DOOR_STATE_OPENING && *this->doorState == DoorState::DOOR_STATE_CLOSED) {
                this->startOpening = millis();
            }
            if (doorState == DoorState::DOOR_STATE_OPEN && *this->doorState == DoorState::DOOR_STATE_OPENING) {
                if (this->startOpening > 0) {
                    auto duration = (millis() - this->startOpening) / 1000;
                    duration = *this->openingDuration > 0 ? (duration + *this->openingDuration) / 2 : duration;
                    this->setOpeningDuration(round(duration * 10) / 10);
                }
            }
            if (doorState == DoorState::DOOR_STATE_CLOSING && *this->doorState == DoorState::DOOR_STATE_OPEN) {
                this->startClosing = millis();
            }
            if (doorState == DoorState::DOOR_STATE_CLOSED && *this->doorState == DoorState::DOOR_STATE_CLOSING) {
                if (this->startClosing > 0) {
                    auto duration = (millis() - this->startClosing) / 1000;
                    duration = *this->closingDuration > 0 ? (duration + *this->closingDuration) / 2 : duration;
                    this->setClosingDuration(round(duration * 10) / 10);
                }
            }
            if (doorState == DoorState::DOOR_STATE_STOPPED) {
                this->startOpening = -1;
                this->startClosing = -1;
            }

            if (doorState == DoorState::DOOR_STATE_OPEN) {
                this->doorPosition = 1.0;
            } else if (doorState == DoorState::DOOR_STATE_CLOSED) {
                this->doorPosition = 0.0;
            } else {
                if (*this->closingDuration == 0 || *this->openingDuration == 0 || *this->doorPosition == DOOR_POSITION_UNKNOWN) {
                    this->doorPosition = 0.5; // best guess
                }
            }

            if (doorState == DoorState::DOOR_STATE_OPENING && !this->movingToPosition) {
                this->positionSyncWhileOpening(1.0 - *this->doorPosition);
                this->movingToPosition = true;
            }
            if (doorState == DoorState::DOOR_STATE_CLOSING && !this->movingToPosition) {
                this->positionSyncWhileClosing(*this->doorPosition);
                this->movingToPosition = true;
            }

            if (doorState == DoorState::DOOR_STATE_OPEN || doorState == DoorState::DOOR_STATE_CLOSED || doorState == DoorState::DOOR_STATE_STOPPED) {
                this->cancelPositionSyncCallbacks();
            }

            this->lightState = static_cast<LightState>((byte2 >> 1) & 1);
            this->lockState = static_cast<LockState>(byte2 & 1);
            this->motionState = MotionState::MOTION_STATE_CLEAR; // when the status message is read, reset motion state to 0|clear
            this->motorState = MotorState::MOTOR_STATE_OFF; // when the status message is read, reset motor state to 0|off
            // this->obstructionState = static_cast<ObstructionState>((byte1 >> 6) & 1);

            if (doorState == DoorState::DOOR_STATE_CLOSED && doorState != *this->doorState) {
                transmit(command::GET_OPENINGS);
            }

            this->doorState = doorState;

            ESP_LOGD(TAG, "Status: door=%s light=%s lock=%s",
                door_state_to_string(*this->doorState),
                light_state_to_string(*this->lightState),
                lock_state_to_string(*this->lockState));
        } else if (cmd == command::LIGHT) {
            if (nibble == 0) {
                this->lightState = LightState::LIGHT_STATE_OFF;
            } else if (nibble == 1) {
                this->lightState = LightState::LIGHT_STATE_ON;
            } else if (nibble == 2) { // toggle
                this->lightState = light_state_toggle(*this->lightState);
            }
            ESP_LOGD(TAG, "Light: action=%s state=%s",
                nibble == 0 ? "OFF" : nibble == 1 ? "ON"
                                                  : "TOGGLE",
                light_state_to_string(*this->lightState));
        } else if (cmd == command::MOTOR_ON) {
            this->motorState = MotorState::MOTOR_STATE_ON;
            ESP_LOGD(TAG, "Motor: state=%s", motor_state_to_string(*this->motorState));
        } else if (cmd == command::OPEN) {
            this->buttonState = (byte1 & 1) == 1 ? ButtonState::BUTTON_STATE_PRESSED : ButtonState::BUTTON_STATE_RELEASED;
            ESP_LOGD(TAG, "Open: button=%s", button_state_to_string(*this->buttonState));
        } else if (cmd == command::OPENINGS) {
            this->openings = (byte1 << 8) | byte2;
            ESP_LOGD(TAG, "Openings: %d", *this->openings);
        } else if (cmd == command::MOTION) {
            this->motionState = MotionState::MOTION_STATE_DETECTED;
            if (*this->lightState == LightState::LIGHT_STATE_OFF) {
                transmit(command::GET_STATUS);
            }
            ESP_LOGD(TAG, "Motion: %s", motion_state_to_string(*this->motionState));
        } else {
            ESP_LOGD(TAG, "Unhandled command: cmd=%03x nibble=%02x byte1=%02x byte2=%02x fixed=%010" PRIx64 " data=%08" PRIx32, cmd, nibble, byte1, byte2, fixed, data);
        }
        return cmd;
    }

    void RATGDOComponent::getRollingCode(command::cmd command, uint32_t data, bool increment)
    {
        uint64_t fixed = ((command & ~0xff) << 24) | this->remote_id;
        uint32_t send_data = (data << 8) | (command & 0xff);

        ESP_LOGV(TAG, "[%ld] Encode for transmit rolling=%07" PRIx32 " fixed=%010" PRIx64 " data=%08" PRIx32, millis(), *this->rollingCodeCounter, fixed, send_data);
        encode_wireline(*this->rollingCodeCounter, fixed, send_data, this->txRollingCode);

        printRollingCode();
        if (increment) {
            incrementRollingCodeCounter();
        }
    }

    void RATGDOComponent::setOpeningDuration(float duration)
    {
        ESP_LOGD(TAG, "Set opening duration: %.1fs", duration);
        this->openingDuration = duration;
        this->openingDurationPref_.save(&this->openingDuration);

        if (*this->closingDuration == 0 && duration != 0) {
            this->setClosingDuration(duration);
        }
    }

    void RATGDOComponent::setClosingDuration(float duration)
    {
        ESP_LOGD(TAG, "Set closing duration: %.1fs", duration);
        this->closingDuration = duration;
        this->closingDurationPref_.save(&this->closingDuration);

        if (*this->openingDuration == 0 && duration != 0) {
            this->setOpeningDuration(duration);
        }
    }

    void RATGDOComponent::setRollingCodeCounter(uint32_t counter)
    {
        ESP_LOGV(TAG, "Set rolling code counter to %d", counter);
        this->rollingCodeCounter = counter;
        this->rollingCodePref_.save(&this->rollingCodeCounter);
    }

    void RATGDOComponent::incrementRollingCodeCounter(int delta)
    {
        this->rollingCodeCounter = (*this->rollingCodeCounter + delta) & 0xfffffff;
    }

    void RATGDOComponent::printRollingCode()
    {
        ESP_LOGV(TAG, "Counter: %d Send code: [%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X]",
            *this->rollingCodeCounter,
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
                    readRollingCode();
                    return;
                }
            }
        }
    }

    void RATGDOComponent::query_status()
    {
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
        // increment rolling code counter by some amount in case we crashed without writing to flash the latest value
        this->incrementRollingCodeCounter(MAX_CODES_WITHOUT_FLASH_WRITE);

        set_retry(
            300, 10, [=](uint8_t r) {
                if (*this->doorState != DoorState::DOOR_STATE_UNKNOWN) { // have status
                    if (*this->openings != 0) { // have openings
                        return RetryResult::DONE;
                    } else {
                        transmit(command::GET_OPENINGS);
                        return RetryResult::RETRY;
                    }
                } else {
                    transmit(command::GET_STATUS);
                    return RetryResult::RETRY;
                }
            },
            1.5f);
    }

    void RATGDOComponent::openDoor()
    {
        if (*this->doorState == DoorState::DOOR_STATE_OPENING) {
            return; // gets ignored by opener
        }
        this->cancelPositionSyncCallbacks();

        doorCommand(data::DOOR_OPEN);
    }

    void RATGDOComponent::closeDoor()
    {
        if (*this->doorState == DoorState::DOOR_STATE_CLOSING || *this->doorState == DoorState::DOOR_STATE_OPENING) {
            return; // gets ignored by opener
        }
        this->cancelPositionSyncCallbacks();

        doorCommand(data::DOOR_CLOSE);
    }

    void RATGDOComponent::stopDoor()
    {
        if (*this->doorState != DoorState::DOOR_STATE_OPENING && *this->doorState != DoorState::DOOR_STATE_CLOSING) {
            ESP_LOGW(TAG, "The door is not moving.");
            return;
        }
        doorCommand(data::DOOR_STOP);
    }

    void RATGDOComponent::toggleDoor()
    {
        if (*this->doorState == DoorState::DOOR_STATE_OPENING) {
            return; // gets ignored by opener
        }
        this->cancelPositionSyncCallbacks();

        doorCommand(data::DOOR_TOGGLE);
    }

    void RATGDOComponent::positionSyncWhileOpening(float delta, float update_period)
    {
        if (*this->openingDuration == 0) {
            ESP_LOGW(TAG, "I don't know opening duration, ignoring position sync");
            return;
        }
        auto updates = *this->openingDuration * 1000 * delta / update_period;
        auto position_update = delta / updates;
        auto count = int(updates);
        ESP_LOGD(TAG, "[Opening] Position sync %d times: ", count);
        // try to keep position in sync while door is moving
        set_retry("position_sync_while_moving", update_period, count, [=](uint8_t r) {
            ESP_LOGD(TAG, "[Opening] Position sync: %d: ", r);
            this->doorPosition = *this->doorPosition + position_update;
            return RetryResult::RETRY;
        });
    }

    void RATGDOComponent::positionSyncWhileClosing(float delta, float update_period)
    {
        if (*this->closingDuration == 0) {
            ESP_LOGW(TAG, "I don't know closing duration, ignoring position sync");
            return;
        }
        auto updates = *this->closingDuration * 1000 * delta / update_period;
        auto position_update = delta / updates;
        auto count = int(updates);
        ESP_LOGD(TAG, "[Closing] Position sync %d times: ", count);
        // try to keep position in sync while door is moving
        set_retry("position_sync_while_moving", update_period, count, [=](uint8_t r) {
            ESP_LOGD(TAG, "[Closing] Position sync: %d: ", r);
            this->doorPosition = *this->doorPosition - position_update;
            return RetryResult::RETRY;
        });
    }

    void RATGDOComponent::setDoorPosition(float position)
    {
        if (*this->doorState == DoorState::DOOR_STATE_OPENING || *this->doorState == DoorState::DOOR_STATE_CLOSING) {
            ESP_LOGW(TAG, "The door is moving, ignoring.");
            return;
        }

        auto delta = position - *this->doorPosition;
        if (delta == 0) {
            ESP_LOGD(TAG, "Door is already at position %.2f", position);
            return;
        }

        auto duration = delta > 0 ? *this->openingDuration : *this->closingDuration;
        if (duration == 0) {
            ESP_LOGW(TAG, "I don't know duration, ignoring move to position");
            return;
        }

        if (delta > 0) { // open
            doorCommand(data::DOOR_OPEN);
            this->positionSyncWhileOpening(delta);
        } else { // close
            delta = -delta;
            doorCommand(data::DOOR_CLOSE);
            this->positionSyncWhileClosing(delta);
        }

        auto operation_time = duration * 1000 * delta;
        ESP_LOGD(TAG, "Moving to position %.2f in %.1fs", position, operation_time / 1000.0);
        this->movingToPosition = true;
        set_timeout("move_to_position", operation_time, [=] {
            doorCommand(data::DOOR_STOP);
            this->movingToPosition = false;
            this->doorPosition = position;
        });
    }

    void RATGDOComponent::cancelPositionSyncCallbacks()
    {
        if (this->movingToPosition) {
            ESP_LOGD(TAG, "Cancelling position callbacks");
            cancel_timeout("move_to_position");
            cancel_retry("position_sync_while_moving");
        }
        movingToPosition = false;
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
        this->lightState = light_state_toggle(*this->lightState);
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
        this->lockState = lock_state_toggle(*this->lockState);
        transmit(command::LOCK, data::LOCK_TOGGLE);
    }

    void RATGDOComponent::saveCounter()
    {
        this->rollingCodePref_.save(&this->rollingCodeCounter);
        // Forcing a sync results in a soft reset if there are too many
        // writes to flash in a short period of time. To avoid this,
        // we have configured preferences to write every 5s
    }

    LightState RATGDOComponent::getLightState()
    {
        return *this->lightState;
    }

    void RATGDOComponent::subscribe_rolling_code_counter(std::function<void(uint32_t)>&& f)
    {
        // change update to children is defered until after component loop
        // if multiple changes occur during component loop, only the last one is notified
        this->rollingCodeCounter.subscribe([=](uint32_t state) { defer("rolling_code_counter", [=] { f(state); }); });
    }
    void RATGDOComponent::subscribe_opening_duration(std::function<void(float)>&& f)
    {
        this->openingDuration.subscribe([=](float state) { defer("opening_duration", [=] { f(state); }); });
    }
    void RATGDOComponent::subscribe_closing_duration(std::function<void(float)>&& f)
    {
        this->closingDuration.subscribe([=](float state) { defer("closing_duration", [=] { f(state); }); });
    }
    void RATGDOComponent::subscribe_openings(std::function<void(uint16_t)>&& f)
    {
        this->openings.subscribe([=](uint16_t state) { defer("openings", [=] { f(state); }); });
    }
    void RATGDOComponent::subscribe_door_state(std::function<void(DoorState, float)>&& f)
    {
        this->doorState.subscribe([=](DoorState state) {
            defer("door_state", [=] { f(state, *this->doorPosition); });
        });
        this->doorPosition.subscribe([=](float position) {
            defer("door_state", [=] { f(*this->doorState, position); });
        });
    }
    void RATGDOComponent::subscribe_light_state(std::function<void(LightState)>&& f)
    {
        this->lightState.subscribe([=](LightState state) { defer("light_state", [=] { f(state); }); });
    }
    void RATGDOComponent::subscribe_lock_state(std::function<void(LockState)>&& f)
    {
        this->lockState.subscribe([=](LockState state) { defer("lock_state", [=] { f(state); }); });
    }
    void RATGDOComponent::subscribe_obstruction_state(std::function<void(ObstructionState)>&& f)
    {
        this->obstructionState.subscribe([=](ObstructionState state) { defer("obstruction_state", [=] { f(state); }); });
    }
    void RATGDOComponent::subscribe_motor_state(std::function<void(MotorState)>&& f)
    {
        this->motorState.subscribe([=](MotorState state) { defer("motor_state", [=] { f(state); }); });
    }
    void RATGDOComponent::subscribe_button_state(std::function<void(ButtonState)>&& f)
    {
        this->buttonState.subscribe([=](ButtonState state) { defer("button_state", [=] { f(state); }); });
    }
    void RATGDOComponent::subscribe_motion_state(std::function<void(MotionState)>&& f)
    {
        this->motionState.subscribe([=](MotionState state) { defer("motion_state", [=] { f(state); }); });
    }

} // namespace ratgdo
} // namespace esphome
