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

    void IRAM_ATTR HOT RATGDOStore::isr_obstruction(RATGDOStore* arg)
    {
        if (arg->input_obst.digital_read()) {
            arg->last_obstruction_high = millis();
        } else {
            arg->obstruction_low_count++;
        }
    }

    void RATGDOComponent::setup()
    {
        this->rolling_code_counter_pref_ = global_preferences->make_preference<int>(734874333U);
        uint32_t rolling_code_counter = 0;
        this->rolling_code_counter_pref_.load(&rolling_code_counter);
        this->rolling_code_counter = rolling_code_counter;
        // observers are subscribed in the setup() of children defer notify until after setup()
        defer([=] { this->rolling_code_counter.notify(); });

        this->opening_duration_pref_ = global_preferences->make_preference<float>(734874334U);
        float opening_duration = 0;
        this->opening_duration_pref_.load(&opening_duration);
        this->set_opening_duration(opening_duration);
        defer([=] { this->opening_duration.notify(); });

        this->closing_duration_pref_ = global_preferences->make_preference<float>(734874335U);
        float closing_duration = 0;
        this->closing_duration_pref_.load(&closing_duration);
        this->set_closing_duration(closing_duration);
        defer([=] { this->closing_duration.notify(); });

        this->output_gdo_pin_->setup();
        this->input_gdo_pin_->setup();
        this->input_obst_pin_->setup();

        this->isr_store_.input_obst = this->input_obst_pin_->to_isr();

        this->output_gdo_pin_->pin_mode(gpio::FLAG_OUTPUT);
        this->input_gdo_pin_->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLUP);
        this->input_obst_pin_->pin_mode(gpio::FLAG_INPUT);

        this->sw_serial.begin(9600, SWSERIAL_8N1, this->input_gdo_pin_->get_pin(), this->output_gdo_pin_->get_pin(), true);

        this->input_obst_pin_->attach_interrupt(RATGDOStore::isr_obstruction, &this->isr_store_, gpio::INTERRUPT_ANY_EDGE);

        ESP_LOGV(TAG, "Syncing rolling code counter after reboot...");

        // many things happening at startup, use some delay for sync
        set_timeout(SYNC_DELAY, [=] { this->sync(); });
    }

    void RATGDOComponent::loop()
    {
        obstruction_loop();
        gdo_state_loop();
    }

    void RATGDOComponent::dump_config()
    {
        ESP_LOGCONFIG(TAG, "Setting up RATGDO...");
        LOG_PIN("  Output GDO Pin: ", this->output_gdo_pin_);
        LOG_PIN("  Input GDO Pin: ", this->input_gdo_pin_);
        LOG_PIN("  Input Obstruction Pin: ", this->input_obst_pin_);
        ESP_LOGCONFIG(TAG, "  Rolling Code Counter: %d", *this->rolling_code_counter);
        ESP_LOGCONFIG(TAG, "  Remote ID: %d", this->remote_id_);
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

    uint16_t RATGDOComponent::decode_packet(const WirePacket& packet)
    {
        uint32_t rolling = 0;
        uint64_t fixed = 0;
        uint32_t data = 0;

        decode_wireline(packet, &rolling, &fixed, &data);

        uint16_t cmd = ((fixed >> 24) & 0xf00) | (data & 0xff);
        data &= ~0xf000; // clear parity nibble

        if ((fixed & 0xfff) == this->remote_id_) { // my commands
            ESP_LOGV(TAG, "[%ld] received mine: rolling=%07" PRIx32 " fixed=%010" PRIx64 " data=%08" PRIx32, millis(), rolling, fixed, data);
            return 0;
        } else {
            ESP_LOGV(TAG, "[%ld] received rolling=%07" PRIx32 " fixed=%010" PRIx64 " data=%08" PRIx32, millis(), rolling, fixed, data);
        }

        uint8_t nibble = (data >> 8) & 0xff;
        uint8_t byte1 = (data >> 16) & 0xff;
        uint8_t byte2 = (data >> 24) & 0xff;

        ESP_LOGV(TAG, "cmd=%03x (%s) byte2=%02x byte1=%02x nibble=%01x", cmd, cmd_name(cmd), byte2, byte1, nibble);

        if (cmd == command::STATUS) {

            auto door_state = static_cast<DoorState>(nibble);
            auto prev_door_state = *this->door_state;

            if (door_state == DoorState::DOOR_STATE_OPENING && prev_door_state == DoorState::DOOR_STATE_CLOSED) {
                this->start_opening = millis();
            }
            if (door_state == DoorState::DOOR_STATE_OPEN && prev_door_state == DoorState::DOOR_STATE_OPENING) {
                if (this->start_opening > 0) {
                    auto duration = (millis() - this->start_opening) / 1000;
                    duration = *this->opening_duration > 0 ? (duration + *this->opening_duration) / 2 : duration;
                    this->set_opening_duration(round(duration * 10) / 10);
                }
            }
            if (door_state == DoorState::DOOR_STATE_CLOSING && prev_door_state == DoorState::DOOR_STATE_OPEN) {
                this->start_closing = millis();
            }
            if (door_state == DoorState::DOOR_STATE_CLOSED && prev_door_state == DoorState::DOOR_STATE_CLOSING) {
                if (this->start_closing > 0) {
                    auto duration = (millis() - this->start_closing) / 1000;
                    duration = *this->closing_duration > 0 ? (duration + *this->closing_duration) / 2 : duration;
                    this->set_closing_duration(round(duration * 10) / 10);
                }
            }
            if (door_state == DoorState::DOOR_STATE_STOPPED) {
                this->start_opening = -1;
                this->start_closing = -1;
            }

            if (door_state == DoorState::DOOR_STATE_OPEN) {
                this->door_position = 1.0;
            } else if (door_state == DoorState::DOOR_STATE_CLOSED) {
                this->door_position = 0.0;
            } else {
                if (*this->closing_duration == 0 || *this->opening_duration == 0 || *this->door_position == DOOR_POSITION_UNKNOWN) {
                    this->door_position = 0.5; // best guess
                }
            }

            if (door_state == DoorState::DOOR_STATE_OPENING && !this->moving_to_position) {
                this->position_sync_while_opening(1.0 - *this->door_position);
                this->moving_to_position = true;
            }
            if (door_state == DoorState::DOOR_STATE_CLOSING && !this->moving_to_position) {
                this->position_sync_while_closing(*this->door_position);
                this->moving_to_position = true;
            }

            if (door_state == DoorState::DOOR_STATE_OPEN || door_state == DoorState::DOOR_STATE_CLOSED || door_state == DoorState::DOOR_STATE_STOPPED) {
                this->cancel_position_sync_callbacks();
            }

            this->door_state = door_state;
            this->light_state = static_cast<LightState>((byte2 >> 1) & 1);
            this->lock_state = static_cast<LockState>(byte2 & 1);
            this->motion_state = MotionState::MOTION_STATE_CLEAR; // when the status message is read, reset motion state to 0|clear
            this->motor_state = MotorState::MOTOR_STATE_OFF; // when the status message is read, reset motor state to 0|off
            // this->obstruction_state = static_cast<ObstructionState>((byte1 >> 6) & 1);

            if (door_state == DoorState::DOOR_STATE_CLOSED && door_state != prev_door_state) {
                transmit(command::GET_OPENINGS);
            }

            ESP_LOGD(TAG, "Status: door=%s light=%s lock=%s",
                door_state_to_string(*this->door_state),
                light_state_to_string(*this->light_state),
                lock_state_to_string(*this->lock_state));
        } else if (cmd == command::LIGHT) {
            if (nibble == 0) {
                this->light_state = LightState::LIGHT_STATE_OFF;
            } else if (nibble == 1) {
                this->light_state = LightState::LIGHT_STATE_ON;
            } else if (nibble == 2) { // toggle
                this->light_state = light_state_toggle(*this->light_state);
            }
            ESP_LOGD(TAG, "Light: action=%s state=%s",
                nibble == 0 ? "OFF" : nibble == 1 ? "ON"
                                                  : "TOGGLE",
                light_state_to_string(*this->light_state));
        } else if (cmd == command::MOTOR_ON) {
            this->motor_state = MotorState::MOTOR_STATE_ON;
            ESP_LOGD(TAG, "Motor: state=%s", motor_state_to_string(*this->motor_state));
        } else if (cmd == command::OPEN) {
            this->button_state = (byte1 & 1) == 1 ? ButtonState::BUTTON_STATE_PRESSED : ButtonState::BUTTON_STATE_RELEASED;
            ESP_LOGD(TAG, "Open: button=%s", button_state_to_string(*this->button_state));
        } else if (cmd == command::OPENINGS) {
            this->openings = (byte1 << 8) | byte2;
            ESP_LOGD(TAG, "Openings: %d", *this->openings);
        } else if (cmd == command::MOTION) {
            this->motion_state = MotionState::MOTION_STATE_DETECTED;
            if (*this->light_state == LightState::LIGHT_STATE_OFF) {
                transmit(command::GET_STATUS);
            }
            ESP_LOGD(TAG, "Motion: %s", motion_state_to_string(*this->motion_state));
        } else {
            ESP_LOGV(TAG, "Unhandled command: cmd=%03x nibble=%02x byte1=%02x byte2=%02x fixed=%010" PRIx64 " data=%08" PRIx32, cmd, nibble, byte1, byte2, fixed, data);
        }
        return cmd;
    }

    void RATGDOComponent::encode_packet(command::cmd command, uint32_t data, bool increment, WirePacket& packet)
    {
        uint64_t fixed = ((command & ~0xff) << 24) | this->remote_id_;
        uint32_t send_data = (data << 8) | (command & 0xff);

        ESP_LOGV(TAG, "[%ld] Encode for transmit rolling=%07" PRIx32 " fixed=%010" PRIx64 " data=%08" PRIx32, millis(), *this->rolling_code_counter, fixed, send_data);
        encode_wireline(*this->rolling_code_counter, fixed, send_data, packet);

        print_packet(packet);
        if (increment) {
            increment_rolling_code_counter();
        }
    }

    void RATGDOComponent::set_opening_duration(float duration)
    {
        ESP_LOGD(TAG, "Set opening duration: %.1fs", duration);
        this->opening_duration = duration;
        this->opening_duration_pref_.save(&this->opening_duration);

        if (*this->closing_duration == 0 && duration != 0) {
            this->set_closing_duration(duration);
        }
    }

    void RATGDOComponent::set_closing_duration(float duration)
    {
        ESP_LOGD(TAG, "Set closing duration: %.1fs", duration);
        this->closing_duration = duration;
        this->closing_duration_pref_.save(&this->closing_duration);

        if (*this->opening_duration == 0 && duration != 0) {
            this->set_opening_duration(duration);
        }
    }

    void RATGDOComponent::set_rolling_code_counter(uint32_t counter)
    {
        ESP_LOGV(TAG, "Set rolling code counter to %d", counter);
        this->rolling_code_counter = counter;
        this->rolling_code_counter_pref_.save(&this->rolling_code_counter);
    }

    void RATGDOComponent::increment_rolling_code_counter(int delta)
    {
        this->rolling_code_counter = (*this->rolling_code_counter + delta) & 0xfffffff;
    }

    void RATGDOComponent::print_packet(const WirePacket& packet) const
    {
        ESP_LOGV(TAG, "Counter: %d Send code: [%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X]",
            *this->rollingCodeCounter,
            packet[0],
            packet[1],
            packet[2],
            packet[3],
            packet[4],
            packet[5],
            packet[6],
            packet[7],
            packet[8],
            packet[9],
            packet[10],
            packet[11],
            packet[12],
            packet[13],
            packet[14],
            packet[15],
            packet[16],
            packet[17],
            packet[18]);
    }

    /*************************** OBSTRUCTION DETECTION ***************************/

    void RATGDOComponent::obstruction_loop()
    {
        long current_millis = millis();
        static unsigned long last_millis = 0;

        // the obstruction sensor has 3 states: clear (HIGH with LOW pulse every 7ms), obstructed (HIGH), asleep (LOW)
        // the transitions between awake and asleep are tricky because the voltage drops slowly when falling asleep
        // and is high without pulses when waking up

        // If at least 3 low pulses are counted within 50ms, the door is awake, not obstructed and we don't have to check anything else

        // Every 50ms
        if (current_millis - last_millis > 50) {
            // check to see if we got between 3 and 8 low pulses on the line
            if (this->isr_store_.obstruction_low_count >= 3 && this->isr_store_.obstruction_low_count <= 8) {
                // obstructionCleared();
                this->obstruction_state = ObstructionState::OBSTRUCTION_STATE_CLEAR;

                // if there have been no pulses the line is steady high or low
            } else if (this->isr_store_.obstruction_low_count == 0) {
                // if the line is high and the last high pulse was more than 70ms ago, then there is an obstruction present
                if (this->input_obst_pin_->digital_read() && current_millis - this->isr_store_.last_obstruction_high > 70) {
                    this->obstruction_state = ObstructionState::OBSTRUCTION_STATE_OBSTRUCTED;
                    // obstructionDetected();
                } else {
                    // asleep
                }
            }

            last_millis = current_millis;
            this->isr_store_.obstruction_low_count = 0;
        }
    }

    void RATGDOComponent::gdo_state_loop()
    {
        static bool reading_msg = false;
        static uint32_t msg_start = 0;
        static uint16_t byte_count = 0;
        static WirePacket rx_packet;

        if (!reading_msg) {
            while (this->sw_serial.available()) {
                uint8_t ser_byte = this->sw_serial.read();
                if (ser_byte != 0x55 && ser_byte != 0x01 && ser_byte != 0x00) {
                    byte_count = 0;
                    continue;
                }
                msg_start = ((msg_start << 8) | ser_byte) & 0xffffff;
                byte_count++;

                // if we are at the start of a message, capture the next 16 bytes
                if (msg_start == 0x550100) {
                    rx_packet[0] = 0x55;
                    rx_packet[1] = 0x01;
                    rx_packet[2] = 0x00;

                    reading_msg = true;
                    break;
                }
            }
        }
        if (reading_msg) {
            while (this->sw_serial.available()) {
                uint8_t ser_byte = this->sw_serial.read();
                rx_packet[byte_count] = ser_byte;
                byte_count++;

                if (byte_count == PACKET_LENGTH) {
                    reading_msg = false;
                    byte_count = 0;
                    decode_packet(rx_packet);
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
        WirePacket tx_packet;

        encode_packet(command, data, increment, tx_packet);
        this->output_gdo_pin_->digital_write(true); // pull the line high for 1305 micros so the
                                                    // door opener responds to the message
        delayMicroseconds(1305);
        this->output_gdo_pin_->digital_write(false); // bring the line low

        delayMicroseconds(1260); // "LOW" pulse duration before the message start
        this->sw_serial.write(tx_packet, PACKET_LENGTH);

        save_rolling_code_counter();
    }

    void RATGDOComponent::sync()
    {
        // increment rolling code counter by some amount in case we crashed without writing to flash the latest value
        this->increment_rolling_code_counter(MAX_CODES_WITHOUT_FLASH_WRITE);

        set_retry(
            300, 10, [=](uint8_t r) {
                if (*this->door_state != DoorState::DOOR_STATE_UNKNOWN) { // have status
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

    void RATGDOComponent::open_door()
    {
        if (*this->door_state == DoorState::DOOR_STATE_OPENING) {
            return; // gets ignored by opener
        }
        this->cancel_position_sync_callbacks();

        door_command(data::DOOR_OPEN);
    }

    void RATGDOComponent::close_door()
    {
        if (*this->door_state == DoorState::DOOR_STATE_CLOSING || *this->door_state == DoorState::DOOR_STATE_OPENING) {
            return; // gets ignored by opener
        }
        this->cancel_position_sync_callbacks();

        door_command(data::DOOR_CLOSE);
    }

    void RATGDOComponent::stop_door()
    {
        if (*this->door_state != DoorState::DOOR_STATE_OPENING && *this->door_state != DoorState::DOOR_STATE_CLOSING) {
            ESP_LOGW(TAG, "The door is not moving.");
            return;
        }
        door_command(data::DOOR_STOP);
    }

    void RATGDOComponent::toggle_door()
    {
        if (*this->door_state == DoorState::DOOR_STATE_OPENING) {
            return; // gets ignored by opener
        }
        this->cancel_position_sync_callbacks();

        door_command(data::DOOR_TOGGLE);
    }

    void RATGDOComponent::position_sync_while_opening(float delta, float update_period)
    {
        if (*this->opening_duration == 0) {
            ESP_LOGW(TAG, "I don't know opening duration, ignoring position sync");
            return;
        }
        auto updates = *this->opening_duration * 1000 * delta / update_period;
        auto position_update = delta / updates;
        auto count = int(updates);
        ESP_LOGV(TAG, "[Opening] Position sync %d times: ", count);
        // try to keep position in sync while door is moving
        set_retry("position_sync_while_moving", update_period, count, [=](uint8_t r) {
            ESP_LOGV(TAG, "[Opening] Position sync: %d: ", r);
            this->door_position = *this->door_position + position_update;
            return RetryResult::RETRY;
        });
    }

    void RATGDOComponent::position_sync_while_closing(float delta, float update_period)
    {
        if (*this->closing_duration == 0) {
            ESP_LOGW(TAG, "I don't know closing duration, ignoring position sync");
            return;
        }
        auto updates = *this->closing_duration * 1000 * delta / update_period;
        auto position_update = delta / updates;
        auto count = int(updates);
        ESP_LOGV(TAG, "[Closing] Position sync %d times: ", count);
        // try to keep position in sync while door is moving
        set_retry("position_sync_while_moving", update_period, count, [=](uint8_t r) {
            ESP_LOGV(TAG, "[Closing] Position sync: %d: ", r);
            this->door_position = *this->door_position - position_update;
            return RetryResult::RETRY;
        });
    }

    void RATGDOComponent::door_move_to_position(float position)
    {
        if (*this->door_state == DoorState::DOOR_STATE_OPENING || *this->door_state == DoorState::DOOR_STATE_CLOSING) {
            ESP_LOGW(TAG, "The door is moving, ignoring.");
            return;
        }

        auto delta = position - *this->door_position;
        if (delta == 0) {
            ESP_LOGD(TAG, "Door is already at position %.2f", position);
            return;
        }

        auto duration = delta > 0 ? *this->opening_duration : *this->closing_duration;
        if (duration == 0) {
            ESP_LOGW(TAG, "I don't know duration, ignoring move to position");
            return;
        }

        if (delta > 0) { // open
            door_command(data::DOOR_OPEN);
            this->position_sync_while_opening(delta);
        } else { // close
            delta = -delta;
            door_command(data::DOOR_CLOSE);
            this->position_sync_while_closing(delta);
        }

        auto operation_time = duration * 1000 * delta;
        ESP_LOGD(TAG, "Moving to position %.2f in %.1fs", position, operation_time / 1000.0);
        this->moving_to_position = true;
        set_timeout("move_to_position", operation_time, [=] {
            door_command(data::DOOR_STOP);
            this->moving_to_position = false;
            this->door_position = position;
        });
    }

    void RATGDOComponent::cancel_position_sync_callbacks()
    {
        if (this->moving_to_position) {
            ESP_LOGD(TAG, "Cancelling position callbacks");
            cancel_timeout("move_to_position");
            cancel_retry("position_sync_while_moving");
        }
        moving_to_position = false;
    }

    void RATGDOComponent::door_command(uint32_t data)
    {
        data |= (1 << 16); // button 1 ?
        data |= (1 << 8); // button press
        transmit(command::OPEN, data, false);
        set_timeout(100, [=] {
            auto data2 = data & ~(1 << 8); // button release
            transmit(command::OPEN, data2);
        });
    }

    void RATGDOComponent::light_on()
    {
        this->light_state = LightState::LIGHT_STATE_ON;
        transmit(command::LIGHT, data::LIGHT_ON);
    }

    void RATGDOComponent::light_off()
    {
        this->light_state = LightState::LIGHT_STATE_OFF;
        transmit(command::LIGHT, data::LIGHT_OFF);
    }

    void RATGDOComponent::toggle_light()
    {
        this->light_state = light_state_toggle(*this->light_state);
        transmit(command::LIGHT, data::LIGHT_TOGGLE);
    }

    // Lock functions
    void RATGDOComponent::lock()
    {
        this->lock_state = LockState::LOCK_STATE_LOCKED;
        transmit(command::LOCK, data::LOCK_ON);
    }

    void RATGDOComponent::unlock()
    {
        this->lock_state = LockState::LOCK_STATE_UNLOCKED;
        transmit(command::LOCK, data::LOCK_OFF);
    }

    void RATGDOComponent::toggle_lock()
    {
        this->lock_state = lock_state_toggle(*this->lock_state);
        transmit(command::LOCK, data::LOCK_TOGGLE);
    }

    void RATGDOComponent::save_rolling_code_counter()
    {
        this->rolling_code_counter_pref_.save(&this->rolling_code_counter);
        // Forcing a sync results in a soft reset if there are too many
        // writes to flash in a short period of time. To avoid this,
        // we have configured preferences to write every 5s
    }

    LightState RATGDOComponent::get_light_state() const
    {
        return *this->light_state;
    }

    void RATGDOComponent::subscribe_rolling_code_counter(std::function<void(uint32_t)>&& f)
    {
        // change update to children is defered until after component loop
        // if multiple changes occur during component loop, only the last one is notified
        this->rolling_code_counter.subscribe([=](uint32_t state) { defer("rolling_code_counter", [=] { f(state); }); });
    }
    void RATGDOComponent::subscribe_opening_duration(std::function<void(float)>&& f)
    {
        this->opening_duration.subscribe([=](float state) { defer("opening_duration", [=] { f(state); }); });
    }
    void RATGDOComponent::subscribe_closing_duration(std::function<void(float)>&& f)
    {
        this->closing_duration.subscribe([=](float state) { defer("closing_duration", [=] { f(state); }); });
    }
    void RATGDOComponent::subscribe_openings(std::function<void(uint16_t)>&& f)
    {
        this->openings.subscribe([=](uint16_t state) { defer("openings", [=] { f(state); }); });
    }
    void RATGDOComponent::subscribe_door_state(std::function<void(DoorState, float)>&& f)
    {
        this->door_state.subscribe([=](DoorState state) {
            defer("door_state", [=] { f(state, *this->door_position); });
        });
        this->door_position.subscribe([=](float position) {
            defer("door_state", [=] { f(*this->door_state, position); });
        });
    }
    void RATGDOComponent::subscribe_light_state(std::function<void(LightState)>&& f)
    {
        this->light_state.subscribe([=](LightState state) { defer("light_state", [=] { f(state); }); });
    }
    void RATGDOComponent::subscribe_lock_state(std::function<void(LockState)>&& f)
    {
        this->lock_state.subscribe([=](LockState state) { defer("lock_state", [=] { f(state); }); });
    }
    void RATGDOComponent::subscribe_obstruction_state(std::function<void(ObstructionState)>&& f)
    {
        this->obstruction_state.subscribe([=](ObstructionState state) { defer("obstruction_state", [=] { f(state); }); });
    }
    void RATGDOComponent::subscribe_motor_state(std::function<void(MotorState)>&& f)
    {
        this->motor_state.subscribe([=](MotorState state) { defer("motor_state", [=] { f(state); }); });
    }
    void RATGDOComponent::subscribe_button_state(std::function<void(ButtonState)>&& f)
    {
        this->button_state.subscribe([=](ButtonState state) { defer("button_state", [=] { f(state); }); });
    }
    void RATGDOComponent::subscribe_motion_state(std::function<void(MotionState)>&& f)
    {
        this->motion_state.subscribe([=](MotionState state) { defer("motion_state", [=] { f(state); }); });
    }

} // namespace ratgdo
} // namespace esphome
