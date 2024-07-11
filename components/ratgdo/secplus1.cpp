
#include "secplus1.h"
#include "ratgdo.h"

#include "esphome/core/gpio.h"
#include "esphome/core/log.h"
#include "esphome/core/scheduler.h"

namespace esphome {
namespace ratgdo {
    namespace secplus1 {

        static const char* const TAG = "ratgdo_secplus1";

        void Secplus1::setup(RATGDOComponent* ratgdo, Scheduler* scheduler, InternalGPIOPin* rx_pin, InternalGPIOPin* tx_pin)
        {
            this->ratgdo_ = ratgdo;
            this->scheduler_ = scheduler;
            this->tx_pin_ = tx_pin;
            this->rx_pin_ = rx_pin;

            this->sw_serial_.begin(1200, SWSERIAL_8E1, rx_pin->get_pin(), tx_pin->get_pin(), true);

            this->traits_.set_features(HAS_DOOR_STATUS | HAS_LIGHT_TOGGLE | HAS_LOCK_TOGGLE);
        }

        void Secplus1::loop()
        {
            auto rx_cmd = this->read_command();
            if (rx_cmd) {
                this->handle_command(rx_cmd.value());
            }
            auto tx_cmd = this->pending_tx();
            if (
                (millis() - this->last_tx_) > 200 && // don't send twice in a period
                (millis() - this->last_rx_) > 50 && // time to send it
                tx_cmd && // have pending command
                !(this->is_0x37_panel_ && tx_cmd.value() == CommandType::TOGGLE_LOCK_PRESS) && this->wall_panel_emulation_state_ != WallPanelEmulationState::ENABLED) {
                this->do_transmit_if_pending();
            }
        }

        void Secplus1::dump_config()
        {
            ESP_LOGCONFIG(TAG, "  Protocol: SEC+ v1");
        }

        void Secplus1::sync()
        {
            this->wall_panel_emulation_start_ = millis();
            this->door_state = DoorState::UNKNOWN;
            this->light_state = LightState::UNKNOWN;
            this->scheduler_->cancel_timeout(this->ratgdo_, "wall_panel_emulation");
            this->wall_panel_emulation();

            this->scheduler_->set_timeout(this->ratgdo_, "", 45000, [=] {
                if (this->door_state == DoorState::UNKNOWN) {
                    ESP_LOGW(TAG, "Triggering sync failed actions.");
                    this->ratgdo_->sync_failed = true;
                }
            });
        }

        void Secplus1::set_enable_emulation_mode(bool state)
        {
            this->wall_panel_emulation_state_ = state ? WallPanelEmulationState::ENABLED : WallPanelEmulationState::DISABLED;
            this->wall_panel_emulation();
        }

        void Secplus1::wall_panel_emulation(size_t index)
        {
            if (this->wall_panel_emulation_state_ == WallPanelEmulationState::DISABLED){
                ESP_LOGD(TAG, "Emulation mode is disabled");
            }  else if (this->wall_panel_emulation_state_ == WallPanelEmulationState::ENABLED) {
                // ESP_LOG2(TAG, "[Wall panel emulation] Sending byte: [%02X]", secplus1_states[index]);

                if (index < 15 || !this->do_transmit_if_pending()) {
                    this->transmit_byte(secplus1_states[index]);
                    // gdo response simulation for testing
                    // auto resp = secplus1_states[index] == 0x39 ? 0x00 :
                    //             secplus1_states[index] == 0x3A ? 0x5C :
                    //             secplus1_states[index] == 0x38 ? 0x52 : 0xFF;
                    // if (resp != 0xFF) {
                    //     this->transmit_byte(resp, true);
                    // }

                    index += 1;
                    if (index == 18) {
                        index = 15;
                    }
                }
                this->scheduler_->set_timeout(this->ratgdo_, "wall_panel_emulation", 250, [=] {
                    this->wall_panel_emulation(index);
                });
            }
        }

        void Secplus1::light_action(LightAction action)
        {
            ESP_LOG1(TAG, "Light action: %s", LightAction_to_string(action));
            if (action == LightAction::UNKNOWN) {
                return;
            }
            if (
                action == LightAction::TOGGLE || (action == LightAction::ON && this->light_state == LightState::OFF) || (action == LightAction::OFF && this->light_state == LightState::ON)) {
                this->toggle_light();
            }
        }

        void Secplus1::lock_action(LockAction action)
        {
            ESP_LOG1(TAG, "Lock action: %s", LockAction_to_string(action));
            if (action == LockAction::UNKNOWN) {
                return;
            }
            if (
                action == LockAction::TOGGLE || (action == LockAction::LOCK && this->lock_state == LockState::UNLOCKED) || (action == LockAction::UNLOCK && this->lock_state == LockState::LOCKED)) {
                this->toggle_lock();
            }
        }

        void Secplus1::door_action(DoorAction action)
        {
            ESP_LOG1(TAG, "Door action: %s, door state: %s", DoorAction_to_string(action), DoorState_to_string(this->door_state));
            if (action == DoorAction::UNKNOWN) {
                return;
            }

            const uint32_t double_toggle_delay = 1000;
            if (action == DoorAction::TOGGLE) {
                this->toggle_door();
            } else if (action == DoorAction::OPEN) {
                if (this->door_state == DoorState::CLOSED || this->door_state == DoorState::CLOSING) {
                    this->toggle_door();
                } else if (this->door_state == DoorState::STOPPED) {
                    this->toggle_door(); // this starts closing door
                    this->on_door_state_([=](DoorState s) {
                        if (s == DoorState::CLOSING) {
                            // this changes direction of the door on some openers, on others it stops it
                            this->toggle_door();
                            this->on_door_state_([=](DoorState s) {
                                if (s == DoorState::STOPPED) {
                                    this->toggle_door();
                                }
                            });
                        }
                    });
                }
            } else if (action == DoorAction::CLOSE) {
                if (this->door_state == DoorState::OPEN) {
                    this->toggle_door();
                } else if (this->door_state == DoorState::OPENING) {
                    this->toggle_door(); // this switches to stopped
                    // another toggle needed to close
                    this->on_door_state_([=](DoorState s) {
                        if (s == DoorState::STOPPED) {
                            this->toggle_door();
                        }
                    });
                } else if (this->door_state == DoorState::STOPPED) {
                    this->toggle_door();
                }
            } else if (action == DoorAction::STOP) {
                if (this->door_state == DoorState::OPENING) {
                    this->toggle_door();
                } else if (this->door_state == DoorState::CLOSING) {
                    this->toggle_door(); // this switches to opening

                    // another toggle needed to stop
                    this->on_door_state_([=](DoorState s) {
                        if (s == DoorState::OPENING) {
                            this->toggle_door();
                        }
                    });
                }
            }
        }

        void Secplus1::toggle_light()
        {
            this->enqueue_transmit(CommandType::TOGGLE_LIGHT_PRESS);
        }

        void Secplus1::toggle_lock()
        {
            this->enqueue_transmit(CommandType::TOGGLE_LOCK_PRESS);
        }

        void Secplus1::toggle_door()
        {
            this->enqueue_transmit(CommandType::TOGGLE_DOOR_PRESS);
            this->enqueue_transmit(CommandType::QUERY_DOOR_STATUS);
            if (this->door_state == DoorState::STOPPED || this->door_state == DoorState::OPEN || this->door_state == DoorState::CLOSED) {
                this->door_moving_ = true;
            }
        }

        Result Secplus1::call(Args args)
        {
            using Tag = Args::Tag;
            if (args.tag == Tag::set_enable_emulation_mode) {
                this->set_enable_emulation_mode(args.value.set_enable_emulation_mode.enable_emulation_mode);
            }
            return {};
        }

        optional<RxCommand> Secplus1::read_command()
        {
            static bool reading_msg = false;
            static uint32_t msg_start = 0;
            static uint16_t byte_count = 0;
            static RxPacket rx_packet;

            if (!reading_msg) {
                while (this->sw_serial_.available()) {
                    uint8_t ser_byte = this->sw_serial_.read();
                    this->last_rx_ = millis();

                    if (ser_byte < 0x30 || ser_byte > 0x3A) {
                        ESP_LOG2(TAG, "[%d] Ignoring byte [%02X], baud: %d", millis(), ser_byte, this->sw_serial_.baudRate());
                        byte_count = 0;
                        continue;
                    }
                    rx_packet[byte_count++] = ser_byte;
                    ESP_LOG2(TAG, "[%d] Received byte: [%02X]", millis(), ser_byte);
                    reading_msg = true;

                    if (ser_byte == 0x37 || (ser_byte >= 0x30 && ser_byte <= 0x35)) {
                        rx_packet[byte_count++] = 0;
                        reading_msg = false;
                        byte_count = 0;
                        ESP_LOG2(TAG, "[%d] Received command: [%02X]", millis(), rx_packet[0]);
                        return this->decode_packet(rx_packet);
                    }

                    break;
                }
            }
            if (reading_msg) {
                while (this->sw_serial_.available()) {
                    uint8_t ser_byte = this->sw_serial_.read();
                    this->last_rx_ = millis();
                    rx_packet[byte_count++] = ser_byte;
                    ESP_LOG2(TAG, "[%d] Received byte: [%02X]", millis(), ser_byte);

                    if (byte_count == RX_LENGTH) {
                        reading_msg = false;
                        byte_count = 0;
                        this->print_rx_packet(rx_packet);
                        return this->decode_packet(rx_packet);
                    }
                }

                if (millis() - this->last_rx_ > 100) {
                    // if we have a partial packet and it's been over 100ms since last byte was read,
                    // the rest is not coming (a full packet should be received in ~20ms),
                    // discard it so we can read the following packet correctly
                    ESP_LOGW(TAG, "[%d] Discard incomplete packet: [%02X ...]", millis(), rx_packet[0]);
                    reading_msg = false;
                    byte_count = 0;
                }
            }

            return {};
        }

        void Secplus1::print_rx_packet(const RxPacket& packet) const
        {
            ESP_LOG2(TAG, "[%d] Received packet: [%02X %02X]", millis(), packet[0], packet[1]);
        }

        void Secplus1::print_tx_packet(const TxPacket& packet) const
        {
            ESP_LOG2(TAG, "[%d] Sending packet: [%02X %02X]", millis(), packet[0], packet[1]);
        }

        optional<RxCommand> Secplus1::decode_packet(const RxPacket& packet) const
        {
            CommandType cmd_type = to_CommandType(packet[0], CommandType::UNKNOWN);
            return RxCommand { cmd_type, packet[1] };
        }

        // unknown meaning of observed command-responses:
        // 40 00 and 40 80
        // 53 01
        // C0 3F
        // F8 3F
        // FE 3F

        void Secplus1::handle_command(const RxCommand& cmd)
        {
            if (cmd.req == CommandType::QUERY_DOOR_STATUS) {

                DoorState door_state;
                auto val = cmd.resp & 0x7;
                // 000 0x0 stopped
                // 001 0x1 opening
                // 010 0x2 open
                // 100 0x4 closing
                // 101 0x5 closed
                // 110 0x6 stopped

                if (val == 0x2) {
                    door_state = DoorState::OPEN;
                } else if (val == 0x5) {
                    door_state = DoorState::CLOSED;
                } else if (val == 0x0 || val == 0x6) {
                    door_state = DoorState::STOPPED;
                } else if (val == 0x1) {
                    door_state = DoorState::OPENING;
                } else if (val == 0x4) {
                    door_state = DoorState::CLOSING;
                } else {
                    door_state = DoorState::UNKNOWN;
                }

                if (this->maybe_door_state != door_state) {
                    this->on_door_state_.trigger(door_state);
                }

                if (!this->is_0x37_panel_ && door_state != this->maybe_door_state) {
                    this->maybe_door_state = door_state;
                    ESP_LOG1(TAG, "Door maybe %s, waiting for 2nd status message to confirm", DoorState_to_string(door_state));
                } else {
                    this->maybe_door_state = door_state;
                    this->door_state = door_state;
                    if (this->door_state == DoorState::STOPPED || this->door_state == DoorState::OPEN || this->door_state == DoorState::CLOSED) {
                        this->door_moving_ = false;
                    }
                    this->ratgdo_->received(door_state);
                }
            } else if (cmd.req == CommandType::QUERY_DOOR_STATUS_0x37) {
                this->is_0x37_panel_ = true;
                auto cmd = this->pending_tx();
                if (cmd && cmd.value() == CommandType::TOGGLE_LOCK_PRESS) {
                    this->do_transmit_if_pending();
                } else {
                    // inject door status request
                    if (door_moving_ || (millis() - this->last_status_query_ > 10000)) {
                        this->transmit_byte(static_cast<uint8_t>(CommandType::QUERY_DOOR_STATUS));
                        this->last_status_query_ = millis();
                    }
                }
            } else if (cmd.req == CommandType::QUERY_OTHER_STATUS) {
                LightState light_state = to_LightState((cmd.resp >> 2) & 1, LightState::UNKNOWN);

                if (!this->is_0x37_panel_ && light_state != this->maybe_light_state) {
                    this->maybe_light_state = light_state;
                } else {
                    this->light_state = light_state;
                    this->ratgdo_->received(light_state);
                }

                LockState lock_state = to_LockState((~cmd.resp >> 3) & 1, LockState::UNKNOWN);
                if (!this->is_0x37_panel_ && lock_state != this->maybe_lock_state) {
                    this->maybe_lock_state = lock_state;
                } else {
                    this->lock_state = lock_state;
                    this->ratgdo_->received(lock_state);
                }
            } else if (cmd.req == CommandType::OBSTRUCTION) {
                ObstructionState obstruction_state = cmd.resp == 0 ? ObstructionState::CLEAR : ObstructionState::OBSTRUCTED;
                this->ratgdo_->received(obstruction_state);
            } else if (cmd.req == CommandType::TOGGLE_DOOR_RELEASE) {
                if (cmd.resp == 0x31) {
                    this->wall_panel_starting_ = true;
                }
            } else if (cmd.req == CommandType::TOGGLE_LIGHT_PRESS) {
                // motion was detected, or the light toggle button was pressed
                // either way it's ok to trigger motion detection
                if (this->light_state == LightState::OFF) {
                    this->ratgdo_->received(MotionState::DETECTED);
                }
            } else if (cmd.req == CommandType::TOGGLE_DOOR_PRESS) {
                this->ratgdo_->received(ButtonState::PRESSED);
            } else if (cmd.req == CommandType::TOGGLE_DOOR_RELEASE) {
                this->ratgdo_->received(ButtonState::RELEASED);
            }
        }

        bool Secplus1::do_transmit_if_pending()
        {
            auto cmd = this->pop_pending_tx();
            if (cmd) {
                this->enqueue_command_pair(cmd.value());
                this->transmit_byte(static_cast<uint32_t>(cmd.value()));
            }
            return cmd;
        }

        void Secplus1::enqueue_command_pair(CommandType cmd)
        {
            auto now = millis();
            if (cmd == CommandType::TOGGLE_DOOR_PRESS) {
                this->enqueue_transmit(CommandType::TOGGLE_DOOR_RELEASE, now + 500);
            } else if (cmd == CommandType::TOGGLE_LIGHT_PRESS) {
                this->enqueue_transmit(CommandType::TOGGLE_LIGHT_RELEASE, now + 500);
            } else if (cmd == CommandType::TOGGLE_LOCK_PRESS) {
                this->enqueue_transmit(CommandType::TOGGLE_LOCK_RELEASE, now + 3500);
            };
        }

        void Secplus1::enqueue_transmit(CommandType cmd, uint32_t time)
        {
            if (time == 0) {
                time = millis();
            }
            this->pending_tx_.push(TxCommand { cmd, time });
        }

        optional<CommandType> Secplus1::pending_tx()
        {
            if (this->pending_tx_.empty()) {
                return {};
            }
            auto cmd = this->pending_tx_.top();
            if (cmd.time > millis()) {
                return {};
            }
            return cmd.request;
        }

        optional<CommandType> Secplus1::pop_pending_tx()
        {
            auto cmd = this->pending_tx();
            if (cmd) {
                this->pending_tx_.pop();
            }
            return cmd;
        }

        void Secplus1::transmit_byte(uint32_t value)
        {
            bool enable_rx = (value == 0x38) || (value == 0x39) || (value == 0x3A);
            if (!enable_rx) {
                this->sw_serial_.enableIntTx(false);
            }
            this->sw_serial_.write(value);
            this->last_tx_ = millis();
            if (!enable_rx) {
                this->sw_serial_.enableIntTx(true);
            }
            ESP_LOG2(TAG, "[%d] Sent byte: [%02X]", millis(), value);
        }

    } // namespace secplus1
} // namespace ratgdo
} // namespace esphome
