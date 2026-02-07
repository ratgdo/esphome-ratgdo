
#ifdef PROTOCOL_SECPLUSV2

#include "secplus2.h"
#include "ratgdo.h"

#include "esphome/core/gpio.h"
#include "esphome/core/log.h"
#include "esphome/core/scheduler.h"

extern "C" {
#include "secplus.h"
}

namespace esphome {
namespace ratgdo {
    namespace secplus2 {

        // MAX_CODES_WITHOUT_FLASH_WRITE is a bit of a guess
        // since we write the flash at most every every 1min
        //
        // We want the rolling counter to be high enough that the
        // GDO will accept the command after an unexpected reboot
        // that did not save the counter to flash in time which
        // results in the rolling counter being behind what the GDO
        // expects.
        static const uint8_t MAX_CODES_WITHOUT_FLASH_WRITE = 60;

        static const char* const TAG = "ratgdo_secplus2";

        void Secplus2::setup(RATGDOComponent* ratgdo, Scheduler* scheduler, InternalGPIOPin* rx_pin, InternalGPIOPin* tx_pin)
        {
            this->ratgdo_ = ratgdo;
            this->scheduler_ = scheduler;
            this->tx_pin_ = tx_pin;
            this->rx_pin_ = rx_pin;

            this->sw_serial_.begin(9600, SWSERIAL_8N1, rx_pin->get_pin(), tx_pin->get_pin(), true);
            this->sw_serial_.enableIntTx(false);
            this->sw_serial_.enableAutoBaud(true);

            this->traits_.set_features(Traits::all());
        }

        void Secplus2::loop()
        {
            if (this->flags_.transmit_pending) {
                if (!this->transmit_packet()) {
                    return;
                }
            }

            auto cmd = this->read_command();
            if (cmd) {
                this->handle_command(*cmd);
            }
        }

        void Secplus2::dump_config()
        {
            ESP_LOGCONFIG(TAG, "  Rolling Code Counter: %d", *this->rolling_code_counter_);
            ESP_LOGCONFIG(TAG, "  Client ID: %d", this->client_id_);
            ESP_LOGCONFIG(TAG, "  Protocol: SEC+ v2");
        }

        void Secplus2::sync_helper(uint32_t start, uint32_t delay, uint8_t tries)
        {
            bool synced = true;
            if (*this->ratgdo_->door_state == DoorState::UNKNOWN) {
                this->query_status();
                synced = false;
            }
            if (*this->ratgdo_->openings == 0) {
                this->query_openings();
                synced = false;
            }
            if (*this->ratgdo_->paired_total == PAIRED_DEVICES_UNKNOWN) {
                this->query_paired_devices(PairedDevice::ALL);
                synced = false;
            }
            if (*this->ratgdo_->paired_remotes == PAIRED_DEVICES_UNKNOWN) {
                this->query_paired_devices(PairedDevice::REMOTE);
                synced = false;
            }
            if (*this->ratgdo_->paired_keypads == PAIRED_DEVICES_UNKNOWN) {
                this->query_paired_devices(PairedDevice::KEYPAD);
                synced = false;
            }
            if (*this->ratgdo_->paired_wall_controls == PAIRED_DEVICES_UNKNOWN) {
                this->query_paired_devices(PairedDevice::WALL_CONTROL);
                synced = false;
            }
            if (*this->ratgdo_->paired_accessories == PAIRED_DEVICES_UNKNOWN) {
                this->query_paired_devices(PairedDevice::ACCESSORY);
                synced = false;
            }

            if (synced) {
                return;
            }

            if (tries == 2 && *this->ratgdo_->door_state == DoorState::UNKNOWN) { // made a few attempts and no progress (door state is the first sync request)
                // increment rolling code counter by some amount in case we crashed without writing to flash the latest value
                this->increment_rolling_code_counter(MAX_CODES_WITHOUT_FLASH_WRITE);
            }

            // not sync-ed after 30s, notify failure
            if (millis() - start > 30000) {
                ESP_LOGW(TAG, "Triggering sync failed actions.");
                this->ratgdo_->sync_failed = true;
            } else {
                if (tries % 3 == 0) {
                    delay *= 1.5;
                }
                this->scheduler_->set_timeout(this->ratgdo_, "sync", delay, [this, start, delay, tries]() {
                    this->sync_helper(start, delay, tries + 1);
                });
            };
        }

        void Secplus2::sync()
        {
            this->scheduler_->cancel_timeout(this->ratgdo_, "sync");
            this->sync_helper(millis(), 500, 0);
        }

        void Secplus2::light_action(LightAction action)
        {
            if (action == LightAction::UNKNOWN) {
                return;
            }
            this->send_command(Command(CommandType::LIGHT, static_cast<uint8_t>(action)));
        }

        void Secplus2::lock_action(LockAction action)
        {
            if (action == LockAction::UNKNOWN) {
                return;
            }
            this->send_command(Command(CommandType::LOCK, static_cast<uint8_t>(action)));
        }

        void Secplus2::door_action(DoorAction action)
        {
            if (action == DoorAction::UNKNOWN) {
                return;
            }
            this->door_command(action);
        }

        Result Secplus2::call(Args args)
        {
            using Tag = Args::Tag;
            if (args.tag == Tag::query_status) {
                this->send_command(CommandType::GET_STATUS);
            } else if (args.tag == Tag::query_openings) {
                this->send_command(CommandType::GET_OPENINGS);
            } else if (args.tag == Tag::get_rolling_code_counter) {
                return Result(RollingCodeCounter { std::addressof(this->rolling_code_counter_) });
            } else if (args.tag == Tag::set_rolling_code_counter) {
                this->set_rolling_code_counter(args.value.set_rolling_code_counter.counter);
            } else if (args.tag == Tag::set_client_id) {
                this->set_client_id(args.value.set_client_id.client_id);
            } else if (args.tag == Tag::query_paired_devices) {
                this->query_paired_devices(args.value.query_paired_devices.kind);
            } else if (args.tag == Tag::query_paired_devices_all) {
                this->query_paired_devices();
            } else if (args.tag == Tag::clear_paired_devices) {
                this->clear_paired_devices(args.value.clear_paired_devices.kind);
            } else if (args.tag == Tag::activate_learn) {
                this->activate_learn();
            } else if (args.tag == Tag::inactivate_learn) {
                this->inactivate_learn();
            }
            return {};
        }

        void Secplus2::door_command(DoorAction action)
        {
            this->send_command(Command(CommandType::DOOR_ACTION, static_cast<uint8_t>(action), 1, 1), IncrementRollingCode::NO, [this, action]() {
                this->scheduler_->set_timeout(this->ratgdo_, "", 150, [this, action] {
                    this->send_command(Command(CommandType::DOOR_ACTION, static_cast<uint8_t>(action), 0, 1));
                });
            });
        }

        void Secplus2::query_status()
        {
            this->send_command(CommandType::GET_STATUS);
        }

        void Secplus2::query_openings()
        {
            this->send_command(CommandType::GET_OPENINGS);
        }

        void Secplus2::query_paired_devices()
        {
            const auto kinds = {
                PairedDevice::ALL,
                PairedDevice::REMOTE,
                PairedDevice::KEYPAD,
                PairedDevice::WALL_CONTROL,
                PairedDevice::ACCESSORY
            };
            uint32_t timeout = 0;
            for (auto kind : kinds) {
                timeout += 200;
                this->scheduler_->set_timeout(this->ratgdo_, "", timeout, [this, kind] { this->query_paired_devices(kind); });
            }
        }

        void Secplus2::query_paired_devices(PairedDevice kind)
        {
            ESP_LOGD(TAG, "Query paired devices of type: %s", LOG_STR_ARG(PairedDevice_to_string(kind)));
            this->send_command(Command { CommandType::GET_PAIRED_DEVICES, static_cast<uint8_t>(kind) });
        }

        // wipe devices from memory based on get paired devices nibble values
        void Secplus2::clear_paired_devices(PairedDevice kind)
        {
            if (kind == PairedDevice::UNKNOWN) {
                return;
            }
            ESP_LOGW(TAG, "Clear paired devices of type: %s", LOG_STR_ARG(PairedDevice_to_string(kind)));
            if (kind == PairedDevice::ALL) {
                this->scheduler_->set_timeout(this->ratgdo_, "", 200, [this] { this->send_command(Command { CommandType::CLEAR_PAIRED_DEVICES, static_cast<uint8_t>(PairedDevice::REMOTE) - 1 }); }); // wireless
                this->scheduler_->set_timeout(this->ratgdo_, "", 400, [this] { this->send_command(Command { CommandType::CLEAR_PAIRED_DEVICES, static_cast<uint8_t>(PairedDevice::KEYPAD) - 1 }); }); // keypads
                this->scheduler_->set_timeout(this->ratgdo_, "", 600, [this] { this->send_command(Command { CommandType::CLEAR_PAIRED_DEVICES, static_cast<uint8_t>(PairedDevice::WALL_CONTROL) - 1 }); }); // wall controls
                this->scheduler_->set_timeout(this->ratgdo_, "", 800, [this] { this->send_command(Command { CommandType::CLEAR_PAIRED_DEVICES, static_cast<uint8_t>(PairedDevice::ACCESSORY) - 1 }); }); // accessories
                this->scheduler_->set_timeout(this->ratgdo_, "", 1000, [this] { this->query_status(); });
                this->scheduler_->set_timeout(this->ratgdo_, "", 1200, [this] { this->query_paired_devices(); });
            } else {
                uint8_t dev_kind = static_cast<uint8_t>(kind) - 1;
                this->send_command(Command { CommandType::CLEAR_PAIRED_DEVICES, dev_kind }); // just requested device
                this->scheduler_->set_timeout(this->ratgdo_, "", 200, [this] { this->query_status(); });
                this->scheduler_->set_timeout(this->ratgdo_, "", 400, [this, kind] { this->query_paired_devices(kind); });
            }
        }

        // Learn functions
        void Secplus2::activate_learn()
        {
            // Send LEARN with nibble = 0 then nibble = 1 to mimic wall control learn button
            this->send_command(Command { CommandType::LEARN, 0 });
            this->scheduler_->set_timeout(this->ratgdo_, "", 150, [this] { this->send_command(Command { CommandType::LEARN, 1 }); });
            this->scheduler_->set_timeout(this->ratgdo_, "", 500, [this] { this->query_status(); });
        }

        void Secplus2::inactivate_learn()
        {
            // Send LEARN twice with nibble = 0 to inactivate learn and get status to update switch state
            this->send_command(Command { CommandType::LEARN, 0 });
            this->scheduler_->set_timeout(this->ratgdo_, "", 150, [this] { this->send_command(Command { CommandType::LEARN, 0 }); });
            this->scheduler_->set_timeout(this->ratgdo_, "", 500, [this] { this->query_status(); });
        }

        optional<Command> Secplus2::read_command()
        {
            static bool reading_msg = false;
            static uint32_t msg_start = 0;
            static uint16_t byte_count = 0;
            static WirePacket rx_packet;
            static uint32_t last_read = 0;

            if (!reading_msg) {
                while (this->sw_serial_.available()) {
                    uint8_t ser_byte = this->sw_serial_.read();
                    last_read = millis();

                    if (ser_byte != 0x55 && ser_byte != 0x01 && ser_byte != 0x00) {
                        ESP_LOG2(TAG, "Ignoring byte (%d): %02X, baud: %d", byte_count, ser_byte, this->sw_serial_.baudRate());
                        byte_count = 0;
                        continue;
                    }
                    msg_start = ((msg_start << 8) | ser_byte) & 0xffffff;
                    byte_count++;

                    // if we are at the start of a message, capture the next 16 bytes
                    if (msg_start == 0x550100) {
                        ESP_LOG1(TAG, "Baud: %d", this->sw_serial_.baudRate());
                        rx_packet[0] = 0x55;
                        rx_packet[1] = 0x01;
                        rx_packet[2] = 0x00;

                        reading_msg = true;
                        break;
                    }
                }
            }
            if (reading_msg) {
                while (this->sw_serial_.available()) {
                    uint8_t ser_byte = this->sw_serial_.read();
                    last_read = millis();
                    rx_packet[byte_count] = ser_byte;
                    byte_count++;
                    // ESP_LOG2(TAG, "Received byte (%d): %02X, baud: %d", byte_count, ser_byte, this->sw_serial_.baudRate());

                    if (byte_count == PACKET_LENGTH) {
                        reading_msg = false;
                        byte_count = 0;
                        this->print_packet(LOG_STR("Received packet: "), rx_packet);
                        return this->decode_packet(rx_packet);
                    }
                }

                if (millis() - last_read > 100) {
                    // if we have a partial packet and it's been over 100ms since last byte was read,
                    // the rest is not coming (a full packet should be received in ~20ms),
                    // discard it so we can read the following packet correctly
                    ESP_LOGW(TAG, "Discard incomplete packet, length: %d", byte_count);
                    reading_msg = false;
                    byte_count = 0;
                }
            }

            return {};
        }

        void Secplus2::print_packet(const esphome::LogString* prefix, const WirePacket& packet) const
        {
            ESP_LOGD(TAG, "%s: [%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X]",
                LOG_STR_ARG(prefix),
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

        optional<Command> Secplus2::decode_packet(const WirePacket& packet) const
        {
            uint32_t rolling = 0;
            uint64_t fixed = 0;
            uint32_t data = 0;

            int err = decode_wireline(packet, &rolling, &fixed, &data);
            if (err < 0) {
                ESP_LOGW(TAG, "Decode failed (parity error or invalid frame)");
                return {};
            }

            uint16_t cmd = ((fixed >> 24) & 0xf00) | (data & 0xff);
            data &= ~0xf000; // clear parity nibble

            if ((fixed & 0xFFFFFFFF) == this->client_id_) { // my commands
                ESP_LOG1(TAG, "[%ld] received mine: rolling=%07" PRIx32 " fixed=%010" PRIx64 " data=%08" PRIx32, millis(), rolling, fixed, data);
                return {};
            } else {
                ESP_LOG1(TAG, "[%ld] received rolling=%07" PRIx32 " fixed=%010" PRIx64 " data=%08" PRIx32, millis(), rolling, fixed, data);
            }

            CommandType cmd_type = to_CommandType(cmd, CommandType::UNKNOWN);
            uint8_t nibble = (data >> 8) & 0xff;
            uint8_t byte1 = (data >> 16) & 0xff;
            uint8_t byte2 = (data >> 24) & 0xff;

            ESP_LOG1(TAG, "cmd=%03x (%s) byte2=%02x byte1=%02x nibble=%01x", cmd, LOG_STR_ARG(CommandType_to_string(cmd_type)), byte2, byte1, nibble);

            return Command { cmd_type, nibble, byte1, byte2 };
        }

        void Secplus2::handle_command(const Command& cmd)
        {
            ESP_LOG1(TAG, "Handle command: %s", LOG_STR_ARG(CommandType_to_string(cmd.type)));

            if (cmd.type == CommandType::STATUS) {

                this->ratgdo_->received(to_DoorState(cmd.nibble, DoorState::UNKNOWN));
                this->ratgdo_->received(to_LightState((cmd.byte2 >> 1) & 1, LightState::UNKNOWN));
                this->ratgdo_->received(to_LockState((cmd.byte2 & 1), LockState::UNKNOWN));
                // ESP_LOGD(TAG, "Obstruction: reading from byte2, bit2, status=%d", ((byte2 >> 2) & 1) == 1);
                this->ratgdo_->received(to_ObstructionState((cmd.byte1 >> 6) & 1, ObstructionState::UNKNOWN));
                this->ratgdo_->received(to_LearnState((cmd.byte2 >> 5) & 1, LearnState::UNKNOWN));
            } else if (cmd.type == CommandType::LIGHT) {
                this->ratgdo_->received(to_LightAction(cmd.nibble, LightAction::UNKNOWN));
            } else if (cmd.type == CommandType::MOTOR_ON) {
                this->ratgdo_->received(MotorState::ON);
            } else if (cmd.type == CommandType::DOOR_ACTION) {
                auto button_state = (cmd.byte1 & 1) == 1 ? ButtonState::PRESSED : ButtonState::RELEASED;
                this->ratgdo_->received(button_state);
            } else if (cmd.type == CommandType::MOTION) {
                this->ratgdo_->received(MotionState::DETECTED);
            } else if (cmd.type == CommandType::OPENINGS) {
                this->ratgdo_->received(Openings { static_cast<uint16_t>((cmd.byte1 << 8) | cmd.byte2), cmd.nibble });
            } else if (cmd.type == CommandType::SET_TTC) {
                this->ratgdo_->received(TimeToClose { static_cast<uint16_t>((cmd.byte1 << 8) | cmd.byte2) });
            } else if (cmd.type == CommandType::PAIRED_DEVICES) {
                PairedDeviceCount pdc;
                pdc.kind = to_PairedDevice(cmd.nibble, PairedDevice::UNKNOWN);
                if (pdc.kind == PairedDevice::ALL) {
                    pdc.count = cmd.byte2;
                } else if (pdc.kind == PairedDevice::REMOTE) {
                    pdc.count = cmd.byte2;
                } else if (pdc.kind == PairedDevice::KEYPAD) {
                    pdc.count = cmd.byte2;
                } else if (pdc.kind == PairedDevice::WALL_CONTROL) {
                    pdc.count = cmd.byte2;
                } else if (pdc.kind == PairedDevice::ACCESSORY) {
                    pdc.count = cmd.byte2;
                }
                this->ratgdo_->received(pdc);
            } else if (cmd.type == CommandType::BATTERY_STATUS) {
                this->ratgdo_->received(to_BatteryState(cmd.byte1, BatteryState::UNKNOWN));
            }

            ESP_LOG1(TAG, "Done handle command: %s", LOG_STR_ARG(CommandType_to_string(cmd.type)));
        }

        void Secplus2::send_command(Command command, IncrementRollingCode increment)
        {
            ESP_LOGD(TAG, "Send command: %s, data: %02X%02X%02X", LOG_STR_ARG(CommandType_to_string(command.type)), command.byte2, command.byte1, command.nibble);
            if (!this->flags_.transmit_pending) { // have an untransmitted packet
                this->encode_packet(command, this->tx_packet_);
                if (increment == IncrementRollingCode::YES) {
                    this->increment_rolling_code_counter();
                }
            } else {
                // unlikely this would happed (unless not connected to GDO), we're ensuring any pending packet
                // is transmitted each loop before doing anyting else
                if (this->transmit_pending_start_ > 0) {
                    ESP_LOGW(TAG, "Have untransmitted packet, ignoring command: %s", LOG_STR_ARG(CommandType_to_string(command.type)));
                } else {
                    ESP_LOGW(TAG, "Not connected to GDO, ignoring command: %s", LOG_STR_ARG(CommandType_to_string(command.type)));
                }
            }
            this->transmit_packet();
        }

        void Secplus2::send_command(Command command, IncrementRollingCode increment, std::function<void()>&& on_sent)
        {
            this->on_command_sent_(on_sent);
            this->send_command(command, increment);
        }

        void Secplus2::encode_packet(Command command, WirePacket& packet)
        {
            auto cmd = static_cast<uint64_t>(command.type);
            uint64_t fixed = ((cmd & ~0xff) << 24) | this->client_id_;
            uint32_t data = (static_cast<uint64_t>(command.byte2) << 24) | (static_cast<uint64_t>(command.byte1) << 16) | (static_cast<uint64_t>(command.nibble) << 8) | (cmd & 0xff);

            ESP_LOG2(TAG, "[%ld] Encode for transmit rolling=%07" PRIx32 " fixed=%010" PRIx64 " data=%08" PRIx32, millis(), *this->rolling_code_counter_, fixed, data);
            encode_wireline(*this->rolling_code_counter_, fixed, data, packet);
        }

        bool Secplus2::transmit_packet()
        {
            auto now = micros();

            while (micros() - now < 1300) {
                if (this->rx_pin_->digital_read()) {
                    if (!this->flags_.transmit_pending) {
                        this->flags_.transmit_pending = true;
                        this->transmit_pending_start_ = millis();
                        ESP_LOGD(TAG, "Collision detected, waiting to send packet");
                    } else if (millis() - this->transmit_pending_start_ >= 5000) {
                        this->transmit_pending_start_ = 0; // to indicate GDO not connected state
                    }
                    return false;
                }
                delayMicroseconds(100);
            }

            this->print_packet(LOG_STR("Sending packet"), this->tx_packet_);

            // indicate the start of a frame by pulling the 12V line low for at leat 1 byte followed by
            // one STOP bit, which indicates to the receiving end that the start of the message follows
            // The output pin is controlling a transistor, so the logic is inverted
            this->tx_pin_->digital_write(true); // pull the line low for at least 1 byte
            delayMicroseconds(1300);
            this->tx_pin_->digital_write(false); // line high for at least 1 bit
            delayMicroseconds(130);

            this->sw_serial_.write(this->tx_packet_, PACKET_LENGTH);

            this->flags_.transmit_pending = false;
            this->transmit_pending_start_ = 0;
            this->on_command_sent_.trigger();
            return true;
        }

        void Secplus2::increment_rolling_code_counter(int delta)
        {
            this->rolling_code_counter_ = (*this->rolling_code_counter_ + delta) & 0xfffffff;
        }

        void Secplus2::set_rolling_code_counter(uint32_t counter)
        {
            ESP_LOGV(TAG, "Set rolling code counter to %d", counter);
            this->rolling_code_counter_ = counter;
        }

        void Secplus2::set_client_id(uint64_t client_id)
        {
            this->client_id_ = client_id & 0xFFFFFFFF;
        }

    } // namespace secplus2
} // namespace ratgdo
} // namespace esphome

#endif // PROTOCOL_SECPLUSV2
