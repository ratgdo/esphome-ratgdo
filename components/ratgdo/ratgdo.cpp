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
#include "common.h"
#include "secplus1.h"
#include "secplus2.h"

#include "esphome/core/log.h"
#include "esphome/core/gpio.h"
#include "esphome/core/application.h"


namespace esphome {
namespace ratgdo {

    using namespace protocol;

    static const char* const TAG = "ratgdo";
    static const int SYNC_DELAY = 1000;

    void RATGDOComponent::setup()
    {
        this->output_gdo_pin_->setup();
        this->output_gdo_pin_->pin_mode(gpio::FLAG_OUTPUT);

        this->input_gdo_pin_->setup();
        this->input_gdo_pin_->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLUP);

        if (this->input_obst_pin_ == nullptr || this->input_obst_pin_->get_pin() == 0) {
            // Our base.yaml is always going to set this so we check for 0
            // as well to avoid a breaking change.
            this->obstruction_from_status_ = true;
        } else {
            this->input_obst_pin_->setup();
            this->input_obst_pin_->pin_mode(gpio::FLAG_INPUT);
            this->input_obst_pin_->attach_interrupt(RATGDOStore::isr_obstruction, &this->isr_store_, gpio::INTERRUPT_FALLING_EDGE);
        }

        this->protocol_->setup(this, &App.scheduler, this->input_gdo_pin_, this->output_gdo_pin_);

        // many things happening at startup, use some delay for sync
        set_timeout(SYNC_DELAY, [=] { this->sync(); });
    }


    // initializing protocol, this gets called before setup() because 
    // its children components might require that
    void RATGDOComponent::init_protocol()
    {
#ifdef PROTOCOL_SECPLUSV2
        this->protocol_ = new secplus2::Secplus2();
#endif        
#ifdef PROTOCOL_SECPLUSV1
        this->protocol_ = new secplus1::Secplus1();
#endif        
    }

    void RATGDOComponent::loop()
    {
        if (!this->obstruction_from_status_) {
            this->obstruction_loop();
        }
        this->protocol_->loop();
    }

    void RATGDOComponent::dump_config()
    {
        ESP_LOGCONFIG(TAG, "Setting up RATGDO...");
        LOG_PIN("  Output GDO Pin: ", this->output_gdo_pin_);
        LOG_PIN("  Input GDO Pin: ", this->input_gdo_pin_);
        if (this->obstruction_from_status_) {
            ESP_LOGCONFIG(TAG, "  Input Obstruction Pin: not used, will detect from GDO status");
        } else {
            LOG_PIN("  Input Obstruction Pin: ", this->input_obst_pin_);
        }
        this->protocol_->dump_config();
    }


    void RATGDOComponent::received(const DoorState door_state) 
    {
        auto prev_door_state = *this->door_state;

        // opening duration calibration
        if (*this->opening_duration == 0) {
            if (door_state == DoorState::OPENING && prev_door_state == DoorState::CLOSED) {
                this->start_opening = millis();
            }
            if (door_state == DoorState::OPEN && prev_door_state == DoorState::OPENING && this->start_opening > 0) {
                auto duration = (millis() - this->start_opening) / 1000;
                this->set_opening_duration(round(duration * 10) / 10);
            }
            if (door_state == DoorState::STOPPED) {
                this->start_opening = -1;
            }
        }
        // closing duration calibration
        if (*this->closing_duration == 0) {
            if (door_state == DoorState::CLOSING && prev_door_state == DoorState::OPEN) {
                this->start_closing = millis();
            }
            if (door_state == DoorState::CLOSED && prev_door_state == DoorState::CLOSING && this->start_closing > 0) {
                auto duration = (millis() - this->start_closing) / 1000;
                this->set_closing_duration(round(duration * 10) / 10);
            }
            if (door_state == DoorState::STOPPED) {
                this->start_closing = -1;
            }
        }

        if (door_state == DoorState::OPENING) {
            // door started opening
            if (prev_door_state == DoorState::CLOSING) {
                this->door_position_update();
                this->cancel_position_sync_callbacks();
                this->door_move_delta = DOOR_DELTA_UNKNOWN;
            }
            this->door_start_moving = millis();
            this->door_start_position = *this->door_position;
            if (this->door_move_delta == DOOR_DELTA_UNKNOWN) {
                this->door_move_delta = 1.0 - this->door_start_position;
            }
            this->schedule_door_position_sync();
        } else if (door_state == DoorState::CLOSING) {
            // door started closing
            if (prev_door_state == DoorState::OPENING) {
                this->door_position_update();
                this->cancel_position_sync_callbacks();
                this->door_move_delta = DOOR_DELTA_UNKNOWN;
            }
            this->door_start_moving = millis();
            this->door_start_position = *this->door_position;
            if (this->door_move_delta == DOOR_DELTA_UNKNOWN) {
                this->door_move_delta = 0.0 - this->door_start_position;
            }
            this->schedule_door_position_sync();
        } else if (door_state == DoorState::STOPPED) {
            this->door_position_update();
            if (*this->door_position == DOOR_POSITION_UNKNOWN) {
                this->door_position = 0.5; // best guess
            }
            this->cancel_position_sync_callbacks();
        } else if (door_state == DoorState::OPEN) {
            this->door_position = 1.0;
            this->cancel_position_sync_callbacks();
        } else if (door_state == DoorState::CLOSED) {
            this->door_position = 0.0;
            this->cancel_position_sync_callbacks();
        }

        if (door_state == DoorState::OPEN || door_state == DoorState::CLOSED || door_state == DoorState::STOPPED) {
            this->motor_state = MotorState::OFF;
        }

        if (door_state == DoorState::CLOSED && door_state != prev_door_state) {
            this->query_openings();
        }

        this->door_state = door_state;
        this->door_state_received(door_state);

        ESP_LOGD(TAG, "Door state=%s", DoorState_to_string(door_state));
    }

    void RATGDOComponent::received(const LearnState learn_state)
    {
        this->learn_state = learn_state;
        ESP_LOGD(TAG, "Learn state=%s", LearnState_to_string(learn_state));
    }

    void RATGDOComponent::received(const LightState light_state) 
    {
        this->light_state = light_state;
        ESP_LOGD(TAG, "Light state=%s", LightState_to_string(light_state));
    }

    void RATGDOComponent::received(const LockState lock_state) 
    {
        this->lock_state = lock_state;
        ESP_LOGD(TAG, "Lock state=%s", LockState_to_string(lock_state));
    }

    void RATGDOComponent::received(const ObstructionState obstruction_state)
    {
        if (this->obstruction_from_status_) {
            this->obstruction_state = obstruction_state;
            // This isn't very fast to update, but its still better
            // than nothing in the case the obstruction sensor is not
            // wired up.
            ESP_LOGD(TAG, "Obstruction: reading from GDO status byte1, bit6=%s", ObstructionState_to_string(*this->obstruction_state));
        }
    }


    void RATGDOComponent::received(const MotorState motor_state)
    {
        this->motor_state = motor_state;
        ESP_LOGD(TAG, "Motor: state=%s", MotorState_to_string(*this->motor_state));
    }

    void RATGDOComponent::received(const ButtonState button_state)
    {
        this->button_state = button_state;
        ESP_LOGD(TAG, "Button state=%s", ButtonState_to_string(*this->button_state));
    }
    
    void RATGDOComponent::received(const MotionState motion_state)
    {
        this->motion_state = motion_state;
        if (motion_state == MotionState::DETECTED) {
            this->set_timeout("clear_motion", 3000, [=] {
                this->motion_state = MotionState::CLEAR;
            });
            if (*this->light_state == LightState::OFF) {
                this->query_status();
            }
        }
        ESP_LOGD(TAG, "Motion: %s", MotionState_to_string(*this->motion_state));
    }

    void RATGDOComponent::received(const LightAction light_action) 
    {
        if (light_action == LightAction::OFF) {
            this->light_state = LightState::OFF;
        } else if (light_action == LightAction::ON) {
            this->light_state = LightState::ON;
        } else if (light_action == LightAction::TOGGLE) {
            this->light_state = light_state_toggle(*this->light_state);
        }
        ESP_LOGD(TAG, "Light cmd=%s state=%s",
            LightAction_to_string(light_action),
            LightState_to_string(*this->light_state)
        );
    }

    void RATGDOComponent::received(const Openings openings)
    {
        if (openings.flag == 0 || *this->openings != 0) {
            this->openings = openings.count;
            ESP_LOGD(TAG, "Openings: %d", *this->openings);
        } else {
            ESP_LOGD(TAG, "Ignoring openings, not from our request");
        }
    }

    void RATGDOComponent::received(const PairedDeviceCount pdc)
    {
        if (pdc.kind == PairedDevice::ALL) {
            this->paired_total = pdc.count;
        } else if (pdc.kind == PairedDevice::REMOTE) {
            this->paired_remotes = pdc.count;
        } else if (pdc.kind == PairedDevice::KEYPAD) {
            this->paired_keypads = pdc.count;
        } else if (pdc.kind == PairedDevice::WALL_CONTROL) {
            this->paired_wall_controls = pdc.count;
        } else if (pdc.kind == PairedDevice::ACCESSORY) {
            this->paired_accessories = pdc.count;
        }
    }

    void RATGDOComponent::received(const TimeToClose ttc)
    {
        ESP_LOGD(TAG, "Time to close (TTC): %ds", ttc.seconds);
    }

    void RATGDOComponent::schedule_door_position_sync(float update_period)
    {
        ESP_LOG1(TAG, "Schedule position sync: delta %f, start position: %f, start moving: %d",
            this->door_move_delta, this->door_start_position, this->door_start_moving);
        auto duration = this->door_move_delta > 0 ? *this->opening_duration : *this->closing_duration;
        auto count = int(1000 * duration / update_period);
        set_retry("position_sync_while_moving", update_period, count, [=](uint8_t r) {
            this->door_position_update();
            return RetryResult::RETRY;
        });
    }

    void RATGDOComponent::door_position_update()
    {
        if (this->door_start_moving == 0 || this->door_start_position == DOOR_POSITION_UNKNOWN || this->door_move_delta == DOOR_DELTA_UNKNOWN) {
            return;
        }
        auto now = millis();
        auto duration = this->door_move_delta > 0 ? *this->opening_duration : -*this->closing_duration;
        auto position = this->door_start_position + (now - this->door_start_moving) / (1000 * duration);
        ESP_LOG2(TAG, "[%d] Position update: %f", now, position);
        this->door_position = clamp(position, 0.0f, 1.0f);
    }


    void RATGDOComponent::set_opening_duration(float duration)
    {
        ESP_LOGD(TAG, "Set opening duration: %.1fs", duration);
        this->opening_duration = duration;
    }

    void RATGDOComponent::set_closing_duration(float duration)
    {
        ESP_LOGD(TAG, "Set closing duration: %.1fs", duration);
        this->closing_duration = duration;
    }

    void RATGDOComponent::set_rolling_code_counter(uint32_t counter)
    {

        ESP_LOGV(TAG, "Set rolling code counter to %d", counter);
        this->protocol_->call(SetRollingCodeCounter{counter});
    }

    void RATGDOComponent::set_client_id(uint64_t client_id)
    {
        this->protocol_->call(SetClientID{client_id});
    }

    /*************************** OBSTRUCTION DETECTION ***************************/

    void RATGDOComponent::obstruction_loop()
    {
        long current_millis = millis();
        static unsigned long last_millis = 0;
        static unsigned long last_asleep = 0;

        // the obstruction sensor has 3 states: clear (HIGH with LOW pulse every 7ms), obstructed (HIGH), asleep (LOW)
        // the transitions between awake and asleep are tricky because the voltage drops slowly when falling asleep
        // and is high without pulses when waking up

        // If at least 3 low pulses are counted within 50ms, the door is awake, not obstructed and we don't have to check anything else

        const long CHECK_PERIOD = 50;
        const long PULSES_LOWER_LIMIT = 3;

        if (current_millis - last_millis > CHECK_PERIOD) {
            // ESP_LOGD(TAG, "%ld: Obstruction count: %d, expected: %d, since asleep: %ld",
            //     current_millis, this->isr_store_.obstruction_low_count, PULSES_EXPECTED,
            //     current_millis - last_asleep
            // );

            // check to see if we got more then PULSES_LOWER_LIMIT pulses
            if (this->isr_store_.obstruction_low_count > PULSES_LOWER_LIMIT) {
                this->obstruction_state = ObstructionState::CLEAR;
            } else if (this->isr_store_.obstruction_low_count == 0) {
                // if there have been no pulses the line is steady high or low
                if (!this->input_obst_pin_->digital_read()) {
                    // asleep
                    last_asleep = current_millis;
                } else {
                    // if the line is high and was last asleep more than 700ms ago, then there is an obstruction present
                    if (current_millis - last_asleep > 700) {
                        this->obstruction_state = ObstructionState::OBSTRUCTED;
                    }
                }
            }
            last_millis = current_millis;
            this->isr_store_.obstruction_low_count = 0;
        }
    }


    void RATGDOComponent::query_status()
    {
        ESP_LOG2(TAG, "Query status action");
        this->protocol_->call(QueryStatus{});
    }

    void RATGDOComponent::query_openings()
    {
        this->protocol_->call(QueryOpenings{});
    }

    void RATGDOComponent::sync()
    {
        this->protocol_->sync();
    }

    void RATGDOComponent::open_door()
    {
        if (*this->door_state == DoorState::OPENING) {
            return; // gets ignored by opener
        }

        this->protocol_->door_action(DoorAction::OPEN);
    }

    void RATGDOComponent::close_door()
    {
        if (*this->door_state == DoorState::CLOSING) {
            return; // gets ignored by opener
        }

        if (*this->door_state == DoorState::OPENING) {
            // have to stop door first, otherwise close command is ignored
            this->protocol_->door_action(DoorAction::STOP);
            this->door_state_received.then([=](DoorState s) {
                if (s == DoorState::STOPPED) {
                    this->protocol_->door_action(DoorAction::CLOSE);
                } else {
                    ESP_LOGW(TAG, "Door did not stop, ignoring close command");
                }
            });
            return;
        }

        this->protocol_->door_action(DoorAction::CLOSE);
    }

    void RATGDOComponent::stop_door()
    {
        if (*this->door_state != DoorState::OPENING && *this->door_state != DoorState::CLOSING) {
            ESP_LOGW(TAG, "The door is not moving.");
            return;
        }
        this->protocol_->door_action(DoorAction::STOP);
    }

    void RATGDOComponent::toggle_door()
    {
        this->protocol_->door_action(DoorAction::TOGGLE);
    }

    void RATGDOComponent::door_move_to_position(float position)
    {
        if (*this->door_state == DoorState::OPENING || *this->door_state == DoorState::CLOSING) {
            this->protocol_->door_action(DoorAction::STOP);
            this->door_state_received.then([=](DoorState s) {
                if (s == DoorState::STOPPED) {
                    this->door_move_to_position(position);
                }
            });
            return;
        }

        auto delta = position - *this->door_position;
        if (delta == 0) {
            ESP_LOGD(TAG, "Door is already at position %.2f", position);
            return;
        }

        auto duration = delta > 0 ? *this->opening_duration : -*this->closing_duration;
        if (duration == 0) {
            ESP_LOGW(TAG, "I don't know duration, ignoring move to position");
            return;
        }

        auto operation_time = 1000 * duration * delta;
        this->door_move_delta = delta;
        ESP_LOGD(TAG, "Moving to position %.2f in %.1fs", position, operation_time / 1000.0);

        this->protocol_->door_action(delta > 0 ? DoorAction::OPEN : DoorAction::CLOSE);
        set_timeout("move_to_position", operation_time, [=] {
            this->ensure_door_action(DoorAction::STOP);
        });
    }

    void RATGDOComponent::cancel_position_sync_callbacks()
    {
        if (this->door_start_moving != 0) {
            ESP_LOGD(TAG, "Cancelling position callbacks");
            cancel_timeout("move_to_position");
            cancel_retry("position_sync_while_moving");

            this->door_start_moving = 0;
            this->door_start_position = DOOR_POSITION_UNKNOWN;
            this->door_move_delta = DOOR_DELTA_UNKNOWN;
        }
    }

    void RATGDOComponent::ensure_door_action(DoorAction action, uint32_t delay)
    {
        if (action == DoorAction::TOGGLE) {
            ESP_LOGW(TAG, "It's not recommended to use ensure_door_action with non-idempotent commands such as DOOR_TOGGLE");
        }
        auto prev_door_state = *this->door_state;
        this->door_state_received.then([=](DoorState s) {
            if ((action == DoorAction::STOP) && (s != DoorState::STOPPED) && !(prev_door_state == DoorState::OPENING && s == DoorState::OPEN) && !(prev_door_state == DoorState::CLOSING && s == DoorState::CLOSED)) {
                return;
            }
            if (action == DoorAction::OPEN && !(s == DoorState::OPENING || s == DoorState::OPEN)) {
                return;
            }
            if (action == DoorAction::CLOSE && !(s == DoorState::CLOSED || s == DoorState::CLOSING)) {
                return;
            }

            ESP_LOG1(TAG, "Received door status, cancel door command retry");
            cancel_timeout("door_command_retry");
        });
        this->protocol_->door_action(action);
        ESP_LOG1(TAG, "Ensure door command, setup door command retry");
        set_timeout("door_command_retry", delay, [=]() {
            this->ensure_door_action(action);
        });
    }

    void RATGDOComponent::light_on()
    {
        this->light_state = LightState::ON;
        this->protocol_->light_action(LightAction::ON);
    }

    void RATGDOComponent::light_off()
    {
        this->light_state = LightState::OFF;
        this->protocol_->light_action(LightAction::OFF);
    }

    void RATGDOComponent::toggle_light()
    {
        this->light_state = light_state_toggle(*this->light_state);
        this->protocol_->light_action(LightAction::TOGGLE);
    }

    // Lock functions
    void RATGDOComponent::lock()
    {
        this->lock_state = LockState::LOCKED;
        this->protocol_->lock_action(LockAction::LOCK);
    }

    void RATGDOComponent::unlock()
    {
        this->lock_state = LockState::UNLOCKED;
        this->protocol_->lock_action(LockAction::UNLOCK);
    }

    void RATGDOComponent::toggle_lock()
    {
        this->lock_state = lock_state_toggle(*this->lock_state);
        this->protocol_->lock_action(LockAction::TOGGLE);
    }

    LightState RATGDOComponent::get_light_state() const
    {
        return *this->light_state;
    }

    // Learn functions
    void RATGDOComponent::activate_learn()
    {
        this->protocol_->call(ActivateLearn{});
    }

    void RATGDOComponent::inactivate_learn()
    {
        this->protocol_->call(InactivateLearn{});
    }

    void RATGDOComponent::subscribe_rolling_code_counter(std::function<void(uint32_t)>&& f)
    {
        // change update to children is defered until after component loop
        // if multiple changes occur during component loop, only the last one is notified
        auto counter = this->protocol_->call(GetRollingCodeCounter{});
        if (counter.tag==Result::Tag::rolling_code_counter) {
            counter.value.rolling_code_counter.value->subscribe([=](uint32_t state) { defer("rolling_code_counter", [=] { f(state); }); });
        }
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
    void RATGDOComponent::subscribe_paired_devices_total(std::function<void(uint16_t)>&& f)
    {
        this->paired_total.subscribe([=](uint16_t state) { defer("paired_total", [=] { f(state); }); });
    }
    void RATGDOComponent::subscribe_paired_remotes(std::function<void(uint16_t)>&& f)
    {
        this->paired_remotes.subscribe([=](uint16_t state) { defer("paired_remotes", [=] { f(state); }); });
    }
    void RATGDOComponent::subscribe_paired_keypads(std::function<void(uint16_t)>&& f)
    {
        this->paired_keypads.subscribe([=](uint16_t state) { defer("paired_keypads", [=] { f(state); }); });
    }
    void RATGDOComponent::subscribe_paired_wall_controls(std::function<void(uint16_t)>&& f)
    {
        this->paired_wall_controls.subscribe([=](uint16_t state) { defer("paired_wall_controls", [=] { f(state); }); });
    }
    void RATGDOComponent::subscribe_paired_accessories(std::function<void(uint16_t)>&& f)
    {
        this->paired_accessories.subscribe([=](uint16_t state) { defer("paired_accessories", [=] { f(state); }); });
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
    void RATGDOComponent::subscribe_sync_failed(std::function<void(bool)>&& f)
    {
        this->sync_failed.subscribe(std::move(f));
    }
    void RATGDOComponent::subscribe_learn_state(std::function<void(LearnState)>&& f)
    {
        this->learn_state.subscribe([=](LearnState state) { defer("learn_state", [=] { f(state); }); });
    }

} // namespace ratgdo
} // namespace esphome
