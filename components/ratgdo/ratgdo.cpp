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
#include "common.h"
#include "ratgdo_state.h"

#ifdef PROTOCOL_DRYCONTACT
#include "dry_contact.h"
#endif
#ifdef PROTOCOL_SECPLUSV1
#include "secplus1.h"
#endif
#ifdef PROTOCOL_SECPLUSV2
#include "secplus2.h"
#endif

#include "esphome/core/application.h"
#include "esphome/core/gpio.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ratgdo {

    using namespace protocol;

    static const char* const TAG = "ratgdo";
    static const int SYNC_DELAY = 1000;

    // Short static defer names to avoid heap allocations from std::to_string()
    // 3 chars + null terminator = 4 bytes (aligned)
    // IMPORTANT: All names must be unique across the entire list to prevent conflicts
    // Multi-subscriber arrays (use get_defer_name helper)
    static const char* const DEFER_DOOR_STATE[] = { "ds0", "ds1" }; // door_state (2 callers)
    static const char* const DEFER_DOOR_ACTION_DELAYED[] = { "da0" }; // door_action_delayed (1 caller)
#ifdef RATGDO_USE_DISTANCE_SENSOR
    static const char* const DEFER_DISTANCE[] = { "dm0" }; // distance_measurement (1 caller)
#endif
#ifdef RATGDO_USE_VEHICLE_SENSORS
    static const char* const DEFER_VEHICLE_DETECTED[] = { "vd0" }; // vehicle_detected (1 caller)
    static const char* const DEFER_VEHICLE_ARRIVING[] = { "va0", "va1", "va2", "va3" }; // vehicle_arriving (4 callers)
    static const char* const DEFER_VEHICLE_LEAVING[] = { "vl0" }; // vehicle_leaving (1 caller)
#endif
    // Single-subscriber constexpr names
    static constexpr const char* DEFER_ROLLING_CODE = "rcc"; // rolling_code_counter
    static constexpr const char* DEFER_OPENING_DURATION = "opd"; // opening_duration
    static constexpr const char* DEFER_CLOSING_DURATION = "cld"; // closing_duration
    static constexpr const char* DEFER_CLOSING_DELAY = "cly"; // closing_delay
    static constexpr const char* DEFER_OPENINGS = "opn"; // openings
    static constexpr const char* DEFER_PAIRED_TOTAL = "ptl"; // paired_total
    static constexpr const char* DEFER_PAIRED_REMOTES = "prm"; // paired_remotes
    static constexpr const char* DEFER_PAIRED_KEYPADS = "pkp"; // paired_keypads
    static constexpr const char* DEFER_PAIRED_WALL_CONTROLS = "pwc"; // paired_wall_controls
    static constexpr const char* DEFER_PAIRED_ACCESSORIES = "pac"; // paired_accessories
    static constexpr const char* DEFER_LIGHT_STATE = "lts"; // light_state
    static constexpr const char* DEFER_LOCK_STATE = "lck"; // lock_state
    static constexpr const char* DEFER_OBSTRUCTION_STATE = "obs"; // obstruction_state
    static constexpr const char* DEFER_MOTOR_STATE = "mtr"; // motor_state
    static constexpr const char* DEFER_BUTTON_STATE = "btn"; // button_state
    static constexpr const char* DEFER_MOTION_STATE = "mot"; // motion_state
    static constexpr const char* DEFER_LEARN_STATE = "lrn"; // learn_state

    static void log_subscriber_overflow(const LogString* observable_name, uint8_t max)
    {
        ESP_LOGE(TAG, "Too many subscribers for %s (max %d)", LOG_STR_ARG(observable_name), max);
    }

    template <size_t N>
    static const char* get_defer_name(const char* const (&names)[N], uint8_t& counter, const LogString* observable_name)
    {
        if (counter >= N) {
            log_subscriber_overflow(observable_name, N);
            return names[N - 1];
        }
        return names[counter++];
    }

#ifdef RATGDO_USE_VEHICLE_SENSORS
    static const int CLEAR_PRESENCE = 60000; // how long to keep arriving/leaving active
    static const int PRESENCE_DETECT_WINDOW = 300000; // how long to calculate presence after door state change

    // increasing these values increases reliability but also increases detection time
    static constexpr int PRESENCE_DETECTION_ON_THRESHOLD = 5; // Minimum percentage of valid bitset::in_range samples required to detect vehicle
    static constexpr int PRESENCE_DETECTION_OFF_DEBOUNCE = 2; // The number of consecutive bitset::in_range iterations that must be 0 before clearing vehicle detected state
#endif

    void RATGDOComponent::setup()
    {
        this->output_gdo_pin_->setup();
        this->output_gdo_pin_->pin_mode(gpio::FLAG_OUTPUT);

        this->input_gdo_pin_->setup();
        this->input_gdo_pin_->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLUP);

        this->input_obst_pin_->setup();
#ifdef USE_ESP32
        this->input_obst_pin_->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLUP);
#else
        this->input_obst_pin_->pin_mode(gpio::FLAG_INPUT);
#endif
        this->input_obst_pin_->attach_interrupt(RATGDOStore::isr_obstruction, &this->isr_store_, gpio::INTERRUPT_FALLING_EDGE);

        this->protocol_->setup(this, &App.scheduler, this->input_gdo_pin_, this->output_gdo_pin_);

        // many things happening at startup, use some delay for sync
        set_timeout(SYNC_DELAY, [this] { this->sync(); });
        ESP_LOGD(TAG, " _____ _____ _____ _____ ____  _____ ");
        ESP_LOGD(TAG, "| __  |  _  |_   _|   __|    \\|     |");
        ESP_LOGD(TAG, "|    -|     | | | |  |  |  |  |  |  |");
        ESP_LOGD(TAG, "|__|__|__|__| |_| |_____|____/|_____|");
        ESP_LOGD(TAG, "https://paulwieland.github.io/ratgdo/");

        this->subscribe_door_state([this](DoorState state, float position) {
            static DoorState lastState = DoorState::UNKNOWN;

#ifdef RATGDO_USE_VEHICLE_SENSORS
            if (lastState != DoorState::UNKNOWN && state != DoorState::CLOSED && !this->flags_.presence_detect_window_active) {
                this->flags_.presence_detect_window_active = true;
                set_timeout("presence_detect_window", PRESENCE_DETECT_WINDOW, [this] {
                    this->flags_.presence_detect_window_active = false;
                });
            }

            if (state == DoorState::CLOSED) {
                this->flags_.presence_detect_window_active = false;
                cancel_timeout("presence_detect_window");
            }
#endif

            lastState = state;
        });
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
#ifdef PROTOCOL_DRYCONTACT
        this->protocol_ = new dry_contact::DryContact();
#endif
    }

    void RATGDOComponent::loop()
    {
        this->obstruction_loop();
        this->protocol_->loop();
    }

    void RATGDOComponent::dump_config()
    {
        ESP_LOGCONFIG(TAG, "Setting up RATGDO...");
        LOG_PIN("  Output GDO Pin: ", this->output_gdo_pin_);
        LOG_PIN("  Input GDO Pin: ", this->input_gdo_pin_);
        LOG_PIN("  Input Obstruction Pin: ", this->input_obst_pin_);
        this->protocol_->dump_config();
    }

    void RATGDOComponent::received(const DoorState door_state)
    {
        ESP_LOGD(TAG, "Door state=%s", LOG_STR_ARG(DoorState_to_string(door_state)));

        auto prev_door_state = *this->door_state;

        if (prev_door_state == door_state) {
            return;
        }

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
            if (*this->opening_duration != 0) {
                this->schedule_door_position_sync();
            }
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
            if (*this->closing_duration != 0) {
                this->schedule_door_position_sync();
            }
        } else if (door_state == DoorState::STOPPED) {
            this->door_position_update();
            if (*this->door_position == DOOR_POSITION_UNKNOWN) {
                this->door_position = 0.5; // best guess
            }
            this->cancel_position_sync_callbacks();
            cancel_timeout("door_query_state");
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
        this->on_door_state_.trigger(door_state);
    }

    void RATGDOComponent::received(const LearnState learn_state)
    {
        ESP_LOGD(TAG, "Learn state=%s", LOG_STR_ARG(LearnState_to_string(learn_state)));

        if (*this->learn_state == learn_state) {
            return;
        }

        if (learn_state == LearnState::INACTIVE) {
            this->query_paired_devices();
        }

        this->learn_state = learn_state;
    }

    void RATGDOComponent::received(const LightState light_state)
    {
        ESP_LOGD(TAG, "Light state=%s", LOG_STR_ARG(LightState_to_string(light_state)));
        this->light_state = light_state;
    }

    void RATGDOComponent::received(const LockState lock_state)
    {
        ESP_LOGD(TAG, "Lock state=%s", LOG_STR_ARG(LockState_to_string(lock_state)));
        this->lock_state = lock_state;
    }

    void RATGDOComponent::received(const ObstructionState obstruction_state)
    {
        if (!this->flags_.obstruction_sensor_detected) {
            ESP_LOGD(TAG, "Obstruction: state=%s", LOG_STR_ARG(ObstructionState_to_string(*this->obstruction_state)));

            this->obstruction_state = obstruction_state;
            // This isn't very fast to update, but its still better
            // than nothing in the case the obstruction sensor is not
            // wired up.
        }
    }

    void RATGDOComponent::received(const MotorState motor_state)
    {
        ESP_LOGD(TAG, "Motor: state=%s", LOG_STR_ARG(MotorState_to_string(*this->motor_state)));
        this->motor_state = motor_state;
    }

    void RATGDOComponent::received(const ButtonState button_state)
    {
        ESP_LOGD(TAG, "Button state=%s", LOG_STR_ARG(ButtonState_to_string(*this->button_state)));
        this->button_state = button_state;
    }

    void RATGDOComponent::received(const MotionState motion_state)
    {
        ESP_LOGD(TAG, "Motion: %s", LOG_STR_ARG(MotionState_to_string(*this->motion_state)));
        this->motion_state = motion_state;
        if (motion_state == MotionState::DETECTED) {
            this->set_timeout("clear_motion", 3000, [this] {
                this->motion_state = MotionState::CLEAR;
            });
            if (*this->light_state == LightState::OFF) {
                this->query_status();
            }
        }
    }

    void RATGDOComponent::received(const LightAction light_action)
    {
        ESP_LOGD(TAG, "Light cmd=%s state=%s",
            LOG_STR_ARG(LightAction_to_string(light_action)),
            LOG_STR_ARG(LightState_to_string(*this->light_state)));
        if (light_action == LightAction::OFF) {
            this->light_state = LightState::OFF;
        } else if (light_action == LightAction::ON) {
            this->light_state = LightState::ON;
        } else if (light_action == LightAction::TOGGLE) {
            this->light_state = light_state_toggle(*this->light_state);
        }
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
        ESP_LOGD(TAG, "Paired device count, kind=%s count=%d", LOG_STR_ARG(PairedDevice_to_string(pdc.kind)), pdc.count);

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

    void RATGDOComponent::received(const BatteryState battery_state)
    {
        ESP_LOGD(TAG, "Battery state=%s", LOG_STR_ARG(BatteryState_to_string(battery_state)));
    }

    void RATGDOComponent::schedule_door_position_sync(float update_period)
    {
        ESP_LOG1(TAG, "Schedule position sync: delta %f, start position: %f, start moving: %d",
            this->door_move_delta, this->door_start_position, this->door_start_moving);
        auto duration = this->door_move_delta > 0 ? *this->opening_duration : *this->closing_duration;
        if (duration == 0) {
            return;
        }
        auto count = int(1000 * duration / update_period);
        set_retry("position_sync_while_moving", update_period, count, [this](uint8_t r) {
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
        if (duration == 0) {
            return;
        }
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

#ifdef RATGDO_USE_DISTANCE_SENSOR
    void RATGDOComponent::set_target_distance_measurement(int16_t distance)
    {
        this->target_distance_measurement = distance;
    }

    void RATGDOComponent::set_distance_measurement(int16_t distance)
    {
        this->last_distance_measurement = distance;

#ifdef RATGDO_USE_VEHICLE_SENSORS
        this->in_range <<= 1;
        this->in_range.set(0, distance <= *this->target_distance_measurement);
        this->calculate_presence();
#endif
    }
#endif

#ifdef RATGDO_USE_VEHICLE_SENSORS
    void RATGDOComponent::calculate_presence()
    {
        int percent = this->in_range.count() * 100 / this->in_range.size();
        static int last_percent = -1;
        static int off_counter = 0;

        if (percent >= PRESENCE_DETECTION_ON_THRESHOLD)
            this->vehicle_detected_state = VehicleDetectedState::YES;

        if (percent == 0 && *this->vehicle_detected_state == VehicleDetectedState::YES) {
            off_counter++;
            ESP_LOGD(TAG, "Off counter: %d", off_counter);

            if (off_counter / this->in_range.size() >= PRESENCE_DETECTION_OFF_DEBOUNCE) {
                off_counter = 0;
                this->vehicle_detected_state = VehicleDetectedState::NO;
            }
        }

        if (percent != last_percent) {
            ESP_LOGD(TAG, "pct_in_range: %d", percent);
            last_percent = percent;
            off_counter = 0;
        }
        // ESP_LOGD(TAG, "in_range: %s", this->in_range.to_string().c_str());
    }
#endif

#ifdef RATGDO_USE_VEHICLE_SENSORS
    void RATGDOComponent::presence_change(bool sensor_value)
    {
        if (this->flags_.presence_detect_window_active) {
            if (sensor_value) {
                this->vehicle_arriving_state = VehicleArrivingState::YES;
                this->vehicle_leaving_state = VehicleLeavingState::NO;
                set_timeout(CLEAR_PRESENCE, [this] {
                    this->vehicle_arriving_state = VehicleArrivingState::NO;
                });
            } else {
                this->vehicle_arriving_state = VehicleArrivingState::NO;
                this->vehicle_leaving_state = VehicleLeavingState::YES;
                set_timeout(CLEAR_PRESENCE, [this] {
                    this->vehicle_leaving_state = VehicleLeavingState::NO;
                });
            }
        }
    }
#endif

    Result RATGDOComponent::call_protocol(Args args)
    {
        return this->protocol_->call(args);
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
            //     current_millis, this->isr_store_.obstruction_low_count, PULSES_LOWER_LIMIT,
            //     current_millis - last_asleep
            // );

            // check to see if we got more then PULSES_LOWER_LIMIT pulses
            if (this->isr_store_.obstruction_low_count > PULSES_LOWER_LIMIT) {
                this->obstruction_state = ObstructionState::CLEAR;
                this->flags_.obstruction_sensor_detected = true;
            } else if (this->isr_store_.obstruction_low_count == 0) {
                // if there have been no pulses the line is steady high or low
#ifdef USE_ESP32
                if (this->input_obst_pin_->digital_read()) {
#else
                if (!this->input_obst_pin_->digital_read()) {
#endif
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
        this->protocol_->call(QueryStatus {});
    }

    void RATGDOComponent::query_openings()
    {
        this->protocol_->call(QueryOpenings {});
    }

    void RATGDOComponent::query_paired_devices()
    {
        this->protocol_->call(QueryPairedDevicesAll {});
    }

    void RATGDOComponent::query_paired_devices(PairedDevice kind)
    {
        this->protocol_->call(QueryPairedDevices { kind });
    }

    void RATGDOComponent::clear_paired_devices(PairedDevice kind)
    {
        this->protocol_->call(ClearPairedDevices { kind });
    }

    void RATGDOComponent::sync()
    {
        this->protocol_->sync();

        // dry contact protocol:
        // needed to trigger the intial state of the limit switch sensors
        // ideally this would be in drycontact::sync
#ifdef PROTOCOL_DRYCONTACT
        this->protocol_->set_open_limit(this->dry_contact_open_sensor_->state);
        this->protocol_->set_close_limit(this->dry_contact_close_sensor_->state);
#endif
    }

    void RATGDOComponent::door_open()
    {
        if (*this->door_state == DoorState::OPENING) {
            return; // gets ignored by opener
        }

        this->door_action(DoorAction::OPEN);

        if (*this->opening_duration > 0) {
            // query state in case we don't get a status message
            set_timeout("door_query_state", (*this->opening_duration + 2) * 1000, [this]() {
                if (*this->door_state != DoorState::OPEN && *this->door_state != DoorState::STOPPED) {
                    this->received(DoorState::OPEN); // probably missed a status mesage, assume it's open
                    this->query_status(); // query in case we're wrong and it's stopped
                }
            });
        }
    }

    void RATGDOComponent::door_close()
    {
        if (*this->door_state == DoorState::CLOSING) {
            return; // gets ignored by opener
        }

        if (*this->door_state == DoorState::OPENING) {
            // have to stop door first, otherwise close command is ignored
            this->door_action(DoorAction::STOP);
            this->on_door_state_([this](DoorState s) {
                if (s == DoorState::STOPPED) {
                    this->door_action(DoorAction::CLOSE);
                } else {
                    ESP_LOGW(TAG, "Door did not stop, ignoring close command");
                }
            });
            return;
        }

        if (this->flags_.obstruction_sensor_detected) {
            this->door_action(DoorAction::CLOSE);
        } else if (*this->door_state == DoorState::OPEN) {
            ESP_LOGD(TAG, "No obstruction sensors detected. Close using TOGGLE.");
            this->door_action(DoorAction::TOGGLE);
        }

        if (*this->closing_duration > 0) {
            // query state in case we don't get a status message
            set_timeout("door_query_state", (*this->closing_duration + 2) * 1000, [this]() {
                if (*this->door_state != DoorState::CLOSED && *this->door_state != DoorState::STOPPED) {
                    this->received(DoorState::CLOSED); // probably missed a status mesage, assume it's closed
                    this->query_status(); // query in case we're wrong and it's stopped
                }
            });
        }
    }

    void RATGDOComponent::door_stop()
    {
        if (*this->door_state != DoorState::OPENING && *this->door_state != DoorState::CLOSING) {
            ESP_LOGW(TAG, "The door is not moving.");
            return;
        }
        this->door_action(DoorAction::STOP);
    }

    void RATGDOComponent::door_toggle()
    {
        this->door_action(DoorAction::TOGGLE);
    }

    void RATGDOComponent::door_action(DoorAction action)
    {
#ifdef RATGDO_USE_CLOSING_DELAY
        if (*this->closing_delay > 0 && (action == DoorAction::CLOSE || (action == DoorAction::TOGGLE && *this->door_state != DoorState::CLOSED))) {
            this->door_action_delayed = DoorActionDelayed::YES;
            set_timeout("door_action", *this->closing_delay * 1000, [this] {
                this->door_action_delayed = DoorActionDelayed::NO;
                this->protocol_->door_action(DoorAction::CLOSE);
            });
        } else {
            this->protocol_->door_action(action);
        }
#else
        this->protocol_->door_action(action);
#endif
    }

    void RATGDOComponent::door_move_to_position(float position)
    {
        if (*this->door_state == DoorState::OPENING || *this->door_state == DoorState::CLOSING) {
            this->door_action(DoorAction::STOP);
            this->on_door_state_([this, position](DoorState s) {
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

        this->door_action(delta > 0 ? DoorAction::OPEN : DoorAction::CLOSE);
        set_timeout("move_to_position", operation_time, [this] {
            this->door_action(DoorAction::STOP);
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

    void RATGDOComponent::light_toggle()
    {
        this->light_state = light_state_toggle(*this->light_state);
        this->protocol_->light_action(LightAction::TOGGLE);
    }

    LightState RATGDOComponent::get_light_state() const
    {
        return *this->light_state;
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

    void RATGDOComponent::lock_toggle()
    {
        this->lock_state = lock_state_toggle(*this->lock_state);
        this->protocol_->lock_action(LockAction::TOGGLE);
    }

    // Learn functions
    void RATGDOComponent::activate_learn()
    {
        this->protocol_->call(ActivateLearn {});
    }

    void RATGDOComponent::inactivate_learn()
    {
        this->protocol_->call(InactivateLearn {});
    }

    void RATGDOComponent::subscribe_rolling_code_counter(std::function<void(uint32_t)>&& f)
    {
        // change update to children is defered until after component loop
        // if multiple changes occur during component loop, only the last one is notified
        auto counter = this->protocol_->call(GetRollingCodeCounter {});
        if (counter.tag == Result::Tag::rolling_code_counter) {
            counter.value.rolling_code_counter.value->subscribe([this, f = std::move(f)](uint32_t state) { defer(DEFER_ROLLING_CODE, [f, state] { f(state); }); });
        }
    }
    void RATGDOComponent::subscribe_opening_duration(std::function<void(float)>&& f)
    {
        this->opening_duration.subscribe([this, f = std::move(f)](float state) { defer(DEFER_OPENING_DURATION, [f, state] { f(state); }); });
    }
    void RATGDOComponent::subscribe_closing_duration(std::function<void(float)>&& f)
    {
        this->closing_duration.subscribe([this, f = std::move(f)](float state) { defer(DEFER_CLOSING_DURATION, [f, state] { f(state); }); });
    }
#ifdef RATGDO_USE_CLOSING_DELAY
    void RATGDOComponent::subscribe_closing_delay(std::function<void(uint32_t)>&& f)
    {
        this->closing_delay.subscribe([this, f = std::move(f)](uint32_t state) { defer(DEFER_CLOSING_DELAY, [f, state] { f(state); }); });
    }
#endif
    void RATGDOComponent::subscribe_openings(std::function<void(uint16_t)>&& f)
    {
        this->openings.subscribe([this, f = std::move(f)](uint16_t state) { defer(DEFER_OPENINGS, [f, state] { f(state); }); });
    }
    void RATGDOComponent::subscribe_paired_devices_total(std::function<void(uint8_t)>&& f)
    {
        this->paired_total.subscribe([this, f = std::move(f)](uint8_t state) { defer(DEFER_PAIRED_TOTAL, [f, state] { f(state); }); });
    }
    void RATGDOComponent::subscribe_paired_remotes(std::function<void(uint8_t)>&& f)
    {
        this->paired_remotes.subscribe([this, f = std::move(f)](uint8_t state) { defer(DEFER_PAIRED_REMOTES, [f, state] { f(state); }); });
    }
    void RATGDOComponent::subscribe_paired_keypads(std::function<void(uint8_t)>&& f)
    {
        this->paired_keypads.subscribe([this, f = std::move(f)](uint8_t state) { defer(DEFER_PAIRED_KEYPADS, [f, state] { f(state); }); });
    }
    void RATGDOComponent::subscribe_paired_wall_controls(std::function<void(uint8_t)>&& f)
    {
        this->paired_wall_controls.subscribe([this, f = std::move(f)](uint8_t state) { defer(DEFER_PAIRED_WALL_CONTROLS, [f, state] { f(state); }); });
    }
    void RATGDOComponent::subscribe_paired_accessories(std::function<void(uint8_t)>&& f)
    {
        this->paired_accessories.subscribe([this, f = std::move(f)](uint8_t state) { defer(DEFER_PAIRED_ACCESSORIES, [f, state] { f(state); }); });
    }
    void RATGDOComponent::subscribe_door_state(std::function<void(DoorState, float)>&& f)
    {
        const char* name = get_defer_name(DEFER_DOOR_STATE, this->door_state_sub_num_, LOG_STR("door_state"));

        this->door_state.subscribe([this, f, name](DoorState state) {
            defer(name, [this, f, state] { f(state, *this->door_position); });
        });
        this->door_position.subscribe([this, f, name](float position) {
            defer(name, [this, f, position] { f(*this->door_state, position); });
        });
    }
    void RATGDOComponent::subscribe_light_state(std::function<void(LightState)>&& f)
    {
        this->light_state.subscribe([this, f = std::move(f)](LightState state) { defer(DEFER_LIGHT_STATE, [f, state] { f(state); }); });
    }
    void RATGDOComponent::subscribe_lock_state(std::function<void(LockState)>&& f)
    {
        this->lock_state.subscribe([this, f = std::move(f)](LockState state) { defer(DEFER_LOCK_STATE, [f, state] { f(state); }); });
    }
    void RATGDOComponent::subscribe_obstruction_state(std::function<void(ObstructionState)>&& f)
    {
        this->obstruction_state.subscribe([this, f = std::move(f)](ObstructionState state) { defer(DEFER_OBSTRUCTION_STATE, [f, state] { f(state); }); });
    }
    void RATGDOComponent::subscribe_motor_state(std::function<void(MotorState)>&& f)
    {
        this->motor_state.subscribe([this, f = std::move(f)](MotorState state) { defer(DEFER_MOTOR_STATE, [f, state] { f(state); }); });
    }
    void RATGDOComponent::subscribe_button_state(std::function<void(ButtonState)>&& f)
    {
        this->button_state.subscribe([this, f = std::move(f)](ButtonState state) { defer(DEFER_BUTTON_STATE, [f, state] { f(state); }); });
    }
    void RATGDOComponent::subscribe_motion_state(std::function<void(MotionState)>&& f)
    {
        this->motion_state.subscribe([this, f = std::move(f)](MotionState state) { defer(DEFER_MOTION_STATE, [f, state] { f(state); }); });
    }
    void RATGDOComponent::subscribe_sync_failed(std::function<void(bool)>&& f)
    {
        this->sync_failed.subscribe(std::move(f));
    }
    void RATGDOComponent::subscribe_learn_state(std::function<void(LearnState)>&& f)
    {
        this->learn_state.subscribe([this, f = std::move(f)](LearnState state) { defer(DEFER_LEARN_STATE, [f, state] { f(state); }); });
    }
    void RATGDOComponent::subscribe_door_action_delayed(std::function<void(DoorActionDelayed)>&& f)
    {
        const char* name = get_defer_name(DEFER_DOOR_ACTION_DELAYED, this->door_action_delayed_sub_num_, LOG_STR("door_action_delayed"));

        this->door_action_delayed.subscribe([this, f = std::move(f), name](DoorActionDelayed state) { defer(name, [f, state] { f(state); }); });
    }
#ifdef RATGDO_USE_DISTANCE_SENSOR
    void RATGDOComponent::subscribe_distance_measurement(std::function<void(int16_t)>&& f)
    {
        const char* name = get_defer_name(DEFER_DISTANCE, this->distance_sub_num_, LOG_STR("distance_measurement"));
        this->last_distance_measurement.subscribe([this, f = std::move(f), name](int16_t state) { defer(name, [f, state] { f(state); }); });
    }
#endif
#ifdef RATGDO_USE_VEHICLE_SENSORS
    void RATGDOComponent::subscribe_vehicle_detected_state(std::function<void(VehicleDetectedState)>&& f)
    {
        const char* name = get_defer_name(DEFER_VEHICLE_DETECTED, this->vehicle_detected_sub_num_, LOG_STR("vehicle_detected"));
        this->vehicle_detected_state.subscribe([this, f = std::move(f), name](VehicleDetectedState state) { defer(name, [f, state] { f(state); }); });
    }
    void RATGDOComponent::subscribe_vehicle_arriving_state(std::function<void(VehicleArrivingState)>&& f)
    {
        const char* name = get_defer_name(DEFER_VEHICLE_ARRIVING, this->vehicle_arriving_sub_num_, LOG_STR("vehicle_arriving"));
        this->vehicle_arriving_state.subscribe([this, f = std::move(f), name](VehicleArrivingState state) { defer(name, [f, state] { f(state); }); });
    }
    void RATGDOComponent::subscribe_vehicle_leaving_state(std::function<void(VehicleLeavingState)>&& f)
    {
        const char* name = get_defer_name(DEFER_VEHICLE_LEAVING, this->vehicle_leaving_sub_num_, LOG_STR("vehicle_leaving"));
        this->vehicle_leaving_state.subscribe([this, f = std::move(f), name](VehicleLeavingState state) { defer(name, [f, state] { f(state); }); });
    }
#endif

    // dry contact methods
    void RATGDOComponent::set_dry_contact_open_sensor(esphome::binary_sensor::BinarySensor* dry_contact_open_sensor)
    {
        dry_contact_open_sensor_ = dry_contact_open_sensor;
        dry_contact_open_sensor_->add_on_state_callback([this](bool sensor_value) {
            this->protocol_->set_open_limit(sensor_value);
        });
    }

    void RATGDOComponent::set_dry_contact_close_sensor(esphome::binary_sensor::BinarySensor* dry_contact_close_sensor)
    {
        dry_contact_close_sensor_ = dry_contact_close_sensor;
        dry_contact_close_sensor_->add_on_state_callback([this](bool sensor_value) {
            this->protocol_->set_close_limit(sensor_value);
        });
    }

} // namespace ratgdo
} // namespace esphome
