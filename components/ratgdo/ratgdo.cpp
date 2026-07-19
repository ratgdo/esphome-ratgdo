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
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/preferences.h"

namespace esphome::ratgdo {

using namespace protocol;

static const char* const TAG = "ratgdo";
static constexpr int SYNC_DELAY = 1000;
// Door state updates arrive over UART every ~200-400ms during movement.
// 2 seconds gives ample margin for slow openers while still expiring
// stale callbacks before a user could reasonably trigger an unrelated
// door state change.
static constexpr uint32_t DOOR_STATE_CALLBACK_TIMEOUT = 2000;

// Grace period for the opener to broadcast a state change after the encoder detects movement.
// If movement continues without an opener update beyond this threshold, it is attributed to manual operation.
static constexpr uint32_t PROTOCOL_STALE_MS = 500;

using namespace scheduler_ids;

void log_subscriber_overflow(const LogString* observable_name, uint32_t max)
{
    ESP_LOGE(TAG, "Too many subscribers for %s (max %d)",
        LOG_STR_ARG(observable_name), (int)max);
}

#ifdef RATGDO_USE_VEHICLE_SENSORS
static constexpr int CLEAR_PRESENCE = 60000; // how long to keep arriving/leaving active
static constexpr int PRESENCE_DETECT_WINDOW = 300000; // how long to calculate presence after door state change
static constexpr int PRESENCE_DETECT_WINDOW_AFTER_CLOSE = 15000; // how long to keep presence window active after door reaches closed

// increasing these values increases reliability but also increases detection
// time
static constexpr int PRESENCE_DETECTION_ON_THRESHOLD = 5; // Minimum percentage of valid bitset::in_range samples required to
                                                          // detect vehicle
static constexpr int PRESENCE_DETECTION_OFF_DEBOUNCE = 2; // The number of consecutive bitset::in_range iterations that must be 0
                                                          // before clearing vehicle detected state
#endif

#ifdef RATGDO_USE_ENCODER

static constexpr uint32_t ENC_STOPPED_WATCHDOG_MS = 2000; // Maximum expected gap between encoder pulses during door travel,
                                                          // plus a safety margin. If no pulse arrives within this window the
                                                          // door is declared stopped.

static constexpr int8_t ENC_DIRECTION_CHANGE_THRESHOLD = 3; // Number of consecutive ISR pulses in the opposite direction
                                                            // required to confirm a real direction reversal mid-travel.
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
    this->input_obst_pin_->attach_interrupt(RATGDOStore::isr_obstruction,
        &this->isr_store_,
        gpio::INTERRUPT_FALLING_EDGE);

    this->protocol_->setup(this, &App.scheduler, this->input_gdo_pin_,
        this->output_gdo_pin_);

#ifdef RATGDO_USE_ENCODER
    if (encoder_sensor_ != nullptr && enc_pin_a_ != nullptr) {
        enc_pin_a_->setup();
        enc_pin_b_->setup();
        isr_store_.enc_pin_a = enc_pin_a_->to_isr();
        isr_store_.enc_pin_b = enc_pin_b_->to_isr();
        enc_pin_a_->attach_interrupt(RATGDOStore::isr_encoder, &isr_store_, gpio::INTERRUPT_ANY_EDGE);
        enc_pin_b_->attach_interrupt(RATGDOStore::isr_encoder, &isr_store_, gpio::INTERRUPT_ANY_EDGE);
        // Initialise prev_state from the actual pin levels so the first ISR fire
        // gets a valid transition rather than always-from-00.
        bool pa = enc_pin_a_->digital_read();
        bool pb = enc_pin_b_->digital_read();
        {
            InterruptLock lock;
            isr_store_.enc_prev_state = (static_cast<uint8_t>(pa) << 1) | static_cast<uint8_t>(pb);
            isr_store_.enc_delta = 0;
        }
    }
#endif

    // many things happening at startup, use some delay for sync
    this->set_timeout(SYNC_DELAY, [this] { this->sync(); });
    ESP_LOGD(TAG, " _____ _____ _____ _____ ____  _____ ");
    ESP_LOGD(TAG, "| __  |  _  |_   _|   __|    \\|     |");
    ESP_LOGD(TAG, "|    -|     | | | |  |  |  |  |  |  |");
    ESP_LOGD(TAG, "|__|__|__|__| |_| |_____|____/|_____|");
    ESP_LOGD(TAG, "https://paulwieland.github.io/ratgdo/");

    this->subscribe_door_state([this](DoorState state, float position) {
#ifdef RATGDO_USE_VEHICLE_SENSORS
        if (this->last_door_state_for_presence_ != DoorState::UNKNOWN && state != DoorState::CLOSED && !this->flags_.presence_detect_window_active) {
            this->flags_.presence_detect_window_active = true;
            this->set_timeout(
                TIMEOUT_PRESENCE_DETECT_WINDOW, PRESENCE_DETECT_WINDOW,
                [this] { this->flags_.presence_detect_window_active = false; });
        }

        if (state == DoorState::CLOSED) {
            this->set_timeout(
                TIMEOUT_PRESENCE_DETECT_WINDOW, PRESENCE_DETECT_WINDOW_AFTER_CLOSE,
                [this] { this->flags_.presence_detect_window_active = false; });
        }

        this->last_door_state_for_presence_ = state;
#endif
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
    // obstruction_loop() must run before protocol_->loop() because it uses
    // App.get_loop_component_start_time() and protocol_->loop() may block
    // for up to 1.3ms (secplus2 transmit collision wait), which would make
    // the cached timestamp stale.
    this->obstruction_loop();
    this->protocol_->loop();

#ifdef RATGDO_USE_ENCODER
    if (encoder_sensor_ != nullptr) {
        // debounce encoder signals
        static uint32_t last_enc_drain_ms = 0;
        uint32_t now_ms = millis();
        if (now_ms - last_enc_drain_ms >= 100) {
            last_enc_drain_ms = now_ms;
            int16_t delta;
            {
                InterruptLock lock;
                delta = isr_store_.enc_delta;
                isr_store_.enc_delta = 0;
            }
            if (delta != 0)
                on_encoder_update(enc_last_ + delta);
        }
    }
#endif
}

void RATGDOComponent::dump_config()
{
    ESP_LOGCONFIG(TAG, "Setting up RATGDO...");
    LOG_PIN("  Output GDO Pin: ", this->output_gdo_pin_);
    LOG_PIN("  Input GDO Pin: ", this->input_gdo_pin_);
    LOG_PIN("  Input Obstruction Pin: ", this->input_obst_pin_);
    this->protocol_->dump_config();
}

void RATGDOComponent::on_shutdown()
{
    if (this->protocol_ != nullptr) {
        this->protocol_->on_shutdown();
    }
}

// Common method to start or resynchronize the local TTC countdown.
//  - sets COUNTING state
//  - starts (or restarts) the 5s local decrement interval
//  - starts (or restarts) the 90s countdown watchdog
// Used both when the countdown is beginning fresh (a release) and when an
// already-running countdown is being resynced to a fresh broadcast value,
// so all three call sites behave identically.
//
// The watchdog's purpose is to handle comms failures. It needs to be long
// enough that normal timing differences between the GDO and local countdown
// don't set it off accidentally, but short enough that comms failures are
// quickly and safely detected. The GDO nominally transmits TTC_COUNTDOWN
// messages every minute, but in testing these were spaced 63 seconds apart.
// Setting the watchdog to 90 seconds (TTC_COUNTDOWN_WATCHDOG_TIMEOUT)
// allows 30 seconds of margin beyond the expected countdown message interval,
// which is ten times larger than the 3 second variation observed in testing.
//
void RATGDOComponent::start_or_sync_ttc_countdown(uint16_t seconds)
{
    this->ttc_countdown = seconds;
    this->ttc_state = TtcState::COUNTING;
    this->cancel_timeout(scheduler_ids::TTC_COUNTDOWN_WATCHDOG);
    this->set_timeout(scheduler_ids::TTC_COUNTDOWN_WATCHDOG, TTC_COUNTDOWN_WATCHDOG_TIMEOUT * 1000, [this]() {
        // Didn't see a TTC_COUNTDOWN broadcast within TTC_COUNTDOWN_WATCHDOG_TIMEOUT (90) seconds.
        // Assume comms failure and transition to UNKNOWN state.
        this->cancel_interval(scheduler_ids::TTC_COUNTDOWN_LOCAL_DECREMENT);
        this->ttc_countdown = 0;
        this->ttc_state = TtcState::UNKNOWN;
    });
    this->cancel_interval(scheduler_ids::TTC_COUNTDOWN_LOCAL_DECREMENT);
    this->set_interval(scheduler_ids::TTC_COUNTDOWN_LOCAL_DECREMENT, TTC_COUNTDOWN_LOCAL_DECREMENT_INTERVAL * 1000, [this]() {
        uint16_t current = *this->ttc_countdown;
        if (current > TTC_COUNTDOWN_LOCAL_DECREMENT_INTERVAL) {
            this->ttc_countdown = current - TTC_COUNTDOWN_LOCAL_DECREMENT_INTERVAL;
        } else {
            // Local estimate ran out before either a new broadcast or the
            // watchdog fired. From here we wait for the door close,
            // or the watchdog timer to fire.
            this->cancel_interval(scheduler_ids::TTC_COUNTDOWN_LOCAL_DECREMENT);
            this->ttc_countdown = 0;
            this->ttc_state = TtcState::COUNTING_FINISHED;
        }
    });
}

void RATGDOComponent::received(const DoorState door_state)
{
#ifdef RATGDO_USE_ENCODER
    bool protocol_state_changed = false;
    if (this->protocol_door_state_ != door_state) {
        protocol_state_changed = true;
        this->protocol_door_state_ = door_state;
    }

    if (protocol_state_changed || door_state == DoorState::OPENING || door_state == DoorState::CLOSING || door_state == this->encoder_door_state_) {
        this->encoder_motion_onset_ms_ = 0; // Protocol caught up, clear any pending manual operation trip
        if (*this->manually_operated_state != ManuallyOperatedState::NO) {
            this->manually_operated_state = ManuallyOperatedState::NO;
        }
    } else {
        if (*this->manually_operated_state == ManuallyOperatedState::YES) {
            ESP_LOGW(TAG, "Dropping protocol state %s due to latched manual operation", LOG_STR_ARG(DoorState_to_string(door_state)));
            return;
        }
    }
#endif

    this->set_resolved_door_state(door_state);
}

#ifdef RATGDO_USE_ENCODER
void RATGDOComponent::encoder_received(const DoorState door_state)
{
    this->encoder_door_state_ = door_state;

    auto proto_state = this->protocol_door_state_;

    if (proto_state == DoorState::UNKNOWN) {
        this->set_resolved_door_state(door_state);
        return;
    }

    if ((door_state == DoorState::OPENING || door_state == DoorState::CLOSING) && (proto_state == DoorState::OPEN || proto_state == DoorState::CLOSED || proto_state == DoorState::STOPPED)) {
        if (this->encoder_motion_onset_ms_ == 0) {
            this->encoder_motion_onset_ms_ = millis();
        } else if (millis() - this->encoder_motion_onset_ms_ > PROTOCOL_STALE_MS) {
            if (*this->manually_operated_state != ManuallyOperatedState::YES) {
                this->manually_operated_state = ManuallyOperatedState::YES;
            }
            this->set_resolved_door_state(door_state);
        }
    } else {
        this->encoder_motion_onset_ms_ = 0;
        if (door_state == DoorState::STOPPED || door_state == DoorState::OPEN || door_state == DoorState::CLOSED) {
            if (*this->manually_operated_state == ManuallyOperatedState::YES) {
                this->set_resolved_door_state(door_state);
            }
        }
    }
}
#endif

void RATGDOComponent::set_resolved_door_state(const DoorState door_state)
{
    auto prev_door_state = *this->door_state;

    if (prev_door_state == door_state) {
        return;
    }

    ESP_LOGD(TAG, "Door state=%s", LOG_STR_ARG(DoorState_to_string(door_state)));

#ifdef RATGDO_USE_ENCODER
    // Auto-calibration logic
#ifndef PROTOCOL_DRYCONTACT
    if (this->protocol_ != nullptr && encoder_sensor_ != nullptr) {
        if (door_state == DoorState::OPENING || door_state == DoorState::CLOSING) {
            if (enc_travel_dir_ != 0) {
                bool enc_is_opening = (enc_travel_dir_ > 0) ? !flags_.reverse_encoder : flags_.reverse_encoder;
                bool proto_is_opening = (door_state == DoorState::OPENING);
                if (enc_is_opening != proto_is_opening) {
                    flags_.reverse_encoder = !flags_.reverse_encoder;
                    ESP_LOGI(TAG, "Auto-calibrated encoder direction: reverse_encoder=%d", flags_.reverse_encoder);

                    if (this->encoder_door_state_ == DoorState::OPENING)
                        this->encoder_door_state_ = DoorState::CLOSING;
                    else if (this->encoder_door_state_ == DoorState::CLOSING)
                        this->encoder_door_state_ = DoorState::OPENING;
                }
            }
        }
    }
#endif // PROTOCOL_DRYCONTACT
#endif

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
        this->flags_.ttc_limit_learned = false; // force relearn ttc_limit from this cycle's broadcasts
                                                // These can start before OPEN is reached
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
#ifdef RATGDO_USE_ENCODER
            if (encoder_sensor_ == nullptr)
#endif
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
#ifdef RATGDO_USE_ENCODER
            if (encoder_sensor_ == nullptr)
#endif
                this->schedule_door_position_sync();
        }
    } else if (door_state == DoorState::STOPPED) {
#ifdef RATGDO_USE_ENCODER
        if (encoder_sensor_ == nullptr)
#endif
            this->door_position_update();
        if (*this->door_position == DOOR_POSITION_UNKNOWN) {
            this->door_position = 0.5; // best guess
        }
        this->cancel_position_sync_callbacks();
        this->cancel_timeout(TIMEOUT_DOOR_QUERY_STATE);
#ifdef RATGDO_USE_ENCODER
        enc_intended_dir_ = 0; // intent expires when door comes to rest
#endif
    } else if (door_state == DoorState::OPEN) {
        this->door_position = 1.0;
        this->cancel_position_sync_callbacks();
#ifdef RATGDO_USE_ENCODER
        enc_intended_dir_ = 0; // open intent satisfied
#endif
    } else if (door_state == DoorState::CLOSED) {
        this->door_position = 0.0;
        this->cancel_position_sync_callbacks();
        this->cancel_timeout(scheduler_ids::TTC_COUNTDOWN_WATCHDOG);
        this->cancel_interval(scheduler_ids::TTC_COUNTDOWN_LOCAL_DECREMENT);
        this->ttc_state = TtcState::UNKNOWN;
        this->ttc_countdown = 0;
#ifdef RATGDO_USE_ENCODER
        enc_intended_dir_ = 0; // close intent satisfied
#endif
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
    ESP_LOGD(TAG, "Learn state=%s",
        LOG_STR_ARG(LearnState_to_string(learn_state)));

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
    ESP_LOGD(TAG, "Light state=%s",
        LOG_STR_ARG(LightState_to_string(light_state)));
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
        ESP_LOGD(TAG, "Obstruction: state=%s",
            LOG_STR_ARG(ObstructionState_to_string(*this->obstruction_state)));

        this->obstruction_state = obstruction_state;
        // This isn't very fast to update, but its still better
        // than nothing in the case the obstruction sensor is not
        // wired up.
    }
}

void RATGDOComponent::received(const MotorState motor_state)
{
    ESP_LOGD(TAG, "Motor: state=%s",
        LOG_STR_ARG(MotorState_to_string(*this->motor_state)));
    this->motor_state = motor_state;
}

void RATGDOComponent::received(const ButtonState button_state)
{
    ESP_LOGD(TAG, "Button state=%s",
        LOG_STR_ARG(ButtonState_to_string(*this->button_state)));
    this->button_state = button_state;
}

void RATGDOComponent::received(const MotionState motion_state)
{
    ESP_LOGD(TAG, "Motion: %s",
        LOG_STR_ARG(MotionState_to_string(*this->motion_state)));
    this->motion_state = motion_state;
    if (motion_state == MotionState::DETECTED) {
        this->set_timeout(TIMEOUT_CLEAR_MOTION, 3000,
            [this] { this->motion_state = MotionState::CLEAR; });
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
    ESP_LOGD(TAG, "Paired device count, kind=%s count=%d",
        LOG_STR_ARG(PairedDevice_to_string(pdc.kind)), pdc.count);

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

// The TTC_SET_LIMIT message is only transmitted by the wall control
// when the user is changing the limit. So it might only ever be
// transmitted one time when the GDO is installed. Therefore it might
// seem hard to determine the ttc_limit. However, the ttc limit can
// be inferred from the TTC_COUNTDOWN messages, because the first
// TTC_COUNTDOWN broadcast of a cycle is equal to the configured
// ttc_limit.
//
// So RATGDO captures and remembers the first TTC_COUNTDOWN value
// after a door open (or release) as the ttc_limit value.
// Alternatively, if a TTC_COUNTDOWN message contains a value larger
// than the currently saved ttc_limit, then the saved ttc_limit must
// be out of date, and it is updated accordingly.
//
// NOTE: The value of ttc_limit is only stored in RAM, so it is lost
// during a reboot. If a reboot happens in the middle of a ttc
// countdown, there's no way to know the true limit value (the limit
// may have changed while ratgdo was offline). The logic described
// above will incorrectly capture a too-small limit for this
// cycle. This is acceptable because it self-corrects on the very next
// release or door-open cycle, both of which set ttc_limit_learned=false
// and let a fresh broadcast capture the real value.
void RATGDOComponent::received(const TtcLimit limit)
{
    ESP_LOGD(TAG, "Time to close (TTC) limit: %ds", limit.seconds);
    this->ttc_limit = limit.seconds;
    this->flags_.ttc_limit_learned = true;
}

void RATGDOComponent::received(const TtcCountdown countdown)
{
    ESP_LOGD(TAG, "TTC countdown broadcast: %ds remaining", countdown.seconds);
    auto ds = *this->door_state;
    if ((!this->flags_.ttc_limit_learned && (ds == DoorState::OPENING || ds == DoorState::OPEN))
        || countdown.seconds > *this->ttc_limit) {
        this->ttc_limit = countdown.seconds;
        this->flags_.ttc_limit_learned = true;
    }
    this->start_or_sync_ttc_countdown(countdown.seconds);
}

void RATGDOComponent::received(const TtcToggleHold)
{
    ESP_LOGD(TAG, "TTC_TOGGLE_HOLD observed");
    this->apply_ttc_toggle();
}

void RATGDOComponent::received(const BatteryState battery_state)
{
    ESP_LOGD(TAG, "Battery state=%s",
        LOG_STR_ARG(BatteryState_to_string(battery_state)));
}

void RATGDOComponent::schedule_door_position_sync(float update_period)
{
    ESP_LOG1(
        TAG,
        "Schedule position sync: delta %f, start position: %f, start moving: %d",
        this->door_move_delta, this->door_start_position,
        this->door_start_moving);
    auto duration = this->door_move_delta > 0 ? *this->opening_duration
                                              : *this->closing_duration;
    if (duration == 0) {
        return;
    }
    this->position_sync_remaining_ = std::max(static_cast<uint16_t>(1000 * duration / update_period),
        static_cast<uint16_t>(1));
    set_interval(INTERVAL_POSITION_SYNC, static_cast<uint32_t>(update_period),
        [this]() {
            this->door_position_update();
            if (--this->position_sync_remaining_ == 0) {
                cancel_interval(INTERVAL_POSITION_SYNC);
            }
        });
}

void RATGDOComponent::door_position_update()
{
    if (this->door_start_moving == 0 || this->door_start_position == DOOR_POSITION_UNKNOWN || this->door_move_delta == DOOR_DELTA_UNKNOWN) {
        return;
    }
    auto now = millis();
    auto duration = this->door_move_delta > 0 ? *this->opening_duration
                                              : -*this->closing_duration;
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

    if (percent >= PRESENCE_DETECTION_ON_THRESHOLD)
        this->vehicle_detected_state = VehicleDetectedState::YES;

    if (percent == 0 && *this->vehicle_detected_state == VehicleDetectedState::YES) {
        this->presence_off_counter_++;
        ESP_LOGD(TAG, "Off counter: %d", this->presence_off_counter_);

        if (this->presence_off_counter_ / this->in_range.size() >= PRESENCE_DETECTION_OFF_DEBOUNCE) {
            this->presence_off_counter_ = 0;
            this->vehicle_detected_state = VehicleDetectedState::NO;
        }
    }

    if (percent != this->last_presence_percent_) {
        ESP_LOGD(TAG, "pct_in_range: %d", percent);
        this->last_presence_percent_ = percent;
        this->presence_off_counter_ = 0;
    }
    // ESP_LOGD(TAG, "in_range: %s", this->in_range.to_string().c_str());
}
#endif

#ifdef RATGDO_USE_VEHICLE_SENSORS
void RATGDOComponent::presence_change(bool sensor_value)
{
    if (this->flags_.presence_detect_window_active) {
        // Arriving and leaving are mutually exclusive — each branch clears the
        // other state. Sharing TIMEOUT_CLEAR_PRESENCE ensures that switching from
        // arriving to leaving (or vice versa) cancels the previous clear timeout,
        // which is correct since the previous state was already cleared above.
        if (sensor_value) {
            this->vehicle_arriving_state = VehicleArrivingState::YES;
            this->vehicle_leaving_state = VehicleLeavingState::NO;
            this->set_timeout(TIMEOUT_CLEAR_PRESENCE, CLEAR_PRESENCE, [this] {
                this->vehicle_arriving_state = VehicleArrivingState::NO;
            });
        } else {
            this->vehicle_arriving_state = VehicleArrivingState::NO;
            this->vehicle_leaving_state = VehicleLeavingState::YES;
            this->set_timeout(TIMEOUT_CLEAR_PRESENCE, CLEAR_PRESENCE, [this] {
                this->vehicle_leaving_state = VehicleLeavingState::NO;
            });
        }
        // if the door is closed, clear the presence detect window since a vehicle
        // can't be arriving or leaving with the door shut
        if (*this->door_state == DoorState::CLOSED) {
            this->flags_.presence_detect_window_active = false;
            this->cancel_timeout(TIMEOUT_PRESENCE_DETECT_WINDOW);
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
    // Safe to use cached loop timestamp here because obstruction_loop()
    // runs before protocol_->loop() which contains the 1.3ms blocking
    // transmit in secplus2. The 50ms CHECK_PERIOD has ample margin.
    const uint32_t current_millis = App.get_loop_component_start_time();
    static uint32_t last_millis = 0;
    static uint32_t last_asleep = 0;

    // the obstruction sensor has 3 states: clear (HIGH with LOW pulse every 7ms),
    // obstructed (HIGH), asleep (LOW) the transitions between awake and asleep
    // are tricky because the voltage drops slowly when falling asleep and is high
    // without pulses when waking up

    // If at least 3 low pulses are counted within 50ms, the door is awake, not
    // obstructed and we don't have to check anything else

    constexpr uint32_t CHECK_PERIOD = 50;
    constexpr uint32_t PULSES_LOWER_LIMIT = 3;

    if (current_millis - last_millis > CHECK_PERIOD) {
        // ESP_LOGD(TAG, "%ld: Obstruction count: %d, expected: %d, since asleep:
        // %ld",
        //     current_millis, this->isr_store_.obstruction_low_count,
        //     PULSES_LOWER_LIMIT, current_millis - last_asleep
        // );

        // check to see if we got more then PULSES_LOWER_LIMIT pulses
        if (this->isr_store_.obstruction_low_count > PULSES_LOWER_LIMIT) {
            this->obstruction_state = ObstructionState::CLEAR;
            this->flags_.obstruction_sensor_detected = true;
        } else if (this->isr_store_.obstruction_low_count == 0) {
            // if there have been no pulses the line is steady high or low
            if (this->input_obst_pin_->digital_read() != this->flags_.obst_sleep_low) {
                // asleep
                last_asleep = current_millis;
            } else {
                // if the line is high and was last asleep more than 700ms ago, then
                // there is an obstruction present
                if (current_millis - last_asleep > 700) {
                    this->obstruction_state = ObstructionState::OBSTRUCTED;
                }
            }
        }
        last_millis = current_millis;
        this->isr_store_.obstruction_low_count = 0;
    }
}

void RATGDOComponent::query_status() { this->protocol_->call(QueryStatus { }); }

void RATGDOComponent::query_openings()
{
    this->protocol_->call(QueryOpenings { });
}

void RATGDOComponent::query_paired_devices()
{
    this->protocol_->call(QueryPairedDevicesAll { });
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
    // needed to trigger the initial state of the limit switch sensors
    // ideally this would be in drycontact::sync
#ifdef PROTOCOL_DRYCONTACT
#ifdef RATGDO_USE_ENCODER
    if (this->encoder_sensor_ != nullptr) {
        // Power-loss detection: if the saved position is outside calibrated bounds,
        // the door was moved while powered off. Clear calibration and re-learn.
        if (flags_.enc_first_update) {
            flags_.enc_first_update = false;
            bool contradiction = false;
            if (!flags_.reverse_encoder) {
                if (flags_.enc_min_cal && enc_last_ < enc_min_)
                    contradiction = true;
                if (flags_.enc_max_cal && enc_last_ > enc_max_)
                    contradiction = true;
            } else {
                if (flags_.enc_min_cal && enc_last_ > enc_min_)
                    contradiction = true;
                if (flags_.enc_max_cal && enc_last_ < enc_max_)
                    contradiction = true;
            }
            if (contradiction) {
                ESP_LOGW(TAG, "Encoder moved while powered off; clearing calibration");
                reset_encoder_calibration();
            }
            // Publish the NVS-restored position
            encoder_sensor_->publish_state(static_cast<float>(enc_last_));
        }
        // Encoder mode: derive door state from saved calibration, no limit switches.
        if (this->flags_.enc_min_cal && this->flags_.enc_max_cal && this->enc_max_ != this->enc_min_) {
            int16_t range = static_cast<int16_t>(std::abs(this->enc_max_ - this->enc_min_));
            int16_t target_closed = this->flags_.reverse_encoder ? this->enc_max_ : this->enc_min_;
            int16_t target_open = this->flags_.reverse_encoder ? this->enc_min_ : this->enc_max_;
            int16_t dist_closed = static_cast<int16_t>(std::abs(this->enc_last_ - target_closed));
            int16_t dist_open = static_cast<int16_t>(std::abs(this->enc_last_ - target_open));
            (void)range;
            float pos = (float)(this->enc_last_ - this->enc_min_) / (float)(this->enc_max_ - this->enc_min_);
            if (this->flags_.reverse_encoder)
                pos = 1.0f - pos;
            this->door_position = clamp(pos, 0.0f, 1.0f);
            if (dist_closed <= 1 && dist_closed <= dist_open) {
                this->received(DoorState::CLOSED);
            } else if (dist_open <= 1 && dist_open < dist_closed) {
                this->received(DoorState::OPEN);
            } else {
                this->received(DoorState::STOPPED);
            }
        } else {
            ESP_LOGD(TAG, "Encoder not yet calibrated; door state unknown");
        }
        return; // skip limit-switch path
    }
#endif
    if (this->dry_contact_open_sensor_ != nullptr) {
        this->protocol_->set_open_limit(this->dry_contact_open_sensor_->state);
    }
    if (this->dry_contact_close_sensor_ != nullptr) {
        this->protocol_->set_close_limit(this->dry_contact_close_sensor_->state);
    }
#endif
}

void RATGDOComponent::set_door_state_expiry()
{
    this->set_timeout(TIMEOUT_DOOR_STATE_EXPIRY, DOOR_STATE_CALLBACK_TIMEOUT,
        [this]() {
            ESP_LOGW(TAG, "Door state callback expired, clearing");
            this->on_door_state_.clear();
        });
}

void RATGDOComponent::cancel_door_state_expiry()
{
    this->cancel_timeout(TIMEOUT_DOOR_STATE_EXPIRY);
}

void RATGDOComponent::door_open()
{
    if (*this->door_state == DoorState::OPENING) {
        return; // gets ignored by opener
    }

#ifdef RATGDO_USE_ENCODER
    // Record intended direction so on_encoder_update can detect a wrong-way GDO response.
    enc_intended_dir_ = 1;
#endif
    this->door_action(DoorAction::OPEN);

    if (*this->opening_duration > 0) {
        // query state in case we don't get a status message
        this->set_timeout(
            TIMEOUT_DOOR_QUERY_STATE, (*this->opening_duration + 2) * 1000,
            [this]() {
                if (*this->door_state != DoorState::OPEN
                    && *this->door_state != DoorState::STOPPED
#ifdef RATGDO_USE_ENCODER
                    // If the encoder confirmed CLOSED the door did not open silently;
                    // don't override with a spurious OPEN assumption.
                    && *this->door_state != DoorState::CLOSED
#endif
                ) {
                    this->received(DoorState::OPEN); // probably missed a status message,
                                                     // assume it's open
                    this->query_status(); // query in case we're wrong and it's stopped
                }
            });
    }
}

void RATGDOComponent::door_close()
{
    if (*this->door_state == DoorState::CLOSING) {
        // Door is already heading in the right direction. If the user explicitly
        // requested CLOSE (e.g., after a partial move_to_position), remember the intent
        // so we retry once the door comes to rest (the move_to_position timer may
        // stop it before it reaches fully closed).
#ifdef RATGDO_USE_ENCODER
        enc_intended_dir_ = -1;
        this->on_door_state([this](DoorState s) {
            if (s == DoorState::STOPPED) {
                this->door_close();
            }
        });
#endif
        return;
    }

    if (*this->door_state == DoorState::OPENING) {
        // have to stop door first, otherwise close command is ignored
        this->door_action(DoorAction::STOP);
        this->on_door_state([this](DoorState s) {
            if (s == DoorState::STOPPED) {
                this->door_action(DoorAction::CLOSE);
            } else {
                ESP_LOGW(TAG, "Door did not stop, ignoring close command");
            }
        });
        return;
    }

#ifdef RATGDO_USE_ENCODER
    // Record intended direction so on_encoder_update can detect a wrong-way GDO response.
    // Set unconditionally here — mirrors door_open() — so the intent is captured regardless
    // of whether an obstruction sensor is present.
    enc_intended_dir_ = -1;
#endif
    if (this->flags_.obstruction_sensor_detected) {
        this->door_action(DoorAction::CLOSE);
    } else if (*this->door_state == DoorState::OPEN || *this->door_state == DoorState::STOPPED) {
        // ROW openers without obstruction sensors ignore explicit CLOSE commands.
        // As a compromise, when STOPPED we fallback to TOGGLE. The GDO will typically
        // reverse its last direction, so we cannot guarantee it will actually
        // close instead of opening, but it is better than silently failing.
        ESP_LOGD(TAG, "No obstruction sensors detected. Close using TOGGLE.");
        this->door_action(DoorAction::TOGGLE);
    }

    if (*this->closing_duration > 0) {
        // query state in case we don't get a status message
        this->set_timeout(
            TIMEOUT_DOOR_QUERY_STATE, (*this->closing_duration + 2) * 1000,
            [this]() {
                if (*this->door_state != DoorState::CLOSED
                    && *this->door_state != DoorState::STOPPED
                    && *this->door_state != DoorState::OPEN) {
                    this->received(DoorState::CLOSED); // probably missed a status
                                                       // message, assume it's closed
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

void RATGDOComponent::door_toggle() { this->door_action(DoorAction::TOGGLE); }

void RATGDOComponent::door_action(DoorAction action)
{
#ifdef RATGDO_USE_CLOSING_DELAY
    if (*this->closing_delay > 0 && (action == DoorAction::CLOSE || (action == DoorAction::TOGGLE && *this->door_state != DoorState::CLOSED))) {
        this->door_action_delayed = DoorActionDelayed::YES;
        this->set_timeout(TIMEOUT_DOOR_ACTION, *this->closing_delay * 1000, [this, action] {
            this->door_action_delayed = DoorActionDelayed::NO;
            this->protocol_->door_action(action);
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
        this->on_door_state([this, position](DoorState s) {
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
    ESP_LOGD(TAG, "Moving to position %.2f in %.1fs", position,
        operation_time / 1000.0);

#ifdef RATGDO_USE_ENCODER
    // Record intended direction so on_encoder_update can detect a wrong-way GDO response.
    // door_move_to_position calls door_action() directly (not door_open/door_close)
    // so we must set this here as well.
    enc_intended_dir_ = (delta > 0) ? 1 : -1;
#endif
    this->door_action(delta > 0 ? DoorAction::OPEN : DoorAction::CLOSE);
    this->set_timeout(TIMEOUT_MOVE_TO_POSITION, operation_time,
        [this] {
#ifdef RATGDO_USE_ENCODER
            flags_.enc_position_stop_pending = true;
#endif
            this->door_action(DoorAction::STOP);
        });
}

void RATGDOComponent::cancel_position_sync_callbacks()
{
    if (this->door_start_moving != 0) {
        ESP_LOGD(TAG, "Cancelling position callbacks");
        this->cancel_timeout(TIMEOUT_MOVE_TO_POSITION);
        cancel_interval(INTERVAL_POSITION_SYNC);

        this->door_start_moving = 0;
        this->door_start_position = DOOR_POSITION_UNKNOWN;
        this->door_move_delta = DOOR_DELTA_UNKNOWN;
        // enc_intended_dir_ is intentionally NOT cleared here.
        // It persists until the direction is confirmed (first correct-direction encoder
        // tick clears it) or the door reaches the commanded boundary (received(OPEN/CLOSED)).
        // Clearing it here was wiping intent set by a close/open command that arrived
        // just before the door came to rest.
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

void RATGDOComponent::ttc_toggle_hold()
{
    if (ttc_is_unknown(*this->ttc_state)) {
        // Don't transmit while we haven't observed any TTC activity on the
        // wire this cycle: we can't meaningfully pause or resume something
        // we have no confirmed state for.
        return;
    }
    ESP_LOGD(TAG, "Toggle TTC");
    this->protocol_->call(TtcToggleHoldTx { });
    this->apply_ttc_toggle();
}

// Shared pause/release state-machine transition for TTC hold, applied
// whether the toggle was observed on the wire (from the wall panel) or
// initiated by RATGDO.
void RATGDOComponent::apply_ttc_toggle()
{
    if (ttc_is_counting(*this->ttc_state)) {
        // Pause the countdown: stop decrementing locally, cancel the
        // countdown watchdog, and go to HOLDING state.
        this->cancel_timeout(scheduler_ids::TTC_COUNTDOWN_WATCHDOG);
        this->cancel_interval(scheduler_ids::TTC_COUNTDOWN_LOCAL_DECREMENT);
        this->ttc_countdown = 0;
        this->ttc_state = TtcState::HOLDING;
    } else if (*this->ttc_state == TtcState::HOLDING) {
        // Release hold. Restart the local countdown and refresh the TTC
        // limit from the next countdown broadcast.
        this->flags_.ttc_limit_learned = false;
        this->start_or_sync_ttc_countdown(*this->ttc_limit);
    }
}

// Learn functions
void RATGDOComponent::activate_learn()
{
    this->protocol_->call(ActivateLearn { });
}

void RATGDOComponent::inactivate_learn()
{
    this->protocol_->call(InactivateLearn { });
}

// Subscribe implementations are now templates in ratgdo.h

// dry contact methods
void RATGDOComponent::set_dry_contact_open_sensor(
    esphome::binary_sensor::BinarySensor* dry_contact_open_sensor)
{
    dry_contact_open_sensor_ = dry_contact_open_sensor;
    dry_contact_open_sensor_->add_on_state_callback([this](bool sensor_value) {
        this->protocol_->set_open_limit(sensor_value);
        this->door_position = 1.0;
    });
}

void RATGDOComponent::set_dry_contact_close_sensor(
    esphome::binary_sensor::BinarySensor* dry_contact_close_sensor)
{
    dry_contact_close_sensor_ = dry_contact_close_sensor;
    dry_contact_close_sensor_->add_on_state_callback([this](bool sensor_value) {
        this->protocol_->set_close_limit(sensor_value);
        this->door_position = 0.0;
    });
}

#ifdef RATGDO_USE_ENCODER
void IRAM_ATTR HOT RATGDOStore::isr_encoder(RATGDOStore* arg)
{
    // Quadrature encoder lookup table
    // Index = (prev_state << 2) | curr_state, where state = (a << 1) | b
    // Maps every state transition to +1 (CW) or -1 (CCW)
    // Invalid transitions (skip-2 states from excessive bounce) map to 0
    static const int8_t ENC_TABLE[16] = {
        0,
        -1,
        +1,
        0, // prev=00 → {00,01,10,11}
        +1,
        0,
        0,
        -1, // prev=01 → {00,01,10,11}
        -1,
        0,
        0,
        +1, // prev=10 → {00,01,10,11}
        0,
        +1,
        -1,
        0, // prev=11 → {00,01,10,11}
    };
    bool a = arg->enc_pin_a.digital_read();
    bool b = arg->enc_pin_b.digital_read();
    uint8_t curr = (static_cast<uint8_t>(a) << 1) | static_cast<uint8_t>(b);
    int8_t step = ENC_TABLE[(arg->enc_prev_state << 2) | curr];
    arg->enc_prev_state = curr;

    if (step == 0)
        return; // invalid/skip-2 transition; update prev_state but don't count

    // Net running sum: accumulate signed steps and emit when the dominant
    // direction has built up 4 net counts. This tolerates an occasional
    // wrong-direction transition from noise.
    arg->enc_cycle_count += step;
    if (arg->enc_cycle_count >= 4) {
        arg->enc_delta += 1;
        arg->enc_cycle_count = 0;
    } else if (arg->enc_cycle_count <= -4) {
        arg->enc_delta -= 1;
        arg->enc_cycle_count = 0;
    }
}

void RATGDOComponent::set_encoder_sensor(esphome::sensor::Sensor* s)
{
    encoder_sensor_ = s;
    flags_.enc_first_update = 1; // flags_ zero-inits; set explicitly to match old { true } initialiser
    encoder_pref_ = global_preferences->make_preference<RATGDOEncoderSettings>(fnv1_hash("ratgdo_encoder"));
    RATGDOEncoderSettings saved;
    if (encoder_pref_.load(&saved)) {
        enc_min_ = saved.min;
        enc_max_ = saved.max;
        enc_last_ = saved.last;
        flags_.enc_min_cal = saved.min_calibrated;
        flags_.enc_max_cal = saved.max_calibrated;
        ESP_LOGD(TAG, "Encoder: loaded cal min=%d max=%d last=%d min_cal=%d max_cal=%d",
            enc_min_, enc_max_, enc_last_, flags_.enc_min_cal, flags_.enc_max_cal);
    } else {
        ESP_LOGD(TAG, "Encoder: no saved calibration");
    }
}

void RATGDOComponent::on_encoder_update(int16_t raw)
{
    int16_t delta = static_cast<int16_t>(raw - enc_last_);
    enc_last_ = raw;

    if (delta == 0)
        return;

    encoder_sensor_->publish_state(static_cast<float>(raw));

    // Track direction so check_encoder_stopped knows which boundary we hit.
    enc_last_dir_ = (delta > 0) ? 1 : -1;

    // Latch the travel direction from the first step of each move.
    // Subsequent steps opposite to the dominant direction are counted; only after
    // ENC_DIRECTION_CHANGE_THRESHOLD consecutive opposite steps is enc_travel_dir_
    // updated, filtering oscillations
    if (enc_travel_dir_ == 0) {
        enc_travel_dir_ = enc_last_dir_; // first step of a new move
        enc_reverse_count_ = 0;
    } else if (enc_last_dir_ != enc_travel_dir_) {
        if (++enc_reverse_count_ >= ENC_DIRECTION_CHANGE_THRESHOLD) {
            enc_travel_dir_ = enc_last_dir_; // confirmed real reversal
            enc_reverse_count_ = 0;
        }
    } else {
        enc_reverse_count_ = 0; // step agrees with dominant direction; reset counter
    }

    ESP_LOGD(TAG, "Encoder: step=%d min=%d max=%d", raw, enc_min_, enc_max_);

    if (flags_.enc_min_cal && flags_.enc_max_cal && enc_max_ != enc_min_) {
        int16_t dist_closed = static_cast<int16_t>(std::abs(raw - enc_min_));
        int16_t dist_open = static_cast<int16_t>(std::abs(raw - enc_max_));
        float pos;
        if (dist_closed <= 1 && dist_closed <= dist_open) {
            pos = flags_.reverse_encoder ? 1.0f : 0.0f;
        } else if (dist_open <= 1 && dist_open < dist_closed) {
            pos = flags_.reverse_encoder ? 0.0f : 1.0f;
        } else {
            pos = (float)(raw - enc_min_) / (float)(enc_max_ - enc_min_);
            if (flags_.reverse_encoder)
                pos = 1.0f - pos;
        }
        this->door_position = clamp(pos, 0.0f, 1.0f);

        // Derive in_motion from enc_travel_dir_ (the confirmed dominant direction)
        // rather than enc_last_dir_ so that oscillation noise does not flip the
        // reported door state or cancel the move-to-position timer.
        // enc_travel_dir_ only changes after ENC_DIRECTION_CHANGE_THRESHOLD
        // consecutive opposite steps.
        int8_t effective_dir = (enc_travel_dir_ != 0) ? enc_travel_dir_ : enc_last_dir_;
        DoorState in_motion = (effective_dir > 0)
            ? (flags_.reverse_encoder ? DoorState::CLOSING : DoorState::OPENING)
            : (flags_.reverse_encoder ? DoorState::OPENING : DoorState::CLOSING);

        // Check if the door moved in the opposite direction from what was commanded.
#if ENC_DIRECTION_CORRECTION_ENABLED
        if (enc_intended_dir_ != 0) {
            bool correct = (in_motion == DoorState::OPENING) == (enc_intended_dir_ > 0);
            if (!correct) {
                int8_t intended = enc_intended_dir_;
                enc_intended_dir_ = 0; // clear — correction is firing
                ESP_LOGD(TAG, "Wrong direction detected (wanted %s, got %s); stopping to correct",
                    intended > 0 ? "open" : "close",
                    in_motion == DoorState::OPENING ? "opening" : "closing");
                this->set_timeout(500, [this] {
                    this->door_action(DoorAction::STOP);
                });
                // Cancel the door_open/door_close query-state safety timer so it
                // doesn't fire a spurious OPEN/CLOSED state after the correction cycle.
                this->cancel_timeout(TIMEOUT_DOOR_QUERY_STATE);
                // Defer the retry to check_encoder_stopped()
                enc_dir_correction_pending_ = true;
                enc_dir_correction_intended_ = intended;
            }
            // If correct direction: do NOT clear enc_intended_dir_ here.
            // It stays set so a mid-travel reversal (confirmed after
            // ENC_DIRECTION_CHANGE_THRESHOLD opposite ticks) can still trigger
            // the correction. check_encoder_stopped() clears it when the move ends.
        }
#endif // ENC_DIRECTION_CORRECTION_ENABLED
        this->encoder_received(in_motion);
    }

    // Re-arm the stopped watchdog after the last pulse
    set_timeout(scheduler_ids::TIMEOUT_ENCODER_STOPPED, ENC_STOPPED_WATCHDOG_MS,
        [this] { this->check_encoder_stopped(); });
}

void RATGDOComponent::check_encoder_stopped()
{
    ESP_LOGI(TAG, "Encoder stopped: step=%d min=%d max=%d dir=%d", enc_last_, enc_min_, enc_max_, enc_travel_dir_);
    bool update_pref = false;

    // Use the latched travel direction rather than enc_last_dir_ so that
    // magnet-hover oscillations at a limit do not corrupt boundary classification.
    const bool decreasing = (enc_travel_dir_ < 0);

    // Clear enc_travel_dir_ now so the next move starts with a fresh latch.
    enc_travel_dir_ = 0;
    enc_reverse_count_ = 0;
    // Clear enc_intended_dir_ so a stale intent from a previous ratgdo command
    // cannot trigger the wrong-direction correction on a subsequent wall-control command
    enc_intended_dir_ = 0;
    const DoorState boundary_state = decreasing
        ? (flags_.reverse_encoder ? DoorState::OPEN : DoorState::CLOSED)
        : (flags_.reverse_encoder ? DoorState::CLOSED : DoorState::OPEN);

    if ((!flags_.enc_min_cal && decreasing) || (!flags_.enc_max_cal && !decreasing)) {
        // First time seeing this boundary direction — calibrate it.
        if (decreasing) {
            enc_min_ = enc_last_;
            flags_.enc_min_cal = true;
        } else {
            enc_max_ = enc_last_;
            flags_.enc_max_cal = true;
        }
        update_pref = true;
        this->encoder_received(boundary_state);
        ESP_LOGD(TAG, "Encoder: initial %s boundary set to %d",
            decreasing ? "lower(min)" : "upper(max)", enc_last_);
    } else if (!flags_.enc_min_cal || !flags_.enc_max_cal) {
        // Hit the same direction twice before the other end was seen — update this end.
        if (decreasing) {
            enc_min_ = enc_last_;
            flags_.enc_min_cal = true;
        } else {
            enc_max_ = enc_last_;
            flags_.enc_max_cal = true;
        }
        update_pref = true;
        this->encoder_received(boundary_state);
        ESP_LOGD(TAG, "Encoder: re-set %s boundary to %d",
            decreasing ? "lower(min)" : "upper(max)", enc_last_);
    } else {
        // Both boundaries calibrated. If this stop was triggered by TIMEOUT_MOVE_TO_POSITION
        // (a mid-travel position command) the door may have landed within 1 step of a
        // boundary purely by coincidence. Skip snap/extension so we don't corrupt calibration.
        if (flags_.enc_position_stop_pending) {
            flags_.enc_position_stop_pending = false;
            this->encoder_received(DoorState::STOPPED);
            return;
        }

        // Check if we stopped within 1 pulse of a limit.
        int16_t target_closed = flags_.reverse_encoder ? enc_max_ : enc_min_;
        int16_t target_open = flags_.reverse_encoder ? enc_min_ : enc_max_;
        int16_t dist_closed = static_cast<int16_t>(std::abs(enc_last_ - target_closed));
        int16_t dist_open = static_cast<int16_t>(std::abs(enc_last_ - target_open));

        // Define approaching flags dynamically
        const bool approaching_closed = (decreasing == !flags_.reverse_encoder);
        const bool approaching_open = (decreasing == flags_.reverse_encoder);

        // A door moving 'beyond' a limit is just moving towards it and overshooting.
        // So beyond_open means it is approaching OPEN and overshot.
        const bool beyond_open = approaching_open && (decreasing ? (enc_last_ < enc_min_) : (enc_last_ > enc_max_));
        const bool beyond_closed = approaching_closed && (decreasing ? (enc_last_ < enc_min_) : (enc_last_ > enc_max_));

        if (dist_closed <= 1 && dist_closed <= dist_open && approaching_closed) {
            // Snap CLOSED boundary — only when approaching CLOSED.
            if (decreasing)
                enc_min_ = enc_last_;
            else
                enc_max_ = enc_last_;
            update_pref = true;
            ESP_LOGI(TAG, "Encoder: CLOSED boundary snapped to %d (min=%d max=%d)", enc_last_, enc_min_, enc_max_);
            this->encoder_received(DoorState::CLOSED);
        } else if (dist_open <= 1 && dist_open < dist_closed && approaching_open) {
            // Snap OPEN boundary — only when approaching OPEN.
            if (decreasing)
                enc_min_ = enc_last_;
            else
                enc_max_ = enc_last_;
            update_pref = true;
            ESP_LOGI(TAG, "Encoder: OPEN boundary snapped to %d (min=%d max=%d)", enc_last_, enc_min_, enc_max_);
            this->encoder_received(DoorState::OPEN);
        } else if (beyond_open) {
            // Door stopped past the known OPEN boundary — extend it and snap OPEN.
            if (decreasing)
                enc_min_ = enc_last_;
            else
                enc_max_ = enc_last_;
            update_pref = true;
            ESP_LOGI(TAG, "Encoder: OPEN boundary extended to %d (min=%d max=%d)", enc_last_, enc_min_, enc_max_);
            this->encoder_received(DoorState::OPEN);
        } else if (beyond_closed) {
            // Door stopped past the known CLOSED boundary — extend it and snap CLOSED.
            if (decreasing)
                enc_min_ = enc_last_;
            else
                enc_max_ = enc_last_;
            update_pref = true;
            ESP_LOGI(TAG, "Encoder: CLOSED boundary extended to %d (min=%d max=%d)", enc_last_, enc_min_, enc_max_);
            this->encoder_received(DoorState::CLOSED);
        } else {
            this->encoder_received(DoorState::STOPPED);
        }
    }

    if (update_pref) {
        RATGDOEncoderSettings s;
        s.min = enc_min_;
        s.max = enc_max_;
        s.last = enc_last_;
        s.min_calibrated = flags_.enc_min_cal;
        s.max_calibrated = flags_.enc_max_cal;
        encoder_pref_.save(&s);
    }

#if ENC_DIRECTION_CORRECTION_ENABLED
    // If a wrong-direction correction was pending, retry the intended action now that
    // the encoder has confirmed the door has actually stopped.
    if (enc_dir_correction_pending_) {
        enc_dir_correction_pending_ = false;
        int8_t intended = enc_dir_correction_intended_;
        enc_dir_correction_intended_ = 0;
        auto action = (intended > 0) ? DoorAction::OPEN : DoorAction::CLOSE;
        ESP_LOGD(TAG, "Direction correction retry: door stopped, sending %s",
            intended > 0 ? "OPEN" : "CLOSE");
        this->door_action(action);
    }
#endif
}

void RATGDOComponent::reset_encoder_calibration()
{
    enc_min_ = enc_max_ = enc_last_ = 0;
    flags_.enc_min_cal = flags_.enc_max_cal = false;
    enc_travel_dir_ = 0;
    enc_reverse_count_ = 0;
    RATGDOEncoderSettings s;
    s.min = 0;
    s.max = 0;
    s.last = 0;
    s.min_calibrated = false;
    s.max_calibrated = false;
    encoder_pref_.save(&s);
    if (encoder_sensor_ != nullptr)
        encoder_sensor_->publish_state(0.0f);
    ESP_LOGI(TAG, "Encoder calibration and step counter cleared; will re-learn on next full open/close cycle");
}

void RATGDOComponent::encoder_apply_state(int16_t raw)
{
    int16_t target_closed = flags_.reverse_encoder ? enc_max_ : enc_min_;
    int16_t target_open = flags_.reverse_encoder ? enc_min_ : enc_max_;
    int16_t dist_closed = static_cast<int16_t>(std::abs(raw - target_closed));
    int16_t dist_open = static_cast<int16_t>(std::abs(raw - target_open));

    if (enc_max_ != enc_min_) {
        float pos = (float)(raw - enc_min_) / (float)(enc_max_ - enc_min_);
        if (flags_.reverse_encoder)
            pos = 1.0f - pos;
        this->door_position = clamp(pos, 0.0f, 1.0f);
    }

    ESP_LOGI(TAG, "Encoder: step=%d min=%d max=%d dist_closed=%d dist_open=%d reversed=%d",
        raw, enc_min_, enc_max_, dist_closed, dist_open, flags_.reverse_encoder);

    if (dist_closed <= 1 && dist_closed <= dist_open) {
        this->received(DoorState::CLOSED);
    } else if (dist_open <= 1 && dist_open < dist_closed) {
        this->received(DoorState::OPEN);
    } else {
        this->received(DoorState::STOPPED);
    }
}

void RATGDOComponent::recalculate_encoder_state()
{
    if (flags_.enc_min_cal && flags_.enc_max_cal && enc_max_ != enc_min_)
        encoder_apply_state(enc_last_);
}
#endif

} // namespace esphome::ratgdo
