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

#pragma once

#ifndef ENC_DIRECTION_CORRECTION_ENABLED
#if defined(PROTOCOL_DRYCONTACT)
#define ENC_DIRECTION_CORRECTION_ENABLED 1 // Set to 0 to disable the wrong-direction stop-and-retry logic
#else
#define ENC_DIRECTION_CORRECTION_ENABLED 0
#endif
#endif

#include "esphome/components/binary_sensor/binary_sensor.h"
#ifdef RATGDO_USE_ENCODER
#include "esphome/components/sensor/sensor.h"
#endif
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"

#include <bitset>
#include <type_traits>
#include <utility>

#include "callbacks.h"
#include "macros.h"
#include "observable.h"
#include "protocol.h"
#include "ratgdo_state.h"

// Observable subscriber counts — set by Python codegen via cg.add_define().
// Missing defines are a build error to catch codegen issues early.
#ifndef RATGDO_MAX_DOOR_STATE_SUBSCRIBERS
#error "RATGDO_MAX_DOOR_STATE_SUBSCRIBERS must be defined by codegen"
#endif
#ifndef RATGDO_MAX_DOOR_ACTION_DELAYED_SUBSCRIBERS
#error "RATGDO_MAX_DOOR_ACTION_DELAYED_SUBSCRIBERS must be defined by codegen"
#endif
#ifndef RATGDO_MAX_DISTANCE_SUBSCRIBERS
#error "RATGDO_MAX_DISTANCE_SUBSCRIBERS must be defined by codegen"
#endif
#ifndef RATGDO_MAX_VEHICLE_DETECTED_SUBSCRIBERS
#error "RATGDO_MAX_VEHICLE_DETECTED_SUBSCRIBERS must be defined by codegen"
#endif
#ifndef RATGDO_MAX_VEHICLE_ARRIVING_SUBSCRIBERS
#error "RATGDO_MAX_VEHICLE_ARRIVING_SUBSCRIBERS must be defined by codegen"
#endif
#ifndef RATGDO_MAX_VEHICLE_LEAVING_SUBSCRIBERS
#error "RATGDO_MAX_VEHICLE_LEAVING_SUBSCRIBERS must be defined by codegen"
#endif

namespace esphome {
class InternalGPIOPin;
} // namespace esphome

namespace esphome::ratgdo {

class RATGDOComponent;
typedef Parented<RATGDOComponent> RATGDOClient;

const float DOOR_POSITION_UNKNOWN = -1.0;
const float DOOR_DELTA_UNKNOWN = -2.0;
const uint8_t PAIRED_DEVICES_UNKNOWN = 0xFF;

struct RATGDOStore {
    volatile uint32_t obstruction_low_count = 0; // count obstruction low pulses

    static void IRAM_ATTR HOT isr_obstruction(RATGDOStore* arg)
    {
        arg->obstruction_low_count++;
    }

#ifdef RATGDO_USE_ENCODER
    volatile int16_t enc_delta { 0 };
    volatile uint8_t enc_prev_state { 0 };
    // Signed net accumulator for ISR-level step filtering.
    // Accumulates +1/-1 per valid quadrature transition; emits a verified step
    // to enc_delta when the sum reaches ±4. Using a net sum (rather than a
    // consecutive-count reset-on-reversal) means a single stray wrong-direction
    // transition from noise delays the next emission by 2 counts
    volatile int8_t enc_cycle_count { 0 };
    ISRInternalGPIOPin enc_pin_a;
    ISRInternalGPIOPin enc_pin_b;
    static void IRAM_ATTR HOT isr_encoder(RATGDOStore* arg);
#endif
};

#ifdef RATGDO_USE_ENCODER
// Calibration data persisted in NVS.
// int16_t is sufficient: typical travel is 12-18 pulses so even with decades
// of floating-bound drift the values stay well within ±32767.
struct __attribute__((packed)) RATGDOEncoderSettings {
    int16_t min;
    int16_t max;
    int16_t last;
    uint8_t min_calibrated;
    uint8_t max_calibrated;
};
#endif

using protocol::Args;
using protocol::Result;

class RATGDOComponent : public Component {
public:
    RATGDOComponent()
    {
    }

    void setup() override;
    void loop() override;
    void dump_config() override;
    void on_shutdown() override;

    void init_protocol();

    void obstruction_loop();

    float start_opening { -1 };
    single_observable<float> opening_duration { 0 };
    float start_closing { -1 };
    single_observable<float> closing_duration { 0 };
#ifdef RATGDO_USE_CLOSING_DELAY
    single_observable<uint32_t> closing_delay { 0 };
#endif

#ifdef RATGDO_USE_DISTANCE_SENSOR
    single_observable<int16_t> target_distance_measurement { -1 };
    std::bitset<256> in_range; // the length of this bitset determines how many out of range readings are required for presence detection to change states
    observable<int16_t, RATGDO_MAX_DISTANCE_SUBSCRIBERS> last_distance_measurement { 0 };
#endif

    single_observable<uint16_t> openings { 0 }; // number of times the door has been opened
    single_observable<uint8_t> paired_total { PAIRED_DEVICES_UNKNOWN };
    single_observable<uint8_t> paired_remotes { PAIRED_DEVICES_UNKNOWN };
    single_observable<uint8_t> paired_keypads { PAIRED_DEVICES_UNKNOWN };
    single_observable<uint8_t> paired_wall_controls { PAIRED_DEVICES_UNKNOWN };
    single_observable<uint8_t> paired_accessories { PAIRED_DEVICES_UNKNOWN };

    observable<DoorState, RATGDO_MAX_DOOR_STATE_SUBSCRIBERS> door_state { DoorState::UNKNOWN };
    observable<float, RATGDO_MAX_DOOR_STATE_SUBSCRIBERS> door_position { DOOR_POSITION_UNKNOWN };
    observable<DoorActionDelayed, RATGDO_MAX_DOOR_ACTION_DELAYED_SUBSCRIBERS> door_action_delayed { DoorActionDelayed::NO };
#ifdef RATGDO_USE_ENCODER
    observable<ManuallyOperatedState, RATGDO_MAX_MANUALLY_OPERATED_SUBSCRIBERS> manually_operated_state { ManuallyOperatedState::NO };
#endif

    unsigned long door_start_moving { 0 };
    float door_start_position { DOOR_POSITION_UNKNOWN };
    float door_move_delta { DOOR_DELTA_UNKNOWN };
    uint16_t position_sync_remaining_ { 0 };

    single_observable<LightState> light_state { LightState::UNKNOWN };
    single_observable<LockState> lock_state { LockState::UNKNOWN };
    single_observable<ObstructionState> obstruction_state { ObstructionState::UNKNOWN };
    single_observable<MotorState> motor_state { MotorState::UNKNOWN };
    single_observable<ButtonState> button_state { ButtonState::UNKNOWN };
    single_observable<MotionState> motion_state { MotionState::UNKNOWN };
    single_observable<LearnState> learn_state { LearnState::UNKNOWN };

    single_observable<TtcState> ttc_state { TtcState::UNKNOWN };
    single_observable<uint16_t> ttc_countdown { 0 };
    single_observable<uint16_t> ttc_limit { 0 };

    static constexpr uint16_t TTC_COUNTDOWN_LOCAL_DECREMENT_INTERVAL = 5;

#ifdef RATGDO_USE_VEHICLE_SENSORS
    observable<VehicleDetectedState, RATGDO_MAX_VEHICLE_DETECTED_SUBSCRIBERS> vehicle_detected_state { VehicleDetectedState::NO };
    observable<VehicleArrivingState, RATGDO_MAX_VEHICLE_ARRIVING_SUBSCRIBERS> vehicle_arriving_state { VehicleArrivingState::NO };
    observable<VehicleLeavingState, RATGDO_MAX_VEHICLE_LEAVING_SUBSCRIBERS> vehicle_leaving_state { VehicleLeavingState::NO };
#endif

    OnceCallbacks<void(DoorState)> on_door_state_;

    single_observable<bool> sync_failed { false };

    void set_output_gdo_pin(InternalGPIOPin* pin) { this->output_gdo_pin_ = pin; }
    void set_input_gdo_pin(InternalGPIOPin* pin) { this->input_gdo_pin_ = pin; }
    void set_input_obst_pin(InternalGPIOPin* pin) { this->input_obst_pin_ = pin; }
    void set_obst_sleep_low(bool low) { this->flags_.obst_sleep_low = low; }

    // dry contact methods
    void set_dry_contact_open_sensor(esphome::binary_sensor::BinarySensor* dry_contact_open_sensor_);
    void set_dry_contact_close_sensor(esphome::binary_sensor::BinarySensor* dry_contact_close_sensor_);
    void set_discrete_open_pin(InternalGPIOPin* pin) { this->protocol_->set_discrete_open_pin(pin); }
    void set_discrete_close_pin(InternalGPIOPin* pin) { this->protocol_->set_discrete_close_pin(pin); }

#ifdef RATGDO_USE_ENCODER
    // encoder methods
    void set_encoder_sensor(esphome::sensor::Sensor* s);
    void set_reverse_encoder(bool r) { this->flags_.reverse_encoder = r; }
    void reset_encoder_calibration();
    void on_encoder_update(int16_t raw);
    void check_encoder_stopped();
    void recalculate_encoder_state();
    void encoder_apply_state(int16_t raw);
    void set_encoder_pin_a(InternalGPIOPin* pin) { enc_pin_a_ = pin; }
    void set_encoder_pin_b(InternalGPIOPin* pin) { enc_pin_b_ = pin; }
#endif

    Result call_protocol(Args args);

    void set_resolved_door_state(const DoorState door_state);
    void received(const DoorState door_state);
#ifdef RATGDO_USE_ENCODER
    void encoder_received(const DoorState door_state);
#endif
    void received(const LightState light_state);
    void received(const LockState lock_state);
    void received(const ObstructionState obstruction_state);
    void received(const LightAction light_action);
    void received(const MotorState motor_state);
    void received(const ButtonState button_state);
    void received(const MotionState motion_state);
    void received(const LearnState light_state);
    void received(const Openings openings);
    void received(const TtcLimit limit);
    void received(const TtcCountdown countdown);
    void received(const TtcToggleHold toggle_hold);
    void received(const PairedDeviceCount pdc);
    void received(const BatteryState pdc);

    // door
    void door_toggle();
    void door_open();
    void door_close();
    void door_stop();

    void door_action(DoorAction action);
    void ensure_door_action(DoorAction action, uint32_t delay = 1500);
    void door_move_to_position(float position);
    void set_door_position(float door_position) { this->door_position = door_position; }
    void set_opening_duration(float duration);
    void set_closing_duration(float duration);
#ifdef RATGDO_USE_CLOSING_DELAY
    void set_closing_delay(uint32_t delay) { this->closing_delay = delay; }
#endif
    void schedule_door_position_sync(float update_period = 500);
    void door_position_update();
    void cancel_position_sync_callbacks();
#ifdef RATGDO_USE_DISTANCE_SENSOR
    void set_target_distance_measurement(int16_t distance);
    void set_distance_measurement(int16_t distance);
#endif
#ifdef RATGDO_USE_VEHICLE_SENSORS
    void calculate_presence();
    void presence_change(bool sensor_value);
#endif

    // light
    void light_toggle();
    void light_on();
    void light_off();
    LightState get_light_state() const;

    // lock
    void lock_toggle();
    void lock();
    void unlock();

    // TTC (time-to-close)
    void ttc_toggle_hold();
    void start_or_sync_ttc_countdown(uint16_t seconds);

    // Learn & Paired
    void activate_learn();
    void inactivate_learn();
    void query_paired_devices();
    void query_paired_devices(PairedDevice kind);
    void clear_paired_devices(PairedDevice kind);

    // Uses length + first character instead of string comparisons to avoid
    // string literals in RODATA which consume RAM on ESP8266.
    // Valid values: "all" (3,a), "remote" (6,r), "keypad" (6,k), "wall" (4,w), "accessory" (9,a)
    // Template so it works with std::string, StringRef, or any type with length() and operator[].
    template <typename StringT>
    void clear_paired_devices(const StringT& kind)
    {
        PairedDevice device;
        if (kind.length() == 3 && kind[0] == 'a') {
            device = PairedDevice::ALL;
        } else if (kind.length() == 6 && kind[0] == 'r') {
            device = PairedDevice::REMOTE;
        } else if (kind.length() == 6 && kind[0] == 'k') {
            device = PairedDevice::KEYPAD;
        } else if (kind.length() == 4 && kind[0] == 'w') {
            device = PairedDevice::WALL_CONTROL;
        } else if (kind.length() == 9 && kind[0] == 'a') {
            device = PairedDevice::ACCESSORY;
        } else {
            return;
        }
        this->clear_paired_devices(device);
    }

    // button functionality
    void query_status();
    void query_openings();
    void sync();

    using Component::cancel_interval;
    using Component::set_interval;
    using Component::set_timeout;

    void set_door_state_expiry();
    void cancel_door_state_expiry();

    // Register a one-shot door state callback with automatic expiry.
    //
    // Handles secplus1's nested callback chains where opening from
    // STOPPED requires multiple state transitions:
    //
    //   on_door_state(outer_cb)          // wait for CLOSING
    //     → set_door_state_expiry()      // expiry A
    //     → [door reports CLOSING]
    //       → outer_cb fires, calls:
    //         toggle_door()
    //         on_door_state(inner_cb)    // wait for STOPPED
    //           → set_door_state_expiry() // expiry B (replaces A)
    //           → [door reports STOPPED]
    //             → inner_cb fires
    //               toggle_door()        // door now opening
    //               count()==0 → cancel expiry B
    //
    // The user callback runs BEFORE the expiry check because it may
    // re-arm the chain by calling on_door_state() again. If it does,
    // the new call sets expiry B which replaces expiry A (same timeout
    // ID = replace, not add). We only cancel expiry when count()==0,
    // meaning no new callback was queued — otherwise we'd cancel
    // expiry B here and leave the inner callback without protection.
    template <typename F>
    void on_door_state(F&& callback)
    {
        using Cb = std::decay_t<F>;
        this->on_door_state_([this, cb = Cb(std::forward<F>(callback))](DoorState s) {
            cb(s);
            if (!this->on_door_state_.count()) {
                this->cancel_door_state_expiry();
            }
        });
        this->set_door_state_expiry();
    }

    // children subscriptions — type-safe templates (no std::function)
    // Callbacks must be trivially copyable and fit in Callback storage
    // (3 * sizeof(void*)), e.g. [this] or [this, f] lambdas.
    // Enforced at compile time by Callback::create().
    template <typename F>
    void subscribe_rolling_code_counter(F&& f);
    template <typename F>
    void subscribe_opening_duration(F&& f);
    template <typename F>
    void subscribe_closing_duration(F&& f);
#ifdef RATGDO_USE_CLOSING_DELAY
    template <typename F>
    void subscribe_closing_delay(F&& f);
#endif
    template <typename F>
    void subscribe_openings(F&& f);
    template <typename F>
    void subscribe_paired_devices_total(F&& f);
    template <typename F>
    void subscribe_paired_remotes(F&& f);
    template <typename F>
    void subscribe_paired_keypads(F&& f);
    template <typename F>
    void subscribe_paired_wall_controls(F&& f);
    template <typename F>
    void subscribe_paired_accessories(F&& f);
    template <typename F>
    void subscribe_door_state(F&& f);
    template <typename F>
    void subscribe_light_state(F&& f);
    template <typename F>
    void subscribe_lock_state(F&& f);
    template <typename F>
    void subscribe_obstruction_state(F&& f);
    template <typename F>
    void subscribe_motor_state(F&& f);
#ifdef RATGDO_USE_ENCODER
    template <typename F>
    void subscribe_manually_operated_state(F&& f);
#endif
    template <typename F>
    void subscribe_button_state(F&& f);
    template <typename F>
    void subscribe_motion_state(F&& f);
    template <typename F>
    void subscribe_sync_failed(F&& f);
    template <typename F>
    void subscribe_learn_state(F&& f);
    template <typename F>
    void subscribe_ttc_state(F&& f);
    template <typename F>
    void subscribe_ttc_countdown(F&& f);
    template <typename F>
    void subscribe_ttc_limit(F&& f);
    template <typename F>
    void subscribe_door_action_delayed(F&& f);
#ifdef RATGDO_USE_DISTANCE_SENSOR
    template <typename F>
    void subscribe_distance_measurement(F&& f);
#endif
#ifdef RATGDO_USE_VEHICLE_SENSORS
    template <typename F>
    void subscribe_vehicle_detected_state(F&& f);
    template <typename F>
    void subscribe_vehicle_arriving_state(F&& f);
    template <typename F>
    void subscribe_vehicle_leaving_state(F&& f);
#endif

protected:
    // Pointers first (4-byte aligned)
    protocol::Protocol* protocol_;
    InternalGPIOPin* output_gdo_pin_;
    InternalGPIOPin* input_gdo_pin_;
    InternalGPIOPin* input_obst_pin_;
    esphome::binary_sensor::BinarySensor* dry_contact_open_sensor_;
    esphome::binary_sensor::BinarySensor* dry_contact_close_sensor_;

    // 4-byte members
    RATGDOStore isr_store_ { };

    // Bool members packed into bitfield
    struct {
        uint8_t obstruction_sensor_detected : 1;
        uint8_t obst_sleep_low : 1;
        uint8_t ttc_limit_learned : 1; // whether ttc_limit reflects the current open cycle
#ifdef RATGDO_USE_VEHICLE_SENSORS
        uint8_t presence_detect_window_active : 1;
#endif
#ifdef RATGDO_USE_ENCODER
        uint8_t reverse_encoder : 1;
        uint8_t enc_min_cal : 1;
        uint8_t enc_max_cal : 1;
        uint8_t enc_first_update : 1; // set in set_encoder_sensor(); flags_ zero-inits to 0
        uint8_t enc_position_stop_pending : 1;
#endif
    } flags_ { 0 };

#ifdef RATGDO_USE_ENCODER
    esphome::sensor::Sensor* encoder_sensor_ { nullptr };
    ESPPreferenceObject encoder_pref_;
    int16_t enc_min_ { 0 };
    int16_t enc_max_ { 0 };
    int16_t enc_last_ { 0 };
    int8_t enc_last_dir_ { 0 }; // sign of last delta: +1 increasing, -1 decreasing
    int8_t enc_travel_dir_ { 0 }; // direction of current/last move, latched from first tick; immune to end-of-travel oscillation
    int8_t enc_reverse_count_ { 0 }; // consecutive steps opposite to enc_travel_dir_; used to confirm real reversals
    DoorState protocol_door_state_ { DoorState::UNKNOWN };
    DoorState encoder_door_state_ { DoorState::UNKNOWN };
    uint32_t encoder_motion_onset_ms_ { 0 };

    int8_t enc_intended_dir_ { 0 }; // intended motion: +1=open, -1=close, 0=none
    bool enc_dir_correction_pending_ { false }; // set when wrong direction detected and correction is pending
    int8_t enc_dir_correction_intended_ { 0 }; // direction to retry: +1=open, -1=close
    InternalGPIOPin* enc_pin_a_ { nullptr };
    InternalGPIOPin* enc_pin_b_ { nullptr };
#endif

    // Subscriber counters for defer name allocation
    uint8_t door_state_sub_num_ { 0 };
    uint8_t door_action_delayed_sub_num_ { 0 };
#ifdef RATGDO_USE_ENCODER
    uint8_t manually_operated_sub_num_ { 0 };
#endif
#ifdef RATGDO_USE_DISTANCE_SENSOR
    uint8_t distance_sub_num_ { 0 };
#endif
#ifdef RATGDO_USE_VEHICLE_SENSORS
    uint8_t vehicle_detected_sub_num_ { 0 };
    uint8_t vehicle_arriving_sub_num_ { 0 };
    uint8_t vehicle_leaving_sub_num_ { 0 };
    int last_presence_percent_ { -1 };
    int presence_off_counter_ { 0 };
    DoorState last_door_state_for_presence_ { DoorState::UNKNOWN };
#endif
}; // RATGDOComponent

void log_subscriber_overflow(const LogString* observable_name, uint32_t max);

inline uint32_t get_scheduler_id(uint32_t base, uint32_t count, uint8_t& counter, const LogString* observable_name)
{
    if (count == 0) {
        log_subscriber_overflow(observable_name, count);
        return base;
    }
    if (counter >= count) {
        log_subscriber_overflow(observable_name, count);
        return base + count - 1; // reuse last ID to avoid collision with first subscriber
    }
    return base + counter++;
}

// Scheduler IDs using uint32_t ranges to avoid heap allocations
// Bases are auto-generated from counts to prevent ID conflicts
namespace scheduler_ids {
    inline constexpr uint32_t INTERVAL_POSITION_SYNC = 0;

    // Multi-subscriber ranges — counts derived from codegen defines
    inline constexpr uint32_t DEFER_DOOR_STATE_COUNT = RATGDO_MAX_DOOR_STATE_SUBSCRIBERS;
    inline constexpr uint32_t DEFER_DOOR_STATE_BASE = INTERVAL_POSITION_SYNC + 1;

    inline constexpr uint32_t DEFER_DOOR_ACTION_DELAYED_COUNT = RATGDO_MAX_DOOR_ACTION_DELAYED_SUBSCRIBERS;
    inline constexpr uint32_t DEFER_DOOR_ACTION_DELAYED_BASE = DEFER_DOOR_STATE_BASE + DEFER_DOOR_STATE_COUNT;

#ifdef RATGDO_USE_DISTANCE_SENSOR
    inline constexpr uint32_t DEFER_DISTANCE_COUNT = RATGDO_MAX_DISTANCE_SUBSCRIBERS;
    inline constexpr uint32_t DEFER_DISTANCE_BASE = DEFER_DOOR_ACTION_DELAYED_BASE + DEFER_DOOR_ACTION_DELAYED_COUNT;
    inline constexpr uint32_t DEFER_DISTANCE_END = DEFER_DISTANCE_BASE + DEFER_DISTANCE_COUNT;
#else
    inline constexpr uint32_t DEFER_DISTANCE_END = DEFER_DOOR_ACTION_DELAYED_BASE + DEFER_DOOR_ACTION_DELAYED_COUNT;
#endif

#ifdef RATGDO_USE_VEHICLE_SENSORS
    inline constexpr uint32_t DEFER_VEHICLE_DETECTED_COUNT = RATGDO_MAX_VEHICLE_DETECTED_SUBSCRIBERS;
    inline constexpr uint32_t DEFER_VEHICLE_DETECTED_BASE = DEFER_DISTANCE_END;
    inline constexpr uint32_t DEFER_VEHICLE_ARRIVING_COUNT = RATGDO_MAX_VEHICLE_ARRIVING_SUBSCRIBERS;
    inline constexpr uint32_t DEFER_VEHICLE_ARRIVING_BASE = DEFER_VEHICLE_DETECTED_BASE + DEFER_VEHICLE_DETECTED_COUNT;
    inline constexpr uint32_t DEFER_VEHICLE_LEAVING_COUNT = RATGDO_MAX_VEHICLE_LEAVING_SUBSCRIBERS;
    inline constexpr uint32_t DEFER_VEHICLE_LEAVING_BASE = DEFER_VEHICLE_ARRIVING_BASE + DEFER_VEHICLE_ARRIVING_COUNT;
    inline constexpr uint32_t DEFER_VEHICLE_END = DEFER_VEHICLE_LEAVING_BASE + DEFER_VEHICLE_LEAVING_COUNT;
#else
    inline constexpr uint32_t DEFER_VEHICLE_END = DEFER_DISTANCE_END;
#endif
#ifdef RATGDO_USE_ENCODER
    inline constexpr uint32_t DEFER_MANUALLY_OPERATED_COUNT = RATGDO_MAX_MANUALLY_OPERATED_SUBSCRIBERS;
    inline constexpr uint32_t DEFER_MANUALLY_OPERATED_BASE = DEFER_VEHICLE_END;
    inline constexpr uint32_t DEFER_MULTI_END = DEFER_MANUALLY_OPERATED_BASE + DEFER_MANUALLY_OPERATED_COUNT;
#else
    inline constexpr uint32_t DEFER_MULTI_END = DEFER_VEHICLE_END;
#endif
    // Single-subscriber IDs
    enum : uint32_t {
        DEFER_ROLLING_CODE = DEFER_MULTI_END,
        DEFER_OPENING_DURATION,
        DEFER_CLOSING_DURATION,
        DEFER_CLOSING_DELAY,
        DEFER_OPENINGS,
        DEFER_PAIRED_TOTAL,
        DEFER_PAIRED_REMOTES,
        DEFER_PAIRED_KEYPADS,
        DEFER_PAIRED_WALL_CONTROLS,
        DEFER_PAIRED_ACCESSORIES,
        DEFER_LIGHT_STATE,
        DEFER_LOCK_STATE,
        DEFER_OBSTRUCTION_STATE,
        DEFER_MOTOR_STATE,
        DEFER_BUTTON_STATE,
        DEFER_MOTION_STATE,
        DEFER_LEARN_STATE,
        DEFER_TTC_STATE,
        DEFER_TTC_COUNTDOWN,
        DEFER_TTC_LIMIT,

        // Named timeout IDs (replacing string-based names)
        TIMEOUT_DOOR_QUERY_STATE,
        TIMEOUT_DOOR_ACTION,
        TIMEOUT_MOVE_TO_POSITION,
        TIMEOUT_CLEAR_MOTION,
        // Shared by RATGDOComponent and Secplus1 — safe because only one
        // protocol is compiled at a time (#ifdef PROTOCOL_SECPLUSV1) and
        // both use ratgdo_ as the scheduler owner.
        TIMEOUT_DOOR_STATE_EXPIRY,
        TIMEOUT_PRESENCE_DETECT_WINDOW,
        TIMEOUT_CLEAR_PRESENCE,
        TIMEOUT_WALL_PANEL_EMULATION,
        TIMEOUT_SYNC,
        INTERVAL_STATUS_WATCHDOG,
        TIMEOUT_ENCODER_STOPPED,
        TTC_COUNTDOWN_WATCHDOG,
        TTC_COUNTDOWN_LOCAL_DECREMENT,
    };
} // namespace scheduler_ids

// Template implementations for subscribe methods.
// Each wraps the callback in a deferred call so that if the observable
// fires multiple times during one loop iteration, only the last value
// is dispatched to the child component.

template <typename F>
void RATGDOComponent::subscribe_rolling_code_counter(F&& f)
{
    // change update to children is defered until after component loop
    // if multiple changes occur during component loop, only the last one is notified
    auto counter = this->protocol_->call(protocol::GetRollingCodeCounter { });
    if (counter.tag == protocol::Result::Tag::rolling_code_counter) {
        counter.value.rolling_code_counter.value->subscribe([this, f](uint32_t state) {
            defer(scheduler_ids::DEFER_ROLLING_CODE, [f, state] { f(state); });
        });
    }
}

template <typename F>
void RATGDOComponent::subscribe_opening_duration(F&& f)
{
    this->opening_duration.subscribe([this, f](float state) {
        defer(scheduler_ids::DEFER_OPENING_DURATION, [f, state] { f(state); });
    });
}

template <typename F>
void RATGDOComponent::subscribe_closing_duration(F&& f)
{
    this->closing_duration.subscribe([this, f](float state) {
        defer(scheduler_ids::DEFER_CLOSING_DURATION, [f, state] { f(state); });
    });
}

#ifdef RATGDO_USE_CLOSING_DELAY
template <typename F>
void RATGDOComponent::subscribe_closing_delay(F&& f)
{
    this->closing_delay.subscribe([this, f](uint32_t state) {
        defer(scheduler_ids::DEFER_CLOSING_DELAY, [f, state] { f(state); });
    });
}
#endif

template <typename F>
void RATGDOComponent::subscribe_openings(F&& f)
{
    this->openings.subscribe([this, f](uint16_t state) {
        defer(scheduler_ids::DEFER_OPENINGS, [f, state] { f(state); });
    });
}

template <typename F>
void RATGDOComponent::subscribe_paired_devices_total(F&& f)
{
    this->paired_total.subscribe([this, f](uint8_t state) {
        defer(scheduler_ids::DEFER_PAIRED_TOTAL, [f, state] { f(state); });
    });
}

template <typename F>
void RATGDOComponent::subscribe_paired_remotes(F&& f)
{
    this->paired_remotes.subscribe([this, f](uint8_t state) {
        defer(scheduler_ids::DEFER_PAIRED_REMOTES, [f, state] { f(state); });
    });
}

template <typename F>
void RATGDOComponent::subscribe_paired_keypads(F&& f)
{
    this->paired_keypads.subscribe([this, f](uint8_t state) {
        defer(scheduler_ids::DEFER_PAIRED_KEYPADS, [f, state] { f(state); });
    });
}

template <typename F>
void RATGDOComponent::subscribe_paired_wall_controls(F&& f)
{
    this->paired_wall_controls.subscribe([this, f](uint8_t state) {
        defer(scheduler_ids::DEFER_PAIRED_WALL_CONTROLS, [f, state] { f(state); });
    });
}

template <typename F>
void RATGDOComponent::subscribe_paired_accessories(F&& f)
{
    this->paired_accessories.subscribe([this, f](uint8_t state) {
        defer(scheduler_ids::DEFER_PAIRED_ACCESSORIES, [f, state] { f(state); });
    });
}

template <typename F>
void RATGDOComponent::subscribe_door_state(F&& f)
{
    uint32_t id = get_scheduler_id(scheduler_ids::DEFER_DOOR_STATE_BASE, scheduler_ids::DEFER_DOOR_STATE_COUNT,
        this->door_state_sub_num_, LOG_STR("door_state"));
    this->door_state.subscribe([this, f, id](DoorState state) {
        defer(id, [this, f, state] { f(state, *this->door_position); });
    });
    this->door_position.subscribe([this, f, id](float position) {
        defer(id, [this, f, position] { f(*this->door_state, position); });
    });
}

template <typename F>
void RATGDOComponent::subscribe_light_state(F&& f)
{
    this->light_state.subscribe([this, f](LightState state) {
        defer(scheduler_ids::DEFER_LIGHT_STATE, [f, state] { f(state); });
    });
}

template <typename F>
void RATGDOComponent::subscribe_lock_state(F&& f)
{
    this->lock_state.subscribe([this, f](LockState state) {
        defer(scheduler_ids::DEFER_LOCK_STATE, [f, state] { f(state); });
    });
}

template <typename F>
void RATGDOComponent::subscribe_obstruction_state(F&& f)
{
    this->obstruction_state.subscribe([this, f](ObstructionState state) {
        defer(scheduler_ids::DEFER_OBSTRUCTION_STATE, [f, state] { f(state); });
    });
}

template <typename F>
void RATGDOComponent::subscribe_motor_state(F&& f)
{
    this->motor_state.subscribe([this, f](MotorState state) {
        defer(scheduler_ids::DEFER_MOTOR_STATE, [f, state] { f(state); });
    });
}

#ifdef RATGDO_USE_ENCODER
template <typename F>
void RATGDOComponent::subscribe_manually_operated_state(F&& f)
{
    uint32_t id = get_scheduler_id(scheduler_ids::DEFER_MANUALLY_OPERATED_BASE, scheduler_ids::DEFER_MANUALLY_OPERATED_COUNT,
        this->manually_operated_sub_num_, LOG_STR("manually_operated"));
    this->manually_operated_state.subscribe([this, f, id](ManuallyOperatedState state) {
        defer(id, [f, state] { f(state); });
    });
}
#endif

template <typename F>
void RATGDOComponent::subscribe_button_state(F&& f)
{
    this->button_state.subscribe([this, f](ButtonState state) {
        defer(scheduler_ids::DEFER_BUTTON_STATE, [f, state] { f(state); });
    });
}

template <typename F>
void RATGDOComponent::subscribe_motion_state(F&& f)
{
    this->motion_state.subscribe([this, f](MotionState state) {
        defer(scheduler_ids::DEFER_MOTION_STATE, [f, state] { f(state); });
    });
}

template <typename F>
void RATGDOComponent::subscribe_sync_failed(F&& f)
{
    this->sync_failed.subscribe(std::forward<F>(f));
}

template <typename F>
void RATGDOComponent::subscribe_learn_state(F&& f)
{
    this->learn_state.subscribe([this, f](LearnState state) {
        defer(scheduler_ids::DEFER_LEARN_STATE, [f, state] { f(state); });
    });
}

template <typename F>
void RATGDOComponent::subscribe_ttc_state(F&& f)
{
    this->ttc_state.subscribe([this, f](TtcState state) {
        defer(scheduler_ids::DEFER_TTC_STATE, [f, state] { f(state); });
    });
}

template <typename F>
void RATGDOComponent::subscribe_ttc_countdown(F&& f)
{
    this->ttc_countdown.subscribe([this, f](uint16_t seconds) {
        defer(scheduler_ids::DEFER_TTC_COUNTDOWN, [f, seconds] { f(seconds); });
    });
}

template <typename F>
void RATGDOComponent::subscribe_ttc_limit(F&& f)
{
    this->ttc_limit.subscribe([this, f](uint16_t seconds) {
        defer(scheduler_ids::DEFER_TTC_LIMIT, [f, seconds] { f(seconds); });
    });
}

template <typename F>
void RATGDOComponent::subscribe_door_action_delayed(F&& f)
{
    uint32_t id = get_scheduler_id(scheduler_ids::DEFER_DOOR_ACTION_DELAYED_BASE, scheduler_ids::DEFER_DOOR_ACTION_DELAYED_COUNT,
        this->door_action_delayed_sub_num_, LOG_STR("door_action_delayed"));
    this->door_action_delayed.subscribe([this, f, id](DoorActionDelayed state) {
        defer(id, [f, state] { f(state); });
    });
}

#ifdef RATGDO_USE_DISTANCE_SENSOR
template <typename F>
void RATGDOComponent::subscribe_distance_measurement(F&& f)
{
    uint32_t id = get_scheduler_id(scheduler_ids::DEFER_DISTANCE_BASE, scheduler_ids::DEFER_DISTANCE_COUNT,
        this->distance_sub_num_, LOG_STR("distance_measurement"));
    this->last_distance_measurement.subscribe([this, f, id](int16_t state) {
        defer(id, [f, state] { f(state); });
    });
}
#endif

#ifdef RATGDO_USE_VEHICLE_SENSORS
template <typename F>
void RATGDOComponent::subscribe_vehicle_detected_state(F&& f)
{
    uint32_t id = get_scheduler_id(scheduler_ids::DEFER_VEHICLE_DETECTED_BASE, scheduler_ids::DEFER_VEHICLE_DETECTED_COUNT,
        this->vehicle_detected_sub_num_, LOG_STR("vehicle_detected"));
    this->vehicle_detected_state.subscribe([this, f, id](VehicleDetectedState state) {
        defer(id, [f, state] { f(state); });
    });
}

template <typename F>
void RATGDOComponent::subscribe_vehicle_arriving_state(F&& f)
{
    uint32_t id = get_scheduler_id(scheduler_ids::DEFER_VEHICLE_ARRIVING_BASE, scheduler_ids::DEFER_VEHICLE_ARRIVING_COUNT,
        this->vehicle_arriving_sub_num_, LOG_STR("vehicle_arriving"));
    this->vehicle_arriving_state.subscribe([this, f, id](VehicleArrivingState state) {
        defer(id, [f, state] { f(state); });
    });
}

template <typename F>
void RATGDOComponent::subscribe_vehicle_leaving_state(F&& f)
{
    uint32_t id = get_scheduler_id(scheduler_ids::DEFER_VEHICLE_LEAVING_BASE, scheduler_ids::DEFER_VEHICLE_LEAVING_COUNT,
        this->vehicle_leaving_sub_num_, LOG_STR("vehicle_leaving"));
    this->vehicle_leaving_state.subscribe([this, f, id](VehicleLeavingState state) {
        defer(id, [f, state] { f(state); });
    });
}
#endif

} // namespace esphome::ratgdo
