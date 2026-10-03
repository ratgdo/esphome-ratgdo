
#ifdef PROTOCOL_DRYCONTACT

#include "dry_contact.h"
#include "esphome/core/gpio.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/core/scheduler.h"
#include "ratgdo.h"

namespace esphome::ratgdo {
namespace dry_contact {

    static const char* const TAG = "ratgdo_dry_contact";

    void DryContact::setup(RATGDOComponent* ratgdo, Scheduler* scheduler, InternalGPIOPin* rx_pin, InternalGPIOPin* tx_pin)
    {
        this->ratgdo_ = ratgdo;
        this->scheduler_ = scheduler;
        this->tx_pin_ = tx_pin;
        this->rx_pin_ = rx_pin;

        this->limits_.open_limit_reached = 0;
        this->limits_.last_open_limit = 0;
        this->limits_.close_limit_reached = 0;
        this->limits_.last_close_limit = 0;
        this->door_state_ = DoorState::UNKNOWN;

        if (this->obstruction_while_opening_ != DryContactBehavior::UNSET
            || this->obstruction_while_closing_ != DryContactBehavior::UNSET) {
            // not deferred: a brief OBSTRUCTED is followed by CLEAR one check later
            this->ratgdo_->obstruction_state.subscribe([this](ObstructionState state) {
                if (state == ObstructionState::OBSTRUCTED) {
                    this->obstructed();
                }
            });
        }
    }

    void DryContact::loop()
    {
    }

    void DryContact::dump_config()
    {
        ESP_LOGCONFIG(TAG, "  Protocol: dry contact");
        if (this->toggle_configured()) {
            ESP_LOGCONFIG(TAG, "  Toggle while opening: %s, closing: %s, stopped: %s",
                LOG_STR_ARG(DryContactBehavior_to_string(this->toggle_while_opening_)),
                LOG_STR_ARG(DryContactBehavior_to_string(this->toggle_while_closing_)),
                LOG_STR_ARG(DryContactBehavior_to_string(this->toggle_while_stopped_)));
        }
        if (this->obstruction_while_opening_ != DryContactBehavior::UNSET
            || this->obstruction_while_closing_ != DryContactBehavior::UNSET) {
            ESP_LOGCONFIG(TAG, "  Obstruction while opening: %s, closing: %s",
                LOG_STR_ARG(DryContactBehavior_to_string(this->obstruction_while_opening_)),
                LOG_STR_ARG(DryContactBehavior_to_string(this->obstruction_while_closing_)));
        }
    }

    void DryContact::sync()
    {
        ESP_LOG1(TAG, "Ignoring sync action");
    }

    void DryContact::set_open_limit(bool state)
    {
        ESP_LOGD(TAG, "Set open_limit_reached to %d", state);
        this->limits_.last_open_limit = this->limits_.open_limit_reached;
        this->limits_.last_close_limit = false;
        this->limits_.open_limit_reached = state;
        this->send_door_state();
    }

    void DryContact::set_close_limit(bool state)
    {
        ESP_LOGD(TAG, "Set close_limit_reached to %d", state);
        this->limits_.last_close_limit = this->limits_.close_limit_reached;
        this->limits_.last_open_limit = false;
        this->limits_.close_limit_reached = state;
        this->send_door_state();
    }

    void DryContact::send_door_state()
    {
        if (this->limits_.open_limit_reached) {
            this->door_state_ = DoorState::OPEN;
        } else if (this->limits_.close_limit_reached) {
            this->door_state_ = DoorState::CLOSED;
        } else if (!this->limits_.close_limit_reached && !this->limits_.open_limit_reached) {
            if (this->limits_.last_close_limit) {
                this->door_state_ = DoorState::OPENING;
            }

            if (this->limits_.last_open_limit) {
                this->door_state_ = DoorState::CLOSING;
            }
        }

        this->report(this->door_state_);
    }

    void DryContact::light_action(LightAction action)
    {
        ESP_LOG1(TAG, "Ignoring light action: %s", LOG_STR_ARG(LightAction_to_string(action)));
        return;
    }

    void DryContact::lock_action(LockAction action)
    {
        ESP_LOG1(TAG, "Ignoring lock action: %s", LOG_STR_ARG(LockAction_to_string(action)));
        return;
    }

    void DryContact::door_action(DoorAction action)
    {
        if (!this->sequence_action(action)) {
            this->press(action);
        }
    }

    void DryContact::press(DoorAction action)
    {
        // Encoder builds do not have physical limit switches
        // so `limits_` is never updated. Instead, we rely on the derived `door_state`
        // which correctly reflects the software-calibrated limits.
#ifdef RATGDO_USE_ENCODER
        auto current_state = *this->ratgdo_->door_state;
        if (action == DoorAction::OPEN && current_state == DoorState::OPEN) {
            ESP_LOGW(TAG, "The door is already fully open. Ignoring door action: %s", LOG_STR_ARG(DoorAction_to_string(action)));
            return;
        }
        if (action == DoorAction::CLOSE && current_state == DoorState::CLOSED) {
            ESP_LOGW(TAG, "The door is already fully closed. Ignoring door action: %s", LOG_STR_ARG(DoorAction_to_string(action)));
            return;
        }
#else
        if (action == DoorAction::OPEN && this->limits_.open_limit_reached) {
            ESP_LOGW(TAG, "The door is already fully open. Ignoring door action: %s", LOG_STR_ARG(DoorAction_to_string(action)));
            return;
        }
        if (action == DoorAction::CLOSE && this->limits_.close_limit_reached) {
            ESP_LOGW(TAG, "The door is already fully closed. Ignoring door action: %s", LOG_STR_ARG(DoorAction_to_string(action)));
            return;
        }
#endif

        ESP_LOG1(TAG, "Door action: %s", LOG_STR_ARG(DoorAction_to_string(action)));

        if (action == DoorAction::OPEN && this->discrete_open_pin_ != nullptr) {
            this->discrete_open_pin_->digital_write(1);
            this->ratgdo_->set_timeout(500, [this] {
                this->discrete_open_pin_->digital_write(0);
            });
        }

        if (action == DoorAction::CLOSE && this->discrete_close_pin_ != nullptr) {
            this->discrete_close_pin_->digital_write(1);
            this->ratgdo_->set_timeout(500, [this] {
                this->discrete_close_pin_->digital_write(0);
            });
        }

        this->tx_pin_->digital_write(1); // Single button control
        this->ratgdo_->set_timeout(500, [this] {
            this->tx_pin_->digital_write(0);
        });
        this->next_toggle_ms_ = millis() + MIN_TOGGLE_INTERVAL_MS;
    }

    Result DryContact::call(Args args)
    {
        return { };
    }

    /*************************** TOGGLE SEQUENCE ***************************/

    // With toggle_while_* set, OPEN, CLOSE and STOP are reached one toggle at a time,
    // replanned from each resolved state. The sequence follows the states this protocol
    // reports through report().

    void DryContact::set_toggle_behavior(DryContactBehavior while_opening, DryContactBehavior while_closing,
        DryContactBehavior while_stopped)
    {
        this->toggle_while_opening_ = while_opening;
        this->toggle_while_closing_ = while_closing;
        this->toggle_while_stopped_ = while_stopped;
    }

    void DryContact::set_obstruction_behavior(DryContactBehavior while_opening, DryContactBehavior while_closing)
    {
        this->obstruction_while_opening_ = while_opening;
        this->obstruction_while_closing_ = while_closing;
    }

    bool DryContact::toggle_configured() const
    {
        return this->toggle_while_opening_ != DryContactBehavior::UNSET;
    }

    void DryContact::report(DoorState state)
    {
        const DoorState before = *this->ratgdo_->door_state;
        this->ratgdo_->received(state);
        const DoorState after = *this->ratgdo_->door_state;
        if (after != before) {
            this->on_resolved(after);
        }
    }

    // true: the action was handled here and must not be pressed as is
    bool DryContact::sequence_action(DoorAction action)
    {
        if (action == DoorAction::TOGGLE) {
            if (this->request_ != DoorAction::UNKNOWN) {
                ESP_LOGD(TAG, "Toggle pressed, dropping %s", LOG_STR_ARG(DoorAction_to_string(this->request_)));
                this->cancel_request();
            }
            return false;
        }
        if (!this->toggle_configured()
            || (action != DoorAction::OPEN && action != DoorAction::CLOSE && action != DoorAction::STOP)) {
            return false;
        }
        if (this->request_ != DoorAction::UNKNOWN) {
            ESP_LOGD(TAG, "Request changed from %s to %s", LOG_STR_ARG(DoorAction_to_string(this->request_)),
                LOG_STR_ARG(DoorAction_to_string(action)));
            this->request_ = action;
            this->toggle_count_ = 0;
            return true;
        }
        const DoorState state = *this->ratgdo_->door_state;
        if (this->request_reached(action, state)) {
            return true;
        }
        if (this->state_after_toggle(state) == DoorState::UNKNOWN) {
            return false; // endpoint, or stopped with no known direction: one press
        }
        if (action == DoorAction::STOP && this->toggle_while_opening_ != DryContactBehavior::STOP
            && this->toggle_while_closing_ != DryContactBehavior::STOP) {
            ESP_LOGW(TAG, "Opener does not stop on toggle, ignoring stop");
            return true;
        }
        this->request_ = action;
        this->toggle_count_ = 0;
        this->ratgdo_->cancel_timeout(scheduler_ids::TIMEOUT_DOOR_QUERY_STATE);
        this->ratgdo_->cancel_timeout(scheduler_ids::TIMEOUT_MOVE_TO_POSITION);
        this->step();
        return true;
    }

    bool DryContact::request_reached(DoorAction action, DoorState state) const
    {
        switch (action) {
        case DoorAction::OPEN:
            return state == DoorState::OPENING || state == DoorState::OPEN;
        case DoorAction::CLOSE:
            return state == DoorState::CLOSING || state == DoorState::CLOSED;
        case DoorAction::STOP:
            return state == DoorState::STOPPED || state == DoorState::OPEN || state == DoorState::CLOSED;
        default:
            return true;
        }
    }

    DoorState DryContact::state_after(DryContactBehavior behavior, DoorState moving) const
    {
        if (behavior == DryContactBehavior::STOP) {
            return DoorState::STOPPED;
        }
        if (behavior == DryContactBehavior::REVERSE) {
            return moving == DoorState::OPENING ? DoorState::CLOSING : DoorState::OPENING;
        }
        return moving;
    }

    // UNKNOWN: endpoint, or STOPPED with no known direction.
    DoorState DryContact::state_after_toggle(DoorState state) const
    {
        if (state == DoorState::OPENING) {
            return this->state_after(this->toggle_while_opening_, DoorState::OPENING);
        }
        if (state == DoorState::CLOSING) {
            return this->state_after(this->toggle_while_closing_, DoorState::CLOSING);
        }
        if (state == DoorState::STOPPED) {
            if (this->toggle_while_stopped_ == DryContactBehavior::IGNORE) {
                return DoorState::STOPPED;
            }
            if (this->last_direction_ == DoorState::OPENING) {
                return DoorState::CLOSING;
            }
            if (this->last_direction_ == DoorState::CLOSING) {
                return DoorState::OPENING;
            }
        }
        return DoorState::UNKNOWN;
    }

    void DryContact::step()
    {
        if (this->request_ == DoorAction::UNKNOWN) {
            return;
        }
        const DoorState state = *this->ratgdo_->door_state;
        if (this->request_reached(this->request_, state)) {
            ESP_LOGD(TAG, "%s done, door %s", LOG_STR_ARG(DoorAction_to_string(this->request_)),
                LOG_STR_ARG(DoorState_to_string(state)));
            this->request_ = DoorAction::UNKNOWN;
            if (state == DoorState::OPENING) {
                this->ratgdo_->set_open_endpoint_timer();
            } else if (state == DoorState::CLOSING) {
                this->ratgdo_->set_closed_endpoint_timer();
            }
            return;
        }
        const int32_t wait = static_cast<int32_t>(this->next_toggle_ms_ - millis());
        if (wait > 0) {
            this->ratgdo_->set_timeout(scheduler_ids::TIMEOUT_DRY_CONTACT_STEP, static_cast<uint32_t>(wait),
                [this] { this->step(); });
            return;
        }
        const DoorState expected = this->state_after_toggle(state);
        if (expected == DoorState::UNKNOWN) {
            const DoorAction action = this->request_;
            this->request_ = DoorAction::UNKNOWN;
            if (action == DoorAction::OPEN) {
                this->press(action);
                this->ratgdo_->set_open_endpoint_timer();
            } else if (action == DoorAction::CLOSE) {
                this->press(action);
                this->ratgdo_->set_closed_endpoint_timer();
            }
            return;
        }
        if (expected == state) {
            ESP_LOGW(TAG, "Opener ignores toggle while %s, cannot %s", LOG_STR_ARG(DoorState_to_string(state)),
                LOG_STR_ARG(DoorAction_to_string(this->request_)));
            this->request_ = DoorAction::UNKNOWN;
            return;
        }
        if (++this->toggle_count_ > MAX_TOGGLES) {
            ESP_LOGW(TAG, "%s not reached after %d toggles, giving up", LOG_STR_ARG(DoorAction_to_string(this->request_)),
                MAX_TOGGLES);
            this->request_ = DoorAction::UNKNOWN;
            return;
        }
        ESP_LOGD(TAG, "Toggle while %s, expecting %s (%s)", LOG_STR_ARG(DoorState_to_string(state)),
            LOG_STR_ARG(DoorState_to_string(expected)), LOG_STR_ARG(DoorAction_to_string(this->request_)));
        this->send_toggle(expected);
    }

    void DryContact::send_toggle(DoorState expected)
    {
        this->expected_state_ = expected;
        auto send = [this] {
            this->cancel_step();
            this->press(DoorAction::TOGGLE);
            this->toggle_pending_ = true;
            // timeout, not direct: a caller can register on_door_state() first
            this->ratgdo_->set_timeout(scheduler_ids::TIMEOUT_DRY_CONTACT_STEP, 0,
                [this] { this->report(this->expected_state_); });
        };
#ifdef RATGDO_USE_CLOSING_DELAY
        // a CLOSE request was already delayed by the component
        if (expected == DoorState::CLOSING && this->request_ != DoorAction::CLOSE && *this->ratgdo_->closing_delay > 0) {
            this->delayed_press_ = true;
            this->ratgdo_->door_action_delayed = DoorActionDelayed::YES;
            this->ratgdo_->set_timeout(scheduler_ids::TIMEOUT_DRY_CONTACT_STEP, *this->ratgdo_->closing_delay * 1000, send);
            return;
        }
#endif
        send();
    }

    void DryContact::cancel_request()
    {
        this->request_ = DoorAction::UNKNOWN;
        this->toggle_pending_ = false;
        this->cancel_step();
    }

    // Cancels a pending step, delayed press or expected-state report.
    void DryContact::cancel_step()
    {
        this->ratgdo_->cancel_timeout(scheduler_ids::TIMEOUT_DRY_CONTACT_STEP);
        if (this->delayed_press_) {
            this->delayed_press_ = false;
            this->ratgdo_->door_action_delayed = DoorActionDelayed::NO;
        }
    }

    void DryContact::on_resolved(DoorState state)
    {
        if (state == DoorState::OPENING || state == DoorState::CLOSING) {
            this->last_direction_ = state;
        }
        if (this->request_ == DoorAction::UNKNOWN) {
            return;
        }
        this->cancel_step();
        if (this->toggle_pending_) {
            this->toggle_pending_ = false;
            if (state != this->expected_state_) {
                ESP_LOGW(TAG, "Expected %s after toggle, door is %s; stopping %s",
                    LOG_STR_ARG(DoorState_to_string(this->expected_state_)), LOG_STR_ARG(DoorState_to_string(state)),
                    LOG_STR_ARG(DoorAction_to_string(this->request_)));
                this->request_ = DoorAction::UNKNOWN;
                return;
            }
        }
        this->step();
    }

    // What the opener does by itself when the obstruction sensor trips while moving.
    void DryContact::obstructed()
    {
        const DoorState state = *this->ratgdo_->door_state;
        DryContactBehavior behavior = DryContactBehavior::UNSET;
        if (state == DoorState::OPENING) {
            behavior = this->obstruction_while_opening_;
        } else if (state == DoorState::CLOSING) {
            behavior = this->obstruction_while_closing_;
        }
        if (behavior == DryContactBehavior::UNSET || behavior == DryContactBehavior::IGNORE) {
            return;
        }
        if (this->request_ != DoorAction::UNKNOWN) {
            ESP_LOGD(TAG, "Obstruction, dropping %s", LOG_STR_ARG(DoorAction_to_string(this->request_)));
            this->cancel_request();
        }
        // opener reacts by itself; endpoint timer would report the old endpoint
        this->ratgdo_->cancel_timeout(scheduler_ids::TIMEOUT_DOOR_QUERY_STATE);
        ESP_LOGD(TAG, "Obstruction while %s, behavior %s", LOG_STR_ARG(DoorState_to_string(state)),
            LOG_STR_ARG(DryContactBehavior_to_string(behavior)));
        this->report(this->state_after(behavior, state));
    }

} // namespace dry_contact
} // namespace esphome::ratgdo

#endif // PROTOCOL_DRYCONTACT
