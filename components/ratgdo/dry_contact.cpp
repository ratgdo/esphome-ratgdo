
#include "dry_contact.h"
#include "esphome/core/gpio.h"
#include "esphome/core/log.h"
#include "esphome/core/scheduler.h"
#include "ratgdo.h"

namespace esphome {
namespace ratgdo {
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
        }

        void DryContact::loop()
        {
        }

        void DryContact::dump_config()
        {
            ESP_LOGCONFIG(TAG, "  Protocol: dry contact");
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

            this->ratgdo_->received(this->door_state_);
        }

        void DryContact::light_action(LightAction action)
        {
            ESP_LOG1(TAG, "Ignoring light action: %s", LightAction_to_string(action));
            return;
        }

        void DryContact::lock_action(LockAction action)
        {
            ESP_LOG1(TAG, "Ignoring lock action: %s", LockAction_to_string(action));
            return;
        }

        void DryContact::door_action(DoorAction action)
        {
            if (action == DoorAction::OPEN && this->door_state_ != DoorState::CLOSED) {
                ESP_LOGW(TAG, "The door is not closed. Ignoring door action: %s", DoorAction_to_string(action));
                return;
            }
            if (action == DoorAction::CLOSE && this->door_state_ != DoorState::OPEN) {
                ESP_LOGW(TAG, "The door is not open. Ignoring door action: %s", DoorAction_to_string(action));
                return;
            }

            ESP_LOG1(TAG, "Door action: %s", DoorAction_to_string(action));

            if (action == DoorAction::OPEN) {
                this->discrete_open_pin_->digital_write(1);
                this->scheduler_->set_timeout(this->ratgdo_, "", 500, [=] {
                    this->discrete_open_pin_->digital_write(0);
                });
            }

            if (action == DoorAction::CLOSE) {
                this->discrete_close_pin_->digital_write(1);
                this->scheduler_->set_timeout(this->ratgdo_, "", 500, [=] {
                    this->discrete_close_pin_->digital_write(0);
                });
            }

            this->tx_pin_->digital_write(1); // Single button control
            this->scheduler_->set_timeout(this->ratgdo_, "", 500, [=] {
                this->tx_pin_->digital_write(0);
            });
        }

        Result DryContact::call(Args args)
        {
            return {};
        }

    } // namespace dry_contact
} // namespace ratgdo
} // namespace esphome
