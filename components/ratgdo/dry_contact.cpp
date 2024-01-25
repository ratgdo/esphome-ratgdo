
#include "dry_contact.h"
#include "ratgdo.h"

#include "esphome/core/gpio.h"
#include "esphome/core/log.h"
#include "esphome/core/scheduler.h"

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
           ESP_LOG1(TAG, "Door action: %s, door state: %s", DoorAction_to_string(action), DoorState_to_string(*this->ratgdo_->door_state));
            if (action == DoorAction::UNKNOWN) {
                return;
            }

            const uint32_t double_toggle_delay = 1000;
            if (action == DoorAction::TOGGLE) {
                this->toggle_door();
            } else if (action == DoorAction::OPEN) {
                if (*this->ratgdo_->door_state == DoorState::CLOSED || *this->ratgdo_->door_state == DoorState::CLOSING) {
                    this->toggle_door();
                } else if (*this->ratgdo_->door_state == DoorState::STOPPED) {
                    this->toggle_door(); // this starts closing door
                    this->ratgdo_->on_door_state_([=](DoorState s) {
                        if (s == DoorState::CLOSING) {
                            // this changes direction of the door on some openers, on others it stops it
                            this->toggle_door();
                            this->ratgdo_->on_door_state_([=](DoorState s) {
                                if (s == DoorState::STOPPED) {
                                    this->toggle_door();
                                }
                            });
                        }
                    });
                }
            } else if (action == DoorAction::CLOSE) {
                if (*this->ratgdo_->door_state == DoorState::OPEN) {
                    this->toggle_door();
                } else if (*this->ratgdo_->door_state == DoorState::OPENING) {
                    this->toggle_door(); // this switches to stopped
                    // another toggle needed to close
                    this->ratgdo_->on_door_state_([=](DoorState s) {
                        if (s == DoorState::STOPPED) {
                            this->toggle_door();
                        }
                    });
                } else if (*this->ratgdo_->door_state == DoorState::STOPPED) {
                    this->toggle_door();
                }
            } else if (action == DoorAction::STOP) {
                if (*this->ratgdo_->door_state == DoorState::OPENING) {
                    this->toggle_door();
                } else if (*this->ratgdo_->door_state == DoorState::CLOSING) {
                    this->toggle_door(); // this switches to opening

                    // another toggle needed to stop
                    this->ratgdo_->on_door_state_([=](DoorState s) {
                        if (s == DoorState::OPENING) {
                            this->toggle_door();
                        }
                    });
                }
            }

        }

        void DryContact::toggle_door()
        {
            this->tx_pin_->digital_write(1);
            this->scheduler_->set_timeout(this->ratgdo_, "", 200, [=] {
                this->tx_pin_->digital_write(0);
            });
        }

        Result DryContact::call(Args args)
        {
            return {};
        }

    } // namespace DryContact
} // namespace ratgdo
} // namespace esphome
