
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
            
            this->open_limit_reached_ = 0;
            this->last_open_limit_ = 0;
            this->close_limit_reached_ = 0;
            this->last_close_limit_ = 0;
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

        void DryContact::set_open_limit(bool val)
        {
            ESP_LOGD(TAG, "Set open_limit_reached to %d", val);
            this->last_open_limit_ = this->open_limit_reached_;
            this->open_limit_reached_ = val;
            this->send_door_state();
        }

        void DryContact::set_close_limit(bool val)
        {
            ESP_LOGD(TAG, "Set close_limit_reached to %d", val);
            this->last_close_limit_ = this->close_limit_reached_;
            this->close_limit_reached_ = val;
            this->send_door_state();
        }
        
        void DryContact::send_door_state(){
            DoorState door_state;

            if(this->open_limit_reached_){
                door_state = DoorState::OPEN;
            }else if(this->close_limit_reached_){
                door_state = DoorState::CLOSED;
            }else if(!this->close_limit_reached_ && !this->open_limit_reached_){
                if(this->last_close_limit_){
                    door_state = DoorState::OPENING;
                }

                if(this->last_open_limit_){
                    door_state = DoorState::CLOSING;
                }
            }

            this->ratgdo_->received(door_state);
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
            if (action != DoorAction::TOGGLE) {
                ESP_LOG1(TAG, "Ignoring door action: %s", DoorAction_to_string(action));
                return;
            }
            ESP_LOG1(TAG, "Door action: %s", DoorAction_to_string(action));

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
