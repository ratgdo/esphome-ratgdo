#include "ratgdo_output.h"
#include "../ratgdo_state.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ratgdo {

static const char *TAG = "ratgdo.output";

void RATGDOOutput::setup(){
    ESP_LOGD(TAG,"Output was setup");

    if(this->output_type_ == OutputType::RATGDO_BEEPER){
        this->beeper_->add_on_finished_playback_callback([=] { this->finished_playback(); });

        this->parent_->subscribe_vehicle_arriving_state([=](VehicleArrivingState state) {
            if(state == VehicleArrivingState::YES){
                this->play();
            }
        });

        this->parent_->subscribe_door_action_delayed([=](DoorActionDelayed state) {
            if(state == DoorActionDelayed::YES){
                this->play();
                this->repeat_ = true;
            } else if(state == DoorActionDelayed::NO) {
                this->repeat_ = false;
            }
        });

    }
}

void RATGDOOutput::play(){
    this->beeper_->play(this->rtttlSong_);
}

void RATGDOOutput::finished_playback(){
    if(this->repeat_) this->play();
}

void RATGDOOutput::dump_config() {
    if (this->output_type_ == OutputType::RATGDO_BEEPER) {
        ESP_LOGCONFIG(TAG, "  Type: Beeper");
    }
}

void RATGDOOutput::set_output_type(OutputType output_type_) {
    this->output_type_ = output_type_;
}

} //namespace ratgdo
} //namespace esphome