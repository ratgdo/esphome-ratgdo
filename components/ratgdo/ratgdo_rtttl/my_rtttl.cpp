#include "esphome/core/log.h"
#include "my_rtttl.h"

namespace esphome {
namespace ratgdo {

static const char *TAG = "my_rtttl";

void RATGDOFloatOutput::setup(){
  ESP_LOGD("yyy","Output was setup");
  this->parent_->door_stop();
}

void RATGDOFloatOutput::write_state(float state){

}

void RATGDOFloatOutput::dump_config() {
    ESP_LOGCONFIG(TAG, "Empty custom float output");
}

void RATGDOFloatOutput::set_output_type(OutputType output_type_) {
  this->output_type_ = output_type_;
}

} //namespace ratgdo
} //namespace esphome