#include "ratgdo_binary_sensor.h"
#include "esphome/core/log.h"
#include "../ratgdo_state.h"

namespace esphome {
namespace ratgdo {

static const char *const TAG = "ratgdo.binary_sensor";

void RATGDOBinarySensor::dump_config() {
  LOG_BINARY_SENSOR("", "RATGDO BinarySensor", this);
  ESP_LOGCONFIG("  Type: %s", this->type_);

}
void RATGDOBinarySensor::on_motion_state(esphome::ratgdo::MotionState state) {
  if (this->type_ == SensorType::RATGDO_SENSOR_MOTION)
    this->publish_state(state == esphome::ratgdo::MotionState::MOTION_STATE_DETECTED);
}
void RATGDOBinarySensor::on_obstruction_state(esphome::ratgdo::ObstructionState state) {
  if (this->type_ == SensorType::RATGDO_SENSOR_OBSTRUCTION)
    this->publish_state(state == esphome::ratgdo::ObstructionState::OBSTRUCTION_STATE_OBSTRUCTED);
}
}  // namespace ratgdo
}  // namespace esphome

