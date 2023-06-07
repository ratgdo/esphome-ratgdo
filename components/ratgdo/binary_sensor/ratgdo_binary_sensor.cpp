#include "ratgdo_binary_sensor.h"
#include "esphome/core/log.h"
#include "../ratgdo_state.h"

#ifdef USE_ESP32

namespace esphome {
namespace ratgdo {

using namespace esphome::binary_sensor;

void RATGDOBinarySensor::dump_config() {
  LOG_BINARY_SENSOR("", "RATGDO BinarySensor", this);
}

void RATGDOBinarySensor::setup() {}
void RATGDOBinarySensor::on_door_state(esphome::ratgdo::DoorState state) {}
void RATGDOBinarySensor::on_light_state(esphome::ratgdo::LightState state) {}
void RATGDOBinarySensor::on_lock_state(esphome::ratgdo::LockState state) {}
void RATGDOBinarySensor::on_motion_state(esphome::ratgdo::MotionState state) {}
void RATGDOBinarySensor::on_obstruction_state(esphome::ratgdo::ObstructionState state) {}

void RATGDOBinarySensor::loop() {}

}  // namespace ratgdo
}  // namespace esphome

#endif
