#include "ratgdo_binary_sensor.h"
#include "esphome/core/log.h"
#include "../ratgdo_state.h"

namespace esphome {
namespace ratgdo {

static const char *const TAG = "ratgdo.binary_sensor";

class RATGDOBinarySensor : public binary_sensor::BinarySensor, public RATGDOClient, public Component {
void dump_config() {
  LOG_BINARY_SENSOR("", "RATGDO BinarySensor", this);
}

void on_door_state(esphome::ratgdo::DoorState state) {}
void on_light_state(esphome::ratgdo::LightState state) {}
void on_lock_state(esphome::ratgdo::LockState state) {}
void on_motion_state(esphome::ratgdo::MotionState state) {}
void on_obstruction_state(esphome::ratgdo::ObstructionState state) {}
 };
}  // namespace ratgdo
}  // namespace esphome

