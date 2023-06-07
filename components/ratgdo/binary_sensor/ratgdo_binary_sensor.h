#pragma once

#include "esphome/core/component.h"
#include "../ratgdo_child.h"
#include "../ratgdo_state.h"
#include "../ratgdo.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

namespace esphome {
namespace ratgdo {

class RATGDOBinarySensor : public binary_sensor::BinarySensor, public RATGDOClient, public Component {
 public:
  void dump_config() override;

  void on_door_state(esphome::ratgdo::DoorState state) override;
  void on_light_state(esphome::ratgdo::LightState state) override;
  void on_lock_state(esphome::ratgdo::LockState state) override;
  void on_motion_state(esphome::ratgdo::MotionState state) override;
  void on_obstruction_state(esphome::ratgdo::ObstructionState state) override;

};

}  // namespace ratgdo
}  // namespace esphome

