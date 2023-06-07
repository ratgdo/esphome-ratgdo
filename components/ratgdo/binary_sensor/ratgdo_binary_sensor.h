#pragma once

#include "../ratgdo.h"
#include "../ratgdo_child.h"
#include "../ratgdo_state.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/core/component.h"

namespace esphome {
namespace ratgdo {

    enum SensorType {
        RATGDO_SENSOR_MOTION,
        RATGDO_SENSOR_OBSTRUCTION,
    };

    class RATGDOBinarySensor : public binary_sensor::BinarySensor, public RATGDOClient, public Component {
    public:
        void dump_config() override;
        void set_type(SensorType type) { type_ = type_; }

        void on_motion_state(esphome::ratgdo::MotionState state) override;
        void on_obstruction_state(esphome::ratgdo::ObstructionState state) override;
        void on_door_state(esphome::ratgdo::DoorState state) override;
        void on_light_state(esphome::ratgdo::LightState state) override;
        void on_lock_state(esphome::ratgdo::LockState state) override;

    protected:
        SensorType type_;
    };

} // namespace ratgdo
} // namespace esphome
