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
        RATGDO_SENSOR_MOTOR
    };

    class RATGDOBinarySensor : public binary_sensor::BinarySensor, public RATGDOClient, public Component {
    public:
        void setup() override;
        void dump_config() override;
        void set_binary_sensor_type(SensorType binary_sensor_type_) { this->binary_sensor_type_ = binary_sensor_type_; }

        void on_motion_state(MotionState state) override;
        void on_obstruction_state(ObstructionState state) override;
        void on_motor_state(MotorState state) override;

    protected:
        SensorType binary_sensor_type_;
    };

} // namespace ratgdo
} // namespace esphome
