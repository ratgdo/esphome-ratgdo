#include "ratgdo_binary_sensor.h"
#include "../ratgdo_state.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ratgdo {

    static const char* const TAG = "ratgdo.binary_sensor";

    void RATGDOBinarySensor::setup()
    {
        if (this->binary_sensor_type_ == SensorType::RATGDO_SENSOR_MOTION || this->binary_sensor_type_ == SensorType::RATGDO_SENSOR_OBSTRUCTION || this->binary_sensor_type_ == SensorType::RATGDO_SENSOR_BUTTON)
            this->publish_state(false);
    }

    void RATGDOBinarySensor::dump_config()
    {
        LOG_BINARY_SENSOR("", "RATGDO BinarySensor", this);
        if (this->binary_sensor_type_ == SensorType::RATGDO_SENSOR_MOTION) {
            ESP_LOGCONFIG(TAG, "  Type: Motion");
        } else if (this->binary_sensor_type_ == SensorType::RATGDO_SENSOR_OBSTRUCTION) {
            ESP_LOGCONFIG(TAG, "  Type: Obstruction");
        } else if (this->binary_sensor_type_ == SensorType::RATGDO_SENSOR_MOTOR) {
            ESP_LOGCONFIG(TAG, "  Type: Motor");
        } else if (this->binary_sensor_type_ == SensorType::RATGDO_SENSOR_BUTTON) {
            ESP_LOGCONFIG(TAG, "  Type: Button");
        }
    }
    void RATGDOBinarySensor::on_motion_state(MotionState state)
    {
        if (this->binary_sensor_type_ != SensorType::RATGDO_SENSOR_MOTION)
            return;
        this->publish_state(state == MotionState::MOTION_STATE_DETECTED);
    }
    void RATGDOBinarySensor::on_obstruction_state(ObstructionState state)
    {
        if (this->binary_sensor_type_ != SensorType::RATGDO_SENSOR_OBSTRUCTION)
            return;
        this->publish_state(state == ObstructionState::OBSTRUCTION_STATE_OBSTRUCTED);
    }
    void RATGDOBinarySensor::on_motor_state(MotorState state)
    {
        if (this->binary_sensor_type_ != SensorType::RATGDO_SENSOR_MOTOR)
            return;
        this->publish_state(state == MotorState::MOTOR_STATE_ON);
    }
    void RATGDOBinarySensor::on_button_state(ButtonState state)
    {
        if (this->binary_sensor_type_ != SensorType::RATGDO_SENSOR_BUTTON)
            return;
        this->publish_state(state == ButtonState::BUTTON_STATE_PRESSED);
    }

} // namespace ratgdo
} // namespace esphome
