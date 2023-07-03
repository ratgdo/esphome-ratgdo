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

        if (this->binary_sensor_type_ == SensorType::RATGDO_SENSOR_MOTION) {
            this->parent_->subscribe_motion_state([=](MotionState state) {
                this->publish_state(state == MotionState::MOTION_STATE_DETECTED);
            });
        } else if (this->binary_sensor_type_ == SensorType::RATGDO_SENSOR_OBSTRUCTION) {
            this->parent_->subscribe_obstruction_state([=](ObstructionState state) {
                this->publish_state(state == ObstructionState::OBSTRUCTION_STATE_OBSTRUCTED);
            });
        } else if (this->binary_sensor_type_ == SensorType::RATGDO_SENSOR_MOTOR) {
            this->parent_->subscribe_motor_state([=](MotorState state) {
                this->publish_state(state == MotorState::MOTOR_STATE_ON);
            });
        } else if (this->binary_sensor_type_ == SensorType::RATGDO_SENSOR_BUTTON) {
            this->parent_->subscribe_button_state([=](ButtonState state) {
                this->publish_state(state == ButtonState::BUTTON_STATE_PRESSED);
            });
        }
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

} // namespace ratgdo
} // namespace esphome
