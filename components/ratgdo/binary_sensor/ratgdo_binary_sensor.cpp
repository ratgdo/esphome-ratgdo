#include "ratgdo_binary_sensor.h"
#include "../ratgdo_state.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ratgdo {

    static const char* const TAG = "ratgdo.binary_sensor";

    void RATGDOBinarySensor::setup()
    {
        if (this->binary_sensor_type_ == SensorType::RATGDO_SENSOR_MOTION || this->binary_sensor_type_ == SensorType::RATGDO_SENSOR_OBSTRUCTION) {
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
        ESP_LOGD(TAG, "name: %s this->type_:%d on_motion_state: %d", this->get_name(), this->binary_sensor_type_, state);
        this->publish_state(state == MotionState::MOTION_STATE_DETECTED);
    }
    void RATGDOBinarySensor::on_obstruction_state(ObstructionState state)
    {
        if (this->binary_sensor_type_ != SensorType::RATGDO_SENSOR_OBSTRUCTION)
            return;
        ESP_LOGD(TAG, "name: %s this->type_:%d on_obstruction_state: %d", this->get_name(), this->binary_sensor_type_, state);
        this->publish_state(state == ObstructionState::OBSTRUCTION_STATE_OBSTRUCTED);
    }
    void RATGDOBinarySensor::on_motor_state(MotorState state)
    {
        if (this->binary_sensor_type_ != SensorType::RATGDO_SENSOR_MOTOR)
            return;
        ESP_LOGD(TAG, "name: %s this->type_:%d on_motor_state: %d", this->get_name(), this->binary_sensor_type_, state);
        this->publish_state(state == MotorState::MOTOR_STATE_ON);
    }
    void RATGDOBinarySensor::on_button_state(ButtonState state)
    {
        if (this->binary_sensor_type_ != SensorType::RATGDO_SENSOR_BUTTON)
            return;
        ESP_LOGD(TAG, "name: %s this->type_:%d on_button_state: %d", this->get_name(), this->binary_sensor_type_, state);
        this->publish_state(state == ButtonState::BUTTON_STATE_PRESSED);
    }

} // namespace ratgdo
} // namespace esphome
