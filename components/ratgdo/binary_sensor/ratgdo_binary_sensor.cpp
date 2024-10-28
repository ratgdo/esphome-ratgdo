#include "ratgdo_binary_sensor.h"
#include "../ratgdo_state.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ratgdo {

    static const char* const TAG = "ratgdo.binary_sensor";

    void RATGDOBinarySensor::setup()
    {
        if (this->binary_sensor_type_ == SensorType::RATGDO_SENSOR_MOTION) {
            this->publish_initial_state(false);
            this->parent_->subscribe_motion_state([=](MotionState state) {
                this->publish_state(state == MotionState::DETECTED);
            });
        } else if (this->binary_sensor_type_ == SensorType::RATGDO_SENSOR_OBSTRUCTION) {
            this->publish_initial_state(false);
            this->parent_->subscribe_obstruction_state([=](ObstructionState state) {
                this->publish_state(state == ObstructionState::OBSTRUCTED);
            });
        } else if (this->binary_sensor_type_ == SensorType::RATGDO_SENSOR_MOTOR) {
            this->parent_->subscribe_motor_state([=](MotorState state) {
                this->publish_state(state == MotorState::ON);
            });
        } else if (this->binary_sensor_type_ == SensorType::RATGDO_SENSOR_BUTTON) {
            this->publish_initial_state(false);
            this->parent_->subscribe_button_state([=](ButtonState state) {
                this->publish_state(state == ButtonState::PRESSED);
            });
        } else if(this->binary_sensor_type_ == SensorType::RATGDO_SENSOR_VEHICLE_DETECTED) {
            this->publish_initial_state(false);
            this->parent_->subscribe_vehicle_detected_state([=](VehicleDetectedState state) {
                this->publish_state(state == VehicleDetectedState::YES);
                this->parent_->presence_change(state == VehicleDetectedState::YES);
            });
        } else if(this->binary_sensor_type_ == SensorType::RATGDO_SENSOR_VEHICLE_ARRIVING) {
            this->publish_initial_state(false);
            this->parent_->subscribe_vehicle_arriving_state([=](VehicleArrivingState state) {
                this->publish_state(state == VehicleArrivingState::YES);
            });
        } else if(this->binary_sensor_type_ == SensorType::RATGDO_SENSOR_VEHICLE_LEAVING) {
            this->publish_initial_state(false);
            this->parent_->subscribe_vehicle_leaving_state([=](VehicleLeavingState state) {
                this->publish_state(state == VehicleLeavingState::YES);
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
        } else if (this->binary_sensor_type_ == SensorType::RATGDO_SENSOR_VEHICLE_DETECTED) {
            ESP_LOGCONFIG(TAG, " Type: VehicleDetected");
        } else if (this->binary_sensor_type_ == SensorType::RATGDO_SENSOR_VEHICLE_ARRIVING) {
            ESP_LOGCONFIG(TAG, " Type: VehicleArriving");
        } else if (this->binary_sensor_type_ == SensorType::RATGDO_SENSOR_VEHICLE_LEAVING) {
            ESP_LOGCONFIG(TAG, " Type: VehicleLeaving");
        }
    }

} // namespace ratgdo
} // namespace esphome
