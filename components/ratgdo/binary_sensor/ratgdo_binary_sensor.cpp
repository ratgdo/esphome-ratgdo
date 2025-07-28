#include "ratgdo_binary_sensor.h"
#include "../ratgdo_state.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ratgdo {

    static const char* const TAG = "ratgdo.binary_sensor";

    void RATGDOBinarySensor::setup()
    {
        switch (this->binary_sensor_type_) {
        case SensorType::RATGDO_SENSOR_MOTION:
            this->publish_initial_state(false);
            this->parent_->subscribe_motion_state([this](MotionState state) {
                this->publish_state(state == MotionState::DETECTED);
            });
            break;
        case SensorType::RATGDO_SENSOR_OBSTRUCTION:
            this->publish_initial_state(false);
            this->parent_->subscribe_obstruction_state([this](ObstructionState state) {
                this->publish_state(state == ObstructionState::OBSTRUCTED);
            });
            break;
        case SensorType::RATGDO_SENSOR_MOTOR:
            this->parent_->subscribe_motor_state([this](MotorState state) {
                this->publish_state(state == MotorState::ON);
            });
            break;
        case SensorType::RATGDO_SENSOR_BUTTON:
            this->publish_initial_state(false);
            this->parent_->subscribe_button_state([this](ButtonState state) {
                this->publish_state(state == ButtonState::PRESSED);
            });
            break;
#ifdef RATGDO_USE_VEHICLE_SENSORS
        case SensorType::RATGDO_SENSOR_VEHICLE_DETECTED:
            this->publish_initial_state(false);
            this->parent_->subscribe_vehicle_detected_state([this](VehicleDetectedState state) {
                this->publish_state(state == VehicleDetectedState::YES);
                this->parent_->presence_change(state == VehicleDetectedState::YES);
            });
            break;
        case SensorType::RATGDO_SENSOR_VEHICLE_ARRIVING:
            this->publish_initial_state(false);
            this->parent_->subscribe_vehicle_arriving_state([this](VehicleArrivingState state) {
                this->publish_state(state == VehicleArrivingState::YES);
            });
            break;
        case SensorType::RATGDO_SENSOR_VEHICLE_LEAVING:
            this->publish_initial_state(false);
            this->parent_->subscribe_vehicle_leaving_state([this](VehicleLeavingState state) {
                this->publish_state(state == VehicleLeavingState::YES);
            });
            break;
#endif
        default:
            break;
        }
    }

    void RATGDOBinarySensor::dump_config()
    {
        LOG_BINARY_SENSOR("", "RATGDO BinarySensor", this);
        switch (this->binary_sensor_type_) {
        case SensorType::RATGDO_SENSOR_MOTION:
            ESP_LOGCONFIG(TAG, "  Type: Motion");
            break;
        case SensorType::RATGDO_SENSOR_OBSTRUCTION:
            ESP_LOGCONFIG(TAG, "  Type: Obstruction");
            break;
        case SensorType::RATGDO_SENSOR_MOTOR:
            ESP_LOGCONFIG(TAG, "  Type: Motor");
            break;
        case SensorType::RATGDO_SENSOR_BUTTON:
            ESP_LOGCONFIG(TAG, "  Type: Button");
            break;
#ifdef RATGDO_USE_VEHICLE_SENSORS
        case SensorType::RATGDO_SENSOR_VEHICLE_DETECTED:
            ESP_LOGCONFIG(TAG, "  Type: VehicleDetected");
            break;
        case SensorType::RATGDO_SENSOR_VEHICLE_ARRIVING:
            ESP_LOGCONFIG(TAG, "  Type: VehicleArriving");
            break;
        case SensorType::RATGDO_SENSOR_VEHICLE_LEAVING:
            ESP_LOGCONFIG(TAG, "  Type: VehicleLeaving");
            break;
#endif
        default:
            break;
        }
    }

} // namespace ratgdo
} // namespace esphome
