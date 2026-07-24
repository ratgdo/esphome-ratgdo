#include "ratgdo_sensor.h"
#include "../ratgdo_state.h"
#include "esphome/core/log.h"

namespace esphome::ratgdo {

static const char* const TAG = "ratgdo.sensor";

void RATGDOSensor::setup()
{
    switch (this->ratgdo_sensor_type_) {
    case RATGDOSensorType::RATGDO_OPENINGS:
        this->parent_->subscribe_openings([this](uint16_t value) {
            this->publish_state(value);
        });
        break;
    case RATGDOSensorType::RATGDO_PAIRED_DEVICES_TOTAL:
        this->parent_->subscribe_paired_devices_total([this](uint8_t value) {
            this->publish_state(value);
        });
        break;
    case RATGDOSensorType::RATGDO_PAIRED_REMOTES:
        this->parent_->subscribe_paired_remotes([this](uint8_t value) {
            this->publish_state(value);
        });
        break;
    case RATGDOSensorType::RATGDO_PAIRED_KEYPADS:
        this->parent_->subscribe_paired_keypads([this](uint8_t value) {
            this->publish_state(value);
        });
        break;
    case RATGDOSensorType::RATGDO_PAIRED_WALL_CONTROLS:
        this->parent_->subscribe_paired_wall_controls([this](uint8_t value) {
            this->publish_state(value);
        });
        break;
    case RATGDOSensorType::RATGDO_PAIRED_ACCESSORIES:
        this->parent_->subscribe_paired_accessories([this](uint8_t value) {
            this->publish_state(value);
        });
        break;
    case RATGDOSensorType::RATGDO_ENCODER:
#ifdef RATGDO_USE_ENCODER
        this->parent_->set_encoder_sensor(this);
#endif
        break;
    default:
        break;
    }
}

void RATGDOSensor::dump_config()
{
    LOG_SENSOR("", "RATGDO Sensor", this);
    switch (this->ratgdo_sensor_type_) {
    case RATGDOSensorType::RATGDO_OPENINGS:
        ESP_LOGCONFIG(TAG, "  Type: Openings");
        break;
    case RATGDOSensorType::RATGDO_PAIRED_DEVICES_TOTAL:
        ESP_LOGCONFIG(TAG, "  Type: Paired Devices");
        break;
    case RATGDOSensorType::RATGDO_PAIRED_REMOTES:
        ESP_LOGCONFIG(TAG, "  Type: Paired Remotes");
        break;
    case RATGDOSensorType::RATGDO_PAIRED_KEYPADS:
        ESP_LOGCONFIG(TAG, "  Type: Paired Keypads");
        break;
    case RATGDOSensorType::RATGDO_PAIRED_WALL_CONTROLS:
        ESP_LOGCONFIG(TAG, "  Type: Paired Wall Controls");
        break;
    case RATGDOSensorType::RATGDO_PAIRED_ACCESSORIES:
        ESP_LOGCONFIG(TAG, "  Type: Paired Accessories");
        break;
    case RATGDOSensorType::RATGDO_ENCODER:
        ESP_LOGCONFIG(TAG, "  Type: Encoder");
        break;
    default:
        break;
    }
}

} // namespace esphome::ratgdo
