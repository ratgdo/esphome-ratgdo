#include "ratgdo_sensor.h"
#include "../ratgdo_state.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ratgdo {

    static const char* const TAG = "ratgdo.sensor";

    void RATGDOSensor::setup()
    {
        if (this->ratgdo_sensor_type_ == RATGDOSensorType::RATGDO_OPENINGS) {
            this->parent_->subscribe_openings([=](uint16_t value) {
                this->publish_state(value);
            });
        } else if (this->ratgdo_sensor_type_ == RATGDOSensorType::RATGDO_PAIRED_DEVICES_TOTAL) {
            this->parent_->subscribe_paired_devices_total([=](uint16_t value) {
                this->publish_state(value);
            });
        } else if (this->ratgdo_sensor_type_ == RATGDOSensorType::RATGDO_PAIRED_REMOTES) {
            this->parent_->subscribe_paired_remotes([=](uint16_t value) {
                this->publish_state(value);
            });
        } else if (this->ratgdo_sensor_type_ == RATGDOSensorType::RATGDO_PAIRED_KEYPADS) {
            this->parent_->subscribe_paired_keypads([=](uint16_t value) {
                this->publish_state(value);
            });
        } else if (this->ratgdo_sensor_type_ == RATGDOSensorType::RATGDO_PAIRED_WALL_CONTROLS) {
            this->parent_->subscribe_paired_wall_controls([=](uint16_t value) {
                this->publish_state(value);
            });
        } else if (this->ratgdo_sensor_type_ == RATGDOSensorType::RATGDO_PAIRED_ACCESSORIES) {
            this->parent_->subscribe_paired_accessories([=](uint16_t value) {
                this->publish_state(value);
            });
        }
    }

    void RATGDOSensor::dump_config()
    {
        LOG_SENSOR("", "RATGDO Sensor", this);
        if (this->ratgdo_sensor_type_ == RATGDOSensorType::RATGDO_OPENINGS) {
            ESP_LOGCONFIG(TAG, "  Type: Openings");
        } else if (this->ratgdo_sensor_type_ == RATGDOSensorType::RATGDO_PAIRED_DEVICES_TOTAL) {
            ESP_LOGCONFIG(TAG, "  Type: Paired Devices");
        } else if (this->ratgdo_sensor_type_ == RATGDOSensorType::RATGDO_PAIRED_REMOTES) {
            ESP_LOGCONFIG(TAG, "  Type: Paired Remotes");
        } else if (this->ratgdo_sensor_type_ == RATGDOSensorType::RATGDO_PAIRED_KEYPADS) {
            ESP_LOGCONFIG(TAG, "  Type: Paired Keypads");
        } else if (this->ratgdo_sensor_type_ == RATGDOSensorType::RATGDO_PAIRED_WALL_CONTROLS) {
            ESP_LOGCONFIG(TAG, "  Type: Paired Wall Controls");
        } else if (this->ratgdo_sensor_type_ == RATGDOSensorType::RATGDO_PAIRED_ACCESSORIES) {
            ESP_LOGCONFIG(TAG, "  Type: Paired Accessories");
        }
    }

} // namespace ratgdo
} // namespace esphome
