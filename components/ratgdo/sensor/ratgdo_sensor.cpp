#include "ratgdo_sensor.h"
#include "../ratgdo_state.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ratgdo {

    static const char* const TAG = "ratgdo.sensor";

    void RATGDOSensor::dump_config()
    {
        LOG_SENSOR("", "RATGDO Sensor", this);
        if (this->ratgdo_sensor_type_ == RATGDOSensorType::RATGDO_OPENINGS) {
            ESP_LOGCONFIG(TAG, "  Type: Openings");
        } else {
            ESP_LOGCONFIG(TAG, "  Type: Auto Close time");
        }
    }
    void RATGDOSensor::on_openings_change(uint32_t openings)
    {
        if (this->ratgdo_sensor_type_ == RATGDOSensorType::RATGDO_OPENINGS)
            this->publish_state(openings);
    }
    void RATGDOSensor::on_auto_close_time_change(time_t autoCloseTime)
    {
        if (this->ratgdo_sensor_type_ == RATGDOSensorType::RATGDO_AUTO_CLOSE_TIME && autoCloseTime != 0) {
            this->publish_state(autoCloseTime);
        }
    }

} // namespace ratgdo
} // namespace esphome
