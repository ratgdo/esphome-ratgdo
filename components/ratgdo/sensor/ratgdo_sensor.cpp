#include "ratgdo_sensor.h"
#include "../ratgdo_state.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ratgdo {

    static const char* const TAG = "ratgdo.sensor";

    void RATGDOSensor::dump_config()
    {
        LOG_SENSOR("", "RATGDO Sensor", this);
        ESP_LOGCONFIG(TAG, "  Type: Openings");
    }
    void RATGDOSensor::on_openings_change(uint32_t openings)
    {
        this->publish_state(openings);
    }

} // namespace ratgdo
} // namespace esphome
