#pragma once

#include "../ratgdo.h"
#include "../ratgdo_child.h"
#include "../ratgdo_state.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

namespace esphome {
namespace ratgdo {

    enum RATGDOSensorType {
        RATGDO_OPENINGS,
        RATGDO_AUTO_CLOSE_TIME
    };

    class RATGDOSensor : public sensor::Sensor, public RATGDOClient, public Component {
    public:
        void dump_config() override;
        void set_ratgdo_sensor_type(RATGDOSensorType ratgdo_sensor_type_) { this->ratgdo_sensor_type_ = ratgdo_sensor_type_; }

        void on_openings_change(uint32_t openings) override;
        void on_auto_close_time_change(time_t autoCloseTime) override;

    protected:
        RATGDOSensorType ratgdo_sensor_type_;
    };

} // namespace ratgdo
} // namespace esphome
