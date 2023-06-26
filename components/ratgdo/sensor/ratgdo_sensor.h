#pragma once

#include "../ratgdo.h"
#include "../ratgdo_state.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

namespace esphome {
namespace ratgdo {

    enum RATGDOSensorType {
        RATGDO_OPENINGS
    };

    class RATGDOSensor : public sensor::Sensor, public RATGDOClient, public Component {
    public:
        void dump_config() override;
        void setup() override;
        void set_ratgdo_sensor_type(RATGDOSensorType ratgdo_sensor_type_) { this->ratgdo_sensor_type_ = ratgdo_sensor_type_; }

    protected:
        RATGDOSensorType ratgdo_sensor_type_;
    };

} // namespace ratgdo
} // namespace esphome
