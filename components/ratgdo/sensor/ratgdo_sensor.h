#pragma once

#include "../ratgdo.h"
#include "../ratgdo_state.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

namespace esphome {
namespace ratgdo {

    enum RATGDOSensorType {
        RATGDO_OPENINGS,
        RATGDO_PAIRED_DEVICES_TOTAL,
        RATGDO_PAIRED_REMOTES,
        RATGDO_PAIRED_KEYPADS,
        RATGDO_PAIRED_WALL_CONTROLS,
        RATGDO_PAIRED_ACCESSORIES
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
