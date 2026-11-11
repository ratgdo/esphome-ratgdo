#pragma once

#include "../ratgdo.h"
#include "../ratgdo_state.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"

namespace esphome::ratgdo {

enum RATGDOSensorType : uint8_t {
    RATGDO_OPENINGS,
    RATGDO_PAIRED_DEVICES_TOTAL,
    RATGDO_PAIRED_REMOTES,
    RATGDO_PAIRED_KEYPADS,
    RATGDO_PAIRED_WALL_CONTROLS,
    RATGDO_PAIRED_ACCESSORIES,
    RATGDO_DISTANCE,
    RATGDO_ENCODER,
};

class RATGDOSensor : public sensor::Sensor, public RATGDOClient, public Component {
public:
    void dump_config() override;
    void setup() override;
    void set_ratgdo_sensor_type(RATGDOSensorType ratgdo_sensor_type_) { this->ratgdo_sensor_type_ = ratgdo_sensor_type_; }

protected:
    RATGDOSensorType ratgdo_sensor_type_;
};

} // namespace esphome::ratgdo
