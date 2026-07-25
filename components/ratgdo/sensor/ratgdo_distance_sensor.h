#pragma once

#include "ratgdo_sensor.h"

#ifdef RATGDO_USE_DISTANCE_SENSOR
#include "Wire.h"
#include "vl53l4cx_class.h"
#define I2C Wire

namespace esphome::ratgdo {

class RATGDODistanceSensor : public RATGDOSensor {
public:
    void dump_config() override;
    void setup() override;
    void loop() override;

protected:
    VL53L4CX distance_sensor_;
};

} // namespace esphome::ratgdo

#endif
