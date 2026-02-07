#pragma once

#include "../ratgdo.h"
#include "../ratgdo_state.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"

namespace esphome {
namespace ratgdo {

    enum SensorType : uint8_t {
        RATGDO_SENSOR_MOTION,
        RATGDO_SENSOR_OBSTRUCTION,
        RATGDO_SENSOR_MOTOR,
        RATGDO_SENSOR_BUTTON,
#ifdef RATGDO_USE_VEHICLE_SENSORS
        RATGDO_SENSOR_VEHICLE_DETECTED,
        RATGDO_SENSOR_VEHICLE_ARRIVING,
        RATGDO_SENSOR_VEHICLE_LEAVING,
#endif
    };

    class RATGDOBinarySensor : public binary_sensor::BinarySensor, public RATGDOClient, public Component {
    public:
        void setup() override;
        void dump_config() override;
        void set_binary_sensor_type(SensorType binary_sensor_type) { this->binary_sensor_type_ = binary_sensor_type; }

    protected:
        SensorType binary_sensor_type_;
    };

} // namespace ratgdo
} // namespace esphome
