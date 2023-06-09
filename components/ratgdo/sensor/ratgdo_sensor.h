#pragma once

#include "../ratgdo.h"
#include "../ratgdo_child.h"
#include "../ratgdo_state.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

namespace esphome {
namespace ratgdo {

    enum SensorType {
        RATGDO_OPENINGS
    };

    class RATGDOSensor : public sensor::Sensor, public RATGDOClient, public Component {
    public:
        void setup() override;
        void dump_config() override;
        void set_sensor_type(SensorType sensor_type_) { this->sensor_type_ = sensor_type_; }

        void on_openings_change(uint32_t openings) override;

    protected:
        SensorType sensor_type_;
    };

} // namespace ratgdo
} // namespace esphome
