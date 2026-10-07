#pragma once

#include "../ratgdo.h"
#include "../ratgdo_state.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"

namespace esphome::ratgdo {

enum RATGDOTextSensorType : uint8_t {
    RATGDO_TTC_STATE,
};

class RATGDOTextSensor : public text_sensor::TextSensor, public RATGDOClient, public Component {
public:
    void dump_config() override;
    void setup() override;
    void set_ratgdo_text_sensor_type(RATGDOTextSensorType type) { this->ratgdo_text_sensor_type_ = type; }

protected:
    RATGDOTextSensorType ratgdo_text_sensor_type_;
};

} // namespace esphome::ratgdo
