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
    enum class InterruptMode : uint8_t {
        MODE_PROBING,
        MODE_INTERRUPT,
        MODE_POLLING,
    };

    static void IRAM_ATTR isr_data_ready(void* arg);

    VL53L4CX distance_sensor_;
    uint32_t last_distance_poll_ms_ { 0 };
    int16_t last_distance_ { -1 };
    InterruptMode interrupt_mode_ { InterruptMode::MODE_PROBING };
    uint8_t interrupt_probe_count_ { 0 };
};

} // namespace esphome::ratgdo

#endif
