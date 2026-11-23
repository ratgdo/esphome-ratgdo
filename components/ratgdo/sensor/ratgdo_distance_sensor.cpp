#include "ratgdo_distance_sensor.h"

#ifdef RATGDO_USE_DISTANCE_SENSOR
#include "../common.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

namespace esphome::ratgdo {

ESPHOME_LOG_TAG(TAG, "ratgdo.sensor");
static constexpr int MIN_DISTANCE = 100; // ignore bugs crawling on the distance sensor & dust protection film
static constexpr int MAX_DISTANCE = 4500; // default maximum distance
static constexpr uint8_t XSHUT_PIN = 32;
static constexpr uint8_t INTERRUPT_PIN = 35;
static constexpr uint8_t INTERRUPT_DETECT_SAMPLES = 3;
static constexpr uint8_t INTERRUPT_PROBE_MAX_FAILURES = 3;
static constexpr uint32_t DISTANCE_INTERRUPT_WATCHDOG_MS = 250;

void IRAM_ATTR RATGDODistanceSensor::isr_data_ready(void* arg)
{
    static_cast<RATGDODistanceSensor*>(arg)->enable_loop_soon_any_context();
}

void RATGDODistanceSensor::setup()
{
    pinMode(INTERRUPT_PIN, INPUT);
    this->distance_sensor_.setI2cDevice(&I2C);
    this->distance_sensor_.setXShutPin(XSHUT_PIN);
    // I2C.begin(17,16);
    I2C.begin(19, 18);
    // Wire defaults to 100 kHz, where each reading spends about 19 ms on the bus; the sensor supports 400 kHz
    I2C.setClock(400000);
    this->distance_sensor_.begin();
    this->distance_sensor_.VL53L4CX_Off();
    this->distance_sensor_.InitSensor(0x59);
    this->distance_sensor_.VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_LONG);
    this->distance_sensor_.VL53L4CX_StartMeasurement();
    this->parent_->subscribe_distance_measurement([this](int16_t value) {
        this->publish_state(value);
    });
}

void RATGDODistanceSensor::dump_config()
{
    RATGDOSensor::dump_config();
    ESP_LOGCONFIG(TAG, "  Type: Distance");
}

void RATGDODistanceSensor::loop()
{
    uint32_t now = App.get_loop_component_start_time();
    uint8_t dataReady = 0;

    if (this->interrupt_mode_ == InterruptMode::MODE_INTERRUPT) {
        if (digitalRead(INTERRUPT_PIN) == LOW) {
            dataReady = 1;
            this->last_distance_poll_ms_ = now;
        } else if (now - this->last_distance_poll_ms_ >= DISTANCE_INTERRUPT_WATCHDOG_MS) {
            this->last_distance_poll_ms_ = now;
            if (this->distance_sensor_.VL53L4CX_GetMeasurementDataReady(&dataReady) != 0) {
                dataReady = 0;
            }
        } else {
            // Nothing pending; the ISR or the watchdog interval re-enables the loop
            this->disable_loop();
            return;
        }
    } else if (this->distance_sensor_.VL53L4CX_GetMeasurementDataReady(&dataReady) != 0) {
        dataReady = 0;
    }

    if (!dataReady)
        return;

    bool low_before_clear = false;
    if (this->interrupt_mode_ == InterruptMode::MODE_PROBING) {
        low_before_clear = (digitalRead(INTERRUPT_PIN) == LOW);
    }

    VL53L4CX_MultiRangingData_t distanceData;
    int16_t maxDistance = -1;
    int status = this->distance_sensor_.VL53L4CX_GetMultiRangingData(&distanceData);
    int objCount = distanceData.NumberOfObjectsFound;

    for (int i = 0; i < objCount; i++) {
        VL53L4CX_TargetRangeData_t* d = &distanceData.RangeData[i];
        if (d->RangeStatus == 0) {
            maxDistance = std::max(maxDistance, d->RangeMilliMeter);
            maxDistance = maxDistance <= MIN_DISTANCE ? -1 : maxDistance;
        }
    }

    if (maxDistance < 0)
        maxDistance = MAX_DISTANCE;

    /*
     * if the sensor is pointed at glass, there are many error -1 readings which will fill the
     * vector with out of range data. The sensor should be sensitive enough to detect the floor
     * in most situations, but daylight and/or really high ceilings can cause long distance
     * measurements to be out of range.
     */
    if (maxDistance != this->last_distance_) {
        ESP_LOGI(TAG, "distance changed: %d -> %d (obj=%d)", this->last_distance_, maxDistance, objCount);
        this->last_distance_ = maxDistance;
    }
    this->parent_->set_distance_measurement(maxDistance);

    if (status == 0) {
        status = this->distance_sensor_.VL53L4CX_ClearInterruptAndStartMeasurement();
    }

    if (status != 0) {
        // Without a successful clear the pin stays low, so in interrupt mode retry on the
        // watchdog interval instead of on every loop. An I2C error says nothing about the
        // pin either, so it does not count as a probe sample.
        if (this->interrupt_mode_ == InterruptMode::MODE_INTERRUPT)
            this->disable_loop();
        return;
    }

    if (this->interrupt_mode_ != InterruptMode::MODE_PROBING)
        return;

    delayMicroseconds(2);
    if (!low_before_clear || digitalRead(INTERRUPT_PIN) != HIGH) {
        this->interrupt_probe_count_ = 0;
        if (++this->interrupt_probe_failures_ >= INTERRUPT_PROBE_MAX_FAILURES) {
            this->interrupt_mode_ = InterruptMode::MODE_POLLING;
            ESP_LOGI(TAG, "VL53L4CX interrupt not detected, polling");
        }
        return;
    }
    if (++this->interrupt_probe_count_ < INTERRUPT_DETECT_SAMPLES)
        return;

    this->interrupt_mode_ = InterruptMode::MODE_INTERRUPT;
    attachInterruptArg(INTERRUPT_PIN, &RATGDODistanceSensor::isr_data_ready, this, FALLING);
    // Wakes the loop so the watchdog poll still runs if an interrupt is missed
    this->set_interval(DISTANCE_INTERRUPT_WATCHDOG_MS, [this]() { this->enable_loop(); });
    ESP_LOGI(TAG, "VL53L4CX interrupt detected on GPIO%u", INTERRUPT_PIN);
}

} // namespace esphome::ratgdo

#endif
