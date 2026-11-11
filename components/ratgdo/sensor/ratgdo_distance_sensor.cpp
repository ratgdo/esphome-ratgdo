#include "ratgdo_distance_sensor.h"

#ifdef RATGDO_USE_DISTANCE_SENSOR
#include "../common.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

namespace esphome::ratgdo {

ESPHOME_LOG_TAG(TAG, "ratgdo.sensor");
static const int MIN_DISTANCE = 100; // ignore bugs crawling on the distance sensor & dust protection film
static const int MAX_DISTANCE = 4500; // default maximum distance

void RATGDODistanceSensor::setup()
{
    this->distance_sensor_.setI2cDevice(&I2C);
    this->distance_sensor_.setXShutPin(32);
    // I2C.begin(17,16);
    I2C.begin(19, 18);
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

static const uint32_t DISTANCE_POLL_INTERVAL_MS = 50;

void RATGDODistanceSensor::loop()
{
    uint32_t now = App.get_loop_component_start_time();
    if (now - this->last_distance_poll_ms_ < DISTANCE_POLL_INTERVAL_MS)
        return;
    this->last_distance_poll_ms_ = now;

    VL53L4CX_MultiRangingData_t distanceData;
    VL53L4CX_MultiRangingData_t* pDistanceData = &distanceData;
    uint8_t dataReady = 0;
    int objCount = 0;
    int16_t maxDistance = -1;
    int status;

    if (this->distance_sensor_.VL53L4CX_GetMeasurementDataReady(&dataReady) == 0 && dataReady) {
        status = this->distance_sensor_.VL53L4CX_GetMultiRangingData(pDistanceData);
        objCount = pDistanceData->NumberOfObjectsFound;

        for (int i = 0; i < distanceData.NumberOfObjectsFound; i++) {
            VL53L4CX_TargetRangeData_t* d = &pDistanceData->RangeData[i];
            if (d->RangeStatus == 0) {
                maxDistance = std::max(maxDistance, d->RangeMilliMeter);
                maxDistance = maxDistance <= MIN_DISTANCE ? -1 : maxDistance;
            }
        }

        if (maxDistance < 0)
            maxDistance = MAX_DISTANCE;

        // maxDistance = objCount == 0 ? -1 : pDistanceData->RangeData[objCount - 1].RangeMilliMeter;
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
    }
}

} // namespace esphome::ratgdo

#endif
