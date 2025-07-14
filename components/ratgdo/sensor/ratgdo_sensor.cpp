#include "ratgdo_sensor.h"
#include "../ratgdo_state.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ratgdo {

    static const char* const TAG = "ratgdo.sensor";
    static const int MIN_DISTANCE = 100; // ignore bugs crawling on the distance sensor & dust protection film
    static const int MAX_DISTANCE = 4500; // default maximum distance

    void RATGDOSensor::setup()
    {
        if (this->ratgdo_sensor_type_ == RATGDOSensorType::RATGDO_OPENINGS) {
            this->parent_->subscribe_openings([this](uint16_t value) {
                this->publish_state(value);
            });
        } else if (this->ratgdo_sensor_type_ == RATGDOSensorType::RATGDO_PAIRED_DEVICES_TOTAL) {
            this->parent_->subscribe_paired_devices_total([this](uint8_t value) {
                this->publish_state(value);
            });
        } else if (this->ratgdo_sensor_type_ == RATGDOSensorType::RATGDO_PAIRED_REMOTES) {
            this->parent_->subscribe_paired_remotes([this](uint8_t value) {
                this->publish_state(value);
            });
        } else if (this->ratgdo_sensor_type_ == RATGDOSensorType::RATGDO_PAIRED_KEYPADS) {
            this->parent_->subscribe_paired_keypads([this](uint8_t value) {
                this->publish_state(value);
            });
        } else if (this->ratgdo_sensor_type_ == RATGDOSensorType::RATGDO_PAIRED_WALL_CONTROLS) {
            this->parent_->subscribe_paired_wall_controls([this](uint8_t value) {
                this->publish_state(value);
            });
        } else if (this->ratgdo_sensor_type_ == RATGDOSensorType::RATGDO_PAIRED_ACCESSORIES) {
            this->parent_->subscribe_paired_accessories([this](uint8_t value) {
                this->publish_state(value);
            });
        } else if (this->ratgdo_sensor_type_ == RATGDOSensorType::RATGDO_DISTANCE) {
#ifdef USE_DISTANCE
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
#endif
        }
    }

    void RATGDOSensor::dump_config()
    {
        LOG_SENSOR("", "RATGDO Sensor", this);
        if (this->ratgdo_sensor_type_ == RATGDOSensorType::RATGDO_OPENINGS) {
            ESP_LOGCONFIG(TAG, "  Type: Openings");
        } else if (this->ratgdo_sensor_type_ == RATGDOSensorType::RATGDO_PAIRED_DEVICES_TOTAL) {
            ESP_LOGCONFIG(TAG, "  Type: Paired Devices");
        } else if (this->ratgdo_sensor_type_ == RATGDOSensorType::RATGDO_PAIRED_REMOTES) {
            ESP_LOGCONFIG(TAG, "  Type: Paired Remotes");
        } else if (this->ratgdo_sensor_type_ == RATGDOSensorType::RATGDO_PAIRED_KEYPADS) {
            ESP_LOGCONFIG(TAG, "  Type: Paired Keypads");
        } else if (this->ratgdo_sensor_type_ == RATGDOSensorType::RATGDO_PAIRED_WALL_CONTROLS) {
            ESP_LOGCONFIG(TAG, "  Type: Paired Wall Controls");
        } else if (this->ratgdo_sensor_type_ == RATGDOSensorType::RATGDO_PAIRED_ACCESSORIES) {
            ESP_LOGCONFIG(TAG, "  Type: Paired Accessories");
        } else if (this->ratgdo_sensor_type_ == RATGDOSensorType::RATGDO_DISTANCE) {
            ESP_LOGCONFIG(TAG, "  Type: Distance");
        }
    }

    void RATGDOSensor::loop()
    {
#ifdef USE_DISTANCE
        if (this->ratgdo_sensor_type_ == RATGDOSensorType::RATGDO_DISTANCE) {
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
                this->parent_->set_distance_measurement(maxDistance);

                // ESP_LOGD(TAG,"# obj found %d; distance %d",objCount, maxDistance);

                if (status == 0) {
                    status = this->distance_sensor_.VL53L4CX_ClearInterruptAndStartMeasurement();
                }
            }
        }
#endif
    }

} // namespace ratgdo
} // namespace esphome
