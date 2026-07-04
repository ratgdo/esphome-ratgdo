import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.types import ConfigType

from .. import (
    RATGDO_CLIENT_SCHMEA,
    ratgdo_ns,
    register_ratgdo_child,
    subscribe_distance,
    validate_unique,
)

CONF_DISTANCE = "distance"

DEPENDENCIES = ["ratgdo"]

RATGDOSensor = ratgdo_ns.class_("RATGDOSensor", sensor.Sensor, cg.Component)
RATGDOSensorType = ratgdo_ns.enum("RATGDOSensorType")

CONF_TYPE = "type"
TYPES = {
    "openings": RATGDOSensorType.RATGDO_OPENINGS,
    "paired_devices_total": RATGDOSensorType.RATGDO_PAIRED_DEVICES_TOTAL,
    "paired_devices_remotes": RATGDOSensorType.RATGDO_PAIRED_REMOTES,
    "paired_devices_keypads": RATGDOSensorType.RATGDO_PAIRED_KEYPADS,
    "paired_devices_wall_controls": RATGDOSensorType.RATGDO_PAIRED_WALL_CONTROLS,
    "paired_devices_accessories": RATGDOSensorType.RATGDO_PAIRED_ACCESSORIES,
    "distance": RATGDOSensorType.RATGDO_DISTANCE,
    "encoder": RATGDOSensorType.RATGDO_ENCODER,
}


def validate_unique_type(config: ConfigType) -> ConfigType:
    """Validate that each sensor type is only used once."""
    sensor_type = config[CONF_TYPE]
    validate_unique(
        "sensor", sensor_type, f"Only one sensor of type '{sensor_type}' is allowed"
    )
    return config


CONFIG_SCHEMA = cv.All(
    sensor.sensor_schema(RATGDOSensor)
    .extend(
        {
            cv.Required(CONF_TYPE): cv.enum(TYPES, lower=True),
        }
    )
    .extend(RATGDO_CLIENT_SCHMEA),
    validate_unique_type,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await sensor.register_sensor(var, config)
    await cg.register_component(var, config)
    cg.add(var.set_ratgdo_sensor_type(config[CONF_TYPE]))
    await register_ratgdo_child(var, config)

    if config["type"] == "distance":
        cg.add_library(name="Wire", version=None)
        cg.add_library(
            name="vl53l4cx",
            repository="https://github.com/stm32duino/VL53L4CX",
            version=None,
        )
        cg.add_define("RATGDO_USE_DISTANCE_SENSOR")
        subscribe_distance()
    if config["type"] == "encoder":
        cg.add_define("RATGDO_USE_ENCODER")
