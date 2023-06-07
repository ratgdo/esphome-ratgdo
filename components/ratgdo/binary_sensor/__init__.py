import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from .. import (
    ratgdo_ns,
    register_ratgdo_child,
)

DEPENDENCIES = ["ratgdo"]

RATGDOBinarySensor = ratgdo_ns.class_(
    "RATGDOBinarySensor", binary_sensor.BinarySensor, cg.Component
)

TYPES = {"motion", "obstruction"}


CONFIG_SCHEMA = binary_sensor.BINARY_SENSOR_SCHEMA.extend(
    {cv.Optional(type): binary_sensor.binary_sensor_schema() for type in TYPES}
)


async def to_code(config):
    for type in TYPES:
        if type in config:
            conf = config[type]
            var = await binary_sensor.new_binary_sensor(conf)
            await register_ratgdo_child(var, config)
