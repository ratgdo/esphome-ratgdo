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

CONF_TYPE = "xtype"
TYPES = {"motion", "obstruction"}


CONFIG_SCHEMA = binary_sensor.BINARY_SENSOR_SCHEMA.extend(
    {cv.Required(CONF_TYPE): str}
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = await binary_sensor.new_binary_sensor(config)
    await cg.register_component(var, config)
    await register_ratgdo_child(var, config)
