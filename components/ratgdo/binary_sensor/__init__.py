import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.components import binary_sensor
from .. import (
    ratgdo_ns,
    register_ratgdo_child,
    RATGDO_CLIENT_SCHMEA
)

DEPENDENCIES = ["ratgdo"]

RATGDOBinarySensor = ratgdo_ns.class_(
    "RATGDOBinarySensor", binary_sensor.BinarySensor, cg.Component
)

CONF_TYPE = "type"
TYPES = {"motion", "obstruction"}


CONFIG_SCHEMA = binary_sensor.binary_sensor_schema(RATGDOBinarySensor).extend(
    {
        cv.Required(CONF_TYPE): str
    }
).extend(RATGDO_CLIENT_SCHMEA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await binary_sensor.register_binary_sensor(var, config)
    await cg.register_component(var, config)
    await register_ratgdo_child(var, config)
