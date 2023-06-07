import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import (
    CONF_OUTPUT_ID,  # New in 2023.5
)
from esphome.components import light
from .. import ratgdo_ns, register_ratgdo_child, RATGDO_CLIENT_SCHMEA

DEPENDENCIES = ["ratgdo"]

RATGDOLightOutput = ratgdo_ns.class_(
    "RATGDOLightOutput", light.LightOutput, cg.Component
)


CONFIG_SCHEMA = light.LIGHT_SCHEMA.extend(
    {cv.GenerateID(CONF_OUTPUT_ID): cv.declare_id(RATGDOLightOutput)}
).extend(RATGDO_CLIENT_SCHMEA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_OUTPUT_ID])
    await cg.register_component(var, config)
    await light.register_light(var, config)
    await register_ratgdo_child(var, config)
