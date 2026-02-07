import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import light
from esphome.const import CONF_OUTPUT_ID  # New in 2023.5

from .. import RATGDO_CLIENT_SCHMEA, ratgdo_ns, register_ratgdo_child

DEPENDENCIES = ["ratgdo"]

# Track if light has been used
USED_LIGHTS: set[str] = set()

RATGDOLightOutput = ratgdo_ns.class_(
    "RATGDOLightOutput", light.LightOutput, cg.Component
)


def validate_single_light(config):
    """Validate that only one RATGDO light is configured."""
    light_id = "ratgdo_light"
    if light_id in USED_LIGHTS:
        raise cv.Invalid("Only one RATGDO light is allowed")
    USED_LIGHTS.add(light_id)
    return config


CONFIG_SCHEMA = cv.All(
    light.LIGHT_SCHEMA.extend(
        {cv.GenerateID(CONF_OUTPUT_ID): cv.declare_id(RATGDOLightOutput)}
    ).extend(RATGDO_CLIENT_SCHMEA),
    validate_single_light,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_OUTPUT_ID])
    await cg.register_component(var, config)
    await light.register_light(var, config)
    await register_ratgdo_child(var, config)
