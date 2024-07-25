import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import output
from esphome.const import CONF_ID, CONF_PIN

from .. import RATGDO_CLIENT_SCHMEA, ratgdo_ns, register_ratgdo_child

DEPENDENCIES = ["esp32","ratgdo"]

RATGDOFloatOutput = ratgdo_ns.class_("RATGDOFloatOutput", output.FloatOutput, cg.Component)
OutputType = ratgdo_ns.enum("OutputType")

CONF_TYPE = "type"
TYPES = {
    "test": OutputType.RATGDO_TEST
}

CONFIG_SCHEMA = (
    output.FLOAT_OUTPUT_SCHEMA.extend(
        {
      cv.Required(CONF_ID): cv.declare_id(RATGDOFloatOutput),
      cv.Required(CONF_TYPE): cv.enum(TYPES, lower=True),
      cv.Required(CONF_PIN): pins.internal_gpio_output_pin_schema,
    }
  )
    .extend(RATGDO_CLIENT_SCHMEA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await output.register_output(var, config)
    await cg.register_component(var, config)