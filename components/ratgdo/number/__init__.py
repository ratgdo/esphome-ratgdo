import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import CONF_ID

from .. import RATGDO_CLIENT_SCHMEA, ratgdo_ns, register_ratgdo_child

DEPENDENCIES = ["ratgdo"]

RATGDONumber = ratgdo_ns.class_("RATGDONumber", number.Number, cg.Component)
NumberType = ratgdo_ns.enum("NumberType")

CONF_TYPE = "type"
TYPES = {
    "client_id": NumberType.RATGDO_CLIENT_ID,
    "rolling_code_counter": NumberType.RATGDO_ROLLING_CODE_COUNTER,
    "opening_duration": NumberType.RATGDO_OPENING_DURATION,
    "closing_duration": NumberType.RATGDO_CLOSING_DURATION,
}


CONFIG_SCHEMA = (
    number.number_schema(RATGDONumber)
    .extend(
        {
            cv.Required(CONF_TYPE): cv.enum(TYPES, lower=True),
        }
    )
    .extend(RATGDO_CLIENT_SCHMEA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await number.register_number(var, config, step=1, min_value=0, max_value=4294967295)
    await cg.register_component(var, config)
    cg.add(var.set_number_type(config[CONF_TYPE]))
    await register_ratgdo_child(var, config)
