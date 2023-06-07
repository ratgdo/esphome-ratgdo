import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.components import number
from .. import (
    ratgdo_ns,
    register_ratgdo_child,
    RATGDO_CLIENT_SCHMEA
)

DEPENDENCIES = ["ratgdo"]

RATGDONumber = ratgdo_ns.class_(
    "RATGDONumber", number.Number, cg.Component
)
NumberType = ratgdo_ns.enum("NumberType")

CONF_TYPE = "type"
TYPES = {
    "rolling_code_counter": NumberType.RATGDO_ROLLING_CODE_COUNTER,
}


CONFIG_SCHEMA = number.number_schema(RATGDONumber).extend(
    {
        cv.Required(CONF_TYPE): cv.enum(TYPES, lower=True),
    }
).extend(RATGDO_CLIENT_SCHMEA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await number.register_number(var, config, step=1)
    await cg.register_component(var, config)
    cg.add(var.set_number_type(config[CONF_TYPE]))
    await register_ratgdo_child(var, config)
