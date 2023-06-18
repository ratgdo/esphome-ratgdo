import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button
from esphome.const import CONF_ID

from .. import RATGDO_CLIENT_SCHMEA, ratgdo_ns, register_ratgdo_child

DEPENDENCIES = ["ratgdo"]

RATGDOButton = ratgdo_ns.class_("RATGDOButton", button.Button, cg.Component)
ButtonType = ratgdo_ns.enum("ButtonType")

CONF_TYPE = "type"
TYPES = {
    "sync": ButtonType.RATGDO_SYNC,
    "query": ButtonType.RATGDO_QUERY,
    "close_beep": ButtonType.RATGDO_CLOSE_BEEP,
}


CONFIG_SCHEMA = (
    button.button_schema(RATGDOButton)
    .extend(
        {
            cv.Required(CONF_TYPE): cv.enum(TYPES, lower=True),
        }
    )
    .extend(RATGDO_CLIENT_SCHMEA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await button.register_button(var, config)
    await cg.register_component(var, config)
    cg.add(var.set_button_type(config[CONF_TYPE]))
    await register_ratgdo_child(var, config)
