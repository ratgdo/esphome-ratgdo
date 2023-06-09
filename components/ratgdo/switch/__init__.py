import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.components import switch
from .. import ratgdo_ns, register_ratgdo_child, RATGDO_CLIENT_SCHMEA

DEPENDENCIES = ["ratgdo"]

RATGDOSwitch = ratgdo_ns.class_("RATGDOSwitch", switch.Switch, cg.Component)
SwitchType = ratgdo_ns.enum("SwitchType")

CONF_TYPE = "type"
TYPES = {
    "lock": SwitchType.RATGDO_LOCK,
}


CONFIG_SCHEMA = (
    switch.switch_schema(RATGDOSwitch)
    .extend(
        {
            cv.Required(CONF_TYPE): cv.enum(TYPES, lower=True),
        }
    )
    .extend(RATGDO_CLIENT_SCHMEA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await switch.register_switch(var, config)
    await cg.register_component(var, config)
    cg.add(var.set_switch_type(config[CONF_TYPE]))
    await register_ratgdo_child(var, config)
