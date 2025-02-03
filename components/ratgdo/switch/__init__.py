import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import switch
from esphome import pins
from esphome.const import CONF_ID, CONF_PIN

from .. import RATGDO_CLIENT_SCHMEA, ratgdo_ns, register_ratgdo_child

DEPENDENCIES = ["ratgdo"]

RATGDOSwitch = ratgdo_ns.class_("RATGDOSwitch", switch.Switch, cg.Component)
SwitchType = ratgdo_ns.enum("SwitchType")

CONF_TYPE = "type"
TYPES = {
    "learn": SwitchType.RATGDO_LEARN,
    "led": SwitchType.RATGDO_LED
}


CONFIG_SCHEMA = (
    switch.switch_schema(RATGDOSwitch)
    .extend(
        {
            cv.Required(CONF_TYPE): cv.enum(TYPES, lower=True),
            cv.Optional(CONF_PIN): pins.gpio_output_pin_schema,
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
    if CONF_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_PIN])
        cg.add(var.set_pin(pin))
