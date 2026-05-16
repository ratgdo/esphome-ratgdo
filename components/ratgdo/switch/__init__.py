from esphome import pins
import esphome.codegen as cg
from esphome.components import switch
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_PIN

from .. import (
    RATGDO_CLIENT_SCHMEA,
    ratgdo_ns,
    register_ratgdo_child,
    subscribe_vehicle_arriving,
)

DEPENDENCIES = ["ratgdo"]

RATGDOSwitch = ratgdo_ns.class_("RATGDOSwitch", switch.Switch, cg.Component)
SwitchType = ratgdo_ns.enum("SwitchType")

CONF_TYPE = "type"
CONF_VEHICLE_AUTO_CONTROL = "vehicle_auto_control"
TYPES = {"learn": SwitchType.RATGDO_LEARN, "led": SwitchType.RATGDO_LED}


CONFIG_SCHEMA = (
    switch.switch_schema(RATGDOSwitch)
    .extend(
        {
            cv.Required(CONF_TYPE): cv.enum(TYPES, lower=True),
            cv.Optional(CONF_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_VEHICLE_AUTO_CONTROL, default=True): cv.boolean,
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
    # LED switches optionally follow vehicle_arriving (parking-assist auto on/off).
    # Only register the subscription when the user has not opted out, so the
    # observable's compile-time sizing matches the actual subscriber count.
    if config[CONF_TYPE] == "led":
        auto_control = config[CONF_VEHICLE_AUTO_CONTROL]
        cg.add(var.set_vehicle_auto_control(auto_control))
        if auto_control:
            subscribe_vehicle_arriving()
