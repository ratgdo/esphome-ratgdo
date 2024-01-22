import esphome.codegen as cg
import esphome.config_validation as cv
import voluptuous as vol
from esphome import automation, pins
from esphome.const import CONF_ID, CONF_TRIGGER_ID
from esphome.components import binary_sensor

DEPENDENCIES = ["preferences"]
MULTI_CONF = True


ratgdo_ns = cg.esphome_ns.namespace("ratgdo")
RATGDO = ratgdo_ns.class_("RATGDOComponent", cg.Component)


SyncFailed = ratgdo_ns.class_("SyncFailed", automation.Trigger.template())

CONF_OUTPUT_GDO = "output_gdo_pin"
DEFAULT_OUTPUT_GDO = (
    "D4"  # D4 red control terminal / GarageDoorOpener (UART1 TX) pin is D4 on D1 Mini
)
CONF_INPUT_GDO = "input_gdo_pin"
DEFAULT_INPUT_GDO = (
    "D2"  # D2 red control terminal / GarageDoorOpener (UART1 RX) pin is D2 on D1 Mini
)
CONF_INPUT_OBST = "input_obst_pin"
DEFAULT_INPUT_OBST = "D7"  # D7 black obstruction sensor terminal

CONF_RATGDO_ID = "ratgdo_id"

CONF_ON_SYNC_FAILED = "on_sync_failed"

CONF_PROTOCOL = "protocol"

PROTOCOL_SECPLUSV1 = "secplusv1"
PROTOCOL_SECPLUSV2 = "secplusv2"
PROTOCOL_DRYCONTACT = "drycontact"
SUPPORTED_PROTOCOLS = [PROTOCOL_SECPLUSV1, PROTOCOL_SECPLUSV2, PROTOCOL_DRYCONTACT]

DOOR_CLOSED = "door_closed"
DOOR_OPEN = "door_open"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(RATGDO),
        cv.Optional(
            CONF_OUTPUT_GDO, default=DEFAULT_OUTPUT_GDO
        ): pins.gpio_output_pin_schema,
        cv.Optional(
            CONF_INPUT_GDO, default=DEFAULT_INPUT_GDO
        ): pins.gpio_input_pin_schema,
        cv.Optional(CONF_INPUT_OBST, default=DEFAULT_INPUT_OBST): cv.Any(
            cv.none, pins.gpio_input_pin_schema
        ),
        cv.Optional(CONF_ON_SYNC_FAILED): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(SyncFailed),
            }
        ),
        cv.Optional(CONF_PROTOCOL, default=PROTOCOL_SECPLUSV2): vol.In(
            SUPPORTED_PROTOCOLS
        ),
        cv.Optional(DOOR_CLOSED): cv.Any(
            cv.none, cv.use_id(binary_sensor.BinarySensor)
        ),
        cv.Optional(DOOR_OPEN): cv.Any(
            cv.none, cv.use_id(binary_sensor.BinarySensor)
        ),
    }
).extend(cv.COMPONENT_SCHEMA)

RATGDO_CLIENT_SCHMEA = cv.Schema(
    {
        cv.Required(CONF_RATGDO_ID): cv.use_id(RATGDO),
    }
)


async def register_ratgdo_child(var, config):
    parent = await cg.get_variable(config[CONF_RATGDO_ID])
    cg.add(var.set_parent(parent))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    pin = await cg.gpio_pin_expression(config[CONF_OUTPUT_GDO])
    cg.add(var.set_output_gdo_pin(pin))
    pin = await cg.gpio_pin_expression(config[CONF_INPUT_GDO])
    cg.add(var.set_input_gdo_pin(pin))
    if CONF_INPUT_OBST in config and config[CONF_INPUT_OBST]:
        pin = await cg.gpio_pin_expression(config[CONF_INPUT_OBST])
        cg.add(var.set_input_obst_pin(pin))

    for conf in config.get(CONF_ON_SYNC_FAILED, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)

    cg.add_library(
        name="secplus",
        repository="https://github.com/ratgdo/secplus#f98c3220356c27717a25102c0b35815ebbd26ccc",
        version=None,
    )
    cg.add_library(
        name="espsoftwareserial",
        repository="https://github.com/ratgdo/espsoftwareserial#autobaud",
        version=None,
    )

    if config[CONF_PROTOCOL] == PROTOCOL_SECPLUSV1:
        cg.add_define("PROTOCOL_SECPLUSV1")
    elif config[CONF_PROTOCOL] == PROTOCOL_SECPLUSV2:
        cg.add_define("PROTOCOL_SECPLUSV2")
    elif config[CONF_PROTOCOL] == PROTOCOL_DRYCONTACT:
        cg.add_define("PROTOCOL_DRYCONTACT")
    cg.add(var.init_protocol())
    if DOOR_CLOSED in config and config[DOOR_CLOSED] is not None:
        door_closed = await cg.get_variable(config[DOOR_CLOSED])
        cg.add(var.set_door_closed_sensor(door_closed))
    if DOOR_OPEN in config and config[DOOR_OPEN] is not None:
        door_open = await cg.get_variable(config[DOOR_OPEN])
        cg.add(var.set_door_open_sensor(door_open))
    
