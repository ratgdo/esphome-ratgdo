import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome import pins, automation

DEPENDENCIES = ["preferences"]
MULTI_CONF = True


ratgdo_ns = cg.esphome_ns.namespace("ratgdo")
RATGDO = ratgdo_ns.class_("RATGDOComponent", cg.Component)


CONF_OUTPUT_GDO = "output_gdo_pin"
DEFAULT_OUTPUT_GDO = (
    2  # D4 ed control terminal / GarageDoorOpener (UART1 TX) pin is D4 on D1 Mini
)
CONF_INPUT_GDO = "input_gdo_pin"
DEFAULT_INPUT_GDO = (
    4  # D2 red control terminal / GarageDoorOpener (UART1 RX) pin is D2 on D1 Mini
)
CONF_INPUT_OBST = "input_obst_pin"
DEFAULT_INPUT_OBST = 13  # D7 black obstruction sensor terminal

CONF_TRIGGER_OPEN = "trigger_open_pin"
DEFAULT_TRIGGER_OPEN = 14  # D5 dry contact for opening door
CONF_TRIGGER_CLOSE = "trigger_close_pin"
DEFAULT_TRIGGER_CLOSE = 12  # D6 dry contact for closing door
CONF_TRIGGER_LIGHT = "trigger_light_pin"
DEFAULT_TRIGGER_LIGHT = 0  # D3 dry contact for triggering light (no discrete light commands, so toggle only)

CONF_STATUS_DOOR = "status_door_pin"
DEFAULT_STATUS_DOOR = 16  # D0 output door status, HIGH for open, LOW for closed
CONF_STATUS_OBST = "status_obst_pin"
DEFAULT_STATUS_OBST = (
    15  # D8 output for obstruction status, HIGH for obstructed, LOW for clear
)
CONF_RATGDO_ID = "ratgdo_id"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(RATGDO),
            cv.Optional(
                CONF_OUTPUT_GDO, default=DEFAULT_OUTPUT_GDO
            ): pins.internal_gpio_input_pin_schema,
            cv.Optional(
                CONF_INPUT_GDO, default=DEFAULT_INPUT_GDO
            ): pins.internal_gpio_input_pin_schema,
            cv.Optional(
                CONF_INPUT_OBST, default=DEFAULT_INPUT_OBST
            ): pins.internal_gpio_input_pin_schema,
            cv.Optional(
                CONF_TRIGGER_OPEN, default=DEFAULT_TRIGGER_OPEN
            ): pins.internal_gpio_input_pin_schema,
            cv.Optional(
                CONF_TRIGGER_CLOSE, default=DEFAULT_TRIGGER_CLOSE
            ): pins.internal_gpio_input_pin_schema,
            cv.Optional(
                CONF_TRIGGER_LIGHT, default=DEFAULT_TRIGGER_LIGHT
            ): pins.internal_gpio_input_pin_schema,
            cv.Optional(
                CONF_STATUS_DOOR, default=DEFAULT_STATUS_DOOR
            ): pins.internal_gpio_input_pin_schema,
            cv.Optional(
                CONF_STATUS_OBST, default=DEFAULT_STATUS_OBST
            ): pins.internal_gpio_input_pin_schema,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)

RATGDO_CLIENT_SCHMEA = cv.Schema(
    {
        cv.Required(CONF_RATGDO_ID): cv.use_id(RATGDO),
    }
)

async def register_ratgdo_child(var, config):
    parent = await cg.get_variable(config[CONF_RATGDO_ID])
    cg.add(parent.register_child(var))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    pin = await cg.gpio_pin_expression(config[CONF_OUTPUT_GDO])
    cg.add(var.set_output_gdo_pin(pin))
    pin = await cg.gpio_pin_expression(config[CONF_INPUT_GDO])
    cg.add(var.set_input_gdo_pin(pin))
    pin = await cg.gpio_pin_expression(config[CONF_INPUT_OBST])
    cg.add(var.set_input_obst_pin(pin))

    pin = await cg.gpio_pin_expression(config[CONF_TRIGGER_OPEN])
    cg.add(var.set_trigger_open_pin(pin))
    pin = await cg.gpio_pin_expression(config[CONF_TRIGGER_CLOSE])
    cg.add(var.set_trigger_close_pin(pin))
    pin = await cg.gpio_pin_expression(config[CONF_TRIGGER_LIGHT])
    cg.add(var.set_trigger_light_pin(pin))

    pin = await cg.gpio_pin_expression(config[CONF_STATUS_DOOR])
    cg.add(var.set_status_door_pin(pin))
    pin = await cg.gpio_pin_expression(config[CONF_STATUS_OBST])
    cg.add(var.set_status_obst_pin(pin))

    cg.add_library(
        name="secplus",
        repository="https://github.com/bdraco/secplus",
        version="f98c3220356c27717a25102c0b35815ebbd26ccc",
    )
    cg.add_library(
        name="espsoftwareserial",
        repository="https://github.com/bdraco/espsoftwareserial",
        version="2f408224633316b997f82339e5b2731b1e561060",
    )