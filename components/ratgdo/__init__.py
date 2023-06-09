import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import uart
from esphome.const import CONF_ID

DEPENDENCIES = ["preferences", "uart"]
MULTI_CONF = True


ratgdo_ns = cg.esphome_ns.namespace("ratgdo")
RATGDO = ratgdo_ns.class_("RATGDOComponent", cg.Component, uart.UARTDevice)


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

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(RATGDO),
            cv.Optional(
                CONF_OUTPUT_GDO, default=DEFAULT_OUTPUT_GDO
            ): pins.gpio_output_pin_schema,
            cv.Optional(
                CONF_INPUT_GDO, default=DEFAULT_INPUT_GDO
            ): pins.gpio_input_pin_schema,
            cv.Optional(
                CONF_INPUT_OBST, default=DEFAULT_INPUT_OBST
            ): pins.gpio_input_pin_schema,
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

    await uart.register_uart_device(var, config)

    cg.add_library(
        name="secplus",
        repository="https://github.com/esphome-ratgdo/secplus",
        version="f98c3220356c27717a25102c0b35815ebbd26ccc",
    )
