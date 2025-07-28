import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import CONF_ID

from .. import RATGDO_CLIENT_SCHMEA, ratgdo_ns, register_ratgdo_child

DEPENDENCIES = ["ratgdo"]

# Track which number types have been used
USED_TYPES: set[str] = set()

RATGDONumber = ratgdo_ns.class_("RATGDONumber", number.Number, cg.Component)
NumberType = ratgdo_ns.enum("NumberType")

CONF_TYPE = "type"
TYPES = {
    "client_id": NumberType.RATGDO_CLIENT_ID,
    "rolling_code_counter": NumberType.RATGDO_ROLLING_CODE_COUNTER,
    "opening_duration": NumberType.RATGDO_OPENING_DURATION,
    "closing_duration": NumberType.RATGDO_CLOSING_DURATION,
    "closing_delay": NumberType.RATGDO_CLOSING_DELAY,
    "target_distance_measurement": NumberType.RATGDO_TARGET_DISTANCE_MEASUREMENT,
}


def validate_unique_type(config):
    """Validate that each number type is only used once."""
    number_type = config[CONF_TYPE]
    if number_type in USED_TYPES:
        raise cv.Invalid(f"Only one number of type '{number_type}' is allowed")
    USED_TYPES.add(number_type)
    return config


CONFIG_SCHEMA = cv.All(
    number.number_schema(RATGDONumber)
    .extend(
        {
            cv.Required(CONF_TYPE): cv.enum(TYPES, lower=True),
        }
    )
    .extend(RATGDO_CLIENT_SCHMEA),
    validate_unique_type,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await number.register_number(var, config, step=1, min_value=0, max_value=4294967295)
    await cg.register_component(var, config)
    cg.add(var.set_number_type(config[CONF_TYPE]))
    await register_ratgdo_child(var, config)
