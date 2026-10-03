import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.types import ConfigType

from .. import (
    RATGDO_CLIENT_SCHMEA,
    ratgdo_ns,
    register_ratgdo_child,
    subscribe_ttc_state,
    validate_unique,
)

DEPENDENCIES = ["ratgdo"]

RATGDOTextSensor = ratgdo_ns.class_(
    "RATGDOTextSensor", text_sensor.TextSensor, cg.Component
)
RATGDOTextSensorType = ratgdo_ns.enum("RATGDOTextSensorType")

CONF_TYPE = "type"
TYPES = {
    "ttc_state": RATGDOTextSensorType.RATGDO_TTC_STATE,  # only meaningful with protocol: secplusv2
}


def validate_unique_type(config: ConfigType) -> ConfigType:
    """Validate that each text_sensor type is only used once."""
    sensor_type = config[CONF_TYPE]
    validate_unique(
        "text_sensor",
        sensor_type,
        f"Only one text_sensor of type '{sensor_type}' is allowed",
    )
    return config


CONFIG_SCHEMA = cv.All(
    text_sensor.text_sensor_schema(RATGDOTextSensor)
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
    await text_sensor.register_text_sensor(var, config)
    await cg.register_component(var, config)
    cg.add(var.set_ratgdo_text_sensor_type(config[CONF_TYPE]))
    await register_ratgdo_child(var, config)

    if config[CONF_TYPE] == "ttc_state":
        subscribe_ttc_state()
