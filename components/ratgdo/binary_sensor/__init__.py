import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.const import CONF_ID
import esphome.final_validate as fv
from esphome.types import ConfigType

from .. import (
    CONF_ENCODER_PIN_A,
    CONF_ENCODER_SENSOR,
    CONF_RATGDO_ID,
    RATGDO_CLIENT_SCHMEA,
    ratgdo_ns,
    register_ratgdo_child,
    subscribe_manually_operated,
    subscribe_vehicle_arriving,
    subscribe_vehicle_detected,
    subscribe_vehicle_leaving,
    validate_unique,
)

DEPENDENCIES = ["ratgdo"]

RATGDOBinarySensor = ratgdo_ns.class_(
    "RATGDOBinarySensor", binary_sensor.BinarySensor, cg.Component
)
SensorType = ratgdo_ns.enum("SensorType")

CONF_TYPE = "type"
TYPES = {
    "motion": SensorType.RATGDO_SENSOR_MOTION,
    "obstruction": SensorType.RATGDO_SENSOR_OBSTRUCTION,
    "motor": SensorType.RATGDO_SENSOR_MOTOR,
    "button": SensorType.RATGDO_SENSOR_BUTTON,
    "manually_operated": SensorType.RATGDO_SENSOR_MANUALLY_OPERATED,
    "vehicle_detected": SensorType.RATGDO_SENSOR_VEHICLE_DETECTED,
    "vehicle_arriving": SensorType.RATGDO_SENSOR_VEHICLE_ARRIVING,
    "vehicle_leaving": SensorType.RATGDO_SENSOR_VEHICLE_LEAVING,
}

# Sensor types that require vehicle sensor support
VEHICLE_SENSOR_TYPES = {"vehicle_detected", "vehicle_arriving", "vehicle_leaving"}


def validate_unique_type(config: ConfigType) -> ConfigType:
    """Validate that each sensor type is only used once."""
    sensor_type = config[CONF_TYPE]
    validate_unique(
        "binary_sensor",
        sensor_type,
        f"Only one binary sensor of type '{sensor_type}' is allowed",
    )
    return config


CONFIG_SCHEMA = cv.All(
    binary_sensor.binary_sensor_schema(RATGDOBinarySensor)
    .extend(
        {
            cv.Required(CONF_TYPE): cv.enum(TYPES, lower=True),
        }
    )
    .extend(RATGDO_CLIENT_SCHMEA),
    validate_unique_type,
)


def final_validate(config):
    if config[CONF_TYPE] == "manually_operated":
        ratgdo_id = config.get(CONF_RATGDO_ID)
        try:
            full_cfg = fv.full_config.get()
            ratgdo_configs = full_cfg.get("ratgdo", [])
        except LookupError:
            raise cv.Invalid(
                "Internal error: full_config contextvar is unavailable. "
                "Cannot validate manually_operated sensor without access to the full configuration."
            )

        if isinstance(ratgdo_configs, dict):
            ratgdo_configs = [ratgdo_configs]

        has_encoder = False
        for cfg in ratgdo_configs:
            if cfg.get(CONF_ID) == ratgdo_id:
                has_encoder = CONF_ENCODER_SENSOR in cfg or CONF_ENCODER_PIN_A in cfg
                break

        if not has_encoder:
            raise cv.Invalid(
                "The manually_operated binary sensor requires an encoder to be configured "
                "on the main ratgdo component."
            )
    return config


FINAL_VALIDATE_SCHEMA = cv.All(final_validate)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await binary_sensor.register_binary_sensor(var, config)
    await cg.register_component(var, config)
    cg.add(var.set_binary_sensor_type(config[CONF_TYPE]))
    await register_ratgdo_child(var, config)

    # Add defines for enabled features and register observable subscriptions
    sensor_type = config[CONF_TYPE]
    if sensor_type in VEHICLE_SENSOR_TYPES:
        cg.add_define("RATGDO_USE_VEHICLE_SENSORS")
    if sensor_type == "vehicle_detected":
        subscribe_vehicle_detected()
    elif sensor_type == "vehicle_arriving":
        subscribe_vehicle_arriving()
    elif sensor_type == "vehicle_leaving":
        subscribe_vehicle_leaving()
    elif sensor_type == "manually_operated":
        subscribe_manually_operated()
