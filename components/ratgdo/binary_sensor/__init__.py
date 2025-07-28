import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_ID

from .. import RATGDO_CLIENT_SCHMEA, ratgdo_ns, register_ratgdo_child

DEPENDENCIES = ["ratgdo"]

# Track which sensor types have been used
USED_TYPES: set[str] = set()

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
    "vehicle_detected": SensorType.RATGDO_SENSOR_VEHICLE_DETECTED,
    "vehicle_arriving": SensorType.RATGDO_SENSOR_VEHICLE_ARRIVING,
    "vehicle_leaving": SensorType.RATGDO_SENSOR_VEHICLE_LEAVING,
}

# Sensor types that require vehicle sensor support
VEHICLE_SENSOR_TYPES = {"vehicle_detected", "vehicle_arriving", "vehicle_leaving"}


def validate_unique_type(config):
    """Validate that each sensor type is only used once."""
    sensor_type = config[CONF_TYPE]
    if sensor_type in USED_TYPES:
        raise cv.Invalid(f"Only one binary sensor of type '{sensor_type}' is allowed")
    USED_TYPES.add(sensor_type)
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


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await binary_sensor.register_binary_sensor(var, config)
    await cg.register_component(var, config)
    cg.add(var.set_binary_sensor_type(config[CONF_TYPE]))
    await register_ratgdo_child(var, config)

    # Add defines for enabled features
    if config[CONF_TYPE] in VEHICLE_SENSOR_TYPES:
        cg.add_define("RATGDO_USE_VEHICLE_SENSORS")
