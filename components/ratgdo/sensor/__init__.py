import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID

from .. import RATGDO_CLIENT_SCHMEA, ratgdo_ns, register_ratgdo_child

DEPENDENCIES = ["ratgdo"]

RATGDOSensor = ratgdo_ns.class_("RATGDOSensor", sensor.Sensor, cg.Component)
RATGDOSensorType = ratgdo_ns.enum("RATGDOSensorType")

CONF_TYPE = "type"
TYPES = {
    "openings": RATGDOSensorType.RATGDO_OPENINGS,
    "auto_close_time": RATGDOSensorType.RATGDO_AUTO_CLOSE_TIME,
}


CONFIG_SCHEMA = (
    sensor.sensor_schema(RATGDOSensor)
    .extend(
        {
            cv.Required(CONF_TYPE): cv.enum(TYPES, lower=True),
        }
    )
    .extend(RATGDO_CLIENT_SCHMEA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await sensor.register_sensor(var, config)
    await cg.register_component(var, config)
    cg.add(var.set_ratgdo_sensor_type(config[CONF_TYPE]))
    await register_ratgdo_child(var, config)
