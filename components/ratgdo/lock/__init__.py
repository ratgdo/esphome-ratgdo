import esphome.codegen as cg
from esphome.components import lock
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.types import ConfigType

from .. import RATGDO_CLIENT_SCHMEA, ratgdo_ns, register_ratgdo_child, validate_unique

DEPENDENCIES = ["ratgdo"]

RATGDOLock = ratgdo_ns.class_("RATGDOLock", lock.Lock, cg.Component)


def validate_single_lock(config: ConfigType) -> ConfigType:
    """Validate that only one RATGDO lock is configured."""
    validate_unique("lock", "ratgdo_lock", "Only one RATGDO lock is allowed")
    return config


CONFIG_SCHEMA = cv.All(
    lock.lock_schema(RATGDOLock)
    .extend(
        {
            cv.GenerateID(): cv.declare_id(RATGDOLock),
        }
    )
    .extend(RATGDO_CLIENT_SCHMEA),
    validate_single_lock,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await lock.register_lock(var, config)
    await cg.register_component(var, config)
    await register_ratgdo_child(var, config)
