import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import lock
from esphome.const import CONF_ID

from .. import RATGDO_CLIENT_SCHMEA, ratgdo_ns, register_ratgdo_child

DEPENDENCIES = ["ratgdo"]

# Track if lock has been used
USED_LOCKS: set[str] = set()

RATGDOLock = ratgdo_ns.class_("RATGDOLock", lock.Lock, cg.Component)


def validate_single_lock(config):
    """Validate that only one RATGDO lock is configured."""
    lock_id = "ratgdo_lock"
    if lock_id in USED_LOCKS:
        raise cv.Invalid("Only one RATGDO lock is allowed")
    USED_LOCKS.add(lock_id)
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
