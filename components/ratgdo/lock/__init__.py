import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import lock
from esphome.const import CONF_ID

from .. import RATGDO_CLIENT_SCHMEA, ratgdo_ns, register_ratgdo_child

DEPENDENCIES = ["ratgdo"]

RATGDOLock = ratgdo_ns.class_("RATGDOLock", lock.Lock, cg.Component)

CONFIG_SCHEMA = (
    lock.LOCK_SCHEMA
    .extend(
        {
            cv.GenerateID(): cv.declare_id(RATGDOLock),
        }
    )
    .extend(RATGDO_CLIENT_SCHMEA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await lock.register_lock(var, config)
    await cg.register_component(var, config)
    await register_ratgdo_child(var, config)
