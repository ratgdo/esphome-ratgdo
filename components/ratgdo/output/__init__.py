import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import rtttl
from esphome.const import CONF_ID

CONF_RTTTL = "rtttl"
CONF_SONG = "song"


from .. import RATGDO_CLIENT_SCHMEA, ratgdo_ns, register_ratgdo_child

DEPENDENCIES = ["esp32", "ratgdo", "rtttl"]

RATGDOOutput = ratgdo_ns.class_("RATGDOOutput", cg.Component)
OutputType = ratgdo_ns.enum("OutputType")

CONF_TYPE = "type"
TYPES = {"beeper": OutputType.RATGDO_BEEPER}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ID): cv.declare_id(RATGDOOutput),
        cv.Required(CONF_TYPE): cv.enum(TYPES, lower=True),
        cv.Required(CONF_RTTTL): cv.use_id(rtttl),
        cv.Required(CONF_SONG): cv.string,
    }
).extend(RATGDO_CLIENT_SCHMEA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    rtttl = await cg.get_variable(config[CONF_RTTTL])
    cg.add(var.set_rtttl(rtttl))
    cg.add(var.set_song(config[CONF_SONG]))
    await register_ratgdo_child(var, config)
