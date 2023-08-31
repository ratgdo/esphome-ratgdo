import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select
from esphome.const import CONF_ID

from .. import RATGDO_CLIENT_SCHMEA, ratgdo_ns, register_ratgdo_child

DEPENDENCIES = ["ratgdo"]

RATGDOSelect = ratgdo_ns.class_("RATGDOSelect", select.Select, cg.Component)
RATGDOSelectType = ratgdo_ns.enum("RATGDOSelectType")

CONF_TYPE = "type"
TYPES = {
    "ttc": RATGDOSelectType.RATGDO_TTC,
}


CONFIG_SCHEMA = (
    select.select_schema(RATGDOSelect)
    .extend(
        {
            cv.Required(CONF_TYPE): cv.enum(TYPES, lower=True),
        }
    )
    .extend(RATGDO_CLIENT_SCHMEA)
)


async def to_code(config):
    #var = cg.new_Pvariable(config[CONF_ID])
    var = await select.new_select(config, options=["Off","1 Minute","5 Minutes","10 Minutes"])
    await cg.register_component(var, config)
    cg.add(var.set_ratgdo_select_type(config[CONF_TYPE]))
    await register_ratgdo_child(var, config)
