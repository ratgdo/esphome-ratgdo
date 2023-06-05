import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

DEPENDENCIES = ["preferences"]


ratgdo_ns = cg.esphome_ns.namespace("ratgdo")
RATGDO = ratgdo_ns.class_("RATGDO", cg.Component)

CONF_ROLLING_CODES = "rolling_codes"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(RATGDO),
        cv.Optional(CONF_ROLLING_CODES, default=True): cv.boolean,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    # rolling_codes = await cg.get_variable(config[CONF_ROLLING_CODES])
    # cg.add(var.set_microphone(mic))
    cg.add_library(
        name="secplus",
        repository="https://github.com/bdraco/secplus",
        version="f98c3220356c27717a25102c0b35815ebbd26ccc",
    )
    cg.add_library(
        name="espsoftwareserial",
        repository="https://github.com/bdraco/espsoftwareserial",
        version="2f408224633316b997f82339e5b2731b1e561060",
    )