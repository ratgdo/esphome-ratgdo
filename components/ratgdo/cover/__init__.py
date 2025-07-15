import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import cover
from esphome.const import CONF_ID, CONF_TRIGGER_ID

from .. import RATGDO_CLIENT_SCHMEA, ratgdo_ns, register_ratgdo_child

DEPENDENCIES = ["ratgdo"]

RATGDOCover = ratgdo_ns.class_("RATGDOCover", cover.Cover, cg.Component)


# Triggers
CoverOpeningTrigger = ratgdo_ns.class_(
    "CoverOpeningTrigger", automation.Trigger.template()
)
CoverClosingTrigger = ratgdo_ns.class_(
    "CoverClosingTrigger", automation.Trigger.template()
)
CoverStateTrigger = ratgdo_ns.class_("CoverStateTrigger", automation.Trigger.template())

CONF_ON_OPENING = "on_opening"
CONF_ON_CLOSING = "on_closing"
CONF_ON_STATE_CHANGE = "on_state_change"

CONFIG_SCHEMA = (
    cover.cover_schema(RATGDOCover)
    .extend(
        {
            cv.GenerateID(): cv.declare_id(RATGDOCover),
            cv.Optional(CONF_ON_OPENING): automation.validate_automation(
                {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(CoverOpeningTrigger)}
            ),
            cv.Optional(CONF_ON_CLOSING): automation.validate_automation(
                {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(CoverClosingTrigger)}
            ),
            cv.Optional(CONF_ON_STATE_CHANGE): automation.validate_automation(
                {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(CoverStateTrigger)}
            ),
        }
    )
    .extend(RATGDO_CLIENT_SCHMEA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cover.register_cover(var, config)

    for conf in config.get(CONF_ON_OPENING, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)
    for conf in config.get(CONF_ON_CLOSING, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)
    for conf in config.get(CONF_ON_STATE_CHANGE, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)

    await register_ratgdo_child(var, config)
