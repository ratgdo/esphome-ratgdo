from esphome import automation
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import mdns, wifi, light, lock, sensor, switch, climate, pn532, fan, cover
from esphome.const import PLATFORM_ESP32, CONF_ID, CONF_TRIGGER_ID
from esphome.core import ID, Lambda
from esphome.components.esp32 import add_idf_component
from .. import homekit_base

AUTO_LOAD = ["homekit_base"]
DEPENDENCIES = ['esp32', 'network', 'homekit_base']
CODEOWNERS = ["@rednblkx"]

homekit_ns = homekit_base.homekit_ns
HAPRootComponent = homekit_base.HAPRootComponent
TemperatureUnits = homekit_ns.enum("TemperatureUnits")
AInfo = homekit_ns.enum("AInfo")
HKFinish = homekit_ns.enum("HKFinish")
HAPAccessory = homekit_ns.class_('HAPAccessory', cg.Component)
LightEntity = homekit_ns.class_('LightEntity')
SensorEntity = homekit_ns.class_('SensorEntity')
SwitchEntity = homekit_ns.class_('SwitchEntity')
LockEntity = homekit_ns.class_('LockEntity')
FanEntity = homekit_ns.class_('FanEntity')
ClimateEntity = homekit_ns.class_('ClimateEntity')
CoverEntity = homekit_ns.class_('CoverEntity')
OnHkSuccessTrigger = homekit_ns.class_(
    "HKAuthTrigger", automation.Trigger.template(cg.std_string, cg.std_string)
)
OnHkFailTrigger = homekit_ns.class_(
    "HKFailTrigger", automation.Trigger.template()
)
CONF_IDENTIFY_LAMBDA = "identify_fn"

TEMP_UNITS = {
    "CELSIUS": TemperatureUnits.CELSIUS,
    "FAHRENHEIT": TemperatureUnits.FAHRENHEIT
}

ACC_INFO = {
    "name": AInfo.NAME,
    "model": AInfo.MODEL,
    "manufacturer": AInfo.MANUFACTURER,
    "serial_number": AInfo.SN,
    "fw_rev": AInfo.FW_REV,
}

HK_HW_FINISH = {
    "TAN": HKFinish.TAN,
    "GOLD": HKFinish.GOLD,
    "SILVER": HKFinish.SILVER,
    "BLACK": HKFinish.BLACK
}

ACCESSORY_INFORMATION = {
    cv.Optional(i): cv.string for i in ACC_INFO
}

CONFIG_SCHEMA = cv.All(cv.Schema({
    cv.GenerateID() : cv.declare_id(HAPAccessory),
    cv.Optional("light"): cv.ensure_list({cv.Required(CONF_ID): cv.use_id(light.LightState), cv.Optional("meta") : ACCESSORY_INFORMATION}),
    cv.Optional("lock"):  cv.ensure_list({cv.Required(CONF_ID): cv.use_id(lock.Lock), cv.Optional("nfc_id") : cv.use_id(pn532.PN532),
        cv.Optional("on_hk_success"): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(OnHkSuccessTrigger),
            }
        ),
        cv.Optional("on_hk_fail"): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(OnHkFailTrigger),
            }
        ),
        cv.Optional("hk_hw_finish", default="BLACK") : cv.enum(HK_HW_FINISH),
        cv.Optional("meta") : ACCESSORY_INFORMATION
        }),
    cv.Optional("sensor"):  cv.ensure_list({cv.Required(CONF_ID): cv.use_id(sensor.Sensor), cv.Optional("temp_units", default="CELSIUS") : cv.enum(TEMP_UNITS), cv.Optional("meta") : ACCESSORY_INFORMATION}),
    cv.Optional("fan"):  cv.ensure_list({cv.Required(CONF_ID): cv.use_id(fan.Fan), cv.Optional("meta") : ACCESSORY_INFORMATION}),
    cv.Optional("switch"):  cv.ensure_list({cv.Required(CONF_ID): cv.use_id(switch.Switch), cv.Optional("meta") : ACCESSORY_INFORMATION}),
    cv.Optional("climate"):  cv.ensure_list({cv.Required(CONF_ID): cv.use_id(climate.Climate), cv.Optional("meta") : ACCESSORY_INFORMATION}),
    cv.Optional("cover"):  cv.ensure_list({cv.Required(CONF_ID): cv.use_id(cover.Cover), cv.Optional("meta") : ACCESSORY_INFORMATION}),
}).extend(cv.COMPONENT_SCHEMA),
cv.only_on([PLATFORM_ESP32]),
cv.only_with_esp_idf)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    if 'light' in config:
        for l in config["light"]:
            light_entity = cg.Pvariable(ID(f"{l['id'].id}_hk_light_entity", type=LightEntity), var.add_light(await cg.get_variable(l['id'])))
            if "meta" in l:
                info_temp = []
                for m in l["meta"]:
                    info_temp.append([ACC_INFO[m], l["meta"][m]])
                cg.add(light_entity.setInfo(info_temp))
    if 'sensor' in config:
        for l in config["sensor"]:
            sensor_entity = cg.Pvariable(ID(f"{l['id'].id}_hk_sensor_entity", type=SensorEntity), var.add_sensor(await cg.get_variable(l['id']), l['temp_units']))
            if "meta" in l:
                info_temp = []
                for m in l["meta"]:
                    info_temp.append([ACC_INFO[m], l["meta"][m]])
                cg.add(sensor_entity.setInfo(info_temp))
    if 'lock' in config:
        for l in config["lock"]:
            lock_entity = cg.Pvariable(ID(f"{l['id'].id}_hk_lock_entity", type=LockEntity), var.add_lock(await cg.get_variable(l['id'])))
            if "nfc_id" in l:
                cg.add_build_flag("-fexceptions")
                cg.add_platformio_option("build_unflags", "-fno-exceptions")
                nfc = await cg.get_variable(l["nfc_id"])
                cg.add(var.set_nfc_ctx(nfc))
                cg.add(var.set_hk_hw_finish(l["hk_hw_finish"]))
                cg.add_define("USE_HOMEKEY")
                add_idf_component(
                    name="HK-HomeKit-Lib",
                    repo="https://github.com/rednblkx/HK-HomeKit-Lib.git",
                    ref="a4af730ec54536e1ba931413206fec89ce2b6c4f"
                )
                for conf in l.get("on_hk_success", []):
                    trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
                    cg.add(lock_entity.register_onhk_trigger(trigger))
                    await automation.build_automation(trigger, [(cg.std_string, "x"), (cg.std_string, "y")], conf)
                for conf in l.get("on_hk_fail", []):
                    trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
                    cg.add(lock_entity.register_onhkfail_trigger(trigger))
                    await automation.build_automation(trigger, [], conf)
            if "meta" in l:
                info_temp = []
                for m in l["meta"]:
                    info_temp.append([ACC_INFO[m], l["meta"][m]])
                cg.add(lock_entity.setInfo(info_temp))
    if "fan" in config:
        for l in config["fan"]:
            fan_entity = cg.Pvariable(ID(f"{l['id'].id}_hk_fan_entity", type=FanEntity), var.add_fan(await cg.get_variable(l['id'])))
            if "meta" in l:
                info_temp = []
                for m in l["meta"]:
                    info_temp.append([ACC_INFO[m], l["meta"][m]])
                cg.add(fan_entity.setInfo(info_temp))
    if "switch" in config:
        for l in config["switch"]:
            switch_entity = cg.Pvariable(ID(f"{l['id'].id}_hk_switch_entity", type=SwitchEntity), var.add_switch(await cg.get_variable(l['id'])))
            if "meta" in l:
                info_temp = []
                for m in l["meta"]:
                    info_temp.append([ACC_INFO[m], l["meta"][m]])
                cg.add(switch_entity.setInfo(info_temp))
    if "climate" in config:
        for l in config["climate"]:
            climate_entity = cg.Pvariable(ID(f"{l['id'].id}_hk_climate_entity", type=ClimateEntity), var.add_climate(await cg.get_variable(l['id'])))
            if "meta" in l:
                info_temp = []
                for m in l["meta"]:
                    info_temp.append([ACC_INFO[m], l["meta"][m]])
                cg.add(climate_entity.setInfo(info_temp))
    if "cover" in config:
        for l in config["cover"]:
            cover_entity = cg.Pvariable(ID(f"{l['id'].id}_hk_cover_entity", type=CoverEntity), var.add_cover(await cg.get_variable(l['id'])))
            if "meta" in l:
                info_temp = []
                for m in l["meta"]:
                    info_temp.append([ACC_INFO[m], l["meta"][m]])
                cg.add(cover_entity.setInfo(info_temp))