from dataclasses import dataclass, field

from esphome import automation, pins
import esphome.codegen as cg
from esphome.components import binary_sensor, sensor
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_TRIGGER_ID
from esphome.core import CORE
from esphome.coroutine import CoroPriority, coroutine_with_priority
import voluptuous as vol

DEPENDENCIES = ["preferences"]
MULTI_CONF = False

DOMAIN = "ratgdo"

ratgdo_ns = cg.esphome_ns.namespace("ratgdo")
RATGDO = ratgdo_ns.class_("RATGDOComponent", cg.Component)


@dataclass
class RATGDOData:
    """Per-run state: observable subscriber counts and used entity types."""

    door_state: int = 0
    door_action_delayed: int = 0
    distance: int = 0
    vehicle_detected: int = 0
    vehicle_arriving: int = 0
    vehicle_leaving: int = 0
    used_types: dict[str, set[str]] = field(default_factory=dict)


def _get_data() -> RATGDOData:
    if DOMAIN not in CORE.data:
        CORE.data[DOMAIN] = RATGDOData()
    return CORE.data[DOMAIN]


def subscribe_door_state() -> None:
    _get_data().door_state += 1


def subscribe_door_action_delayed() -> None:
    _get_data().door_action_delayed += 1


def subscribe_distance() -> None:
    _get_data().distance += 1


def subscribe_vehicle_detected() -> None:
    _get_data().vehicle_detected += 1


def subscribe_vehicle_arriving() -> None:
    _get_data().vehicle_arriving += 1


def subscribe_vehicle_leaving() -> None:
    _get_data().vehicle_leaving += 1


def validate_unique(kind: str, value: str, message: str) -> None:
    """Raise cv.Invalid if (kind, value) was already seen in this validation run.

    State lives in CORE.data (cleared by CORE.reset() between runs) rather than
    a module-level global, which would leak across validation runs in
    long-lived processes like the dashboard's `esphome vscode --ace` loop.
    """
    used = _get_data().used_types.setdefault(kind, set())
    if value in used:
        raise cv.Invalid(message)
    used.add(value)


@coroutine_with_priority(CoroPriority.FINAL)
async def _emit_subscriber_defines():
    """Emit observable subscriber count defines after all children have registered."""
    data = _get_data()
    cg.add_define("RATGDO_MAX_DOOR_STATE_SUBSCRIBERS", data.door_state)
    cg.add_define(
        "RATGDO_MAX_DOOR_ACTION_DELAYED_SUBSCRIBERS", data.door_action_delayed
    )
    cg.add_define("RATGDO_MAX_DISTANCE_SUBSCRIBERS", data.distance)
    cg.add_define("RATGDO_MAX_VEHICLE_DETECTED_SUBSCRIBERS", data.vehicle_detected)
    cg.add_define("RATGDO_MAX_VEHICLE_ARRIVING_SUBSCRIBERS", data.vehicle_arriving)
    cg.add_define("RATGDO_MAX_VEHICLE_LEAVING_SUBSCRIBERS", data.vehicle_leaving)


SyncFailed = ratgdo_ns.class_("SyncFailed", automation.Trigger.template())

CONF_OUTPUT_GDO = "output_gdo_pin"
DEFAULT_OUTPUT_GDO = (
    "D4"  # D4 red control terminal / GarageDoorOpener (UART1 TX) pin is D4 on D1 Mini
)
CONF_INPUT_GDO = "input_gdo_pin"
DEFAULT_INPUT_GDO = (
    "D2"  # D2 red control terminal / GarageDoorOpener (UART1 RX) pin is D2 on D1 Mini
)
CONF_INPUT_OBST = "input_obst_pin"
DEFAULT_INPUT_OBST = "D7"  # D7 black obstruction sensor terminal

CONF_DISCRETE_OPEN_PIN = "discrete_open_pin"
CONF_DISCRETE_CLOSE_PIN = "discrete_close_pin"

CONF_RATGDO_ID = "ratgdo_id"

CONF_ON_SYNC_FAILED = "on_sync_failed"

CONF_PROTOCOL = "protocol"

PROTOCOL_SECPLUSV1 = "secplusv1"
PROTOCOL_SECPLUSV2 = "secplusv2"
PROTOCOL_DRYCONTACT = "drycontact"
SUPPORTED_PROTOCOLS = [PROTOCOL_SECPLUSV1, PROTOCOL_SECPLUSV2, PROTOCOL_DRYCONTACT]

_PROTOCOL_SOURCE_FILES = {
    PROTOCOL_SECPLUSV1: "secplus1.cpp",
    PROTOCOL_SECPLUSV2: "secplus2.cpp",
    PROTOCOL_DRYCONTACT: "dry_contact.cpp",
}


def FILTER_SOURCE_FILES() -> list[str]:
    """Exclude protocol implementations that are not selected in YAML."""
    selected = CORE.config.get(DOMAIN, {}).get(CONF_PROTOCOL, PROTOCOL_SECPLUSV2)
    return [
        source
        for protocol, source in _PROTOCOL_SOURCE_FILES.items()
        if protocol != selected
    ]


CONF_DRY_CONTACT_OPEN_SENSOR = "dry_contact_open_sensor"
CONF_DRY_CONTACT_CLOSE_SENSOR = "dry_contact_close_sensor"
CONF_DRY_CONTACT_SENSOR_GROUP = "dry_contact_sensor_group"
CONF_ENCODER_PIN_A = "encoder_pin_a"
CONF_ENCODER_PIN_B = "encoder_pin_b"
CONF_ENCODER_SENSOR = "encoder_sensor"


def validate_protocol(config):
    is_dry = config.get(CONF_PROTOCOL, None) == PROTOCOL_DRYCONTACT
    has_open = CONF_DRY_CONTACT_OPEN_SENSOR in config
    has_close = CONF_DRY_CONTACT_CLOSE_SENSOR in config
    has_encoder = CONF_ENCODER_SENSOR in config

    if is_dry:
        has_limits = has_open and has_close
        if not has_limits and not has_encoder:
            raise cv.Invalid(
                "drycontact protocol requires either both "
                "dry_contact_open_sensor and dry_contact_close_sensor, "
                "or encoder_sensor"
            )
        if has_limits and has_encoder:
            raise cv.Invalid(
                "drycontact protocol cannot use both limit switch sensors "
                "and encoder_sensor simultaneously"
            )
        if (has_open and not has_close) or (has_close and not has_open):
            raise cv.Invalid(
                "dry_contact_open_sensor and dry_contact_close_sensor must both be defined"
            )
        if has_encoder:
            has_pin_a = CONF_ENCODER_PIN_A in config
            has_pin_b = CONF_ENCODER_PIN_B in config
            if not has_pin_a or not has_pin_b:
                raise cv.Invalid(
                    "encoder_sensor requires both encoder_pin_a and "
                    "encoder_pin_b to be defined in the ratgdo: config"
                )
    else:
        if has_encoder:
            raise cv.Invalid(
                "encoder_sensor is only supported with protocol: drycontact"
            )
        if has_open or has_close:
            raise cv.Invalid(
                "dry_contact_open_sensor and dry_contact_close_sensor are only valid "
                "when using protocol drycontact"
            )

    if not has_encoder and (
        CONF_ENCODER_PIN_A in config or CONF_ENCODER_PIN_B in config
    ):
        raise cv.Invalid(
            "encoder_pin_a and encoder_pin_b are only valid when using encoder_sensor"
        )
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(RATGDO),
            cv.Optional(
                CONF_OUTPUT_GDO, default=DEFAULT_OUTPUT_GDO
            ): pins.gpio_output_pin_schema,
            cv.Optional(
                CONF_INPUT_GDO, default=DEFAULT_INPUT_GDO
            ): pins.gpio_input_pin_schema,
            cv.Optional(CONF_INPUT_OBST, default=DEFAULT_INPUT_OBST): cv.Any(
                cv.none, pins.gpio_input_pin_schema
            ),
            cv.Optional(CONF_DISCRETE_OPEN_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_DISCRETE_CLOSE_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_ON_SYNC_FAILED): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(SyncFailed),
                }
            ),
            cv.Optional(CONF_PROTOCOL, default=PROTOCOL_SECPLUSV2): cv.All(
                vol.In(SUPPORTED_PROTOCOLS)
            ),
            # cv.Inclusive(CONF_DRY_CONTACT_OPEN_SENSOR,CONF_DRY_CONTACT_SENSOR_GROUP): cv.use_id(binary_sensor.BinarySensor),
            # cv.Inclusive(CONF_DRY_CONTACT_CLOSE_SENSOR,CONF_DRY_CONTACT_SENSOR_GROUP): cv.use_id(binary_sensor.BinarySensor),
            cv.Optional(CONF_DRY_CONTACT_OPEN_SENSOR): cv.use_id(
                binary_sensor.BinarySensor
            ),
            cv.Optional(CONF_DRY_CONTACT_CLOSE_SENSOR): cv.use_id(
                binary_sensor.BinarySensor
            ),
            cv.Optional(CONF_ENCODER_PIN_A): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_ENCODER_PIN_B): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_ENCODER_SENSOR): cv.use_id(sensor.Sensor),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    validate_protocol,
)

RATGDO_CLIENT_SCHMEA = cv.Schema(
    {
        cv.GenerateID(CONF_RATGDO_ID): cv.use_id(RATGDO),
    }
)


async def register_ratgdo_child(var, config):
    parent = await cg.get_variable(config[CONF_RATGDO_ID])
    cg.add(var.set_parent(parent))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    pin = await cg.gpio_pin_expression(config[CONF_OUTPUT_GDO])
    cg.add(var.set_output_gdo_pin(pin))
    pin = await cg.gpio_pin_expression(config[CONF_INPUT_GDO])
    cg.add(var.set_input_gdo_pin(pin))
    if config.get(CONF_INPUT_OBST):
        pin = await cg.gpio_pin_expression(config[CONF_INPUT_OBST])
        cg.add(var.set_input_obst_pin(pin))

    if config.get(CONF_DRY_CONTACT_OPEN_SENSOR):
        dry_contact_open_sensor = await cg.get_variable(
            config[CONF_DRY_CONTACT_OPEN_SENSOR]
        )
        cg.add(var.set_dry_contact_open_sensor(dry_contact_open_sensor))

    if config.get(CONF_DRY_CONTACT_CLOSE_SENSOR):
        dry_contact_close_sensor = await cg.get_variable(
            config[CONF_DRY_CONTACT_CLOSE_SENSOR]
        )
        cg.add(var.set_dry_contact_close_sensor(dry_contact_close_sensor))

    if config.get(CONF_ENCODER_SENSOR):
        encoder_sensor = await cg.get_variable(config[CONF_ENCODER_SENSOR])
        cg.add(var.set_encoder_sensor(encoder_sensor))

    if config.get(CONF_ENCODER_PIN_A):
        pin = await cg.gpio_pin_expression(config[CONF_ENCODER_PIN_A])
        cg.add(var.set_encoder_pin_a(pin))
    if config.get(CONF_ENCODER_PIN_B):
        pin = await cg.gpio_pin_expression(config[CONF_ENCODER_PIN_B])
        cg.add(var.set_encoder_pin_b(pin))

    for conf in config.get(CONF_ON_SYNC_FAILED, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)

    if CORE.is_esp32 and not CORE.using_arduino:
        from esphome.components import esp32

        esp32.include_builtin_idf_component("esp_driver_rmt")
        esp32.add_idf_component(
            name="secplus",
            repo="https://github.com/ratgdo/secplus.git",
            ref="add-esp-idf-support",
        )
    else:
        cg.add_library(
            name="secplus",
            repository="https://github.com/ratgdo/secplus#f98c3220356c27717a25102c0b35815ebbd26ccc",
            version=None,
        )
    if CORE.is_esp8266:
        cg.add_library(
            name="espsoftwareserial",
            repository="https://github.com/ratgdo/espsoftwareserial#autobaud",
            version=None,
        )

    if config[CONF_PROTOCOL] == PROTOCOL_SECPLUSV1:
        cg.add_build_flag("-DPROTOCOL_SECPLUSV1")
    elif config[CONF_PROTOCOL] == PROTOCOL_SECPLUSV2:
        cg.add_build_flag("-DPROTOCOL_SECPLUSV2")
    elif config[CONF_PROTOCOL] == PROTOCOL_DRYCONTACT:
        cg.add_build_flag("-DPROTOCOL_DRYCONTACT")
    cg.add(var.init_protocol())

    # RATGDOComponent::setup() subscribes to door_state
    subscribe_door_state()

    # Emit observable subscriber count defines after all children register
    CORE.add_job(_emit_subscriber_defines)

    if config.get(CONF_DISCRETE_OPEN_PIN):
        pin = await cg.gpio_pin_expression(config[CONF_DISCRETE_OPEN_PIN])
        cg.add(var.set_discrete_open_pin(pin))
    if config.get(CONF_DISCRETE_CLOSE_PIN):
        pin = await cg.gpio_pin_expression(config[CONF_DISCRETE_CLOSE_PIN])
        cg.add(var.set_discrete_close_pin(pin))
