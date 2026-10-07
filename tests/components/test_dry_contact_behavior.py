"""Tests for the dry contact toggle and obstruction behavior options (issue #467)."""

from pathlib import Path
import sys

import pytest

# Add repo root to path so components.ratgdo is importable
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import esphome.config_validation as cv  # noqa: E402

from components.ratgdo import (  # noqa: E402
    CONF_DRY_CONTACT_CLOSE_SENSOR,
    CONF_DRY_CONTACT_OPEN_SENSOR,
    CONF_ENCODER_PIN_A,
    CONF_ENCODER_PIN_B,
    CONF_ENCODER_SENSOR,
    CONF_INPUT_OBST,
    CONF_OBSTRUCTION_WHILE_CLOSING,
    CONF_OBSTRUCTION_WHILE_OPENING,
    CONF_PROTOCOL,
    CONF_TOGGLE_WHILE_CLOSING,
    CONF_TOGGLE_WHILE_OPENING,
    CONF_TOGGLE_WHILE_STOPPED,
    PROTOCOL_DRYCONTACT,
    PROTOCOL_SECPLUSV2,
    validate_protocol,
)

TOGGLE = {
    CONF_TOGGLE_WHILE_OPENING: "stop",
    CONF_TOGGLE_WHILE_CLOSING: "stop",
    CONF_TOGGLE_WHILE_STOPPED: "reverse",
}


def dry_contact_config(**extra):
    config = {
        CONF_PROTOCOL: PROTOCOL_DRYCONTACT,
        CONF_DRY_CONTACT_OPEN_SENSOR: "open",
        CONF_DRY_CONTACT_CLOSE_SENSOR: "close",
    }
    config.update(extra)
    return config


def test_toggle_keys_accepted_for_dry_contact() -> None:
    config = dry_contact_config(**TOGGLE)
    assert validate_protocol(config) is config


def test_toggle_keys_rejected_for_other_protocols() -> None:
    config = {CONF_PROTOCOL: PROTOCOL_SECPLUSV2, **TOGGLE}
    with pytest.raises(cv.Invalid, match="require protocol drycontact"):
        validate_protocol(config)


def test_toggle_keys_rejected_with_encoder() -> None:
    config = {
        CONF_PROTOCOL: PROTOCOL_DRYCONTACT,
        CONF_ENCODER_SENSOR: "encoder",
        CONF_ENCODER_PIN_A: "D5",
        CONF_ENCODER_PIN_B: "D6",
        **TOGGLE,
    }
    with pytest.raises(cv.Invalid, match="without encoder_sensor"):
        validate_protocol(config)


def test_obstruction_keys_accepted_for_dry_contact() -> None:
    config = dry_contact_config(
        **{
            CONF_INPUT_OBST: "D7",
            CONF_OBSTRUCTION_WHILE_OPENING: "ignore",
            CONF_OBSTRUCTION_WHILE_CLOSING: "reverse",
        }
    )
    assert validate_protocol(config) is config


@pytest.mark.parametrize(
    "key", [CONF_OBSTRUCTION_WHILE_OPENING, CONF_OBSTRUCTION_WHILE_CLOSING]
)
def test_obstruction_behavior_requires_obstruction_input(key: str) -> None:
    config = dry_contact_config(**{CONF_INPUT_OBST: None, key: "reverse"})
    with pytest.raises(cv.Invalid, match=f"{key} requires {CONF_INPUT_OBST}"):
        validate_protocol(config)
