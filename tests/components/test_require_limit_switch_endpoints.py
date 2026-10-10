"""Tests for the require_limit_switch_endpoints option (issue #684)."""

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
    CONF_PROTOCOL,
    CONF_REQUIRE_LIMIT_SWITCH_ENDPOINTS,
    PROTOCOL_DRYCONTACT,
    PROTOCOL_SECPLUSV2,
    validate_protocol,
)


def test_accepted_with_limit_switches() -> None:
    config = {
        CONF_PROTOCOL: PROTOCOL_DRYCONTACT,
        CONF_DRY_CONTACT_OPEN_SENSOR: "open",
        CONF_DRY_CONTACT_CLOSE_SENSOR: "close",
        CONF_REQUIRE_LIMIT_SWITCH_ENDPOINTS: True,
    }
    assert validate_protocol(config) is config


def test_false_is_accepted_anywhere() -> None:
    config = {
        CONF_PROTOCOL: PROTOCOL_SECPLUSV2,
        CONF_REQUIRE_LIMIT_SWITCH_ENDPOINTS: False,
    }
    assert validate_protocol(config) is config


def test_rejected_for_other_protocols() -> None:
    config = {
        CONF_PROTOCOL: PROTOCOL_SECPLUSV2,
        CONF_REQUIRE_LIMIT_SWITCH_ENDPOINTS: True,
    }
    with pytest.raises(cv.Invalid, match="requires protocol drycontact"):
        validate_protocol(config)


def test_rejected_with_encoder() -> None:
    config = {
        CONF_PROTOCOL: PROTOCOL_DRYCONTACT,
        CONF_ENCODER_SENSOR: "encoder",
        CONF_ENCODER_PIN_A: "D5",
        CONF_ENCODER_PIN_B: "D6",
        CONF_REQUIRE_LIMIT_SWITCH_ENDPOINTS: True,
    }
    with pytest.raises(cv.Invalid, match="requires protocol drycontact"):
        validate_protocol(config)
