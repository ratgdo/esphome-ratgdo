"""Tests for the validate_unique helper in components/ratgdo/__init__.py.

The unique type checks must reject duplicates within a single validation
run but must not leak state across runs. Long lived processes such as the
dashboard editor validator (esphome vscode --ace) call CORE.reset() and
then revalidate, so state stored outside CORE.data would raise false
"Only one ... is allowed" errors on every validation after the first.
Regression test for https://github.com/ratgdo/esphome-ratgdo/issues/625.
"""

from pathlib import Path
import sys

import pytest

# Add repo root to path so components.ratgdo is importable
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import esphome.config_validation as cv  # noqa: E402
from esphome.core import CORE  # noqa: E402

from components.ratgdo import validate_unique  # noqa: E402


@pytest.fixture(autouse=True)
def reset_core():
    """Start and leave each test with clean per-run state."""
    CORE.reset()
    yield
    CORE.reset()


def test_duplicate_within_run_raises():
    validate_unique("sensor", "openings", "Only one openings sensor is allowed")
    with pytest.raises(cv.Invalid, match="Only one openings sensor is allowed"):
        validate_unique("sensor", "openings", "Only one openings sensor is allowed")


def test_state_does_not_leak_across_runs():
    validate_unique("sensor", "openings", "Only one openings sensor is allowed")
    CORE.reset()
    # A new validation run of the same config must pass
    validate_unique("sensor", "openings", "Only one openings sensor is allowed")


def test_kinds_are_isolated():
    # The same type name under different platforms must not collide
    validate_unique("sensor", "openings", "sensor dup")
    validate_unique("binary_sensor", "openings", "binary_sensor dup")


def test_different_values_within_run_pass():
    validate_unique("sensor", "openings", "dup")
    validate_unique("sensor", "paired_devices_total", "dup")
