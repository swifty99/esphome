"""Shared helpers for isotp config-validation tests.

Mirrors ``tests/component_tests/can_gateway/common.py``: the same ESP32-C6 / IDF
core setup, a ``validate()`` that runs the component's ``CONFIG_SCHEMA`` (imported
late, after the core is set up), and a minimal valid base config that individual
tests copy and mutate.
"""

from __future__ import annotations

from typing import Any

from esphome.components.esp32.const import KEY_BOARD, KEY_VARIANT, VARIANT_ESP32C6
import esphome.config_validation as cv
from esphome.const import KEY_FRAMEWORK_VERSION, PlatformFramework

# The can_gateway port these instances layer on. Only the id matters at schema
# time: 'port_id' is a cv.use_id reference, resolved later.
PORT_ID = "diag_bus"

# 11-bit normal addressing, the common OBD-II request/response pair.
BASE = {
    "port_id": PORT_ID,
    "tx_id": 0x7E0,
    "rx_id": 0x7E8,
}


def setup_c6(set_core_config) -> None:
    """Set the core up as an ESP32-C6 / IDF target."""
    set_core_config(
        PlatformFramework.ESP32_IDF,
        core_data={KEY_FRAMEWORK_VERSION: cv.Version(5, 5, 4)},
        platform_data={
            KEY_BOARD: "esp32-c6-devkitc-1",
            KEY_VARIANT: VARIANT_ESP32C6,
        },
    )


def validate(config):
    """Run the component's CONFIG_SCHEMA (imported late, after setup_c6)."""
    from esphome.components.isotp import CONFIG_SCHEMA

    return CONFIG_SCHEMA(config)


def isotp(**overrides: Any) -> dict[str, Any]:
    """A copy of the base instance config with overrides applied.

    A value of ``None`` removes the key, so tests can drop ``tx_id`` / ``rx_id``
    when exercising the addressing modes that forbid them.
    """
    config = {**BASE, **overrides}
    return {key: value for key, value in config.items() if value is not None}


def normal_fixed(**overrides: Any) -> dict[str, Any]:
    """A normal_fixed instance: no raw identifiers, source/target instead."""
    defaults: dict[str, Any] = {
        "addressing": "normal_fixed",
        "tx_id": None,
        "rx_id": None,
        "source": 0xF1,
        "target": 0x01,
    }
    return isotp(**{**defaults, **overrides})
