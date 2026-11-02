"""Tests for the use_rmt option of the GPIO 1-wire bus."""

from collections.abc import Callable
from pathlib import Path

import pytest

from esphome.components import esp32
from esphome.components.gpio.one_wire import final_validate, validate_use_rmt
import esphome.config_validation as cv
from esphome.const import PlatformFramework
from esphome.core import CORE
from tests.component_tests.types import SetCoreConfigCallable

CONFIG_DIR = Path(__file__).parent / "config"


def _rmt_defined() -> bool:
    return any(define.name == "USE_ONE_WIRE_RMT" for define in CORE.defines)


def _rmt_driver_included() -> bool:
    return (
        "esp_driver_rmt" not in CORE.data[esp32.KEY_ESP32][esp32.KEY_EXCLUDE_COMPONENTS]
    )


def test_bit_banging_is_the_default(generate_main: Callable[[str | Path], str]) -> None:
    generate_main(CONFIG_DIR / "esp32_default.yaml")

    assert not _rmt_defined()
    assert not _rmt_driver_included()


def test_use_rmt_on_esp32(generate_main: Callable[[str | Path], str]) -> None:
    main_cpp = generate_main(CONFIG_DIR / "esp32_rmt.yaml")

    assert "gpio::GPIOOneWireBus" in main_cpp
    assert _rmt_defined()
    assert _rmt_driver_included()


def test_two_buses_share_the_rmt_driver(
    generate_main: Callable[[str | Path], str],
) -> None:
    main_cpp = generate_main(CONFIG_DIR / "esp32_rmt_two_buses.yaml")

    assert "ow_bus_1" in main_cpp
    assert "ow_bus_2" in main_cpp
    assert _rmt_driver_included()


def test_use_rmt_is_rejected_off_esp32(set_core_config: SetCoreConfigCallable) -> None:
    set_core_config(PlatformFramework.ESP8266_ARDUINO)

    with pytest.raises(cv.Invalid, match="only available on"):
        validate_use_rmt({"use_rmt": True})
    assert validate_use_rmt({"use_rmt": False}) == {"use_rmt": False}


@pytest.mark.parametrize(
    ("variant", "accepted"),
    [(esp32.VARIANT_ESP32, True), (esp32.VARIANT_ESP32C2, False)],
)
def test_use_rmt_needs_rmt_hardware(
    set_core_config: SetCoreConfigCallable, variant: str, accepted: bool
) -> None:
    set_core_config(
        PlatformFramework.ESP32_IDF,
        platform_data={esp32.KEY_VARIANT: variant},
    )

    if accepted:
        assert validate_use_rmt({"use_rmt": True}) == {"use_rmt": True}
    else:
        with pytest.raises(cv.Invalid, match="no RMT hardware"):
            validate_use_rmt({"use_rmt": True})
    # Bit-banging stays available everywhere
    assert validate_use_rmt({"use_rmt": False}) == {"use_rmt": False}


def test_use_rmt_rejects_an_inverted_pin(
    set_core_config: SetCoreConfigCallable,
) -> None:
    set_core_config(PlatformFramework.ESP32_IDF)

    with pytest.raises(cv.Invalid, match="inverted"):
        validate_use_rmt({"use_rmt": True, "pin": {"inverted": True}})
    assert validate_use_rmt({"use_rmt": False, "pin": {"inverted": True}})


@pytest.mark.parametrize(
    ("settings", "accepted"),
    [([True, True], True), ([False, False], True), ([True, False], False)],
)
def test_all_buses_must_agree_on_use_rmt(
    set_core_config: SetCoreConfigCallable, settings: list[bool], accepted: bool
) -> None:
    buses = [{"platform": "gpio", "use_rmt": use_rmt} for use_rmt in settings]
    set_core_config(PlatformFramework.ESP32_IDF, full_config={"one_wire": buses})

    if accepted:
        assert final_validate(buses[0]) == buses[0]
    else:
        with pytest.raises(cv.Invalid, match="same 'use_rmt'"):
            final_validate(buses[0])
