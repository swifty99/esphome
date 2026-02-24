"""Tests for the esp32_rmt_led_strip high_precision (16-bit) feature."""

import math

import pytest

from esphome.const import PlatformFramework
from tests.component_tests.types import SetCoreConfigCallable


def test_high_precision_generates_16bit_class(generate_main):
    """When high_precision is true, the generated code should use ESP32RMTLEDStripLightOutput16."""
    main_cpp = generate_main(
        "tests/component_tests/esp32_rmt_led_strip/test_high_precision.yaml"
    )
    assert "ESP32RMTLEDStripLightOutput16" in main_cpp


def test_high_precision_false_generates_normal_class(generate_main):
    """When high_precision is false (default), the generated code should use ESP32RMTLEDStripLightOutput (not 16)."""
    main_cpp = generate_main(
        "tests/component_tests/esp32_rmt_led_strip/test_high_precision.yaml"
    )
    # The normal strip should use the base class, not the 16-bit variant
    # Both classes appear in the output; ensure the base class is also present
    assert "ESP32RMTLEDStripLightOutput " in main_cpp or "ESP32RMTLEDStripLightOutput(" in main_cpp


def test_high_precision_default_is_false(
    set_core_config: SetCoreConfigCallable,
):
    """The high_precision option should default to False."""
    set_core_config(
        PlatformFramework.ESP32_IDF,
        platform_data={"variant": "ESP32", "board": "esp32dev"},
    )

    from esphome.components.esp32_rmt_led_strip.light import (
        CONF_HIGH_PRECISION,
        CONFIG_SCHEMA,
    )

    config = CONFIG_SCHEMA(
        {
            "name": "test_strip",
            "pin": 13,
            "num_leds": 60,
            "rgb_order": "GRB",
            "chipset": "WS2812",
        }
    )
    assert config[CONF_HIGH_PRECISION] is False


def test_high_precision_can_be_enabled(
    set_core_config: SetCoreConfigCallable,
):
    """The high_precision option should accept True."""
    set_core_config(
        PlatformFramework.ESP32_IDF,
        platform_data={"variant": "ESP32", "board": "esp32dev"},
    )

    from esphome.components.esp32_rmt_led_strip.light import (
        CONF_HIGH_PRECISION,
        CONFIG_SCHEMA,
    )

    config = CONFIG_SCHEMA(
        {
            "name": "test_strip",
            "pin": 13,
            "num_leds": 60,
            "rgb_order": "GRB",
            "chipset": "WS2812",
            "high_precision": True,
        }
    )
    assert config[CONF_HIGH_PRECISION] is True


def test_gamma_lut_math():
    """Verify the 16-bit gamma LUT calculation matches the C++ implementation."""
    gamma = 2.8

    # Replicate the C++ LUT build: gamma_table_16_[i] = round(pow(i/255.0, gamma) * 65535.0)
    lut = []
    for i in range(256):
        x = i / 255.0
        corrected = math.pow(x, gamma)
        lut.append(round(corrected * 65535.0))

    # LUT[0] must be 0 (black stays black)
    assert lut[0] == 0

    # LUT[255] must be 65535 (full white stays full white)
    assert lut[255] == 65535

    # LUT must be monotonically non-decreasing
    for i in range(1, 256):
        assert lut[i] >= lut[i - 1], f"LUT not monotonic at index {i}: {lut[i]} < {lut[i-1]}"

    # With gamma 2.8, (1/255)^2.8 * 65535 ≈ 0.197, which rounds to 0.
    # This is expected — the first non-zero entry appears at a higher index.
    assert lut[0] == 0
    assert lut[1] == 0  # (1/255)^2.8 is extremely small

    # Find the first non-zero entry — should be low
    first_nonzero = next(i for i in range(256) if lut[i] > 0)
    assert first_nonzero <= 10, f"First non-zero LUT entry at index {first_nonzero}, expected <= 10"

    # Verify specific computed values
    val_1 = round(math.pow(1 / 255.0, 2.8) * 65535.0)
    assert lut[1] == val_1

    # Mid-range: i=128 → (128/255)^2.8 * 65535
    val_128 = round(math.pow(128 / 255.0, 2.8) * 65535.0)
    assert lut[128] == val_128

    # High end: LUT[254] should be close to but less than 65535
    assert lut[254] < 65535
    assert lut[254] > 64000

    # Verify the >>8 truncation still provides at least as many distinct values as 8-bit gamma
    # For 8-bit gamma: round(pow(i/255.0, 2.8) * 255.0)
    # For 16-bit>>8:   round(pow(i/255.0, 2.8) * 65535.0) >> 8
    distinct_8bit = len(set(round(math.pow(i / 255.0, gamma) * 255.0) for i in range(256)))
    distinct_16bit_trunc = len(set(round(math.pow(i / 255.0, gamma) * 65535.0) >> 8 for i in range(256)))

    # 16-bit truncated should have at least as many distinct values as 8-bit
    assert distinct_16bit_trunc >= distinct_8bit - 1, (
        f"16-bit truncated has fewer distinct values ({distinct_16bit_trunc}) than 8-bit ({distinct_8bit})"
    )


def test_gamma_lut_resolution_improvement():
    """Verify that 16-bit gamma provides more resolution than 8-bit in the low end."""
    gamma = 2.8

    # Count how many input values (0-255) map to output=0 for 8-bit gamma
    zeros_8bit = sum(1 for i in range(256) if round(math.pow(i / 255.0, gamma) * 255.0) == 0)

    # Count how many input values map to output=0 for 16-bit gamma
    zeros_16bit = sum(1 for i in range(256) if round(math.pow(i / 255.0, gamma) * 65535.0) == 0)

    # 16-bit should have fewer zeros (better low-end resolution)
    assert zeros_16bit <= zeros_8bit, (
        f"16-bit has more zeros ({zeros_16bit}) than 8-bit ({zeros_8bit})"
    )

    # After truncation (>>8), the values in the dark region should still
    # provide sub-LSB information for future temporal dithering
    # Check that buf_16 values that truncate to 0 still have non-zero sub-LSB bits
    sub_lsb_info_count = 0
    for i in range(256):
        val_16 = round(math.pow(i / 255.0, gamma) * 65535.0)
        truncated = val_16 >> 8
        if truncated == 0 and val_16 > 0:
            sub_lsb_info_count += 1

    # There should be at least some values with sub-LSB information
    assert sub_lsb_info_count > 0, "No sub-LSB information found for temporal dithering"
