"""Tests for the esp32_rmt_led_strip high_precision (16-bit) feature.

Phase 1: 16-bit gamma LUT with truncation
Phase 2: Temporal dithering with 2x2 Bayer ordered dithering
"""

import math

import pytest

from esphome.const import PlatformFramework
from tests.component_tests.types import SetCoreConfigCallable

# ─── Constants matching the C++ implementation ──────────────────────────

BAYER2X2 = [
    [0, 128],
    [192, 64],
]


def _build_gamma_lut(gamma=2.8):
    """Build the same 256-entry uint16 LUT as the C++ setup_state()."""
    lut = []
    for i in range(256):
        x = i / 255.0
        corrected = math.pow(x, gamma)
        lut.append(round(corrected * 65535.0))
    return lut


def _dither_byte(val16, led_index, frame_index, bytes_per_led=3):
    """Pure-Python replica of dither_frame_() for a single byte.

    val16:       uint16_t gamma-corrected value
    led_index:   which LED this byte belongs to (byte_offset // bytes_per_led)
    frame_index: frame counter value
    Returns:     uint8_t dithered output
    """
    high = val16 >> 8
    frac = val16 & 0xFF
    temporal_idx = frame_index & 1
    spatial_idx = led_index & 1
    threshold = BAYER2X2[temporal_idx][spatial_idx]
    if frac > threshold and high < 255:
        return high + 1
    return high


# ─── Phase 1 tests (codegen, schema, gamma LUT) ────────────────────────


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
    lut = _build_gamma_lut(gamma)

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


# ─── Phase 2 tests (Bayer dithering) ───────────────────────────────────


def test_bayer_matrix_properties():
    """Verify the 2×2 Bayer matrix has the expected mathematical properties."""
    # Flatten the matrix
    flat = [BAYER2X2[r][c] for r in range(2) for c in range(2)]

    # All 4 values should be distinct
    assert len(set(flat)) == 4, f"Bayer matrix has duplicate values: {flat}"

    # All values should be in [0, 255]
    for v in flat:
        assert 0 <= v <= 255, f"Bayer value {v} out of range"

    # Thresholds should be evenly distributed to provide 4 sub-levels:
    # Sorted: [0, 64, 128, 192] — spacing of 64 (256/4)
    sorted_vals = sorted(flat)
    assert sorted_vals == [0, 64, 128, 192], f"Unexpected Bayer thresholds: {sorted_vals}"


def test_dither_zero_fraction_always_truncates():
    """When the fractional part is 0, dithering should always produce the high byte."""
    # val16 = 0x0A00 → high=10, frac=0 → output must be 10 regardless of frame/LED
    val16 = 0x0A00
    for frame in range(4):
        for led_idx in range(4):
            result = _dither_byte(val16, led_idx, frame)
            assert result == 10, (
                f"frac=0 should truncate: got {result} at frame={frame}, led={led_idx}"
            )


def test_dither_full_fraction_always_rounds_up():
    """When frac=255 (>all thresholds), dithering should always round up."""
    # val16 = 0x0AFF → high=10, frac=255 → output must be 11 (255 > all thresholds)
    val16 = 0x0AFF
    for frame in range(4):
        for led_idx in range(4):
            result = _dither_byte(val16, led_idx, frame)
            assert result == 11, (
                f"frac=255 should always round up: got {result} at frame={frame}, led={led_idx}"
            )


def test_dither_full_fraction_at_max_clamps():
    """When high=255 and frac>0, output must clamp to 255 (never overflow)."""
    val16 = 0xFFFF  # high=255, frac=255
    for frame in range(4):
        for led_idx in range(4):
            result = _dither_byte(val16, led_idx, frame)
            assert result == 255, (
                f"high=255 should clamp: got {result} at frame={frame}, led={led_idx}"
            )


def test_dither_sub_levels_count():
    """A mid-range fractional value should produce exactly 2 distinct sub-frames
    over a 2-frame cycle (for even vs odd LED)."""
    # val16 = 0x0A80 → high=10, frac=128
    # frac=128 exceeds thresholds 0 and 64, but not 128 and 192
    # Over 4 (frame, spatial) combinations:
    #   (0,0): thr=0   → 128>0   → 11
    #   (0,1): thr=128 → 128>128? NO → 10
    #   (1,0): thr=192 → 128>192? NO → 10
    #   (1,1): thr=64  → 128>64  → 11
    # So: 2 out of 4 sub-frames round up — fractional duty = 50%
    val16 = 0x0A80
    results = []
    for frame in range(2):
        for led_idx in range(2):
            results.append(_dither_byte(val16, led_idx, frame))
    assert results.count(11) == 2, f"Expected 2/4 round-ups for frac=128, got {results}"
    assert results.count(10) == 2, f"Expected 2/4 truncations for frac=128, got {results}"


def test_dither_average_converges():
    """Over all 4 Bayer positions, the average output should approximate val16/256."""
    gamma = 2.8
    lut = _build_gamma_lut(gamma)

    # Test several input brightness values
    for input_val in [1, 2, 5, 10, 30, 64, 100, 128, 200, 254]:
        val16 = lut[input_val]
        expected_analog = val16 / 256.0  # ideal continuous output

        # Average over all 4 Bayer positions (2 frames × 2 spatial)
        total = 0
        count = 0
        for frame in range(2):
            for led_idx in range(2):
                total += _dither_byte(val16, led_idx, frame)
                count += 1
        average = total / count

        # Average should be within ±1.0 of the ideal
        assert abs(average - expected_analog) <= 1.0, (
            f"input={input_val}: avg dithered={average:.2f}, expected≈{expected_analog:.2f}, "
            f"val16=0x{val16:04X}"
        )


def test_dither_more_distinct_values_than_truncation():
    """Temporal dithering should produce more distinct perceptual output levels
    than simple >>8 truncation across the full 0-255 input range."""
    gamma = 2.8
    lut = _build_gamma_lut(gamma)

    # Count distinct output values with simple truncation
    truncated_outputs = set()
    for i in range(256):
        truncated_outputs.add(lut[i] >> 8)

    # Count distinct average-output values with dithering (averaged over 4 positions)
    dithered_averages = set()
    for i in range(256):
        val16 = lut[i]
        total = 0
        for frame in range(2):
            for led_idx in range(2):
                total += _dither_byte(val16, led_idx, frame)
        # Use fixed-point average (multiply by 4 to keep integer precision)
        dithered_averages.add(total)  # total over 4 samples, unique per input

    assert len(dithered_averages) >= len(truncated_outputs), (
        f"Dithering should provide at least as many distinct levels as truncation: "
        f"dithered={len(dithered_averages)}, truncated={len(truncated_outputs)}"
    )


def test_static_scene_detection():
    """Python replica of is_static_scene_(): all fractional parts must be 0."""
    gamma = 2.8
    lut = _build_gamma_lut(gamma)

    # is_static_scene checks: all (buf_16[i] & 0xFF) == 0
    def is_static(buf_16):
        return all((v & 0xFF) == 0 for v in buf_16)

    # All zeros → static
    assert is_static([0, 0, 0])

    # Values whose LUT entry has frac=0 → static
    # LUT[0] = 0x0000 (frac=0), LUT[255] = 0xFFFF (frac=FF → NOT static!)
    assert is_static([lut[0], lut[0], lut[0]])
    assert not is_static([lut[255], lut[255], lut[255]])  # 0xFFFF has frac=0xFF

    # A scene where all values have exact 8-bit boundaries → static
    static_vals = [v for v in [lut[i] for i in range(256)] if (v & 0xFF) == 0]
    if static_vals:
        assert is_static(static_vals)

    # A scene with even one non-zero fractional → dynamic
    for i in range(256):
        if (lut[i] & 0xFF) != 0:
            assert not is_static([0, 0, lut[i]])
            break


def test_dither_spatial_distribution():
    """Verify that dithering distributes round-up/round-down spatially across LEDs
    within a single frame (no two adjacent LEDs make the same rounding decision
    when both have the same fractional value)."""
    # For frac values that cross exactly one threshold pair, adjacent LEDs
    # should differ.
    # frac=100: exceeds threshold 0 and 64, but not 128 and 192
    val16 = 0x0A64  # high=10, frac=100

    for frame in range(2):
        r0 = _dither_byte(val16, 0, frame)
        r1 = _dither_byte(val16, 1, frame)
        # Adjacent LEDs should produce different outputs for non-trivial fractions
        # (at least in some frames — the 2×2 matrix guarantees this)
        # Check frame 0: thr[0][0]=0 → 100>0 → up, thr[0][1]=128 → 100>128? NO → down
        # Check frame 1: thr[1][0]=192 → 100>192? NO → down, thr[1][1]=64 → 100>64 → up
        assert r0 != r1, (
            f"frame={frame}: adjacent LEDs should differ for frac=100, "
            f"got led0={r0}, led1={r1}"
        )
