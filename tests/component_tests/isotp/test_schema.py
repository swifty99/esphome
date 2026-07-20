"""isotp schema tests (V4, V6, V10, plus general schema and defaults).

V4   max_message_size in 8-4095.
V6   block_size at least 1.
V10  st_min accepts 0-127 ms or 100-900 us, and rejects the reserved encodings.

V5 (block_size against the port's observe_queue_depth) lives in
FINAL_VALIDATE_SCHEMA and needs a full-config fixture; it is covered elsewhere.
"""

from __future__ import annotations

import pytest

from esphome import config_validation as cv

from .common import isotp, setup_c6, validate

# ------------------------------------------------------------------------- V4


@pytest.mark.parametrize("size", [8, 64, 256, 4095])
def test_v4_max_message_size_accepted(set_core_config, size: int) -> None:
    setup_c6(set_core_config)
    assert validate(isotp(max_message_size=size))["max_message_size"] == size


@pytest.mark.parametrize("size", [7, 4096])
def test_v4_max_message_size_rejected(set_core_config, size: int) -> None:
    """Below one CAN frame is meaningless; above 4095 needs the 2016 escape."""
    setup_c6(set_core_config)
    with pytest.raises(cv.Invalid):
        validate(isotp(max_message_size=size))


# ------------------------------------------------------------------------- V6


def test_v6_block_size_zero_rejected(set_core_config) -> None:
    """Granting 0 invites the peer to send every remaining frame back to back."""
    setup_c6(set_core_config)
    with pytest.raises(cv.Invalid):
        validate(isotp(block_size=0))


@pytest.mark.parametrize("size", [1, 8, 255])
def test_v6_block_size_accepted(set_core_config, size: int) -> None:
    setup_c6(set_core_config)
    assert validate(isotp(block_size=size))["block_size"] == size


def test_v6_block_size_above_255_rejected(set_core_config) -> None:
    """The STmin/BS fields are single bytes on the wire."""
    setup_c6(set_core_config)
    with pytest.raises(cv.Invalid):
        validate(isotp(block_size=256))


# ------------------------------------------------------------------ V10 st_min

# ISO 15765-2 STmin encoding: 0x00-0x7F is whole milliseconds, 0xF1-0xF9 is
# 100-900 microseconds. Everything else is reserved and must not be emitted.


@pytest.mark.parametrize(
    ("value", "raw"),
    [
        ("0ms", 0x00),
        ("1ms", 0x01),
        ("20ms", 0x14),
        ("127ms", 0x7F),
        ("100us", 0xF1),
        ("500us", 0xF5),
        ("900us", 0xF9),
    ],
)
def test_v10_st_min_encoding(set_core_config, value: str, raw: int) -> None:
    setup_c6(set_core_config)
    assert validate(isotp(st_min=value))["st_min"] == raw


@pytest.mark.parametrize(
    "value",
    [
        "128ms",  # past the 0x7F millisecond range
        "150us",  # not a whole 100us step
        "1500us",  # neither a whole millisecond nor inside 100-900us
    ],
)
def test_v10_st_min_rejected(set_core_config, value: str) -> None:
    setup_c6(set_core_config)
    with pytest.raises(cv.Invalid, match="st_min"):
        validate(isotp(st_min=value))


# -------------------------------------------------------------- general schema


def test_port_id_is_required(set_core_config) -> None:
    setup_c6(set_core_config)
    with pytest.raises(cv.Invalid, match="required key not provided"):
        validate(isotp(port_id=None))


def test_padding_is_optional(set_core_config) -> None:
    setup_c6(set_core_config)
    assert "padding" not in validate(isotp())


def test_padding_accepts_a_byte(set_core_config) -> None:
    setup_c6(set_core_config)
    assert validate(isotp(padding=0xCC))["padding"] == 0xCC


def test_padding_rejects_values_above_a_byte(set_core_config) -> None:
    setup_c6(set_core_config)
    with pytest.raises(cv.Invalid):
        validate(isotp(padding=0x100))


def test_unknown_key_rejected(set_core_config) -> None:
    setup_c6(set_core_config)
    with pytest.raises(cv.Invalid):
        validate(isotp(not_a_real_key=1))


@pytest.mark.parametrize("mode", ["normal", "extended"])
def test_addressing_mode_accepted(set_core_config, mode: str) -> None:
    setup_c6(set_core_config)
    extra = {"target_address": 0x13} if mode == "extended" else {}
    assert validate(isotp(addressing=mode, **extra))["addressing"] == mode


def test_unknown_addressing_mode_rejected(set_core_config) -> None:
    setup_c6(set_core_config)
    with pytest.raises(cv.Invalid):
        validate(isotp(addressing="mixed"))


# -------------------------------------------------------------------- defaults


def test_defaults(set_core_config) -> None:
    setup_c6(set_core_config)
    validated = validate(isotp())
    assert validated["max_message_size"] == 256
    assert validated["block_size"] == 8
    assert validated["st_min"] == 0
    assert validated["use_extended_id"] is False


@pytest.mark.parametrize("key", ["n_bs_timeout", "n_cr_timeout", "tx_stall_timeout"])
def test_timeout_defaults_are_1000ms(set_core_config, key: str) -> None:
    setup_c6(set_core_config)
    assert validate(isotp())[key].total_milliseconds == 1000


@pytest.mark.parametrize("key", ["n_bs_timeout", "n_cr_timeout", "tx_stall_timeout"])
def test_timeouts_overridable(set_core_config, key: str) -> None:
    setup_c6(set_core_config)
    assert validate(isotp(**{key: "2500ms"}))[key].total_milliseconds == 2500
