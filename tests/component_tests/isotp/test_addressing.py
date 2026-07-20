"""isotp addressing tests (V1, V3, V9).

V1  tx_id and rx_id must differ.
V3  normal_fixed takes source/target and derives the identifiers; normal and
    extended take raw tx_id/rx_id; extended additionally requires target_address.
V9  Identifier width must match the addressing mode.
"""

from __future__ import annotations

import pytest

from esphome import config_validation as cv

from .common import isotp, normal_fixed, setup_c6, validate

# ------------------------------------------------------------------- V3 normal


def test_v3_normal_accepts_raw_identifiers(set_core_config) -> None:
    setup_c6(set_core_config)
    validated = validate(isotp())
    assert validated["tx_id"] == 0x7E0
    assert validated["rx_id"] == 0x7E8


def test_v3_normal_is_the_default_addressing(set_core_config) -> None:
    setup_c6(set_core_config)
    assert validate(isotp())["addressing"] == "normal"


def test_v3_normal_rejects_source(set_core_config) -> None:
    setup_c6(set_core_config)
    with pytest.raises(cv.Invalid, match="only used with addressing 'normal_fixed'"):
        validate(isotp(source=0xF1))


def test_v3_normal_rejects_target(set_core_config) -> None:
    setup_c6(set_core_config)
    with pytest.raises(cv.Invalid, match="only used with addressing 'normal_fixed'"):
        validate(isotp(target=0x01))


def test_v3_normal_requires_tx_id(set_core_config) -> None:
    setup_c6(set_core_config)
    with pytest.raises(cv.Invalid, match="'tx_id' is required"):
        validate(isotp(tx_id=None))


def test_v3_normal_requires_rx_id(set_core_config) -> None:
    setup_c6(set_core_config)
    with pytest.raises(cv.Invalid, match="'rx_id' is required"):
        validate(isotp(rx_id=None))


# ------------------------------------------------------------- V3 normal_fixed


def test_v3_normal_fixed_requires_source(set_core_config) -> None:
    setup_c6(set_core_config)
    with pytest.raises(cv.Invalid, match="'source' is required"):
        validate(normal_fixed(source=None))


def test_v3_normal_fixed_requires_target(set_core_config) -> None:
    setup_c6(set_core_config)
    with pytest.raises(cv.Invalid, match="'target' is required"):
        validate(normal_fixed(target=None))


def test_v3_normal_fixed_rejects_tx_id(set_core_config) -> None:
    setup_c6(set_core_config)
    with pytest.raises(cv.Invalid, match="'tx_id' is not used with addressing"):
        validate(normal_fixed(tx_id=0x7E0))


def test_v3_normal_fixed_rejects_rx_id(set_core_config) -> None:
    setup_c6(set_core_config)
    with pytest.raises(cv.Invalid, match="'rx_id' is not used with addressing"):
        validate(normal_fixed(rx_id=0x7E8))


def test_v3_normal_fixed_derives_identifiers(set_core_config) -> None:
    """ISO 15765-2 physical addressing: 0x18DA<target><source>."""
    setup_c6(set_core_config)
    validated = validate(normal_fixed(source=0xF1, target=0x01))
    # Tester (source 0xF1) sends to the ECU (target 0x01).
    assert validated["tx_id"] == 0x18DA01F1
    # The ECU answers with the addresses swapped.
    assert validated["rx_id"] == 0x18DAF101


def test_v3_normal_fixed_forces_extended_id(set_core_config) -> None:
    """The derived identifiers are 29-bit, so the flag is set regardless of input."""
    setup_c6(set_core_config)
    assert validate(normal_fixed())["use_extended_id"] is True
    assert validate(normal_fixed(use_extended_id=False))["use_extended_id"] is True


# ----------------------------------------------------------------- V3 extended


def test_v3_extended_requires_target_address(set_core_config) -> None:
    setup_c6(set_core_config)
    with pytest.raises(cv.Invalid, match="'target_address' is required"):
        validate(isotp(addressing="extended"))


def test_v3_extended_accepts_target_address(set_core_config) -> None:
    setup_c6(set_core_config)
    validated = validate(isotp(addressing="extended", target_address=0x13))
    assert validated["target_address"] == 0x13
    # Extended addressing still carries raw identifiers.
    assert validated["tx_id"] == 0x7E0


def test_v3_normal_rejects_target_address(set_core_config) -> None:
    setup_c6(set_core_config)
    with pytest.raises(
        cv.Invalid, match="'target_address' is only used with addressing 'extended'"
    ):
        validate(isotp(target_address=0x13))


def test_v3_normal_fixed_rejects_target_address(set_core_config) -> None:
    setup_c6(set_core_config)
    with pytest.raises(
        cv.Invalid, match="'target_address' is only used with addressing 'extended'"
    ):
        validate(normal_fixed(target_address=0x13))


# ------------------------------------------------------------------------- V9


def test_v9_eleven_bit_rejects_tx_id_above_7ff(set_core_config) -> None:
    setup_c6(set_core_config)
    with pytest.raises(cv.Invalid, match="does not fit a 11-bit identifier"):
        validate(isotp(tx_id=0x800))


def test_v9_eleven_bit_rejects_rx_id_above_7ff(set_core_config) -> None:
    setup_c6(set_core_config)
    with pytest.raises(cv.Invalid, match="does not fit a 11-bit identifier"):
        validate(isotp(rx_id=0x800))


def test_v9_eleven_bit_accepts_boundary(set_core_config) -> None:
    setup_c6(set_core_config)
    validated = validate(isotp(tx_id=0x7FF, rx_id=0x7FE))
    assert validated["tx_id"] == 0x7FF


def test_v9_same_values_accepted_with_extended_id(set_core_config) -> None:
    """The values rejected as 11-bit are fine once 29-bit identifiers are asked for."""
    setup_c6(set_core_config)
    validated = validate(isotp(tx_id=0x800, rx_id=0x801, use_extended_id=True))
    assert validated["tx_id"] == 0x800
    assert validated["rx_id"] == 0x801


def test_v9_extended_addressing_also_width_checked(set_core_config) -> None:
    setup_c6(set_core_config)
    with pytest.raises(cv.Invalid, match="does not fit a 11-bit identifier"):
        validate(isotp(addressing="extended", target_address=0x13, tx_id=0x800))


# ------------------------------------------------------------------------- V1


def test_v1_normal_rejects_equal_identifiers(set_core_config) -> None:
    setup_c6(set_core_config)
    with pytest.raises(cv.Invalid, match="must differ"):
        validate(isotp(tx_id=0x7E0, rx_id=0x7E0))


def test_v1_extended_rejects_equal_identifiers(set_core_config) -> None:
    setup_c6(set_core_config)
    with pytest.raises(cv.Invalid, match="must differ"):
        validate(isotp(addressing="extended", target_address=0x13, rx_id=0x7E0))


def test_v1_normal_fixed_rejects_equal_source_and_target(set_core_config) -> None:
    """Equal source and target derive the same identifier for both directions."""
    setup_c6(set_core_config)
    with pytest.raises(cv.Invalid, match="must differ"):
        validate(normal_fixed(source=0x01, target=0x01))
