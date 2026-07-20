"""Signal-decode schema tests (cfg-decode, cfg-v19/v20/v25).

Decode entries live on the sensor / binary_sensor / text_sensor platforms and
are told apart from the diagnostic entries by carrying a `can_id`.
"""

from __future__ import annotations

import pytest

from esphome import config_validation as cv

from .common import PORT_A, gateway, setup_c6


def _decode_gateway(set_core_config):
    """A validated single-port gateway registering port 'port_a'."""
    from esphome.components.can_gateway import CONFIG_SCHEMA

    setup_c6(set_core_config)
    CONFIG_SCHEMA(gateway(ports=[dict(PORT_A)], routes=[]))


def _sensor(config):
    from esphome.components.can_gateway.sensor import CONFIG_SCHEMA

    return CONFIG_SCHEMA(config)


def _binary_sensor(config):
    from esphome.components.can_gateway.binary_sensor import CONFIG_SCHEMA

    return CONFIG_SCHEMA(config)


def _text_sensor(config):
    from esphome.components.can_gateway.text_sensor import CONFIG_SCHEMA

    return CONFIG_SCHEMA(config)


_NAME_COUNTER = [0]


def _decode(**extra):
    # Unique name per call: ESPHome rejects duplicate entity names within a run.
    _NAME_COUNTER[0] += 1
    base = {"port_id": "port_a", "can_id": 0x2A0, "name": f"Signal {_NAME_COUNTER[0]}"}
    base.update(extra)
    return base


# ---------------------------------------------------------------- cfg-decode


def test_byte_aligned_accepted(set_core_config) -> None:
    _decode_gateway(set_core_config)
    validated = _sensor(_decode(offset=2, length=2, byte_order="big"))
    assert validated["offset"] == 2
    assert validated["length"] == 2
    assert validated["byte_order"] == "big"
    assert validated["signed"] is False


def test_bit_level_accepted(set_core_config) -> None:
    _decode_gateway(set_core_config)
    validated = _sensor(_decode(bit_offset=24, bit_length=4))
    assert validated["bit_offset"] == 24
    assert validated["bit_length"] == 4


def test_signed_accepted(set_core_config) -> None:
    _decode_gateway(set_core_config)
    validated = _sensor(_decode(offset=0, length=1, signed=True))
    assert validated["signed"] is True


def test_binary_sensor_bit_accepted(set_core_config) -> None:
    _decode_gateway(set_core_config)
    validated = _binary_sensor(
        {"port_id": "port_a", "can_id": 0x3C0, "bit": 5, "name": "Brake"}
    )
    assert validated["bit"] == 5


def test_text_sensor_map_accepted(set_core_config) -> None:
    _decode_gateway(set_core_config)
    validated = _text_sensor(
        {
            "port_id": "port_a",
            "can_id": 0x1A0,
            "offset": 0,
            "length": 1,
            "map": {0: "Park", 1: "Reverse"},
            "name": "Gear",
        }
    )
    assert validated["map"] == {0: "Park", 1: "Reverse"}


# ---------------------------------------------------------------- cfg-v19


def test_v19_can_id_fits_frame_type(set_core_config) -> None:
    _decode_gateway(set_core_config)
    with pytest.raises(cv.Invalid, match="0x7FF"):
        _sensor(_decode(can_id=0x800, offset=0, length=1))
    # Extended is fine when use_extended_id is set.
    _sensor(_decode(can_id=0x18DAF110, use_extended_id=True, offset=0, length=1))


# ---------------------------------------------------------------- cfg-v20


def test_v20_byte_signal_past_frame_rejected(set_core_config) -> None:
    _decode_gateway(set_core_config)
    with pytest.raises(cv.Invalid, match="past the 8-byte frame"):
        _sensor(_decode(offset=6, length=4))


def test_v20_byte_signal_over_32_bits_rejected(set_core_config) -> None:
    _decode_gateway(set_core_config)
    with pytest.raises(cv.Invalid, match="32 bits"):
        _sensor(_decode(offset=0, length=5))


def test_v20_bit_signal_past_frame_rejected(set_core_config) -> None:
    _decode_gateway(set_core_config)
    with pytest.raises(cv.Invalid, match="past the 64-bit frame"):
        _sensor(_decode(bit_offset=40, bit_length=32))


def test_v20_both_forms_rejected(set_core_config) -> None:
    _decode_gateway(set_core_config)
    with pytest.raises(cv.Invalid, match="not both"):
        _sensor(_decode(offset=0, length=2, bit_offset=0, bit_length=4))


def test_v20_no_position_rejected(set_core_config) -> None:
    _decode_gateway(set_core_config)
    with pytest.raises(cv.Invalid, match="needs a position"):
        _sensor(_decode())


def test_v20_partial_byte_form_rejected(set_core_config) -> None:
    _decode_gateway(set_core_config)
    with pytest.raises(cv.Invalid, match="both offset and length"):
        _sensor(_decode(offset=2))


def test_v20_big_endian_bit_level_rejected(set_core_config) -> None:
    _decode_gateway(set_core_config)
    with pytest.raises(cv.Invalid, match="bit-level"):
        _sensor(_decode(bit_offset=0, bit_length=4, byte_order="big"))


# ---------------------------------------------------------------- cfg-v25


def test_v25_throttle_accepted(set_core_config) -> None:
    _decode_gateway(set_core_config)
    validated = _sensor(_decode(offset=0, length=2, throttle="500ms"))
    assert validated["throttle"].total_milliseconds == 500
    # A non-time value is rejected.
    with pytest.raises(cv.Invalid):
        _sensor(_decode(offset=0, length=2, throttle="soon"))


def test_v25_text_map_duplicate_keys_rejected(set_core_config) -> None:
    _decode_gateway(set_core_config)
    # Duplicate keys after normalization (0 and 0x0) are the same integer;
    # a YAML dict collapses them, so uniqueness is inherent. An explicit empty
    # map is rejected.
    with pytest.raises(cv.Invalid):
        _text_sensor(
            {
                "port_id": "port_a",
                "can_id": 0x1A0,
                "offset": 0,
                "length": 1,
                "map": {},
                "name": "Gear",
            }
        )
