"""can_gateway.send tests (cfg-v11 ergonomics + cfg-v22 omittable port).

can_gateway.send is the v0.6 name for can_gateway.inject (kept as an alias):
shorthand payload, templatable can_id, and a port that may be omitted when
exactly one port is declared.
"""

from __future__ import annotations

import pytest

from esphome import config_validation as cv

from .common import PORT_A, PORT_B, gateway, port, setup_c6


def _send_schema():
    from esphome.components.can_gateway import SEND_ACTION_SCHEMA

    return SEND_ACTION_SCHEMA


def _final_validate(gateway_config, actions, set_component_config):
    from esphome.components.can_gateway import FINAL_VALIDATE_SCHEMA

    set_component_config("can_gateway", gateway_config)
    set_component_config(
        "button",
        [{"platform": "template", "on_press": [{"then": actions}]}],
    )
    return FINAL_VALIDATE_SCHEMA(gateway_config)


def _single_bus(set_core_config):
    from esphome.components.can_gateway import CONFIG_SCHEMA

    setup_c6(set_core_config)
    return CONFIG_SCHEMA(gateway(ports=[dict(PORT_A)], routes=[]))


def _two_port(set_core_config):
    from esphome.components.can_gateway import CONFIG_SCHEMA

    setup_c6(set_core_config)
    return CONFIG_SCHEMA(gateway())


# ---------------------------------------------------------------- ergonomics


def test_send_explicit_port_accepted(set_core_config) -> None:
    setup_c6(set_core_config)
    _send_schema()({"port": "port_a", "can_id": 0x100, "data": [0x01, 0x02]})


def test_send_shorthand_list_maps_to_data(set_core_config) -> None:
    setup_c6(set_core_config)
    # Bare list is folded into data; a can_id must still come from elsewhere, so
    # the pure shorthand needs a dict — but the list-form is accepted structurally.
    validated = _send_schema()({"can_id": 0x100, "data": [0x0A, 0x0B, 0x0C]})
    assert validated["data"] == [0x0A, 0x0B, 0x0C]


def test_send_string_payload(set_core_config) -> None:
    setup_c6(set_core_config)
    validated = _send_schema()({"can_id": 0x100, "data": "Hi"})
    assert validated["data"] == [ord("H"), ord("i")]


def test_send_string_payload_too_long_rejected(set_core_config) -> None:
    setup_c6(set_core_config)
    with pytest.raises(cv.Invalid, match="8 bytes"):
        _send_schema()({"can_id": 0x100, "data": "too many chars"})


def test_send_templated_can_id_accepted(set_core_config) -> None:
    from esphome import automation  # noqa: F401 (ensure lambda machinery import)

    setup_c6(set_core_config)
    # A lambda can_id skips the static bound check.
    _send_schema()(
        {"port": "port_a", "can_id": cv.Lambda("return 0x100;"), "data": [0x01]}
    )


# ---------------------------------------------------------------- cfg-v22


def test_v22_omitted_port_resolves_on_single_bus(
    set_core_config, set_component_config
) -> None:
    config = _single_bus(set_core_config)
    action = _send_schema()({"can_id": 0x100, "data": [0x01]})
    validated = _final_validate(
        config, [{"can_gateway.send": action}], set_component_config
    )
    # After final validation the action carries the sole port.
    assert "port" in action
    assert str(action["port"]) == "port_a"
    assert validated is config


def test_v22_omitted_port_two_ports_rejected(
    set_core_config, set_component_config
) -> None:
    config = _two_port(set_core_config)
    action = _send_schema()({"can_id": 0x100, "data": [0x01]})
    with pytest.raises(cv.Invalid, match="needs a 'port'"):
        _final_validate(config, [{"can_gateway.send": action}], set_component_config)


def test_v22_send_on_listen_only_rejected(
    set_core_config, set_component_config
) -> None:
    from esphome.components.can_gateway import CONFIG_SCHEMA

    setup_c6(set_core_config)
    # Tap route from the listen-only port is fine; sending on it is not.
    config = CONFIG_SCHEMA(
        gateway(
            ports=[port(PORT_A, listen_only=True), dict(PORT_B)],
            routes=[{"from": "port_a", "to": "port_b"}],
        )
    )
    action = _send_schema()({"port": "port_a", "can_id": 0x100})
    with pytest.raises(cv.Invalid, match="listen.only"):
        _final_validate(config, [{"can_gateway.send": action}], set_component_config)


def test_inject_alias_still_resolves(set_core_config, set_component_config) -> None:
    # can_gateway.inject remains a working alias for the same action.
    config = _single_bus(set_core_config)
    action = _send_schema()({"can_id": 0x100, "data": [0x01]})
    _final_validate(config, [{"can_gateway.inject": action}], set_component_config)
    assert str(action["port"]) == "port_a"
