"""Single-bus / v0.6 topology tests: cfg-v17 (port count), cfg-v18 (routes
require two ports), cfg-v21 (enable switch needs a route), cfg-v24 (bus_load on
a route-less port).
"""

from __future__ import annotations

import pytest

from esphome import config_validation as cv

from .common import PORT_A, PORT_B, gateway, port, setup_c6, validate


def _final_validate(gateway_config, set_component_config, **platforms):
    """Run the component final-validate with extra platforms planted."""
    from esphome.components.can_gateway import FINAL_VALIDATE_SCHEMA

    set_component_config("can_gateway", gateway_config)
    for name, value in platforms.items():
        set_component_config(name, value)
    return FINAL_VALIDATE_SCHEMA(gateway_config)


# ---------------------------------------------------------------- cfg-v17


def test_v17_single_port_accepted(set_core_config) -> None:
    setup_c6(set_core_config)
    validated = validate(gateway(ports=[dict(PORT_A)], routes=[]))
    assert len(validated["ports"]) == 1
    assert "routes" not in validated


def test_v17_zero_ports_rejected(set_core_config) -> None:
    setup_c6(set_core_config)
    with pytest.raises(cv.Invalid):
        validate({"ports": [], "routes": []})


def test_v17_three_ports_rejected(set_core_config) -> None:
    setup_c6(set_core_config)
    third = port(PORT_A, id="port_c", rx_pin="GPIO5", tx_pin="GPIO4")
    with pytest.raises(cv.Invalid, match="1 or 2 ports"):
        validate(gateway(ports=[dict(PORT_A), dict(PORT_B), third]))


# ---------------------------------------------------------------- cfg-v18


def test_v18_single_port_with_route_rejected(set_core_config) -> None:
    setup_c6(set_core_config)
    with pytest.raises(cv.Invalid, match="routes require two ports"):
        validate(
            gateway(ports=[dict(PORT_A)], routes=[{"from": "port_a", "to": "port_a"}])
        )


def test_v18_two_ports_no_routes_accepted(set_core_config) -> None:
    setup_c6(set_core_config)
    validate(gateway(routes=[]))


# ---------------------------------------------------------------- cfg-v23


def test_v23_max_frames_per_loop_defaults_to_depth(set_core_config) -> None:
    setup_c6(set_core_config)
    validated = validate(
        gateway(ports=[port(PORT_A, observe_queue_depth=16)], routes=[])
    )
    # Omitted max_frames_per_loop defaults to the ring depth.
    assert validated["ports"][0]["max_frames_per_loop"] == 16


def test_v23_max_frames_per_loop_over_depth_rejected(set_core_config) -> None:
    setup_c6(set_core_config)
    with pytest.raises(cv.Invalid, match="cannot exceed"):
        validate(
            gateway(
                ports=[port(PORT_A, observe_queue_depth=16, max_frames_per_loop=32)],
                routes=[],
            )
        )


# ---------------------------------------------------------------- cfg-v21


def test_v21_switch_on_single_bus_rejected(
    set_core_config, set_component_config
) -> None:
    setup_c6(set_core_config)
    config = validate(gateway(ports=[dict(PORT_A)], routes=[]))
    with pytest.raises(cv.Invalid, match="no routes to gate"):
        _final_validate(
            config,
            set_component_config,
            switch=[{"platform": "can_gateway", "name": "enable"}],
        )


def test_v21_switch_on_bridge_accepted(set_core_config, set_component_config) -> None:
    setup_c6(set_core_config)
    config = validate(gateway())
    _final_validate(
        config,
        set_component_config,
        switch=[{"platform": "can_gateway", "name": "enable"}],
    )


# ---------------------------------------------------------------- cfg-v24


def test_v24_bus_load_on_single_bus_port_accepted(
    set_core_config, set_component_config
) -> None:
    # In single-bus mode the port is the only bus; its load is exactly what the
    # user wants (was rejected pre-v0.6).
    setup_c6(set_core_config)
    config = validate(gateway(ports=[dict(PORT_A)], routes=[]))
    _final_validate(
        config,
        set_component_config,
        sensor=[
            {
                "platform": "can_gateway",
                "port_id": "port_a",
                "bus_load": {"name": "load"},
            }
        ],
    )


def test_v24_bus_load_on_route_source_accepted(
    set_core_config, set_component_config
) -> None:
    setup_c6(set_core_config)
    config = validate(gateway())  # route port_a -> port_b
    _final_validate(
        config,
        set_component_config,
        sensor=[
            {
                "platform": "can_gateway",
                "port_id": "port_a",
                "bus_load": {"name": "load"},
            }
        ],
    )


def test_v24_bus_load_on_pure_destination_rejected(
    set_core_config, set_component_config
) -> None:
    # port_b is only a route destination (a -> b), never a source: it does not
    # receive through the gateway, so bus_load would be misleading.
    setup_c6(set_core_config)
    config = validate(gateway())
    with pytest.raises(cv.Invalid, match="route destination only"):
        _final_validate(
            config,
            set_component_config,
            sensor=[
                {
                    "platform": "can_gateway",
                    "port_id": "port_b",
                    "bus_load": {"name": "load"},
                }
            ],
        )
