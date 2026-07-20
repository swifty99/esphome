"""V5: the can_gateway port's observation ring must hold a whole granted block.

This rule lives in FINAL_VALIDATE_SCHEMA rather than CONFIG_SCHEMA because it needs the
can_gateway port's config, not just the isotp block. It is worth testing carefully: the failure
it prevents is silently dropped consecutive frames, which surfaces much later as an intermittent
N_Cr timeout with nothing obvious pointing back at the ring.
"""

from __future__ import annotations

import pytest

import esphome.config_validation as cv
import esphome.final_validate as fv


def _full_config(observe_queue_depth: int) -> dict:
    """A minimal stand-in for the resolved full config, with one can_gateway port."""
    return {
        "can_gateway": {
            "id": "diag",
            "ports": [
                {
                    "id": "diag_bus",
                    "rx_pin": "GPIO3",
                    "tx_pin": "GPIO2",
                    "bit_rate": 500000,
                    "observe_queue_depth": observe_queue_depth,
                }
            ],
        }
    }


def _run_final_validate(isotp_config: dict, observe_queue_depth: int):
    """Invoke the component's final validation with a populated full-config context."""
    from esphome.components.isotp import _final_validate

    token = fv.full_config.set(_full_config(observe_queue_depth))
    try:
        return _final_validate(isotp_config)
    finally:
        fv.full_config.reset(token)


def _isotp_config(block_size: int) -> dict:
    return {"port_id": "diag_bus", "block_size": block_size}


# block_size + BLOCK_SIZE_RING_MARGIN (4) is the requirement, so block_size 8 needs depth 12.
@pytest.mark.parametrize(
    ("block_size", "depth"),
    [
        (8, 12),  # exactly the requirement
        (8, 32),  # the can_gateway default, comfortably above
        (1, 5),  # smallest legal block
        (28, 32),  # largest block the default ring can carry
    ],
)
def test_ring_depth_sufficient_accepted(block_size: int, depth: int) -> None:
    config = _isotp_config(block_size)
    assert _run_final_validate(config, depth) is config


@pytest.mark.parametrize(
    ("block_size", "depth"),
    [
        (8, 11),  # one short of the requirement
        (16, 12),
        (29, 32),  # one past what the default ring can carry
        (255, 32),  # a large block against a default port
    ],
)
def test_ring_depth_insufficient_rejected(block_size: int, depth: int) -> None:
    with pytest.raises(cv.Invalid, match="observe_queue_depth"):
        _run_final_validate(_isotp_config(block_size), depth)


def test_rejection_names_both_numbers() -> None:
    """The message has to be actionable: it should say what the ring is and what it needs."""
    with pytest.raises(cv.Invalid) as excinfo:
        _run_final_validate(_isotp_config(16), 12)
    message = str(excinfo.value)
    assert "16" in message  # the block size the user asked for
    assert "20" in message  # what the ring must therefore be
    assert "12" in message  # what it currently is


def test_missing_gateway_config_is_not_an_error() -> None:
    """An unexpected config shape is can_gateway's to report, not ours to crash on."""
    from esphome.components.isotp import _final_validate

    token = fv.full_config.set({})
    try:
        config = _isotp_config(8)
        assert _final_validate(config) is config
    finally:
        fv.full_config.reset(token)


def test_unknown_port_id_is_not_an_error() -> None:
    """port_id already resolved at schema time, so a miss here means a shape we do not model."""
    config = {"port_id": "some_other_bus", "block_size": 8}
    assert _run_final_validate(config, 32) is config
