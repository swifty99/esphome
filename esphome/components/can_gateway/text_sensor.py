"""can_gateway text sensors: last-RX-frame snapshot (debug), plus v0.6 enum
signal decode (a byte-aligned field mapped raw integer -> string)."""

from __future__ import annotations

import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_LENGTH,
    CONF_OFFSET,
    CONF_THROTTLE,
    ENTITY_CATEGORY_DIAGNOSTIC,
)
from esphome.core import ID

from . import (
    BYTE_ORDER_BIG,
    BYTE_ORDER_LITTLE,
    CONF_BYTE_ORDER,
    CONF_CAN_GATEWAY_ID,
    CONF_CAN_ID,
    CONF_MAP,
    CONF_PORT_ID,
    CONF_USE_EXTENDED_ID,
    CanGateway,
    GatewayPort,
    can_gateway_ns,
    decode_source_schema,
    validate_decode_id,
    validate_signal_position,
)

CONF_LAST_FRAME = "last_frame"

CanGatewayDecodeTextSensor = can_gateway_ns.class_(
    "CanGatewayDecodeTextSensor", text_sensor.TextSensor
)
CanGatewayEnumEntry = can_gateway_ns.struct("CanGatewayEnumEntry")


def _validate_map(value):
    """V25: raw integer -> string map, keys unique and in range."""
    if not isinstance(value, dict) or not value:
        raise cv.Invalid("map must be a non-empty mapping of raw integer -> string")
    result: dict[int, str] = {}
    for raw_key, raw_text in value.items():
        key = cv.int_range(min=0, max=0xFFFFFFFF)(raw_key)
        if key in result:
            raise cv.Invalid(f"duplicate map key {key}")
        result[key] = cv.string(raw_text)
    return result


LAST_FRAME_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_CAN_GATEWAY_ID): cv.use_id(CanGateway),
        cv.Required(CONF_PORT_ID): cv.use_id(GatewayPort),
        cv.Optional(CONF_THROTTLE, default="1s"): cv.positive_time_period_milliseconds,
        cv.Required(CONF_LAST_FRAME): text_sensor.text_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    }
)

# Decode text sensor (v0.6): a byte-aligned field mapped to a string.
DECODE_SCHEMA = cv.All(
    text_sensor.text_sensor_schema(CanGatewayDecodeTextSensor).extend(
        {
            **decode_source_schema(),
            cv.Required(CONF_OFFSET): cv.int_range(min=0, max=7),
            cv.Required(CONF_LENGTH): cv.int_range(min=1, max=8),
            cv.Optional(CONF_BYTE_ORDER, default=BYTE_ORDER_LITTLE): cv.one_of(
                BYTE_ORDER_LITTLE, BYTE_ORDER_BIG, lower=True
            ),
            cv.Required(CONF_MAP): _validate_map,
            cv.Optional(CONF_THROTTLE): cv.positive_time_period_milliseconds,
        }
    ),
    validate_decode_id,
    validate_signal_position,
)


def CONFIG_SCHEMA(config):
    if isinstance(config, dict) and CONF_CAN_ID in config:
        return DECODE_SCHEMA(config)
    return LAST_FRAME_SCHEMA(config)


def _c_string(text: str) -> str:
    escaped = text.replace("\\", "\\\\").replace('"', '\\"')
    return f'"{escaped}"'


async def _decode_to_code(config):
    cg.add_define("USE_CAN_GATEWAY_OBSERVE")
    var = await text_sensor.new_text_sensor(config)
    port = await cg.get_variable(config[CONF_PORT_ID])
    big_endian = config[CONF_BYTE_ORDER] == BYTE_ORDER_BIG
    cg.add(var.set_signal(config[CONF_OFFSET], config[CONF_LENGTH], big_endian))
    entries = [
        cg.RawExpression(f"{{{key}, {_c_string(text)}}}")
        for key, text in config[CONF_MAP].items()
    ]
    arr_id = ID(
        f"{config[CONF_ID]}_map",
        is_declaration=True,
        type=CanGatewayEnumEntry,
    )
    arr = cg.static_const_array(arr_id, cg.ArrayInitializer(*entries))
    cg.add(var.set_map(arr, len(entries)))
    if (throttle := config.get(CONF_THROTTLE)) is not None:
        cg.add(var.set_throttle(throttle.total_milliseconds))
    cg.add(
        port.subscribe_consumer(config[CONF_CAN_ID], config[CONF_USE_EXTENDED_ID], var)
    )


async def to_code(config):
    if CONF_CAN_ID in config:
        await _decode_to_code(config)
        return
    # Compiles the ISR-side snapshot path in: without any last_frame sensor the
    # fast path carries zero snapshot cost.
    cg.add_define("USE_CAN_GATEWAY_SNAPSHOT")
    gateway = await cg.get_variable(config[CONF_CAN_GATEWAY_ID])
    port = await cg.get_variable(config[CONF_PORT_ID])
    sens = await text_sensor.new_text_sensor(config[CONF_LAST_FRAME])
    cg.add(
        gateway.set_last_frame_text_sensor(
            port, sens, config[CONF_THROTTLE].total_milliseconds
        )
    )
