"""can_gateway binary sensors: port bus-off state, plus v0.6 single-bit
signal decode (one bit of a message published as a binary sensor)."""

from __future__ import annotations

import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.const import DEVICE_CLASS_PROBLEM, ENTITY_CATEGORY_DIAGNOSTIC

from . import (
    CONF_BIT,
    CONF_CAN_GATEWAY_ID,
    CONF_CAN_ID,
    CONF_PORT_ID,
    CONF_USE_EXTENDED_ID,
    CanGateway,
    GatewayPort,
    can_gateway_ns,
    decode_source_schema,
    validate_decode_id,
)

CONF_BUS_OFF = "bus_off"

CanGatewayDecodeBinarySensor = can_gateway_ns.class_(
    "CanGatewayDecodeBinarySensor", binary_sensor.BinarySensor
)

BUS_OFF_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_CAN_GATEWAY_ID): cv.use_id(CanGateway),
        cv.Required(CONF_PORT_ID): cv.use_id(GatewayPort),
        cv.Required(CONF_BUS_OFF): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_PROBLEM,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    }
)

# Decode binary sensor (v0.6): a single bit (byte * 8 + bit) of a message.
DECODE_SCHEMA = cv.All(
    binary_sensor.binary_sensor_schema(CanGatewayDecodeBinarySensor).extend(
        {
            **decode_source_schema(),
            cv.Required(CONF_BIT): cv.int_range(min=0, max=63),
        }
    ),
    validate_decode_id,
)


def CONFIG_SCHEMA(config):
    if isinstance(config, dict) and CONF_CAN_ID in config:
        return DECODE_SCHEMA(config)
    return BUS_OFF_SCHEMA(config)


async def to_code(config):
    if CONF_CAN_ID in config:
        cg.add_define("USE_CAN_GATEWAY_OBSERVE")
        var = await binary_sensor.new_binary_sensor(config)
        port = await cg.get_variable(config[CONF_PORT_ID])
        cg.add(var.set_bit(config[CONF_BIT]))
        cg.add(
            port.subscribe_consumer(
                config[CONF_CAN_ID], config[CONF_USE_EXTENDED_ID], var
            )
        )
        return
    gateway = await cg.get_variable(config[CONF_CAN_GATEWAY_ID])
    port = await cg.get_variable(config[CONF_PORT_ID])
    sens = await binary_sensor.new_binary_sensor(config[CONF_BUS_OFF])
    cg.add(gateway.set_bus_off_binary_sensor(port, sens))
