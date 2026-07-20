"""can_gateway sensors: route/port diagnostic counters and gauges, plus v0.6
signal-decode sensors (a numeric field extracted from a message)."""

from __future__ import annotations

import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_LENGTH,
    CONF_OFFSET,
    CONF_THROTTLE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_PERCENT,
)

from . import (
    BYTE_ORDER_BIG,
    BYTE_ORDER_LITTLE,
    CONF_BIT_LENGTH,
    CONF_BIT_OFFSET,
    CONF_BYTE_ORDER,
    CONF_CAN_ID,
    CONF_PORT_ID,
    CONF_ROUTE_ID,
    CONF_SIGNED,
    CONF_USE_EXTENDED_ID,
    GatewayPort,
    GatewayRoute,
    can_gateway_ns,
    decode_source_schema,
    validate_decode_id,
    validate_signal_position,
)

CanGatewaySensorHub = can_gateway_ns.class_("CanGatewaySensorHub", cg.PollingComponent)
CanGatewayDecodeSensor = can_gateway_ns.class_("CanGatewayDecodeSensor", sensor.Sensor)

# Order defines the C++ `kind` index passed to set_counter_sensor(); it must
# match the KindIndex enum in can_gateway.h exactly. Append only — inserting
# would silently shift every later sensor onto the wrong counter. The order
# is locked by a test in tests/component_tests/can_gateway/test_entities.py.
ROUTE_COUNTERS = ("forwarded", "filtered", "tx_full", "bus_off", "disabled")
PORT_COUNTERS = ("injected", "tx_fail", "bus_err", "recoveries")
PORT_GAUGES = ("tec", "rec")
# Statistics gauges compile in the ISR bit accounting; zero cost when
# no such sensor is configured.
PORT_STATS = ("bus_load",)
# Observation counters (v0.6): appended last to keep the enum order stable.
PORT_OBSERVE = ("observed", "observe_overflow")
ALL_KINDS = ROUTE_COUNTERS + PORT_COUNTERS + PORT_GAUGES + PORT_STATS + PORT_OBSERVE


def _counter_schema():
    return sensor.sensor_schema(
        state_class=STATE_CLASS_TOTAL_INCREASING,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        accuracy_decimals=0,
        icon="mdi:counter",
    )


def _gauge_schema():
    return sensor.sensor_schema(
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        accuracy_decimals=0,
        icon="mdi:alert-circle-outline",
    )


def _percent_schema():
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_PERCENT,
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        accuracy_decimals=1,
        icon="mdi:gauge",
    )


def _validate_kinds(config):
    """V8: sub-sensors must match the owner kind (route vs port)."""
    configured = [kind for kind in ALL_KINDS if kind in config]
    if not configured:
        raise cv.Invalid("configure at least one counter or gauge sensor")
    if CONF_ROUTE_ID in config:
        for kind in configured:
            if kind not in ROUTE_COUNTERS:
                raise cv.Invalid(
                    f"'{kind}' is a port sensor; use port_id instead of route_id"
                )
    else:
        for kind in configured:
            if kind in ROUTE_COUNTERS:
                raise cv.Invalid(
                    f"'{kind}' is a route sensor; use route_id instead of port_id"
                )
    return config


DIAGNOSTIC_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(CanGatewaySensorHub),
            cv.Optional(CONF_ROUTE_ID): cv.use_id(GatewayRoute),
            cv.Optional(CONF_PORT_ID): cv.use_id(GatewayPort),
            **{cv.Optional(kind): _counter_schema() for kind in ROUTE_COUNTERS},
            **{cv.Optional(kind): _counter_schema() for kind in PORT_COUNTERS},
            **{cv.Optional(kind): _gauge_schema() for kind in PORT_GAUGES},
            **{cv.Optional(kind): _percent_schema() for kind in PORT_STATS},
            **{cv.Optional(kind): _counter_schema() for kind in PORT_OBSERVE},
        }
    ).extend(cv.polling_component_schema("60s")),
    cv.has_exactly_one_key(CONF_ROUTE_ID, CONF_PORT_ID),
    _validate_kinds,
)

# Decode sensor (v0.6, S9): a numeric signal named by position in a message.
DECODE_SCHEMA = cv.All(
    sensor.sensor_schema(CanGatewayDecodeSensor).extend(
        {
            **decode_source_schema(),
            cv.Optional(CONF_OFFSET): cv.int_range(min=0, max=7),
            cv.Optional(CONF_LENGTH): cv.int_range(min=1, max=8),
            cv.Optional(CONF_BIT_OFFSET): cv.int_range(min=0, max=63),
            cv.Optional(CONF_BIT_LENGTH): cv.int_range(min=1, max=32),
            cv.Optional(CONF_BYTE_ORDER, default=BYTE_ORDER_LITTLE): cv.one_of(
                BYTE_ORDER_LITTLE, BYTE_ORDER_BIG, lower=True
            ),
            cv.Optional(CONF_SIGNED, default=False): cv.boolean,
            cv.Optional(CONF_THROTTLE): cv.positive_time_period_milliseconds,
        }
    ),
    validate_decode_id,
    validate_signal_position,
)


def CONFIG_SCHEMA(config):
    # A decode entry names a can_id; a diagnostic entry names a route/port and
    # its counters. Dispatch on that discriminator (V8 stays on the diagnostic).
    if isinstance(config, dict) and CONF_CAN_ID in config:
        return DECODE_SCHEMA(config)
    return DIAGNOSTIC_SCHEMA(config)


async def _decode_to_code(config):
    cg.add_define("USE_CAN_GATEWAY_OBSERVE")
    var = await sensor.new_sensor(config)
    port = await cg.get_variable(config[CONF_PORT_ID])
    bit_mode = CONF_BIT_OFFSET in config
    if bit_mode:
        offset = config[CONF_BIT_OFFSET]
        length = config[CONF_BIT_LENGTH]
    else:
        offset = config[CONF_OFFSET]
        length = config[CONF_LENGTH]
    big_endian = config[CONF_BYTE_ORDER] == BYTE_ORDER_BIG
    cg.add(var.set_signal(offset, length, bit_mode, big_endian, config[CONF_SIGNED]))
    if (throttle := config.get(CONF_THROTTLE)) is not None:
        cg.add(var.set_throttle(throttle.total_milliseconds))
    cg.add(
        port.subscribe_consumer(config[CONF_CAN_ID], config[CONF_USE_EXTENDED_ID], var)
    )


async def _diagnostic_to_code(config):
    hub = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(hub, config)
    if (route_id := config.get(CONF_ROUTE_ID)) is not None:
        cg.add(hub.set_route(await cg.get_variable(route_id)))
    if (port_id := config.get(CONF_PORT_ID)) is not None:
        cg.add(hub.set_port(await cg.get_variable(port_id)))
    for kind_index, kind in enumerate(ALL_KINDS):
        if (conf := config.get(kind)) is not None:
            if kind in PORT_STATS:
                # Compiles the ISR-side bit accounting in; without any
                # statistics gauge the fast path carries zero cost.
                cg.add_define("USE_CAN_GATEWAY_STATS")
            sens = await sensor.new_sensor(conf)
            cg.add(hub.set_counter_sensor(kind_index, sens))


async def to_code(config):
    if CONF_CAN_ID in config:
        await _decode_to_code(config)
    else:
        await _diagnostic_to_code(config)
