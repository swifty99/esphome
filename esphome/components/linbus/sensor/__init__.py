"""LIN Bus sensor platform."""

import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_PERCENT,
)

from .. import CONF_LINBUS_ID, LinBusComponent, linbus_ns

DEPENDENCIES = ["linbus"]

LinBusSensor = linbus_ns.class_("LinBusSensor", sensor.Sensor, cg.Component)

# Sensor types
CONF_MASTER_REQUESTS = "master_requests"
CONF_ID_REQUESTS_ANSWERED = "id_requests_answered"
CONF_DATA_BYTES_RECEIVED = "data_bytes_received"
CONF_CHECKSUM_ERRORS = "checksum_errors"
CONF_SEND_FAILURES = "send_failures"
CONF_BUS_LOAD = "bus_load"
CONF_ERROR_RATE = "error_rate"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_LINBUS_ID): cv.use_id(LinBusComponent),
        cv.Optional(CONF_MASTER_REQUESTS): sensor.sensor_schema(
            unit_of_measurement="requests",
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
        cv.Optional(CONF_ID_REQUESTS_ANSWERED): sensor.sensor_schema(
            unit_of_measurement="requests",
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
        cv.Optional(CONF_DATA_BYTES_RECEIVED): sensor.sensor_schema(
            unit_of_measurement="bytes",
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
        cv.Optional(CONF_CHECKSUM_ERRORS): sensor.sensor_schema(
            unit_of_measurement="errors",
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
        cv.Optional(CONF_SEND_FAILURES): sensor.sensor_schema(
            unit_of_measurement="failures",
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
        cv.Optional(CONF_BUS_LOAD): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ERROR_RATE): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=2,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    }
)


async def to_code(config):
    """Generate code for LIN bus sensors."""
    parent = await cg.get_variable(config[CONF_LINBUS_ID])

    if conf := config.get(CONF_MASTER_REQUESTS):
        sens = await sensor.new_sensor(conf)
        cg.add(parent.set_master_requests_sensor(sens))

    if conf := config.get(CONF_ID_REQUESTS_ANSWERED):
        sens = await sensor.new_sensor(conf)
        cg.add(parent.set_id_requests_answered_sensor(sens))

    if conf := config.get(CONF_DATA_BYTES_RECEIVED):
        sens = await sensor.new_sensor(conf)
        cg.add(parent.set_data_bytes_received_sensor(sens))

    if conf := config.get(CONF_CHECKSUM_ERRORS):
        sens = await sensor.new_sensor(conf)
        cg.add(parent.set_checksum_errors_sensor(sens))

    if conf := config.get(CONF_SEND_FAILURES):
        sens = await sensor.new_sensor(conf)
        cg.add(parent.set_send_failures_sensor(sens))

    if conf := config.get(CONF_BUS_LOAD):
        sens = await sensor.new_sensor(conf)
        cg.add(parent.set_bus_load_sensor(sens))

    if conf := config.get(CONF_ERROR_RATE):
        sens = await sensor.new_sensor(conf)
        cg.add(parent.set_error_rate_sensor(sens))
