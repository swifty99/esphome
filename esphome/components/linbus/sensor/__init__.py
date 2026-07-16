"""LIN Bus sensor platform."""

import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_PERCENT,
)

from .. import CONF_LINBUS_ID, LinBusComponent

DEPENDENCIES = ["linbus"]

# Sensor types
CONF_MASTER_REQUESTS = "master_requests"
CONF_ID_REQUESTS_ANSWERED = "id_requests_answered"
CONF_DATA_BYTES_RECEIVED = "data_bytes_received"
CONF_FRAMES_RECEIVED = "frames_received"
CONF_UART_BREAKS = "uart_breaks"
CONF_CHECKSUM_ERRORS = "checksum_errors"
CONF_CHECKSUM_FALLBACK = "checksum_fallback"
CONF_PID_ERRORS = "pid_errors"
CONF_FRAMING_ERRORS = "framing_errors"
CONF_COLLISIONS = "collisions"
CONF_FRAMES_COALESCED = "frames_coalesced"
CONF_SEND_FAILURES = "send_failures"
CONF_BUS_LOAD = "bus_load"
CONF_ERROR_RATE = "error_rate"


def _count_sensor(unit):
    # Lifetime totals fed to these are monotonic (never window-reset), so TOTAL_INCREASING is
    # correct and they never sawtooth in Home Assistant (H4/N6).
    return sensor.sensor_schema(
        unit_of_measurement=unit,
        accuracy_decimals=0,
        state_class=STATE_CLASS_TOTAL_INCREASING,
    )


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_LINBUS_ID): cv.use_id(LinBusComponent),
        cv.Optional(CONF_MASTER_REQUESTS): _count_sensor("requests"),
        cv.Optional(CONF_ID_REQUESTS_ANSWERED): _count_sensor("requests"),
        cv.Optional(CONF_DATA_BYTES_RECEIVED): _count_sensor("bytes"),
        cv.Optional(CONF_FRAMES_RECEIVED): _count_sensor("frames"),
        cv.Optional(CONF_UART_BREAKS): _count_sensor("breaks"),
        cv.Optional(CONF_CHECKSUM_ERRORS): _count_sensor("errors"),
        cv.Optional(CONF_CHECKSUM_FALLBACK): _count_sensor("frames"),
        cv.Optional(CONF_PID_ERRORS): _count_sensor("errors"),
        cv.Optional(CONF_FRAMING_ERRORS): _count_sensor("errors"),
        cv.Optional(CONF_COLLISIONS): _count_sensor("collisions"),
        cv.Optional(CONF_FRAMES_COALESCED): _count_sensor("frames"),
        cv.Optional(CONF_SEND_FAILURES): _count_sensor("failures"),
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

    if conf := config.get(CONF_FRAMES_RECEIVED):
        sens = await sensor.new_sensor(conf)
        cg.add(parent.set_frames_received_sensor(sens))

    if conf := config.get(CONF_UART_BREAKS):
        sens = await sensor.new_sensor(conf)
        cg.add(parent.set_uart_breaks_sensor(sens))

    if conf := config.get(CONF_CHECKSUM_ERRORS):
        sens = await sensor.new_sensor(conf)
        cg.add(parent.set_checksum_errors_sensor(sens))

    if conf := config.get(CONF_CHECKSUM_FALLBACK):
        sens = await sensor.new_sensor(conf)
        cg.add(parent.set_checksum_fallback_sensor(sens))

    if conf := config.get(CONF_PID_ERRORS):
        sens = await sensor.new_sensor(conf)
        cg.add(parent.set_pid_errors_sensor(sens))

    if conf := config.get(CONF_FRAMING_ERRORS):
        sens = await sensor.new_sensor(conf)
        cg.add(parent.set_framing_errors_sensor(sens))

    if conf := config.get(CONF_COLLISIONS):
        sens = await sensor.new_sensor(conf)
        cg.add(parent.set_collisions_sensor(sens))

    if conf := config.get(CONF_FRAMES_COALESCED):
        sens = await sensor.new_sensor(conf)
        cg.add(parent.set_frames_coalesced_sensor(sens))

    if conf := config.get(CONF_SEND_FAILURES):
        sens = await sensor.new_sensor(conf)
        cg.add(parent.set_send_failures_sensor(sens))

    if conf := config.get(CONF_BUS_LOAD):
        sens = await sensor.new_sensor(conf)
        cg.add(parent.set_bus_load_sensor(sens))

    if conf := config.get(CONF_ERROR_RATE):
        sens = await sensor.new_sensor(conf)
        cg.add(parent.set_error_rate_sensor(sens))
