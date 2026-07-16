"""LIN Bus binary sensor platform."""

import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv

from .. import CONF_LINBUS_ID, LinBusComponent

DEPENDENCIES = ["linbus"]

# Binary sensor types
CONF_SELF_TEST_PASS = "self_test_pass"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_LINBUS_ID): cv.use_id(LinBusComponent),
        cv.Optional(CONF_SELF_TEST_PASS): binary_sensor.binary_sensor_schema(),
    }
)


async def to_code(config):
    """Generate code for LIN bus binary sensors."""
    parent = await cg.get_variable(config[CONF_LINBUS_ID])

    if conf := config.get(CONF_SELF_TEST_PASS):
        bs = await binary_sensor.new_binary_sensor(conf)
        cg.add(parent.set_self_test_pass_binary_sensor(bs))
