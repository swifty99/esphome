"""Optional per-instance diagnostic counters (B14).

One `sensor:` entry declares any subset of the four counters for one isotp instance, backed by a
single polling component. Declaring one sensor per platform entry would work too, but a config
that talks to several ECUs would then carry a PollingComponent per counter per ECU.
"""

import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_TOTAL_INCREASING,
)

from . import IsoTpProtocol, isotp_ns

DEPENDENCIES = ["isotp"]

IsoTpDiagnostics = isotp_ns.class_("IsoTpDiagnostics", cg.PollingComponent)

CONF_ISOTP_ID = "isotp_id"
CONF_MESSAGES_SENT = "messages_sent"
CONF_MESSAGES_RECEIVED = "messages_received"
CONF_TRANSFERS_FAILED = "transfers_failed"
CONF_FRAMES_IGNORED = "frames_ignored"

COUNTERS = (
    CONF_MESSAGES_SENT,
    CONF_MESSAGES_RECEIVED,
    CONF_TRANSFERS_FAILED,
    CONF_FRAMES_IGNORED,
)


def _counter_schema() -> cv.Schema:
    """Counters are monotonic totals, so they all share one entity shape."""
    return sensor.sensor_schema(
        accuracy_decimals=0,
        state_class=STATE_CLASS_TOTAL_INCREASING,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    )


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(IsoTpDiagnostics),
            cv.GenerateID(CONF_ISOTP_ID): cv.use_id(IsoTpProtocol),
            **{cv.Optional(counter): _counter_schema() for counter in COUNTERS},
        }
    ).extend(cv.polling_component_schema("60s")),
    # An entry declaring no counter would silently create a polling component that publishes
    # nothing, which reads as a working config that does not work.
    cv.has_at_least_one_key(*COUNTERS),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cg.register_parented(var, config[CONF_ISOTP_ID])

    for counter in COUNTERS:
        if (conf := config.get(counter)) is not None:
            sens = await sensor.new_sensor(conf)
            cg.add(getattr(var, f"set_{counter}_sensor")(sens))
