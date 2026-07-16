"""LIN Bus component for ESPHome."""

from esphome import automation, pins
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_BAUD_RATE, CONF_ID, CONF_MODE, CONF_TRIGGER_ID

DEPENDENCIES = ["esp32"]
AUTO_LOAD = ["sensor", "binary_sensor"]
CODEOWNERS = ["@swifty99"]

# Namespace
linbus_ns = cg.esphome_ns.namespace("linbus")
LinBusComponent = linbus_ns.class_("LinBusComponent", cg.Component)
LinFrameTrigger = linbus_ns.class_(
    "LinFrameTrigger",
    automation.Trigger.template(cg.std_vector.template(cg.uint8), cg.uint8),
)
LinErrorTrigger = linbus_ns.class_(
    "LinErrorTrigger", automation.Trigger.template(cg.std_string)
)
UpdateResponseAction = linbus_ns.class_("UpdateResponseAction", automation.Action)
RunSelfTestAction = linbus_ns.class_("RunSelfTestAction", automation.Action)

# Configuration constants
CONF_CS_PIN = "cs_pin"
CONF_SCHEDULE = "schedule"
CONF_RESPONSES = "responses"
CONF_RX_IDS = "rx_ids"
CONF_LIN_ID = "lin_id"
CONF_INTERVAL = "interval"
CONF_DATA = "data"
CONF_CHECKSUM = "checksum"
CONF_LENGTH = "length"
CONF_SELF_TEST = "self_test"
CONF_ENABLE = "enable"
CONF_UART_PORT = "uart_port"
CONF_TX_PIN = "tx_pin"
CONF_RX_PIN = "rx_pin"
CONF_ON_FRAME = "on_frame"
CONF_ON_ERROR = "on_error"
CONF_LINBUS_ID = "linbus_id"

# A LIN data frame carries 1..8 data bytes (LIN 2.1 §2.2.3). The `length` hint is that data
# length; the decoder finalizes the frame after length+1 bytes (payload + checksum). 0 is not a
# valid LIN frame and doubles as the C++ "unknown length" sentinel, so it is rejected here.
CHECKSUM_ENUM = {"classic": False, "enhanced": True}

# Mode constants
MODE_MASTER = "master"
MODE_SLAVE = "slave"
MODE_LISTENER = "listener"

# LIN 2.1 caps the bus at 20 kbit/s; 0 would divide-by-zero in the driver (UPGRADES.md H6).
UART_PORTS = ["UART_NUM_0", "UART_NUM_1", "UART_NUM_2"]


def _validate_interval(value):
    # 0 ms is the "free slot" sentinel in the C++ scheduler, so an entry with interval: 0ms
    # would silently never be scheduled. Reject it here. See UPGRADES.md H6.
    period = cv.positive_time_period_milliseconds(value)
    if period.total_milliseconds < 1:
        raise cv.Invalid("interval must be at least 1ms")
    return period


# Validation schemas
SCHEDULE_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_LIN_ID): cv.int_range(min=0, max=63),
        cv.Required(CONF_INTERVAL): _validate_interval,
        cv.Optional(CONF_LENGTH): cv.int_range(min=1, max=8),
    }
)

RESPONSE_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_LIN_ID): cv.int_range(min=0, max=63),
        cv.Required(CONF_DATA): cv.All([cv.hex_uint8_t], cv.Length(min=0, max=8)),
        cv.Optional(CONF_CHECKSUM, default="enhanced"): cv.enum(CHECKSUM_ENUM, lower=True),
    }
)


def _validate_rx_id_entry(conf):
    # An rx_ids entry with neither length nor checksum would carry no information (V5).
    if CONF_LENGTH not in conf and CONF_CHECKSUM not in conf:
        raise cv.Invalid("each rx_ids entry needs at least one of 'length' or 'checksum'")
    return conf


# rx_ids: per-received-ID framing/checksum hints for IDs this node does NOT publish (sniffer /
# listener). checksum has no default here — absence means "let the decoder infer / fall back".
RX_ID_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.Required(CONF_LIN_ID): cv.int_range(min=0, max=63),
            cv.Optional(CONF_LENGTH): cv.int_range(min=1, max=8),
            cv.Optional(CONF_CHECKSUM): cv.enum(CHECKSUM_ENUM, lower=True),
        }
    ),
    _validate_rx_id_entry,
)


def _no_duplicate_lin_ids(key):
    # A duplicate lin_id in schedule/responses was silently last-wins in C++ (M9). Reject it.
    def validator(value):
        seen = set()
        for item in value:
            lin_id = item[CONF_LIN_ID]
            if lin_id in seen:
                raise cv.Invalid(f"duplicate lin_id {lin_id} in {key}")
            seen.add(lin_id)
        return value

    return validator


def _validate_mode_config(config):
    mode = config.get(CONF_MODE, MODE_MASTER)
    if CONF_SCHEDULE in config and mode != MODE_MASTER:
        raise cv.Invalid("linbus schedule is only valid with mode: master")
    if CONF_RESPONSES in config and mode == MODE_LISTENER:
        raise cv.Invalid("linbus responses are not valid with mode: listener")
    # The self-test drives the bus against its own looped-back frames, so it needs a master.
    if config.get(CONF_SELF_TEST) and mode != MODE_MASTER:
        raise cv.Invalid("linbus self_test requires mode: master")
    return config


# Main configuration schema
CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(LinBusComponent),
            cv.Required(CONF_TX_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_RX_PIN): pins.gpio_input_pin_schema,
            cv.Optional(CONF_CS_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_BAUD_RATE, default=19200): cv.int_range(
                min=1000, max=20000
            ),
            cv.Optional(CONF_UART_PORT, default="UART_NUM_1"): cv.one_of(
                *UART_PORTS, upper=True
            ),
            cv.Optional(CONF_MODE, default=MODE_MASTER): cv.enum(
                {MODE_MASTER: 0, MODE_SLAVE: 1, MODE_LISTENER: 2}, lower=True
            ),
            cv.Optional(CONF_SCHEDULE): cv.All(
                cv.ensure_list(SCHEDULE_SCHEMA),
                cv.Length(max=16),
                _no_duplicate_lin_ids(CONF_SCHEDULE),
            ),
            cv.Optional(CONF_RESPONSES): cv.All(
                cv.ensure_list(RESPONSE_SCHEMA),
                cv.Length(max=16),
                _no_duplicate_lin_ids(CONF_RESPONSES),
            ),
            cv.Optional(CONF_RX_IDS): cv.All(
                cv.ensure_list(RX_ID_SCHEMA),
                cv.Length(max=32),
                _no_duplicate_lin_ids(CONF_RX_IDS),
            ),
            cv.Optional(CONF_SELF_TEST, default=False): cv.boolean,
            cv.Optional(CONF_ON_FRAME): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(LinFrameTrigger),
                    cv.Required(CONF_LIN_ID): cv.int_range(min=0, max=63),
                    cv.Optional(CONF_LENGTH): cv.int_range(min=1, max=8),
                    cv.Optional(CONF_CHECKSUM): cv.enum(CHECKSUM_ENUM, lower=True),
                }
            ),
            cv.Optional(CONF_ON_ERROR): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(LinErrorTrigger),
                }
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _validate_mode_config,
)


async def to_code(config):
    """Generate C++ code for the LIN bus component."""
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Pin configuration
    tx_pin = await cg.gpio_pin_expression(config[CONF_TX_PIN])
    cg.add(var.set_tx_pin(tx_pin))

    rx_pin = await cg.gpio_pin_expression(config[CONF_RX_PIN])
    cg.add(var.set_rx_pin(rx_pin))

    if CONF_CS_PIN in config:
        cs_pin = await cg.gpio_pin_expression(config[CONF_CS_PIN])
        cg.add(var.set_cs_pin(cs_pin))

    # UART port and baud rate
    cg.add(var.set_uart_port_str(config[CONF_UART_PORT]))
    cg.add(var.set_baud_rate(config[CONF_BAUD_RATE]))

    # Mode
    cg.add(var.set_mode(config[CONF_MODE]))

    # Encode an optional checksum enum as the C++ tri-state: -1 undeclared, 0 classic, 1 enhanced.
    # cv.enum does NOT replace the config value with the mapping result — it returns a
    # string-like object carrying the mapped bool in .enum_value (applied only when the value
    # is rendered by codegen). Truth-testing the object directly is therefore always True
    # ("classic" is a non-empty string) — that bug shipped classic rx_ids/on_frame
    # declarations as enhanced (caught on the bench, HW-4).
    def _checksum_mode(conf):
        if CONF_CHECKSUM not in conf:
            return -1
        enhanced = conf[CONF_CHECKSUM]
        enhanced = getattr(enhanced, "enum_value", enhanced)
        return 1 if enhanced else 0

    # Schedule (master mode)
    if CONF_SCHEDULE in config:
        for item in config[CONF_SCHEDULE]:
            cg.add(
                var.add_schedule_item(
                    item[CONF_LIN_ID], item[CONF_INTERVAL].total_milliseconds
                )
            )
            if CONF_LENGTH in item:
                cg.add(var.add_rx_hint(item[CONF_LIN_ID], item[CONF_LENGTH], -1))

    # Responses
    if CONF_RESPONSES in config:
        for resp in config[CONF_RESPONSES]:
            cg.add(
                var.add_response(
                    resp[CONF_LIN_ID], resp[CONF_DATA], resp[CONF_CHECKSUM]
                )
            )

    # Per-received-ID hints (sniffer/listener IDs this node does not publish)
    for rx in config.get(CONF_RX_IDS, []):
        cg.add(
            var.add_rx_hint(rx[CONF_LIN_ID], rx.get(CONF_LENGTH, 0), _checksum_mode(rx))
        )

    # Self-test
    cg.add(var.set_self_test(config[CONF_SELF_TEST]))

    # on_frame triggers
    for conf in config.get(CONF_ON_FRAME, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var, conf[CONF_LIN_ID])
        cg.add(var.register_frame_trigger(trigger))
        if CONF_LENGTH in conf or CONF_CHECKSUM in conf:
            cg.add(
                var.add_rx_hint(
                    conf[CONF_LIN_ID], conf.get(CONF_LENGTH, 0), _checksum_mode(conf)
                )
            )
        await automation.build_automation(
            trigger,
            [(cg.std_vector.template(cg.uint8), "data"), (cg.uint8, "data_length")],
            conf,
        )

    # on_error triggers (fault class passed as std::string `x`)
    for conf in config.get(CONF_ON_ERROR, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
        cg.add(var.register_error_trigger(trigger))
        await automation.build_automation(trigger, [(cg.std_string, "x")], conf)


# Action: update_response
@automation.register_action(
    "linbus.update_response",
    UpdateResponseAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(LinBusComponent),
            cv.Required(CONF_LIN_ID): cv.templatable(cv.int_range(min=0, max=63)),
            cv.Required(CONF_DATA): cv.templatable(
                cv.All([cv.hex_uint8_t], cv.Length(min=0, max=8))
            ),
        }
    ),
    synchronous=True,
)
async def update_response_to_code(config, action_id, template_arg, args):
    """Generate code for update_response action."""
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)

    # Handle templatable lin_id
    template_ = await cg.templatable(config[CONF_LIN_ID], args, cg.uint8)
    cg.add(var.set_lin_id(template_))

    # Handle templatable or static data
    data = config[CONF_DATA]
    if cg.is_template(data):
        template_ = await cg.templatable(data, args, cg.std_vector.template(cg.uint8))
        cg.add(var.set_data_template(template_))
    else:
        cg.add(var.set_data_static(data))

    return var


# Action: run_self_test — start/stop the built-in single-node loopback exerciser (S6).
@automation.register_action(
    "linbus.run_self_test",
    RunSelfTestAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(LinBusComponent),
            cv.Optional(CONF_ENABLE, default=True): cv.templatable(cv.boolean),
        }
    ),
    # play() flips a flag on the component inline — nothing deferred.
    synchronous=True,
)
async def run_self_test_to_code(config, action_id, template_arg, args):
    """Generate code for the run_self_test action."""
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)
    template_ = await cg.templatable(config[CONF_ENABLE], args, bool)
    cg.add(var.set_enable(template_))
    return var
