"""LIN Bus component for ESPHome."""

from esphome import automation, pins
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_BAUD_RATE, CONF_ID, CONF_MODE, CONF_TRIGGER_ID

DEPENDENCIES = ["esp32"]
CODEOWNERS = ["@swifty99"]

# Namespace
linbus_ns = cg.esphome_ns.namespace("linbus")
LinBusComponent = linbus_ns.class_("LinBusComponent", cg.Component)
LinFrameTrigger = linbus_ns.class_(
    "LinFrameTrigger",
    automation.Trigger.template(cg.std_vector.template(cg.uint8), cg.uint8),
)
UpdateResponseAction = linbus_ns.class_("UpdateResponseAction", automation.Action)

# Configuration constants
CONF_CS_PIN = "cs_pin"
CONF_SCHEDULE = "schedule"
CONF_RESPONSES = "responses"
CONF_LIN_ID = "lin_id"
CONF_INTERVAL = "interval"
CONF_DATA = "data"
CONF_CHECKSUM = "checksum"
CONF_UART_PORT = "uart_port"
CONF_TX_PIN = "tx_pin"
CONF_RX_PIN = "rx_pin"
CONF_ON_FRAME = "on_frame"
CONF_LINBUS_ID = "linbus_id"

# Mode constants
MODE_MASTER = "master"
MODE_SLAVE = "slave"
MODE_LISTENER = "listener"

# Validation schemas
SCHEDULE_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_LIN_ID): cv.int_range(min=0, max=63),
        cv.Required(CONF_INTERVAL): cv.positive_time_period_milliseconds,
    }
)

RESPONSE_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_LIN_ID): cv.int_range(min=0, max=63),
        cv.Required(CONF_DATA): cv.All([cv.hex_uint8_t], cv.Length(min=0, max=8)),
        cv.Optional(CONF_CHECKSUM, default="enhanced"): cv.enum(
            {"classic": False, "enhanced": True}, lower=True
        ),
    }
)

# Main configuration schema
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(LinBusComponent),
        cv.Required(CONF_TX_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_RX_PIN): pins.gpio_input_pin_schema,
        cv.Optional(CONF_CS_PIN): pins.gpio_output_pin_schema,
        cv.Optional(CONF_BAUD_RATE, default=19200): cv.int_,
        cv.Optional(CONF_UART_PORT, default="UART_NUM_1"): cv.string,
        cv.Optional(CONF_MODE, default=MODE_MASTER): cv.enum(
            {MODE_MASTER: 0, MODE_SLAVE: 1, MODE_LISTENER: 2}, lower=True
        ),
        cv.Optional(CONF_SCHEDULE): cv.All(
            cv.ensure_list(SCHEDULE_SCHEMA),
            cv.Length(max=16),
        ),
        cv.Optional(CONF_RESPONSES): cv.All(
            cv.ensure_list(RESPONSE_SCHEMA),
            cv.Length(max=16),
        ),
        cv.Optional(CONF_ON_FRAME): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(LinFrameTrigger),
                cv.Required(CONF_LIN_ID): cv.int_range(min=0, max=63),
            }
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


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

    # Schedule (master mode)
    if CONF_SCHEDULE in config:
        for item in config[CONF_SCHEDULE]:
            cg.add(
                var.add_schedule_item(
                    item[CONF_LIN_ID], item[CONF_INTERVAL].total_milliseconds
                )
            )

    # Responses
    if CONF_RESPONSES in config:
        for resp in config[CONF_RESPONSES]:
            cg.add(
                var.add_response(
                    resp[CONF_LIN_ID], resp[CONF_DATA], resp[CONF_CHECKSUM]
                )
            )

    # on_frame triggers
    for conf in config.get(CONF_ON_FRAME, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var, conf[CONF_LIN_ID])
        cg.add(var.register_frame_trigger(trigger))
        await automation.build_automation(
            trigger,
            [(cg.std_vector.template(cg.uint8), "data"), (cg.uint8, "data_length")],
            conf,
        )


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
