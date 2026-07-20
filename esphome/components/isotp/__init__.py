"""ISO 15765-2 (ISO-TP) transport, layered on a can_gateway port.

Validation rules referenced as V1-V10 are defined in the component specification; the numbering
is kept stable so test names and error messages stay traceable.
"""

from esphome import automation, core
import esphome.codegen as cg
from esphome.components.can_gateway import (
    CONF_OBSERVE_QUEUE_DEPTH,
    CONF_PORTS,
    GatewayPort,
)
import esphome.config_validation as cv
from esphome.const import CONF_DATA, CONF_ID
import esphome.final_validate as fv

CODEOWNERS = ["@swifty99"]
DEPENDENCIES = ["can_gateway"]
MULTI_CONF = True

isotp_ns = cg.esphome_ns.namespace("isotp")
IsoTpProtocol = isotp_ns.class_("IsoTpProtocol", cg.Component)
MessageView = isotp_ns.struct("MessageView")
IsoTpError = isotp_ns.enum("IsoTpError", is_class=True)
SendAction = isotp_ns.class_("SendAction", automation.Action)

CONF_PORT_ID = "port_id"
CONF_USE_EXTENDED_ID = "use_extended_id"
CONF_ADDRESSING = "addressing"
CONF_TX_ID = "tx_id"
CONF_RX_ID = "rx_id"
CONF_SOURCE = "source"
CONF_TARGET = "target"
CONF_TARGET_ADDRESS = "target_address"
CONF_MAX_MESSAGE_SIZE = "max_message_size"
CONF_BLOCK_SIZE = "block_size"
CONF_ST_MIN = "st_min"
CONF_PADDING = "padding"
CONF_N_BS_TIMEOUT = "n_bs_timeout"
CONF_N_CR_TIMEOUT = "n_cr_timeout"
CONF_TX_STALL_TIMEOUT = "tx_stall_timeout"
CONF_ON_MESSAGE = "on_message"
CONF_ON_ERROR = "on_error"

ADDRESSING_NORMAL = "normal"
ADDRESSING_NORMAL_FIXED = "normal_fixed"
ADDRESSING_EXTENDED = "extended"
ADDRESSING_MODES = [ADDRESSING_NORMAL, ADDRESSING_NORMAL_FIXED, ADDRESSING_EXTENDED]

STANDARD_ID_MAX = 0x7FF
EXTENDED_ID_MAX = 0x1FFFFFFF

# ISO 15765-2 caps a classic-CAN message at the 12-bit FirstFrame length field. Longer messages
# need the 2016 escape sequence, which is deferred (V4).
MAX_MESSAGE_SIZE_LIMIT = 4095
MIN_MESSAGE_SIZE = 8

# A granted block arrives as one burst well inside a loop period, so the port's ring has to hold
# it with room to spare for whatever else that port carries.
BLOCK_SIZE_RING_MARGIN = 4

# Normal-fixed addressing builds 29-bit identifiers as 0x18DA<target><source> (ISO 15765-2).
NORMAL_FIXED_BASE = 0x18DA0000


def _validate_st_min(value):
    """Encode an STmin duration into its raw ISO byte (V10).

    0x00-0x7F are whole milliseconds. 0xF1-0xF9 are 100-900 microseconds. Nothing else is
    expressible, and reserved encodings are never emitted.
    """
    period = cv.positive_time_period_microseconds(value)
    microseconds = period.total_microseconds
    if microseconds % 1000 == 0 and microseconds // 1000 <= 0x7F:
        return microseconds // 1000
    if 100 <= microseconds <= 900 and microseconds % 100 == 0:
        return 0xF0 + microseconds // 100
    raise cv.Invalid(
        f"st_min must be a whole number of milliseconds up to 127ms, or 100-900us in 100us "
        f"steps; got {microseconds}us"
    )


def _validate_addressing(config):
    """Enforce that the identifier keys match the addressing mode (V3, V9)."""
    mode = config[CONF_ADDRESSING]
    raw_keys = [CONF_TX_ID, CONF_RX_ID]
    fixed_keys = [CONF_SOURCE, CONF_TARGET]

    if mode == ADDRESSING_NORMAL_FIXED:
        for key in raw_keys:
            if key in config:
                raise cv.Invalid(
                    f"'{key}' is not used with addressing '{mode}'; the identifiers are derived "
                    f"from '{CONF_SOURCE}' and '{CONF_TARGET}'",
                    path=[key],
                )
        for key in fixed_keys:
            if key not in config:
                raise cv.Invalid(
                    f"'{key}' is required with addressing '{mode}'", path=[key]
                )
        source = config[CONF_SOURCE]
        target = config[CONF_TARGET]
        config[CONF_TX_ID] = NORMAL_FIXED_BASE | (target << 8) | source
        config[CONF_RX_ID] = NORMAL_FIXED_BASE | (source << 8) | target
        config[CONF_USE_EXTENDED_ID] = True
    else:
        for key in fixed_keys:
            if key in config:
                raise cv.Invalid(
                    f"'{key}' is only used with addressing '{ADDRESSING_NORMAL_FIXED}'",
                    path=[key],
                )
        for key in raw_keys:
            if key not in config:
                raise cv.Invalid(
                    f"'{key}' is required with addressing '{mode}'", path=[key]
                )
        extended = config[CONF_USE_EXTENDED_ID]
        limit = EXTENDED_ID_MAX if extended else STANDARD_ID_MAX
        for key in raw_keys:
            if config[key] > limit:
                raise cv.Invalid(
                    f"'{key}' 0x{config[key]:X} does not fit a "
                    f"{'29' if extended else '11'}-bit identifier",
                    path=[key],
                )

    if mode == ADDRESSING_EXTENDED and CONF_TARGET_ADDRESS not in config:
        raise cv.Invalid(
            f"'{CONF_TARGET_ADDRESS}' is required with addressing '{mode}'",
            path=[CONF_TARGET_ADDRESS],
        )
    if mode != ADDRESSING_EXTENDED and CONF_TARGET_ADDRESS in config:
        raise cv.Invalid(
            f"'{CONF_TARGET_ADDRESS}' is only used with addressing '{ADDRESSING_EXTENDED}'",
            path=[CONF_TARGET_ADDRESS],
        )

    # V1: an instance that talks to itself would reassemble its own requests.
    if config[CONF_TX_ID] == config[CONF_RX_ID]:
        raise cv.Invalid(
            f"'{CONF_TX_ID}' and '{CONF_RX_ID}' must differ (both are 0x{config[CONF_TX_ID]:X})"
        )
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(IsoTpProtocol),
            cv.Required(CONF_PORT_ID): cv.use_id(GatewayPort),
            cv.Optional(CONF_ADDRESSING, default=ADDRESSING_NORMAL): cv.one_of(
                *ADDRESSING_MODES, lower=True
            ),
            cv.Optional(CONF_TX_ID): cv.int_range(min=0, max=EXTENDED_ID_MAX),
            cv.Optional(CONF_RX_ID): cv.int_range(min=0, max=EXTENDED_ID_MAX),
            cv.Optional(CONF_USE_EXTENDED_ID, default=False): cv.boolean,
            cv.Optional(CONF_SOURCE): cv.hex_uint8_t,
            cv.Optional(CONF_TARGET): cv.hex_uint8_t,
            cv.Optional(CONF_TARGET_ADDRESS): cv.hex_uint8_t,
            cv.Optional(CONF_MAX_MESSAGE_SIZE, default=256): cv.int_range(
                min=MIN_MESSAGE_SIZE, max=MAX_MESSAGE_SIZE_LIMIT
            ),
            # V6: granting block size 0 would invite the peer to send every remaining frame back
            # to back, which is exactly the burst the ring cannot absorb.
            cv.Optional(CONF_BLOCK_SIZE, default=8): cv.int_range(min=1, max=255),
            cv.Optional(CONF_ST_MIN, default="0ms"): _validate_st_min,
            cv.Optional(CONF_PADDING): cv.hex_uint8_t,
            cv.Optional(
                CONF_N_BS_TIMEOUT, default="1000ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(
                CONF_N_CR_TIMEOUT, default="1000ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(
                CONF_TX_STALL_TIMEOUT, default="1000ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_ON_MESSAGE): automation.validate_automation(),
            cv.Optional(CONF_ON_ERROR): automation.validate_automation(),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _validate_addressing,
)


def _find_port_config(full_config, port_id):
    """Locate a can_gateway port block by its declared id, whatever shape the gateway config has."""
    gateway_config = full_config.get("can_gateway")
    if gateway_config is None:
        return None
    blocks = gateway_config if isinstance(gateway_config, list) else [gateway_config]
    for block in blocks:
        for port in block.get(CONF_PORTS, []):
            if port.get(CONF_ID) == port_id:
                return port
    return None


def _final_validate(config):
    """V5: the port's observation ring must be able to hold a whole granted block.

    A config error rather than a warning on purpose. The failure this prevents is silently
    dropped consecutive frames, which surfaces as intermittent N_Cr timeouts a long way from
    the cause.
    """
    port = _find_port_config(fv.full_config.get(), config[CONF_PORT_ID])
    if port is None:
        # The port id resolved at schema time, so a miss here means an unexpected config shape
        # rather than a user error; leave it to can_gateway's own validation to report.
        return config
    depth = port[CONF_OBSERVE_QUEUE_DEPTH]
    required = config[CONF_BLOCK_SIZE] + BLOCK_SIZE_RING_MARGIN
    if depth < required:
        raise cv.Invalid(
            f"'{CONF_BLOCK_SIZE}' {config[CONF_BLOCK_SIZE]} needs the can_gateway port's "
            f"'{CONF_OBSERVE_QUEUE_DEPTH}' to be at least {required} (it is {depth}). A granted "
            f"block arrives as one burst, so a smaller ring drops consecutive frames and the "
            f"transfer fails with an N_Cr timeout. Raise '{CONF_OBSERVE_QUEUE_DEPTH}' on the "
            f"port, or lower '{CONF_BLOCK_SIZE}'.",
            path=[CONF_BLOCK_SIZE],
        )
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config):
    # V8: can_gateway emits this only from its decode platforms, so an isotp-only config would
    # otherwise compile the observation path out and receive nothing at all, silently.
    cg.add_define("USE_CAN_GATEWAY_OBSERVE")

    port = await cg.get_variable(config[CONF_PORT_ID])
    var = cg.new_Pvariable(
        config[CONF_ID],
        port,
        config[CONF_TX_ID],
        config[CONF_RX_ID],
        config[CONF_USE_EXTENDED_ID],
    )
    await cg.register_component(var, config)

    cg.add(var.set_max_message_size(config[CONF_MAX_MESSAGE_SIZE]))
    cg.add(var.set_block_size(config[CONF_BLOCK_SIZE]))
    cg.add(var.set_st_min_raw(config[CONF_ST_MIN]))
    cg.add(var.set_n_bs_timeout(config[CONF_N_BS_TIMEOUT]))
    cg.add(var.set_n_cr_timeout(config[CONF_N_CR_TIMEOUT]))
    cg.add(var.set_tx_stall_timeout(config[CONF_TX_STALL_TIMEOUT]))
    if (padding := config.get(CONF_PADDING)) is not None:
        cg.add(var.set_padding(padding))
    if (target_address := config.get(CONF_TARGET_ADDRESS)) is not None:
        cg.add(var.set_address_extension(target_address))

    # Registration must land before App.setup(), which is where can_gateway freezes the
    # subscriber registry. Codegen statements run in main.cpp ahead of setup(), so this is safe.
    cg.add(
        port.subscribe_consumer(config[CONF_RX_ID], config[CONF_USE_EXTENDED_ID], var)
    )

    for conf in config.get(CONF_ON_MESSAGE, []):
        await automation.build_callback_automation(
            var, "add_on_message_callback", [(MessageView, "x")], conf
        )
    for conf in config.get(CONF_ON_ERROR, []):
        await automation.build_callback_automation(
            var, "add_on_error_callback", [(IsoTpError, "x")], conf
        )


SEND_ACTION_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ID): cv.use_id(IsoTpProtocol),
        cv.Required(CONF_DATA): cv.templatable(
            cv.Any(
                cv.All(cv.ensure_list(cv.hex_uint8_t), cv.Length(min=1)),
                cv.string,
            )
        ),
    }
)


@automation.register_action(
    "isotp.send", SendAction, SEND_ACTION_SCHEMA, synchronous=True
)
async def isotp_send_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    data = config[CONF_DATA]
    if isinstance(data, str):
        data = list(data.encode())
    if isinstance(data, core.Lambda):
        templ = await cg.templatable(data, args, cg.std_vector.template(cg.uint8))
        cg.add(var.set_data_template(templ))
    else:
        cg.add(var.set_data_static(data))
    return var
