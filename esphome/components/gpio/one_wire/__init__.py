import logging

from esphome import final_validate as fv, pins
import esphome.codegen as cg
from esphome.components import esp32, esp32_rmt
from esphome.components.one_wire import OneWireBus
from esphome.config_helpers import filter_source_files_from_platform
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_INVERTED,
    CONF_NUMBER,
    CONF_PIN,
    CONF_PLATFORM,
    PlatformFramework,
)
from esphome.types import ConfigType

from .. import gpio_ns

_LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@ssieb"]

CONF_USE_RMT = "use_rmt"

GPIOOneWireBus = gpio_ns.class_("GPIOOneWireBus", OneWireBus, cg.Component)


def validate_use_rmt(config: ConfigType) -> ConfigType:
    if config[CONF_USE_RMT]:
        cv.only_on_esp32(config[CONF_USE_RMT])
        if config.get(CONF_PIN, {}).get(CONF_INVERTED):
            raise cv.Invalid(f"'{CONF_USE_RMT}' does not support an inverted pin")
        if esp32.get_esp32_variant() in esp32_rmt.VARIANTS_NO_RMT:
            raise cv.Invalid(
                f"'{CONF_USE_RMT}' is not available on {esp32.get_esp32_variant()} "
                "(no RMT hardware)"
            )
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(GPIOOneWireBus),
            cv.Required(CONF_PIN): pins.internal_gpio_output_pin_schema,
            cv.Optional(CONF_USE_RMT, default=False): cv.boolean,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    validate_use_rmt,
)


def final_validate(config: ConfigType) -> ConfigType:
    # The RMT code is compiled for all GPIO buses or for none
    buses = [
        bus
        for bus in fv.full_config.get().get("one_wire", [])
        if bus[CONF_PLATFORM] == "gpio"
    ]
    if len({bus[CONF_USE_RMT] for bus in buses}) > 1:
        raise cv.Invalid(
            f"All GPIO 1-wire buses must have the same '{CONF_USE_RMT}' setting"
        )
    return config


FINAL_VALIDATE_SCHEMA = final_validate


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    await cg.add_pin(var.set_pin, config[CONF_PIN])

    if config[CONF_USE_RMT]:
        # ESP-IDF's RMT driver is excluded from the build by default
        esp32.include_builtin_idf_component("esp_driver_rmt")
        cg.add_define("USE_ONE_WIRE_RMT")
        _LOGGER.info(
            "1-wire bus on GPIO%s uses the RMT peripheral",
            config[CONF_PIN][CONF_NUMBER],
        )


FILTER_SOURCE_FILES = filter_source_files_from_platform(
    {
        "gpio_one_wire_rmt.cpp": {
            PlatformFramework.ESP32_ARDUINO,
            PlatformFramework.ESP32_IDF,
        },
    }
)
