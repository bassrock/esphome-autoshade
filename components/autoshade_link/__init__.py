import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, uart
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
    UNIT_VOLT,
)

CODEOWNERS = ["@danielaaronbrooks408"]
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "binary_sensor"]

autoshade_link_ns = cg.esphome_ns.namespace("autoshade_link")
AutoShadeLink = autoshade_link_ns.class_(
    "AutoShadeLink", cg.PollingComponent, uart.UARTDevice
)

CONF_AUTOSHADE_LINK_ID = "autoshade_link_id"
CONF_SUPPLY_VOLTAGE = "supply_voltage"
CONF_BATTERY_VOLTAGE = "battery_voltage"
CONF_SCALE = "scale"

# ADC counts to volts, from the original firmware's own comments.
DEFAULT_SUPPLY_SCALE = 0.01238
DEFAULT_BATTERY_SCALE = 0.009872


def _voltage_schema(default_scale):
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT,
    ).extend({cv.Optional(CONF_SCALE, default=default_scale): cv.float_})


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AutoShadeLink),
            cv.Optional(CONF_SUPPLY_VOLTAGE): _voltage_schema(DEFAULT_SUPPLY_SCALE),
            cv.Optional(CONF_BATTERY_VOLTAGE): _voltage_schema(DEFAULT_BATTERY_SCALE),
        }
    )
    .extend(cv.polling_component_schema("500ms"))
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if CONF_SUPPLY_VOLTAGE in config:
        conf = config[CONF_SUPPLY_VOLTAGE]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_supply_sensor(sens))
        cg.add(var.set_supply_scale(conf[CONF_SCALE]))

    if CONF_BATTERY_VOLTAGE in config:
        conf = config[CONF_BATTERY_VOLTAGE]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_battery_sensor(sens))
        cg.add(var.set_battery_scale(conf[CONF_SCALE]))
