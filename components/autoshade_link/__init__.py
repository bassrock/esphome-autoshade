from pathlib import Path
import re

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, uart, usb_uart
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
    UNIT_VOLT,
)

CODEOWNERS = ["@danielaaronbrooks408"]
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "binary_sensor", "number"]

autoshade_link_ns = cg.esphome_ns.namespace("autoshade_link")
AutoShadeLink = autoshade_link_ns.class_(
    "AutoShadeLink", cg.PollingComponent, uart.UARTDevice
)

CONF_AUTOSHADE_LINK_ID = "autoshade_link_id"
CONF_SUPPLY_VOLTAGE = "supply_voltage"
CONF_BATTERY_VOLTAGE = "battery_voltage"
CONF_SCALE = "scale"
# The usb_uart channel the board is on. Lets the ESP pulse DTR, which resets
# the '328P into its bootloader, for a board firmware update.
CONF_USB_CHANNEL = "usb_channel"
# Intel HEX of autoshade_dumb.ino to bundle for board updates. Defaults to the
# one built by avr/build.sh and shipped with this component.
CONF_AVR_FIRMWARE = "avr_firmware"
BUNDLED_AVR_FIRMWARE = Path(__file__).parent / "firmware" / "autoshade_dumb.hex"
# Winding model for the board's PWM drive modes (firmware 3.4+). Defaults are
# the StepperOnline 23HS22-2804S: 0.92 ohm, 2.68 mH, and a back-EMF constant
# of ~0.32 V per rad/s (1.26 N.m holding at 2.8 A, both phases).
CONF_MOTOR_RESISTANCE = "motor_resistance"
CONF_MOTOR_INDUCTANCE = "motor_inductance"
CONF_MOTOR_BACK_EMF = "motor_back_emf"
# ATmega328P flash below Optiboot's 512 bytes.
AVR_APP_MAX = 32256

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
            cv.Optional(CONF_USB_CHANNEL): cv.use_id(usb_uart.USBUartChannel),
            cv.Optional(CONF_AVR_FIRMWARE): cv.file_,
            cv.Optional(CONF_MOTOR_RESISTANCE, default=0.92): cv.float_range(min=0.1, max=60),  # ohm
            cv.Optional(CONF_MOTOR_INDUCTANCE, default=2.68): cv.float_range(min=0, max=60),  # mH
            cv.Optional(CONF_MOTOR_BACK_EMF, default=0.318): cv.float_range(min=0, max=60),  # V/(rad/s)
        }
    )
    .extend(cv.polling_component_schema("500ms"))
    .extend(uart.UART_DEVICE_SCHEMA)
)


def _load_hex(path):
    """Flatten an Intel HEX file into one image starting at address 0."""
    image = bytearray(b"\xff" * AVR_APP_MAX)
    top = 0
    base = 0
    for n, line in enumerate(Path(path).read_text().splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        if not line.startswith(":"):
            raise cv.Invalid(f"{path}:{n}: not Intel HEX")
        rec = bytes.fromhex(line[1:])
        if sum(rec) & 0xFF:
            raise cv.Invalid(f"{path}:{n}: bad checksum")
        length, addr, kind, data = rec[0], (rec[1] << 8) | rec[2], rec[3], rec[4 : 4 + rec[0]]
        if kind == 0x00:
            start = base + addr
            if start + length > AVR_APP_MAX:
                raise cv.Invalid(f"{path}: image runs into the bootloader at 0x{start + length:04X}")
            image[start : start + length] = data
            top = max(top, start + length)
        elif kind == 0x01:
            break
        elif kind == 0x02:
            base = ((data[0] << 8) | data[1]) << 4
        elif kind == 0x04:
            base = ((data[0] << 8) | data[1]) << 16
    if top == 0:
        raise cv.Invalid(f"{path}: empty image")
    return bytes(image[:top])


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(
        var.set_motor_model(
            config[CONF_MOTOR_RESISTANCE],
            config[CONF_MOTOR_INDUCTANCE],
            config[CONF_MOTOR_BACK_EMF],
        )
    )

    if CONF_USB_CHANNEL in config:
        channel = await cg.get_variable(config[CONF_USB_CHANNEL])
        cg.add(var.set_usb_channel(channel))
        cg.add_define("AUTOSHADE_LINK_USB")

    hex_path = config.get(CONF_AVR_FIRMWARE, BUNDLED_AVR_FIRMWARE)
    if Path(hex_path).is_file():
        image = _load_hex(hex_path)
        m = re.search(rb"AUTOSHADE-DUMB (\d+\.\d+)", image)
        version = m.group(1).decode() if m else "unknown"
        arr = f"autoshade_avr_image_{config[CONF_ID].id}"
        body = ",".join(str(b) for b in image)
        cg.add_global(cg.RawStatement(f"static const uint8_t {arr}[] = {{{body}}};"))
        cg.add(var.set_avr_image(cg.RawExpression(arr), len(image), version))
    elif CONF_AVR_FIRMWARE in config:
        raise cv.Invalid(f"{hex_path}: not found")

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
