import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor

from . import CONF_AUTOSHADE_LINK_ID, AutoShadeLink

DEPENDENCIES = ["autoshade_link"]

CONF_BUTTON = "button"

# Bit positions in the board's K= mask, following the Adafruit RGB LCD shield.
BUTTONS = {
    "select": 0,
    "right": 1,
    "down": 2,
    "up": 3,
    "left": 4,
}

CONFIG_SCHEMA = binary_sensor.binary_sensor_schema().extend(
    {
        cv.GenerateID(CONF_AUTOSHADE_LINK_ID): cv.use_id(AutoShadeLink),
        cv.Required(CONF_BUTTON): cv.enum(BUTTONS, lower=True),
    }
)


async def to_code(config):
    var = await binary_sensor.new_binary_sensor(config)
    parent = await cg.get_variable(config[CONF_AUTOSHADE_LINK_ID])
    cg.add(parent.register_button(config[CONF_BUTTON], var))
