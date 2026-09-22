import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import cover

from . import CONF_AUTOSHADE_LINK_ID, AutoShadeLink, autoshade_link_ns

DEPENDENCIES = ["autoshade_link"]

AutoShadeCover = autoshade_link_ns.class_("AutoShadeCover", cover.Cover, cg.Component)

HoldMode = autoshade_link_ns.enum("HoldMode")
HOLD_MODES = {
    "coast": HoldMode.HOLD_COAST,
    "brake": HoldMode.HOLD_BRAKE,
}

CONF_MOTOR = "motor"
CONF_TRAVEL_STEPS = "travel_steps"
CONF_MAX_SPEED = "max_speed"
CONF_ACCELERATION = "acceleration"
CONF_HOLD = "hold"
CONF_HOME_OVERRUN = "home_overrun"

CONFIG_SCHEMA = (
    cover.cover_schema(AutoShadeCover)
    .extend(
        {
            cv.GenerateID(CONF_AUTOSHADE_LINK_ID): cv.use_id(AutoShadeLink),
            # Motor number as silkscreened on the board: SHADE 1 .. SHADE 6.
            cv.Required(CONF_MOTOR): cv.int_range(min=1, max=6),
            # Full drop in steps. 2124 steps per foot, so 7 ft = 14868.
            cv.Required(CONF_TRAVEL_STEPS): cv.int_range(min=100, max=200000),
            # The board's I2C bus tops out near 797 steps/s across all motors.
            cv.Optional(CONF_MAX_SPEED, default=400): cv.int_range(min=20, max=1500),
            cv.Optional(CONF_ACCELERATION, default=400): cv.int_range(min=20, max=4000),
            # brake shorts the windings at rest: no current, resists back-drive.
            cv.Optional(CONF_HOLD, default="brake"): cv.enum(HOLD_MODES, lower=True),
            # Steps driven past zero into the top hard stop when homing.
            cv.Optional(CONF_HOME_OVERRUN, default=150): cv.int_range(min=0, max=2000),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await cover.new_cover(config)
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_AUTOSHADE_LINK_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_motor(config[CONF_MOTOR]))
    cg.add(var.set_travel_steps(config[CONF_TRAVEL_STEPS]))
    cg.add(var.set_max_speed(config[CONF_MAX_SPEED]))
    cg.add(var.set_acceleration(config[CONF_ACCELERATION]))
    cg.add(var.set_hold_mode(config[CONF_HOLD]))
    cg.add(var.set_home_overrun(config[CONF_HOME_OVERRUN]))
    cg.add(parent.register_cover(var))
