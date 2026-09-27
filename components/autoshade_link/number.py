import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import CONF_MODE, ENTITY_CATEGORY_CONFIG

from . import autoshade_link_ns
from .cover import AutoShadeCover

DEPENDENCIES = ["autoshade_link"]

AutoShadeLengthNumber = autoshade_link_ns.class_("AutoShadeLengthNumber", number.Number)

CONF_COVER_ID = "cover_id"
CONF_STEPS_PER_FOOT = "steps_per_foot"
CONF_MAX_LENGTH = "max_length"

# A shade's full drop, in inches, editable from HA. Stored by the cover, so it
# survives reboots and overrides the cover's travel_steps from YAML.
CONFIG_SCHEMA = number.number_schema(
    AutoShadeLengthNumber,
    unit_of_measurement="in",
    icon="mdi:arrow-expand-vertical",
    entity_category=ENTITY_CATEGORY_CONFIG,
).extend(
    {
        cv.Required(CONF_COVER_ID): cv.use_id(AutoShadeCover),
        # From the stock firmware: 2124 steps per foot of drop.
        cv.Optional(CONF_STEPS_PER_FOOT, default=2124): cv.float_range(min=100, max=20000),
        cv.Optional(CONF_MAX_LENGTH, default=240): cv.float_range(min=12, max=600),
        cv.Optional(CONF_MODE, default="BOX"): cv.enum(number.NUMBER_MODES, upper=True),
    }
)


async def to_code(config):
    spi = config[CONF_STEPS_PER_FOOT] / 12.0
    var = await number.new_number(
        config, min_value=1.0, max_value=config[CONF_MAX_LENGTH], step=0.25
    )
    cov = await cg.get_variable(config[CONF_COVER_ID])
    cg.add(var.set_cover(cov, spi))
    cg.add(cov.set_length_number(var, spi))
