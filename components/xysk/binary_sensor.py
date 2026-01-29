import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_OUTPUT

from . import CONF_XYSK_ID, XYSK_COMPONENT_SCHEMA

DEPENDENCIES = ["xysk"]

CODEOWNERS = ["@syssi"]

# CONF_OUTPUT from const
CONF_KEY_LOCK = "key_lock"
CONF_CONSTANT_CURRENT_MODE = "constant_current_mode"

BINARY_SENSORS = [
    CONF_OUTPUT,
    CONF_KEY_LOCK,
    CONF_CONSTANT_CURRENT_MODE,
]

CONFIG_SCHEMA = XYSK_COMPONENT_SCHEMA.extend(
    {
        cv.Optional(CONF_OUTPUT): binary_sensor.binary_sensor_schema(icon="mdi:power"),
        cv.Optional(CONF_KEY_LOCK): binary_sensor.binary_sensor_schema(
            icon="mdi:play-box-lock-outline"
        ),
        cv.Optional(CONF_CONSTANT_CURRENT_MODE): binary_sensor.binary_sensor_schema(
            icon="mdi:current-dc"
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_XYSK_ID])
    for key in BINARY_SENSORS:
        if key in config:
            conf = config[key]
            sens = cg.new_Pvariable(conf[CONF_ID])
            await binary_sensor.register_binary_sensor(sens, conf)
            cg.add(getattr(hub, f"set_{key}_binary_sensor")(sens))
