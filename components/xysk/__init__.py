import esphome.codegen as cg
from esphome.components import modbus
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@syssi"]

DEPENDENCIES = ["modbus"]
AUTO_LOAD = ["binary_sensor", "number", "sensor", "switch", "text_sensor"]
MULTI_CONF = True

CONF_XYSK_ID = "xysk_id"
CONF_CURRENT_RESOLUTION = "current_resolution"

xysk_ns = cg.esphome_ns.namespace("xysk")
Xysk = xysk_ns.class_("Xysk", cg.PollingComponent, modbus.ModbusDevice)

CurrentResolution = xysk_ns.enum("CurrentResolution")
CURRENT_RESOLUTION_OPTIONS = {
    "AUTO": CurrentResolution.XYSK_CURRENT_RESOLUTION_AUTO,
    "LOW": CurrentResolution.XYSK_CURRENT_RESOLUTION_LOW,
    "HIGH": CurrentResolution.XYSK_CURRENT_RESOLUTION_HIGH,
}

XYSK_COMPONENT_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_XYSK_ID): cv.use_id(Xysk),
    }
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Xysk),
            cv.Optional(CONF_CURRENT_RESOLUTION, default="AUTO"): cv.enum(
                CURRENT_RESOLUTION_OPTIONS, upper=True
            ),
        }
    )
    .extend(cv.polling_component_schema("5s"))
    .extend(modbus.modbus_device_schema(0x01))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await modbus.register_modbus_device(var, config)

    cg.add(var.set_current_resolution(config[CONF_CURRENT_RESOLUTION]))
