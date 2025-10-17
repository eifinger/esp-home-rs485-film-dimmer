import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import cover, uart
from esphome.const import CONF_OUTPUT_ID
from esphome import pins

CODEOWNERS = ["@eifinger"]
DEPENDENCIES = ["uart"]

rs485_dimmer_ns = cg.esphome_ns.namespace("rs485_dimmer")
RS485Dimmer = rs485_dimmer_ns.class_(
    "RS485Dimmer", cover.Cover, cg.Component, uart.UARTDevice
)

CONF_TX_ENABLE_PIN = "tx_enable_pin"

CONFIG_SCHEMA = cover.COVER_SCHEMA.extend(
    {
        cv.GenerateID(CONF_OUTPUT_ID): cv.declare_id(RS485Dimmer),
        cv.Required(CONF_TX_ENABLE_PIN): pins.gpio_output_pin_schema,
    }
).extend(uart.UART_DEVICE_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_OUTPUT_ID])
    await cg.register_component(var, config)
    await cover.register_cover(var, config)
    await uart.register_uart_device(var, config)

    tx_enable_pin = await cg.gpio_pin_expression(config[CONF_TX_ENABLE_PIN])
    cg.add(var.set_tx_enable_pin(tx_enable_pin))
