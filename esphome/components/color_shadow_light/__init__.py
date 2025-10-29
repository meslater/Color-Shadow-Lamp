import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import light as light_component
from esphome.const import CONF_OUTPUT_ID

AUTO_LOAD = ["light"]
DEPENDENCIES = []

color_shadow_ns = cg.esphome_ns.namespace("color_shadow_light")
ColorShadowLight = color_shadow_ns.class_("ColorShadowLight", light_component.LightOutput, cg.Component)

CONFIG_SCHEMA = light_component.RGB_LIGHT_SCHEMA.extend({
    cv.GenerateID(CONF_OUTPUT_ID): cv.declare_id(ColorShadowLight),
}).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    cg.add_global(cg.RawStatement('#include "esphome/components/color_shadow_light/color_shadow_light.h"'))
    var = cg.new_Pvariable(config[CONF_OUTPUT_ID])
    await cg.register_component(var, config)
    await light_component.register_light(var, config)
