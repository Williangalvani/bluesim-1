// Surfacing drops over a given texture (e.g. a SubViewport's, when the rect itself is inside a SubViewport, where
// the compatibility renderer gives a blank screen texture). Heartfelt by BigWings, CC BY-NC-SA 3.0; see
// surfacing_drops.gdshaderinc.
shader_type canvas_item;
render_mode unshaded;
uniform sampler2D scene_tex : filter_linear, repeat_disable;
#define BASE_UV UV
#define BASE_SCALE vec2(1.0)
#include "res://scripts/fx/surfacing_drops.gdshaderinc"
