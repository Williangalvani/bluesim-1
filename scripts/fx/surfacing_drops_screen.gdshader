// Surfacing drops over the screen behind the rect. Heartfelt by BigWings, CC BY-NC-SA 3.0; see surfacing_drops.gdshaderinc.
shader_type canvas_item;
render_mode unshaded;
uniform sampler2D scene_tex : hint_screen_texture, filter_linear_mipmap, repeat_disable;
#define BASE_UV SCREEN_UV
#define BASE_SCALE (full_size * SCREEN_PIXEL_SIZE)
#define BASE_LOD
#include "res://scripts/fx/surfacing_drops.gdshaderinc"
