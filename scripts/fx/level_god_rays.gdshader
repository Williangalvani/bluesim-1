// A custom level's fake light shafts (the reef's UnderwaterFX/GodRays, scripts/fx/god_rays.gdshader in its pack),
// redrawn for BlueSim's sea: the level was built for a surface 3.4 m higher than BlueSim's water plane, so its shafts
// were brightest above the water and were cut off where the water's underside is drawn over them.
// - render_priority 1 on the material (set by levels/baselevel.gd) draws them after the water, not under it;
// - brightness follows the depth below BlueSim's surface (surface_y): a short fade-in under the surface, nothing above
//   it, and the level's fade towards the seafloor (floor_y);
// - speed scales the shimmer, which was 0.45 and 0.31 rad/s.
// Additive, unshaded, no depth texture: WebGL2 / gl_compatibility safe like the original.
shader_type spatial;
render_mode blend_add, unshaded, cull_disabled, depth_draw_never, shadows_disabled, fog_disabled;
uniform vec4 tint : source_color = vec4(0.55, 0.86, 1.0, 1.0);
uniform float strength = 0.16;
uniform float near_fade_begin = 1.5;
uniform float near_fade_end = 7.0;
uniform float far_fade_begin = 28.0;
uniform float far_fade_end = 52.0;
uniform float speed = 2.5;            // shimmer speed, 1 = the level's original
uniform float surface_y = 3.53;       // world height of the water surface
uniform float surface_fade = 0.3;     // metres under the surface over which the shafts fade in
uniform float floor_y = -5.0;         // world height the shafts reach at the seafloor
varying float v_int;
varying float v_phase;
varying vec3 v_wpos;
void vertex() {
	v_int = COLOR.r; v_phase = COLOR.g * 6.2831;
	v_wpos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}
void fragment() {
	float depth = surface_y - v_wpos.y;
	if (depth <= 0.0) discard;                                                         // above the water: no shaft
	float edge = pow(abs(dot(normalize(NORMAL), normalize(VIEW))), 2.0);             // soft cylinder edges
	float t = depth / max(surface_y - floor_y, 0.1);
	float along = smoothstep(0.0, surface_fade, depth) * (1.0 - smoothstep(0.45, 1.0, t)); // bright near surface, fades down
	float time = TIME * speed;
	float n = 0.55 + 0.45 * sin(time * 0.45 + v_phase + v_wpos.x * 0.6 + v_wpos.y * 0.15) * sin(time * 0.31 + v_wpos.z * 0.8 + v_phase * 1.7);
	float d = length(VERTEX);                                                              // view-space distance
	float df = smoothstep(near_fade_begin, near_fade_end, d) * (1.0 - smoothstep(far_fade_begin, far_fade_end, d));
	ALBEDO = tint.rgb * strength * v_int * edge * along * n * df;
}
