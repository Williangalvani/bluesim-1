// An air bubble stuck on the outside of the camera's dome, underwater: a little lens of air that shows the view around
// it shrunk and the right way up, a bright rim where the water's surface around the air catches the light, brighter on
// the side towards the sun, a darker ring inside it, and the glints of the sun: one where the cap's curve turns it towards
// the camera, and a fainter one opposite where the light coming in through the bubble meets its far side, so they move
// round the bubble as the camera turns. Drawn on a camera-facing quad per particle (scripts/dome_bubbles.gd), which
// gives it the sun's direction and colour, with four reads of the screen texture per pixel.
//
// The screen texture only holds what was drawn before the transparent pass, in both Forward+ and Compatibility, so the
// water's surface is not in it: where the view through the bubble goes up to the surface, what is already drawn under
// the bubble shows through instead. Compatibility gives it no mipmaps (textureLod reads the full-size image) and no
// screen texture in vertex(), so the light around the bubble is read from the view around it, three times as wide.
// It also works on sRGB values where Forward+ works on linear ones, so the bubble is worked out in sRGB in both, and its
// colours are given in sRGB.
shader_type spatial;
render_mode unshaded, blend_premul_alpha, cull_disabled, depth_draw_never, depth_test_disabled, fog_disabled;

uniform float surface_altitude;
uniform sampler2D scene : hint_screen_texture, filter_linear_mipmap, repeat_disable;
// the water's, as in the camera's Environment, for how much of the surface shows through it
uniform float fog_density = 0.124;
// how the surface looks from under it, lighting the bubbles under it
uniform vec3 surface_colour = vec3(0.68, 0.83, 0.96);
// how far the bubble bends the view through it, 1 being an air lens in water's (most of the view around it shows in it)
uniform float refraction : hint_range(0.0, 2.0) = 0.3;
// turns the view through it over, as a drop of water in air does; a lens of air in water leaves it the right way up
uniform bool inverted = false;
// how much further the blue is bent than the red at the rim
uniform float dispersion : hint_range(0.0, 0.5) = 0.12;
// 1 is a half sphere, smaller a flatter cap
uniform float cap_height : hint_range(0.2, 1.0) = 0.8;
uniform vec3 rim_tint = vec3(0.8, 0.93, 1.0);
uniform float rim_strength : hint_range(0.0, 2.0) = 0.7;
uniform float ring_darkness : hint_range(0.0, 1.0) = 0.45;
uniform float highlight_strength : hint_range(0.0, 3.0) = 1.0;
// towards the sun, in world space
uniform vec3 light_direction = vec3(0.198, 0.883, 0.425);
// the sun's, in sRGB, its largest component 1, times how bright it is
uniform vec3 light_colour = vec3(1.0);

varying float world_y;
// the bubble's centre, in view space
varying vec3 centre;
// its turn (cosine, sine) on the dome and how round it is, the same for all its life
varying vec3 shape;
// where on the bubble the sun glints, along its own axes as q is below
varying vec2 glint_at;
// how bright the near and far glints are, and how big the near one is, by where the sun is
varying vec3 glint_light;
// across the view, towards the sun, as long as the sun is far from straight ahead or behind
varying vec2 light_across;

vec2 to_screen(mat4 projection, vec3 direction) {
	vec4 clip = projection * vec4(direction, 0.0);
	return clip.xy / max(clip.w, 1e-3) * 0.5 + 0.5;
}

vec3 to_srgb(vec3 colour, bool srgb) {
	return srgb ? colour : pow(max(colour, 0.0), vec3(1.0 / 2.2));
}

vec3 from_srgb(vec3 colour, bool srgb) {
	return srgb ? colour : pow(max(colour, 0.0), vec3(2.2));
}

vec3 world_direction(mat4 inv_projection, mat4 inv_view, vec2 screen) {
	vec4 point = inv_projection * vec4(screen * 2.0 - 1.0, 0.5, 1.0);
	return mat3(inv_view) * normalize(point.xyz / point.w);
}

// how much of the surface shows through the water, looking along direction from height y
float surface_seen(vec3 direction, float y) {
	if (direction.y <= 0.0) {
		return 0.0;
	}
	return exp(-fog_density * max(surface_altitude - y, 0.0) / direction.y);
}

void vertex() {
	// face the camera, at the particle's size
	float size = length(MODEL_MATRIX[0].xyz);
	MODELVIEW_MATRIX = VIEW_MATRIX * mat4(INV_VIEW_MATRIX[0], INV_VIEW_MATRIX[1], INV_VIEW_MATRIX[2], MODEL_MATRIX[3]);
	MODELVIEW_MATRIX = MODELVIEW_MATRIX * mat4(vec4(size, 0.0, 0.0, 0.0), vec4(0.0, size, 0.0, 0.0),
			vec4(0.0, 0.0, size, 0.0), vec4(0.0, 0.0, 0.0, 1.0));
	MODELVIEW_NORMAL_MATRIX = mat3(MODELVIEW_MATRIX);
	world_y = MODEL_MATRIX[3].y + (mat3(INV_VIEW_MATRIX) * VERTEX * size).y;
	centre = (MODELVIEW_MATRIX * vec4(0.0, 0.0, 0.0, 1.0)).xyz;
	// the particle's slot and its lifetime, which the process material draws at random for each bubble it emits
	float id = float(INSTANCE_ID) * 1.618 + INSTANCE_CUSTOM.w * 31.7;
	vec2 random = fract(sin(vec2(id * 12.9898 + 4.1, id * 78.233 + 1.3)) * 43758.5453);
	shape = vec3(cos(random.x * TAU), sin(random.x * TAU), mix(0.8, 1.0, random.y));

	// the sun glints where the cap's normal, taken as facing the camera, is halfway between the way to the sun and the
	// way to the camera
	vec3 light = normalize(mat3(VIEW_MATRIX) * light_direction);
	vec3 to_camera = -normalize(centre);
	vec3 half_way = light + to_camera + vec3(0.0, 1e-4, 0.0);
	// on a cap of height cap_height, the normal leans out by as much as the half way does at
	// r = across * cap_height / sqrt(along^2 + (across * cap_height)^2); kept inside the rim
	float across = length(half_way.xy);
	float along = max(half_way.z, 1e-4);
	float lean = min(across * cap_height * inversesqrt(along * along + across * across * cap_height * cap_height), 0.85);
	// the normal's xy is turn's inverse of (q.x, q.y / shape.z), so q is along this
	mat2 turn = mat2(vec2(shape.x, -shape.y), vec2(shape.y, shape.x));
	vec2 towards = turn * half_way.xy;
	towards.y *= shape.z;
	glint_at = towards * (lean / max(length(towards), 1e-5));
	// with the sun ahead of the camera, past the bubble, the near glint is a dimmer, smaller one near the rim, and the far
	// one, of the light coming through the bubble, is brighter
	float facing = dot(light, to_camera);
	float lit = smoothstep(-0.4, 0.2, facing);
	glint_light = vec3(mix(0.55, 1.0, lit), mix(0.6, 0.3, smoothstep(-0.4, 0.4, facing)), mix(0.8, 1.0, lit));
	light_across = light.xy;
}

void fragment() {
	float camera_y = INV_VIEW_MATRIX[3].y;
	// they are drawn over the water, so from above it they would show through it
	if (world_y > surface_altitude || camera_y > surface_altitude) {
		discard;
	}
	// across the quad, x right and y up, then along the bubble's own axes, where it is a circle of radius 1
	vec2 p = (UV * 2.0 - 1.0) * vec2(1.0, -1.0);
	mat2 turn = mat2(vec2(shape.x, -shape.y), vec2(shape.y, shape.x));
	vec2 q = turn * p;
	q.y /= shape.z;
	float r = length(q);
	float pixel = fwidth(r);
	float coverage = 1.0 - smoothstep(1.0 - pixel, 1.0 + pixel, r);
	if (coverage <= 0.0) {
		discard;
	}

	// the cap's normal out into the water, in view space; it bulges away from the camera
	float height = sqrt(max(1.0 - r * r, 0.0)) * cap_height;
	vec3 normal = normalize(vec3(transpose(turn) * vec2(q.x, q.y / shape.z), -height));
	vec3 view = normalize(VERTEX);
	// out of the air into the water, the view through the bubble opens out
	vec3 through = refract(view, -normal, 1.0 / 1.333);
	vec2 bend = to_screen(PROJECTION_MATRIX, through) - to_screen(PROJECTION_MATRIX, view);
	vec2 middle = to_screen(PROJECTION_MATRIX, centre);
	vec2 from = SCREEN_UV;
	if (inverted) {
		from = 2.0 * middle - SCREEN_UV;
		bend = -bend;
	}
	bend *= refraction;
	float spread = dispersion * smoothstep(0.4, 1.0, r);
	vec2 look = clamp(from + bend, 0.0, 1.0);
	vec3 refracted = vec3(
			texture(scene, clamp(from + bend * (1.0 - spread), 0.0, 1.0)).r,
			texture(scene, look).g,
			texture(scene, clamp(from + bend * (1.0 + spread), 0.0, 1.0)).b);
	refracted = to_srgb(refracted, OUTPUT_IS_SRGB);
	vec2 near_by = clamp(middle + 3.0 * (SCREEN_UV - middle), 0.0, 1.0);
	vec3 around = to_srgb(textureLod(scene, near_by, 3.0).rgb, OUTPUT_IS_SRGB);
	around = mix(around, surface_colour,
			surface_seen(world_direction(INV_PROJECTION_MATRIX, INV_VIEW_MATRIX, near_by), camera_y));
	// near its edge the view through it is bent so far it is only a blur of the light around
	refracted = mix(refracted, around, smoothstep(0.6, 0.9, r));
	float surface = surface_seen(world_direction(INV_PROJECTION_MATRIX, INV_VIEW_MATRIX, look), camera_y);
	float brightness = clamp(dot(around, vec3(0.3, 0.5, 0.2)) * 3.0, 0.0, 1.0);

	// a few pixels wide however small the bubble, so the smallest ones still show a ring
	float rim_width = max(0.14, 2.0 * pixel);
	float rim = smoothstep(1.0 - rim_width, 1.0 - 0.3 * rim_width, r)
			* (1.0 - 0.4 * smoothstep(1.0 - 0.3 * rim_width, 1.0, r));
	float ring = smoothstep(0.68, 0.86, r) * (1.0 - rim);
	// brighter in the middle by about as much as the ring is darker, so on the whole it is as bright as the water around
	float lift = 0.25 * ring_darkness * (1.0 - smoothstep(0.4, 0.72, r));

	// the sun, glinting off the bubble towards the camera, and through it off its far side, opposite
	float glint_size = max(0.17 * glint_light.z, 1.5 * pixel);
	float glint = glint_light.x * exp(-dot(q - glint_at, q - glint_at) / (glint_size * glint_size));
	vec2 far_glint_at = -0.75 * glint_at;
	float far_glint_size = max(0.11, 1.2 * pixel);
	glint += glint_light.y * exp(-dot(q - far_glint_at, q - far_glint_at) / (far_glint_size * far_glint_size));
	// the rim is brighter on the side towards the sun, the more so the further the sun is to that side, and round the
	// whole of it about as bright as it would be without the sun
	vec3 rim_light = 0.8 + 0.5 * max(dot(p, light_across), 0.0) / max(length(p), 1e-3) * light_colour;
	// and paler where, curving towards the camera, it mirrors the way up to the surface
	vec3 mirrored = reflect(view, vec3(normal.xy, -normal.z));
	float sky = surface_seen(mat3(INV_VIEW_MATRIX) * mirrored, camera_y);

	// premultiplied: what is already drawn under the bubble shows through by how much is left of the surface
	float shade = 1.0 - ring_darkness * ring;
	// greyer as well as brighter, so it looks like clear air rather than more of the water's colour
	vec3 colour = (refracted * shade + lift * vec3(dot(refracted, vec3(0.3, 0.5, 0.2)))) * (1.0 - surface);
	float behind = min(surface * (shade + lift), 1.0);
	float rim_amount = clamp(rim * rim_strength, 0.0, 1.0);
	// silvery, as a mirror is, more than the colour of the water it shows
	vec3 reflected = mix(around, vec3(dot(around, vec3(0.3, 0.5, 0.2))), 0.5);
	reflected = mix(reflected, surface_colour, 0.5 * sky);
	colour = mix(colour, rim_tint * (0.25 + 1.5 * reflected) * rim_light, rim_amount);
	behind *= 1.0 - rim_amount;
	colour += glint * highlight_strength * (0.4 + 0.6 * brightness) * light_colour;
	ALBEDO = from_srgb(colour, OUTPUT_IS_SRGB) * coverage;
	ALPHA = coverage * (1.0 - behind);
}
