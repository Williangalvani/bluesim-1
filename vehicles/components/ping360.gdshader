shader_type canvas_item;

// Ping360Viewport.gd's scan: a row per degree of head angle, clockwise from the vehicle's forward,
// and a column per hundredth of the range
uniform sampler2D my_array : filter_nearest;

const float RINGS = 4.0;

// dark blue for silence, through cyan and yellow, to white, as ping360.gd colours the map
vec3 sonar_colour(float intensity) {
	float x = clamp(intensity, 0.0, 1.0) * 3.0;
	vec3 low = mix(vec3(0.02, 0.05, 0.3), vec3(0.0, 1.0, 1.0), clamp(x, 0.0, 1.0));
	vec3 mid = mix(low, vec3(1.0, 1.0, 0.0), clamp(x - 1.0, 0.0, 1.0));
	return mix(mid, vec3(1.0), clamp(x - 2.0, 0.0, 1.0));
}

void fragment() {
	// a circle as wide as the panel's shorter side, the vehicle's forward up
	vec2 size = 1.0 / TEXTURE_PIXEL_SIZE;
	vec2 position = (UV - 0.5) * 2.0 * size / min(size.x, size.y);
	float radius = length(position);
	float angle = mod(degrees(atan(position.x, -position.y)), 360.0);
	ivec2 texel = ivec2(min(int(radius * 100.0), 99), int(angle) % 360);
	vec3 colour = sonar_colour(texelFetch(my_array, texel, 0).r);
	float pixel = 2.0 / min(size.x, size.y);
	float ring = abs(fract(radius * RINGS + 0.5) - 0.5) / RINGS;
	colour = mix(colour, vec3(0.6), 0.4 * (1.0 - smoothstep(0.0, pixel, ring)));
	COLOR = vec4(colour, 1.0 - smoothstep(1.0 - pixel, 1.0, radius));
}
