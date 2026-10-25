extends CanvasLayer
# What the vehicle's camera shows as it crosses the water surface: the sky and no water fog above it, water drops
# drying off its lens as it comes out, and whitewater on each crossing

const DRY_TIME = 4.0  # s, for the last drop on the lens to dry
const SPLASH_TIME = 0.4  # s
# m, so a camera bobbing at the surface does not cross it on every ripple
const HYSTERESIS = 0.005

# water on the lens, drawn over the image rather than refracting it, since the compatibility renderer gives a canvas
# in a SubViewport a blank screen texture: drops that run down around the dome or dry off one by one, and whitewater on
# a splash
const LensDrops = preload("res://scripts/lens_drops.gd")

const LENS_WATER_SHADER = """
shader_type canvas_item;

uniform float wetness = 0.0;
// s since the camera came out of the water
uniform float age = 0.0;
uniform float splash = 0.0;
// where the small drops of this time out of the water land
uniform vec2 seed;
uniform mat3 view_to_dome;
uniform float tan_half_fov;
// the dome's up, and the camera's forward along its horizon, as the camera came out: the drops run down its meridians
uniform vec3 dome_up;
uniform vec3 dome_forward;
// around the dome, down it, radius, and the length of its tail if it ran
uniform vec4 drops[%d];
uniform int drop_count;
uniform sampler2D trails : filter_linear;
uniform float map_around;
uniform float dry_time;

vec2 hash(vec2 p) {
	p = vec2(dot(p, vec2(127.1, 311.7)), dot(p, vec2(269.5, 183.3)));
	return fract(sin(p) * 43758.5453);
}

// how far out of a drop d is, 1 at its edge, d being from -1 to 1 across its head: water clinging to the dome sags into
// a wider bottom with an uneven edge, and a running drop draws a tail behind its head, as long as given
float drop_shape(vec2 d, float tail, float wobble) {
	if (d.y < 0.0 && tail > 0.0) {
		float t = -d.y / tail;
		return t >= 1.0 ? 2.0 : abs(d.x) / max(pow(1.0 - t, 2.5) * (1.0 + 2.0 * t), 0.001);
	}
	float a = atan(d.y, d.x);
	float edge = 1.0 + 0.07 * sin(3.0 * a + wobble * 6.28) + 0.04 * sin(5.0 * a + wobble * 17.0);
	return length(vec2(d.x * (1.0 - 0.2 * d.y), d.y)) / edge;
}

// a drop is a lens, showing what is around it upside down: the dark water in its top half and the sky in its bottom
// half, with the light it gathers on its bottom edge, a dark edge, and a glint of the sun
vec4 shade(vec2 d, float l) {
	if (l > 1.0) {
		return vec4(0.0);
	}
	float rim = smoothstep(0.8, 1.0, l);
	// a tail is too thin to show much
	float inverted = smoothstep(0.5, -0.7, d.y) * smoothstep(-2.0, -1.0, d.y);
	float gathered = smoothstep(0.45, 0.85, l) * (1.0 - rim) * smoothstep(0.1, 0.7, d.y);
	float glint = 0.8 * smoothstep(0.12, 0.0, length(d - vec2(-0.25, -0.45)));
	vec3 color = mix(vec3(0.7, 0.85, 0.97), vec3(0.03, 0.13, 0.18), 0.8 * inverted);
	color = mix(color, vec3(0.02, 0.07, 0.1), 0.6 * rim) + 0.5 * gathered;
	return vec4(min(color + glint, 1.0), 0.15 + 0.3 * inverted + 0.35 * rim + 0.3 * gathered + 0.6 * glint);
}

// the small drop in this cell of the dome's grid, which clings where it landed until it dries; across the dome is
// squeezed by how close the meridians get toward its top
vec4 clinging_drop(vec2 g, float squeeze) {
	vec2 cell = floor(g);
	vec2 r = hash(cell);
	// each drop dries at its own wetness, so they vanish one by one, and only some cells get one
	if (r.x > wetness * 0.4) {
		return vec4(0.0);
	}
	vec2 d = (fract(g) - (0.3 + 0.4 * hash(cell + 17.0))) / (0.08 + 0.15 * r.y);
	d.x *= squeeze;
	return shade(d, drop_shape(d, 0.0, r.x));
}

// the bigger drops, which the script runs down the dome
vec4 big_drop(vec2 m, float squeeze) {
	vec4 found = vec4(0.0);
	for (int i = 0; i < drop_count; i++) {
		vec4 drop = drops[i];
		vec2 d = vec2((m.x - drop.x) * squeeze, m.y - drop.y) / drop.z;
		vec4 shaded = shade(d, drop_shape(d, drop.w, fract(drop.z * 997.0)));
		if (shaded.a > found.a) {
			found = shaded;
		}
	}
	return found;
}

// a running drop leaves a line of tiny beads behind it, which dry soon after
vec4 bead(vec2 g, float since) {
	vec2 cell = floor(g);
	vec2 r = hash(cell + 5.0);
	if (r.x > 0.4 || since > 0.5 + 1.5 * r.y) {
		return vec4(0.0);
	}
	vec2 d = (fract(g) - 0.5) / (0.15 + 0.15 * r.y);
	return shade(d, length(d));
}

void fragment() {
	float aspect = SCREEN_PIXEL_SIZE.y / SCREEN_PIXEL_SIZE.x;
	vec2 ndc = SCREEN_UV * 2.0 - 1.0;
	vec3 dome = normalize(view_to_dome * vec3(ndc.x * tan_half_fov * aspect, -ndc.y * tan_half_fov, -1.0));
	// where on the dome this is: which meridian, and how far down it from the top, in radians
	float down = acos(clamp(dot(dome, dome_up), -1.0, 1.0));
	float around = atan(dot(dome, cross(dome_up, dome_forward)), dot(dome, dome_forward));
	vec2 m = vec2(around, down);
	float squeeze = sin(down);
	vec2 on_map = vec2(around / map_around * 0.5 + 0.5, down / PI);
	vec2 trail = abs(around) < map_around ? texture(trails, on_map).rg : vec2(0.0);
	// s since a drop last ran here, its trail drying from where it started
	float since = age - trail.g * dry_time;
	vec4 water = big_drop(m, squeeze);
	// which wipes the small drops off its way
	if (trail.r < 0.15) {
		vec4 small = clinging_drop((m + seed) * 8.0, squeeze);
		water = small.a > water.a ? small : water;
	} else if (trail.r > 0.6) {
		vec4 beaded = bead(vec2(around * squeeze, down) * 70.0, since);
		vec4 film = vec4(0.85, 0.95, 1.0, 0.1 * smoothstep(0.6, 0.9, trail.r) * clamp(1.0 - since / 1.5, 0.0, 1.0));
		beaded = beaded.a > film.a ? beaded : film;
		water = beaded.a > water.a ? beaded : water;
	}
	float foam = 0.5 + 0.5 * sin(SCREEN_UV.y * 40.0 + TIME * 25.0) * sin(SCREEN_UV.x * 30.0 - TIME * 20.0);
	float whitewater = splash * (0.25 + 0.35 * foam);
	COLOR = water.a > whitewater ? water : vec4(0.9, 0.96, 1.0, whitewater);
}
"""

var camera: Camera3D
var underwater_environment: Environment
var surface_environment = make_surface_environment()
var above_water = false
var wetness = 0.0
var splash = 0.0
var overlay = ColorRect.new()
var lens_drops = LensDrops.new()


# the levels' own sun lights the water, too dim for BlueSim's physical sky, so above it the camera gets a fixed one
static func make_surface_environment() -> Environment:
	var sky_material = ProceduralSkyMaterial.new()
	sky_material.sky_top_color = Color(0.25, 0.5, 0.85)
	sky_material.sky_horizon_color = Color(0.72, 0.82, 0.9)
	sky_material.ground_horizon_color = Color(0.72, 0.82, 0.9)
	sky_material.ground_bottom_color = Color(0.1, 0.3, 0.4)
	var environment = Environment.new()
	environment.background_mode = Environment.BG_SKY
	environment.sky = Sky.new()
	environment.sky.sky_material = sky_material
	environment.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	return environment


func _init(vehicle_camera: Camera3D):
	camera = vehicle_camera
	underwater_environment = camera.environment
	var shader = Shader.new()
	# the material only compiles the code it has when it gets the shader, not code set afterwards
	shader.code = LENS_WATER_SHADER % LensDrops.MAX_DROPS
	overlay.material = ShaderMaterial.new()
	overlay.material.shader = shader
	overlay.set_anchors_preset(Control.PRESET_FULL_RECT)
	overlay.mouse_filter = Control.MOUSE_FILTER_IGNORE
	overlay.visible = false
	add_child(overlay)


func _ready():
	overlay.material.set_shader_parameter("trails", lens_drops.texture)
	overlay.material.set_shader_parameter("map_around", LensDrops.MAP_AROUND)
	overlay.material.set_shader_parameter("dry_time", DRY_TIME)


func _process(delta):
	var height = camera.global_position.y - Globals.surface_altitude
	if above_water != (height > (-HYSTERESIS if above_water else HYSTERESIS)):
		above_water = not above_water
		camera.environment = surface_environment if above_water else underwater_environment
		# a level's underwater effects, like light shafts, are meshes that would show in the air
		get_tree().call_group("underwater_fx", "set_fx", not above_water)
		splash = 1.0
		# coming out the lens is wet, and going back in the water washes the drops off
		wetness = 1.0 if above_water else 0.0
		overlay.material.set_shader_parameter("seed", Vector2(randf(), randf()) * 10)
		# down as the camera came out, so the drops do not jump about as the vehicle rocks
		var dome_up = (camera.get_parent_node_3d().global_transform.basis.inverse() * Vector3.UP).normalized()
		var forward = view_to_dome() * Vector3.FORWARD
		overlay.material.set_shader_parameter("dome_up", dome_up)
		overlay.material.set_shader_parameter("dome_forward", (forward - dome_up * forward.dot(dome_up)).normalized())
		if above_water:
			# from the top of the image to its bottom, down the dome
			var tan_half_fov = tan(deg_to_rad(camera.fov / 2))
			var top = (view_to_dome() * Vector3(0, tan_half_fov, -1)).normalized()
			var bottom = (view_to_dome() * Vector3(0, -tan_half_fov, -1)).normalized()
			lens_drops.wet(acos(top.dot(dome_up)), acos(bottom.dot(dome_up)))
	wetness = max(wetness - delta / DRY_TIME, 0)
	splash = max(splash - delta / SPLASH_TIME, 0)
	# blending over every pixel of the image is not free, so only while there is water to draw
	overlay.visible = wetness > 0 or splash > 0
	if overlay.visible:
		overlay.material.set_shader_parameter("wetness", wetness)
		overlay.material.set_shader_parameter("age", (1 - wetness) * DRY_TIME)
		overlay.material.set_shader_parameter("splash", splash)
		overlay.material.set_shader_parameter("view_to_dome", view_to_dome())
		overlay.material.set_shader_parameter("tan_half_fov", tan(deg_to_rad(camera.fov / 2)))
		lens_drops.step(delta, wetness)
		overlay.material.set_shader_parameter("drops", lens_drops.packed())
		overlay.material.set_shader_parameter("drop_count", lens_drops.drops.size())


# the dome is on the vehicle, which the camera tilts inside of
func view_to_dome() -> Basis:
	return camera.get_parent_node_3d().global_transform.basis.inverse() * camera.global_transform.basis

