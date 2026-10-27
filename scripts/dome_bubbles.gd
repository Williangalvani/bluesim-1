extends GPUParticles3D
# Bubbles from the thrusters' wash that stick to the camera's dome, staying put however the vehicle moves until they
# pop

const BUBBLE_SHADER = preload("res://scripts/fx/dome_bubble.gdshader")
const DOME_DISTANCE = 0.055  # m, from the camera, just past its near plane
const RADIUS = 0.0065  # m, of the biggest bubble before process_material's scale
const SUN_ENERGY = 0.75  # the sun's light_energy at which the glints are as bright as the shader draws them
const SUN_SEARCH_INTERVAL = 1.0  # s, between searches for the sun while there is none

# the scene's sun, which the glints are of, searched for again if it goes
var _sun: DirectionalLight3D
var _sun_search_wait := 0.0  # s, until the next search for the sun


func _init():
	name = "DomeBubbles"
	process_material = ParticleProcessMaterial.new()
	# a patch of the dome the size of the camera's view, which from the camera looks the same as the curved dome
	process_material.emission_shape = ParticleProcessMaterial.EMISSION_SHAPE_BOX
	process_material.emission_box_extents = Vector3(0.08, 0.04, 0.002)
	process_material.initial_velocity_min = 0
	process_material.initial_velocity_max = 0
	process_material.gravity = Vector3.ZERO
	process_material.lifetime_randomness = 0.6
	process_material.scale_min = 0.1
	process_material.scale_max = 0.35
	var pop = Curve.new()
	pop.add_point(Vector2(0, 0))
	pop.add_point(Vector2(0.05, 1))
	pop.add_point(Vector2(0.9, 1))
	pop.add_point(Vector2(1, 0))
	process_material.scale_curve = CurveTexture.new()
	process_material.scale_curve.curve = pop
	var material = ShaderMaterial.new()
	material.shader = BUBBLE_SHADER
	# after the water and the light shafts (priority 1), which are further away
	material.render_priority = 2
	material.set_shader_parameter("surface_altitude", Globals.surface_altitude)
	# the shader turns it to face the camera and draws the bubble's sphere on it
	draw_pass_1 = QuadMesh.new()
	draw_pass_1.size = Vector2(2 * RADIUS, 2 * RADIUS)
	draw_pass_1.material = material
	local_coords = true
	amount = 64
	lifetime = 8.0
	emitting = false
	# in front of the camera it is a child of
	position = Vector3(0, 0, -DOME_DISTANCE)


func _ready():
	# the camera's underwater one, which it has until it surfaces, when the shader hides the bubbles
	var camera = get_parent() as Camera3D
	if camera and camera.environment:
		var density = camera.environment.fog_density if camera.environment.fog_enabled else 0.0
		draw_pass_1.material.set_shader_parameter("fog_density", density)


func _process(delta):
	var material = draw_pass_1.material as ShaderMaterial if draw_pass_1 else null
	if not material or not is_visible_in_tree():
		return
	if not is_instance_valid(_sun):
		_sun_search_wait -= delta
		if _sun_search_wait <= 0:
			_sun = _find_sun()
			# straight away when a found sun goes, otherwise not every frame
			_sun_search_wait = 0.0 if _sun else SUN_SEARCH_INTERVAL
	var direction = Vector3.UP
	var colour = Vector3.ONE
	if _sun and _sun.is_visible_in_tree():
		direction = _sun.global_transform.basis.z.normalized()
		var energy = clamp(_sun.light_energy / SUN_ENERGY, 0.0, 2.0)
		# its hue, as white as it can be, so a tinted sun does not dim the glints
		var hue = Vector3(_sun.light_color.r, _sun.light_color.g, _sun.light_color.b)
		var peak = max(hue.x, hue.y, hue.z)
		colour = hue / peak * energy if peak > 0.0 else Vector3.ZERO
	material.set_shader_parameter("light_direction", direction)
	material.set_shader_parameter("light_colour", colour)


# the first sun shining in the scene, if there is one
func _find_sun() -> DirectionalLight3D:
	for light in get_tree().root.find_children("*", "DirectionalLight3D", true, false):
		if light.is_visible_in_tree():
			return light
	return null


# intensity is how much air the thrusters are taking in, from 0 to 1
func blow(intensity: float):
	emitting = intensity >= 0.05
	if emitting:
		amount_ratio = min(intensity * 2, 1)
