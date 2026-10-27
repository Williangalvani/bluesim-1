extends GPUParticles3D
# Bubbles from the thrusters' wash that stick to the camera's dome, staying put however the vehicle moves until they
# pop

const BUBBLE_SHADER = preload("res://scripts/fx/dome_bubble.gdshader")
const DOME_DISTANCE = 0.055  # m, from the camera, just past its near plane
const RADIUS = 0.0065  # m, of the biggest bubble before process_material's scale


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


# intensity is how much air the thrusters are taking in, from 0 to 1
func blow(intensity: float):
	emitting = intensity >= 0.05
	if emitting:
		amount_ratio = min(intensity * 2, 1)
