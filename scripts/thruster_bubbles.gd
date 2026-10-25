extends GPUParticles3D
# Bubbles that show the pilot a thruster is sucking in air at the surface, blown toward the camera from a thruster
# beside it, since the ones taking in the air are out of its view

const BUBBLE_SHADER = """
shader_type spatial;
render_mode unshaded, depth_draw_never, cull_back;

uniform float surface_altitude;
varying float world_y;

void vertex() {
	world_y = (MODEL_MATRIX * vec4(VERTEX, 1.0)).y;
}

void fragment() {
	// they are drawn after the water, so from above it they would show through it at full strength
	if (world_y > surface_altitude || INV_VIEW_MATRIX[3].y > surface_altitude) {
		discard;
	}
	// a bubble shows mostly as a bright rim, so it reads against both the sky and the deep water
	float rim = 1.0 - abs(dot(NORMAL, VIEW));
	ALBEDO = vec3(0.9, 0.97, 1.0);
	ALPHA = 0.2 + 0.8 * rim * rim;
}
"""

static var shared_process_material: ParticleProcessMaterial
static var shared_mesh: SphereMesh


static func make_shared_resources():
	shared_process_material = ParticleProcessMaterial.new()
	shared_process_material.emission_shape = ParticleProcessMaterial.EMISSION_SHAPE_SPHERE
	shared_process_material.emission_sphere_radius = 0.05
	shared_process_material.direction = Vector3(0, 0, -1)
	shared_process_material.spread = 30
	shared_process_material.initial_velocity_min = 0.8
	shared_process_material.initial_velocity_max = 1.6
	# drag proportional to speed, as in water: the wash dies out within a quarter of a metre, and buoyancy over drag gives
	# the bubbles a rise speed of ~0.3 m/s, as millimetre-sized bubbles have
	shared_process_material.particle_flag_damping_as_friction = true
	shared_process_material.damping_min = 3
	shared_process_material.damping_max = 5
	shared_process_material.gravity = Vector3(0, 1.2, 0)
	shared_process_material.lifetime_randomness = 0.5
	# the turbulent wash scatters them, instead of rising in a single column
	shared_process_material.turbulence_enabled = true
	shared_process_material.turbulence_noise_scale = 1.0
	shared_process_material.turbulence_influence_min = 0.03
	shared_process_material.turbulence_influence_max = 0.1
	shared_process_material.scale_min = 0.21
	shared_process_material.scale_max = 1.12
	var shrink = Curve.new()
	shrink.add_point(Vector2(0, 1))
	shrink.add_point(Vector2(0.8, 1))
	shrink.add_point(Vector2(1, 0))
	var shrink_texture = CurveTexture.new()
	shrink_texture.curve = shrink
	shared_process_material.scale_curve = shrink_texture

	var shader = Shader.new()
	shader.code = BUBBLE_SHADER
	var material = ShaderMaterial.new()
	material.shader = shader
	# transparent objects are sorted by their centres, and the water plane's, tens of metres across, is often nearer
	# than the bubbles; they are always under it, so drawing them after it is always right
	material.render_priority = 1
	material.set_shader_parameter("surface_altitude", Globals.surface_altitude)
	shared_mesh = SphereMesh.new()
	shared_mesh.radius = 0.005
	shared_mesh.height = 0.01
	shared_mesh.radial_segments = 12
	shared_mesh.rings = 6
	shared_mesh.material = material


# direction is where in the vehicle's frame the thruster blows them
func _init(thruster: Node3D, direction: Vector3):
	if shared_process_material == null:
		make_shared_resources()
	process_material = shared_process_material
	draw_pass_1 = shared_mesh
	amount = 800
	# long enough to rise to the surface, where the shader hides them
	lifetime = 3.0
	emitting = false
	# not a child of the thruster node, whose scale would shrink the bubbles and their speed
	transform = Transform3D(Basis.looking_at(direction), thruster.position)


# intensity is how much air the thrusters are taking in, from 0 to 1
func blow(intensity: float):
	emitting = intensity >= 0.05
	if emitting:
		# a thruster half in the air already shows the most bubbles, since they are there to be noticed
		amount_ratio = min(intensity * 2, 1)
