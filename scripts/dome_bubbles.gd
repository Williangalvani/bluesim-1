extends GPUParticles3D
# Bubbles from the thrusters' wash that stick to the camera's dome, staying put however the vehicle moves until they
# pop

const ThrusterBubbles = preload("res://scripts/thruster_bubbles.gd")
const DOME_DISTANCE = 0.055  # m, from the camera, just past its near plane


func _init():
	if ThrusterBubbles.shared_mesh == null:
		ThrusterBubbles.make_shared_resources()
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
	# this close to the camera the thruster bubbles' few faces would show
	draw_pass_1 = ThrusterBubbles.shared_mesh.duplicate()
	draw_pass_1.radial_segments = 32
	draw_pass_1.rings = 16
	local_coords = true
	amount = 64
	lifetime = 8.0
	emitting = false
	# in front of the camera it is a child of
	position = Vector3(0, 0, -DOME_DISTANCE)


# intensity is how much air the thrusters are taking in, from 0 to 1
func blow(intensity: float):
	emitting = intensity >= 0.05
	if emitting:
		amount_ratio = min(intensity * 2, 1)
