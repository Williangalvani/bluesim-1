extends RigidBody3D

# Hydrodynamic drag on one tether section, split along the section's long (capsule) axis,
# which is its local Y. Quadratic drag for a cylinder in water: across the section
# (form drag, Cd ~1.2) and along it (skin friction, ~0.01 on the wetted surface). The old
# linear term is kept so the tether still settles at very low speeds.
const DRAG = 0.2  # N per m/s across the section, low-speed damping
const WATER_DENSITY = 1000.0
const DIAMETER = 0.012  # m, matches the visible tether mesh
const LENGTH = 0.145  # m, section spacing in levels/tether.gd
const NORMAL_CD = 1.2
const AXIAL_CF = 0.01
const NORMAL_K = 0.5 * WATER_DENSITY * NORMAL_CD * DIAMETER * LENGTH
const AXIAL_K = 0.5 * WATER_DENSITY * AXIAL_CF * PI * DIAMETER * LENGTH

func _physics_process(delta):
	var underwater = global_transform.origin.y < 3.6
	if underwater:
		var forward = self.global_transform.basis.y.normalized()
		var body_frame_speeds = self.linear_velocity
		var forward_speed = body_frame_speeds.dot(forward)
		var forward_velocity_component = forward * forward_speed
		var perpendicular_velocity_component = body_frame_speeds - forward_velocity_component
		var drag = -perpendicular_velocity_component * (DRAG + NORMAL_K * perpendicular_velocity_component.length())
		drag -= forward_velocity_component * AXIAL_K * abs(forward_speed)
		# drag can slow the section down but never push it backwards within one step
		apply_central_force(drag.limit_length(mass * body_frame_speeds.length() / delta))

	var under_surface = global_transform.origin.y < 3.53
	if under_surface:
		apply_central_force(Vector3(0,0.02,0))
