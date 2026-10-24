extends RigidBody3D

const DRAG = 0.2

func _physics_process(delta):
	var underwater = global_transform.origin.y < 3.6
	if underwater: 
		var forward = self.transform.basis.z.normalized()
		var body_frame_speeds = self.linear_velocity
		var forward_speed = body_frame_speeds.dot(forward)
		var forward_velocity_component = forward * forward_speed
		var perpendicular_velocity_component = body_frame_speeds - forward_velocity_component
		apply_central_force(-perpendicular_velocity_component * DRAG)

	var under_surface = global_transform.origin.y < 3.53
	if under_surface:
		apply_central_force(Vector3(0,0.02,0))
