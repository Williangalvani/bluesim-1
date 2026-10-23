extends "res://rope/section.gd"
# A/B variant of rope/section.gd for tether_test.gd --drag-axis=y: the section's long
# axis is its local Y (the capsule axis), so drag only the motion across it.


func _physics_process(_delta):
	var underwater = global_transform.origin.y < 3.6
	if underwater:
		var axis = global_transform.basis.y.normalized()
		var v = linear_velocity
		apply_central_force(-(v - axis * v.dot(axis)) * DRAG)

	var under_surface = global_transform.origin.y < 3.53
	if under_surface:
		apply_central_force(Vector3(0, 0.02, 0))
