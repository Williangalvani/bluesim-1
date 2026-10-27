extends Node
## Test harness (not part of the game): the sea level with the boat, the ROV held on a scripted path so its camera
## goes under -> surfaces -> stays out while the drops run off and dry -> dives -> surfaces -> dives while wet.
## Prints LENS lines; `--write-movie` it for a video. Args after `--`: mask (draw the water mask on black).
const BOAT = preload("res://vehicles/boat/boat.tscn")
# (time s, camera height above the surface m)
const PATH := [[0.0, -1.0], [2.5, -1.0], [3.5, 0.25], [13.0, 0.25], [14.0, -1.0], [16.0, -1.0], [17.0, 0.25],
		[19.5, 0.25], [20.0, -1.0], [22.0, -1.0]]
var rov: RigidBody3D
var cam: Camera3D
var drops: Control
var t := 0.0
var cam_offset := 0.0
var yaw := 0.0


func _enter_tree():
	if not Globals.active_level:
		ProjectSettings.load_resource_pack("res://levels/reef.pck")
		Globals.active_level = "res://custom_level.tscn"
		Globals.active_vehicle = "bluerovheavy"


func _ready():
	var base = load("res://levels/baselevel.tscn").instantiate()
	add_child(base)
	var boat = BOAT.instantiate(); boat.position = Vector3(-4.0, 0.0, -10.0); boat.rotation_degrees.y = 22.0
	base.get_node("ViewportCamera/SubViewport").add_child(boat)
	await get_tree().process_frame
	await get_tree().process_frame
	rov = get_tree().get_first_node_in_group("vehicles")
	rov.freeze_mode = RigidBody3D.FREEZE_MODE_KINEMATIC; rov.freeze = true
	cam = rov.get_node("Camera3D")
	cam_offset = cam.global_position.y - rov.global_position.y
	# turn the ROV so its camera looks at the boat
	var fl = rov.global_basis.inverse() * (-cam.global_basis.z)
	var d = Vector3(-4.0, 0, -10.0) - rov.global_position
	yaw = atan2(-d.x, -d.z) - atan2(-fl.x, -fl.z)
	drops = get_tree().get_first_node_in_group("lens_surfacing")
	print("LENS renderer=", RenderingServer.get_current_rendering_method(), " drops=", drops, " parent=", drops.get_parent().name if drops else "-",
		" source_viewport=", drops.source_viewport if drops else "-", " cam_offset=", cam_offset)
	if drops:
		await get_tree().process_frame
		print("LENS shader=", drops.material.shader.resource_path, " rect size=", drops.size, " global=", drops.get_global_rect(), " index=", drops.get_index(), " siblings=", drops.get_parent().get_children().map(func(c): return c.name))
		drops.surfaced_changed.connect(func(a): print("LENS t=%.2f surfaced=%s" % [t, a]))
		drops.dried.connect(func(): print("LENS t=%.2f dried" % t))
		drops.debug_mask = "mask" in OS.get_cmdline_user_args()
		drops.test_seed = 7   # same bursts in the normal and the mask run
		for a in OS.get_cmdline_user_args():
			if a.begins_with("seed="): drops.test_seed = int(a.substr(5))
			if a == "drops_only": drops.debug_part = 1


func _height_at(tt: float) -> float:
	for i in PATH.size() - 1:
		if tt <= PATH[i + 1][0]:
			var f = (tt - PATH[i][0]) / (PATH[i + 1][0] - PATH[i][0])
			return lerpf(PATH[i][1], PATH[i + 1][1], smoothstep(0.0, 1.0, f))
	return PATH[-1][1]


func _physics_process(delta):
	if not rov:
		return
	t += delta
	var h = _height_at(t)
	if h > 0.1: h += 0.03 * sin(t * 2.1)
	var y = Globals.surface_altitude + h - cam_offset
	# facing the boat, drifting slowly towards it
	rov.global_transform = Transform3D(Basis.from_euler(Vector3(0, yaw, 0)), Vector3(-0.05 * t, y, -0.12 * t))
	rov.linear_velocity = Vector3.ZERO; rov.angular_velocity = Vector3.ZERO
	if int(t * 100) % 50 == 0:
		print("LENS t=%.2f cam_h=%.3f wet=%s visible=%s age=%.2f" % [t, cam.global_position.y - Globals.surface_altitude,
			drops.is_wet() if drops else false, drops.visible if drops else false, drops.age if drops else 0.0])
	if t >= PATH[-1][0]:
		get_tree().quit()
