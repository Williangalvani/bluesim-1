extends Node
## Test harness (not part of the game): the reef level with the ROV held on a scripted path so its camera crosses the
## water surface, screenshots of what it shows, to check that the surfacing drops (scripts/surfacing_drops.gd) and the
## waterline split (scripts/camera_waterline.gd) agree, and whether the camera's near plane clips the water plane
## before the split engages. Prints SURF lines. Args after `--`: out=DIR (default user://surfacing), mode=shots|video
## (video: one pose, for `--write-movie`), speed=M_PER_S (video ascent speed), hold=S (extra wait after each shot),
## seed=N (the drops' test_seed), pose=NAME (only that pose).

# name: [pitch up, roll] in degrees
const POSES := {
	"up60": [60.0, 0.0],
	"up40_roll25": [40.0, 25.0],
	"level": [0.0, 0.0],
	"level_roll30": [0.0, 30.0],
	"down30": [-30.0, 0.0],
}
# camera heights above the surface (m) to take a shot at on the way up
const TARGETS := [-0.12, -0.09, -0.07, -0.05, -0.035, -0.02, -0.005, 0.01, 0.03, 0.06, 0.15]

var rov: RigidBody3D
var cam: Camera3D
var drops: Control
var waterline: Node
var out := "user://surfacing"
var mode := "shots"
var speed := 0.4
var hold_s := 0.0
var drop_seed := 7
var only_pose := ""
var base_pos: Vector3
var yaw := 0.0
var pitch_sign := 1.0
var hold: Transform3D
var holding := false
var cam_offset := 0.0
var goal_h := 0.0
var rate := 0.0
var t := 0.0
var next_print := 0.0


func _enter_tree() -> void:
	if not Globals.active_level:
		ProjectSettings.load_resource_pack("res://levels/reef.pck")
		Globals.active_level = "res://custom_level.tscn"
		Globals.active_vehicle = "bluerovheavy"
	for a in OS.get_cmdline_user_args():
		if a.begins_with("out="):
			out = a.substr(4)
		if a.begins_with("mode="):
			mode = a.substr(5)
		if a.begins_with("speed="):
			speed = float(a.substr(6))
		if a.begins_with("hold="):
			hold_s = float(a.substr(5))
		if a.begins_with("seed="):
			drop_seed = int(a.substr(5))
		if a.begins_with("pose="):
			only_pose = a.substr(5)
	DirAccess.make_dir_recursive_absolute(out)


func _ready() -> void:
	var base = load("res://levels/baselevel.tscn").instantiate()
	add_child(base)
	await get_tree().process_frame
	await get_tree().process_frame
	rov = get_tree().get_first_node_in_group("vehicles")
	rov.freeze_mode = RigidBody3D.FREEZE_MODE_KINEMATIC
	rov.freeze = true
	cam = rov.get_node("Camera3D")
	cam.current = true
	base_pos = rov.global_position
	yaw = rov.global_rotation.y
	# the camera looks along the body's +Z on the BlueROV, where a positive X rotation pitches it down
	var forward := rov.global_basis.inverse() * -cam.global_basis.z
	pitch_sign = -1.0 if forward.z > 0.0 else 1.0
	for c in rov.get_children():
		if "above_water" in c:
			waterline = c
	drops = get_tree().get_first_node_in_group("lens_surfacing")
	if drops:
		drops.test_seed = drop_seed
	print(
		"SURF renderer=",
		RenderingServer.get_current_rendering_method(),
		" surface_altitude=",
		Globals.surface_altitude,
		" drops=",
		drops,
		" waterline=",
		waterline,
		" near=",
		cam.near,
		" fov=",
		cam.fov,
		" keep_aspect=",
		cam.keep_aspect
	)
	if mode == "video":
		await _run_video(only_pose if only_pose else "up40_roll25")
	else:
		for p in POSES:
			if not only_pose or p == only_pose:
				await _run_pose(p)
	get_tree().quit()


func _h() -> float:
	return cam.global_position.y - Globals.surface_altitude


# ROV in the pose's orientation with its camera at h above the surface (negative = below), still
func _place(pose: String, h: float) -> void:
	var pr: Array = POSES[pose]
	var b := Basis.from_euler(Vector3(pitch_sign * deg_to_rad(pr[0]), yaw, deg_to_rad(pr[1])))
	rov.global_transform = Transform3D(b, base_pos)
	cam_offset = cam.global_position.y - rov.global_position.y
	hold = Transform3D(b, Vector3(base_pos.x, Globals.surface_altitude + h - cam_offset, base_pos.z))
	rov.global_transform = hold
	goal_h = h
	rate = 0.0
	holding = true


func _move_to(h: float, m_per_s: float) -> void:
	goal_h = h
	rate = m_per_s
	while absf(_h() - h) > 1e-4:
		await get_tree().physics_frame
	rate = 0.0


func _physics_process(delta: float) -> void:
	if not holding:
		return
	var cur: float = hold.origin.y + cam_offset - Globals.surface_altitude
	hold.origin.y += clampf(goal_h - cur, -rate * delta, rate * delta)
	rov.global_transform = hold
	rov.linear_velocity = Vector3.ZERO
	rov.angular_velocity = Vector3.ZERO
	if mode == "video":
		t += delta
		if t >= next_print:
			next_print += 0.1
			print("SURF t=%.2f h=%.3f %s" % [t, _h(), _state()])


func _flag(node: Object, prop: String) -> String:
	return str(node.get(prop)) if node else "-"


func _state() -> String:
	# the near plane's corners, as world-space height offsets from the camera
	var hh := cam.near * tan(deg_to_rad(cam.fov / 2.0))
	var size := cam.get_viewport().get_visible_rect().size
	var hw := hh * size.x / size.y
	var reach := -INF
	var low := INF
	for c in [
		Vector3(-hw, -hh, -cam.near),
		Vector3(hw, -hh, -cam.near),
		Vector3(-hw, hh, -cam.near),
		Vector3(hw, hh, -cam.near)
	]:
		var y: float = (cam.global_basis * c).y
		reach = maxf(reach, y)
		low = minf(low, y)
	var split: String = str(waterline.split.visible) if waterline else "-"
	var wet: String = str(drops.is_wet()) if drops else "-"
	return (
		"near=%.4f split=%s drops_visible=%s drops_wet=%s above=%s reach=%.3f low=%.3f"
		% [cam.near, split, _flag(drops, "visible"), wet, _flag(waterline, "above_water"), reach, low]
	)


func _shot(file: String, pose: String) -> void:
	await get_tree().process_frame
	await get_tree().process_frame
	await RenderingServer.frame_post_draw
	var p := out.path_join(file + ".png")
	get_viewport().get_texture().get_image().save_png(p)
	print("SURF shot %s pose=%s h=%.3f %s" % [p, pose, _h(), _state()])
	if hold_s > 0.0:
		await get_tree().create_timer(hold_s).timeout


func _mm(h: float) -> String:
	return "%s%03d" % ["p" if h >= 0.0 else "m", roundi(absf(h) * 1000.0)]


func _run_pose(pose: String) -> void:
	_place(pose, -0.35)
	for i in 30:
		await get_tree().process_frame
	for target in TARGETS:
		await _move_to(target, 0.25)
		await _shot("%s_h%s" % [pose, _mm(target)], pose)
	await get_tree().create_timer(1.5).timeout
	await _shot(pose + "_hold1p5", pose)
	await _move_to(-0.02, 0.5)
	await _shot(pose + "_redive_m020", pose)
	await _move_to(-0.10, 0.5)
	await _shot(pose + "_redive_m100", pose)
	await _move_to(-0.35, 0.5)


func _run_video(pose: String) -> void:
	_place(pose, -0.5)
	await get_tree().create_timer(1.0).timeout
	await _move_to(0.25, speed)
	await get_tree().create_timer(4.0).timeout
