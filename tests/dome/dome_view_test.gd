extends Node
## Test harness (not part of the game): the reef level, the ROV held still, screenshots of its camera dome from a camera
## outside the ROV and from the ROV camera, underwater and surfaced (lens drops on). Prints DOME lines.
## Args after `--`: out=DIR (default user://dome), hold=S (wait S seconds after each shot, for outside screenshots on web).
var rov: RigidBody3D
var cam: Camera3D
var ext: Camera3D
var out := "user://dome"
var hold_s := 0.0
var base_pos: Vector3
var yaw := 0.0
var hold: Transform3D
var holding := false


func _enter_tree():
	if not Globals.active_level:
		ProjectSettings.load_resource_pack("res://levels/reef.pck")
		Globals.active_level = "res://custom_level.tscn"
		Globals.active_vehicle = "bluerovheavy"
	for a in OS.get_cmdline_user_args():
		if a.begins_with("out="): out = a.substr(4)
		if a.begins_with("hold="): hold_s = float(a.substr(5))
	DirAccess.make_dir_recursive_absolute(out)


func _ready():
	var base = load("res://levels/baselevel.tscn").instantiate()
	add_child(base)
	await get_tree().process_frame
	await get_tree().process_frame
	rov = get_tree().get_first_node_in_group("vehicles")
	rov.freeze_mode = RigidBody3D.FREEZE_MODE_KINEMATIC; rov.freeze = true
	cam = rov.get_node("Camera3D")
	base_pos = rov.global_position
	print("DOME renderer=", RenderingServer.get_current_rendering_method(), " rov=", rov.get_path(), " cam near=", cam.near,
		" cull_mask=", cam.cull_mask, " fov=", cam.fov)
	_dump(rov)
	ext = Camera3D.new()
	ext.fov = 50; ext.near = 0.02
	cam.get_parent().get_parent().add_child(ext)
	await _run()
	get_tree().quit()


func _dump(n: Node):
	for c in n.find_children("*", "MeshInstance3D", true, false):
		var mi := c as MeshInstance3D
		if not mi.mesh: continue
		for i in mi.mesh.get_surface_count():
			var m = mi.get_active_material(i)
			var s = "DOME mesh=%s surf=%d layers=%d vis_range=%s/%s cast_shadow=%d mat=%s" % [rov.get_path_to(mi), i, mi.layers,
				mi.visibility_range_begin, mi.visibility_range_end, mi.cast_shadow, m.resource_name if m else "null"]
			if m is BaseMaterial3D:
				s += " transp=%d blend=%d albedo=%s metallic=%.3f rough=%.3f spec=%.2f cull=%d depth=%d prio=%d shading=%d refr=%s" % [
					m.transparency, m.blend_mode, m.albedo_color, m.metallic, m.roughness, m.metallic_specular, m.cull_mode,
					m.depth_draw_mode, m.render_priority, m.shading_mode, m.refraction_enabled]
				if m.distance_fade_mode: s += " dfade=%d %.2f..%.2f" % [m.distance_fade_mode, m.distance_fade_min_distance, m.distance_fade_max_distance]
			elif m: s += " class=%s" % m.get_class()
			print(s)


func _place(cam_h: float):
	# camera at cam_h above the surface (negative = below), ROV level, fixed heading
	var y = Globals.surface_altitude + cam_h - (cam.global_position.y - rov.global_position.y)
	hold = Transform3D(Basis.from_euler(Vector3(0, yaw, 0)), Vector3(base_pos.x, y, base_pos.z))
	holding = true
	rov.global_transform = hold


func _look_from(local_offset: Vector3):
	# the outside camera, at an offset in the ROV camera's frame (x right, y up, -z forward), looking at the dome
	var target = cam.global_position + cam.global_basis * Vector3(0, 0, 0.0)
	ext.global_position = cam.global_position + cam.global_basis * local_offset
	ext.look_at(target, Vector3.UP)
	ext.environment = cam.environment
	ext.current = true


func _physics_process(_delta):
	if holding:
		rov.global_transform = hold
		rov.linear_velocity = Vector3.ZERO; rov.angular_velocity = Vector3.ZERO


func _shot(name: String, frames := 20):
	for i in frames: await get_tree().process_frame
	await RenderingServer.frame_post_draw
	var img = get_viewport().get_texture().get_image()
	var p = out.path_join(name + ".png")
	img.save_png(p)
	print("DOME shot ", p, " cam_h=%.3f rov_cam_current=%s" % [cam.global_position.y - Globals.surface_altitude, cam.current])
	if hold_s > 0: await get_tree().create_timer(hold_s).timeout


func _run():
	_place(-1.5)
	cam.current = true
	await _shot("rov_underwater", 30)
	_look_from(Vector3(0.25, 0.08, -0.45))
	await _shot("outside_front_underwater")
	_look_from(Vector3(0.12, 0.3, -0.4))
	await _shot("outside_side_underwater")
	_look_from(Vector3(0.0, 0.0, -0.14))
	await _shot("outside_close_underwater")
	# surfaced: the lens drops burst onto the dome
	ext.current = false
	cam.current = true
	_place(0.25)
	await _shot("rov_surfaced_drops", 30)
	_look_from(Vector3(0.25, 0.08, -0.45))
	await _shot("outside_front_surfaced")
	ext.current = false
	cam.current = true
	await _shot("rov_surfaced_later", 30)
