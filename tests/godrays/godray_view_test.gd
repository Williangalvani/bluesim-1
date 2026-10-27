extends Node
## Test harness (not part of the game): the reef level with the ROV held still underwater, screenshots of the level's
## light shafts (UnderwaterFX/GodRays) from the ROV camera and from a free camera, looking up at the surface, sideways,
## and at the waterline. Prints GODRAY lines. Args after `--`: out=DIR (default user://godrays), hold=S (wait S seconds
## after each shot, for screenshots taken by a web driver), nowater (hide BlueSim's water plane, for diagnosis),
## prio=N (force the shafts' render priority, for diagnosis), speed=X (the shafts' shimmer speed), anim (also save a short burst of frames to show the animation speed).
var rov: RigidBody3D
var cam: Camera3D
var ext: Camera3D
var out := "user://godrays"
var hold_s := 0.0
var no_water := false
var anim := false
var prio := -1
var speed := -1.0
var base_pos: Vector3
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
		if a == "nowater": no_water = true
		if a == "anim": anim = true
		if a.begins_with("prio="): prio = int(a.substr(5))
		if a.begins_with("speed="): speed = float(a.substr(6))
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
	var water = base.get_node_or_null("ViewportCamera/SubViewport/water") as MeshInstance3D
	print("GODRAY renderer=", RenderingServer.get_current_rendering_method(), " surface_altitude=", Globals.surface_altitude)
	if water:
		var wm = water.material_override
		print("GODRAY water y=%.3f prio=%d aabb=%s" % [water.global_position.y, wm.render_priority, water.get_aabb()])
		if no_water: water.visible = false
	for gr in get_tree().root.find_children("GodRays", "", true, false):
		var g := gr as GeometryInstance3D
		var aabb: AABB = g.global_transform * g.get_aabb()
		var m = g.material_override
		if prio >= 0 and m: m.render_priority = prio
		if speed >= 0 and m is ShaderMaterial: m.set_shader_parameter("speed", speed)
		print("GODRAY node=%s visible_in_tree=%s y=%.3f..%.3f aabb=%s prio=%s vis_range_end=%s" % [g.get_path(), g.is_visible_in_tree(),
			aabb.position.y, aabb.end.y, aabb, m.render_priority if m else "-", g.visibility_range_end])
		if m is ShaderMaterial:
			for p in ["strength", "speed", "surface_y", "surface_fade", "near_fade_begin", "near_fade_end"]:
				var v = m.get_shader_parameter(p)
				if v != null: print("GODRAY param %s=%s" % [p, v])
	ext = Camera3D.new()
	ext.fov = 70; ext.near = 0.05; ext.far = 200
	cam.get_parent().get_parent().add_child(ext)
	await _run()
	get_tree().quit()


func _place(cam_h: float, pitch_deg := 0.0, yaw := 0.0):
	# ROV camera at cam_h above the surface (negative = below), pitched up by pitch_deg
	var b = Basis.from_euler(Vector3(deg_to_rad(-pitch_deg), yaw, 0))
	rov.global_transform = Transform3D(b, rov.global_position)
	var y = Globals.surface_altitude + cam_h - (cam.global_position.y - rov.global_position.y)
	hold = Transform3D(b, Vector3(base_pos.x, rov.global_position.y + (y - rov.global_position.y), base_pos.z))
	holding = true
	rov.global_transform = hold


func _free(pos: Vector3, target: Vector3):
	ext.global_position = pos
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
	var c: Camera3D = ext if ext.current else cam
	print("GODRAY shot ", p, " t=%.2f cam_y=%.3f cam_h=%.3f" % [Time.get_ticks_msec() / 1000.0, c.global_position.y if c else 0.0,
		(c.global_position.y - Globals.surface_altitude) if c else 0.0])
	if hold_s > 0: await get_tree().create_timer(hold_s).timeout


func _run():
	var s = Globals.surface_altitude
	var p0 = base_pos
	# ROV camera, 2 m down, level and pitched up
	cam.current = true
	_place(-2.0, 0)
	await _shot("rov_d2_level", 40)
	_place(-2.0, 35)
	await _shot("rov_d2_up35")
	_place(-0.6, 20)
	await _shot("rov_d06_up20")
	# free camera: sideways across the shafts from mid water, and up toward the surface
	_free(p0 + Vector3(-6, -1.5, 0), p0 + Vector3(4, -0.5, 0))
	await _shot("free_side_d1p5")
	_free(p0 + Vector3(-6, -1.5, 0), p0 + Vector3(4, 2.0, 0))
	await _shot("free_up_d1p5")
	_free(Vector3(p0.x - 4, s - 0.4, p0.z), Vector3(p0.x + 6, s - 0.2, p0.z))
	await _shot("free_justbelow")
	_free(Vector3(p0.x - 10, s - 5.0, p0.z + 2), Vector3(p0.x + 6, s - 0.5, p0.z))
	await _shot("free_deep_up")
	# steeply up at the shafts around the spawn, where they meet the surface
	_free(Vector3(p0.x + 1, s - 3.5, p0.z + 1), Vector3(p0.x + 7, s + 0.5, p0.z - 1))
	await _shot("free_meet_up")
	_free(Vector3(p0.x - 2, s - 2.0, p0.z + 3), Vector3(p0.x + 6, s - 0.3, p0.z - 3))
	await _shot("free_meet_side")
	# from above the water, looking down (the level hides its shafts above water)
	ext.current = false
	cam.current = true
	_place(0.3, -25)
	await _shot("rov_above_down25", 30)
	if anim:
		# the shafts' shimmer, from the deep view looking up: 30 frames 0.2 s apart (the ROV camera back underwater first,
		# as the level hides its shafts and fog while the ROV camera is above the water)
		_place(-2.0, 0)
		await _shot("anim_rov_under", 20)
		_free(Vector3(p0.x - 10, s - 5.0, p0.z + 2), Vector3(p0.x + 6, s - 0.5, p0.z))
		await _shot("anim_00", 20)
		for i in range(1, 30):
			await get_tree().create_timer(0.2).timeout
			await _shot("anim_%02d" % i, 1)
