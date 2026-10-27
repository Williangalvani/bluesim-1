extends Node
## Test harness (not part of the game): the reef level, the ROV held still, its dome covered in the bubbles a
## ventilating thruster blows onto it, and screenshots of its camera at a few depths and headings. Prints BUBBLES lines.
## Args after `--`: out=DIR (default user://dome_bubbles), hold=S (wait S seconds after each shot), seed=N (where the
## bubbles land), count=N (the particles' amount), fill=S (seconds of bubbles blown onto the dome before each shot),
## p=NAME:VALUE[,NAME:VALUE] (shader parameters, to tune them), bench=1 (frame times with the bubbles shown and hidden
## instead of screenshots), frames=N (per bench run).
var rov: RigidBody3D
var cam: Camera3D
var bubbles: GPUParticles3D
var out := "user://dome_bubbles"
var hold_s := 0.0
var particle_seed := 7
var count := 0
var fill_s := 6.0
var bench := false
var bench_frames := 600
var params := {}
var base_pos: Vector3
var hold: Transform3D
var holding := false
var blowing := false
var light_energy := 0.0


func _enter_tree():
	if not Globals.active_level:
		ProjectSettings.load_resource_pack("res://levels/reef.pck")
		Globals.active_level = "res://custom_level.tscn"
		Globals.active_vehicle = "bluerovheavy"
	for a in OS.get_cmdline_user_args():
		if a.begins_with("out="):
			out = a.substr(4)
		elif a.begins_with("hold="):
			hold_s = float(a.substr(5))
		elif a.begins_with("seed="):
			particle_seed = int(a.substr(5))
		elif a.begins_with("count="):
			count = int(a.substr(6))
		elif a.begins_with("fill="):
			fill_s = float(a.substr(5))
		elif a.begins_with("bench="):
			bench = a.substr(6) not in ["0", "false"]
		elif a.begins_with("frames="):
			bench_frames = int(a.substr(7))
		elif a.begins_with("p="):
			for kv in a.substr(2).split(","):
				var parts = kv.split(":")
				var v = parts[1]
				params[parts[0]] = (v == "true") if v in ["true", "false"] else float(v)
	DirAccess.make_dir_recursive_absolute(out)
	# after the vehicle's own physics step, which stops the bubbles when its thrusters take in no air
	process_physics_priority = 100


func _ready():
	var base = load("res://levels/baselevel.tscn").instantiate()
	add_child(base)
	await get_tree().process_frame
	await get_tree().process_frame
	rov = get_tree().get_first_node_in_group("vehicles")
	rov.freeze_mode = RigidBody3D.FREEZE_MODE_KINEMATIC
	rov.freeze = true
	cam = rov.get_node("Camera3D")
	bubbles = _find_bubbles()
	base_pos = rov.global_position
	light_energy = rov.get_node("light1").light_energy
	if count > 0:
		bubbles.amount = count
	bubbles.use_fixed_seed = true
	bubbles.seed = particle_seed
	var material: ShaderMaterial = bubbles.draw_pass_1.material
	for k in params:
		material.set_shader_parameter(k, params[k])
	print(
		"BUBBLES renderer=",
		RenderingServer.get_current_rendering_method(),
		" cam near=",
		cam.near,
		" fov=",
		cam.fov,
		" amount=",
		bubbles.amount,
		" seed=",
		bubbles.seed,
		" light_energy=",
		light_energy,
		" params=",
		params
	)
	cam.current = true
	if bench:
		await _bench()
	else:
		await _run()
	get_tree().quit()


# by its name, or by its script where it has none, as in the code before the bubbles had their own shader
func _find_bubbles() -> GPUParticles3D:
	var named = cam.get_node_or_null("DomeBubbles")
	if named:
		return named
	for child in cam.get_children():
		var script = child.get_script()
		if script and script.resource_path.ends_with("dome_bubbles.gd"):
			return child
	return null


# the camera depth m below the surface, pitched up by pitch degrees, at a fixed heading
func _place(depth: float, pitch := 0.0):
	if "camera_tilt" in rov:
		rov.camera_tilt = 0.0
	cam.rotation.x = 0.0
	var cam_basis = Basis.from_euler(Vector3(deg_to_rad(pitch), 0, 0))
	var rov_basis = cam_basis * cam.transform.basis.inverse()
	var cam_pos = Vector3(base_pos.x, Globals.surface_altitude - depth, base_pos.z)
	hold = Transform3D(rov_basis, cam_pos - rov_basis * cam.transform.origin)
	holding = true
	rov.global_transform = hold


func _physics_process(_delta):
	if holding:
		rov.global_transform = hold
		rov.linear_velocity = Vector3.ZERO
		rov.angular_velocity = Vector3.ZERO
	if blowing:
		bubbles.blow(1.0)


# the same bubbles each time: as many as fill_s seconds of blowing leave on the dome, then held still
func _cover_dome():
	bubbles.speed_scale = 1.0
	bubbles.preprocess = fill_s
	blowing = true
	await get_tree().physics_frame
	bubbles.restart()
	# drawn frames too: on the web many physics ticks can run in one frame, before the particles were processed
	var start = Engine.get_physics_frames()
	var drawn := 0
	while drawn < 10 or Engine.get_physics_frames() - start < 10:
		await get_tree().process_frame
		await RenderingServer.frame_post_draw
		drawn += 1
	blowing = false
	bubbles.speed_scale = 0.0


func _shot(name: String):
	await _cover_dome()
	for i in 20:
		await get_tree().process_frame
	await RenderingServer.frame_post_draw
	var img = get_viewport().get_texture().get_image()
	var p = out.path_join(name + ".png")
	img.save_png(p)
	print("BUBBLES shot ", p, " cam_h=%.3f" % (cam.global_position.y - Globals.surface_altitude))
	var w = img.get_width()
	var h = img.get_height()
	var crop = img.get_region(Rect2i(w / 2 - 240, h / 2 - 135, 480, 270))
	crop.resize(960, 540, Image.INTERPOLATE_NEAREST)
	p = out.path_join(name + "_crop.png")
	crop.save_png(p)
	print("BUBBLES shot ", p)
	if hold_s > 0:
		await get_tree().create_timer(hold_s).timeout


func _run():
	_place(0.4)
	await _shot("level_0.4m")
	_place(1.5)
	await _shot("level_1.5m")
	_place(1.5, 30)
	await _shot("up30_1.5m")
	_place(3.0, -30)
	rov.set_spotlights_energy(0)
	await _shot("down30_3m_lights_off")
	# the most the pilot's lights control gives
	rov.set_spotlights_energy(5)
	await _shot("down30_3m_lights_on")
	rov.set_spotlights_energy(light_energy)


func _bench():
	DisplayServer.window_set_vsync_mode(DisplayServer.VSYNC_DISABLED)
	Engine.max_fps = 0
	var vp = get_viewport().get_viewport_rid()
	RenderingServer.viewport_set_measure_render_time(vp, true)
	_place(1.5)
	await _cover_dome()
	# keep the dome covered, popping and replacing them as in the game
	bubbles.speed_scale = 1.0
	blowing = true
	# past the frames where the level was still loading, which the timings would include
	for i in 240:
		await get_tree().process_frame
	for run in 4:
		for shown in [true, false]:
			bubbles.visible = shown
			for i in 30:
				await get_tree().process_frame
			var gpu := 0.0
			var cpu := 0.0
			var process := 0.0
			var start = Time.get_ticks_usec()
			for i in bench_frames:
				await get_tree().process_frame
				gpu += RenderingServer.viewport_get_measured_render_time_gpu(vp)
				cpu += RenderingServer.viewport_get_measured_render_time_cpu(vp)
				process += Performance.get_monitor(Performance.TIME_PROCESS) * 1000.0
			var frame = (Time.get_ticks_usec() - start) / 1000.0 / bench_frames
			print(
				(
					"BUBBLES bench run=%d visible=%s amount=%d gpu_ms=%.3f cpu_ms=%.3f process_ms=%.3f frame_ms=%.3f"
					% [
						run,
						shown,
						bubbles.amount,
						gpu / bench_frames,
						cpu / bench_frames,
						process / bench_frames,
						frame
					]
				)
			)
	bubbles.visible = true
