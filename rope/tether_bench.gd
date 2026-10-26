extends Node
# Benchmark for the tether alternatives, run in the web build with ?tetherbench&tether=MODE&... or natively with
#   godot4.7 --headless --fixed-fps 100 --path . res://levels/baselevel.tscn -- tetherbench tether=MODE ...
# MODE: none | pbd | analytic | lumped | rigid | softbody. Other keys: n (segments / rigid links), sub, iter,
# every (solve every N ticks), len (m), cam=chase, secs (default 30), drive=0 to leave the vehicle alone.
# It flies the vehicle through a fixed script of forces and publishes window.tetherBench (and prints a
# TETHERBENCH line) with per-tick solve time, Godot's physics/process times, frame times and the load on the vehicle.

var params := {}
var rov: RigidBody3D
var impl: Node
var t := 0.0
var secs := 30.0
var drive := true
var done := false
var cam: Camera3D
var last_frame_usec := 0
var frame_us := []
var phys_ms := []
var proc_ms := []
var solve_us := []
var render_us := []
var steps_per_frame := []
var last_phys_frames := 0
var phase_stats := {}
var max_force := 0.0
var start_usec := 0
var tick_us := []  # whole physics ticks (all scripts + Jolt's step), from back-to-back ticks in one frame
var _last_tick_usec := 0
var _last_tick_frame := -1


static func parse_params() -> Dictionary:
	var out = {}
	var items = []
	if OS.has_feature("web"):
		var q = str(JavaScriptBridge.eval("location.search.substring(1)"))
		items = q.split("&", false)
	else:
		items = OS.get_cmdline_user_args()
	for item in items:
		var kv = str(item).split("=", true, 1)
		out[kv[0]] = kv[1] if kv.size() > 1 else "1"
	return out


func _p(key, default):
	return type_convert(params[key], typeof(default)) if params.has(key) else default


func _ready():
	params = parse_params()
	process_physics_priority = -1000  # first in every tick, so tick starts bracket a whole tick
	secs = _p("secs", 30.0)
	drive = _p("drive", 1) == 1
	rov = get_tree().get_first_node_in_group("vehicles")
	if typeof(Globals.active_vehicle) == TYPE_STRING:
		Globals.active_vehicle = rov  # native runs skip the autopilot link, but the cameras still follow the vehicle
	var old = get_tree().root.find_child("tether", true, false)
	var holder = old.get_parent()
	var xform = old.transform
	holder.remove_child(old)
	old.free()
	var mode = _p("tether", "pbd")
	var node: Node3D
	match mode:
		"none":
			node = null
		"pbd":
			node = Node3D.new()
			node.set_script(load("res://rope/pbd_tether.gd"))
		"analytic", "lumped":
			node = Node3D.new()
			node.set_script(load("res://rope/analytic_tether.gd"))
			node.draw_shape = mode == "analytic"
		"rigid":
			node = Node3D.new()
			node.set_script(load("res://levels/tether.gd"))
			node.straight_loops = _p("n", 200)
			node.coiled_loops = 0
		"softbody":
			node = Node3D.new()
			node.set_script(load("res://rope/softbody_tether.gd"))
			node.precision = _p("iter", 5)
	if node and mode != "rigid":
		node.length = _p("len", 30.0)
		node.segments = _p("n", 40)
		if mode != "softbody":
			node.anchor_distance = _p("anchor", 10.0)
		if mode == "pbd":
			node.substeps = _p("sub", 2)
			node.iterations = _p("iter", 4)
			node.update_every = _p("every", 1)
			node.render_subdivisions = _p("rsub", 4)
	if node:
		node.name = "tether"
		node.transform = xform  # before _ready, which lays the tether out from here
		holder.add_child(node)
	impl = node
	if _p("cam", "") == "chase":
		cam = Camera3D.new()
		cam.far = 200
		holder.add_child(cam)
		cam.top_level = true
		cam.make_current()
	start_usec = Time.get_ticks_usec()
	last_phys_frames = Engine.get_physics_frames()
	print("TETHERBENCH start mode=%s params=%s rov_forward=%s tether_away=%s" % [mode, params, rov.global_basis.z, (holder.global_transform * xform.origin - rov.global_position).normalized()])


func _phase() -> String:
	if t < 2: return "settle"
	if t < 6: return "dive"
	if t < 16: return "forward"
	if t < 24: return "turn"
	return "coast"


func _physics_process(delta):
	if done:
		return
	var now = Time.get_ticks_usec()
	var frame = Engine.get_process_frames()
	if frame == _last_tick_frame and t > 2.0:
		tick_us.append(now - _last_tick_usec)
	_last_tick_usec = now
	_last_tick_frame = frame
	t += delta
	var fwd = rov.global_basis.z
	var up = rov.global_basis.y
	var ph = _phase()
	if drive:
		match ph:
			"dive":
				rov.apply_central_force(-up * 30)
			"forward":
				rov.apply_central_force(fwd * 40 - up * 8)
			"turn":
				rov.apply_central_force(fwd * 20 - up * 8)
				rov.apply_torque(up * 15)
	var f = impl.get("force") if impl else null
	var fm = f.length() if f is Vector3 else 0.0
	max_force = max(max_force, fm)
	var s = phase_stats.get(ph, {"n": 0, "speed": 0.0, "force": 0.0, "depth": 0.0, "fmax": 0.0, "f2": 0.0, "dist": 0.0})
	s.n += 1
	s.speed += rov.linear_velocity.length()
	s.force += fm
	s.f2 += fm * fm
	s.fmax = max(s.fmax, fm)
	if impl and impl.get("anchor") != null:
		s.dist = rov.global_position.distance_to(impl.anchor)
	s.depth += Globals.surface_altitude - rov.global_position.y
	phase_stats[ph] = s
	if impl and impl.get("solve_usec") != null:
		solve_us.append(impl.solve_usec)
	if t >= secs:
		_finish()


func _process(_delta):
	var now = Time.get_ticks_usec()
	if last_frame_usec and not done and t > 2.0:
		frame_us.append(now - last_frame_usec)
		phys_ms.append(Performance.get_monitor(Performance.TIME_PHYSICS_PROCESS) * 1000.0)
		proc_ms.append(Performance.get_monitor(Performance.TIME_PROCESS) * 1000.0)
		var pf = Engine.get_physics_frames()
		steps_per_frame.append(pf - last_phys_frames)
		if impl and impl.get("render_usec") != null:
			render_us.append(impl.render_usec)
	last_phys_frames = Engine.get_physics_frames()
	last_frame_usec = now
	if cam:
		var fwd = rov.global_basis.z
		var target = rov.global_position
		cam.global_position = target - fwd * 2.5 + Vector3.UP * 1.6 + rov.global_basis.x * 2.0
		cam.look_at(target - fwd * 1.0, Vector3.UP)


static func _stats(a: Array) -> Dictionary:
	if a.is_empty():
		return {}
	var b = a.duplicate()
	b.sort()
	var sum = 0.0
	for e in b:
		sum += e
	return {"mean": sum / b.size(), "p50": b[b.size() / 2], "p95": b[int(b.size() * 0.95)], "max": b[-1], "n": b.size()}


func _finish():
	done = true
	var phases = {}
	for k in phase_stats:
		var s = phase_stats[k]
		var mean_f = s.force / s.n
		phases[k] = {"speed": s.speed / s.n, "force": mean_f, "depth": s.depth / s.n, "force_max": s.fmax,
			"force_std": sqrt(max(0.0, s.f2 / s.n - mean_f * mean_f)), "anchor_dist_end": s.dist}
	var result = {
		"params": params, "web": OS.has_feature("web"), "secs": t,
		"wall_s": (Time.get_ticks_usec() - start_usec) / 1e6,
		"physics_frames": Engine.get_physics_frames(),
		"frame_us": _stats(frame_us), "physics_ms": _stats(phys_ms), "process_ms": _stats(proc_ms),
		"steps_per_frame": _stats(steps_per_frame), "solve_us": _stats(solve_us), "tick_us": _stats(tick_us), "render_us": _stats(render_us),
		"max_force": max_force, "phases": phases,
		"rov_pos": [rov.global_position.x, rov.global_position.y, rov.global_position.z],
		"nan": not rov.global_position.is_finite(),
	}
	if impl and impl.get("x") != null and impl.x.size() > 0:
		var stretch = 0.0
		for i in range(impl.x.size() - 1):
			stretch += impl.x[i].distance_to(impl.x[i + 1])
		result["drawn_length"] = stretch
	if impl and impl.get("pinned") != null:
		result["softbody_points"] = impl.points
		result["softbody_pinned"] = impl.pinned
	var js = JSON.stringify(result)
	print("TETHERBENCH " + js)
	if OS.has_feature("web"):
		JavaScriptBridge.eval("window.tetherBench = " + js + ";")
	elif _p("quit", 1) == 1:
		get_tree().quit()
