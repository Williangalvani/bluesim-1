extends Node
# Benchmark for rope/spool_tether.gd, run in the web build with ?spoolbench&KEY=VALUE... or natively with
#   godot4.7 --headless --fixed-fps 120 --path . res://levels/baselevel.tscn -- spoolbench KEY=VALUE ...
# scenario=far (default): dive to `depth` m, drive straight out at `thrust` N until the spool is empty, keep pulling for
#   `hold` s, then turn 90 degrees and drive on for `turn` s.
# scenario=wrap: a pillar (radius `pr`) `pillar_at` m ahead; loop around it, then drive away for `hold` s so the
#   tether pulls taut around it.
# scenario=blade: a 3 cm thick plate across the path (top edge 1.5 m deep); the vehicle passes over it and dives
#   behind it, so the tether is pulled down over the edge. scenario=corner: a 2 x 2 m block standing out of the water
#   beside the path; the vehicle goes past and round behind it, so the tether bends round a vertical corner.
# scenario=slalom: a row of `gates` thin plates (2 to 10 cm thick, from the seabed to above the water) across alternate
#   halves of the path, `gate_gap` m apart; the vehicle zigzags round their ends, then keeps driving on at `thrust`.
#   The tether slides along them, wraps and catches, and friction and drag pile up until the vehicle stalls (no
#   `stall_dist` m of progress in `stall_secs` s), or `hold` s pass. Shots: before, sliding, wrapped, stalled.
# scenario=rovloop: out 14 m, U-turn and back zigzagging across its own slack cable (neutrally buoyant, buoy=1.0 by
#   default, so it stays at the vehicle's depth; the return legs aim at the cable's depth): the hull pushes the cable
#   aside. scenario=rovpress: wrap two posts (pr=0.4, at pillar_at=6 and +span=8 m) with a short spool (max=24.5, all out on arrival), then
#   drive sideways into the level span between them for `hold` s: the taut cable stops and deflects the vehicle. Both log the cable's push on the hull (rov_force), hull contacts and how
#   deep the cable's centreline gets inside the hull (rov_pen; rovcol=0 turns the hull contact off to compare).
#   scenario=clawpinch: the gripper's jaws are horizontal plates that close sideways, so they can only take a cable that
#   crosses their plane: the vehicle wraps the cable round a post (pr=0.3) at wrap_depth=4 m near the spool (boat=0,
#   the placeholder) and pulls it taut, which leaves a steep span from the surface down to the post. Then, jaws open,
#   it drives nose first at the span with its jaw centre steered at the span's point at grab_depth=2 m (a PID force,
#   kinematic-ish: assist_k/assist_i/assist_c, assist_max N), closes the jaws for 2 s, holds 5 s, backs 2 m with them
#   closed, opens them and backs another 1.5 m. taut=1 (default) has the spool wind in at 2 N meanwhile (reel), taut=0
#   leaves the span as laid. Logs (claw_series, per tick) and sums up the cable's centreline inside the claws, the
#   claws' reactions and spin, the vehicle's jitter, and how far the cable is from the jaw centre (dragged or slipped).
#   claw_collide=0 compares the old one-way claw contact.
# scenario=clawgrab: the same, with the jaws holding the cable (the tether's grip; grip=0 to compare): it takes the
#   span, holds, drags it 2 m astern, holds still for `still` s, lets go and backs off. Logs and sums up the pull on
#   the jaws, how far the held particle is from the jaw point, slips and releases. slip=1 pulls hard while dragging (the
#   spool brakes at slip_brake N and the vehicle backs drag=4 m with up to slip_max N), so the cable should slip out.
#   noshot=1 takes no screenshots (for a video recording at a fixed camera).
#   pen_every=N (frames) measures how much of the tether is inside solids: a ray each way along every segment.
#   With ret=S it then drives back towards the spool at half thrust for S s (try with reel=1).
# topolog=1 logs every split/merge/payout and per-tick motion (spool_tether.gd topology_log) into
#   topolog_out.events.csv and topolog_out.ticks.csv (topolog_out: a path prefix, default user://topolog).
# boat=1 (default) puts the support boat (vehicles/boat) boat_at m astern; boat=0 uses the tether's placeholder spool.
# tether=pbd swaps in the old fixed-length rope for comparison; tether=none flies with no tether.
# Tether exports are set from the keys in KEYS. secs caps the run. cam=chase|wide|spool|pillar picks the camera, and
# at milestones it asks whatever drives the page for a screenshot (window.benchShot, answered with window.benchShotAck).
# Publishes window.tetherBench / prints a SPOOLBENCH line with per-tick costs, a time series and the loads.

const KEYS = {"max": "max_length", "len0": "initial_length", "sp": "spacing", "fine": "fine_segments",
	"np": "max_particles", "fixed": "fixed_count", "sub": "substeps", "iter": "iterations", "brake": "brake_tension",
	"damp": "payout_damping", "vmax": "max_payout_speed", "reel": "reel_in", "collide": "collide",
	"q": "queries_per_tick", "bend": "max_bend_deg", "fric": "friction", "rsub": "render_subdivisions",
	"sides": "radial_sides", "lod": "lod_distance", "n": "segments", "len": "length", "anchor": "anchor_distance",
	"seg": "segment_collide", "sq": "segment_queries_per_tick", "adapt": "adaptive", "cull": "cull", "nsp": "near_spacing",
	"maxseg": "max_segment", "mtot": "max_total_particles", "rbend": "refine_bend_deg", "cstraight": "cull_straight_deg",
	"segend": "seg_end_t", "refresh": "contact_refresh_per_tick", "stop": "stop_length",
	"rovcol": "rov_collide", "rovskip": "rov_skip_length", "rovfric": "rov_friction", "rovmax": "rov_max_force",
	"buoy": "buoyancy_ratio", "bendr": "min_bend_radius", "cf": "axial_cf", "cd": "normal_cd",
	"bendsp": "bend_spacing", "claw_collide": "claw_collide", "clawr": "claw_contact_radius", "clawfric": "claw_friction",
	"clawmax": "claw_max_force", "topolog": "topology_log",
	"grip": "grip", "grip_r": "grip_radius", "grip_slip": "grip_slip_force",
	"grip_ema": "grip_smoothing", "grip_piece": "grip_min_piece"}
const JAW = Vector3(-0.02, -0.25, 0.53)  # the gripper's jaw centre, in the vehicle's frame

var params := {}
var rov: RigidBody3D
var tether: Node3D
var holder: Node
var cam: Camera3D
var cam_mode := "chase"
var scenario := "far"
var state := "settle"
var state_t := 0.0
var t := 0.0
var secs := 400.0
var done := false
var start := Vector3.ZERO
var fwd0 := Vector3.FORWARD
var side0 := Vector3.RIGHT
var depth := 3.0
var thrust := 40.0
var waypoints: Array[Vector3] = []
var wp := 0
var pillar := Vector3.INF
var pillar_r := 1.0
var focus := Vector3.INF  # what the close-up camera looks at
var wp_depth: Array[float] = []
var pull_to := Vector3.INF
var pull_depth := -1.0
var pen := {"samples": 0, "with": 0, "segs_sum": 0, "segs_max": 0, "chord_max": 0.0, "chord_sum": 0.0}
var pen_series := []
var _pen_frame := 0
var target := Vector3.ZERO
var series := []
var solve_us := []
var query_us := []
var render_us := []
var rings := []
var frame_us := []
var tick_us := []
var steps := []
var by_state := {}
var peak_tension := 0.0
var peak_spool := 0.0
var peak_at := 0.0
var nan_at := -1.0
var milestones := [10.0, 50.0, 100.0, 200.0]
var shots := []  # [name, cam]
var shot_wait := -1
var shot_name := ""
var shots_taken := []
var _last_tick_usec := 0
var _last_tick_frame := -1
var _last_frame_usec := 0
var _last_phys := 0
var _next_sample := 0.0
var _full_at := -1.0
var _result_js := ""
var _best_progress := -1e9
var _best_at := 0.0
var stalled_at := -1.0
var _stall_t := -1.0
var gates := []
var rov_pen_now := 0.0
var bend_r_now := -1.0
var _ta0 := Vector3.INF
var ta_drift := 0.0  # how far the tether's topside end (the boat's TetherAnchor) moved horizontally
var bend_at := Vector3.INF  # where the tightest bend is (for the "bend" close-up)
var bend_rs := []
var rov_pen := {"samples": 0, "with": 0, "max": 0.0}
var rov_force_peak := 0.0
var rov_us := []
var rov_touch_at := -1.0
var _last_v := Vector3.ZERO
var rov_contact_hold := 0
var rov_acc_peak := 0.0
var rov_acc_free := 0.0
var rov_spin_peak := 0.0
var rov_speed_peak := 0.0
var rov_contact_ticks := 0  # slalom: [fwd, side_from, side_to, thickness]
var claw_run := false  # clawpinch or clawgrab
var joints := []  # clawpinch: the claws' hinge joints (ljoint.gd, rjoint.gd)
var grab_c := Vector3.INF  # the span point the jaw centre goes for
var grab_h := Vector3.ZERO  # the heading it is taken with (across the span)
var _ierr := Vector3.ZERO
var _acc_now := 0.0
var claw_log := []  # per tick from the stand-off: see _claw_tick
var _drag0 := Vector3.INF
var _jaw0 := Vector3.INF
var drag := {"slip_at": -1.0}
var grip_ev := {"engaged_at": -1.0, "engaged_in": "", "snap": -1.0, "released_at": -1.0, "slipped_at": -1.0}
var _was_gripping := false
var max_stretch := 0.0  # m, drawn minus paid-out length, worst 0.5 s sample


func _p(key, default):
	return type_convert(params[key], typeof(default)) if params.has(key) else default


func _ready():
	params = load("res://rope/tether_bench.gd").parse_params()
	process_physics_priority = -1000
	secs = _p("secs", 400.0)
	scenario = _p("scenario", "far")
	claw_run = scenario in ["clawpinch", "clawgrab"]
	depth = _p("depth", 3.0)
	thrust = _p("thrust", 40.0)
	rov = get_tree().get_first_node_in_group("vehicles")
	if typeof(Globals.active_vehicle) == TYPE_STRING:
		Globals.active_vehicle = rov
	start = rov.global_position
	fwd0 = rov.global_basis.z
	fwd0.y = 0
	fwd0 = fwd0.normalized()
	side0 = fwd0.cross(Vector3.UP).normalized()
	var old = get_tree().root.find_child("tether", true, false)
	holder = old.get_parent()
	var xform = old.transform
	holder.remove_child(old)
	old.free()
	if scenario == "wrap":
		_add_pillar()
	elif scenario in ["blade", "corner"]:
		_add_obstacle()
	elif scenario == "slalom":
		_add_slalom()
	elif scenario == "rovloop":
		if not params.has("buoy"):
			params["buoy"] = "1.0"
		depth = _p("depth", 2.0)
		for w in [[14.0, 0.0], [15.0, 3.0], [9.0, 2.5], [7.0, -2.5], [4.0, -2.5], [3.0, 2.5], [0.0, 2.5]]:
			waypoints.append(start + fwd0 * w[0] + side0 * w[1])
			wp_depth.append(depth)
		focus = start + fwd0 * 7.0
		focus.y = Globals.surface_altitude - depth
		pull_to = start + fwd0 * 7.0 - side0 * 30.0
	elif scenario == "rovpress":
		# two posts the cable wraps round, so the span between them is taut and level at the vehicle's depth
		if not params.has("max"):
			params["max"] = "24.5"
		if not params.has("pr"):
			params["pr"] = "0.4"
		if not params.has("pillar_at"):
			params["pillar_at"] = "6"
		if not params.has("buoy"):
			params["buoy"] = "1.0"
		_add_pillar()
		var pa = _p("pillar_at", 6.0)
		var pb = pa + _p("span", 8.0)
		var post2 = get_parent().get_node("TestPillar").duplicate()
		post2.name = "TestPillar2"
		get_parent().add_child(post2)
		post2.global_position = Vector3(0, pillar.y, 0) + start + fwd0 * pb
		post2.global_position.y = pillar.y
		waypoints.clear()
		depth = _p("depth", 2.0)
		for w in [[pa - 1.0, -2.0], [pa + 2.0, -1.5], [pa + 2.0, 1.5], [pb + 2.0, 1.5], [pb + 2.0, -2.0], [(pa + pb) / 2.0, -3.0]]:
			waypoints.append(start + fwd0 * w[0] + side0 * w[1])
			wp_depth.append(depth)
		focus = start + fwd0 * ((pa + pb) / 2.0)
		focus.y = Globals.surface_altitude - depth
		pillar = focus  # the top view centres on the span
		pull_to = start + fwd0 * ((pa + pb) / 2.0) + side0 * 30.0
	elif claw_run:
		for kv in [["boat", "0"], ["buoy", "1.0"], ["pr", "0.15"]]:
			if not params.has(kv[0]):
				params[kv[0]] = kv[1]
		# a bar across the path (the pillar laid along side0) between the spool and the vehicle's start; the vehicle goes
		# over it, dives, and passes under it, so the cable runs from the spool steeply down to the bar's underside
		_add_pillar()
		var bar_at = _p("bar_at", -2.5)
		var bar_depth = _p("bar_depth", 4.0)
		pillar = start + fwd0 * bar_at
		pillar.y = Globals.surface_altitude - bar_depth
		get_parent().get_node("TestPillar").global_transform = Transform3D(Basis(fwd0, side0, fwd0.cross(side0)), pillar)
		waypoints.clear()
		for w in [[bar_at - 0.8, 1.0], [bar_at - 0.8, bar_depth + 2.0], [bar_at + 3.5, bar_depth + 2.0]]:
			waypoints.append(start + fwd0 * w[0])
			wp_depth.append(w[1])
		depth = 1.0
		pull_to = start + fwd0 * (bar_at + 20.0)
		pull_depth = bar_depth + 1.0
		for jn in ["ljoint", "rjoint"]:
			var node = get_tree().root.find_child(jn, true, false)
			if node and node.has_method("is_closed"):
				joints.append(node)
	if _p("boat", 1) == 1 and ResourceLoader.exists("res://vehicles/boat/boat.tscn"):
		_add_boat()
	var mode = _p("tether", "spool")
	if mode != "none":
		tether = Node3D.new()
		tether.set_script(load("res://rope/spool_tether.gd" if mode == "spool" else "res://rope/pbd_tether.gd"))
		for k in KEYS:
			if params.has(k) and KEYS[k] in tether:
				var ty = typeof(tether.get(KEYS[k]))
				tether.set(KEYS[k], int(params[k]) != 0 if ty == TYPE_BOOL else type_convert(params[k], ty))
		tether.name = "tether"
		tether.transform = xform
		holder.add_child(tether)
	cam_mode = _p("cam", "")
	if cam_mode != "":
		cam = Camera3D.new()
		cam.far = 600
		holder.add_child(cam)
		cam.top_level = true
		cam.make_current()
	_shot("start_spool", "spool")
	_shot("start_spool_under", "spool_under")
	print("SPOOLBENCH start params=%s start=%s fwd=%s physics_hz=%d spool=%s at %s" % [params, start, fwd0, Engine.physics_ticks_per_second,
		tether.anchor_node.get_path() if tether and tether.get("anchor_node") else "-", tether.anchor if tether and tether.get("anchor") != null else "-"])


func _add_boat():
	# the support boat (vehicles/boat) astern of the vehicle, its stern towards it, so the tether leaves its
	# TetherAnchor about `boat_at` m behind where the vehicle starts
	var boat = load("res://vehicles/boat/boat.tscn").instantiate()
	boat.name = "Boat"
	var back = _p("boat_at", 5.0)
	var anchor_local = Vector3(0.6, 0.12, 3.06)
	var basis = Basis.looking_at(fwd0, Vector3.UP)  # -Z (the bow) along fwd0... then turn it round: stern to the vehicle
	basis = basis.rotated(Vector3.UP, PI)
	var at = start - fwd0 * back - basis * anchor_local
	at.y = Globals.surface_altitude
	boat.transform = Transform3D(basis, at)  # before it enters the tree: its float script keeps the heading it starts with
	var gt = boat.get_node_or_null("GroundTackle")
	if gt:
		gt.swing_degrees = _p("boat_swing", 0.0)  # the anchored boat's yaw swing round its anchor
		if params.has("anchor") and int(params["anchor"]) == 0:
			gt.queue_free()
	holder.get_parent().add_child(boat)


func _add_slalom():
	# gates across alternate halves of the path; each plate's free end is where the vehicle goes round
	var surface = Globals.surface_altitude
	var n = int(_p("gates", 5))
	var gap = _p("gate_gap", 4.0)
	var first = _p("gate_at", 7.0)
	var width = _p("gate_w", 5.0)
	var b0 = Basis(side0, Vector3.UP, -fwd0)
	var d = _p("depth", 2.0)
	depth = d
	for k in range(n):
		var thick = lerp(0.02, 0.10, float(k) / max(1, n - 1))
		var sgn = 1.0 if k % 2 == 0 else -1.0
		var f = first + gap * k
		# plate from gate_overlap m past the middle out to gate_w on its side; the vehicle goes round its free end
		var s_from = -_p("gate_overlap", 2.0) * sgn
		var s_to = s_from + sgn * width
		gates.append([f, s_from, s_to, thick])
		var body = StaticBody3D.new()
		var shape = CollisionShape3D.new()
		var box = BoxShape3D.new()
		box.size = Vector3(width, surface + 1.0 + 4.6, thick)
		shape.shape = box
		body.add_child(shape)
		var mesh = MeshInstance3D.new()
		var bm = BoxMesh.new()
		bm.size = box.size
		mesh.mesh = bm
		var mat = StandardMaterial3D.new()
		mat.albedo_color = Color(0.85, 0.45 + 0.1 * (k % 2), 0.2, 0.85)
		mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
		mesh.material_override = mat
		body.add_child(mesh)
		body.name = "SlalomGate%d" % k
		var centre = start + fwd0 * f + side0 * (s_from + s_to) * 0.5
		centre.y = (surface + 1.0 - 4.6) / 2
		body.transform = Transform3D(b0, centre)
		get_parent().add_child(body)
		# round the free end: alongside it, then past it
		var round_side = s_from - sgn * 1.8
		waypoints.append(start + fwd0 * (f - 1.0) + side0 * round_side)
		waypoints.append(start + fwd0 * (f + 1.0) + side0 * round_side)
		wp_depth.append(d)
		wp_depth.append(d)
	focus = start + fwd0 * (first + gap * (n - 1) * 0.5)
	focus.y = surface - d
	pull_to = start + fwd0 * (first + gap * n + 200.0)
	pull_depth = d


func _progress() -> float:
	return (rov.global_position - start).dot(fwd0)


func _slalom_watch():
	# screenshots at the moments that matter, and the stall: no stall_dist m forward in stall_secs s while pulling
	if not shots_taken.has("before") and t > 2.5:
		shots_taken.append("before")
		_shot("before_chase", "chase")
		_shot("before_top", "course_top")
	if not shots_taken.has("sliding") and wp >= 3 and tether and int(tether.seg_contacts) + int(tether.contacts) > 0:
		shots_taken.append("sliding")
		_shot("sliding_chase", "chase")
		_shot("sliding_top", "course_top")
	if not shots_taken.has("wrapped") and wp >= 6:
		shots_taken.append("wrapped")
		_shot("wrapped_chase", "chase")
		_shot("wrapped_top", "course_top")
	var pr = _progress()
	if pr > _best_progress + _p("stall_dist", 0.3):
		_best_progress = pr
		_best_at = t
	if stalled_at < 0 and t - _best_at > _p("stall_secs", 10.0) and t > 10.0:
		stalled_at = _best_at
		_stall_t = t
		print("SPOOLBENCH stalled at t=%.1f progress=%.2f m (gates %d, wp %d) tension %.1f N, paid %.1f m" % [stalled_at, _best_progress, gates.size(), wp, tether.tension, tether.total])
		shots_taken.append("stalled")
		_shot("stalled_chase", "chase")
		_shot("stalled_top", "course_top")


func _add_obstacle():
	var body = StaticBody3D.new()
	var shape = CollisionShape3D.new()
	var box = BoxShape3D.new()
	var surface = Globals.surface_altitude
	var centre: Vector3
	var b0 = Basis(side0, Vector3.UP, -fwd0)  # x across the path, z back along it
	if scenario == "blade":
		var top = surface - _p("edge_depth", 1.5)
		box.size = Vector3(8.0, top + 4.6, _p("thick", 0.03))
		centre = start + fwd0 * _p("blade_at", 8.0)
		centre.y = (top - 4.6) / 2
		focus = Vector3(centre.x, top, centre.z)
		for w in [[4.0, 0.6], [_p("blade_at", 8.0) + 3.0, 0.6], [_p("blade_at", 8.0) + 10.0, 5.0], [_p("blade_at", 8.0) + 18.0, 5.5]]:
			waypoints.append(start + fwd0 * w[0])
			wp_depth.append(w[1])
		pull_to = start + fwd0 * 80.0
		pull_depth = 5.5
	else:
		box.size = Vector3(2.0, surface + 1.0 + 4.6, 2.0)
		centre = start + fwd0 * 9.0 + side0 * 1.5
		centre.y = (surface + 1.0 - 4.6) / 2
		focus = start + fwd0 * 10.0 + side0 * 0.5
		focus.y = surface - 0.3
		for w in [[5.0, -1.2], [12.0, -1.2], [12.0, 4.0], [8.0, 5.0]]:
			waypoints.append(start + fwd0 * w[0] + side0 * w[1])
			wp_depth.append(2.0)
		pull_to = start + fwd0 * 6.0 + side0 * 40.0
		pull_depth = 2.0
	shape.shape = box
	body.add_child(shape)
	var mesh = MeshInstance3D.new()
	var bm = BoxMesh.new()
	bm.size = box.size
	mesh.mesh = bm
	var mat = StandardMaterial3D.new()
	mat.albedo_color = Color(0.55, 0.55, 0.6, 0.6)
	mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	mesh.material_override = mat
	body.add_child(mesh)
	body.name = "TestObstacle"
	body.transform = Transform3D(b0, centre)
	get_parent().add_child(body)


func _measure_penetration():
	# ground truth, outside the tether's own (time-sliced) queries: rays both ways along every segment; a segment in
	# a solid hits its entry going forward and its exit going back (or ends inside it)
	var space = rov.get_world_3d().direct_space_state
	var xs: PackedVector3Array = tether.x
	var excl = [rov.get_rid()]
	var boat = get_tree().get_first_node_in_group("boats")
	if boat:
		excl.append(boat.get_rid())
	var count = 0
	var chord_max = 0.0
	var chord_sum = 0.0
	for j in range(xs.size() - 1):
		var a = xs[j]
		var b = xs[j + 1]
		var q = PhysicsRayQueryParameters3D.create(a, b)
		q.exclude = excl
		var hf = space.intersect_ray(q)
		var q2 = PhysicsRayQueryParameters3D.create(b, a)
		q2.exclude = excl
		var hr = space.intersect_ray(q2)
		if hf.is_empty() and hr.is_empty():
			continue
		var chord = 0.0
		if not hf.is_empty() and not hr.is_empty():
			chord = hf.position.distance_to(hr.position)
		elif not hf.is_empty():
			chord = hf.position.distance_to(b)
		else:
			chord = hr.position.distance_to(a)
		if chord < 0.005:
			continue
		count += 1
		chord_max = max(chord_max, chord)
		chord_sum += chord
	pen.samples += 1
	if count:
		pen.with += 1
		pen.segs_sum += count
		pen.segs_max = max(pen.segs_max, count)
		pen.chord_max = max(pen.chord_max, chord_max)
		pen.chord_sum += chord_sum
	pen_series.append([snapped(t, 0.01), count, snapped(chord_max, 0.001)])


func _add_pillar():
	pillar_r = _p("pr", 1.0)
	pillar = start + fwd0 * _p("pillar_at", 10.0)
	pillar.y = -1.0
	var body = StaticBody3D.new()
	var shape = CollisionShape3D.new()
	var cyl = CylinderShape3D.new()
	cyl.radius = pillar_r
	cyl.height = 14.0
	shape.shape = cyl
	body.add_child(shape)
	var mesh = MeshInstance3D.new()
	var cm = CylinderMesh.new()
	cm.top_radius = pillar_r
	cm.bottom_radius = pillar_r
	cm.height = 14.0
	mesh.mesh = cm
	var mat = StandardMaterial3D.new()
	mat.albedo_color = Color(0.55, 0.55, 0.6, 0.55)
	mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA  # so the top view shows the tether wrapped round it
	mesh.material_override = mat
	body.add_child(mesh)
	body.name = "TestPillar"
	get_parent().add_child(body)
	body.global_position = pillar
	var r = _p("loop_r", 3.0)
	for a in [150.0, 90.0, 0.0, -90.0, -180.0, 90.0, 0.0]:
		var ar = deg_to_rad(a)
		waypoints.append(pillar + (fwd0 * cos(ar) + side0 * sin(ar)) * r)


func _shot(name: String, mode: String):
	if OS.has_feature("web") and cam_mode != "" and _p("noshot", 0) == 0:
		shots.append([name, mode])


func _steer(to_point: Vector3, force: float):
	target = to_point
	var to = to_point - rov.global_position
	to.y = 0
	var fwd = rov.global_basis.z
	fwd.y = 0
	fwd = fwd.normalized()
	var e = fwd.signed_angle_to(to.normalized(), Vector3.UP) if to.length() > 0.01 else 0.0
	rov.apply_torque(Vector3.UP * clamp(25.0 * e - 20.0 * rov.angular_velocity.y, -15.0, 15.0))
	var c = max(0.0, cos(e))
	rov.apply_central_force(rov.global_basis.z * force * c * c)
	_hold_depth()


func _bend_stats() -> Dictionary:
	if bend_rs.is_empty():
		return {}
	var srt := bend_rs.duplicate()
	srt.sort()
	var tail: Array = bend_rs.slice(int(bend_rs.size() * 0.75))
	tail.sort()
	return {"min": srt[0], "median": srt[srt.size() / 2], "end_median": tail[tail.size() / 2], "samples": srt.size()}


func _bend_radius(turn: float) -> float:
	# the cable's tightest bend: over every run of joints that turns by `turn` in all, the shortest run length (half
	# the two outer segments plus the inner ones) over its total turn; -1 if it never turns that much
	var xs: PackedVector3Array = tether.x
	var m := xs.size()
	if m < 3:
		return -1.0
	var ang := PackedFloat32Array()
	var seg := PackedFloat32Array()
	ang.resize(m)
	seg.resize(m - 1)
	for i in range(m - 1):
		seg[i] = xs[i].distance_to(xs[i + 1])
	for i in range(1, m - 1):
		var a: Vector3 = xs[i] - xs[i - 1]
		var b: Vector3 = xs[i + 1] - xs[i]
		ang[i] = a.angle_to(b) if a.length_squared() > 1e-12 and b.length_squared() > 1e-12 else 0.0
	var best := INF
	for i in range(1, m - 1):
		var acc := 0.0
		var inner := 0.0
		for k in range(i, m - 1):
			acc += ang[k]
			if k > i:
				inner += seg[k - 1]
			if acc >= turn:
				var rr: float = (0.5 * seg[i - 1] + inner + 0.5 * seg[k]) / acc
				if rr < best:
					best = rr
					bend_at = xs[(i + k) / 2] if (k - i) % 2 == 0 else (xs[(i + k) / 2] + xs[(i + k) / 2 + 1]) * 0.5
				break
			if inner > best * turn:
				break
	return best if best < INF else -1.0


func _cable_depth_near(q: Vector3) -> float:
	# depth of the cable (past the first 2 m) where it passes closest to q, seen from above
	var best := INF
	var y: float = Globals.surface_altitude - depth
	var al := 0.0
	for i in range(1, tether.x.size() - 1):
		al += tether.rest[i - 1]
		if al < 2.0:
			continue
		var a: Vector3 = tether.x[i]
		var b: Vector3 = tether.x[i + 1]
		var ab := Vector2(b.x - a.x, b.z - a.z)
		var u: float = clamp(Vector2(q.x - a.x, q.z - a.z).dot(ab) / max(ab.length_squared(), 1e-9), 0.0, 1.0)
		var c := a.lerp(b, u)
		var dd := Vector2(c.x - q.x, c.z - q.z).length()
		if dd < best:
			best = dd
			y = c.y
	return clamp(Globals.surface_altitude - y, 0.6, 20.0)


func _hold_depth():
	var want = Globals.surface_altitude - depth
	rov.apply_central_force(Vector3.UP * clamp(30.0 * (want - rov.global_position.y) - 30.0 * rov.linear_velocity.y, -30.0, 30.0))


func _aim():
	# clawpinch: where the span from the spool comes down through grab_depth (looked for from the spool end), and a
	# heading across it, from the side the vehicle is on
	var xs: PackedVector3Array = tether.x
	var y: float = Globals.surface_altitude - _p("grab_depth", 2.0)
	for j in range(xs.size() - 2, 0, -1):
		if (xs[j].y - y) * (xs[j + 1].y - y) <= 0.0 and xs[j].y != xs[j + 1].y:
			grab_c = xs[j].lerp(xs[j + 1], (y - xs[j].y) / (xs[j + 1].y - xs[j].y))
			var along = xs[j + 1] - xs[j]
			along.y = 0
			grab_h = along.normalized().cross(Vector3.UP) if along.length() > 1e-3 else side0
			if (grab_c - rov.global_position).dot(grab_h) < 0.0:
				grab_h = -grab_h
			focus = grab_c
			return


func _assist(jset: Vector3):
	# the jaw centre pulled to jset (a PID force on the vehicle, at most assist_max N) and the nose turned to grab_h
	target = jset
	var jw = rov.global_transform * JAW
	var e = jset - jw
	var vj = rov.linear_velocity + rov.angular_velocity.cross(jw - rov.global_position)
	_ierr = (_ierr + e * get_physics_process_delta_time()).limit_length(0.5)
	var f = e * _p("assist_k", 300.0) + _ierr * _p("assist_i", 150.0) - vj * _p("assist_c", 80.0)
	rov.apply_central_force(f.limit_length(_p("assist_max", 40.0)))
	var fwd = rov.global_basis.z
	fwd.y = 0
	var a = fwd.normalized().signed_angle_to(grab_h, Vector3.UP)
	rov.apply_torque(Vector3.UP * clamp(25.0 * a - 20.0 * rov.angular_velocity.y, -15.0, 15.0))


func _drag_dist() -> float:
	# m the jaws back off with the cable (clawgrab: drag=M; further when pulling hard)
	return _p("drag", 4.0 if _p("slip", 0) == 1 else 2.0) if scenario == "clawgrab" else 2.0


func _cable_near(q: Vector3) -> Array:
	# [distance, point] of the cable's centreline nearest q (not the attachment's segment)
	var xs: PackedVector3Array = tether.x
	var best = [INF, q]
	for j in range(1, xs.size() - 1):
		var d: Vector3 = xs[j + 1] - xs[j]
		var c: Vector3 = xs[j] + d * clamp((q - xs[j]).dot(d) / max(d.length_squared(), 1e-12), 0.0, 1.0)
		if c.distance_to(q) < best[0]:
			best = [c.distance_to(q), c]
	return best


func _claw_tick():
	# per tick: [t, state, centreline depth in a claw, its clearance (< 0 inside), claw contacts, then per claw its
	# force (N) and spin relative to the vehicle (rad/s), per joint its angle, the cable's distance from the jaw centre,
	# the vehicle's acceleration, solve us, claw us; then the grip: held (0/1), its pull (N), the held particle's
	# distance from the jaw point (m), grip us, and the vehicle's spin (rad/s)]
	var pen_c: float = tether.claw_penetration()
	var clr: float = tether.claw_clearance
	var row = [snapped(t, 0.001), state, snapped(pen_c, 1e-4), snapped(clr, 1e-4) if is_finite(clr) else 9.0,
		tether.claw_contacts]
	for c in range(tether.claws.size()):
		row.append(snapped(tether.claw_force[c].length(), 0.01))
		row.append(snapped((tether.claws[c].angular_velocity - rov.angular_velocity).length(), 0.001))
		if nan_at < 0 and not tether.claws[c].global_position.is_finite():
			nan_at = t
	for jn in joints:
		row.append(snapped(jn.angle, 0.001))
	row.append_array([snapped(_cable_near(rov.global_transform * JAW)[0], 1e-4), snapped(_acc_now, 0.01),
		tether.solve_usec, tether.claw_usec])
	var held: bool = tether.gripping
	row.append_array([int(held), snapped(tether.grip_force.length(), 0.01) if held else 0.0,
		snapped(tether.grip_err, 1e-6) if held and _was_gripping else -1.0, tether.grip_usec,
		snapped(rov.angular_velocity.length(), 1e-4)])
	if nan_at < 0 and held and not (tether.grip_force.is_finite() and tether.x[tether.grip_i].is_finite()):
		nan_at = t
	if held and not _was_gripping:
		if grip_ev.engaged_at < 0:
			grip_ev.merge({"engaged_at": t, "engaged_in": state, "snap": tether.grip_snap}, true)
	elif _was_gripping and not held:
		grip_ev["slipped_at" if tether.slips > 0 and grip_ev.slipped_at < 0 else "released_at"] = t
	_was_gripping = held
	claw_log.append(row)


func _claw_summary() -> Dictionary:
	# from claw_log, over the grab (approach on) and per state
	var nc: int = tether.claws.size()
	var out = {"claws": tether.claws.map(func(c): return str(c.name)), "drag": drag, "by_state": {},
		"grip_events": grip_ev, "grips": tether.grips, "slips": tether.slips, "releases": tether.releases,
		"nan_at": nan_at}
	var g0: int = 9 + 2 * nc + joints.size()
	var all = {}
	for row in claw_log:
		if row[1] == "standoff":
			continue
		for key in [row[1], "grab"]:
			var s = all.get(key, {"pen": [], "clr": [], "cont": [], "gap": [], "acc": [], "solve": [], "claw_us": [],
				"gf": [], "df": [], "err": [], "gus": [], "spin": [], "last_f": -1.0})
			s.pen.append(row[2])
			s.clr.append(row[3])
			s.cont.append(row[4])
			s.gap.append(row[5 + 2 * nc + joints.size()])
			s.acc.append(row[6 + 2 * nc + joints.size()])
			s.solve.append(row[7 + 2 * nc + joints.size()])
			s.claw_us.append(row[8 + 2 * nc + joints.size()])
			if row[g0] == 1:
				s.gf.append(row[g0 + 1])
				if s.last_f >= 0.0:
					s.df.append(row[g0 + 1] - s.last_f)  # tick to tick
				if row[g0 + 2] >= 0.0:
					s.err.append(row[g0 + 2])
				s.gus.append(row[g0 + 3])
			s.last_f = row[g0 + 1] if row[g0] == 1 else -1.0
			s.spin.append(row[g0 + 4])
			for c in range(nc):
				var fk = "f%d" % c
				var wk = "w%d" % c
				if not s.has(fk):
					s[fk] = []
					s[wk] = []
				if row[5 + 2 * c] > 0.0:
					s[fk].append(row[5 + 2 * c])
				s[wk].append(row[6 + 2 * c])
			all[key] = s
	for key in all:
		var s = all[key]
		var d = {"ticks": s.pen.size(), "pen": _stats(s.pen, 1e-5), "clear_min": s.clr.min(),
			"contact_ticks": s.cont.filter(func(v): return v > 0).size(), "contacts_max": s.cont.max(),
			"gap": _stats(s.gap, 1e-5), "rov_acc": _stats(s.acc, 1e-5), "solve_us": _stats(s.solve),
			"claw_us": _stats(s.claw_us), "grip_ticks": s.gf.size(), "grip_force": _stats(s.gf, 0.01),
			"grip_df_std": snapped(_std(s.df), 0.001), "pin_err": _stats(s.err, 1e-6), "grip_us": _stats(s.gus),
			"rov_spin": _stats(s.spin, 1e-4)}
		for c in range(nc):
			d["force_%s" % out.claws[c]] = _stats(s["f%d" % c], 0.01)
			d["spin_%s" % out.claws[c]] = _stats(s["w%d" % c], 1e-4)
		if key == "grab":
			out.merge(d)
		else:
			out.by_state[key] = d
	return out


func _go(s: String):
	state = s
	state_t = 0.0
	if claw_run and tether:
		var jw = rov.global_transform * JAW
		print("SPOOLBENCH state %s t=%.2f jaw %s grab %s heading %s cable gap %.3f at %s spool %.2f N reel %s"
			% [s, t, jw, grab_c, grab_h, _cable_near(jw)[0], _cable_near(jw)[1], tether.spool_tension, tether.get("reel_in")])


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
	state_t += delta
	var total = tether.get("total") if tether and tether.get("total") != null else 0.0
	var max_len = tether.get("max_length") if tether and tether.get("max_length") != null else 0.0
	match state:
		"settle":
			_hold_depth()
			if t > 2.0 and (not claw_run or absf(Globals.surface_altitude - depth - rov.global_position.y) < 0.3):
				_go("out" if scenario == "far" else ("route" if claw_run else "loop"))
				if wp_depth.size():
					depth = wp_depth[0]
		"out":
			_steer(start + fwd0 * 2000.0, thrust)
			if _full_at < 0 and max_len > 0 and total >= max_len - 0.01:
				_full_at = t
			if (_full_at >= 0 and t - _full_at > _p("hold", 8.0)) or (max_len == 0 and state_t > _p("out", 60.0)):
				_shot("full_top", "top")
				_go("turn")
		"turn":
			_steer(rov.global_position + side0 * 1000.0, thrust)
			if state_t > _p("turn", 20.0):
				_shot("turned_top", "top")
				_shot("turned_chase", "chase")
				if _p("ret", 0.0) > 0:
					_go("return")
				else:
					_finish()
		"return":
			# back towards the spool (with reel=1 the spool takes the slack in)
			var a = tether.anchor if tether and tether.get("anchor") != null else start
			_steer(a + (start - a) * 0.5, thrust * 0.5)
			if state_t > _p("ret", 0.0):
				_shot("returned_top", "top")
				_finish()
		"loop":
			if scenario == "slalom":
				_slalom_watch()
				if _stall_t >= 0 and t - _stall_t > 5.0 and shots.is_empty() and shot_wait < 0:
					_finish()  # stuck in the gates before getting through them
			_steer(waypoints[wp], thrust)
			var d = waypoints[wp] - rov.global_position
			d.y = 0
			if d.length() < 1.2:
				wp += 1
				if wp >= waypoints.size():
					_go("pull")
					if pull_depth >= 0:
						depth = pull_depth
					if scenario == "rovpress":
						depth = _cable_depth_near(focus)  # level with the span it is about to press on
				elif wp_depth.size() > wp:
					depth = wp_depth[wp]
					if scenario == "rovloop" and wp >= 2:
						depth = _cable_depth_near(waypoints[wp])  # aim into its own slack cable
			if state_t > 120.0:
				_go("pull")
		"pull":
			if scenario == "slalom":
				_slalom_watch()
				_steer(pull_to, thrust)
				if _stall_t >= 0 and t - _stall_t > 5.0 and shots.is_empty() and shot_wait < 0:
					_finish()
				if state_t > _p("hold", 90.0):
					_finish()
			elif claw_run:
				# pull the cable taut round the post, then go and take the span
				_steer(pull_to, thrust)
				if state_t > _p("pull", 6.0):
					_go("standoff")
			else:
				_steer(pull_to if pull_to.is_finite() else pillar + fwd0 * 60.0 + side0 * 10.0, thrust)
				if state_t > _p("hold", 25.0 if scenario in ["rovloop", "rovpress"] else 15.0) - 3.0 and not shots_taken.has("wrapped") and shots.is_empty() and shot_wait < 0:
					shots_taken.append("wrapped")
					if scenario == "wrap":
						_shot("wrapped_pillar", "pillar")
					_shot("end_close", "close")
					_shot("end_chase", "chase")
					_shot("end_top", "pillar_top")
					_shot("end_bend", "bend")
					_shot("end_bendtop", "bend_top")
				if state_t > _p("hold", 25.0 if scenario in ["rovloop", "rovpress"] else 15.0):
					_finish()
		"route":
			# clawpinch: waypoints at a depth each, vertical legs too (slowing down onto each)
			depth = wp_depth[wp]
			var d = waypoints[wp] - rov.global_position
			d.y = 0
			var dz = absf(Globals.surface_altitude - depth - rov.global_position.y)
			_steer(waypoints[wp], thrust * clamp(d.length() / 2.0, 0.0, 1.0) if dz < 1.0 else 0.0)
			if d.length() < 0.6 and dz < 0.4:
				wp += 1
				if wp >= waypoints.size():
					depth = pull_depth
					_go("pull")
			elif state_t > 120.0:
				_go("pull")
		"standoff":
			# out in front of the span, level with where the jaws will take it (up first, clear of the bar)
			if not grab_c.is_finite() or state_t < 0.5:
				_aim()
			depth = _p("grab_depth", 2.0) + JAW.y
			if grab_c.is_finite():
				var dz = absf(Globals.surface_altitude - depth - rov.global_position.y)
				_steer(grab_c - grab_h * 1.4, thrust * 0.5 if dz < 0.6 else 0.0)
				var d = grab_c - grab_h * 1.4 - rov.global_position
				d.y = 0
				if (d.length() < 0.35 or state_t > 40.0) and state_t > 1.0:
					_aim()
					_ierr = Vector3.ZERO
					if _p("taut", 1) == 1 and "reel_in" in tether:
						tether.reel_in = true  # the winch keeps the span taut (reel_tension)
					_go("approach")
			else:
				_hold_depth()
		"approach":
			# nose first into the span, jaws open, the jaw centre steered at it
			var k = clamp(state_t / _p("approach_s", 6.0), 0.0, 1.0)
			_assist(grab_c - grab_h * lerp(_p("standoff", 1.0), 0.0, k))
			if state_t > _p("approach_s", 6.0) + 1.5:
				_go("aim")
		"aim":
			# onto where the cable is now (pushed into a jaw it would be swept out as they close), level
			var nc = _cable_near(rov.global_transform * JAW)[1]
			grab_c = grab_c.lerp(Vector3(nc.x, grab_c.y, nc.z), _p("aim_gain", 0.05))
			_assist(grab_c)
			if not shots_taken.has("open"):
				shots_taken.append("open")
				_shot("open_jaw_side", "jaw_side")
				_shot("open_jaw_below", "jaw_below")
				_shot("open_chase", "chase")
			if state_t > _p("aim_s", 3.0):
				_go("close")
		"close", "hold":
			_assist(grab_c)
			for jn in joints:
				jn.close()
			if state == "close" and state_t > 2.0:
				_go("hold")
			elif state == "hold" and state_t > 3.0 and not shots_taken.has("closed"):
				shots_taken.append("closed")
				_shot("closed_jaw_side", "jaw_side")
				_shot("closed_jaw_below", "jaw_below")
				_shot("closed_close", "close")
			elif state == "hold" and state_t > 5.0:
				_go("back")
		"back", "still":
			# 2 m astern in 4 s with the jaws shut: is the cable dragged along, or does it slip out?
			if scenario == "clawgrab" and state == "back" and state_t > 4.0:
				state = "still"  # (state_t runs on)
			if scenario == "clawgrab" and _p("slip", 0) == 1 and not drag.has("brake"):
				drag["brake"] = _p("slip_brake", 100.0)
				tether.brake_tension = drag.brake
				params["assist_max"] = str(_p("slip_max", 150.0))
			_assist(grab_c - grab_h * _drag_dist() * clamp(state_t / 4.0, 0.0, 1.0))
			for jn in joints:
				jn.close()
			var near = _cable_near(rov.global_transform * JAW)
			if not _drag0.is_finite():
				_drag0 = near[1]
				_jaw0 = rov.global_transform * JAW
			drag["gap_max"] = max(drag.get("gap_max", 0.0), near[0])
			if near[0] > 0.05 and drag.slip_at < 0:
				drag.slip_at = t
			var shot = "dragging" if scenario == "clawgrab" else "backing"
			if state_t > 3.5 and not shots_taken.has(shot):
				shots_taken.append(shot)
				_shot(shot + "_jaw_side", "jaw_side")
				_shot(shot + "_jaw_below", "jaw_below")
				_shot(shot + "_chase", "chase")
			if state_t > 4.0 + (_p("still", 3.0) if scenario == "clawgrab" else 1.0):
				drag["jaw_moved"] = _jaw0.distance_to(rov.global_transform * JAW)
				drag["cable_moved"] = _drag0.distance_to(near[1])  # the cable's point next to the jaw centre
				drag["gap_end"] = near[0]
				_go("release")
		"release", "away":
			for jn in joints:
				jn.open()
			var k = clamp(state_t / 3.0, 0.0, 1.0) if state == "away" else 0.0
			_assist(grab_c - grab_h * (_drag_dist() + 1.5 * k))
			if scenario == "clawgrab" and state == "release" and state_t > 1.2 and not shots_taken.has("released"):
				shots_taken.append("released")
				_shot("released_jaw_side", "jaw_side")
				_shot("released_jaw_below", "jaw_below")
				_shot("released_chase", "chase")
			if state == "release" and state_t > 1.5:
				_go("away")
			elif state == "away" and state_t > 3.5 and not shots_taken.has("away"):
				shots_taken.append("away")
				_shot("away_chase", "chase")
			if state == "away" and state_t > 4.0 and shots.is_empty() and shot_wait < 0:
				_finish()
	if claw_run and state in ["settle", "route", "pull", "standoff", "approach", "aim"]:
		for jn in joints:
			jn.open()  # wide open (it starts part open)
	if t >= secs and not done:
		_finish()
	if tether == null:
		return
	if tether.get("solve_usec") != null and t > 2.0:
		solve_us.append(tether.solve_usec)
		if tether.get("query_usec") != null:
			query_us.append(tether.query_usec)
		var bs = by_state.get(state, [])
		bs.append(tether.solve_usec)
		by_state[state] = bs
	if t > 2.0 and tether.get("anchor") != null:
		var ta: Vector3 = tether.anchor
		if not _ta0.is_finite():
			_ta0 = ta
		ta_drift = max(ta_drift, Vector2(ta.x - _ta0.x, ta.z - _ta0.z).length())
	if "rov_force" in tether and t > 2.0:
		rov_force_peak = max(rov_force_peak, tether.rov_force.length())
		# jitter / launch check: the vehicle's acceleration and spin, tick to tick, while the cable touches the hull
		var acc: float = (rov.linear_velocity - _last_v).length() / delta
		_last_v = rov.linear_velocity
		_acc_now = acc
		if tether.rov_contacts > 0 or rov_contact_hold > 0:
			rov_contact_hold = max(rov_contact_hold - 1, 0)
			if tether.rov_contacts > 0:
				rov_contact_hold = 30
			rov_acc_peak = max(rov_acc_peak, acc)
			rov_spin_peak = max(rov_spin_peak, rov.angular_velocity.length())
			rov_speed_peak = max(rov_speed_peak, rov.linear_velocity.length())
			rov_contact_ticks += 1
		else:
			rov_acc_free = max(rov_acc_free, acc)
		rov_us.append(tether.rov_usec)
		if rov_touch_at < 0 and tether.rov_contacts > 0:
			rov_touch_at = t
			if scenario in ["rovloop", "rovpress"]:
				_shot("touch_chase", "chase")
				_shot("touch_top", "course_top")
	if claw_run and "claw_force" in tether and not state in ["settle", "route", "pull"]:
		_claw_tick()
	var tens = tether.tension
	if tens > peak_tension:
		peak_tension = tens
		peak_at = t
	var sp = tether.get("spool_tension")
	if sp != null:
		peak_spool = max(peak_spool, sp)
	if nan_at < 0 and (not rov.global_position.is_finite() or not tether.force.is_finite()):
		nan_at = t
	if nan_at < 0 and tether.x.size() > 0 and not (tether.x[tether.x.size() / 2].is_finite() and tether.x[1].is_finite()):
		nan_at = t
	if _p("shot_at", 0.0) > 0 and t >= _p("shot_at", 0.0) and not shots_taken.has("timed"):
		# shot_at=SECONDS: the same close-up/chase/top shots at a fixed time, to compare runs (e.g. with and without
		# the adaptive tether) in the same place
		shots_taken.append("timed")
		_shot("t%d_close" % int(_p("shot_at", 0.0)), "close")
		_shot("t%d_chase" % int(_p("shot_at", 0.0)), "chase")
		_shot("t%d_top" % int(_p("shot_at", 0.0)), "pillar_top")
		_shot("t%d_bend" % int(_p("shot_at", 0.0)), "bend")
		_shot("t%d_bendtop" % int(_p("shot_at", 0.0)), "bend_top")
	if _p("anchorshots", 0) == 1 and t >= 8.0 and not shots_taken.has("anchor"):
		# the boat's ground tackle (vehicles/boat/boat_anchor.gd): from above the water and down at the anchor
		shots_taken.append("anchor")
		var gt = get_tree().get_first_node_in_group("boats").get_node_or_null("GroundTackle")
		if gt:
			print("SPOOLBENCH anchor at %s depth %.1f links %d boat %s" % [gt.anchor_point, gt._depth, gt._chain.multimesh.instance_count, gt._boat.global_position])
		_shot("anchor_above", "anchor_above")
		_shot("anchor_under", "anchor_under")
		_shot("anchor_seabed", "anchor_seabed")
	for m in milestones:
		if max_len >= m and total >= m and not shots_taken.has(m):
			shots_taken.append(m)
			_shot("len%d_chase" % m, "chase")
			_shot("len%d_top" % m, "top")
			_shot("len%d_side" % m, "side")
	if t >= _next_sample:
		_next_sample += 0.5
		var a = tether.get("anchor")
		# stretch: drawn length past the rest length paid out (the constraints' error), worst over the run
		var stretch_now := 0.0
		if tether.get("x") != null:
			for i in range(tether.x.size() - 1):
				stretch_now += tether.x[i].distance_to(tether.x[i + 1])
			stretch_now -= total
			max_stretch = max(max_stretch, stretch_now)
		series.append([snapped(t, 0.01), snapped(rov.global_position.distance_to(a), 0.01) if a != null else 0,
			snapped(total, 0.01), tether.x.size(), snapped(tens, 0.01), snapped(sp if sp != null else 0.0, 0.01),
			snapped(tether.get("payout_speed") if tether.get("payout_speed") != null else 0.0, 0.001),
			snapped(rov.linear_velocity.length(), 0.001), tether.solve_usec, tether.get("query_usec"),
			tether.get("contacts"), snapped(Globals.surface_altitude - rov.global_position.y, 0.01), state,
			tether.get("near_count"), tether.get("seg_contacts"), tether.get("splits"), tether.get("merges"),
			snapped(_progress(), 0.01),
			snapped(tether.rov_force.length(), 0.01) if "rov_force" in tether else 0.0,
			tether.get("rov_contacts"), snapped(rov_pen_now, 0.001), tether.get("rov_usec"),
			snapped((rov.global_position - start).dot(side0), 0.01), snapped(rov.global_position.y, 0.01),
			snapped(rov.linear_velocity.length(), 0.01), snapped(bend_r_now, 0.001), snapped(stretch_now, 0.001)])


func _process(_delta):
	var now = Time.get_ticks_usec()
	if _last_frame_usec and t > 2.0 and not done:
		frame_us.append(now - _last_frame_usec)
		steps.append(Engine.get_physics_frames() - _last_phys)
		if tether and tether.get("render_usec") != null:
			render_us.append(tether.render_usec)
		if tether and tether.get("rings_drawn") != null:
			rings.append(tether.rings_drawn)
	_last_phys = Engine.get_physics_frames()
	_last_frame_usec = now
	_pen_frame += 1
	if tether and tether.get("x") != null and not done and t > 2.0 and _p("pen_every", 6) > 0 and _pen_frame % _p("pen_every", 6) == 0:
		_measure_penetration()
		bend_r_now = _bend_radius(deg_to_rad(_p("bend_window_deg", 60.0)))
		if bend_r_now > 0.0:
			bend_rs.append(bend_r_now)
		if tether.has_method("rov_penetration"):
			rov_pen_now = tether.rov_penetration()
			rov_pen.samples += 1
			if rov_pen_now > 0.0:
				rov_pen.with += 1
				rov_pen.max = max(rov_pen.max, rov_pen_now)
	_update_shots()
	if cam:
		_place_camera()


func _update_shots():
	if shot_name != "":
		if str(JavaScriptBridge.eval("window.benchShotAck || ''")) == shot_name:
			shot_name = ""
		return
	if shot_wait > 0:
		shot_wait -= 1
		if shot_wait == 0:
			shot_name = shots[0][0]
			shots.pop_front()
			JavaScriptBridge.eval("window.benchShot = '%s';" % shot_name)
			shot_wait = -1
		return
	if not shots.is_empty():
		cam_mode = shots[0][1]
		shot_wait = 6
	elif _result_js != "":
		JavaScriptBridge.eval("window.tetherBench = " + _result_js + ";")
		_result_js = ""


var _clear := false
var _saved_env := {}


func _set_clear_water(on: bool):
	# overview shots: no fog, no surface or underwater effects, so the whole tether shows (not what a pilot sees)
	if on == _clear:
		return
	_clear = on
	for we in get_tree().root.find_children("*", "WorldEnvironment", true, false):
		var env = we.environment
		if env == null:
			continue
		if on:
			_saved_env[env] = [env.fog_enabled, env.volumetric_fog_enabled]
			env.fog_enabled = false
			env.volumetric_fog_enabled = false
		elif _saved_env.has(env):
			env.fog_enabled = _saved_env[env][0]
			env.volumetric_fog_enabled = _saved_env[env][1]
	for name in ["UnderwaterFX", "water"]:
		for n in get_tree().root.find_children(name, "", true, false):
			if "visible" in n:
				n.visible = not on
	cam.projection = Camera3D.PROJECTION_ORTHOGONAL if cam_mode in ["top", "pillar_top", "course_top", "bend_top"] and on else Camera3D.PROJECTION_PERSPECTIVE


func _place_camera():
	var r = rov.global_position
	var a = tether.anchor if tether and tether.get("anchor") != null else start
	var sep = Vector2(r.x - a.x, r.z - a.z).length()
	if tether and "draw_diameter" in tether and not params.has("thick"):
		# overview shots are tens of metres away: draw the tether thicker there (labelled), real size up close
		match cam_mode:
			"chase": tether.draw_diameter = 0.0
			"spool", "spool_under": tether.draw_diameter = 0.025
			"pillar", "pillar_top": tether.draw_diameter = 0.05
			"course_top": tether.draw_diameter = 0.08
			"close", "bend": tether.draw_diameter = 0.02
			"jaw_side", "jaw_below": tether.draw_diameter = 0.0
			"bend_top": tether.draw_diameter = 0.03
			_: tether.draw_diameter = max(0.06, sep * 0.003)
	_set_clear_water(cam_mode in ["top", "side", "pillar_top", "course_top", "bend_top"])
	var surface = Globals.surface_altitude
	match cam_mode:
		"top":
			var m = (r + a) / 2
			cam.size = max(sep * 1.2, 16.0)
			cam.global_position = Vector3(m.x, surface + 50.0, m.z)
			cam.look_at(Vector3(m.x, 0, m.z), fwd0)
		"course_top":
			# the whole slalom from above, with the vehicle
			var c = focus if focus.is_finite() else r
			cam.size = _p("top_size", 26.0)
			cam.global_position = Vector3(c.x, surface + 50.0, c.z)
			cam.look_at(Vector3(c.x, 0, c.z), fwd0)
		"pillar_top":
			var c = focus if focus.is_finite() else (pillar if pillar.is_finite() else r)
			cam.size = 16.0
			cam.global_position = Vector3(c.x, surface + 50.0, c.z)
			cam.look_at(Vector3(c.x, 0, c.z), fwd0)
		"side":
			var m = (r + a) / 2
			var dir = r - a
			dir.y = 0
			var side = dir.normalized().cross(Vector3.UP).normalized() if dir.length() > 0.1 else side0
			var dist = clamp(sep * 0.75, 6.0, 220.0)
			cam.global_position = Vector3(m.x, surface - depth * 0.5, m.z) + side * dist
			cam.look_at(Vector3(m.x, surface - depth * 0.5, m.z), Vector3.UP)
		"chase":
			var f = rov.global_basis.z
			cam.global_position = r - f * 2.5 + Vector3.UP * 1.6 + rov.global_basis.x * 2.0
			cam.global_position.y = min(cam.global_position.y, surface - 0.3)  # stays under water with a shallow ROV
			cam.look_at(r - f * 1.0, Vector3.UP)
		"wide":
			var m = (r + a) / 2
			var dir = r - a
			dir.y = 0
			var side = dir.normalized().cross(Vector3.UP).normalized() if sep > 0.1 else side0
			var dist = clamp(sep * 0.65, 5.0, 160.0)
			var at = m + side * dist
			at.y = min(m.y + dist * 0.2, surface - 0.4)
			cam.global_position = at
			cam.look_at(m, Vector3.UP)
		"spool":
			# from just above the water, so the spool (on the surface) and the cable going into the water both show
			var dir = r - a
			dir.y = 0
			dir = dir.normalized() if dir.length() > 0.1 else fwd0
			var side = dir.cross(Vector3.UP).normalized()
			cam.global_position = a + dir * 2.8 + side * 2.2 + Vector3.UP * 1.1
			cam.look_at(a + dir * 0.4 + Vector3.DOWN * 0.25, Vector3.UP)
		"close":
			# close-up of where the tether meets the obstacle, from the side
			var c = focus if focus.is_finite() else (Vector3(pillar.x, surface - depth, pillar.z) if pillar.is_finite() else r)
			cam.global_position = c + side0 * 2.6 - fwd0 * 1.2 + Vector3.UP * 0.5
			cam.look_at(c, Vector3.UP)
		"anchor_above", "anchor_under", "anchor_seabed":
			var boat = get_tree().get_first_node_in_group("boats")
			var gt = boat.get_node_or_null("GroundTackle") if boat else null
			if gt and gt.anchor_point.is_finite():
				var bow: Vector3 = boat.global_transform * gt.fairlead
				var ap: Vector3 = gt.anchor_point
				var lead = Vector3(ap.x - bow.x, 0, ap.z - bow.z).normalized()
				var sd = lead.cross(Vector3.UP)
				match cam_mode:
					"anchor_above":
						# off the bow, just above the water: the bow roller and the chain going in
						cam.global_position = bow + lead * 2.2 + sd * 3.2 + Vector3.UP * 0.6
						cam.global_position.y = max(cam.global_position.y, surface + 0.35)
						cam.look_at(bow + lead * 0.6 + Vector3.DOWN * 0.35, Vector3.UP)
					"anchor_under":
						# under water off the bow: the hull from below and the chain hanging away to the seabed
						cam.global_position = bow + lead * 3.0 + sd * 4.5 + Vector3.DOWN * 2.0
						cam.global_position.y = min(cam.global_position.y, surface - 1.0)
						cam.look_at(bow + lead * 3.5 + Vector3.DOWN * 2.2, Vector3.UP)
					_:
						cam.global_position = ap + sd * 2.2 - lead * 1.2 + Vector3.UP * 1.1
						cam.look_at(ap + lead * 0.3, Vector3.UP)
		"bend_top":
			# the tightest bend from straight above, 5 m across: the radius the cable takes round the obstacle
			var c = bend_at if bend_at.is_finite() else r
			cam.size = _p("bend_size", 5.0)
			cam.global_position = Vector3(c.x, surface + 50.0, c.z)
			cam.look_at(Vector3(c.x, 0, c.z), fwd0)
		"bend":
			# close-up of the cable's tightest bend, from the side and a little above
			var c = bend_at if bend_at.is_finite() else r
			cam.global_position = c + side0 * 1.7 - fwd0 * 0.9 + Vector3.UP * 1.0
			cam.look_at(c, Vector3.UP)
		"spool_under":
			# under the boat's stern, the cable going down from the spool
			var dir = r - a
			dir.y = 0
			dir = dir.normalized() if dir.length() > 0.1 else fwd0
			var side = dir.cross(Vector3.UP).normalized()
			cam.global_position = a + dir * 3.0 + side * 3.0 + Vector3.DOWN * 1.8
			cam.look_at(a + dir * 0.8 + Vector3.DOWN * 0.4, Vector3.UP)
		"jaw_side":
			# the jaws close up, along their closing axis
			var jw = rov.global_transform * JAW
			cam.global_position = jw + rov.global_basis.x * 0.3 + rov.global_basis.z * 0.15 + Vector3.UP * 0.06
			cam.look_at(jw, Vector3.UP)
		"jaw_below":
			var jw = rov.global_transform * JAW
			cam.global_position = jw + Vector3.DOWN * 0.4 + rov.global_basis.z * 0.12
			cam.look_at(jw, rov.global_basis.z)
		"pillar":
			var c = pillar if pillar.is_finite() else r
			cam.global_position = Vector3(c.x, surface - 0.8, c.z) - side0 * 5.5 - fwd0 * 2.0
			cam.look_at(Vector3(c.x, surface - depth, c.z), Vector3.UP)


static func _write_csv(path: String, cols: Array, rows: Array):
	var f := FileAccess.open(path, FileAccess.WRITE)
	if f == null:
		push_warning("spool_bench: can't write %s" % path)
		return
	f.store_line(",".join(cols))
	for row in rows:
		f.store_line(",".join(row.map(func(e): return String.num(e, 6) if typeof(e) == TYPE_FLOAT else str(e))))


static func _std(a: Array) -> float:
	if a.size() < 2:
		return 0.0
	var m := 0.0
	for e in a:
		m += e / a.size()
	var v := 0.0
	for e in a:
		v += (e - m) * (e - m) / (a.size() - 1)
	return sqrt(v)


static func _stats(a: Array, step := 0.1) -> Dictionary:
	if a.is_empty():
		return {}
	var b = a.duplicate()
	b.sort()
	var sum = 0.0
	for e in b:
		sum += e
	return {"mean": snapped(sum / b.size(), step), "p50": b[b.size() / 2], "p95": b[int(b.size() * 0.95)], "max": b[-1], "n": b.size()}


func _finish():
	if done:
		return
	done = true
	var st = {}
	for k in by_state:
		st[k] = _stats(by_state[k])
	var out_speed = []
	var out_payout = []
	var out_spool = []
	for row in series:
		if row[12] == "out" and row[2] < (tether.max_length - 0.5 if tether and tether.get("max_length") != null else 1e9) and row[0] > 12.0:
			out_speed.append(row[7])
			out_payout.append(row[6])
			out_spool.append(row[5])
	var result = {
		"params": params, "web": OS.has_feature("web"), "secs": t, "physics_frames": Engine.get_physics_frames(),
		"solve_us": _stats(solve_us), "query_us": _stats(query_us), "solve_by_state": st, "tick_us": _stats(tick_us),
		"render_us": _stats(render_us), "rings": _stats(rings), "frame_us": _stats(frame_us), "steps_per_frame": _stats(steps),
		"peak_tension": peak_tension, "peak_at": peak_at, "peak_spool_tension": peak_spool, "nan_at": nan_at, "full_at": _full_at,
		"steady_out": {"speed": _stats(out_speed), "payout": _stats(out_payout), "spool_tension": _stats(out_spool)},
		"particles": tether.x.size() if tether else 0, "total": tether.get("total") if tether else 0,
		"far_spacing": tether.get("far_spacing") if tether else 0,
		"rov_pos": [rov.global_position.x, rov.global_position.y, rov.global_position.z],
		"rov_pen": rov_pen, "rov_force_peak": rov_force_peak, "rov_us": _stats(rov_us), "rov_touch_at": rov_touch_at, "bend_r": _bend_stats(), "anchor_drift": ta_drift,
		"rov_contact": {"ticks": rov_contact_ticks, "acc_peak": rov_acc_peak, "acc_peak_free": rov_acc_free,
			"spin_peak": rov_spin_peak, "speed_peak": rov_speed_peak},
		"waypoints_done": wp, "series": series, "stalled_at": stalled_at, "best_progress": _best_progress, "gates": gates,
		"pen": pen, "pen_series": pen_series,
		"claw": _claw_summary() if claw_run and "claw_force" in tether else {},
		"claw_series": claw_log,
		"max_stretch": max_stretch, "splits": tether.get("splits") if tether else 0, "merges": tether.get("merges") if tether else 0,
	}
	if tether and tether.get("x") != null:
		var drawn = 0.0
		for i in range(tether.x.size() - 1):
			drawn += tether.x[i].distance_to(tether.x[i + 1])
		result["drawn_length"] = drawn
		if pillar.is_finite() and scenario != "rovpress":
			# how much of the tether lies against the pillar, and how far round it it goes
			var touching = 0
			var ang = 0.0
			var prev = null
			for xi in tether.x:
				var d = Vector2(xi.x - pillar.x, xi.z - pillar.z)
				if d.length() < pillar_r + 0.3:
					touching += 1
					if prev != null:
						ang += abs(prev.angle_to(d))
					prev = d
				else:
					prev = null
			result["pillar_touching"] = touching
			result["pillar_wrap_deg"] = rad_to_deg(ang)
	if claw_run:
		print("SPOOLBENCH %s %s" % [scenario, JSON.stringify(result.claw)])
	if tether and tether.get("topology_log"):
		var base: String = _p("topolog_out", "user://topolog")
		_write_csv(base + ".events.csv", tether.TOPO_EVENT_COLS, tether.topo_events)
		_write_csv(base + ".ticks.csv", tether.TOPO_TICK_COLS, tether.topo_ticks)
		print("SPOOLBENCH topolog events=%d ticks=%d to %s" % [tether.topo_events.size(), tether.topo_ticks.size(), base])
	var js = JSON.stringify(result)
	print("SPOOLBENCH " + js)
	if OS.has_feature("web"):
		_result_js = js  # published once the last screenshots are taken
	elif _p("quit", 1) == 1:
		get_tree().quit()
