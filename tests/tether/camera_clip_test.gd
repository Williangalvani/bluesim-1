extends Node
## Test harness (not part of the game): does the tether pass through the ROV's camera? The reef level as a pilot gets it
## (levels/baselevel.tscn with its boat and tether, no autopilot), the ROV's own camera current. The ROV dives, turns its
## camera to the boat (where the spool is) and, holding its attitude with a PD torque (as STABILIZE would), drives
## backwards, away from the boat. Every physics tick it measures how near the drawn cable's centreline comes to the
## camera, whether it cuts the near plane and whether it is inside the hull; at the end it prints a CAMCLIP JSON line and
## writes summary.json and series.csv into out=DIR.
##   godot4.7 --path . --rendering-method gl_compatibility --resolution 1280x720 res://tests/tether/camera_clip_test.tscn \
##     -- variant=slack out=DIR [shots=T1,T2]   (add --write-movie DIR/x.avi --fixed-fps 30 for a video)
## Args after `--`: variant=level | slack | pitch | pitch10 (PRESETS); out=DIR; shots=T,... (s: ROV camera screenshots);
## worst=0 (no screenshot of the worst moment); any preset key: depth=M, approach=S (driving at the boat first, so the
## cable is slack), pitch=DEG (nose up, the camera towards the boat on the surface); speed=M/S (0.7, backwards); back=S
## (12); tilt=DEG (camera tilt servo, 0); secs=S; t.EXPORT=VALUE sets one of the tether's exports.

const PRESETS = {
	"level": {"approach": 0.0, "pitch": 0.0, "depth": 0.8},
	"slack": {"approach": 5.0, "pitch": 0.0, "depth": 0.8},
	"pitch": {"approach": 5.0, "pitch": 20.0, "depth": 2.0},
	"pitch10": {"approach": 5.0, "pitch": 10.0, "depth": 0.8},
}

var rov: RigidBody3D
var cam: Camera3D
var tether: Node3D
var args := {}
var out := "user://camclip"
var variant := "slack"
var t := 0.0
var phase := "settle"
var phase_t := 0.0
var yaw := 0.0
var pitch := 0.0
var want_y := 0.0
var v_target := 0.0
var plan := []  # [phase, seconds]
var plan_i := 0
var series := []
var shots: Array[float] = []
var shots_done := []
var worst := {"surf": INF}
var worst_shot := INF
var worst_shot_t := -1.0
var mesh := {"frames": 0, "min_surf": INF, "clip_frames": 0, "close_frames": 0, "min_t": -1.0}
var stats := {}
var cam_local := Vector3.ZERO
var finished := false
var headless := false


func _enter_tree():
	for a in OS.get_cmdline_user_args():
		var kv = str(a).split("=", true, 1)
		args[kv[0]] = kv[1] if kv.size() > 1 else "1"
	out = _a("out", out)
	variant = _a("variant", variant)
	DirAccess.make_dir_recursive_absolute(out)
	if not Globals.active_level:
		ProjectSettings.load_resource_pack("res://levels/reef.pck")
		Globals.active_level = "res://custom_level.tscn"
		Globals.active_vehicle = "bench"  # the vehicle skips the autopilot link; it is made active below
	for s in str(_a("shots", "")).split(",", false):
		shots.append(float(s))
	headless = DisplayServer.get_name() == "headless"


func _a(key: String, default):
	if args.has(key):
		return type_convert(args[key], typeof(default))
	return PRESETS.get(variant, {}).get(key, default)


func _ready():
	process_physics_priority = -1000  # first in each tick: what it measures is what the last frame drew
	process_priority = 1000  # after the tether has built this frame's mesh
	add_child(load("res://levels/baselevel.tscn").instantiate())
	await get_tree().process_frame
	rov = get_tree().get_first_node_in_group("vehicles")
	Globals.active_vehicle = rov
	cam = rov.get_node("Camera3D")
	cam.current = true
	tether = rov.get_parent().get_node("tether")
	var overrides := {}
	for k in args:
		if k.begins_with("t.") and k.substr(2) in tether:
			var ty := typeof(tether.get(k.substr(2)))
			tether.set(k.substr(2), int(args[k]) != 0 if ty == TYPE_BOOL else type_convert(args[k], ty))
			overrides[k.substr(2)] = tether.get(k.substr(2))
	rov.camera_tilt = _a("tilt", 0.0)
	want_y = Globals.surface_altitude - _a("depth", 2.0)
	yaw = _heading(rov.global_basis.z)
	plan = [
		["settle", 3.0],
		["turn", 7.0],
		["approach", _a("approach", 5.0)],
		["stop", 1.5],
		["reverse", _a("back", 12.0)],
		["end", 1.0]
	]
	cam_local = rov.to_local(cam.global_position)
	var info := {
		"variant": variant,
		"overrides": overrides,
		"params":
		{
			"depth": _a("depth", 2.0),
			"approach": _a("approach", 5.0),
			"pitch": _a("pitch", 0.0),
			"speed": _a("speed", 0.7),
			"back": _a("back", 12.0),
			"tilt": _a("tilt", 0.0)
		},
		"renderer": RenderingServer.get_current_rendering_method(),
		"rov": str(rov.get_path()),
		"cam": str(cam.get_path()),
		"cam_current": cam.current,
		"viewport": str(cam.get_viewport().get_path()),
		"viewport_size": cam.get_viewport().get_visible_rect().size,
		"cam_near": cam.near,
		"cam_fov": cam.fov,
		"cam_local": cam_local,
		"cam_view_local": rov.global_basis.inverse() * -cam.global_basis.z,
		"attach_local": tether.attach_local,
		"rov_skip_length": tether.rov_skip_length,
		"contact_radius": tether.contact_radius,
		"anchor": tether.anchor,
		"anchor_node": str(tether.anchor_node.get_path()),
		"rov_pos": rov.global_position,
		"seabed_y": _seabed_y(),
		"shapes": _shape_info(),
		"camera_clearance": tether.get("camera_clearance")
	}
	tether._rov_closest(cam_local)
	info["cam_hull_d"] = tether._rc_d  # < 0: the camera is inside the vehicle's collision shapes
	stats["info"] = info
	print("CAMCLIP info ", JSON.stringify(info))


func _seabed_y() -> float:
	var q := PhysicsRayQueryParameters3D.create(rov.global_position, rov.global_position + Vector3.DOWN * 100.0)
	q.exclude = [rov.get_rid()]
	var hit := rov.get_world_3d().direct_space_state.intersect_ray(q)
	return hit.position.y if hit else -INF


func _shape_info() -> Array:
	var o := []
	for e in tether._rov_shapes:
		o.append({"kind": e[0], "origin": e[1].origin, "half": e[2]})
	return o


func _heading(v: Vector3) -> float:
	return atan2(v.x, v.z)


func _physics_process(delta):
	if rov == null or finished:
		return
	_measure()
	t += delta
	phase_t += delta
	while plan_i < plan.size() and phase_t >= plan[plan_i][1]:
		phase_t -= plan[plan_i][1]
		plan_i += 1
		if plan_i >= plan.size():
			_finish()
			return
		phase = plan[plan_i][0]
		print(
			(
				"CAMCLIP phase %s t=%.2f pos=%s dist_to_anchor=%.2f paid=%.2f"
				% [phase, t, rov.global_position, rov.global_position.distance_to(tether.anchor), tether.total]
			)
		)
		if phase == "turn":
			var to: Vector3 = tether.anchor - rov.global_position
			yaw = _heading(to)
		if phase in ["approach", "stop", "reverse"]:
			pitch = deg_to_rad(_a("pitch", 0.0))
	match phase:
		"settle", "turn", "end":
			v_target = 0.0
		"approach":
			v_target = 0.4
		"stop":
			v_target = 0.0
		"reverse":
			v_target = -_a("speed", 0.7)
	_drive(delta)
	if t > _a("secs", 1e9):
		_finish()


func _drive(_delta):
	# depth hold, attitude hold (yaw `yaw`, nose up `pitch`, level roll) and a speed hold along the vehicle's axis
	rov.apply_central_force(
		Vector3.UP * clamp(40.0 * (want_y - rov.global_position.y) - 40.0 * rov.linear_velocity.y, -40.0, 40.0)
	)
	var want := Basis(Vector3.UP, yaw) * Basis(Vector3.RIGHT, -pitch)
	var q := Quaternion(want * rov.global_basis.orthonormalized().inverse())
	if q.w < 0.0:
		q = -q
	var ang := 2.0 * acos(clampf(q.w, -1.0, 1.0))
	var axis := Vector3(q.x, q.y, q.z)
	axis = axis.normalized() if axis.length() > 1e-9 else Vector3.UP
	rov.apply_torque((axis * ang * 30.0 - rov.angular_velocity * 14.0).limit_length(30.0))
	var fwd := rov.global_basis.z
	var v := rov.linear_velocity.dot(fwd)
	rov.apply_central_force(fwd * clampf(63.0 * v_target * absf(v_target) + 80.0 * (v_target - v), -60.0, 60.0))


func _att_err() -> float:
	var want := Basis(Vector3.UP, yaw) * Basis(Vector3.RIGHT, -pitch)
	return rad_to_deg(Quaternion(want * rov.global_basis.orthonormalized().inverse()).get_angle())


func _drawn_curve() -> PackedVector3Array:
	# the centreline spool_tether.gd's _process draws (keep in step with it), from the attachment to the spool
	var n: int = tether.x.size() - 1
	var pts: PackedVector3Array = tether.x.duplicate()
	pts[0] = tether.attach_point()
	pts[n] = tether.anchor
	var cam_pos := cam.global_position
	var lod2: float = tether.lod_distance * tether.lod_distance
	var cos_bend := cos(deg_to_rad(tether.bend_subdivide_deg))
	var subs: int = tether.render_subdivisions
	var c := PackedVector3Array()
	for i in range(n):
		var p0: Vector3 = pts[max(i - 1, 0)]
		var p1: Vector3 = pts[i]
		var p2: Vector3 = pts[i + 1]
		var p3: Vector3 = pts[min(i + 2, n)]
		var sub := 1
		if subs > 1 and (p1.distance_squared_to(cam_pos) < lod2 or p2.distance_squared_to(cam_pos) < lod2):
			var d1 := (p2 - p1).normalized()
			if (p1 - p0).normalized().dot(d1) < cos_bend or d1.dot((p3 - p2).normalized()) < cos_bend:
				sub = subs
		if sub == 1:
			c.append(p1)
			continue
		var d01 := maxf(p0.distance_to(p1), 1e-6)
		var d12 := maxf(p1.distance_to(p2), 1e-6)
		var d23 := maxf(p2.distance_to(p3), 1e-6)
		var m1: Vector3 = ((p1 - p0) * (d12 / d01) * d12 + (p2 - p1) * d01) / (d01 + d12) if i > 0 else p2 - p1
		var m2: Vector3 = ((p2 - p1) * d23 + (p3 - p2) * (d12 / d23) * d12) / (d12 + d23) if i + 2 <= n else p2 - p1
		m1 = (p2 - p1).lerp(m1, minf(1.0, 2.0 * d01 / d12))
		m2 = (p2 - p1).lerp(m2, minf(1.0, 2.0 * d23 / d12))
		for sdiv in range(sub):
			var u := float(sdiv) / sub
			var u2 := u * u
			var u3 := u2 * u
			c.append(
				(
					(2.0 * u3 - 3.0 * u2 + 1.0) * p1
					+ (u3 - 2.0 * u2 + u) * m1
					+ (-2.0 * u3 + 3.0 * u2) * p2
					+ (u3 - u2) * m2
				)
			)
	c.append(pts[n])
	return c


static func _nearest(q: Vector3, line: PackedVector3Array) -> Array:
	# [distance, segment, t] of polyline `line`'s point nearest q
	var best := [INF, -1, 0.0]
	for k in range(line.size() - 1):
		var a: Vector3 = line[k]
		var d: Vector3 = line[k + 1] - a
		var u: float = clampf((q - a).dot(d) / maxf(d.length_squared(), 1e-12), 0.0, 1.0)
		var dist := q.distance_to(a + d * u)
		if dist < best[0]:
			best = [dist, k, u]
	return best


func _lens_cut(line: PackedVector3Array, radius: float) -> bool:
	# does the tube cross the near plane inside its rectangle (what shows as the cable cut open across the image)?
	var inv := cam.global_transform.affine_inverse()
	var hh := cam.near * tan(deg_to_rad(cam.fov) * 0.5)
	var size := cam.get_viewport().get_visible_rect().size
	var hw := hh * size.x / maxf(size.y, 1.0)
	var prev := inv * line[0]
	for k in range(1, line.size()):
		var cur := inv * line[k]
		var za := prev.z + cam.near
		var zb := cur.z + cam.near
		if za * zb <= 0.0 and za != zb:
			var c := prev.lerp(cur, za / (za - zb))
			if absf(c.x) <= hw + radius and absf(c.y) <= hh + radius:
				return true
		prev = cur
	return false


func _measure():
	if tether.x.size() < 2:
		return
	var cam_pos := cam.global_position
	var radius: float = (tether.draw_diameter if tether.draw_diameter > 0 else tether.diameter) / 2
	var line := _drawn_curve()
	var nd := _nearest(cam_pos, line)
	var surf: float = nd[0] - radius
	var cd := _nearest(cam_pos, tether.x)  # the particles' straight chords
	var along := 0.0
	if cd[1] >= 0:
		for j in range(cd[1]):
			along += tether.rest[j]
		along += cd[2] * tether.rest[cd[1]]
	var cut := _lens_cut(line, radius)
	# the cable's centreline inside the vehicle's collision shapes (not the attachment itself)
	var ti := rov.global_transform.affine_inverse()
	var deepest := 0.0
	var in_from := INF
	var in_to := -INF
	var s := 0.0
	for k in range(1, line.size()):
		s += line[k - 1].distance_to(line[k])
		tether._rov_closest(ti * line[k])
		if tether._rc_d < 0.0 and k < line.size() - 1:
			deepest = maxf(deepest, -tether._rc_d)
			in_from = minf(in_from, s)
			in_to = maxf(in_to, s)
	var near_lim: float = cam.near
	var row := [
		snappedf(t, 0.01),
		phase,
		snappedf(surf, 1e-4),
		snappedf(cd[0] - radius, 1e-4),
		snappedf(along, 0.001),
		int(cut),
		snappedf(deepest, 1e-4),
		snappedf(in_from, 0.001) if is_finite(in_from) else -1.0,
		snappedf(in_to, 0.001) if is_finite(in_to) else -1.0,
		snappedf(rov.linear_velocity.dot(rov.global_basis.z), 0.001),
		snappedf(_att_err(), 0.01),
		tether.x.size(),
		snappedf(tether.tension, 0.01),
		tether.rov_contacts,
		tether.get("cam_contacts") if tether.get("cam_contacts") != null else -1,
		snappedf(rov.global_position.distance_to(tether.anchor), 0.01),
		snappedf(tether.total, 0.01),
		tether.solve_usec,
		tether.get("cam_usec") if tether.get("cam_usec") != null else -1
	]
	series.append(row)
	var st: Dictionary = stats.get(
		phase,
		{
			"ticks": 0,
			"min_surf": INF,
			"min_chord": INF,
			"clip": 0,
			"close": 0,
			"close10": 0,
			"cut": 0,
			"hull": 0,
			"hull_max": 0.0,
			"speed_sum": 0.0,
			"att_max": 0.0,
			"solve": []
		}
	)
	st.ticks += 1
	st.min_surf = minf(st.min_surf, surf)
	st.min_chord = minf(st.min_chord, cd[0] - radius)
	st.clip += int(surf < near_lim)  # centreline nearer than near + the cable's radius
	st.close += int(surf < 0.05)
	st.close10 += int(surf < 0.10)
	st.cut += int(cut)
	st.hull += int(deepest > 0.0)
	st.hull_max = maxf(st.hull_max, deepest)
	st.speed_sum += rov.linear_velocity.dot(rov.global_basis.z)
	st.att_max = maxf(st.att_max, _att_err())
	st.solve.append(tether.solve_usec)
	stats[phase] = st
	if surf < worst.surf and t > 3.0:
		var seg: int = cd[1]
		worst = {
			"t": snappedf(t, 0.01),
			"phase": phase,
			"surf": surf,
			"chord_surf": cd[0] - radius,
			"along": along,
			"chord_segment": seg,
			"segments": tether.rest.size(),
			"in_skip": along < tether.rov_skip_length,
			"point": line[nd[1]].lerp(line[nd[1] + 1], nd[2]),
			"point_local": rov.to_local(line[nd[1]].lerp(line[nd[1] + 1], nd[2])),
			"cam_local": cam_local,
			"cut": cut,
			"rov_contacts": tether.rov_contacts,
			"hull_depth": deepest,
			"particle_near_local": rov.to_local(tether.x[min(seg + 1, tether.x.size() - 1)]),
			"rest_seg": tether.rest[seg] if seg >= 0 else 0.0
		}
	if phase == "reverse" and _a("worst", 1) == 1 and surf < worst_shot - 0.01:
		worst_shot = surf
		_shot("worst")


func _process(_delta):
	if rov == null or finished or tether._mesh == null or tether._mesh.get_surface_count() == 0:
		return
	# the mesh as drawn this frame: ring centres
	var verts: PackedVector3Array = tether._mesh.surface_get_arrays(0)[Mesh.ARRAY_VERTEX]
	var sides: int = tether.radial_sides
	var rings := PackedVector3Array()
	for k in range(verts.size() / sides):
		var c := Vector3.ZERO
		for s in range(sides):
			c += verts[k * sides + s]
		rings.append(c / sides)
	var radius: float = (tether.draw_diameter if tether.draw_diameter > 0 else tether.diameter) / 2
	var surf: float = _nearest(cam.global_position, rings)[0] - radius
	if t > 3.0:
		mesh.frames += 1
		mesh.clip_frames += int(surf < cam.near)
		mesh.close_frames += int(surf < 0.05)
		if surf < mesh.min_surf:
			mesh.min_surf = surf
			mesh.min_t = snappedf(t, 0.01)
	for st in shots:
		if t >= st and not shots_done.has(st):
			shots_done.append(st)
			_shot("t%05.2f" % st)


func _shot(name: String):
	if headless:
		return
	await RenderingServer.frame_post_draw
	var img := cam.get_viewport().get_texture().get_image()
	var p := out.path_join("%s_%s.png" % [variant, name])
	img.save_png(p)
	if name == "worst":
		worst_shot_t = t
	print("CAMCLIP shot %s t=%.2f" % [p, t])


func _finish():
	if finished:
		return
	finished = true
	var res := {
		"variant": variant, "info": stats.info, "worst": worst, "worst_shot_t": worst_shot_t, "mesh": mesh, "phases": {}
	}
	for k in stats:
		if k == "info":
			continue
		var st: Dictionary = stats[k]
		var sv: Array = st.solve
		sv.sort()
		res.phases[k] = {
			"ticks": st.ticks,
			"min_surf": st.min_surf,
			"min_chord_surf": st.min_chord,
			"clip_ticks": st.clip,
			"close5_ticks": st.close,
			"close10_ticks": st.close10,
			"lens_cut_ticks": st.cut,
			"hull_ticks": st.hull,
			"hull_max_depth": st.hull_max,
			"mean_speed": st.speed_sum / st.ticks,
			"att_err_max_deg": st.att_max,
			"solve_us_mean": _mean(sv),
			"solve_us_p95": sv[int(sv.size() * 0.95)]
		}
	var f := FileAccess.open(out.path_join(variant + "_series.csv"), FileAccess.WRITE)
	(
		f
		. store_line(
			"t,phase,surf,chord_surf,along,cut,hull_depth,hull_from,hull_to,speed,att_err,particles,tension,rov_contacts,cam_contacts,dist_anchor,paid,solve_us,cam_us"
		)
	)
	for row in series:
		f.store_line(",".join(row.map(func(e): return str(e))))
	f.close()
	var js := JSON.stringify(res, "  ")
	f = FileAccess.open(out.path_join(variant + "_summary.json"), FileAccess.WRITE)
	f.store_string(js)
	f.close()
	print("CAMCLIP " + JSON.stringify(res))
	get_tree().quit()


static func _mean(a: Array) -> float:
	var s := 0.0
	for e in a:
		s += e
	return s / maxf(a.size(), 1)
