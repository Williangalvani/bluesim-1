extends Node3D
# Tether validation environment: flies the real BlueROV2 Heavy (with the real tether
# from levels/tether.gd) through scripted scenarios around simple obstacles and
# measures how much the tether stretches and clips into geometry.
#
# Run one scenario:
#   godot --headless --path . res://tests/tether/tether_test.tscn -- --scenario=tow --out=/tmp/tow.json
# Run them all: tests/tether/run_tether_tests.py
#
# Options (after "--"): --scenario=NAME --out=PATH --links=N (default 200, as in baselevel)
#   --hz=N (physics rate, default 120 like the game) --sample-every=N (physics frames)
#   --drag-axis=z|y (z = rope/section.gd as shipped, y = drag across the capsule's long axis)
#   --velocity-steps=N --position-steps=N (Jolt solver steps; default: project settings)
#   --attach=clamp|ball (clamp = rope/joint.tscn limits at the ROV as shipped, ball = free pivot)

const ROV_SCENE = preload("res://vehicles/bluerovheavy/BlueRovHeavy.tscn")
const TETHER_SCRIPT = preload("res://levels/tether.gd")
const SECTION_SPACING = 0.145  # levels/tether.gd section_length
const BEND_LIMITS_DEG = Vector3(10, 10, 2)  # rope/joint.tscn angular limits (x, y, z)
const START = Vector3(0, 1.5, 0)

# obstacles: kept clear of the initial tether line (y=1.5, z=0, x<0)
const WALL = {"pos": Vector3(6, 0.6, 5), "size": Vector3(0.4, 1.2, 6)}  # sharp top edge at y=1.2
const PLATE = {"pos": Vector3(-3, 0.6, 6), "size": Vector3(6, 1.2, 0.02)}  # 2 cm thin plate
const PILLAR = {"pos": Vector3(4, 1.25, -4), "radius": 0.25, "height": 2.5}

var scenario = "rest"
var out_path = "user://tether_test.json"
var links = 200
var hz = 120
var sample_every = 4
var drag_axis = "z"
var velocity_steps = 0
var attach = "clamp"
var position_steps = 0

var rov: RigidBody3D
var tether: Node
var sections = []
var joints = []  # [{joint, a, b, la, lb, rel0}]
var tether_rids = []
var rov_rids = []
var static_rids = []
var t = 0.0
var frames = 0
var plan = []  # list of steps, see _build_plan
var step_i = 0
var step_t = 0.0
var ref_pos = START
var ref_vel = Vector3.ZERO
var ref_yaw = 0.0
var integ = Vector3.ZERO
var open_force = Vector3.ZERO
var done = false
var ready_to_run = false
var world_root: Node3D = self  # everything simulated lives under here

# results
var series = []
var worst = {}
var through_events = []
var through_count = 0
var exploded = false
var phys_ms = []


func _ready():
	for a in OS.get_cmdline_user_args():
		var kv = a.trim_prefix("--").split("=", true, 1)
		match kv[0]:
			"scenario": scenario = kv[1]
			"out": out_path = kv[1]
			"links": links = int(kv[1])
			"hz": hz = int(kv[1])
			"sample-every": sample_every = int(kv[1])
			"drag-axis": drag_axis = kv[1]
			"velocity-steps": velocity_steps = int(kv[1])
			"attach": attach = kv[1]
			"position-steps": position_steps = int(kv[1])
	Engine.physics_ticks_per_second = hz
	if velocity_steps or position_steps:
		# Jolt applies these when a space is created, so set them and then simulate in a
		# fresh World3D (new space) instead of the one created at startup
		if velocity_steps:
			ProjectSettings.set_setting("physics/jolt_physics_3d/simulation/velocity_steps", velocity_steps)
		if position_steps:
			ProjectSettings.set_setting("physics/jolt_physics_3d/simulation/position_steps", position_steps)
		await ProjectSettings.settings_changed
		var vp = SubViewport.new()
		vp.own_world_3d = true
		add_child(vp)
		world_root = Node3D.new()
		vp.add_child(world_root)
	velocity_steps = ProjectSettings.get_setting("physics/jolt_physics_3d/simulation/velocity_steps")
	position_steps = ProjectSettings.get_setting("physics/jolt_physics_3d/simulation/position_steps")
	_build_world()
	_spawn_rov_and_tether()
	_build_plan()
	for k in ["link_err", "pen_static", "pen_rov", "self_clip", "bend_x", "bend_z", "twist", "length_stretch"]:
		worst[k] = {"value": 0.0, "t": 0.0, "link": -1}
	ready_to_run = true
	print("TETHER_TEST start scenario=%s links=%d hz=%d drag_axis=%s steps=%d/%d attach=%s" % [scenario, links, hz, drag_axis, velocity_steps, position_steps, attach])


# ---------------------------------------------------------------- world

func _box(name, pos, size, color):
	var body = StaticBody3D.new()
	body.name = name
	var shape = CollisionShape3D.new()
	shape.shape = BoxShape3D.new()
	shape.shape.size = size
	body.add_child(shape)
	var mesh = MeshInstance3D.new()
	mesh.mesh = BoxMesh.new()
	mesh.mesh.size = size
	mesh.material_override = _mat(color)
	body.add_child(mesh)
	world_root.add_child(body)
	body.global_position = pos
	static_rids.append(body.get_rid())


func _mat(color):
	var m = StandardMaterial3D.new()
	m.albedo_color = color
	return m


func _build_world():
	_box("floor", Vector3(-8, -0.5, 0), Vector3(50, 1, 40), Color(0.35, 0.45, 0.55))
	_box("wall", WALL.pos, WALL.size, Color(0.8, 0.4, 0.3))
	_box("plate", PLATE.pos, PLATE.size, Color(0.3, 0.8, 0.4))
	var pillar = StaticBody3D.new()
	pillar.name = "pillar"
	var shape = CollisionShape3D.new()
	shape.shape = CylinderShape3D.new()
	shape.shape.radius = PILLAR.radius
	shape.shape.height = PILLAR.height
	pillar.add_child(shape)
	var mesh = MeshInstance3D.new()
	mesh.mesh = CylinderMesh.new()
	mesh.mesh.top_radius = PILLAR.radius
	mesh.mesh.bottom_radius = PILLAR.radius
	mesh.mesh.height = PILLAR.height
	mesh.material_override = _mat(Color(0.8, 0.8, 0.3))
	pillar.add_child(mesh)
	world_root.add_child(pillar)
	pillar.global_position = PILLAR.pos
	static_rids.append(pillar.get_rid())

	var light = DirectionalLight3D.new()
	light.rotation_degrees = Vector3(-60, 30, 0)
	world_root.add_child(light)
	var cam = Camera3D.new()
	cam.name = "camera"
	world_root.add_child(cam)
	cam.global_position = Vector3(-6, 9, 12)
	cam.look_at(Vector3(0, 0.5, 0))


func _spawn_rov_and_tether():
	var vehicle = ROV_SCENE.instantiate()
	tether = Node3D.new()
	tether.name = "tether"
	tether.set_script(TETHER_SCRIPT)
	tether.straight_loops = links
	tether.coiled_loops = 0
	tether.position = Vector3(-0.53995, 0, 0)  # same offset as levels/baselevel.tscn
	vehicle.add_child(tether)
	world_root.add_child(vehicle)
	vehicle.global_position = START
	rov = vehicle.get_node("BlueRov")
	for n in vehicle.find_children("*", "RigidBody3D", true, false):
		if n.get_parent() != tether:
			rov_rids.append(n.get_rid())
	for c in tether.get_children():
		if c is RigidBody3D:
			sections.append(c)
			tether_rids.append(c.get_rid())
	if drag_axis == "y":
		var fixed = load("res://tests/tether/section_drag_y.gd")
		for s in sections:
			s.set_script(fixed)
	# joints: first one hangs off the ROV, the rest off each section (see tether.gd addLink)
	for j in [rov.get_node("joint")] + sections.map(func(s): return s.get_node_or_null("joint")):
		if j == null:
			continue
		var a = j.get_node(j.node_a)
		var b = j.get_node(j.node_b)
		var anchor = j.global_position
		joints.append({
			"joint": j, "a": a, "b": b,
			"la": a.global_transform.affine_inverse() * anchor,
			"lb": b.global_transform.affine_inverse() * anchor,
			"rel0": (a.global_transform.basis.inverse() * b.global_transform.basis).orthonormalized(),
		})
	if attach == "ball":
		for ax in ["x", "y", "z"]:
			joints[0].joint.set("angular_limit_%s/enabled" % ax, false)
	assert(sections.size() == links, "expected %d sections, got %d" % [links, sections.size()])


# ---------------------------------------------------------------- scenarios
# steps: ["hold", seconds] | ["go", target, speed] | ["yaw", degrees, deg_per_s]
#        ["force", seconds, force_vector]  (open loop, attitude still held)

func _build_plan():
	var y = START.y
	match scenario:
		"rest":
			plan = [["hold", 15.0]]
		"tow":
			plan = [["hold", 3.0], ["go", Vector3(12, y, 0), 1.0], ["hold", 5.0]]
		"sprint":
			plan = [["hold", 3.0], ["force", 2.0, Vector3(150, 0, 0)], ["force", 2.0, Vector3(-150, 0, 0)],
					["force", 1.5, Vector3(150, 0, 0)], ["hold", 5.0]]
		"spin":
			plan = [["hold", 3.0], ["yaw", 4 * 360.0, 90.0], ["hold", 5.0]]
		"circle":
			plan = [["hold", 2.0], ["go", Vector3(4 - 1.2, 1.0, -4), 0.6]]
			for k in range(1, 2 * 24 + 1):  # two loops around the pillar, 15 deg per waypoint
				var ang = PI + k * TAU / 24.0
				plan.append(["go", Vector3(4 + 1.2 * cos(ang), 1.0, -4 + 1.2 * sin(ang)), 0.6])
			plan.append(["hold", 5.0])
		"edge_drag":
			plan = [["hold", 2.0], ["go", Vector3(0, 1.8, 5), 0.8], ["go", Vector3(9, 1.8, 5), 0.8],
					["go", Vector3(9, 0.5, 5), 0.5], ["go", Vector3(13, 0.5, 5), 0.8],
					["go", Vector3(13, 0.5, 2), 0.8], ["hold", 5.0]]
		"thin_plate":
			plan = [["hold", 2.0], ["go", Vector3(-3, 1.8, 3), 0.8], ["go", Vector3(-3, 1.8, 9), 0.8],
					["go", Vector3(-3, 0.5, 9), 0.5], ["go", Vector3(-3, 0.5, 13), 0.8],
					["go", Vector3(1, 0.5, 13), 0.8], ["hold", 5.0]]
		"floor_drag":
			plan = [["hold", 2.0], ["go", Vector3(0, 0.35, 0), 0.4], ["hold", 6.0],
					["go", Vector3(0, 0.35, -2), 0.6], ["go", Vector3(-4, 0.35, -2), 0.6],
					["go", Vector3(-4, 0.35, 2), 0.6], ["go", Vector3(2, 0.35, 2), 0.6], ["hold", 5.0]]
		_:
			push_error("unknown scenario " + scenario)
			plan = [["hold", 0.1]]


func _advance_plan(delta):
	open_force = Vector3.ZERO
	ref_vel = Vector3.ZERO
	if step_i >= plan.size():
		done = true
		return
	var s = plan[step_i]
	step_t += delta
	var finished = false
	match s[0]:
		"hold":
			finished = step_t >= s[1]
		"go":
			var to = s[1] - ref_pos
			var d = s[2] * delta
			if to.length() <= d:
				ref_pos = s[1]
				finished = true
			else:
				ref_vel = to.normalized() * s[2]
				ref_pos += ref_vel * delta
		"yaw":
			var total = deg_to_rad(s[1])
			var rate = deg_to_rad(s[2])
			ref_yaw = rate * min(step_t, total / rate)
			finished = step_t >= total / rate
		"force":
			open_force = s[2]
			ref_pos = rov.global_position  # follow wherever it ends up
			finished = step_t >= s[1]
	if finished:
		step_i += 1
		step_t = 0.0


# ---------------------------------------------------------------- ROV autopilot

func _drive(delta):
	var m = rov.mass
	var g = ProjectSettings.get_setting("physics/3d/default_gravity") * rov.gravity_scale
	var f = Vector3(0, m * g, 0)  # stand-in for environment.gd buoyancy: neutral ROV
	if open_force != Vector3.ZERO:
		f += open_force
	else:
		var e = ref_pos - rov.global_position
		integ = (integ + e * delta).limit_length(2.0)
		var cmd = m * (4.0 * e + 4.0 * (ref_vel - rov.linear_velocity)) + m * 1.0 * integ
		f += cmd.limit_length(150.0)  # roughly the Heavy's horizontal thrust
	rov.apply_central_force(f)
	# attitude: level, yaw = ref_yaw about world up (ROV faces +X at yaw 0)
	var target = Basis(Vector3.UP, ref_yaw) * _rov_basis0
	var err = (target * rov.global_transform.basis.inverse()).orthonormalized().get_rotation_quaternion()
	if err.w < 0:
		err = -err
	var axis_angle = Vector3(err.x, err.y, err.z) * 2.0
	var tau = (40.0 * axis_angle - 12.0 * rov.angular_velocity).limit_length(40.0)
	rov.apply_torque(tau)

var _rov_basis0: Basis


# ---------------------------------------------------------------- measurements

func _physics_process(delta):
	if not ready_to_run:
		return
	if frames == 0:
		_rov_basis0 = rov.global_transform.basis
	t += delta
	frames += 1
	_advance_plan(delta)
	_drive(delta)
	if frames % sample_every == 0:
		_sample()
	if done or exploded:
		_finish()


func _query_exclude(include_static, include_rov):
	var ex = tether_rids.duplicate()
	if not include_static:
		ex.append_array(static_rids)
	if not include_rov:
		ex.append_array(rov_rids)
	return ex


func _max_depth(space, params):
	var pts = space.collide_shape(params, 16)
	var depth = 0.0
	for k in range(0, pts.size() - 1, 2):
		depth = max(depth, pts[k].distance_to(pts[k + 1]))
	return depth


func _sample():
	var space = world_root.get_world_3d().direct_space_state
	var s = {"t": snapped(t, 0.01), "rov": _v(rov.global_position), "ref": _v(ref_pos)}

	# stretch: separation of each joint's anchor as seen from its two bodies
	var errs = []
	var length = 0.0
	var bend = Vector3.ZERO
	var bend_viol = 0
	for i in joints.size():
		var j = joints[i]
		var pa = j.a.global_transform * j.la
		var pb = j.b.global_transform * j.lb
		var e = pa.distance_to(pb)
		errs.append(e)
		_track("link_err", e, i)
		if i > 0:  # ROV->first section link has a different reference frame
			# rotation of B relative to A since creation, in A's (= the joint's) frame,
			# split into twist about the long axis (local Y) and swing about X / Z
			var q = ((j.a.global_transform.basis.inverse() * j.b.global_transform.basis).orthonormalized() * j.rel0.inverse()).get_rotation_quaternion()
			if q.w < 0:
				q = -q
			var twist = abs(rad_to_deg(2.0 * atan2(q.y, q.w)))
			var qt = Quaternion(0, q.y, 0, q.w).normalized()
			var sw = q * qt.inverse()
			var sv = Vector3(sw.x, sw.y, sw.z) * (2.0 * acos(clamp(sw.w, -1.0, 1.0)) / max(sqrt(1.0 - sw.w * sw.w), 1e-6))
			var d = Vector3(abs(rad_to_deg(sv.x)), twist, abs(rad_to_deg(sv.z)))
			bend = Vector3(max(bend.x, d.x), max(bend.y, d.y), max(bend.z, d.z))
			_track("bend_x", d.x, i); _track("twist", d.y, i); _track("bend_z", d.z, i)
			if d.x > BEND_LIMITS_DEG.x + 2 or d.y > BEND_LIMITS_DEG.y + 2 or d.z > BEND_LIMITS_DEG.z + 2:
				bend_viol += 1
	# the sections themselves are rigid, so all elongation is in the joint gaps
	length = sections.size() * SECTION_SPACING + errs.reduce(func(x, y): return x + y, 0.0)
	var stretch = length / (sections.size() * SECTION_SPACING) - 1.0
	_track("length_stretch", stretch, -1)
	s["link0_err"] = snapped(errs[0], 0.0001)  # ROV -> first section
	errs.sort()
	s["link_err_max"] = snapped(errs[-1], 0.0001)
	s["link_err_p99"] = snapped(errs[int(errs.size() * 0.99)], 0.0001)
	s["link_err_mean"] = snapped(errs.reduce(func(a, b): return a + b, 0.0) / errs.size(), 0.0001)
	s["length_stretch"] = snapped(stretch, 0.0001)
	s["bend_max"] = _v(bend)  # [about X, twist about Y, about Z] in degrees
	s["bend_violations"] = bend_viol

	# clipping
	var pen_static = 0.0
	var pen_rov = 0.0
	var self_clip = 0.0
	var n_pen = 0
	var max_speed = 0.0
	var params = PhysicsShapeQueryParameters3D.new()
	params.collide_with_areas = false
	for i in sections.size():
		var sec = sections[i]
		var cs = sec.get_node("CollisionShape3D")
		params.shape = cs.shape
		params.transform = cs.global_transform
		params.exclude = _query_exclude(true, false)
		var d = _max_depth(space, params)
		pen_static = max(pen_static, d)
		if d > 0.005:
			n_pen += 1
		_track("pen_static", d, i)
		if i > 1:  # the first sections sit against the ROV by design
			params.exclude = _query_exclude(false, true)
			d = _max_depth(space, params)
			pen_rov = max(pen_rov, d)
			_track("pen_rov", d, i)
		# self: other sections except direct neighbours (their capsules overlap by design)
		var ex = static_rids + rov_rids + [sec.get_rid()]
		for k in [i - 2, i - 1, i + 1, i + 2]:
			if k >= 0 and k < sections.size():
				ex.append(sections[k].get_rid())
		params.exclude = ex
		d = _max_depth(space, params)
		self_clip = max(self_clip, d)
		_track("self_clip", d, i)
		max_speed = max(max_speed, sec.linear_velocity.length())
		if not sec.global_position.is_finite() or sec.linear_velocity.length() > 100.0:
			exploded = true
		# links passing straight through solid geometry
		if i + 1 < sections.size():
			var ray = PhysicsRayQueryParameters3D.create(sec.global_position, sections[i + 1].global_position)
			ray.exclude = _query_exclude(true, false)
			var hit = space.intersect_ray(ray)
			if hit:
				through_count += 1
				if through_events.size() < 25:
					through_events.append({"t": snapped(t, 0.01), "link": i, "at": _v(hit.position),
						"collider": str(hit.collider.name)})
	s["pen_static_max"] = snapped(pen_static, 0.0001)
	s["pen_static_n"] = n_pen
	s["pen_rov_max"] = snapped(pen_rov, 0.0001)
	s["self_clip_max"] = snapped(self_clip, 0.0001)
	s["section_speed_max"] = snapped(max_speed, 0.01)
	s["below_floor"] = sections.filter(func(x): return x.global_position.y < -0.02).size()
	s["phys_ms"] = snapped(Performance.get_monitor(Performance.TIME_PHYSICS_PROCESS) * 1000.0, 0.01)
	series.append(s)


func _track(key, value, link):
	if value > worst[key].value:
		worst[key] = {"value": snapped(value, 0.0001), "t": snapped(t, 0.01), "link": link}


func _v(v):
	return [snapped(v.x, 0.001), snapped(v.y, 0.001), snapped(v.z, 0.001)]


func _finish():
	set_physics_process(false)
	var result = {
		"scenario": scenario, "links": links, "hz": hz, "drag_axis": drag_axis,
		"velocity_steps": velocity_steps, "position_steps": position_steps, "attach": attach,
		"engine": Engine.get_version_info().string,
		"physics_engine": ProjectSettings.get_setting("physics/3d/physics_engine"),
		"duration": snapped(t, 0.01), "completed": done, "exploded": exploded,
		"worst": worst, "through_count": through_count, "through_events": through_events,
		"series": series,
	}
	var f = FileAccess.open(out_path, FileAccess.WRITE)
	f.store_string(JSON.stringify(result))
	f.close()
	print("TETHER_TEST done %s -> %s" % [scenario, out_path])
	get_tree().quit()
