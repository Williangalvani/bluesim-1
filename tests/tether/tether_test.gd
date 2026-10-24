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
#   --velocity-steps=N --position-steps=N (Jolt solver steps; default: project settings)
#   --render[=wide|side|attach] (show the simulated world on screen with a camera following the
#     ROV, for --write-movie, with live metrics; links bent past their joint limit are drawn red,
#     links sinking > 5 mm into geometry or the ROV magenta)
#   --seed=N (N > 0 nudges the ROV start by up to 2 mm, for repeat runs that diverge chaotically)
#   --attach=clamp|ball (clamp = rope/joint.tscn limits at the ROV as shipped, ball = free pivot)
#   --bend-limits=X,Y,Z (degrees; override rope/joint.tscn angular_limit_x/y/z on every tether
#     joint. Jolt treats X as twist and Y/Z as pyramid swing in the joint frame; the joint frame
#     is the parent section's, whose long axis is Y, so here X and Z bend the tether, Y twists it)
#   --bend-stiffness=K[,D] (override rope/joint.tscn angular springs on every tether joint:
#     stiffness K in N*m/rad and damping D in N*m*s/rad on all three axes; K=0 disables them)

const ROV_SCENE = preload("res://vehicles/bluerovheavy/BlueRovHeavy.tscn")
const TETHER_SCRIPT = preload("res://levels/tether.gd")
const SECTION_SPACING = 0.145  # levels/tether.gd section_length
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
var bend_override = Vector3.ZERO
var limits_deg = Vector3.ZERO  # actual tether joint angular limits (x, y, z), read at spawn
var spring_override = []  # [stiffness, damping] from --bend-stiffness
var spring = [0.0, 0.0]  # actual tether joint angular spring stiffness / damping, read at spawn
var velocity_steps = 0
var attach = "clamp"
var seed = 0
var render = false
var cam_mode = "wide"
var red_mat: StandardMaterial3D
var pen_mat: StandardMaterial3D
var hud: Label
var tick: ColorRect  # changes colour every physics frame, so frozen recordings are detectable
var follow_cam: Camera3D
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
			"bend-limits":
				var b = kv[1].split_floats(",")
				bend_override = Vector3(b[0], b[1], b[2])
			"bend-stiffness":
				spring_override = Array(kv[1].split_floats(","))
				if spring_override.size() < 2:
					spring_override.append(0.0)
			"velocity-steps": velocity_steps = int(kv[1])
			"attach": attach = kv[1]
			"seed": seed = int(kv[1])
			"render":
				render = true
				if kv.size() > 1:
					cam_mode = kv[1]
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
		if render:
			# put the private world on screen
			var layer = CanvasLayer.new()
			var box = SubViewportContainer.new()
			box.stretch = true
			box.set_anchors_preset(Control.PRESET_FULL_RECT)
			add_child(layer)
			layer.add_child(box)
			box.add_child(vp)
		else:
			add_child(vp)
		world_root = Node3D.new()
		vp.add_child(world_root)
	velocity_steps = ProjectSettings.get_setting("physics/jolt_physics_3d/simulation/velocity_steps")
	position_steps = ProjectSettings.get_setting("physics/jolt_physics_3d/simulation/position_steps")
	_build_world()
	_spawn_rov_and_tether()
	_build_plan()
	for k in ["link_err", "pen_static", "pen_rov", "self_clip", "rot_x", "rot_y", "rot_z", "bend_over", "visible_bend", "length_stretch", "zigzag"]:
		worst[k] = {"value": 0.0, "t": 0.0, "link": -1}
	ready_to_run = true
	print("TETHER_TEST start scenario=%s links=%d hz=%d steps=%d/%d attach=%s limits=%s spring=%s" % [scenario, links, hz, velocity_steps, position_steps, attach, limits_deg, spring])


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
	if render:
		red_mat = _mat(Color(1, 0.1, 0.1))
		red_mat.emission_enabled = true
		red_mat.emission = Color(1, 0, 0)
		pen_mat = _mat(Color(1, 0, 1))
		pen_mat.emission_enabled = true
		pen_mat.emission = Color(1, 0, 1)
		var layer = CanvasLayer.new()
		hud = Label.new()
		hud.position = Vector2(16, 12)
		hud.add_theme_font_size_override("font_size", 22)
		hud.add_theme_color_override("font_color", Color.WHITE)
		hud.add_theme_color_override("font_outline_color", Color.BLACK)
		hud.add_theme_constant_override("outline_size", 6)
		layer.add_child(hud)
		tick = ColorRect.new()
		tick.set_anchors_preset(Control.PRESET_TOP_RIGHT)
		tick.size = Vector2(32, 32)
		tick.position = Vector2(-32, 0)
		layer.add_child(tick)
		add_child(layer)
		follow_cam = Camera3D.new()
		follow_cam.fov = 60
		world_root.add_child(follow_cam)
		follow_cam.make_current()
		var env = WorldEnvironment.new()
		env.environment = Environment.new()
		env.environment.background_mode = Environment.BG_COLOR
		env.environment.background_color = Color(0.12, 0.3, 0.45)
		env.environment.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
		env.environment.ambient_light_color = Color(0.6, 0.7, 0.8)
		world_root.add_child(env)


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
	var start = START
	if seed > 0:
		var rng = RandomNumberGenerator.new()
		rng.seed = seed
		start += Vector3(rng.randf_range(-1, 1), rng.randf_range(-1, 1), rng.randf_range(-1, 1)) * 0.002
	vehicle.global_position = start
	rov = vehicle.get_node("BlueRov")
	for n in vehicle.find_children("*", "RigidBody3D", true, false):
		if n.get_parent() != tether:
			rov_rids.append(n.get_rid())
	for c in tether.get_children():
		if c is RigidBody3D:
			sections.append(c)
			tether_rids.append(c.get_rid())
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
	if bend_override != Vector3.ZERO:
		for j in joints:
			for ax in 3:
				var r = deg_to_rad(bend_override[ax])
				j.joint.set("angular_limit_%s/upper_angle" % "xyz"[ax], r)
				j.joint.set("angular_limit_%s/lower_angle" % "xyz"[ax], -r)
	if spring_override:
		for j in joints:
			for ax in "xyz":
				j.joint.set("angular_spring_%s/enabled" % ax, spring_override[0] > 0.0)
				j.joint.set("angular_spring_%s/stiffness" % ax, spring_override[0])
				j.joint.set("angular_spring_%s/damping" % ax, spring_override[1])
	var j1 = joints[1].joint
	if j1.get("angular_spring_x/enabled"):
		spring = [j1.get("angular_spring_x/stiffness"), j1.get("angular_spring_x/damping")]
	for ax in 3:
		limits_deg[ax] = rad_to_deg(j1.get("angular_limit_%s/upper_angle" % "xyz"[ax]))
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


# acceleration limits for the reference trajectory: ArduPilot's WPNAV_ACC / WPNAV_ACC_Z defaults
const ACCEL_H = 2.5
const ACCEL_V = 1.0


func _limit_accel(want, delta):
	var dv = want - ref_vel
	var h = Vector3(dv.x, 0, dv.z).limit_length(ACCEL_H * delta)
	return ref_vel + Vector3(h.x, clamp(dv.y, -ACCEL_V * delta, ACCEL_V * delta), h.z)


func _advance_plan(delta):
	open_force = Vector3.ZERO
	if step_i >= plan.size():
		done = true
		return
	var s = plan[step_i]
	step_t += delta
	var finished = false
	match s[0]:
		"hold":
			ref_vel = _limit_accel(Vector3.ZERO, delta)
			ref_pos += ref_vel * delta
			finished = step_t >= s[1]
		"go":
			var to = s[1] - ref_pos
			var dist = to.length()
			# slow down in time to stop at the waypoint without exceeding the decel limit
			var dir = to / max(dist, 1e-6)
			var a = lerp(ACCEL_H, ACCEL_V, abs(dir.y))
			var want = dir * min(s[2], sqrt(2.0 * a * dist))
			ref_vel = _limit_accel(want, delta)
			ref_pos += ref_vel * delta
			if dist < 0.01 and ref_vel.length() < 0.05:
				ref_pos = s[1]
				ref_vel = Vector3.ZERO
				finished = true
		"yaw":
			var total = deg_to_rad(s[1])
			var rate = deg_to_rad(s[2])
			ref_yaw = rate * min(step_t, total / rate)
			finished = step_t >= total / rate
		"force":
			# deliberately abusive: open-loop full thrust, no acceleration limit
			open_force = s[2]
			ref_pos = rov.global_position
			ref_vel = rov.linear_velocity
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
		print("TETHER_TEST first_frame=%d" % Engine.get_frames_drawn())
	t += delta
	frames += 1
	_advance_plan(delta)
	_drive(delta)
	if tick:
		tick.color = Color.from_hsv(fmod(frames * 0.37, 1.0), 1.0, 1.0)
	if follow_cam:
		# side-on view of the ROV and the first couple of metres of tether
		var p = rov.global_position
		if cam_mode == "attach":
			# close, from above and behind, on the ROV end of the tether
			var a = joints[0].a.global_transform * joints[0].la
			follow_cam.global_position = a + Vector3(-0.3, 0.45, 0.4)
			follow_cam.look_at(a + Vector3(-0.2, 0, 0))
		elif cam_mode == "side":
			follow_cam.global_position = p + Vector3(-0.9, 0.5, 2.2)
			follow_cam.look_at(p + Vector3(-0.9, -0.1, 0))
		else:
			# wide chase view: the ROV, a few metres of tether and nearby obstacles
			var target = p + Vector3(-1.0, -0.4, 0)
			var want = target + Vector3(-2.1, 1.9, 2.8)
			follow_cam.global_position = want if frames < 2 else follow_cam.global_position.lerp(want, 0.05)
			follow_cam.look_at(target)
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
	var vis_max = 0.0
	var over_max = 0.0
	for i in joints.size():
		var j = joints[i]
		var pa = j.a.global_transform * j.la
		var pb = j.b.global_transform * j.lb
		var e = pa.distance_to(pb)
		errs.append(e)
		_track("link_err", e, i)
		if i > 0:  # ROV->first section link has a different reference frame
			# rotation of B relative to A since creation, in the joint frame, decomposed the
			# way Jolt's 6DOF constraint limits it: twist about X, pyramid swing about Y and Z
			var q = ((j.a.global_transform.basis.inverse() * j.b.global_transform.basis).orthonormalized() * j.rel0.inverse()).get_rotation_quaternion()
			if q.w < 0:
				q = -q
			var qt = Quaternion(q.x, 0, 0, q.w).normalized()
			var sw = q * qt.inverse()
			var d = Vector3(abs(rad_to_deg(2.0 * atan2(qt.x, qt.w))),
					abs(rad_to_deg(2.0 * atan2(sw.y, sw.w))),
					abs(rad_to_deg(2.0 * atan2(sw.z, sw.w))))
			bend = Vector3(max(bend.x, d.x), max(bend.y, d.y), max(bend.z, d.z))
			_track("rot_x", d.x, i); _track("rot_y", d.y, i); _track("rot_z", d.z, i)
			# what you can actually see: the angle between the two links' long axes
			var vis = rad_to_deg(j.a.global_transform.basis.y.angle_to(j.b.global_transform.basis.y))
			_track("visible_bend", vis, i)
			vis_max = max(vis_max, vis)
			var over = (d - limits_deg).max_axis_index()
			var over_deg = (d - limits_deg)[over]
			_track("bend_over", over_deg, i)
			over_max = max(over_max, over_deg)
			if over_deg > 2.0:
				bend_viol += 1
			if render:
				var mi = j.b.get_node("MeshInstance3D")
				mi.material_override = red_mat if over_deg > 2.0 else null
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
	s["rot_max"] = _v(bend)  # degrees: twist about joint X, swing about Y, swing about Z
	s["bend_violations"] = bend_viol
	s["visible_bend_max"] = snapped(vis_max, 0.01)
	s["bend_over_max"] = snapped(over_max, 0.01)
	# sawtooth: neighbouring joints bending in opposite directions. For three consecutive links,
	# a smooth arc turns by the sum of both joint bends from first to last link, a zigzag turns
	# by ~0, so half the shortfall is the zigzag amplitude in degrees (0 for any smooth curve)
	var zz_max = 0.0
	var zz_sum = 0.0
	for k in range(1, sections.size() - 1):
		var a0 = sections[k - 1].global_transform.basis.y
		var a1 = sections[k].global_transform.basis.y
		var a2 = sections[k + 1].global_transform.basis.y
		var zz = rad_to_deg(a0.angle_to(a1) + a1.angle_to(a2) - a0.angle_to(a2)) / 2.0
		_track("zigzag", zz, k)
		zz_max = max(zz_max, zz)
		zz_sum += zz
	s["zigzag_max"] = snapped(zz_max, 0.01)
	s["zigzag_mean"] = snapped(zz_sum / max(1, sections.size() - 2), 0.001)

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
		var clipped = d > 0.005
		pen_static = max(pen_static, d)
		if d > 0.005:
			n_pen += 1
		_track("pen_static", d, i)
		if i > 1:  # the first sections sit against the ROV by design
			params.exclude = _query_exclude(false, true)
			d = _max_depth(space, params)
			clipped = clipped or d > 0.005
			pen_rov = max(pen_rov, d)
			_track("pen_rov", d, i)
		if render and clipped:
			sec.get_node("MeshInstance3D").material_override = pen_mat
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
			var jn = joints[i + 1]
			var anchor = ((jn.a.global_transform * jn.la) + (jn.b.global_transform * jn.lb)) / 2.0
			var hit = {}
			for seg in [[sec.global_position, anchor], [anchor, sections[i + 1].global_position]]:
				var ray = PhysicsRayQueryParameters3D.create(seg[0], seg[1])
				ray.exclude = _query_exclude(true, false)
				hit = space.intersect_ray(ray)
				if hit:
					break
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
	if hud:
		var step = plan[min(step_i, plan.size() - 1)]
		hud.text = ("%s   t = %.1f s   step: %s\n" % [scenario, t, step[0]]
			+ "worst joint gap %.1f mm   stretch %.2f %%\n" % [s.link_err_max * 1000.0, stretch * 100.0]
			+ "into geometry %.1f mm   into ROV %.1f mm   self %.1f mm\n" % [pen_static * 1000.0, pen_rov * 1000.0, self_clip * 1000.0]
			+ "past joint limit %.1f deg   zigzag %.1f deg\n" % [s.bend_over_max, s.zigzag_max]
			+ "(red = past limit, magenta = > 5 mm into something)")


func _track(key, value, link):
	if value > worst[key].value:
		worst[key] = {"value": snapped(value, 0.0001), "t": snapped(t, 0.01), "link": link}


func _v(v):
	return [snapped(v.x, 0.001), snapped(v.y, 0.001), snapped(v.z, 0.001)]


func _finish():
	set_physics_process(false)
	var result = {
		"scenario": scenario, "links": links, "hz": hz, "limits_deg": _v(limits_deg), "spring": spring,
		"velocity_steps": velocity_steps, "position_steps": position_steps, "attach": attach, "seed": seed,
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
