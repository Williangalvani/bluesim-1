extends Node3D
# ROV tether paid out from a spool topside: an XPBD particle rope (no rigid bodies) solved in GDScript.
#
# Particle 0 rides on the vehicle at this node's position; the last particle is pinned, every solve, at the spool: the
# boat's TetherAnchor marker (group tether_anchors, vehicles/boat/boat.tscn) if there is one, else a placeholder spool
# floating at the surface behind the vehicle. The segment next to the spool is
# the cable leaving it: its constraint can only pull with the brake's tension (brake_tension plus payout_damping per
# m/s), so when the vehicle pulls harder the segment grows, i.e. the spool pays out, until max_length is out and the
# tether becomes a hard stop. Once that segment is a full spacing long a new particle is split off it, so particles
# keep a constant spacing next to the vehicle (fine_segments of `spacing`) and a wider one further out, which keeps
# the particle count under max_particles for any max_length. With reel_in the spool takes in slack.
# Each particle collides with the level (sphere queries, time-sliced and only for particles that moved, cached as a
# contact plane that every constraint sweep projects onto, with Coulomb friction so it stays wrapped), and a
# unilateral constraint between particles two apart keeps it from bending tighter than min_bend_radius (a joint may
# turn by its segments' length over that radius, and never more than max_bend_deg). Where the cable bends (round an
# obstacle, an edge or just under load) the segments at the joint are split, graded down to bend_spacing next to it,
# for as long as the bend is there, so a wrap or a corner is drawn and solved with a tight radius.
# The vehicle's own collision shapes (boxes, spheres, capsules, convex hulls as their bounding box, read once from its
# CollisionShape3D children) push the cable out analytically every sweep, except the first rov_skip_length m next to
# the attachment; the vehicle gets the reaction (capped at rov_max_force) at the contact, so it can push the cable.
# Its gripper's claws (rigid bodies jointed to it) do the same where they are moving to, all along the cable, with a
# contact radius small enough for the cable to sit between the jaws; each claw gets its own reaction.
# Closed round the cable, the jaws hold it (grip): the particle at the jaw point is moved with the vehicle like the
# attachment, the vehicle feels the pull on both sides of it there, and above grip_slip_force the cable slips out.
# The pull of the first segment is applied to the vehicle at the attachment point. It is drawn as a Catmull-Rom
# tube, subdivided only where the cable bends and is near the camera.

const WATER_DENSITY = 1000.0
const G = 9.8

@export var max_length := 50.0  # m on the spool
@export var initial_length := 0.0  # m out at start; 0: the distance to the spool plus initial_slack
@export var initial_slack := 0.0  # slack laid out straight snaps taut as it falls; the spool pays out what it needs
@export var spacing := 1.0  # m between particles next to the vehicle
@export var fine_segments := 24  # segments of `spacing` next to the vehicle; the rest share what is left of max_particles
@export var max_particles := 64
@export var fixed_count := false  # compare: max_particles from the start, every segment stretches as it pays out
@export var substeps := 1  # per physics tick; 2 halves the stretch for about 1.8x the solve cost
@export var iterations := 4
@export var diameter := 0.0076  # m, Fathom slim tether
@export var buoyancy_ratio := 1.05
@export var normal_cd := 1.2
@export var axial_cf := 0.005  # skin friction along the cable (halved from 0.01: it slides along itself more freely)
@export var compliance_per_m := 1.3e-5  # m/N per m of tether
@export var max_force := 200.0  # N, the most the tether may pull on the vehicle
@export_group("Spool")
@export var anchor_path: NodePath  # the spool's exit; empty: look for a node named TetherAnchor, else a placeholder
@export var placeholder_distance := 4.0  # m behind the vehicle, at the surface
@export var brake_tension := 5.0  # N the spool holds before it pays out
@export var payout_damping := 8.0  # N per m/s of payout
@export var max_payout_speed := 3.0  # m/s
@export var stop_length := 2.0  # m before the spool is empty over which the brake rises to max_force, a soft end stop
@export var reel_in := false  # take in slack at reel_speed, down to min_length
@export var reel_speed := 0.5  # m/s
@export var reel_tension := 2.0  # N the winch winds in against
@export var min_length := 2.0
@export_group("Obstacles")
@export var collide := true
@export_flags_3d_physics var collision_mask := 1
@export var contact_radius := 0.08  # m kept between the particles and surfaces, so the chords between them stay out too
@export var query_margin := 0.4  # m, contacts are looked for this far ahead
@export var queries_per_tick := 16  # most sphere queries a tick; particles that haven't moved aren't queried again
@export var friction := 0.5
@export var max_bend_deg := 90.0  # largest turn between neighbouring segments; 180 turns the bending limit off
@export var min_bend_radius := 0.1  # m; a joint turns at most (its segments' mean length / this), up to max_bend_deg
@export var bend_compliance := 1e-4
@export var bend_max_force := 2.0  # N; a tether only resists bending weakly, and a hard limit would snap a folded pile open
@export_group("Adaptive")
@export var segment_collide := true  # capsule queries along segments too, so thin edges can't slip between particles
## queries a tick spent re-asking contacts that haven't moved (so the cable can slide round corners)
@export var contact_refresh_per_tick := 2
## segment contacts this close to an end (fraction of its length) are left to the particle there
@export var seg_end_t := 0.0
@export var segment_queries_per_tick := 16
@export var adaptive := true  # split segments near solids (down to near_spacing) and at bends (down to spacing)
@export var near_spacing := 0.25  # m, shortest segment next to a solid
@export var refine_bend_deg := 20.0  # split the segments round a joint that turns more than this
@export var bend_spacing := 0.12  # m, shortest segment a bend next to a solid (or the hull) is split down to; graded x2
@export var free_bend_spacing := 0.5  # m, the same for a bend out in open water (a slack, swirling cable)
@export var cull := true  # merge particles on straight sections away from solids, up to max_segment
@export var cull_straight_deg := 4.0  # a joint this straight (and clear of solids for merge_delay ticks) can go
@export var max_segment := 6.0  # m
@export var merge_delay := 60  # ticks clear of solids before a particle can be merged away
@export var merge_max_stretch := 0.005  # m: no merge next to a segment stretched more than this (it would let it go)
@export var max_total_particles := 128
@export var topology_changes_per_tick := 2  # splits/merges a tick, so nothing pops
@export var topology_scan_per_tick := 24  # joints checked for bends/culling a tick
@export_group("Vehicle contact")
@export var rov_collide := true  # the vehicle's hull pushes the cable (and feels it), away from the attachment
@export var rov_skip_length := 1.75  # m of cable from the attachment that may touch the hull (it leaves through it)
@export var rov_friction := 0.3
@export var rov_max_force := 80.0  # N, cap on the cable's push on the vehicle in one tick
@export_group("Claws")
@export var claw_collide := true  # off: the claws are level geometry to the queries (one way, planes cached)
@export var claw_contact_radius := 0.012  # m: the cable's radius and a skin, so it fits between the jaws
@export var claw_friction := 0.3
@export var claw_max_force := 40.0  # N, cap on the cable's push on each claw in one tick
@export_group("Grip")
@export var grip := true  # jaws closed round the cable hold it: the particle at the jaw point is pinned there
@export var grip_radius := 0.05  # m, how near the jaw point the cable must be as they close
@export var grip_window := 0.5  # s after closing in which it is still taken (later it can only be outside them)
@export var grip_slip_force := 50.0  # N the cable may pull on the jaws, for grip_slip_ticks, before it slips out
@export var grip_slip_ticks := 6
@export var grip_smoothing := 0.3  # weight of each tick in the moving average of the pull on the jaws
@export var grip_min_piece := 0.03  # m, the pieces either side of the held particle (the least a split there leaves)
@export var jaw_point := Vector3(-0.02, -0.25, 0.53)  # in the vehicle's frame; a JawPoint marker on it wins
@export_group("Drawing")
@export var render_subdivisions := 4  # spline points per bent segment near the camera, straight or far ones get 1
@export var bend_subdivide_deg := 6.0
@export var lod_distance := 25.0
@export var radial_sides := 6
@export var draw_diameter := 0.0  # m, 0: the real diameter; thicker makes it visible from far away
@export var color := Color(0.96, 0.75, 0.04)
@export_group("Debug")
@export var topology_log := false  # record each split/merge/payout and per-tick motion into topo_events/topo_ticks

var rov: RigidBody3D
var attach_local := Vector3.ZERO
var anchor_node: Node3D
var anchor := Vector3.ZERO
var total := 0.0  # m paid out
var far_spacing := 1.0
var x := PackedVector3Array()
var v := PackedVector3Array()
var p := PackedVector3Array()
var rest := PackedFloat64Array()  # per segment
var inv_m := PackedFloat64Array()  # per particle, 0 at both ends
var weight := PackedFloat64Array()  # N, out of the water
var lift := PackedFloat64Array()  # N, buoyancy minus weight under water
var kn := PackedFloat64Array()
var kt := PackedFloat64Array()
var lambdas := PackedFloat64Array()
var blambdas := PackedFloat64Array()
var c_has := PackedByteArray()
var c_pt := PackedVector3Array()
var c_n := PackedVector3Array()
var q_at := PackedVector3Array()  # where each particle was last queried
var force := Vector3.ZERO
var tension := 0.0  # N at the vehicle
var spool_tension := 0.0  # N at the spool
var payout_speed := 0.0  # m/s, last tick
var solve_usec := 0
var query_usec := 0
var queries := 0  # last tick
var contacts := 0
var render_usec := 0
var rings_drawn := 0
var far_ticks := PackedInt32Array()  # per particle, ticks clear of solids
var s_has := PackedByteArray()  # per segment contact (capsule query)
var s_pt := PackedVector3Array()
var s_n := PackedVector3Array()
var sa_at := PackedVector3Array()  # segment ends when last queried
var sb_at := PackedVector3Array()
var seg_queries := 0
var seg_contacts := 0
var near_count := 0  # particles at or next to a contact
var splits := 0  # totals, for the bench
var merges := 0
var _cursor := 1
var _scursor := 0
var _rcursor := 1
var _rscursor := 0
var _tcursor := 1
var _capsule: CapsuleShape3D
var _squery: PhysicsShapeQueryParameters3D
var _ticks := 0
var _brake_lim := 0.0
var _query: PhysicsShapeQueryParameters3D
var _query_spool: PhysicsShapeQueryParameters3D
var _mesh: ArrayMesh
var _indices := PackedInt32Array()
var _bend_min := 0.0
var _bend_cap := 0.0
var _rs_kind := PackedInt32Array()  # the same, unpacked into typed arrays for the inner loop
var _rs_inv: Array[Transform3D] = []
var _rs_basis: Array[Basis] = []
var _rs_he := PackedVector3Array()
var _rs_hs := PackedFloat32Array()
var _rov_shapes := []  # [kind (0 box, 1 sphere, 2 capsule), shape transform in the vehicle body, half extents / radius, half segment]
var _rov_bound := 0.0  # m from the body origin that holds every shape
var _rov_split := []  # [segment, t] touching the hull this tick, for _adapt
var rov_force := Vector3.ZERO  # N on the vehicle from hull contacts, last tick
var rov_contacts := 0  # particles + segments touching the hull, last tick
var rov_usec := 0
var _rov_point := Vector3.ZERO
var claws: Array[RigidBody3D] = []
var claw_force: Array[Vector3] = []  # N on each claw from the cable, last tick
var claw_contacts := 0  # particles + segments touching a claw, last tick
var claw_clearance := INF  # m, from claw_penetration(): the cable's centreline nearest a claw's solid, < 0 inside
var claw_usec := 0
var _claw_point: Array[Vector3] = []
var _cs_claw := PackedInt32Array()  # per claw shape: its claw, then as _rs_* (spheres and capsules as their box)
var _cs_xf: Array[Transform3D] = []
var _cs_he := PackedVector3Array()
var _cs_inv: Array[Transform3D] = []  # world to shape, where the claws are this substep
var _cs_basis: Array[Basis] = []
var _claw_bound := 0.0
const TOPO_EVENT_COLS = ["tick", "kind", "index", "n_before", "n_after", "rest_a", "rest_b", "rest_c", "chord_off",
	"stretch_before", "stretch_after", "new_stretch_max", "bend_viol_before", "bend_viol_after", "dpos_max", "dv_max",
	"dsum_rest", "rest_err", "dke", "dmom", "render_d", "acc_before", "jerk_before", "acc_after", "jerk_after", "stale",
	"lifetime", "ke_before", "speed", "nb_stretch_max", "mass_jump", "acc_first"]
const TOPO_TICK_COLS = ["tick", "n", "ke", "acc_max", "acc_max_i", "jerk_max", "topo_dv_max", "rest_err", "events",
	"render_d_max", "stretch_max", "strain_max"]
var topo_events := []  # rows as TOPO_EVENT_COLS; acc/jerk_after fill in over the 10 ticks after the event
var topo_ticks := []  # rows as TOPO_TICK_COLS
var _lg_pid := PackedInt64Array()  # per particle: a stable id, the tick it was made, its acceleration last tick
var _lg_born := PackedInt32Array()
var _lg_a := PackedVector3Array()
var _lg_v := PackedVector3Array()  # velocities at the end of last tick, after its topology changes
var _lg_next_id := 0
var _lg_hist := []  # the last 10 ticks: [ids, |acceleration|, |jerk|]
var _lg_win := []  # events still watched: [row, ids, ticks left]
var _lg_pre := {}
var _lg_tick_events := PackedInt32Array()  # index of each change so far this tick
var _lg_row := []
var _lg_dv := 0.0
var _lg_render := 0.0
var jaw_local := Vector3.ZERO
var grip_joints := []  # the claws' joints that tell is_closed() (ljoint.gd, rjoint.gd)
var grip_i := -1  # the particle the jaws hold, -1: none
var gripping := false
var grip_force := Vector3.ZERO  # N the cable pulls on the jaws (moving average), applied to the vehicle there
var grip_err := 0.0  # m from the held particle to the jaw point where the vehicle got to, measured the next tick
var grip_snap := 0.0  # m the particle was moved onto the jaw point when taken
var grips := 0  # totals, for the bench
var slips := 0
var releases := 0
var grip_usec := 0
var _grip_over := 0  # ticks the pull has been over grip_slip_force
var _closed_ticks := 0  # since the jaws closed; -1 after a slip, until they open


func _ready():
	if Engine.is_editor_hint():
		return
	rov = get_tree().get_first_node_in_group("vehicles")
	if rov == null:
		push_warning("spool_tether: no vehicle")
		set_physics_process(false)
		set_process(false)
		return
	attach_local = rov.to_local(global_position)
	_read_rov_shapes()
	_find_claws()
	var jp := rov.get_node_or_null("JawPoint") as Node3D
	jaw_local = rov.to_local(jp.global_position) if jp else jaw_point
	_find_anchor()
	_brake_lim = brake_tension
	_bend_min = cos(deg_to_rad(clamp(max_bend_deg, 0.0, 180.0)) / 2)
	_bend_cap = deg_to_rad(clamp(max_bend_deg, 0.0, 180.0))
	var fine = min(fine_segments, int(max_length / spacing))
	far_spacing = max(spacing, (max_length - fine * spacing) / max(1, max_particles - 1 - fine))
	var a = attach_point()
	total = initial_length if initial_length > 0 else a.distance_to(anchor) + initial_slack
	total = clamp(total, a.distance_to(anchor) * 1.001, max_length)
	# lay it out straight from the vehicle to the spool, the extra length is slack the constraints leave alone
	var segs: Array[float] = []
	if fixed_count:
		for i in range(max_particles - 1):
			segs.append(total / (max_particles - 1))
	else:
		var left = total
		while left > _spacing_for(segs.size()) + 0.25:
			segs.append(_spacing_for(segs.size()))
			left -= segs[-1]
		segs.append(left)  # the spool segment
	var n = segs.size()
	x.resize(n + 1)
	var along = 0.0
	for i in range(n + 1):
		x[i] = a.lerp(anchor, along / total)
		if i < n:
			along += segs[i]
	v.resize(n + 1)
	v.fill(Vector3.ZERO)
	p = x.duplicate()
	rest.resize(n)
	for i in range(n):
		rest[i] = segs[i]
	lambdas.resize(n)
	blambdas.resize(max(n - 1, 0))
	c_has.resize(n + 1)
	c_has.fill(0)
	c_pt.resize(n + 1)
	c_n.resize(n + 1)
	q_at.resize(n + 1)
	q_at.fill(Vector3.INF)
	far_ticks.resize(n + 1)
	far_ticks.fill(0)
	s_has.resize(n)
	s_has.fill(0)
	s_pt.resize(n)
	s_n.resize(n)
	sa_at.resize(n)
	sa_at.fill(Vector3.INF)
	sb_at.resize(n)
	sb_at.fill(Vector3.INF)
	_update_masses()
	_query = PhysicsShapeQueryParameters3D.new()
	var sphere = SphereShape3D.new()
	sphere.radius = contact_radius + query_margin
	_query.shape = sphere
	_query.collision_mask = collision_mask
	_query.exclude = _own_rids()
	_query_spool = PhysicsShapeQueryParameters3D.new()
	_query_spool.shape = sphere
	_query_spool.collision_mask = collision_mask
	_query_spool.exclude = _own_rids() + _boat_rids()
	_capsule = CapsuleShape3D.new()
	_squery = PhysicsShapeQueryParameters3D.new()
	_squery.shape = _capsule
	_squery.collision_mask = collision_mask
	_squery.exclude = _own_rids() + _boat_rids()
	_build_mesh()


func _spacing_for(segment: int) -> float:
	return spacing if segment < fine_segments else far_spacing


func _find_anchor():
	if not anchor_path.is_empty():
		anchor_node = get_node_or_null(anchor_path)
	if anchor_node == null:
		anchor_node = _boat_anchor()
	if anchor_node == null:
		# placeholder spool floating on the surface next to where the vehicle starts
		anchor_node = Node3D.new()
		anchor_node.name = "TetherSpoolPlaceholder"
		anchor_node.top_level = true
		add_child(anchor_node)
		var back = -rov.global_basis.z
		back.y = 0
		var at = rov.global_position + back.normalized() * placeholder_distance
		at.y = Globals.surface_altitude + 0.05
		anchor_node.global_position = at
		var drum = MeshInstance3D.new()
		var cyl = CylinderMesh.new()
		cyl.top_radius = 0.25
		cyl.bottom_radius = 0.25
		cyl.height = 0.4
		drum.mesh = cyl
		var mat = StandardMaterial3D.new()
		mat.albedo_color = Color(0.9, 0.3, 0.1)
		drum.material_override = mat
		drum.rotation_degrees = Vector3(90, 0, 0)
		drum.position = Vector3(0, 0.3, 0)
		anchor_node.add_child(drum)
	anchor = anchor_node.global_position


func _boat_anchor() -> Node3D:
	# the boat's TetherAnchor marker (vehicles/boat/boat.tscn), where the cable leaves the spool
	var found := get_tree().get_first_node_in_group("tether_anchors") as Node3D
	if found == null:
		found = get_tree().root.find_child("TetherAnchor", true, false) as Node3D
	return found


func _own_rids() -> Array:
	# the vehicle and, when they push the cable themselves, its claws: the queries leave them out
	return [rov.get_rid()] + (claws.map(func(c): return c.get_rid()) if claw_collide else [])


func _boat_rids() -> Array:
	# the cable leaves the spool through the hull, so the particles next to it don't collide with the boat
	var out = []
	var n = anchor_node
	while n != null and n != get_tree().root:
		if n is CollisionObject3D:
			out.append(n.get_rid())
			break
		n = n.get_parent()
	return out


func _update_masses():
	var n = rest.size()
	var area = PI * diameter * diameter / 4
	inv_m.resize(n + 1)
	weight.resize(n + 1)
	lift.resize(n + 1)
	kn.resize(n + 1)
	kt.resize(n + 1)
	for i in range(n + 1):
		_mass_at(i)


func _mass_at(i: int):
	# a particle carries half of each segment next to it: mass, buoyancy and drag follow the tether's length
	var n := rest.size()
	if i < 0 or i > n:
		return
	var area := PI * diameter * diameter / 4
	var l: float = 0.5 * ((rest[i - 1] if i > 0 else 0.0) + (rest[i] if i < n else 0.0))
	var displaced := WATER_DENSITY * area * l
	var cable := displaced / buoyancy_ratio
	# (plus the water it carries sideways, added mass); the ends and the particle in the jaws are moved, not solved
	inv_m[i] = 0.0 if i == 0 or i == n or i == grip_i else 1.0 / (cable + displaced)
	weight[i] = cable * G
	lift[i] = (displaced - cable) * G
	kn[i] = 0.5 * WATER_DENSITY * normal_cd * diameter * l
	kt[i] = 0.5 * WATER_DENSITY * axial_cf * PI * diameter * l


func attach_point() -> Vector3:
	return rov.global_transform * attach_local


func _physics_process(delta):
	if rov == null:
		return
	var t0 = Time.get_ticks_usec()
	_ticks += 1
	if _ticks <= 240 and _ticks % 30 == 0 and anchor_node.name == "TetherSpoolPlaceholder":
		# a boat added after the tether: move the spool end onto its TetherAnchor
		var found = _boat_anchor()
		if found:
			anchor_node.queue_free()
			anchor_node = found
			_query_spool.exclude = _own_rids() + _boat_rids()
			_squery.exclude = _query_spool.exclude
	if is_instance_valid(anchor_node):
		anchor = anchor_node.global_position
	if collide:
		_query_contacts()
		if segment_collide:
			_query_segments()
	if grip or gripping:
		_grip_tick()
	var t1 = Time.get_ticks_usec()
	_solve(delta)
	if topology_log:
		_lg_tick(delta)
	_payout_topology()
	if not fixed_count and (adaptive or cull):
		_adapt()
	if topology_log:
		_lg_end_tick()
	solve_usec = Time.get_ticks_usec() - t0
	query_usec = t1 - t0
	rov.apply_force(force, attach_point() - rov.global_position)
	if rov_force != Vector3.ZERO:
		rov.apply_force(rov_force, _rov_point - rov.global_position)
	if gripping:
		rov.apply_force(grip_force, rov.global_transform * jaw_local - rov.global_position)
	for c in range(claw_force.size()):
		if claw_force[c] != Vector3.ZERO:
			claws[c].apply_force(claw_force[c], _claw_point[c] - claws[c].global_position)


func _grip_tick():
	# the jaws let go when they open (or the cable slips out), and take the cable when they close round it
	var t0 := Time.get_ticks_usec()
	var closed := grip and not grip_joints.is_empty()
	for j in grip_joints:
		closed = closed and j.is_closed()
	var jw: Vector3 = rov.global_transform * jaw_local
	if gripping:
		grip_err = x[grip_i].distance_to(jw)
		if not closed:
			releases += 1
			_grip_release()
		elif _grip_over >= grip_slip_ticks:
			slips += 1
			_grip_release()
			_closed_ticks = -1
	if not closed:
		_closed_ticks = 0
	elif _closed_ticks >= 0:
		_closed_ticks += 1
		if not gripping and _closed_ticks <= grip_window * Engine.physics_ticks_per_second:
			_grip_engage(jw)
	if gripping:
		for k in range(grip_i - 1, grip_i + 2):
			far_ticks[k] = 0  # near a solid: no merging there, and bends next to it are split fine
	grip_usec = Time.get_ticks_usec() - t0


func _grip_engage(jw: Vector3):
	# the cable's point nearest the jaw point, off the attachment's and the spool's segments: an end particle if one is
	# there, else a new one; it becomes kinematic, moved with the jaws (never particle 1 or the spool's last)
	var n := rest.size()
	var best := grip_radius
	var bj := -1
	var bt := 0.0
	for j in range(1, n - 1):
		var d: Vector3 = x[j + 1] - x[j]
		var t: float = clamp((jw - x[j]).dot(d) / maxf(d.length_squared(), 1e-12), 0.0, 1.0)
		var dist: float = (x[j] + d * t).distance_to(jw)
		if dist < best:
			best = dist
			bj = j
			bt = t
	if bj < 0:
		return
	var at: Vector3 = x[bj].lerp(x[bj + 1], bt)
	var r: float = rest[bj]
	var gi := -1
	if at.distance_to(x[bj]) < 0.03 and bj >= 2:
		gi = bj
	elif at.distance_to(x[bj + 1]) < 0.03 and bj + 1 <= n - 2:
		gi = bj + 1
	elif r >= 2.0 * grip_min_piece:
		var t: float = clamp(bt, grip_min_piece / r, 1.0 - grip_min_piece / r)
		_insert(bj + 1, x[bj].lerp(x[bj + 1], t), v[bj].lerp(v[bj + 1], t), r * t, r * (1.0 - t))
		splits += 1
		gi = bj + 1
	elif bt < 0.5 and bj >= 2:
		gi = bj
	elif bj + 1 <= n - 2:
		gi = bj + 1
	if gi < 0:
		return
	grip_i = gi
	gripping = true
	grip_snap = x[gi].distance_to(jw)
	x[gi] = jw
	p[gi] = jw
	c_has[gi] = 0
	_mass_at(gi)
	# its segments are left to their other ends by the claws (which can't move it): keep them short, so the cable
	# leaves the jaws from particles they do push
	for k in [gi, gi - 1]:
		var rk: float = rest[k]
		if rk >= 2.0 * grip_min_piece:
			var f: float = grip_min_piece / rk if k == grip_i else 1.0 - grip_min_piece / rk
			_insert(k + 1, x[k].lerp(x[k + 1], f), v[k].lerp(v[k + 1], f), rk * f, rk * (1.0 - f))
			splits += 1
	grip_err = 0.0
	grip_force = Vector3.ZERO
	_grip_over = 0
	grips += 1


func _grip_release():
	var gi := grip_i
	grip_i = -1
	gripping = false
	grip_force = Vector3.ZERO
	_grip_over = 0
	_mass_at(gi)
	for k in range(gi - 1, gi + 2):
		far_ticks[k] = 0


func _query_contacts():
	var space = get_world_3d().direct_space_state
	var n = x.size() - 1
	queries = 0
	var budget = queries_per_tick
	var reach = query_margin * 0.5
	if _cursor > n - 1 or _cursor < 1:
		_cursor = 1
	if _rcursor > n - 1 or _rcursor < 1:
		_rcursor = 1
	for k in range(n - 1):
		if budget <= 0:
			break
		var i = _cursor
		_cursor = _cursor + 1 if _cursor < n - 1 else 1
		var xi = x[i]
		if xi.distance_squared_to(q_at[i]) < reach * reach:
			continue
		budget -= 1
		_query_particle(space, i)
	# touching particles are asked again in turn even if still (their own small budget): a plane found on one
	# face of a corner would otherwise keep the cable from sliding round it
	var refresh := contact_refresh_per_tick
	for k in range(n - 1):
		if refresh <= 0 or contacts == 0:
			break
		var i = _rcursor
		_rcursor = _rcursor + 1 if _rcursor < n - 1 else 1
		if c_has[i] and q_at[i] != x[i]:
			refresh -= 1
			_query_particle(space, i)
	contacts = 0
	near_count = 0
	for i in range(1, n):
		# a contact plane is only good near where it was found
		if c_has[i] and x[i].distance_squared_to(q_at[i]) > query_margin * query_margin:
			c_has[i] = 0
			q_at[i] = Vector3.INF
		contacts += c_has[i]
		if c_has[i] or s_has[i - 1] or s_has[i]:
			far_ticks[i] = 0
			near_count += 1
		else:
			far_ticks[i] += 1


func _query_particle(space: PhysicsDirectSpaceState3D, i: int):
	var xi: Vector3 = x[i]
	queries += 1
	var q = _query_spool if xi.distance_squared_to(anchor) < 4.0 else _query
	q.transform = Transform3D(Basis.IDENTITY, xi)
	var info = space.get_rest_info(q)
	q_at[i] = xi
	if info.is_empty():
		c_has[i] = 0
	elif c_has[i] and _feature_normal(xi, info).dot(c_n[i]) < 0.0:
		# the answer flipped side: it went into something thin (the surface's far side is nearer now). Keep
		# the side it came from so it's pushed back out, not on through
		pass
	else:
		c_has[i] = 1
		c_pt[i] = info.point
		c_n[i] = _feature_normal(xi, info)


func _feature_normal(at: Vector3, info: Dictionary) -> Vector3:
	# the query shapes are inflated by query_margin, so the engine's normal is that of the face it is least deep
	# into, which near a corner or edge can be the wrong face (and holds the cable on the corner). The direction
	# from the touching point to the cable is the edge's or corner's own normal; the face normal if it's inside
	var d: Vector3 = at - info.point
	var dl := d.length()
	if dl > 1e-3 and d.dot(info.normal) > 0.0:
		return d / dl
	return info.normal


func _query_segments():
	# a capsule round each segment that moved: catches edges and corners between particles, and tells _adapt
	# where the tether is near solids. Contacts are kept as a plane at the touching point.
	var space = get_world_3d().direct_space_state
	var n = rest.size()
	seg_queries = 0
	var budget = segment_queries_per_tick
	var reach2 = query_margin * query_margin * 0.25
	_capsule.radius = contact_radius + query_margin
	if _scursor > n - 2 or _scursor < 0:
		_scursor = 0
	if _rscursor > n - 2 or _rscursor < 0:
		_rscursor = 0
	for k in range(n - 1):  # not the spool segment, which leaves through the hull
		if budget <= 0:
			break
		var j = _scursor
		_scursor = _scursor + 1 if _scursor < n - 2 else 0
		if x[j].distance_squared_to(sa_at[j]) < reach2 and x[j + 1].distance_squared_to(sb_at[j]) < reach2:
			continue
		budget -= 1
		_query_segment(space, j)
	# touching segments asked again in turn even if still, as for particles
	var refresh := contact_refresh_per_tick
	for k in range(n - 1):
		if refresh <= 0 or seg_contacts == 0:
			break
		var j = _rscursor
		_rscursor = _rscursor + 1 if _rscursor < n - 2 else 0
		if s_has[j] and (sa_at[j] != x[j] or sb_at[j] != x[j + 1]):
			refresh -= 1
			_query_segment(space, j)
	seg_contacts = 0
	for j in range(n):
		if s_has[j] and (x[j].distance_squared_to(sa_at[j]) > query_margin * query_margin or x[j + 1].distance_squared_to(sb_at[j]) > query_margin * query_margin):
			s_has[j] = 0
			sa_at[j] = Vector3.INF
		seg_contacts += s_has[j]


func _query_segment(space: PhysicsDirectSpaceState3D, j: int):
	var a: Vector3 = x[j]
	var b: Vector3 = x[j + 1]
	seg_queries += 1
	var d := b - a
	var l := d.length()
	sa_at[j] = a
	sb_at[j] = b
	if l < 1e-4:
		s_has[j] = 0
		return
	var y := d / l
	var ref := Vector3.RIGHT if abs(y.x) < 0.9 else Vector3.FORWARD
	var xx := y.cross(ref).normalized()
	_capsule.height = l + 2.0 * _capsule.radius
	_squery.transform = Transform3D(Basis(xx, y, xx.cross(y)), (a + b) * 0.5)
	var info = space.get_rest_info(_squery)
	var had: bool = s_has[j] == 1
	s_has[j] = 0
	if info.is_empty():
		return
	# the fat look-ahead capsule reports one point of whatever it overlaps, which next to a corner can be on a face
	# well away from the segment: ask again with a thin one, and keep that if the segment is actually that close
	var fat := _capsule.radius
	_capsule.radius = contact_radius * 1.5
	_capsule.height = l + 2.0 * _capsule.radius
	var near_info = space.get_rest_info(_squery)
	_capsule.radius = fat
	if not near_info.is_empty():
		info = near_info
	# the push is kept across the cable: a normal along it (an end cap touching) would act as a stopper
	var t: float = (info.point - a).dot(y) / l
	var nn: Vector3 = _feature_normal(a + d * clamp(t, 0.0, 1.0), info)
	nn -= y * nn.dot(y)
	if had and nn.dot(s_n[j]) < 0.0:
		s_has[j] = 1  # flipped side (deep in something thin): keep the side it came from
	elif t > seg_end_t and t < 1.0 - seg_end_t and nn.length_squared() > 0.01:
		s_has[j] = 1
		s_pt[j] = info.point
		s_n[j] = nn.normalized()


func _read_rov_shapes():
	_rov_shapes = _read_shapes(rov)
	_rs_kind.clear()
	_rs_inv.clear()
	_rs_basis.clear()
	_rs_he.clear()
	_rs_hs.clear()
	_rov_bound = 0.0
	for entry in _rov_shapes:
		_rs_kind.append(entry[0])
		_rs_inv.append(entry[4])
		_rs_basis.append(entry[1].basis)
		_rs_he.append(entry[2])
		_rs_hs.append(entry[3])
		_rov_bound = max(_rov_bound, entry[1].origin.length() + entry[2].length() + entry[3])


func _find_claws():
	# the gripper's claws: rigid bodies that a joint next to the vehicle ties to it (lClaw, rClaw on the heavy)
	var parent := rov.get_parent()
	for j in (parent.get_children() if parent else []):
		if not (j is Joint3D):
			continue
		var a = j.get_node_or_null(j.node_a)
		var b = j.get_node_or_null(j.node_b)
		var claw = b if a == rov else (a if b == rov else null)
		if not (claw is RigidBody3D) or claws.has(claw):
			continue
		if j.has_method("is_closed"):
			grip_joints.append(j)
		for entry in _read_shapes(claw):
			_cs_claw.append(claws.size())
			_cs_xf.append(entry[1])
			_cs_he.append(entry[2] if entry[0] == 0 else Vector3(entry[2].x, entry[2].x + entry[3], entry[2].x))
			_cs_inv.append(entry[4])
			_cs_basis.append(entry[1].basis)
			_claw_bound = max(_claw_bound, entry[1].origin.length() + entry[2].length() + entry[3])
		claws.append(claw)
		claw_force.append(Vector3.ZERO)
		_claw_point.append(Vector3.ZERO)


func _read_shapes(body: Node) -> Array:
	# a body's CollisionShape3D children as [kind (0 box, 1 sphere, 2 capsule), shape transform in the body, half
	# extents / radius, half segment, the transform's inverse]; cylinders and convex hulls as their (local) box
	var out := []
	for c in body.get_children():
		if not (c is CollisionShape3D) or c.disabled or c.shape == null:
			continue
		var sh: Shape3D = c.shape
		var xf: Transform3D = c.transform
		var entry = null
		if sh is BoxShape3D:
			entry = [0, xf, sh.size * 0.5, 0.0]
		elif sh is SphereShape3D:
			entry = [1, xf, Vector3.ONE * sh.radius, 0.0]
		elif sh is CapsuleShape3D:
			entry = [2, xf, Vector3.ONE * sh.radius, max(0.0, sh.height * 0.5 - sh.radius)]
		elif sh is CylinderShape3D:
			entry = [0, xf, Vector3(sh.radius, sh.height * 0.5, sh.radius), 0.0]
		elif sh is ConvexPolygonShape3D and sh.points.size() > 0:
			var box := AABB(sh.points[0], Vector3.ZERO)
			for q in sh.points:
				box = box.expand(q)
			entry = [0, xf * Transform3D(Basis.IDENTITY, box.get_center()), box.size * 0.5, 0.0]
		if entry == null:
			continue
		# a scaled shape transform is folded into the extents
		var sc: Vector3 = entry[1].basis.get_scale()
		entry[1] = Transform3D(entry[1].basis.orthonormalized(), entry[1].origin)
		entry[2] = entry[2] * sc
		entry[3] = entry[3] * sc.y
		entry.append(entry[1].affine_inverse())
		out.append(entry)
	return out


var _rc_d := 0.0  # _rov_closest's answer: signed distance and outward normal (body frame)
var _rc_n := Vector3.UP


func _rov_closest(q: Vector3) -> void:
	# nearest point of the vehicle's shapes to q (both in the body's frame), into _rc_d and _rc_n
	var best_d := INF
	var best_n := Vector3.UP
	for si in range(_rs_kind.size()):
		var l: Vector3 = _rs_inv[si] * q
		var he: Vector3 = _rs_he[si]
		var d: float
		var nl: Vector3
		if _rs_kind[si] == 0:
			var c := l.clamp(-he, he)
			if c == l:
				# inside: out through the nearest face
				var dx: float = he.x - absf(l.x)
				var dy: float = he.y - absf(l.y)
				var dz: float = he.z - absf(l.z)
				if dx <= dy and dx <= dz:
					d = -dx
					nl = Vector3(1.0 if l.x >= 0.0 else -1.0, 0, 0)
				elif dy <= dz:
					d = -dy
					nl = Vector3(0, 1.0 if l.y >= 0.0 else -1.0, 0)
				else:
					d = -dz
					nl = Vector3(0, 0, 1.0 if l.z >= 0.0 else -1.0)
			else:
				var off := l - c
				d = off.length()
				nl = off / d
		else:
			var hs: float = _rs_hs[si]
			var off := l - Vector3(0, clampf(l.y, -hs, hs), 0)
			var ol := off.length()
			d = ol - he.x
			nl = off / ol if ol > 1e-9 else Vector3.UP
		if d < best_d:
			best_d = d
			best_n = _rs_basis[si] * nl
	_rc_d = best_d
	_rc_n = best_n


func _claw_pose(xfs: Array[Transform3D]):
	# the claw shapes' world frames, for claws at xfs
	for si in range(_cs_claw.size()):
		var w: Transform3D = xfs[_cs_claw[si]] * _cs_xf[si]
		_cs_inv[si] = w.affine_inverse()
		_cs_basis[si] = w.basis


func _claw_seg(si: int, pa: Vector3, d: Vector3, oa: Vector3, od: Vector3) -> float:
	# segment pa + d t against claw box si: its nearest point (alternate projections, the box is convex) into _rc_*,
	# world frame. Through the box it goes out across itself (along itself it would stay in), the shortest way back to
	# the side where it was, oa + od t: the plates are thinner than a tick's slide, and the near side can be the far one
	var he: Vector3 = _cs_he[si]
	var a: Vector3 = _cs_inv[si] * pa
	var dl: Vector3 = _cs_inv[si].basis * d
	var dd := dl.length_squared()
	var t := clampf(-a.dot(dl) / dd, 0.0, 1.0) if dd > 1e-10 else 0.0
	var q := a + dl * t
	for kk in range(3):
		if q.clamp(-he, he) == q or dd <= 1e-10:
			break
		t = clampf((q.clamp(-he, he) - a).dot(dl) / dd, 0.0, 1.0)
		q = a + dl * t
	var c := q.clamp(-he, he)
	if c != q:
		_rc_d = (q - c).length()
		_rc_n = _cs_basis[si] * ((q - c) / _rc_d)
		return t
	_rc_d = INF
	var ol: Vector3 = _cs_inv[si] * (oa + od * t)
	for k in range(3):
		var ax: Vector3 = dl.cross(Basis.IDENTITY[k]) if dd > 1e-10 else Basis.IDENTITY[k]
		if ax.length_squared() < 1e-6 * maxf(dd, 1.0):
			continue
		ax = ax.normalized()
		if ax.dot(ol) < 0.0:
			ax = -ax
		var o := absf(ax.x) * he.x + absf(ax.y) * he.y + absf(ax.z) * he.z - ax.dot(q)
		if o < _rc_d:
			_rc_d = o
			_rc_n = _cs_basis[si] * ax
	_rc_d = -_rc_d
	return t


func claw_penetration() -> float:
	# how deep the cable's centreline is inside a claw (m, 0 if clear); claw_clearance gets the signed distance
	claw_clearance = INF
	var xfs: Array[Transform3D] = []
	var mid := Vector3.ZERO
	for c in claws:
		xfs.append(c.global_transform)
		mid += c.global_position / claws.size()
	_claw_pose(xfs)
	for j in range(1, x.size() - 2):  # not the attachment segment, nor the spool's
		var d: Vector3 = x[j + 1] - x[j]
		var t: float = clamp((mid - x[j]).dot(d) / maxf(d.length_squared(), 1e-10), 0.0, 1.0)
		if (x[j] + d * t).distance_to(mid) < _claw_bound + 0.3:
			for si in range(_cs_claw.size()):
				_claw_seg(si, x[j], d, x[j], d)
				claw_clearance = min(claw_clearance, _rc_d)
	return max(0.0, -claw_clearance)


func rov_penetration() -> float:
	# how deep the cable's centreline is inside the hull (m), away from the attachment: for the bench
	if _rov_shapes.is_empty() or rov == null:
		return 0.0
	var ti := rov.global_transform.affine_inverse()
	var along := 0.0
	var deepest := 0.0
	for i in range(1, x.size() - 1):
		along += rest[i - 1]
		if along < rov_skip_length:
			continue
		if x[i].distance_to(rov.global_position) > _rov_bound + rest[i] + 0.5:
			continue
		# the segment's deepest point: alternate projections from its nearest approach to the body origin
		var pa: Vector3 = ti * x[i]
		var d: Vector3 = ti * x[i + 1] - pa
		var dd := d.length_squared()
		var t: float = clamp(-pa.dot(d) / dd, 0.0, 1.0) if dd > 1e-10 else 0.0
		for kk in range(4):
			_rov_closest(pa + d * t)
			deepest = max(deepest, -_rc_d)
			if dd <= 1e-10:
				break
			t = clamp((pa + d * t - _rc_n * _rc_d - pa).dot(d) / dd, 0.0, 1.0)
	return deepest


func _solve(dt):
	var surface: float = Globals.surface_altitude
	var h: float = dt / substeps
	var h2 := h * h
	var n := rest.size()
	var a = attach_point()
	var offset = a - rov.global_position
	var a_vel = rov.linear_velocity + rov.angular_velocity.cross(offset)
	var paying := total < max_length - 1e-6
	var bend := max_bend_deg < 180.0 and n >= 2
	var bmin := PackedFloat32Array()
	if bend:
		# shortest allowed distance between particles two apart: the chord of a turn of mean length / min_bend_radius
		bmin.resize(n - 1)
		var inv_r: float = 1.0 / maxf(min_bend_radius, 1e-4)
		for j in range(n - 1):
			var ra: float = rest[j]
			var rb: float = rest[j + 1]
			var th: float = min(_bend_cap, 0.5 * (ra + rb) * inv_r)
			bmin[j] = sqrt(max(ra * ra + rb * rb + 2.0 * ra * rb * cos(th), 0.0))
	# the particle in the jaws is moved with them: no contact moves it, and its segments are left to their other ends
	var gi := grip_i
	var contact_list := PackedInt32Array()
	if contacts > 0:
		for i in range(1, n):
			if c_has[i] and i != gi:
				contact_list.append(i)
	var seg_list := PackedInt32Array()
	if seg_contacts > 0 and segment_collide:
		for j in range(n - 1):
			if s_has[j]:
				seg_list.append(j)
	# the vehicle's hull: particles and segments near it, past rov_skip_length of cable from the attachment
	var rov_p := PackedInt32Array()
	var rov_s := PackedInt32Array()
	var rov_t0 := Time.get_ticks_usec()
	var rov_xf := rov.global_transform
	var rov_v := rov.linear_velocity
	var rov_w := rov.angular_velocity
	var rov_imp := Vector3.ZERO
	var rov_pt_sum := Vector3.ZERO
	var rov_pt_w := 0.0
	_rov_split.clear()
	if rov_collide and not _rov_shapes.is_empty():
		var reach: float = _rov_bound + contact_radius + rov_v.length() * dt + 0.2
		var along := 0.0
		for i in range(1, n):
			var a0 := along
			along += rest[i - 1]
			if along >= rov_skip_length and x[i].distance_to(rov_xf.origin) < reach and i != gi:
				rov_p.append(i)
			# a segment whose near end is past the skip length and that passes within reach
			if a0 >= rov_skip_length and i - 1 < n - 1 and i != gi and i - 1 != gi:
				var sa: Vector3 = x[i - 1]
				var sd: Vector3 = x[i] - sa
				var tt: float = clamp((rov_xf.origin - sa).dot(sd) / max(sd.length_squared(), 1e-10), 0.0, 1.0)
				if (sa + sd * tt).distance_to(rov_xf.origin) < reach:
					rov_s.append(i - 1)
	var rov_cnt := 0
	var rov_us := Time.get_ticks_usec() - rov_t0
	var rov_pn := PackedFloat32Array()
	rov_pn.resize(rov_p.size())
	var rov_nrm := PackedVector3Array()
	rov_nrm.resize(rov_p.size())
	var rov_st := PackedFloat32Array()
	rov_st.resize(rov_s.size())
	# the claws: segments passing near them (an end touching is that particle), all but the attachment's and the spool's
	var cl_s := PackedInt32Array()
	var cl_xf: Array[Transform3D] = []
	var cl_com := PackedVector3Array()
	var cl_v := PackedVector3Array()
	var cl_w := PackedVector3Array()
	var mid := Vector3.ZERO
	var cl_t0 := Time.get_ticks_usec()
	if claw_collide and not _cs_claw.is_empty():
		var reach: float = _claw_bound + claw_contact_radius + 0.15
		for c in claws:
			var st := PhysicsServer3D.body_get_direct_state(c.get_rid())
			cl_xf.append(c.global_transform)
			cl_com.append(c.global_position + (st.center_of_mass if st else Vector3.ZERO))
			cl_v.append(c.linear_velocity)
			cl_w.append(c.angular_velocity)
			mid += c.global_position / claws.size()
			reach += c.linear_velocity.length() * dt
		for j in range(1, n - 1):
			var sd: Vector3 = x[j + 1] - x[j]
			var tt: float = clamp((mid - x[j]).dot(sd) / max(sd.length_squared(), 1e-10), 0.0, 1.0)
			if (x[j] + sd * tt).distance_to(mid) < reach and j != gi and j + 1 != gi:
				cl_s.append(j)
	var cl_pn := PackedFloat32Array()
	cl_pn.resize(cl_s.size() * _cs_claw.size())
	var cl_imp := PackedVector3Array()
	cl_imp.resize(cl_xf.size())
	var cl_pt := PackedVector3Array()
	cl_pt.resize(cl_xf.size())
	var cl_ptw := PackedFloat64Array()
	cl_ptw.resize(cl_xf.size())
	var cl_cnt := 0
	var cl_us := Time.get_ticks_usec() - cl_t0
	var tension_sum := 0.0
	var spool_sum := 0.0
	var paid := 0.0
	var jw: Vector3 = rov_xf * jaw_local
	var jw_vel: Vector3 = rov_v + rov_w.cross(jw - rov_xf.origin)
	var grip_sum := Vector3.ZERO
	x[n] = anchor
	for s in range(substeps):
		x[0] = a + a_vel * h * s
		v[0] = a_vel
		p[0] = a + a_vel * h * (s + 1)
		p[n] = anchor
		for i in range(1, n):
			var vi: Vector3 = v[i]
			var xi: Vector3 = x[i]
			var im: float = inv_m[i]
			var submerged: float = clamp((surface - xi.y) / diameter + 0.5, 0.0, 1.0)
			vi.y += h * (lift[i] * submerged - weight[i] * (1.0 - submerged)) * im
			if submerged > 0:
				var tangent: Vector3 = x[i + 1] - x[i - 1]
				var tl := tangent.length()
				if tl > 1e-6:
					tangent /= tl
					var vt: Vector3 = tangent * vi.dot(tangent)
					var vn: Vector3 = vi - vt
					# implicit quadratic drag can slow the cable but never reverse it
					vn /= 1.0 + h * kn[i] * submerged * vn.length() * im
					vt /= 1.0 + h * kt[i] * submerged * vt.length() * im
					vi = vn + vt
			v[i] = vi
			p[i] = xi + vi * h
		if gi >= 0:
			# where the jaws will be at the substep's end, as for the attachment
			x[gi] = jw + jw_vel * h * s
			p[gi] = jw + jw_vel * h * (s + 1)
		lambdas.fill(0.0)
		if bend:
			blambdas.fill(0.0)
		var rxf := Transform3D(rov_xf.basis, rov_xf.origin + rov_v * h * (s + 1))
		var rxi := rxf.affine_inverse()
		rov_pn.fill(0.0)
		# how far each contact pushed out over all the sweeps: that's its normal force, and Coulomb friction's limit
		var c_dn := PackedFloat32Array()
		c_dn.resize(contact_list.size())
		var s_dn := PackedFloat32Array()
		s_dn.resize(seg_list.size())
		var tau := h * (s + 1)
		# claw contacts at the sweeps' start, held by them as planes: [k, segment, normal, point on the claw]
		var cl_c := []
		if cl_s.size():
			var tc0 := Time.get_ticks_usec()
			# the claws where they will be at the end of this substep: turned about their centre of mass, and moved
			var adv: Array[Transform3D] = []
			for c in range(cl_xf.size()):
				var wl := cl_w[c].length()
				var r := Basis(cl_w[c] / wl, wl * tau) if wl > 1e-9 else Basis.IDENTITY
				var o: Vector3 = cl_com[c] + cl_v[c] * tau + r * (cl_xf[c].origin - cl_com[c])
				adv.append(Transform3D(r * cl_xf[c].basis, o))
			_claw_pose(adv)
			cl_pn.fill(0.0)
			for sk in range(cl_s.size()):
				for si in range(_cs_claw.size()):
					var j: int = cl_s[sk]
					var t := _claw_seg(si, p[j], p[j + 1] - p[j], x[j], x[j + 1] - x[j])
					if _rc_d < claw_contact_radius:
						cl_c.append([sk * _cs_claw.size() + si, j, _rc_n, p[j].lerp(p[j + 1], t) - _rc_n * _rc_d])
			cl_us += Time.get_ticks_usec() - tc0
		for it in range(iterations):
			var forward := it % 2 == 0
			for k in range(n):
				var j := k if forward else n - 1 - k
				var w1: float = inv_m[j]
				var w2: float = inv_m[j + 1]
				var d: Vector3 = p[j + 1] - p[j]
				var dl := d.length()
				var c: float = dl - rest[j]
				# a rope only pulls: slack segments are left alone, so it can fold without fighting itself
				if c <= 0.0 or dl < 1e-9:
					continue
				var alpha: float = compliance_per_m * rest[j] / h2
				var lam: float = lambdas[j]
				var dlambda := (-c - alpha * lam) / (w1 + w2 + alpha)
				if j == n - 1 and paying and not fixed_count:
					# the spool's brake: the cable leaving it can't pull harder than that, the rest pays out
					dlambda = max(lam + dlambda, -_brake_lim * h2) - lam
				lambdas[j] = lam + dlambda
				var corr := d * (dlambda / dl)
				p[j] -= corr * w1
				p[j + 1] += corr * w2
			if bend and (it % 2 == 1 or iterations == 1):
				# no tighter than min_bend_radius: particles two apart are pushed apart, never pulled together, with
				# at most bend_max_force (every other sweep, which is enough for a soft limit)
				var bend_lim := bend_max_force * h2
				for j in range(n - 1):
					var w1: float = inv_m[j]
					var w2: float = inv_m[j + 2]
					if w1 + w2 == 0.0:
						continue
					var d: Vector3 = p[j + 2] - p[j]
					var dl := d.length()
					var c: float = dl - bmin[j]
					if c >= 0.0 or dl < 1e-9:
						continue
					var alpha: float = bend_compliance / h2
					var lam: float = blambdas[j]
					var dlambda: float = min((-c - alpha * lam) / (w1 + w2 + alpha), bend_lim - lam)
					blambdas[j] = lam + dlambda
					var corr := d * (dlambda / dl)
					p[j] -= corr * w1
					p[j + 2] += corr * w2
			if contact_list.size():
				for ci in range(contact_list.size()):
					var i: int = contact_list[ci]
					var nn: Vector3 = c_n[i]
					var pen: float = (p[i] - c_pt[i]).dot(nn) - contact_radius
					if pen < 0.0:
						p[i] -= nn * pen
						c_dn[ci] -= pen
			if seg_list.size():
				# the point of the segment nearest the contact stays on the outside of its plane; the push is
				# shared by the two ends by where along the segment it is (like a particle on an edge)
				for si in range(seg_list.size()):
					var j: int = seg_list[si]
					var nn: Vector3 = s_n[j]
					var pa: Vector3 = p[j]
					var d: Vector3 = p[j + 1] - pa
					var dd := d.length_squared()
					if dd < 1e-10:
						continue
					var t: float = clamp((s_pt[j] - pa).dot(d) / dd, 0.0, 1.0)
					var pen: float = (pa + d * t - s_pt[j]).dot(nn) - contact_radius
					if pen >= 0.0:
						continue
					var w1: float = inv_m[j] * (1.0 - t)
					var w2: float = inv_m[j + 1] * t
					var den := w1 * (1.0 - t) + w2 * t
					if den < 1e-12:
						continue
					var lam := -pen / den
					p[j] += nn * (lam * w1)
					p[j + 1] += nn * (lam * w2)
					s_dn[si] += -pen
			for e in cl_c:
				# the claws' too: else a taut cable is pulled deeper into a claw each sweep than its plates are thick
				var j: int = e[1]
				var nn: Vector3 = e[2]
				var d: Vector3 = p[j + 1] - p[j]
				var t: float = clamp((e[3] - p[j]).dot(d) / maxf(d.length_squared(), 1e-10), 0.0, 1.0)
				var pen: float = claw_contact_radius - (p[j] + d * t - e[3]).dot(nn)
				var w1: float = inv_m[j] * (1.0 - t)
				var w2: float = inv_m[j + 1] * t
				var den := w1 * (1.0 - t) + w2 * t
				if pen > 0.0 and den > 0.0:
					var c: int = _cs_claw[e[0] % _cs_claw.size()]
					p[j] += nn * (pen * w1 / den)
					p[j + 1] += nn * (pen * w2 / den)
					cl_pn[e[0]] += pen
					cl_imp[c] += nn * (pen / den)
					cl_pt[c] += (p[j] + d * t) * pen
					cl_ptw[c] += pen
		if rov_p.size() or rov_s.size():
			var tr0 := Time.get_ticks_usec()
			# (after the sweeps' other constraints, a few times so the hull wins over a taut cable)
			var near_r: float = _rov_bound + contact_radius
			var near_r2 := near_r * near_r
			for it2 in range(2):
				var touched := false
				for k in range(rov_p.size()):
					var i: int = rov_p[k]
					var ql: Vector3 = rxi * p[i]
					if ql.length_squared() > near_r2:
						continue
					_rov_closest(ql)
					var pen: float = contact_radius - _rc_d
					if pen > 0.0:
						touched = true
						var nw: Vector3 = rxf.basis * _rc_n
						rov_nrm[k] = nw
						var push: Vector3 = nw * pen
						p[i] += push
						rov_pn[k] += pen
						if it2 == 0 and s == substeps - 1:
							rov_cnt += 1
						var m := 1.0 / inv_m[i]
						rov_imp += push * m
						rov_pt_sum += p[i] * (pen * m)
						rov_pt_w += pen * m
				for sk in range(rov_s.size()):
					var j: int = rov_s[sk]
					var pa: Vector3 = rxi * p[j]
					var pb: Vector3 = rxi * p[j + 1]
					var d: Vector3 = pb - pa
					var dd := d.length_squared()
					if dd < 1e-10:
						continue
					# cheap reject: the segment's nearest approach to the body origin is outside every shape
					var t: float = clamp(-pa.dot(d) / dd, 0.0, 1.0)
					if (pa + d * t).length_squared() > near_r2:
						continue
					if it2 == 1:
						if rov_st[sk] < 0.0:
							continue  # was clear in the first pass and has hardly moved since
						t = rov_st[sk]
					# closest point of the segment to the hull: alternate projections (the shapes are convex)
					for kk in range(2 if it2 == 0 else 1):
						_rov_closest(pa + d * t)
						var c: Vector3 = pa + d * t - _rc_n * _rc_d
						t = clamp((c - pa).dot(d) / dd, 0.0, 1.0)
					_rov_closest(pa + d * t)
					var pen: float = contact_radius - _rc_d
					if it2 == 0:
						rov_st[sk] = t if pen > -0.05 else -1.0
					if pen <= 0.0 or t <= 0.0 or t >= 1.0:
						continue  # at an end the particle answers
					var w1: float = inv_m[j] * (1.0 - t)
					var w2: float = inv_m[j + 1] * t
					var den := w1 * (1.0 - t) + w2 * t
					if den < 1e-12:
						continue
					touched = true
					var nn: Vector3 = rxf.basis * _rc_n
					var lam := pen / den
					if it2 == 0 and s == substeps - 1:
						rov_cnt += 1
					p[j] += nn * (lam * w1)
					p[j + 1] += nn * (lam * w2)
					rov_imp += nn * lam
					var at: Vector3 = p[j].lerp(p[j + 1], t)
					rov_pt_sum += at * pen
					rov_pt_w += pen
					if it2 == 0 and s == 0 and rest[j] >= 2.0 * near_spacing:
						_rov_split.append([j, t])
					far_ticks[j] = 0
					far_ticks[j + 1] = 0
				if not touched:
					break  # nothing to re-check
			for k in range(rov_p.size()):
				if rov_pn[k] <= 0.0:
					continue
				var i: int = rov_p[k]
				far_ticks[i] = 0
				if rov_friction > 0.0:
					# Coulomb against the hull's own motion
					var nn: Vector3 = rov_nrm[k]
					var hull_dp: Vector3 = (rov_v + rov_w.cross(p[i] - rxf.origin)) * h
					var rel: Vector3 = (p[i] - x[i]) - hull_dp
					var tang := rel - nn * rel.dot(nn)
					var tl := tang.length()
					if tl > 1e-9:
						var fr: Vector3 = tang * min(1.0, rov_friction * rov_pn[k] / tl)
						p[i] -= fr
						rov_imp -= fr / inv_m[i]
			rov_us += Time.get_ticks_usec() - tr0
		if cl_s.size():
			var tc0 := Time.get_ticks_usec()
			for sk in range(cl_s.size()):
				for si in range(_cs_claw.size()):
					# the segment's point nearest each claw box is pushed out, shared by its ends by where along it is
					var j: int = cl_s[sk]
					var k := sk * _cs_claw.size() + si
					var pa: Vector3 = p[j]
					var d: Vector3 = p[j + 1] - pa
					var t := _claw_seg(si, pa, d, x[j], x[j + 1] - x[j])
					var pen: float = claw_contact_radius - _rc_d
					var w1: float = inv_m[j] * (1.0 - t)
					var w2: float = inv_m[j + 1] * t
					var den := w1 * (1.0 - t) + w2 * t
					if (pen <= 0.0 and cl_pn[k] <= 0.0) or den <= 0.0 \
							or (t <= 0.0 and sk > 0 and cl_s[sk - 1] == j - 1):
						continue  # (a particle between two of them is the earlier one's)
					var c: int = _cs_claw[si]
					var corr: Vector3 = _rc_n * (maxf(pen, 0.0) / den)
					cl_pn[k] += maxf(pen, 0.0)
					if claw_friction > 0.0:
						# then Coulomb against the claw's own motion, at the touching point
						var at: Vector3 = pa + d * t
						var cv: Vector3 = cl_v[c] + cl_w[c].cross(at - cl_com[c] - cl_v[c] * tau)
						var rel: Vector3 = at - x[j].lerp(x[j + 1], t) - cv * h
						var tang := rel - _rc_n * rel.dot(_rc_n)
						var tl := tang.length()
						if tl > 1e-9:
							corr -= tang * (min(1.0, claw_friction * cl_pn[k] / tl) / den)
					p[j] += corr * w1
					p[j + 1] += corr * w2
					cl_imp[c] += corr
					cl_pt[c] += (pa + d * t) * maxf(pen, 0.0)
					cl_ptw[c] += maxf(pen, 0.0)
					if s == substeps - 1:
						cl_cnt += 1
					if s == 0 and t > 0.0 and t < 1.0 and rest[j] >= 2.0 * bend_spacing:
						_rov_split.append([j, t, bend_spacing])
					far_ticks[j] = 0
					far_ticks[j + 1] = 0
			cl_us += Time.get_ticks_usec() - tc0
		if friction > 0.0:
			# Coulomb friction, once the sweeps are done: what slid along a surface this substep is taken back, up to
			# friction x how far that surface pushed out (static friction holds it, past that it slides)
			for ci in range(contact_list.size()):
				if c_dn[ci] <= 0.0:
					continue
				var i: int = contact_list[ci]
				var nn: Vector3 = c_n[i]
				var dp: Vector3 = p[i] - x[i]
				var tang := dp - nn * dp.dot(nn)
				var tl := tang.length()
				if tl > 1e-9:
					p[i] -= tang * min(1.0, friction * c_dn[ci] / tl)
			for si in range(seg_list.size()):
				if s_dn[si] <= 0.0:
					continue
				var j: int = seg_list[si]
				var nn: Vector3 = s_n[j]
				var d: Vector3 = p[j + 1] - p[j]
				var dd := d.length_squared()
				if dd < 1e-10:
					continue
				var t: float = clamp((s_pt[j] - p[j]).dot(d) / dd, 0.0, 1.0)
				var w1: float = inv_m[j] * (1.0 - t)
				var w2: float = inv_m[j + 1] * t
				var den := w1 * (1.0 - t) + w2 * t
				if den < 1e-12:
					continue
				# the touching point's slide, taken back from both ends by where along the segment it is
				var dp: Vector3 = (p[j] - x[j]) * (1.0 - t) + (p[j + 1] - x[j + 1]) * t
				var tang := dp - nn * dp.dot(nn)
				var tl := tang.length()
				if tl > 1e-9:
					var corr: Vector3 = tang * (min(1.0, friction * s_dn[si] / tl) / den)
					p[j] -= corr * w1
					p[j + 1] -= corr * w2
		# pay out what the brake let go of
		if fixed_count:
			var ds := 0.0
			for j in range(n):
				ds += p[j].distance_to(p[j + 1])
			# every segment stretches to share it, still braked: only what pulls past brake_tension pays out
			var over := ds - total
			if paying and over > 0.0 and -lambdas[n - 1] / h2 >= _brake_lim * 0.999:
				var grow = min(over, max_payout_speed * h, max_length - total)
				total += grow
				paid += grow
				for j in range(n):
					rest[j] = total / n
		elif paying:
			var ds := p[n].distance_to(p[n - 1])
			if ds > rest[n - 1]:
				var grow = min(ds - rest[n - 1], max_payout_speed * h, max_length - total)
				rest[n - 1] += grow
				total += grow
				paid += grow
		if reel_in and not fixed_count and total > min_length and -lambdas[n - 1] / h2 < reel_tension:
			# the winch winds in while the cable leaving it pulls less than reel_tension
			var take = min(reel_speed * h, total - min_length, rest[n - 1] - 0.01)
			if take > 0.0:
				rest[n - 1] -= take
				total -= take
				paid -= take
		var stop = clamp(1.0 - (max_length - total) / stop_length, 0.0, 1.0) if stop_length > 0 else 0.0
		_brake_lim = brake_tension + payout_damping * max(paid, 0.0) / (h * (s + 1)) + max_force * stop * stop
		tension_sum += -lambdas[0] / h2
		spool_sum += -lambdas[n - 1] / h2
		if gi >= 0:
			# the jaws feel the tension on each side of the particle they hold, along its two segments
			grip_sum -= ((p[gi - 1] - p[gi]).normalized() * lambdas[gi - 1]
					+ (p[gi + 1] - p[gi]).normalized() * lambdas[gi]) / h2
		for i in range(1, n):
			v[i] = (p[i] - x[i]) / h
			x[i] = p[i]
		x[0] = p[0]
	payout_speed = paid / dt
	# the cable's push back on the hull: what the hull moved it by, as momentum, over the tick
	rov_force = (-rov_imp / (h * dt)).limit_length(rov_max_force) if rov_imp != Vector3.ZERO else Vector3.ZERO
	_rov_point = rov_pt_sum / rov_pt_w if rov_pt_w > 0.0 else rov_xf.origin
	rov_contacts = rov_cnt
	rov_usec = rov_us
	claw_contacts = cl_cnt
	claw_usec = cl_us
	claw_force.fill(Vector3.ZERO)
	for c in range(cl_imp.size()):
		claw_force[c] = (-cl_imp[c] / (h * dt)).limit_length(claw_max_force)
		_claw_point[c] = cl_pt[c] / cl_ptw[c] if cl_ptw[c] > 0.0 else cl_xf[c].origin
	if fixed_count and paid != 0.0:
		_update_masses()
	tension = tension_sum / substeps
	spool_tension = spool_sum / substeps
	if gi >= 0:
		grip_force = grip_force.lerp(grip_sum / substeps, grip_smoothing).limit_length(max_force)
		_grip_over = _grip_over + 1 if grip_force.length() > grip_slip_force else 0
	force = ((x[1] - x[0]).normalized() * tension).limit_length(max_force)


func _payout_topology():
	if fixed_count:
		return
	var n := rest.size()
	# spacing by how much tether is between the vehicle and the last particle (not by index: _adapt changes those)
	var s_new := spacing if total - rest[n - 1] < fine_segments * spacing else far_spacing
	var cap := max_total_particles if (adaptive or cull) else max_particles
	if rest[n - 1] >= s_new + 0.25 and n + 2 <= cap:
		# split the cable leaving the spool: a new particle s_new from the last one
		var r: float = rest[n - 1]
		var f := s_new / r
		_insert(n, x[n - 1].lerp(x[n], f), v[n - 1].lerp(Vector3.ZERO, f), s_new, r - s_new, "payout_add")
	elif n > 1 and rest[n - 1] < 0.05 and x[n - 1].distance_to(x[n]) < 0.3:
		# reeled in past the last particle: it goes onto the spool
		_remove(n - 1, "reel_remove")


func _insert(i: int, pos: Vector3, vel: Vector3, rest_a: float, rest_b: float, kind := ""):
	# a new particle i on segment i - 1, which becomes rest_a (i - 1 to i) and rest_b (i to i + 1): length is kept
	if grip_i >= i:
		grip_i += 1
	if topology_log:
		_lg_before(i, kind, true, pos, rest_a, rest_b)
	x.insert(i, pos)
	v.insert(i, vel)
	p.insert(i, pos)
	c_has.insert(i, 0)
	c_pt.insert(i, Vector3.ZERO)
	c_n.insert(i, Vector3.UP)
	q_at.insert(i, Vector3.INF)
	far_ticks.insert(i, 0)
	rest[i - 1] = rest_a
	rest.insert(i, rest_b)
	lambdas.insert(i, 0.0)
	blambdas.resize(max(rest.size() - 1, 0))
	s_has.insert(i, s_has[i - 1])
	s_pt.insert(i, s_pt[i - 1])
	s_n.insert(i, s_n[i - 1])
	sa_at.insert(i, Vector3.INF)
	sb_at.insert(i, Vector3.INF)
	sa_at[i - 1] = Vector3.INF
	inv_m.insert(i, 0.0)
	weight.insert(i, 0.0)
	lift.insert(i, 0.0)
	kn.insert(i, 0.0)
	kt.insert(i, 0.0)
	for k in [i - 1, i, i + 1]:
		_mass_at(k)
	if topology_log:
		_lg_after(i, 1)


func _remove(i: int, kind := ""):
	# particle i goes, segments i - 1 and i become one (length kept); its momentum goes to its neighbours
	if i == grip_i:
		releases += 1  # (only reeled in onto the spool: merges keep clear of it)
		_grip_release()
	elif grip_i > i:
		grip_i -= 1
	if topology_log:
		_lg_before(i, kind, false, x[i], rest[i - 1], rest[i])
	var mi := 1.0 / inv_m[i] if inv_m[i] > 0 else 0.0
	var ra: float = rest[i - 1]
	var rb: float = rest[i]
	# each neighbour takes on half of the segment beyond it, so that share of i's momentum, over its new mass below
	var mom: Array[Vector3] = []
	for nb in [i - 1, i + 1]:
		var mn: float = 1.0 / inv_m[nb] if inv_m[nb] > 0 else 0.0
		mom.append(v[nb] * mn + v[i] * (mi * (rb if nb == i - 1 else ra) / (ra + rb)))
	rest[i - 1] += rest[i]
	rest.remove_at(i)
	lambdas.remove_at(i)
	blambdas.resize(max(rest.size() - 1, 0))
	x.remove_at(i)
	v.remove_at(i)
	p.remove_at(i)
	c_has.remove_at(i)
	c_pt.remove_at(i)
	c_n.remove_at(i)
	q_at.remove_at(i)
	far_ticks.remove_at(i)
	s_has.remove_at(i)
	s_pt.remove_at(i)
	s_n.remove_at(i)
	sa_at.remove_at(i)
	sb_at.remove_at(i)
	s_has[i - 1] = 0
	sa_at[i - 1] = Vector3.INF
	inv_m.remove_at(i)
	weight.remove_at(i)
	lift.remove_at(i)
	kn.remove_at(i)
	kt.remove_at(i)
	for k in [i - 1, i]:
		_mass_at(k)
		if inv_m[k] > 0 and mi > 0:
			v[k] = mom[k - i + 1] * inv_m[k]
	if topology_log:
		_lg_after(i, -1)


func _adapt():
	var changes := 0
	var n := rest.size()
	if adaptive and not _rov_split.is_empty():
		# touching the hull or a claw: split there (highest index first, so the others stay valid; before the
		# splits below, as the indices are from the solve)
		_rov_split.sort_custom(func(a, b): return a[0] > b[0])
		var last := -1
		for e in _rov_split:
			var j: int = e[0]
			if j == last or changes >= topology_changes_per_tick or x.size() >= max_total_particles or j >= n - 1:
				continue
			last = j
			var piece: float = e[2] if e.size() > 2 else near_spacing  # (a claw gives its own)
			var lo := piece / rest[j]
			var t: float = clamp(e[1], lo, 1.0 - lo)
			if rest[j] < 2.0 * piece:
				continue
			var r: float = rest[j]
			_insert(j + 1, x[j].lerp(x[j + 1], t), v[j].lerp(v[j + 1], t), r * t, r * (1.0 - t),
				"split_claw" if e.size() > 2 else "split_rov")
			changes += 1
			splits += 1
			n += 1
	if adaptive and seg_contacts > 0:
		# near a solid: split the segment where it touches, so a particle (with the segment's contact plane) sits
		# on the edge or corner and the tether bends there instead of cutting through
		var j := 0
		while j < n - 1 and changes < topology_changes_per_tick and x.size() < max_total_particles:
			if s_has[j] and rest[j] >= 2.0 * near_spacing:
				var a: Vector3 = x[j]
				var d: Vector3 = x[j + 1] - a
				var lo := near_spacing / rest[j]
				var t: float = clamp((s_pt[j] - a).dot(d) / max(d.length_squared(), 1e-10), lo, 1.0 - lo)
				var pos := a + d * t
				var pen: float = (pos - s_pt[j]).dot(s_n[j]) - contact_radius
				if pen < 0.0:
					pos -= s_n[j] * pen
				var r: float = rest[j]
				var cpt: Vector3 = s_pt[j]
				var cn: Vector3 = s_n[j]
				_insert(j + 1, pos, v[j].lerp(v[j + 1], t), r * t, r * (1.0 - t), "split_contact")
				c_has[j + 1] = 1
				c_pt[j + 1] = cpt
				c_n[j + 1] = cn
				q_at[j + 1] = pos
				changes += 1
				splits += 1
				n += 1
				j += 2
			else:
				j += 1
	# bends and straight runs: a few joints a tick
	var cos_refine := cos(deg_to_rad(refine_bend_deg))
	var cos_cull := cos(deg_to_rad(cull_straight_deg))
	var cos_keep := cos(deg_to_rad(0.5 * refine_bend_deg))  # a neighbour bending this much keeps its split
	for k in range(topology_scan_per_tick):
		if changes >= topology_changes_per_tick or n < 3:
			break
		if _tcursor >= n - 1:
			_tcursor = 1
		var i := _tcursor
		_tcursor += 1
		var d1: Vector3 = x[i] - x[i - 1]
		var d2: Vector3 = x[i + 1] - x[i]
		var l1 := d1.length()
		var l2 := d2.length()
		if l1 < 1e-6 or l2 < 1e-6:
			continue
		var c := d1.dot(d2) / (l1 * l2)
		if adaptive and c < cos_refine and x.size() < max_total_particles:
			# a bend: split the longer of its two segments next to the joint, at most twice the shorter one away (so
			# the spacing grades down to bend_spacing at a tight bend), for a smooth, tight curve
			var j := i - 1 if rest[i - 1] >= rest[i] else i
			var r: float = rest[j]
			var other: float = rest[i] if j == i - 1 else rest[i - 1]
			var bsp: float = bend_spacing if far_ticks[i] < merge_delay else free_bend_spacing
			if j < n - 1 and r >= 2.0 * bsp and r > 1.5 * other:
				var piece: float = clamp(2.0 * other, bsp, 0.5 * r)
				var t: float = 1.0 - piece / r if j == i - 1 else piece / r  # measured from the joint
				_insert(j + 1, x[j].lerp(x[j + 1], t), v[j].lerp(v[j + 1], t), r * t, r * (1.0 - t), "split_bend")
				changes += 1
				splits += 1
				n += 1
			elif j < n - 1 and r >= 2.0 * bsp:
				# both about as long: halve the longer
				_insert(j + 1, x[j].lerp(x[j + 1], 0.5), v[j].lerp(v[j + 1], 0.5), r * 0.5, r * 0.5, "split_bend")
				changes += 1
				splits += 1
				n += 1
		elif cull and segment_collide and c > cos_cull and i < n - 1 and far_ticks[i] >= merge_delay and not c_has[i] \
				and not s_has[i - 1] and not s_has[i] and rest[i - 1] + rest[i] <= max_segment \
				and maxf(rest[i - 1], rest[i]) <= 2.0 * minf(rest[i - 1], rest[i]) \
				and (i == 1 or far_ticks[i - 1] >= merge_delay) and far_ticks[i + 1] >= merge_delay \
				and not _bent(i - 1, cos_keep) and not _bent(i + 1, cos_keep) and not _stretched(i - 2, i + 1) \
				and (grip_i < 0 or absi(i - grip_i) > 1):
			# straight and clear of solids: one particle fewer. Only pieces about as long as each other (so no
			# neighbour's mass more than doubles: graded pieces go back up by doubling) and nothing stretched next to
			# it, or the new masses let the stretch go in one tick
			_remove(i, "merge_cull")
			changes += 1
			merges += 1
			n -= 1


func _bent(i: int, cos_lim: float) -> bool:
	# does the joint at particle i turn more than acos(cos_lim)?
	if i < 1 or i >= x.size() - 1:
		return false
	var d1: Vector3 = x[i] - x[i - 1]
	var d2: Vector3 = x[i + 1] - x[i]
	var l := d1.length() * d2.length()
	return l > 1e-12 and d1.dot(d2) / l < cos_lim


func _stretched(a: int, b: int) -> bool:
	# is any of segments a..b longer than its rest by more than merge_max_stretch?
	for j in range(maxi(a, 0), mini(b, rest.size() - 1) + 1):
		if x[j].distance_to(x[j + 1]) - rest[j] > merge_max_stretch:
			return true
	return false


func _lg_tick(dt: float):
	# after the solve, before this tick's topology changes: each particle's acceleration and jerk from the solve alone
	var n := x.size() - 1
	if _lg_pid.size() != n + 1:
		_lg_pid.resize(n + 1)
		_lg_born.resize(n + 1)
		for i in range(n + 1):
			_lg_pid[i] = _lg_next_id
			_lg_next_id += 1
		_lg_born.fill(-1)
		_lg_a.resize(n + 1)
		_lg_a.fill(Vector3.INF)
		_lg_v = v.duplicate()
	var accm := PackedFloat32Array()
	var jerkm := PackedFloat32Array()
	accm.resize(n + 1)
	jerkm.resize(n + 1)
	var amax := 0.0
	var ai := -1
	var jmax := 0.0
	for i in range(1, n):
		var a: Vector3 = (v[i] - _lg_v[i]) / dt
		var pa: Vector3 = _lg_a[i]
		accm[i] = a.length()
		jerkm[i] = (a - pa).length() / dt if pa.is_finite() else 0.0
		_lg_a[i] = a
		if accm[i] > amax:
			amax = accm[i]
			ai = i
		jmax = maxf(jmax, jerkm[i])
	_lg_hist.append([_lg_pid.duplicate(), accm, jerkm])
	if _lg_hist.size() > 10:
		_lg_hist.pop_front()
	for w in _lg_win:
		for id in w[1]:
			var k := _lg_pid.find(id)
			if k > 0 and k < n:
				w[0][23] = maxf(w[0][23], accm[k])
				w[0][24] = maxf(w[0][24], jerkm[k])
				if w[2] == 10:
					w[0][31] = maxf(w[0][31], accm[k])
		w[2] -= 1
	_lg_win = _lg_win.filter(func(w): return w[2] > 0)
	_lg_row = [_ticks, n, 0.0, amax, ai, jmax]
	_lg_tick_events.clear()
	_lg_dv = 0.0
	_lg_render = 0.0


func _lg_end_tick():
	var sr := 0.0
	var smax := 0.0
	var emax := 0.0
	for j in range(rest.size()):
		sr += rest[j]
		var e: float = x[j].distance_to(x[j + 1]) - rest[j]
		smax = maxf(smax, e)
		emax = maxf(emax, e / rest[j])
	_lg_row[2] = _lg_energy()[0]
	_lg_row.append_array([_lg_dv, sr - total, _lg_tick_events.size(), _lg_render, smax, emax])
	topo_ticks.append(_lg_row)
	_lg_v = v.duplicate()


func _lg_energy() -> Array:
	var ke := 0.0
	var mom := Vector3.ZERO
	for i in range(1, x.size() - 1):
		if inv_m[i] > 0.0:
			var m := 1.0 / inv_m[i]
			ke += 0.5 * m * v[i].length_squared()
			mom += v[i] * m
	return [ke, mom]


func _lg_stretch(segs: Array) -> Array:
	# [sum over segs of chord - rest, the largest of them]
	var s := 0.0
	var worst := -INF
	for j in segs:
		var e: float = x[j].distance_to(x[j + 1]) - rest[j]
		s += e
		worst = maxf(worst, e)
	return [s, worst]


func _lg_bend_viol(a: int, b: int) -> float:
	# worst shortfall of particles two apart from the bend limit's distance (as _solve's bmin), joints a + 1..b + 1
	if max_bend_deg >= 180.0:
		return 0.0
	var worst := 0.0
	for j in range(maxi(a, 0), mini(b, rest.size() - 2) + 1):
		var ra: float = rest[j]
		var rb: float = rest[j + 1]
		var th: float = min(_bend_cap, 0.5 * (ra + rb) / maxf(min_bend_radius, 1e-4))
		var bm := sqrt(max(ra * ra + rb * rb + 2.0 * ra * rb * cos(th), 0.0))
		worst = maxf(worst, bm - x[j].distance_to(x[j + 2]))
	return worst


func _lg_curve(a: int, b: int) -> PackedVector3Array:
	# the centreline _process draws from particle a to b, as near the camera; keep in step with _process
	var n := x.size() - 1
	var cos_bend := cos(deg_to_rad(bend_subdivide_deg))
	var out := PackedVector3Array()
	for i in range(a, b):
		var p0: Vector3 = x[max(i - 1, 0)]
		var p1: Vector3 = x[i]
		var p2: Vector3 = x[i + 1]
		var p3: Vector3 = x[min(i + 2, n)]
		var sub := 1
		if render_subdivisions > 1:
			var d1 := (p2 - p1).normalized()
			if (p1 - p0).normalized().dot(d1) < cos_bend or d1.dot((p3 - p2).normalized()) < cos_bend:
				sub = render_subdivisions
		if sub == 1:
			out.append(p1)
			continue
		var d01 := maxf(p0.distance_to(p1), 1e-6)
		var d12 := maxf(p1.distance_to(p2), 1e-6)
		var d23 := maxf(p2.distance_to(p3), 1e-6)
		var m1: Vector3 = ((p1 - p0) * (d12 / d01) * d12 + (p2 - p1) * d01) / (d01 + d12) if i > 0 else p2 - p1
		var m2: Vector3 = ((p2 - p1) * d23 + (p3 - p2) * (d12 / d23) * d12) / (d12 + d23) if i + 2 <= n else p2 - p1
		m1 = (p2 - p1).lerp(m1, minf(1.0, 2.0 * d01 / d12))
		m2 = (p2 - p1).lerp(m2, minf(1.0, 2.0 * d23 / d12))
		for sdiv in range(sub):
			var t := float(sdiv) / sub
			var t2 := t * t
			var t3 := t2 * t
			out.append((2.0 * t3 - 3.0 * t2 + 1.0) * p1 + (t3 - 2.0 * t2 + t) * m1 + (-2.0 * t3 + 3.0 * t2) * p2 + (t3 - t2) * m2)
	out.append(x[b])
	return out


static func _lg_far(a: PackedVector3Array, b: PackedVector3Array) -> float:
	# farthest point of polyline a (each piece sampled 4 times) from polyline b
	var worst := 0.0
	for k in range(a.size()):
		for s in range(4 if k < a.size() - 1 else 1):
			var q: Vector3 = a[k].lerp(a[k + 1], s * 0.25) if s > 0 else a[k]
			var best := INF
			for m in range(b.size() - 1):
				best = minf(best, q.distance_squared_to(Geometry3D.get_closest_point_to_segment(q, b[m], b[m + 1])))
			worst = maxf(worst, best)
	return sqrt(worst)


func _lg_before(i: int, kind: String, ins: bool, pos: Vector3, rest_a: float, rest_b: float):
	# i: the new particle's index (insert) or the one going (remove, pos is where it is)
	var n := x.size() - 1
	var nb_hi := i + 2 if ins else i + 3
	var vel := {}
	var at := {}
	var mass := {}
	var ids := []
	for k in range(maxi(i - 3, 1), mini(nb_hi, n - 1) + 1):
		if not ins and k == i:
			continue
		vel[_lg_pid[k]] = v[k]
		at[_lg_pid[k]] = x[k]
		mass[_lg_pid[k]] = 1.0 / inv_m[k] if inv_m[k] > 0.0 else 0.0
		ids.append(_lg_pid[k])
	var nb_st := 0.0
	for j in range(maxi(i - 3, 0), mini(i + 2, n - 1) + 1):
		nb_st = maxf(nb_st, absf(x[j].distance_to(x[j + 1]) - rest[j]))
	var a_b := 0.0
	var j_b := 0.0
	for h in _lg_hist:
		for id in ids + ([] if ins else [_lg_pid[i]]):
			var k: int = h[0].find(id)
			if k >= 0:
				a_b = maxf(a_b, h[1][k])
				j_b = maxf(j_b, h[2][k])
	var lo: Vector3 = x[i - 1]
	var hi: Vector3 = x[i] if ins else x[i + 1]
	var off := pos.distance_to(Geometry3D.get_closest_point_to_segment(pos, lo, hi))
	var s := maxi(0, i - 4)
	var e := mini(n, i + 3 if ins else i + 4)
	var sr := 0.0
	for r in rest:
		sr += r
	var en := _lg_energy()
	var stale := 0
	if kind in ["split_rov", "split_claw"]:
		for ev in _lg_tick_events:
			stale |= int(ev <= i)
	_lg_pre = {"kind": kind, "i": i, "n": n, "ins": ins, "vel": vel, "at": at, "mass": mass, "nb_st": nb_st, "a_b": a_b, "j_b": j_b, "off": off,
		"rests": [rest[i - 1], rest_a, rest_b] if ins else [rest_a, rest_b, rest_a + rest_b],
		"st": _lg_stretch([i - 1] if ins else [i - 1, i]), "bend": _lg_bend_viol(i - 3, i + 1),
		"curve": _lg_curve(s, e), "s": s, "e": e, "sr": sr, "ke": en[0], "mom": en[1], "stale": stale,
		"life": 0 if ins else (_ticks - _lg_born[i] if _lg_born[i] >= 0 else -1), "speed": v[i].length() if not ins else 0.0}


func _lg_after(i: int, dn: int):
	var pr := _lg_pre
	if dn > 0:
		_lg_pid.insert(i, _lg_next_id)
		_lg_next_id += 1
		_lg_born.insert(i, _ticks)
		var a0: Vector3 = _lg_a[i - 1]
		var a1: Vector3 = _lg_a[i]
		_lg_a.insert(i, a0.lerp(a1, 0.5) if a0.is_finite() and a1.is_finite() else Vector3.INF)
		pr.speed = v[i].length()
	else:
		_lg_pid.remove_at(i)
		_lg_born.remove_at(i)
		_lg_a.remove_at(i)
	var n := x.size() - 1
	var dpos := 0.0
	var dv := 0.0
	var mj := 1.0
	for k in range(maxi(i - 3, 1), mini(i + 3, n - 1) + 1):
		var id := _lg_pid[k]
		if pr.at.has(id):
			dpos = maxf(dpos, x[k].distance_to(pr.at[id]))
			dv = maxf(dv, v[k].distance_to(pr.vel[id]))
			var m0: float = pr.mass[id]
			var m1 := 1.0 / inv_m[k] if inv_m[k] > 0.0 else 0.0
			if m0 > 0.0 and m1 > 0.0:
				mj = maxf(mj, maxf(m1 / m0, m0 / m1))
	var st := _lg_stretch([i - 1, i] if dn > 0 else [i - 1])
	var sr := 0.0
	for r in rest:
		sr += r
	var en := _lg_energy()
	var rd := _lg_far(pr.curve, _lg_curve(pr.s, pr.e + dn))
	rd = maxf(rd, _lg_far(_lg_curve(pr.s, pr.e + dn), pr.curve))
	var row := [_ticks, pr.kind, i, pr.n, n, pr.rests[0], pr.rests[1], pr.rests[2], pr.off, pr.st[0], st[0], st[1],
		pr.bend, _lg_bend_viol(i - 3, i + 1), dpos, dv, sr - pr.sr, sr - total, en[0] - pr.ke, (en[1] - pr.mom).length(),
		rd, pr.a_b, pr.j_b, 0.0, 0.0, pr.stale, pr.life, pr.ke, pr.speed, pr.nb_st, mj, 0.0]
	topo_events.append(row)
	var ids := []
	for k in range(maxi(i - 3, 1), mini(i + 3, n - 1) + 1):
		ids.append(_lg_pid[k])
	_lg_win.append([row, ids, 10])
	_lg_tick_events.append(i)
	_lg_dv = maxf(_lg_dv, dv)
	_lg_render = maxf(_lg_render, rd)


func _build_mesh():
	_mesh = ArrayMesh.new()
	var mi = MeshInstance3D.new()
	mi.mesh = _mesh
	mi.top_level = true
	mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	mi.extra_cull_margin = 1000.0
	var mat = StandardMaterial3D.new()
	mat.albedo_color = color
	mat.emission_enabled = true
	mat.emission = color
	mat.emission_energy_multiplier = 0.3
	mi.material_override = mat
	add_child(mi)
	mi.global_transform = Transform3D.IDENTITY
	# rings are joined in order, so the index buffer for fewer rings is a prefix of this one
	var rings = (max(max_particles, max_total_particles) + 1) * render_subdivisions + 2
	_indices.resize((rings - 1) * radial_sides * 6)
	var q = 0
	for r in range(rings - 1):
		for s in range(radial_sides):
			var a0 = r * radial_sides + s
			var a1 = r * radial_sides + (s + 1) % radial_sides
			var b0 = a0 + radial_sides
			var b1 = a1 + radial_sides
			_indices[q] = a0; _indices[q + 1] = b0; _indices[q + 2] = a1
			_indices[q + 3] = a1; _indices[q + 4] = b0; _indices[q + 5] = b1
			q += 6


func _process(_delta):
	if rov == null or _mesh == null:
		return
	var t0 = Time.get_ticks_usec()
	var n := x.size() - 1
	var pts := x.duplicate()
	pts[0] = attach_point()  # drawn where the vehicle is this frame, not where the last tick left it
	pts[n] = anchor
	var cam := get_viewport().get_camera_3d()
	var cam_pos := cam.global_position if cam else pts[0]
	var lod2 := lod_distance * lod_distance
	var cos_bend := cos(deg_to_rad(bend_subdivide_deg))
	var centre := PackedVector3Array()
	centre.resize(n * render_subdivisions + 1)
	var r := 0
	for i in range(n):
		var p0: Vector3 = pts[max(i - 1, 0)]
		var p1: Vector3 = pts[i]
		var p2: Vector3 = pts[i + 1]
		var p3: Vector3 = pts[min(i + 2, n)]
		var sub := 1
		if render_subdivisions > 1 and (p1.distance_squared_to(cam_pos) < lod2 or p2.distance_squared_to(cam_pos) < lod2):
			var d1 := (p2 - p1).normalized()
			if (p1 - p0).normalized().dot(d1) < cos_bend or d1.dot((p3 - p2).normalized()) < cos_bend:
				sub = render_subdivisions
		if sub == 1:
			centre[r] = p1
			r += 1
			continue
		# Hermite with tangents scaled to the neighbouring segments' lengths (a uniform Catmull-Rom overshoots into
		# wide loops where a short segment meets a long one, which is what graded bends are made of)
		var d01 := maxf(p0.distance_to(p1), 1e-6)
		var d12 := maxf(p1.distance_to(p2), 1e-6)
		var d23 := maxf(p2.distance_to(p3), 1e-6)
		var m1: Vector3 = ((p1 - p0) * (d12 / d01) * d12 + (p2 - p1) * d01) / (d01 + d12) if i > 0 else p2 - p1
		var m2: Vector3 = ((p2 - p1) * d23 + (p3 - p2) * (d12 / d23) * d12) / (d12 + d23) if i + 2 <= n else p2 - p1
		# a much shorter neighbour only turns the end in proportion: a stub would bow a long segment far out
		m1 = (p2 - p1).lerp(m1, minf(1.0, 2.0 * d01 / d12))
		m2 = (p2 - p1).lerp(m2, minf(1.0, 2.0 * d23 / d12))
		for sdiv in range(sub):
			var t := float(sdiv) / sub
			var t2 := t * t
			var t3 := t2 * t
			centre[r] = (2.0 * t3 - 3.0 * t2 + 1.0) * p1 + (t3 - 2.0 * t2 + t) * m1 + (-2.0 * t3 + 3.0 * t2) * p2 + (t3 - t2) * m2
			r += 1
	centre[r] = pts[n]
	var rings := r + 1
	rings_drawn = rings
	var verts := PackedVector3Array()
	var normals := PackedVector3Array()
	verts.resize(rings * radial_sides)
	normals.resize(rings * radial_sides)
	var radius := (draw_diameter if draw_diameter > 0 else diameter) / 2
	var ref := Vector3.UP
	var q := 0
	var cs := PackedFloat32Array()
	var sn := PackedFloat32Array()
	for s in range(radial_sides):
		cs.append(cos(TAU * s / radial_sides))
		sn.append(sin(TAU * s / radial_sides))
	for k in range(rings):
		var tangent: Vector3 = centre[min(k + 1, rings - 1)] - centre[max(k - 1, 0)]
		if tangent.length_squared() < 1e-12:
			tangent = Vector3.RIGHT
		tangent = tangent.normalized()
		var side := tangent.cross(ref)
		if side.length_squared() < 1e-6:
			ref = Vector3.RIGHT if abs(tangent.x) < 0.9 else Vector3.FORWARD
			side = tangent.cross(ref)
		side = side.normalized()
		var up := side.cross(tangent)
		ref = up  # parallel-transport the frame so the tube doesn't twist where the cable turns
		var c: Vector3 = centre[k]
		for s in range(radial_sides):
			var nrm := side * cs[s] + up * sn[s]
			normals[q] = nrm
			verts[q] = c + nrm * radius
			q += 1
	var arrays = []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = verts
	arrays[Mesh.ARRAY_NORMAL] = normals
	arrays[Mesh.ARRAY_INDEX] = _indices.slice(0, (rings - 1) * radial_sides * 6)
	_mesh.clear_surfaces()
	_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	render_usec = Time.get_ticks_usec() - t0
