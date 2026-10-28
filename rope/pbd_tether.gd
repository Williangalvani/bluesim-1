extends Node3D
# Cheap ROV tether: an XPBD particle rope (no rigid bodies, no joints) solved in GDScript.
#
# The first particle rides on the vehicle at this node's position, the last one is pinned where the tether was
# laid out (the topside end), unless `anchored` is off. Each physics tick the rope gets net buoyancy, quadratic
# water drag across and along the cable (implicit, so it never overshoots), and inextensible-but-slack distance
# constraints; the pull of the first segment, read from its XPBD multiplier, is applied to the vehicle at the
# attachment point, so the vehicle feels the drag of the tether it is towing and stops when it runs out of slack.
# It is drawn as a Catmull-Rom tube through the particles in _process, so few particles still look smooth.

const WATER_DENSITY = 1000.0

@export var length := 30.0  # m of tether deployed
@export var segments := 40  # particles - 1; 0.75 m each at 30 m
@export var substeps := 2  # per physics tick
@export var iterations := 4  # constraint sweeps per substep, alternating direction
@export var diameter := 0.0076  # m, Fathom slim tether
@export var buoyancy_ratio := 1.05  # buoyancy / weight, slightly positive like a real ROV tether
@export var normal_cd := 1.2  # cylinder form drag across the cable
@export var axial_cf := 0.005  # skin friction along it (was 0.01)
@export var compliance := 1e-5  # m/N per segment: a little stretch keeps the vehicle coupling stable
@export var max_force := 200.0  # N, the most the tether may pull on the vehicle
@export var update_every := 1  # solve every N physics ticks (with N times the step), for level of detail
@export var anchored := true
@export var anchor_distance := 10.0  # m from the vehicle to the topside end, the rest of the tether is slack
@export var render_subdivisions := 4  # spline points per segment
@export var radial_sides := 6
@export var color := Color(0.96, 0.75, 0.04)

var rov: RigidBody3D
var attach_local := Vector3.ZERO
var anchor := Vector3.ZERO
var seg_len := 0.0
var particle_mass := 0.0  # kg, cable plus water it carries along (added mass)
var net_lift := 0.0  # N per particle, buoyancy minus weight, fully submerged
var kn := 0.0  # N/(m/s)^2 per particle, across
var kt := 0.0  # along
var x := PackedVector3Array()
var v := PackedVector3Array()
var p := PackedVector3Array()
var lambdas := PackedFloat64Array()
var force := Vector3.ZERO  # last force applied to the vehicle, world frame
var tension := 0.0  # N, along the first segment
var solve_usec := 0  # time the last solve took
var render_usec := 0
var _tick := 0
var _mesh: ArrayMesh
var _mesh_instance: MeshInstance3D
var _indices := PackedInt32Array()
var _material: StandardMaterial3D


func _ready():
	if Engine.is_editor_hint():
		return
	rov = get_tree().get_first_node_in_group("vehicles")
	if rov == null:
		push_warning("pbd_tether: no vehicle")
		set_physics_process(false)
		set_process(false)
		return
	attach_local = rov.to_local(global_position)
	seg_len = length / segments
	var area = PI * diameter * diameter / 4
	var displaced = WATER_DENSITY * area * seg_len
	var cable_mass = displaced / buoyancy_ratio
	particle_mass = cable_mass + displaced  # added mass of a cylinder moving sideways ~ the water it displaces
	net_lift = (displaced - cable_mass) * 9.8
	kn = 0.5 * WATER_DENSITY * normal_cd * diameter * seg_len
	kt = 0.5 * WATER_DENSITY * axial_cf * PI * diameter * seg_len
	# lay it out away from the vehicle's centre, the way the rigid tether was, zigzagging sideways on the
	# water so the topside end is anchor_distance away and the rest is slack
	var start = global_position
	var away = start - rov.global_position
	away.y = 0
	away = away.normalized() if away.length() > 0.01 else Vector3(-1, 0, 0)
	var side = away.cross(Vector3.UP).normalized()
	var reach = clamp(anchor_distance / length, 0.0, 1.0) if anchored else 1.0
	var swing = sqrt(1.0 - reach * reach)
	x.resize(segments + 1)
	v.resize(segments + 1)
	p.resize(segments + 1)
	lambdas.resize(segments)
	x[0] = start
	v[0] = Vector3.ZERO
	for i in range(1, segments + 1):
		var lateral = swing if (i / 2) % 2 == 0 else -swing
		x[i] = x[i - 1] + (away * reach + side * lateral) * seg_len
		v[i] = Vector3.ZERO
	anchor = x[segments]
	_build_mesh()


func attach_point() -> Vector3:
	return rov.global_transform * attach_local


func _physics_process(delta):
	if rov == null:
		return
	_tick += 1
	if _tick % update_every == 0:
		var t0 = Time.get_ticks_usec()
		_solve(delta * update_every)
		solve_usec = Time.get_ticks_usec() - t0
	# between solves keep pulling with the last force, so a lower update rate doesn't change the load
	rov.apply_force(force, attach_point() - rov.global_position)


func _solve(dt):
	var surface = Globals.surface_altitude
	var h = dt / substeps
	var n = segments
	var a = attach_point()
	var offset = a - rov.global_position
	var a_vel = rov.linear_velocity + rov.angular_velocity.cross(offset)
	var alpha = compliance / (h * h)
	var inv_m = 1.0 / particle_mass
	var last_free = n if not anchored else n - 1
	var tension_sum = 0.0
	for s in range(substeps):
		# the vehicle moves during the tick; follow it along its current velocity
		var a_sub = a + a_vel * h * (s + 1)
		x[0] = a + a_vel * h * s
		v[0] = a_vel
		p[0] = a_sub
		for i in range(1, last_free + 1):
			var vi = v[i]
			var xi = x[i]
			var submerged = clamp((surface - xi.y) / diameter + 0.5, 0.0, 1.0)
			# buoyancy only acts on what is under water; out of it the cable falls back onto the surface
			vi.y += h * (net_lift * submerged - 9.8 * particle_mass * (1.0 - submerged)) * inv_m
			var tangent = x[min(i + 1, n)] - x[i - 1]
			var tl = tangent.length()
			if tl > 1e-6 and submerged > 0:
				tangent /= tl
				var vt = tangent * vi.dot(tangent)
				var vn = vi - vt
				# implicit quadratic drag: v' = v / (1 + h k |v| / m) can slow the cable but never reverse it
				vn /= 1.0 + h * kn * submerged * vn.length() * inv_m
				vt /= 1.0 + h * kt * submerged * vt.length() * inv_m
				vi = vn + vt
			v[i] = vi
			p[i] = xi + vi * h
		if anchored:
			p[n] = anchor
		lambdas.fill(0.0)
		for it in range(iterations):
			var forward = it % 2 == 0
			for k in range(n):
				var j = k if forward else n - 1 - k
				# inverse masses: the vehicle end and the pinned topside end don't move
				var w1 = 0.0 if j == 0 else inv_m
				var w2 = 0.0 if (j + 1 == n and anchored) else inv_m
				var d = p[j + 1] - p[j]
				var dl = d.length()
				var c = dl - seg_len
				# a rope only pulls: slack segments are left alone, so it can fold without fighting itself
				if c <= 0.0 or dl < 1e-9:
					continue
				var dlambda = (-c - alpha * lambdas[j]) / (w1 + w2 + alpha)
				lambdas[j] += dlambda
				var corr = d * (dlambda / dl)
				p[j] -= corr * w1
				p[j + 1] += corr * w2
		# XPBD: the constraint force is lambda / h^2; the first segment's is what the vehicle feels
		tension_sum += -lambdas[0] / (h * h)
		for i in range(1, last_free + 1):
			v[i] = (p[i] - x[i]) / h
			x[i] = p[i]
		x[0] = p[0]
	tension = tension_sum / substeps
	var dir = (x[1] - x[0]).normalized()
	force = (dir * tension).limit_length(max_force)


func _build_mesh():
	_mesh = ArrayMesh.new()
	_mesh_instance = MeshInstance3D.new()
	_mesh_instance.mesh = _mesh
	_mesh_instance.top_level = true
	_mesh_instance.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	_mesh_instance.extra_cull_margin = 1000.0
	_material = StandardMaterial3D.new()
	_material.albedo_color = color
	_material.emission_enabled = true
	_material.emission = color
	_material.emission_energy_multiplier = 0.3
	_mesh_instance.material_override = _material
	add_child(_mesh_instance)
	_mesh_instance.global_transform = Transform3D.IDENTITY
	var rings = segments * render_subdivisions + 1
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
	var n = segments
	var pts = x.duplicate()
	pts[0] = attach_point()  # drawn where the vehicle is this frame, not where the last tick left it
	var rings = n * render_subdivisions + 1
	var centre = PackedVector3Array()
	centre.resize(rings)
	var r = 0
	for i in range(n):
		var p0 = pts[max(i - 1, 0)]
		var p1 = pts[i]
		var p2 = pts[i + 1]
		var p3 = pts[min(i + 2, n)]
		for sdiv in range(render_subdivisions):
			var t = float(sdiv) / render_subdivisions
			var t2 = t * t
			var t3 = t2 * t
			centre[r] = 0.5 * ((2.0 * p1) + (p2 - p0) * t + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2 + (3.0 * p1 - p0 - 3.0 * p2 + p3) * t3)
			r += 1
	centre[r] = pts[n]
	var verts = PackedVector3Array()
	var normals = PackedVector3Array()
	verts.resize(rings * radial_sides)
	normals.resize(rings * radial_sides)
	var radius = diameter / 2
	var ref = Vector3.UP
	var q = 0
	for k in range(rings):
		var tangent = centre[min(k + 1, rings - 1)] - centre[max(k - 1, 0)]
		if tangent.length_squared() < 1e-12:
			tangent = Vector3.RIGHT
		tangent = tangent.normalized()
		var side = tangent.cross(ref)
		if side.length_squared() < 1e-6:
			ref = Vector3.RIGHT
			side = tangent.cross(ref)
		side = side.normalized()
		var up = side.cross(tangent)
		ref = up  # parallel-transport the frame so the tube doesn't twist where the cable turns
		for s in range(radial_sides):
			var ang = TAU * s / radial_sides
			var nrm = side * cos(ang) + up * sin(ang)
			normals[q] = nrm
			verts[q] = centre[k] + nrm * radius
			q += 1
	var arrays = []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = verts
	arrays[Mesh.ARRAY_NORMAL] = normals
	arrays[Mesh.ARRAY_INDEX] = _indices
	_mesh.clear_surfaces()
	_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	render_usec = Time.get_ticks_usec() - t0
