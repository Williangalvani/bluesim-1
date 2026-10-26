extends Node3D
## The boat's ground tackle: an anchor on the seabed ahead of the bow and its chain up to the bow roller.
##
## Child of the boat (boat.tscn). At start it looks for the seabed under a point `scope` x depth ahead of the bow
## (a ray down, excluding the boat), lays the anchor there, and from then on keeps the boat's horizontal position
## and heading fixed relative to it: boat_float.gd still makes the boat heave, pitch and roll, but it does not
## drift, so the TetherAnchor's topside end stays put. swing_degrees > 0 lets it yaw slowly round the anchor
## (the bow stays on the chain's lead), which moves the tether's end by about the boat length x the swing.
## Light for the web build: the anchor is one procedural mesh (148 triangles), the chain one MultiMesh of
## low-poly links along a catenary that is computed once (it is redrawn only while swinging). No rigid bodies,
## but one static collider (layer 1): capsules along the chain and a sphere at the anchor, so the tether rests
## on the chain and the ROV bumps into it.

@export var scope := 3.0  ## horizontal distance to the anchor, in water depths
@export var max_depth := 60.0  ## m; no seabed found within this: the anchor is laid this deep
@export var fairlead := Vector3(0.0, 1.0, -2.88)  ## bow roller, boat frame
@export var link_length := 0.22  ## m, one chain link (drawn alternately rotated 90 degrees)
@export var max_links := 400
@export var chain_on_seabed := 0.3  ## share of the horizontal run lying on the seabed before it lifts
@export var swing_degrees := 0.0  ## yaw swing round the anchor; 0 = the boat holds its heading too
@export var swing_period := 90.0  ## s
@export var anchor_scale := 1.0
@export var chain_collision := true  ## static capsules along the chain and a sphere at the anchor
@export var chain_capsules := 10
@export var chain_radius := 0.03  ## m
@export var anchor_radius := 0.35  ## m

var anchor_point := Vector3.INF  ## where the anchor lies (world)
var _boat: Node3D
var _base_offset := Vector3.ZERO  ## boat origin relative to the anchor, horizontal
var _base_basis: Basis
var _chain: MultiMeshInstance3D
var _anchor_mesh: MeshInstance3D
var _collider: StaticBody3D
var _capsules: Array[CollisionShape3D] = []
var _anchor_shape: CollisionShape3D
var _time := 0.0
var _depth := 0.0


func _ready():
	_boat = get_parent() as Node3D
	top_level = true  # the anchor and chain are placed in world space
	global_transform = Transform3D.IDENTITY
	# after boat_float.gd has floated the boat (it is the parent: its _ready ran first)
	_lay_anchor.call_deferred()


func _lay_anchor():
	var surface := _boat.global_position.y
	var globals = get_node_or_null("/root/Globals")
	if globals:
		surface = globals.surface_altitude
	var fwd := -_boat.global_basis.z
	fwd.y = 0
	fwd = fwd.normalized() if fwd.length() > 1e-3 else Vector3.FORWARD
	var bow := _boat.global_transform * fairlead
	# the depth under the bow first, then the seabed at scope x depth ahead
	var d0 := _seabed_depth(Vector3(bow.x, surface, bow.z), surface)
	var ahead := Vector3(bow.x, surface, bow.z) + fwd * (scope * d0)
	_depth = _seabed_depth(ahead, surface)
	anchor_point = Vector3(ahead.x, surface - _depth, ahead.z)
	_base_offset = _boat.global_position - anchor_point
	_base_offset.y = 0
	_base_basis = _boat.global_basis
	_build_anchor_mesh()
	_build_chain()
	if chain_collision:
		_build_collider()
	_place()


func _seabed_depth(at: Vector3, surface: float) -> float:
	var space := get_world_3d().direct_space_state
	var q := PhysicsRayQueryParameters3D.create(at + Vector3.UP * 0.5, at + Vector3.DOWN * max_depth)
	var excl := [_boat.get_rid()] if _boat is CollisionObject3D else []
	for v in get_tree().get_nodes_in_group("vehicles"):
		if v is CollisionObject3D:
			excl.append(v.get_rid())
	if _collider:
		excl.append(_collider.get_rid())
	q.exclude = excl
	var hit := space.intersect_ray(q)
	return clampf(surface - hit.position.y, 1.0, max_depth) if not hit.is_empty() else max_depth


func _physics_process(delta):
	if not anchor_point.is_finite():
		return
	_time += delta
	# boat_float.gd (the parent, so earlier this tick) set heave/pitch/roll about its fixed heading; this pins the
	# horizontal position, and swings it round the anchor if asked
	var yaw := deg_to_rad(swing_degrees) * sin(TAU * _time / swing_period) if swing_degrees > 0.0 else 0.0
	var rot := Basis(Vector3.UP, yaw)
	var p := anchor_point + rot * _base_offset
	var b := _boat.global_transform
	if yaw != 0.0:
		b.basis = rot * b.basis
	b.origin.x = p.x
	b.origin.z = p.z
	_boat.global_transform = b
	if swing_degrees > 0.0 and Engine.get_physics_frames() % 6 == 0:
		_place()


func _place():
	# the chain's catenary from the bow roller to the anchor
	var top := _boat.global_transform * fairlead
	var bot := anchor_point
	var hdir := Vector3(top.x - bot.x, 0, top.z - bot.z)
	var span := hdir.length()
	hdir = hdir / span if span > 1e-3 else Vector3.FORWARD
	var lying := span * clampf(chain_on_seabed, 0.0, 0.9)
	var X := span - lying  # horizontal run of the hanging part
	var H := top.y - bot.y
	# y = a (cosh(x / a) - 1) through (X, H): bisection on a
	var lo := 0.01
	var hi := 1000.0
	for k in range(60):
		var am := 0.5 * (lo + hi)
		if am * (cosh(X / am) - 1.0) > H:
			lo = am
		else:
			hi = am
	var a := 0.5 * (lo + hi)
	var hang_len := a * sinh(X / a)
	var total := lying + hang_len
	var n := mini(int(total / link_length), max_links)
	var mm := _chain.multimesh
	mm.instance_count = n
	var step: float = total / maxi(n, 1)
	for i in range(n):
		var s := (i + 0.5) * step
		var pos := _chain_at(s, bot, hdir, lying, a)
		# slope dy/dx = sinh(x / a) = (s - lying) / a on the hanging part
		var tan := hdir if s < lying else (hdir + Vector3.UP * ((s - lying) / a)).normalized()
		var side := tan.cross(Vector3.UP)
		side = side.normalized() if side.length() > 1e-3 else Vector3.RIGHT
		var up := side.cross(tan)
		var bas := Basis(side, up, tan) if i % 2 == 0 else Basis(up, -side, tan)
		mm.set_instance_transform(i, Transform3D(bas, pos))
	# the anchor lies on the seabed, its shank along the chain's lead
	var y := hdir
	_anchor_mesh.global_transform = Transform3D(Basis(Vector3.UP.cross(y).normalized(), Vector3.UP, -y).scaled(Vector3.ONE * anchor_scale), bot)
	if _collider:
		# capsules over equal arc-length slices, local Y along each slice's chord (this node is at the world origin)
		var nc := _capsules.size()
		for i in range(nc):
			var p0 := _chain_at(total * i / nc, bot, hdir, lying, a)
			var p1 := _chain_at(total * (i + 1) / nc, bot, hdir, lying, a)
			var cy := p1 - p0
			var l := cy.length()
			cy = cy / l if l > 1e-4 else Vector3.UP
			var cx := cy.cross(Vector3.UP)
			cx = cx.normalized() if cx.length() > 1e-3 else Vector3.RIGHT
			var cs := _capsules[i]
			(cs.shape as CapsuleShape3D).height = l + 2.0 * chain_radius
			cs.transform = Transform3D(Basis(cx, cy, cx.cross(cy)), 0.5 * (p0 + p1))
		# the sphere round the shank and crown, half sunk in the seabed
		_anchor_shape.position = bot + hdir * (0.45 * anchor_scale) + Vector3.UP * (anchor_radius * 0.5)


func _chain_at(s: float, bot: Vector3, hdir: Vector3, lying: float, a: float) -> Vector3:
	# arc length s from the anchor -> point on the chain (lying on the seabed, then the catenary)
	if s < lying:
		return bot + hdir * s + Vector3.UP * 0.03
	var x := a * asinh((s - lying) / a)
	return bot + hdir * (lying + x) + Vector3.UP * (a * (cosh(x / a) - 1.0) + 0.03)


func _build_collider():
	_collider = StaticBody3D.new()
	_collider.name = "ChainCollider"
	_collider.collision_layer = 1
	_collider.collision_mask = 0  # static: it only needs to be hit
	for i in range(maxi(chain_capsules, 1)):
		var cap := CapsuleShape3D.new()
		cap.radius = chain_radius
		var cs := CollisionShape3D.new()
		cs.shape = cap
		_collider.add_child(cs)
		_capsules.append(cs)
	var sph := SphereShape3D.new()
	sph.radius = anchor_radius
	_anchor_shape = CollisionShape3D.new()
	_anchor_shape.shape = sph
	_collider.add_child(_anchor_shape)
	add_child(_collider)


func _metal(c: Color) -> StandardMaterial3D:
	var m := StandardMaterial3D.new()
	m.albedo_color = c
	m.metallic = 0.7
	m.roughness = 0.55
	m.cull_mode = BaseMaterial3D.CULL_DISABLED  # the procedural faces aren't all wound the same way
	return m


func _build_chain():
	_chain = MultiMeshInstance3D.new()
	_chain.name = "Chain"
	_chain.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	var link := TorusMesh.new()  # an oval would be closer; a squashed low-poly torus reads as chain at these sizes
	link.inner_radius = link_length * 0.22
	link.outer_radius = link_length * 0.36
	link.rings = 6
	link.ring_segments = 4  # 48 triangles a link
	var mm := MultiMesh.new()
	mm.transform_format = MultiMesh.TRANSFORM_3D
	mm.mesh = link
	_chain.multimesh = mm
	_chain.material_override = _metal(Color(0.22, 0.22, 0.23))
	# the torus lies in its XZ plane: stretch it along the chain (Z) into a link
	link.material = null
	_chain.multimesh.mesh = _stretched(link)
	add_child(_chain)


func _stretched(m: Mesh) -> ArrayMesh:
	var arr := m.surface_get_arrays(0)
	var v: PackedVector3Array = arr[Mesh.ARRAY_VERTEX]
	for i in range(v.size()):
		v[i] = Vector3(v[i].x * 0.7, v[i].y * 0.6, v[i].z * 1.7)  # long enough to interlock
	arr[Mesh.ARRAY_VERTEX] = v
	var out := ArrayMesh.new()
	out.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arr)
	return out


func _build_anchor_mesh():
	# a stockless (navy) anchor, ~1 m: shank, crown, two flukes and a shackle ring; anchor frame: y up, -z along
	# the shank towards the ring (the chain), origin at the crown on the seabed
	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	_box(st, Vector3(0, 0.07, -0.45), Vector3(0.05, 0.05, 0.45))  # shank
	_box(st, Vector3(0, 0.07, 0.02), Vector3(0.22, 0.07, 0.07))  # crown
	for sx in [-1.0, 1.0]:
		# fluke: a wedge from the crown, angled forward and down into the sand
		var a := Vector3(0.18 * sx, 0.1, 0.0)
		var b := Vector3(0.3 * sx, 0.02, -0.38)
		var c := Vector3(0.08 * sx, 0.02, -0.3)
		var a2 := a + Vector3(0, -0.06, 0)
		_tri(st, a, b, c)
		_tri(st, a2, c + Vector3(0, -0.02, 0), b + Vector3(0, -0.02, 0))
		_quad(st, a, a2, b + Vector3(0, -0.02, 0), b)
		_quad(st, b, b + Vector3(0, -0.02, 0), c + Vector3(0, -0.02, 0), c)
		_quad(st, c, c + Vector3(0, -0.02, 0), a2, a)
	_box(st, Vector3(0, 0.07, -0.93), Vector3(0.015, 0.06, 0.04))  # shackle
	# ring: a few segments of a torus
	var segs := 10
	for i in range(segs):
		var t0 := TAU * i / segs
		var t1 := TAU * (i + 1) / segs
		var c0 := Vector3(0, 0.07 + 0.08 * sin(t0), -1.02 - 0.08 * cos(t0))
		var c1 := Vector3(0, 0.07 + 0.08 * sin(t1), -1.02 - 0.08 * cos(t1))
		_box_between(st, c0, c1, 0.015)
	st.generate_normals()
	var mesh := st.commit()
	_anchor_mesh = MeshInstance3D.new()
	_anchor_mesh.name = "Anchor"
	_anchor_mesh.mesh = mesh
	_anchor_mesh.material_override = _metal(Color(0.16, 0.17, 0.18))
	add_child(_anchor_mesh)


func _tri(st: SurfaceTool, a: Vector3, b: Vector3, c: Vector3):
	st.add_vertex(a)
	st.add_vertex(b)
	st.add_vertex(c)


func _quad(st: SurfaceTool, a: Vector3, b: Vector3, c: Vector3, d: Vector3):
	_tri(st, a, b, c)
	_tri(st, a, c, d)


func _box(st: SurfaceTool, c: Vector3, h: Vector3):
	var p := []
	for i in range(8):
		p.append(c + Vector3(h.x if i & 1 else -h.x, h.y if i & 2 else -h.y, h.z if i & 4 else -h.z))
	for f in [[0, 1, 3, 2], [4, 6, 7, 5], [0, 4, 5, 1], [2, 3, 7, 6], [0, 2, 6, 4], [1, 5, 7, 3]]:
		_quad(st, p[f[0]], p[f[1]], p[f[2]], p[f[3]])


func _box_between(st: SurfaceTool, a: Vector3, b: Vector3, r: float):
	var z := (b - a).normalized()
	var x := z.cross(Vector3.RIGHT if absf(z.x) < 0.9 else Vector3.UP).normalized()
	var y := z.cross(x)
	var p := []
	for i in range(4):
		var o := x * (r if i & 1 else -r) + y * (r if i & 2 else -r)
		p.append(a + o)
		p.append(b + o)
	for f in [[0, 1, 3, 2], [4, 6, 7, 5], [0, 4, 5, 1], [2, 3, 7, 6]]:
		_quad(st, p[f[0]], p[f[1]], p[f[2]], p[f[3]])
