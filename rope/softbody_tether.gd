extends Node3D
# Benchmark alternative: Godot's SoftBody3D (Jolt's soft body) built from a thin cylinder, one end pinned to the
# vehicle and the other in place. The engine gives no reaction force on the vehicle, so `force` stays zero.

@export var length := 30.0
@export var segments := 40
@export var diameter := 0.0076
@export var precision := 5
@export var anchored := true

var rov: RigidBody3D
var sb: SoftBody3D
var force := Vector3.ZERO
var solve_usec := 0
var render_usec := 0
var pinned := 0
var points := 0


func _ready():
	rov = get_tree().get_first_node_in_group("vehicles")
	var start = global_position
	var away = start - rov.global_position
	away.y = 0
	away = away.normalized() if away.length() > 0.01 else Vector3(-1, 0, 0)
	var cyl = CylinderMesh.new()
	cyl.top_radius = diameter / 2
	cyl.bottom_radius = diameter / 2
	cyl.height = length
	cyl.radial_segments = 4
	cyl.rings = segments
	cyl.cap_top = false
	cyl.cap_bottom = false
	var arrays = cyl.get_mesh_arrays()
	var mesh = ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	var mat = StandardMaterial3D.new()
	mat.albedo_color = Color(0.96, 0.75, 0.04)
	mesh.surface_set_material(0, mat)
	sb = SoftBody3D.new()
	sb.mesh = mesh
	sb.simulation_precision = precision
	sb.total_mass = 0.066 * segments
	sb.linear_stiffness = 1.0
	sb.pressure_coefficient = 0.0
	sb.damping_coefficient = 0.1
	sb.ray_pickable = false
	sb.add_collision_exception_with(rov)  # pinned inside the hull, it would otherwise shove the vehicle away
	# the cylinder's +Y end goes on the vehicle, the rest trails away from it
	var bx = away.cross(Vector3.UP).normalized()
	var basis = Basis(bx, -away, bx.cross(-away))
	sb.top_level = true
	sb.transform = Transform3D(basis, start + away * length / 2)
	get_parent().add_child.call_deferred(sb)
	await get_tree().process_frame
	await get_tree().physics_frame
	var verts: PackedVector3Array = arrays[Mesh.ARRAY_VERTEX]
	points = verts.size()
	for i in range(verts.size()):
		if verts[i].y > length / 2 - 0.01:
			sb.set_point_pinned(i, true, sb.get_path_to(rov))
			pinned += 1
		elif anchored and verts[i].y < -length / 2 + 0.01:
			sb.set_point_pinned(i, true)
			pinned += 1
