tool

extends ImmediateGeometry
var m = SpatialMaterial.new()

export var thickness = 0.01
export var height = 0.1
export var thrust = 1

export (bool) var roll_factor = true
export (bool) var pitch_factor = true
export (bool) var yaw_factor = true

export (bool) var vertical_factor = true
export (bool) var forward_factor = true
export (bool) var lateral_factor = true

func get_class():
	return "Thruster"

func _ready():
	if Engine.is_editor_hint():
		set_process(true)
		m.flags_unshaded = true
		m.flags_use_point_size = true
		m.albedo_color = Color(1.0, 1.0, 1.0, 1.0)
		m.depth_enabled = false
	else:
		set_process(false)

func _process(_delta):
	m.albedo_color = Color(1.0, 1.0, 1.0, 1.0)
	m.flags_no_depth_test = true
	m.render_priority = m.RENDER_PRIORITY_MAX
	set_material_override(m)

	clear()

	# Begin draw.
	begin(Mesh.PRIMITIVE_LINES)

	# base
	add_vertex(Vector3(0, -thickness, -thickness))
	add_vertex(Vector3(0, -thickness, thickness))

	add_vertex(Vector3(0, -thickness, thickness))
	add_vertex(Vector3(0, thickness, thickness))
	
	add_vertex(Vector3(0, thickness, thickness))
	add_vertex(Vector3(0, thickness, -thickness))
	
	add_vertex(Vector3(0, thickness, -thickness))
	add_vertex(Vector3(0, -thickness, -thickness))

	# walls
	add_vertex(Vector3(0, -thickness, -thickness))
	add_vertex(Vector3(height, -thickness, -thickness))

	add_vertex(Vector3(0, -thickness, thickness))
	add_vertex(Vector3(height, -thickness, thickness))
	
	add_vertex(Vector3(0, thickness, thickness))
	add_vertex(Vector3(height, thickness, thickness))
	
	add_vertex(Vector3(0, thickness, -thickness))
	add_vertex(Vector3(height, thickness, -thickness))

	# Hat
	add_vertex(Vector3(0.9 * height, 1.3*-thickness, 1.3*-thickness))
	add_vertex(Vector3(1.5*height, 0, 0))
	
	add_vertex(Vector3(0.9 * height, 1.3*thickness, 1.3*-thickness))
	add_vertex(Vector3(1.5*height, 0, 0))
	
	add_vertex(Vector3(0.9 * height, 1.3*thickness, 1.3*thickness))
	add_vertex(Vector3(1.5*height, 0, 0))
	
	add_vertex(Vector3(0.9 * height, 1.3*-thickness, 1.3*thickness))
	add_vertex(Vector3(1.5*height, 0, 0))

	add_vertex(Vector3(0.9 * height, 1.3*-thickness, 1.3*-thickness))
	add_vertex(Vector3(0.9 * height, 1.3*thickness, 1.3*-thickness))
	
	add_vertex(Vector3(0.9 * height, 1.3*thickness, 1.3*-thickness))
	add_vertex(Vector3(0.9 * height, 1.3*thickness, 1.3*thickness))
	
	add_vertex(Vector3(0.9 * height, 1.3*thickness, 1.3*thickness))
	add_vertex(Vector3(0.9 * height, 1.3*-thickness, 1.3*thickness))
	
	add_vertex(Vector3(0.9 * height, 1.3*-thickness, 1.3*thickness))
	add_vertex(Vector3(0.9 * height, 1.3*-thickness, 1.3*-thickness))
	# End drawing.
	end()

