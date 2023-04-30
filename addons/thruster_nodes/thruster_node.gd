@tool

var m = StandardMaterial3D.new()

@export var thickness = 0.01
@export var height = 0.1
@export var thrust = 1

@export var roll_factor = true
@export var pitch_factor = true
@export var yaw_factor = true

@export var vertical_factor = true
@export var forward_factor = true
@export var lateral_factor = true

var last_transform
# keeps track of transform changes to update motors matrix

func get_class():
	return "Thruster"

func _ready():
	#last_transform = transform
	if Engine.is_editor_hint():
		#set_process(true)
		m.flags_unshaded = true
		m.albedo_color = Color(1.0, 1.0, 1.0, 0.8)
		m.depth_enabled = false
		m.render_priority = m.RENDER_PRIORITY_MAX
		m.flags_no_depth_test = true
		#set_material_override(m)
	else:
		pass 
		#set_process(false)

func _process(_delta):
	return
	#if last_transform != transform:
#		get_parent().calculate_motors_matrix()
#		last_transform = transform
	var mesh = ImmediateMesh.new()
	mesh.clear()

	# Begin draw.
	mesh.begin(Mesh.PRIMITIVE_LINES)

	# base
	mesh.add_vertex(Vector3(0, -thickness, -thickness))
	mesh.add_vertex(Vector3(0, -thickness, thickness))

	mesh.add_vertex(Vector3(0, -thickness, thickness))
	mesh.add_vertex(Vector3(0, thickness, thickness))
	
	mesh.add_vertex(Vector3(0, thickness, thickness))
	mesh.add_vertex(Vector3(0, thickness, -thickness))
	
	mesh.add_vertex(Vector3(0, thickness, -thickness))
	mesh.add_vertex(Vector3(0, -thickness, -thickness))

	# walls
	mesh.add_vertex(Vector3(0, -thickness, -thickness))
	mesh.add_vertex(Vector3(height, -thickness, -thickness))

	mesh.add_vertex(Vector3(0, -thickness, thickness))
	mesh.add_vertex(Vector3(height, -thickness, thickness))
	
	mesh.add_vertex(Vector3(0, thickness, thickness))
	mesh.add_vertex(Vector3(height, thickness, thickness))
	
	mesh.add_vertex(Vector3(0, thickness, -thickness))
	mesh.add_vertex(Vector3(height, thickness, -thickness))

	# Hat
	mesh.add_vertex(Vector3(0.9 * height, 1.3*-thickness, 1.3*-thickness))
	mesh.add_vertex(Vector3(1.5*height, 0, 0))
	
	mesh.add_vertex(Vector3(0.9 * height, 1.3*thickness, 1.3*-thickness))
	mesh.add_vertex(Vector3(1.5*height, 0, 0))
	
	mesh.add_vertex(Vector3(0.9 * height, 1.3*thickness, 1.3*thickness))
	mesh.add_vertex(Vector3(1.5*height, 0, 0))
	
	mesh.add_vertex(Vector3(0.9 * height, 1.3*-thickness, 1.3*thickness))
	mesh.add_vertex(Vector3(1.5*height, 0, 0))

	mesh.add_vertex(Vector3(0.9 * height, 1.3*-thickness, 1.3*-thickness))
	mesh.add_vertex(Vector3(0.9 * height, 1.3*thickness, 1.3*-thickness))
	
	mesh.add_vertex(Vector3(0.9 * height, 1.3*thickness, 1.3*-thickness))
	mesh.add_vertex(Vector3(0.9 * height, 1.3*thickness, 1.3*thickness))
	
	mesh.add_vertex(Vector3(0.9 * height, 1.3*thickness, 1.3*thickness))
	mesh.add_vertex(Vector3(0.9 * height, 1.3*-thickness, 1.3*thickness))
	
	mesh.add_vertex(Vector3(0.9 * height, 1.3*-thickness, 1.3*thickness))
	mesh.add_vertex(Vector3(0.9 * height, 1.3*-thickness, 1.3*-thickness))
	# End drawing.
	mesh.end()

