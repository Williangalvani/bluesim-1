@tool
extends Node

const SECTION = preload("res://vehicles/tether/section.tscn")
const LINK = preload("res://vehicles/tether/joint.tscn")

var section_length = Vector3(-0.145,0,0)
@export var straight_loops = 1
@export var coiled_loops = 20
var vehicle

func _ready():
	var parent = get_tree().get_first_node_in_group("vehicles")
	for i in range (straight_loops):
		var child = addSection(parent, i)
		addLink(parent, child, i)
		parent = child
		
	#for i in range (coiled_loops):
		#var child = addSection(parent, i, i * 90)
		#addLink(parent, child, i)
		#parent = child
		#
func addSection(_parent, i, _angle = 0):
	var section = SECTION.instantiate()
	add_child(section)
	section.global_position = self.global_position + section_length * (i+0.5)
	for child in section.get_children():
		child.transform.origin = Vector3(0,0,0)
	return section
	
func addLink(parent, child, i):

	var pin = LINK.instantiate()
	parent.add_child(pin)
	pin.global_position = self.global_position + section_length * i
	pin.set_node_a(parent.get_path())
	pin.set_node_b(child.get_path())
	#pin.set_solver_priority(i+1)
