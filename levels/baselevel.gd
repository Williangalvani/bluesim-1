extends Control

func _ready():
	# the water is BlueSim's, not the level's, so custom levels get buoyancy too
	var environment = Node3D.new()
	environment.set_script(preload("res://scripts/environment.gd"))
	add_child(environment)
	add_child(load(Globals.active_level).instantiate())
