extends Node3D

signal got_pressed

var ever_depressed = false
var pressed = true
# Called when the node enters the scene tree for the first time.
func _ready():
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
#func _process(delta):
	#pass
	#

func _on_area_3d_body_entered(body):
	if body == $buttonitself and ever_depressed:
		pressed = true
		$JoltSliderJoint3D.motor_target_velocity = -0.1
		var updated_material = $buttonitself/bodymesh.get_surface_override_material(0)
		updated_material.emission_energy = 0.5
		$buttonitself/bodymesh.set_surface_override_material(0, updated_material)
		emit_signal("got_pressed")

func _on_area_3d_body_exited(body):
	if body == $buttonitself:
		pressed = false
		ever_depressed = true
