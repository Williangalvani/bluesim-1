extends Generic6DOFJoint3D


# Called when the node enters the scene tree for the first time.
func _ready():
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	pass


func _on_button_got_pressed():
	set_param_x(PARAM_LINEAR_MOTOR_TARGET_VELOCITY, -0.3)
