extends HingeJoint3D


@export var targetAngle: float = -0.28

var starting_angle = 0
var kp = 4
var base
var claw
var angle := 0.0  # rad from the scene's (closed) pose, measured every frame
# Called when the node enters the scene tree for the first time.
func _ready():
	# Godot takes an impulse limit; the claw motors were tuned for 50 N*m of torque
	set_param(PARAM_MOTOR_MAX_IMPULSE, 50.0 / Engine.physics_ticks_per_second)
	base = get_node(self.node_a)
	claw = get_node(self.node_b)
	starting_angle = base.transform.basis.x.signed_angle_to(claw.transform.basis.x,base.transform.basis.y)

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	# calculate the angle between node_a and node_b at every frame:

	angle = base.transform.basis.x.signed_angle_to(claw.transform.basis.x,base.transform.basis.y) - starting_angle
	var output = kp* (targetAngle - angle)
	set_param(PARAM_MOTOR_TARGET_VELOCITY, -output) 

func open():
	targetAngle = min(targetAngle + 0.01, 0.75)

func close():
	targetAngle = max(targetAngle - 0.01, 0)

func is_closed() -> bool:
	# the close command has run to its end and the claw got there
	return targetAngle == 0 and absf(angle) < 0.1
