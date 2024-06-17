extends JoltHingeJoint3D


@export var targetAngle: float = -0.28

var starting_angle = 0
var kp = 4
var base
var claw
# Called when the node enters the scene tree for the first time.
func _ready():
	base = get_node(self.node_a)
	claw = get_node(self.node_b)
	starting_angle = base.transform.basis.x.signed_angle_to(claw.transform.basis.x,base.transform.basis.y)

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(_delta):
	# calculate the angle between node_a and node_b at every frame:

	var angle = base.transform.basis.x.signed_angle_to(claw.transform.basis.x,base.transform.basis.y) - starting_angle
	var output = kp* (targetAngle - angle)
	self.motor_target_velocity = -output 

func open():
	targetAngle = min(targetAngle + 0.01, 0.75)

func close():
	targetAngle = max(targetAngle - 0.01, 0)
