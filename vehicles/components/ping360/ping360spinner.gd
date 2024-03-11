extends Camera3D

var angle: float = 0
var image = null
var speed = 0.9
# Called when the node enters the scene tree for the first time.
func _ready():
	self.get_parent()

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	self.global_transform = $"../..".global_transform
	self.angle = self.angle + deg_to_rad(self.fov)*0.9
	self.transform = self.transform.rotated_local(Vector3(0,1,0), angle + PI)
	if self.angle > 2*PI:
		self.angle = self.angle - 2*PI
