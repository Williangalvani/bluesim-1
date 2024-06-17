extends Control

var ping = null
var previous_frame: Image = null
# Called when the node enters the scene tree for the first time.
func _ready():
	pass # Replace with function body.
	self.ping = get_tree().get_first_node_in_group("ping360viewport")


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(_delta):
	self.previous_frame = self.get_viewport().get_texture().get_image()
	if self.previous_frame:
		self.material.set_shader_parameter("previous_image", ImageTexture.create_from_image(self.previous_frame))
	self.material.set_shader_parameter("depth_map", self.ping.image)
	self.material.set_shader_parameter("camera_angle", self.ping.angle)
	self.material.set_shader_parameter("size", self.ping.size.x)
	self.material.set_shader_parameter("fov", self.ping.fov)
	
