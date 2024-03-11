extends SubViewport

var image = null
var angle = 0
var fov = 0.0

# Called when the node enters the scene tree for the first time.
func _ready():
	await RenderingServer.frame_post_draw


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	self.image = self.get_texture()
	self.angle = $Camera3D.angle
	self.fov = $Camera3D.fov
	
