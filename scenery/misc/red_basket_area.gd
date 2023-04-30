extends Node3D

var count = 0

# Called when the node enters the scene tree for the first time.
func _ready():
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	pass

func update_text():
	$countMesh.mesh.text = str(count) + "/3"

func increase():
	count += 1
	
func decrease():
	count -= 1

func _on_area_3d_body_entered(body):
	if body.is_in_group("red_buoy"):
		self.increase()
		self.update_text()


func _on_area_3d_body_exited(body):
	if body.is_in_group("red_buoy"):
		self.decrease()
		self.update_text()
