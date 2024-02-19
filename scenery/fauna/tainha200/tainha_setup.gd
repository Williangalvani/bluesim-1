@tool
extends Node3D
@export var points: int = 10
@export var max_angle: float = 90

var raycast = preload("res://scenery/fauna/tainha200/raycast.tscn")

# Called when the node enters the scene tree for the first time.
func _ready():
	$AnimationPlayer.play("Animation")

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	pass
