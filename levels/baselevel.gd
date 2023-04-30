extends Control

func _ready():
	add_child(load(Globals.active_level).instantiate())



func _on_streaming_toggle_toggled(button_pressed):
	Globals.streaming = button_pressed
		
