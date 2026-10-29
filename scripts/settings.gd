extends PanelContainer

# Declare member variables here. Examples:
# var a = 2
# var b = "text"


# Called when the node enters the scene tree for the first time.
func _ready():
	Globals.physics_rate = 120
	Engine.physics_ticks_per_second = Globals.physics_rate
	$VBoxContainer/physicsRate.text = 'Physics: ' + str(Globals.physics_rate) + ' Hz'
	$VBoxContainer/physicsRateSlider.value = Globals.physics_rate
	$VBoxContainer/Ping360Toggle.set_pressed_no_signal(Globals.ping360_enabled)
	if Globals.isHTML5:
		# shown inside Cockpit like a camera feed, so start without overlays
		hide()


# Called every frame. 'delta' is the elapsed time since the previous frame.
#func _process(delta):
#	pass


func _input(event):
	if event.is_action_pressed("ui_cancel"):
		if not self.is_visible():
			$VBoxContainer/Ping360Toggle.set_pressed_no_signal(Globals.ping360_enabled)
			self.show()
		else:
			self.hide()


func _on_HSlider_value_changed(value):
	Globals.physics_rate = value
	Engine.physics_ticks_per_second = value
	$VBoxContainer/physicsRate.text = 'Physics: ' + str(Globals.physics_rate) + 'Hz'


func _on_Ping360Toggle_toggled(button_pressed):
	Globals.ping360_enabled = button_pressed
