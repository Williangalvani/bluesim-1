extends SubViewportContainer
# The Ping360 HUD panel: ping360.gdshader draws the scan as a sonar image around the vehicle, from
# an image with a row per degree of head angle and a column per hundredth of the range.

const WIDTH := 100
const HEIGHT := 360
# a ping further on than this from the last one is a jump of the head, not a step to fill in
const MAX_GAP := 30  # degrees

var sonar: Node = null
var data := PackedByteArray()
var img: Image
var texture: ImageTexture
var last_angle := -1
var dirty := false


func _ready() -> void:
	data.resize(WIDTH * HEIGHT)
	img = Image.create_from_data(WIDTH, HEIGHT, false, Image.FORMAT_R8, data)
	texture = ImageTexture.create_from_image(img)
	(material as ShaderMaterial).set_shader_parameter("my_array", texture)
	var sonars := get_tree().get_nodes_in_group("ping360")
	if len(sonars) == 0:
		print("unable to find ping360")
		return
	sonar = sonars[0]
	sonar.connect("updatePing360Display", on_ping360_update)


# `points` are [distance 0..99, intensity 0..1] at head angle `ping_angle` in degrees
func on_ping360_update(ping_angle: int, points: Array) -> void:
	var row := PackedByteArray()
	row.resize(WIDTH)
	for point: Array in points:
		var x := clampi(int(point[0]), 0, WIDTH - 1)
		row[x] = maxi(row[x], int(clampf(point[1], 0, 1) * 255))
	# a step of the head can be wider than a row, so the rows it swept since the last ping are filled
	var first := ping_angle
	var gap := posmod(ping_angle - last_angle, HEIGHT)
	if last_angle >= 0 and gap > 0 and gap <= MAX_GAP:
		first = last_angle + 1
	for angle in range(first, first + posmod(ping_angle - first, HEIGHT) + 1):
		var offset := posmod(angle, HEIGHT) * WIDTH
		for x in WIDTH:
			data[offset + x] = row[x]
	last_angle = ping_angle
	dirty = true


func _process(_delta: float) -> void:
	visible = Globals.ping360_enabled
	var toggle := get_node_or_null("../ViewportCamera/SettingsPanel/VBoxContainer/Ping360Toggle")
	if toggle is CheckButton and toggle.button_pressed != Globals.ping360_enabled:
		(toggle as CheckButton).set_pressed_no_signal(Globals.ping360_enabled)
	if dirty and visible:
		img.set_data(WIDTH, HEIGHT, false, Image.FORMAT_R8, data)
		texture.update(img)
		dirty = false


func _on_Ping360Toggle_toggled(button_pressed: bool) -> void:
	Globals.ping360_enabled = button_pressed
