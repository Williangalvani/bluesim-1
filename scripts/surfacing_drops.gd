class_name SurfacingDrops
extends ColorRect
## Water drops on the ROV camera's dome when it comes out of the water (BlueSim, Godot 4.7, gl_compatibility/WebGL2).
## v2.1: drops only from the burst (exact on-image test per drop), never moving upward, geometry over the on-screen part.
## Shader: the v11 Heartfelt dome rain lens ("Heartfelt" by Martijn Steinrucken / BigWings, CC BY-NC-SA 3.0,
## non-commercial, share-alike) with its drop, trail, droplet, refraction and blur look unchanged; only the timing is
## new (scripts/fx/surfacing_drops.gdshaderinc).
##
## Underwater it is hidden and not processed (no draw, no shader cost). On surfacing the dome is wet at once with a
## Heartfelt drop field as after seconds of rain (pre-warmed), then no drop is added: the drops run down and outward
## off the dome, the rest dries off by `clear_time`, then the node hides itself again. Diving clears it within
## `dive_fade_time`. Each new surfacing starts a new burst.
##
## Drive it either way:
##  * call `set_surfaced(is_above)` (e.g. from the code that already detects the camera crossing the surface; the
##    node is in group "lens_surfacing", so `get_tree().call_group("lens_surfacing", "set_surfaced", above)` works), or
##  * set `camera` and it compares the camera's height with `surface_altitude` (or Globals.surface_altitude) itself.
## Full-rect Control (inside a Container it follows the container's size): put it over the 3D view. It refracts the
## screen behind it (with mipmap blur, as Heartfelt); only if the rect itself is inside a SubViewport (blank screen
## texture on gl_compatibility) it refracts `source_viewport`'s texture instead (8-tap blur).

signal surfaced_changed(is_above: bool)
signal dried

@export_range(1.0, 20.0) var clear_time := 9.0          ## s from surfacing until the dome is dry
@export_range(0.0, 1.0) var rain_amount := 0.9           ## Heartfelt rainAmount of the burst (v11 lens 0.9)
@export_range(0.1, 5.0) var drop_speed := 2.0            ## Heartfelt drop speed (v11 lens 2)
@export_range(0.0, 3.0) var slide_speed := 1.0           ## how fast the drops run off after surfacing (1 = Heartfelt)
@export_range(0.2, 2.0) var drop_scale := 0.6            ## shader zoom, <1 = bigger drops (v11 lens 0.6)
@export_range(0.0, 1.0) var dome_strength := 1.0         ## 0 = flat glass (drops run straight down)
@export var dome_apex := Vector2(0.0, 0.9)               ## apex of the dome in centred image units (y up, image is y -0.5..0.5)
@export_range(0.0, 0.5) var dome_lens := 0.12            ## barrel distortion of the image while wet (v11 0.12)
@export_range(0.0, 1.0) var fog := 0.4                   ## Heartfelt fogged-glass blur (v11 lens 0.4), dries off too
@export_range(0.0, 1.0) var tint := 0.5                  ## Heartfelt colour drift + vignette (v11 lens 0.5)
@export_range(0.0, 2.0) var burst_time := 0.3            ## s of the sheet of water right at surfacing
@export_range(0.0, 3.0) var refraction := 1.0
@export_range(0.0, 2.0) var dive_fade_time := 0.0        ## s to clear when the camera goes back under (0 = instant)
@export var source_viewport: SubViewport                 ## refract this SubViewport's image instead of the screen
@export var camera: Camera3D                             ## optional: detect surfacing from this camera's height
@export var surface_altitude := NAN                      ## NAN: use Globals.surface_altitude (or 0 without Globals)
@export var hysteresis := 0.005                          ## m, so bobbing at the surface does not re-trigger
@export var only_when_camera_current := true             ## with `camera` set: draw only while it is the active camera
@export var debug_mask := false                          ## draw the drop mask (tests)
@export_range(0, 2) var debug_part := 0                  ## tests, with debug_mask: 1 = running drops only, 2 = trail beads only
@export var test_seed := -1                              ## tests: >= 0 gives the same bursts every run (normal and mask runs match)
@export var force_texture_mode := false                  ## tests: refract source_viewport's texture even outside a SubViewport

var surfaced := false
var age := 0.0
var _wet := false
var _fade := 0.0
var _mat := ShaderMaterial.new()
var _bursts := 0


func _ready() -> void:
	add_to_group("lens_surfacing")
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	set_anchors_preset(Control.PRESET_FULL_RECT)
	var texture_mode = source_viewport != null and (force_texture_mode or get_viewport() is SubViewport)
	_mat.shader = load("res://scripts/fx/surfacing_drops_texture.gdshader" if texture_mode else "res://scripts/fx/surfacing_drops_screen.gdshader")
	if texture_mode:
		_mat.set_shader_parameter("scene_tex", source_viewport.get_texture())
	material = _mat
	# a Container (e.g. the SubViewportContainer showing the camera) ignores anchors: follow its size instead
	var parent = get_parent()
	if parent is Container:
		parent.resized.connect(_fit_parent)
		_fit_parent()
	visible = false
	set_process(camera != null)


## true while the camera is out of the water; a false->true change starts a new burst of drops
func set_surfaced(is_above: bool) -> void:
	if is_above == surfaced:
		return
	surfaced = is_above
	if is_above:
		age = 0.0
		_fade = 1.0
		_bursts += 1
		var sd = Vector2(randf(), randf())
		if test_seed >= 0:
			var rng = RandomNumberGenerator.new(); rng.seed = test_seed * 1000 + _bursts
			sd = Vector2(rng.randf(), rng.randf())
		_mat.set_shader_parameter("seed", sd * 10.0)
		_wet = true
	elif dive_fade_time <= 0.0:
		_hide()
	set_process(true)
	surfaced_changed.emit(is_above)


func is_wet() -> bool:
	return _wet


func _height() -> float:
	var level = surface_altitude
	if is_nan(level):
		var globals = get_node_or_null("/root/Globals")
		level = globals.surface_altitude if globals else 0.0
	return camera.global_position.y - level


func _process(delta: float) -> void:
	if camera:
		var h = _height()
		if surfaced != (h > (-hysteresis if surfaced else hysteresis)):
			set_surfaced(not surfaced)
	if not _wet:
		visible = false
		set_process(camera != null)
		return
	# hidden (not drawn) while another camera is showing, but the drops keep drying
	visible = not (only_when_camera_current and camera and not camera.is_current())
	if surfaced:
		age += delta
		if age >= clear_time:
			_hide()
			dried.emit()
			return
	else:
		_fade -= delta / max(dive_fade_time, 1e-3)
		if _fade <= 0.0:
			_hide()
			return
	_mat.set_shader_parameter("age", age)
	_mat.set_shader_parameter("fade", _fade)
	_mat.set_shader_parameter("clear_time", clear_time)
	# drop geometry (image edges, dome apex at the top centre) over the part of the rect that is on screen: BlueSim's
	# ViewportCamera container is larger than the window, so its right and bottom edges are off screen
	var g := get_global_rect()
	var v := g.intersection(get_viewport().get_visible_rect())
	if v.size.x < 1.0 or v.size.y < 1.0 or size.x < 1.0 or size.y < 1.0:
		v = g
	var gs := g.size if g.size.x > 0.0 and g.size.y > 0.0 else Vector2.ONE
	_mat.set_shader_parameter("rect_size", v.size)
	_mat.set_shader_parameter("full_size", size)
	_mat.set_shader_parameter("vis_uv", Vector4((v.position.x - g.position.x) / gs.x, (v.position.y - g.position.y) / gs.y, v.size.x / gs.x, v.size.y / gs.y))
	_mat.set_shader_parameter("rain_amount", rain_amount)
	_mat.set_shader_parameter("drop_speed", drop_speed)
	_mat.set_shader_parameter("slide_speed", slide_speed)
	_mat.set_shader_parameter("fog", fog)
	_mat.set_shader_parameter("tint", tint)
	_mat.set_shader_parameter("zoom", drop_scale)
	_mat.set_shader_parameter("dome", dome_strength)
	_mat.set_shader_parameter("dome_center", dome_apex)
	_mat.set_shader_parameter("dome_lens", dome_lens)
	_mat.set_shader_parameter("burst_time", burst_time)
	_mat.set_shader_parameter("refraction", refraction)
	_mat.set_shader_parameter("debug_mask", debug_mask)
	_mat.set_shader_parameter("debug_part", debug_part)


func _fit_parent() -> void:
	position = Vector2.ZERO
	size = get_parent().size


func _hide() -> void:
	_wet = false
	visible = false
	_fade = 0.0
	set_process(camera != null)
