extends Control

# in metres; the water fog hides anything further away, and each shadow split redraws every multimesh whole
const SUN_SHADOW_DISTANCE = 30

# the web build boots straight into this scene, so the page URL picks the level the menu would; this runs before
# the vehicle's _ready, which only connects to the autopilot once it is the active vehicle, and before Globals' _ready
func _enter_tree():
	# tether benchmark (rope/tether_bench.gd) run natively: boot the reef like the web build, with no autopilot link
	if not OS.has_feature("web") and ("tetherbench" in OS.get_cmdline_user_args() or "spoolbench" in OS.get_cmdline_user_args()) and not Globals.active_level:
		Globals.active_vehicle = "bench"
		ProjectSettings.load_resource_pack("res://levels/reef.pck")
		Globals.active_level = "res://custom_level.tscn"
		return
	if not OS.has_feature("web") or Globals.active_level:
		return
	if JavaScriptBridge.eval("new URLSearchParams(location.search).has('maptiles')"):
		get_tree().root.add_child.call_deferred(load("res://tools/map_tiles.gd").new())
		queue_free()
		return
	Globals.active_vehicle = "bluerovheavy"
	# ?level=NAME starts the custom level packed as res://levels/NAME.pck
	var level = JavaScriptBridge.eval("new URLSearchParams(location.search).get('level') ?? 'reef'")
	ProjectSettings.load_resource_pack("res://levels/" + level.get_file() + ".pck")
	Globals.active_level = "res://custom_level.tscn"

func _ready():
	if is_queued_for_deletion():
		return
	# the water is BlueSim's, not the level's, so custom levels get buoyancy too
	var environment = Node3D.new()
	environment.set_script(preload("res://scripts/environment.gd"))
	add_child(environment)
	var level = load(Globals.active_level).instantiate()
	for sun in level.find_children("*", "DirectionalLight3D", true, false):
		sun.directional_shadow_mode = DirectionalLight3D.SHADOW_ORTHOGONAL
		sun.directional_shadow_max_distance = min(sun.directional_shadow_max_distance, SUN_SHADOW_DISTANCE)
	add_child(level)
	if _tether_bench_requested():
		add_child(load("res://rope/tether_bench.gd").new())
	elif _tether_bench_requested("spoolbench"):
		add_child(load("res://rope/spool_bench.gd").new())


func _tether_bench_requested(key := "tetherbench") -> bool:
	if OS.has_feature("web"):
		return JavaScriptBridge.eval("new URLSearchParams(location.search).has('%s')" % key)
	return key in OS.get_cmdline_user_args()
