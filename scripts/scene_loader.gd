extends Node

# RefCounted: https://docs.godotengine.org/en/3.0/tutorials/io/background_loading.html#doc-background-loading

var current_scene = null
var loader = null
var time_max = 1000/16 # ms
# Control loading scene
var wait_frames = 0

var current_path = ""

func show_error():
	print("Error while loading scene")

func _ready():
	var root = get_tree().get_root()
	current_scene = root.get_child(root.get_child_count() -1)

func goto_scene(path):
	print("going to scene", path)
	current_path = path
	ResourceLoader.load(path)

	set_process(true)

	# get rid of the old scene
	current_scene.queue_free()

	# get_tree().change_scene_to_file("res://scenery/misc/loading/loading.tscn")
	var resource = load(current_path)
	set_new_scene(resource)

func _process(_time):
	pass

func update_progress():
	#var progress = float(ResourceLoader.get_stage()) / loader.get_stage_count()
	#var loading_scene = get_tree().get_current_scene()
	#loading_scene.get_node("Label").text = "Loading.. (%d %%)" % (progress * 100)
	pass

func set_new_scene(scene_resource):
	current_scene = scene_resource.instantiate()
	get_node("/root").add_child(current_scene)
