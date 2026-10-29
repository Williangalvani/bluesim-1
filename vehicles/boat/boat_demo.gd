extends Node
## Demo: the sea level (levels/baselevel.tscn booted with the reef pack, like the web build) with
## the support boat on the surface, its stern towards the ROV so the tether can run from the boat's
## TetherAnchor. Native runs only need this scene; `-- --boat-shot=/path/prefix` saves screenshots
## and quits.

const BOAT = preload("res://vehicles/boat/boat.tscn")
const CameraWaterline = preload("res://scripts/camera_waterline.gd")

## in the world of levels/baselevel.tscn; the ROV starts at (0, 3.41, 0) and y is set by the boat's
## float script
@export var boat_position := Vector3(-4.0, 0.0, -10.0)
@export var boat_yaw_degrees := 22.0

var boat: Node3D


func _enter_tree():
	if not Globals.active_level:
		ProjectSettings.load_resource_pack("res://levels/reef.pck")
		Globals.active_level = "res://custom_level.tscn"
		Globals.active_vehicle = "bluerovheavy"


func _ready():
	var base = load("res://levels/baselevel.tscn").instantiate()
	add_child(base)
	# baselevel has its boat astern of the ROV already; a level without one gets it at boat_position
	boat = base.get_node_or_null("ViewportCamera/SubViewport/Boat")
	if boat == null:
		boat = BOAT.instantiate()
		boat.position = boat_position
		boat.rotation_degrees.y = boat_yaw_degrees
		base.get_node("ViewportCamera/SubViewport").add_child(boat)
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with("--boat-shot="):
			_screenshots(arg.trim_prefix("--boat-shot="))


## The screenshot camera is a plain Camera3D, so unlike the ROV's camera
## (scripts/camera_waterline.gd) nothing swaps its environment at the surface: it would show the
## level's underwater WorldEnvironment (solid blue background, water fog, god rays) even above the
## water. Do the same swap here for each shot.
func _set_above_water(cam: Camera3D, above: bool):
	cam.environment = CameraWaterline.make_surface_environment() if above else null
	get_tree().call_group("underwater_fx", "set_fx", not above)


func _screenshots(prefix):
	var world = boat.get_parent()
	var cam = Camera3D.new()
	cam.fov = 55
	world.add_child(cam)
	await get_tree().create_timer(3.0).timeout
	var anchor = boat.get_node("TetherAnchor")
	var rov = get_tree().get_first_node_in_group("vehicles")
	var surface: float = Globals.surface_altitude
	# boat-local views; the boat bobs, so they are taken relative to it, then levelled to the water
	var t: Transform3D = boat.global_transform
	var a: Vector3 = anchor.global_position
	var side = t.basis.x.normalized()
	var views = {
		"above": [t * Vector3(4.0, 2.6, 5.5), t * Vector3(0, 0.3, 0.2)],
		"pilot": [t * Vector3(-2.3, 1.4, 0.9), t * Vector3(-0.2, 0.65, 0.2)],
		"stern": [a + t.basis * Vector3(1.4, 1.1, 3.3), a + t.basis * Vector3(-0.15, 0.25, -0.7)],
		# low along the starboard side, 0.4 m over the water, to show the waterline on the hull
		"waterline": [t.origin + side * 5.5 + Vector3(0, 0.4, 0), t.origin + Vector3(0, 0.1, 0)],
		"under": [rov.global_position + Vector3(1.5, -1.5, 2.5), t.origin - Vector3(0, 0.3, 0)],
	}
	for name in views:
		cam.position = views[name][0]
		cam.look_at(views[name][1])
		_set_above_water(cam, cam.position.y > surface)
		cam.make_current()
		for i in 6:
			await RenderingServer.frame_post_draw
		get_viewport().get_texture().get_image().save_png("%s_%s.png" % [prefix, name])
		var cam_y := snappedf(cam.position.y, 0.01)
		var above: bool = cam.position.y > surface
		print("BOAT_SHOT_VIEW %s camera_y=%s above_water=%s" % [name, cam_y, above])
	var meshes = boat.get_node("Model").find_children("*", "MeshInstance3D", true, false)
	var hull: MeshInstance3D = meshes[0]
	var aabb = hull.global_transform * hull.get_aabb()
	var at := [anchor.global_position, boat.global_position, rov.global_position]
	print("BOAT_SHOT anchor=%s boat=%s rov=%s" % at)
	var heights := [
		surface,
		snappedf(boat.global_position.y, 0.001),
		snappedf(aabb.position.y, 0.001),
		snappedf(aabb.end.y, 0.001),
		snappedf(surface - aabb.position.y, 0.001),
	]
	var fmt := "BOAT_WATERLINE surface=%s boat_origin_y=%s hull_bottom_y=%s hull_top_y=%s draft=%s"
	print(fmt % heights)
	get_tree().quit()
