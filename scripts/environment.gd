extends Node3D
const BUOYANCY = 20.0  # newtons?
const HEIGHT = 2.4  # TODO: get this programatically
var underwater_env = load("res://scenery/underwaterEnvironment.tres")
var surface_env = load("res://scenery/defaultEnvironment.tres")
# darkest it gets
@onready var cameras = get_tree().get_nodes_in_group("cameras")
@onready var surface_altitude = 3.53

var fancy_water
#@onready var fancy_underwater = $water.get_surface_override_material(0)
#const simple_water = preload("res://assets/maujoe.basic_water_material/materials/basic_water_material.material")

@onready var depth = 0
@onready var last_depth = 0
# Called when the node enters the scene tree for the first time.
func _ready():
	set_physics_process(true)
	update_fog()
	_on_fancy_water_toggle_toggled(false)


func calculate_buoyancy_and_ballast():
	var vehicles = get_tree().get_nodes_in_group("buoyant")
	for vehicle in vehicles:
		if not vehicle is RigidBody3D:
			push_warning("Component %s does not inherit RigidBody3D." % vehicle.name)
			continue

		var buoys = vehicle.find_child("buoys")
		if buoys:
			var children = buoys.get_children()
			for buoy in children:
				# print(buoy.transform.origin)
				var buoyancy = (
					vehicle.buoyancy
					* (surface_altitude - buoy.global_transform.origin.y)
					/ children.size()
				)
				if buoy.global_transform.origin.y > surface_altitude:
					buoyancy = 0
				vehicle.add_force_local_pos(Vector3(0, buoyancy, 0), buoy.transform.origin)
		else:
			var buoyancy = vehicle.buoyancy
			if vehicle.global_transform.origin.y > surface_altitude:
				buoyancy = 0
			vehicle.apply_force(Vector3(0, buoyancy, 0),vehicle.transform.basis.y * +0.3)
		var ballasts = vehicle.find_child("ballasts")
		if ballasts:
			var children = ballasts.get_children()
			for ballast in children:
				vehicle.add_force_local_pos(
					Vector3(0, -vehicle.ballast_kg * vehicle.gravity_scale, 0), ballast.transform.origin
				)


func update_fog():
	var vehicles = get_tree().get_nodes_in_group("vehicles")
	for vehicle in vehicles:
		if not vehicle is RigidBody3D:
			push_warning("Component %s does not inherit RigidBody3D." % vehicle.name)
			continue
		var rov_camera = get_node(str(vehicle.get_path()) + "/Camera3D")
		depth = rov_camera.global_transform.origin.y - surface_altitude
		last_depth = depth

		#var fog_distance = max(50 + 1 * depth, 20)
		#underwater_env.fog = fog_distance
		var deep_factor = min(max(-depth / 50, 0), 1.0)
		Globals.deep_factor = deep_factor
		var new_color = Globals.surface_ambient.lerp(
			Globals.deep_ambient, deep_factor
		)
		#Globals.current_ambient = new_color.darkened(0.5)
		#underwater_env.background_color = new_color
		#underwater_env.background_sky.sky_horizon_color = new_color
		#underwater_env.background_sky.ground_bottom_color = new_color
		#underwater_env.background_sky.ground_horizon_color = new_color
		#underwater_env.fog_light_color = new_color
		#underwater_env.ambient_light_energy = 1.0 - deep_factor
		# underwater_env.ambient_light_color = new_color;
		#underwater_env.ambient_light_color = new_color  #surface_ambient.lerp(deep_ambient, max(1 - depth/50, 0))
		#$sun.light_energy = max(0.3 - 0.5 * deep_factor, 0)
		#underwater_env.background_sky.sky_energy = max(5.0 - 5 * deep_factor, 0.0)

		for camera in cameras:
			depth = camera.global_transform.origin.y - surface_altitude
			camera.environment = surface_env if depth > 0.2 else underwater_env
			if depth > 0:
				camera.cull_mask = 3
			else:
				camera.cull_mask = 5


func _process(_delta):
	update_fog()


func _physics_process(_delta):
	calculate_buoyancy_and_ballast()


func _notification(what):
	if what == Node.NOTIFICATION_WM_CLOSE_REQUEST:
		OS.kill(Globals.sitl_pid)
		get_tree().quit()


func _on_godrayToggle_toggled(button_pressed):
	$Godrays.emitting = button_pressed


func _on_dirtparticlesToggle_toggled(button_pressed):
	$SuspendedParticleHolder/SuspendedParticles.emitting = button_pressed


func _on_fancy_water_toggle_toggled(button_pressed):
	Globals.fancy_water = button_pressed
	if button_pressed:
		pass
		#$water.set_surface_override_material(0, fancy_water)
		#$underwater.set_surface_override_material(0, fancy_underwater)
	else:
		pass
		# save previous materials
		#fancy_underwater = $underwater.get_surface_override_material(0)
		#fancy_water = $water.get_surface_override_material(0)
#		$water.set_surface_override_material(0, simple_water)
#		$underwater.set_surface_override_material(0, simple_water)
