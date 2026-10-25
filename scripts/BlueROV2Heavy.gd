extends RigidBody3D

const THRUST = 50
# quadratic drag along the lateral, vertical and forward body axes, in N/(m/s)^2: a drag coefficient
# of ~0.85 over the frame's side, top and front areas, which tops out at the 1.5 m/s forward speed
# Blue Robotics quotes for the BlueROV2
const DRAG = Vector3(50, 113, 63)
# linear rotational damping around the pitch, yaw and roll body axes, in N*m/(rad/s): what Godot's
# angular_damp of 6 gave pitch and roll, and almost none for yaw, which water resists mostly quadratically
const ANGULAR_DAMP = Vector3(5.9, 1, 7.1)
# quadratic rotational drag around the same axes, in N*m/(rad/s)^2, for a full-thrust yaw rate of ~100 deg/s
const ANGULAR_DRAG = Vector3(5, 21.6, 5)
# a thruster less than a duct diameter under the surface sucks air into its propeller, losing thrust the closer it gets
const VENTILATION_DEPTH = 0.1  # m
const ThrusterBubbles = preload("res://scripts/thruster_bubbles.gd")
const CameraWaterline = preload("res://scripts/camera_waterline.gd")
# down, as a vertical thruster blows, and slightly forward so they rise back up in front of the camera
const BUBBLES_DIRECTION = Vector3(0, -1, 0.1)

var udp = PacketPeerUDP.new()  # UDP socket for fdm in (server)
var peer = false

# In browsers there is no UDP, the SITL runs in Cockpit (sitl:// connection) and talks over this channel
const WEB_CHANNEL_NAME = "cockpit-sitl-json"
var web_channel: JavaScriptObject
var web_on_message: JavaScriptObject  # must stay referenced, or the callback is freed
var web_pwm = []
var start_time = Time.get_ticks_msec()

var last_velocity = Vector3(0, 0, 0)
var calculated_acceleration = Vector3(0, 0, 0)

var buoyancy = 10* self.mass * self.gravity_scale  # Newtons
var hull_height = 0.254  # m, the frame height, over which buoyancy fades out at the surface
var _initial_position = 0
var phys_time = 0
# the tilt servo moves in 1 us steps of ~0.1 degrees, so the camera eases toward it instead of jumping
var camera_tilt = 0.0  # degrees

@onready var light_glows = [$light_glow, $light_glow2, $light_glow3, $light_glow4]
var thruster_bubbles = []
# the most air any thruster took in this physics tick, from 0 to 1
var ventilation = 0.0

@onready var ljoint = get_tree().get_root().find_child("ljoint", true, false)
@onready var rjoint = get_tree().get_root().find_child("rjoint", true, false)
@onready var wait_SITL = Globals.wait_SITL
@onready var ping1d = $ping1d

func connect_fmd_in():
	if Globals.isHTML5:
		web_channel = JavaScriptBridge.create_object("BroadcastChannel", WEB_CHANNEL_NAME)
		web_on_message = JavaScriptBridge.create_callback(_on_web_message)
		web_channel.onmessage = web_on_message
		return
	if udp.bind(9002) != OK:
		print("Failed to connect fdm_in")


func _on_web_message(args):
	var message = JSON.parse_string(str(args[0].data))
	if typeof(message) != TYPE_DICTIONARY or not message.has("pwm"):
		return
	web_pwm = message["pwm"]
	peer = true


func actuate_servos(pwms):
	for i in range(pwms.size()):
		var pwm = pwms[i]
		# ArduPilot sends 0 for outputs that are not initialised yet (e.g. while booting)
		if pwm == 0:
			continue
		actuate_servo(i, (float(pwm) - 1000) / 1000)


func get_servos():
	if Globals.isHTML5:
		# messages arrive between physics ticks, keep applying the latest outputs
		actuate_servos(web_pwm)
		return
	if udp.get_available_packet_count() == 0:
		return

	var buffer = StreamPeerBuffer.new()
	buffer.data_array = udp.get_packet()
	if buffer.data_array.size() == 0:
		return
	if not peer:
		udp.set_dest_address("127.0.0.1", udp.get_packet_port())
		peer = true

	if not udp.get_available_packet_count():
		if wait_SITL:
			udp.wait()

	var magic = buffer.get_u16()
	buffer.seek(2)
	var _framerate = buffer.get_u16()
	#print(_framerate)
	buffer.seek(4)
	var _framecount = buffer.get_u16()

	if magic != 18458:
		return
	var pwms = []
	for i in range(0, 16):
		buffer.seek(8 + i * 2)
		pwms.append(buffer.get_u16())
	actuate_servos(pwms)


func send_fdm():
	var buffer = StreamPeerBuffer.new()

	buffer.put_double((Time.get_ticks_msec() - start_time) / 1000.0)

	var _basis = transform.basis

# These are the same but mean different things, let's keep both for now
	var toNED = Basis(Vector3(-1, 0, 0), Vector3(0, 0, -1), Vector3(1, 0, 0))

	toNED = Basis(Vector3(1, 0, 0), Vector3(0, 0, -1), Vector3(0, 1, 0))

	var toFRD = Basis(Vector3(0, -1, 0), Vector3(0, 0, -1), Vector3(1, 0, 0))

	var _angular_velocity = toFRD * (angular_velocity * _basis)
	var gyro = [_angular_velocity.x, _angular_velocity.y, _angular_velocity.z]

	var _acceleration = toFRD * (calculated_acceleration * _basis)

	var accel = [_acceleration.x, _acceleration.y, _acceleration.z]

	# var orientation = toFRD * Vector3(-rotation.x, - rotation.y, -rotation.z)
	var quaternon = Basis(-_basis.z, _basis.x, _basis.y).rotated(Vector3(1, 0, 0), PI).rotated(Vector3(1, 0, 0), PI / 2).get_rotation_quaternion()

	var euler = quaternon.get_euler()
	euler = [euler.y, euler.x, euler.z]

	var _velocity = toNED * self.linear_velocity
	var velo = [_velocity.x, _velocity.y, _velocity.z]

	# the vehicle scene is placed above the water, so its local origin is not where depth starts
	var _position = toNED * (self.global_position - Vector3(0, Globals.surface_altitude, 0))
	var pos = [_position.x, _position.y, _position.z]

	var IMU_fmt = {"gyro": gyro, "accel_body": accel}
	var JSON_fmt = {
		"timestamp": phys_time,
		"imu": IMU_fmt,
		"position": pos,
		"quaternion": [quaternon.w, quaternon.x, quaternon.y, quaternon.z],
		"velocity": velo,
		"rng_1" : ping1d.get_collision_point().distance_to(ping1d.global_position)

	}
	var JSON_string = "\n" + JSON.stringify(JSON_fmt) + "\n"
	if Globals.isHTML5:
		web_channel.postMessage(JSON_string)
		return
	buffer.put_utf8_string(JSON_string)
	udp.put_packet(buffer.data_array)


func get_motors_table_entry(thruster):
	
	var thruster_vector = (thruster.transform.basis*Vector3(1,0,0)).normalized()
	var roll = Vector3(0,0,-1).cross(thruster.position).normalized().dot(thruster_vector)
	var pitch = Vector3(1,0,0).cross(thruster.position).normalized().dot(thruster_vector)
	var yaw = Vector3(0,1,0).cross(thruster.position).normalized().dot(thruster_vector)
	var forward = Vector3(0,0,-1).dot(thruster_vector)
	var lateral = Vector3(1,0,0).dot(thruster_vector)
	var vertical = Vector3(0,-1,0).dot(thruster_vector)
	if abs(roll) < 0.15 or not thruster.roll_factor:
		roll = 0
	if abs(pitch) < 0.15 or not thruster.pitch_factor:
		pitch = 0
	if abs(yaw) < 0.15 or not thruster.yaw_factor:
		yaw = 0
	if abs(vertical) < 0.15 or not thruster.vertical_factor :
		vertical = 0
	if abs(forward) < 0.15 or not thruster.forward_factor:
		forward = 0
	if abs(lateral) < 0.15 or not thruster.lateral_factor:
		lateral = 0
	return [roll, pitch, yaw, vertical, forward, lateral]

func calculate_motors_matrix():
	print("Calculated Motors Matrix:")
	var thrusters = []
	var i = 1
	for child in get_children():
		if child.get_class() ==  "Thruster":
			thrusters.append(child)
	for thruster in thrusters:
		var entry = get_motors_table_entry(thruster)
		entry.insert(0, i)
		i = i + 1
		print("add_motor_raw_6dof(AP_MOTORS_MOT_%s,\t%s,\t%s,\t%s,\t%s,\t%s,\t%s);" % entry)

func _ready():
	print("ready")
	if Engine.is_editor_hint():
		calculate_motors_matrix()
		return
	if Globals.active_vehicle == "bluerovheavy":
		print("pointing camera to rov...")
		$Camera3D.set_current(true)
	set_spotlights_energy($light1.light_energy)
	for thruster in [$t5, $t6]:
		thruster_bubbles.append(ThrusterBubbles.new(thruster, BUBBLES_DIRECTION))
		add_child(thruster_bubbles[-1])
	add_child(CameraWaterline.new($Camera3D))
	$"bluerov cleaned decimated2/GPUParticles3D2".material_override.set_shader_parameter("surface_altitude", Globals.surface_altitude)
	_initial_position = get_global_transform().origin
	print("setting phyisics process...")
	set_physics_process(true)
	set_process_input(true) 
	if typeof(Globals.active_vehicle) == TYPE_STRING and Globals.active_vehicle == "bluerovheavy":
		print("setting active vehicle....")
		Globals.active_vehicle = self
	else:
		return
	print("connecting fdm...")
	connect_fmd_in()
	print("init done")
	print(is_inside_tree())


func _physics_process(delta):
	if Engine.is_editor_hint():
		return
	phys_time = phys_time + 1.0 / Globals.physics_rate
	process_keys()
	calculated_acceleration = (self.linear_velocity - last_velocity) / delta
	calculated_acceleration -= get_gravity() * gravity_scale
	last_velocity = self.linear_velocity
	apply_water_drag()
	get_servos()
	for bubbles in thruster_bubbles:
		bubbles.blow(ventilation)
	ventilation = 0.0
	$Camera3D.rotation_degrees.x = 0.8 * $Camera3D.rotation_degrees.x + 0.2 * camera_tilt
	if peer:
		send_fdm()



func add_force_local(force: Vector3, pos: Vector3):
	var pos_local = self.transform.basis * pos
	var force_local = self.transform.basis * force
	self.apply_force(force_local, pos_local)

func add_force_local_if_underwater(force: Vector3, pos: Vector3):
	if (self.global_position + self.global_basis * pos).y > Globals.surface_altitude:
		return
	var pos_local = self.transform.basis * pos
	var force_local = self.transform.basis * force
	self.apply_force(force_local, pos_local)


func apply_thrust(thruster: Node3D, force: float):
	var immersion = clamp((Globals.surface_altitude - thruster.global_position.y) / VENTILATION_DEPTH, 0, 1)
	# the thruster nodes are scaled to fit their meshes, which must not scale the force too
	self.add_force_local(thruster.transform.basis.x.normalized() * force * immersion, thruster.position)
	# out of the water there is nothing to carry the air off as bubbles, and from half throttle on a ventilating
	# propeller is already churning in as much air as it can
	if immersion > 0:
		ventilation = max(ventilation, min(abs(force) / THRUST * 2, 1) * (1 - immersion))


func apply_water_drag():
	var body_velocity = self.linear_velocity * self.global_basis
	var drag = -DRAG * body_velocity * body_velocity.abs()
	self.apply_central_force(self.global_basis * drag)
	var body_angular_velocity = self.angular_velocity * self.global_basis
	var angular_drag = -ANGULAR_DAMP * body_angular_velocity - ANGULAR_DRAG * body_angular_velocity * body_angular_velocity.abs()
	self.apply_torque(self.global_basis * angular_drag)


func set_spotlights_energy(energy):
	for light in [$light1, $light2, $light3, $light4]:
		light.light_energy = energy
		# an unlit light still renders its shadow map every frame
		if light.shadow_enabled != (energy > 0):
			light.shadow_enabled = energy > 0


func actuate_servo(id, percentage):
	if percentage == 0:
		return

	# ArduSub drives the thrusters over 1100-1900 us, so full thrust is 0.4 away from neutral, not 0.5
	var force = clamp((percentage - 0.5) / 0.4, -1, 1) * -THRUST
	match id:
		0:
			apply_thrust($t1, force)
		1:
			apply_thrust($t2, force)
		2:
			apply_thrust($t3, force)
		3:
			apply_thrust($t4, force)
		4:
			apply_thrust($t5, force)
		5:
			apply_thrust($t6, force)
		6:
			apply_thrust($t7, force)
		7:
			apply_thrust($t8, force)
		14:
			# ArduSub's servo range, 1100-1900 us, spans the ±45 degrees its SITL mount parameters give the tilt
			camera_tilt = clamp((percentage - 0.5) / 0.4, -1, 1) * 45
		12:
			percentage -= 0.1
			percentage = max(0, percentage)
			set_spotlights_energy(percentage * 5)
			$scatterlight.light_energy = percentage * 0.025
			if percentage < 0.01 and light_glows[0].get_parent() != null:
				for light in light_glows:
					self.remove_child(light)
			elif percentage > 0.01 and light_glows[0].get_parent() == null:
				for light in light_glows:
					self.add_child(light)

		8:
			if percentage < 0.4:
				ljoint.close()
				rjoint.close()
			elif percentage > 0.6:
				ljoint.open()
				rjoint.open()

func _unhandled_input(event):
	if event is InputEventKey:
		# There are for debugging:
		# Some forces:
		if event.pressed and event.keycode == KEY_X:
			self.apply_central_force(Vector3(3000, 0, 0))
		if event.pressed and event.keycode == KEY_Y:
			self.apply_central_force(Vector3(0, 3000, 0))
		if event.pressed and event.keycode == KEY_Z:
			self.apply_central_force(Vector3(0, 0, 30))
		# Reset position
		if event.pressed and event.keycode == KEY_R:
			print("resetting")
			set_position(_initial_position)
		# Some torques
		if event.pressed and event.keycode == KEY_Q:
			self.apply_torque(self.transform.basis * Vector3(THRUST, 0, 0))
		if event.pressed and event.keycode == KEY_T:
			self.apply_torque(self.transform.basis * Vector3(0, THRUST, 0))
		if event.pressed and event.keycode == KEY_E:
			self.apply_torque(self.transform.basis * Vector3(0, 0, THRUST))
		# Some hard-coded positions (used to check accelerometer)
		if event.pressed and event.keycode == KEY_U:
			self.look_at(Vector3(0, 100, 0), Vector3(0, 0, 1))  # expects +X
			# mode = RigidBody3D.FREEZE_MODE_STATIC
		if event.pressed and event.keycode == KEY_I:
			self.look_at(Vector3(100, 0, 0), Vector3(0, 100, 0))  #expects +Z
			# mode = RigidBody3D.FREEZE_MODE_STATIC
		if event.pressed and event.keycode == KEY_O:
			self.look_at(Vector3(100, 0, 0), Vector3(0, 0, -100))  #expects +Y
			# mode = RigidBody3D.FREEZE_MODE_STATIC

		if event.pressed and event.is_action("camera_switch"):
			if $Camera3D.is_current():
				$Camera3D.clear_current(true)
			else:
				$Camera3D.set_current(true)

	if event.is_action("lights_up"):
		var percentage = min(max(0, $light1.light_energy + 0.1), 1)
		print("lights to", percentage)
		if percentage > 0:
			for light in light_glows:
				self.add_child(light)
		set_spotlights_energy(percentage)
		$scatterlight.light_energy = percentage * 0.00005

	if event.is_action("lights_down"):
		var percentage = min(max(0, $light1.light_energy - 0.1), 5)
		print("lights to", percentage)
		set_spotlights_energy(percentage)
		$scatterlight.light_energy = percentage * 0.00005
		if percentage == 0:
			for light in light_glows:
				self.remove_child(light)


func process_keys():
	if Input.is_action_pressed("forward"):
		self.add_force_local(Vector3(0, 0, THRUST), Vector3(0, 0, 0))
	elif Input.is_action_pressed("backwards"):
		self.add_force_local(Vector3(0, 0, -THRUST), Vector3(0, 0, 0))

	if Input.is_action_pressed("strafe_right"):
		self.add_force_local(Vector3(-THRUST, 0, 0), Vector3(0, -0.05, 0))
	elif Input.is_action_pressed("strafe_left"):
		self.add_force_local(Vector3(THRUST, 0, 0), Vector3(0, -0.05, 0))

	if Input.is_action_pressed("upwards"):
		self.add_force_local_if_underwater(Vector3(0, THRUST, 0), Vector3(0, -0.05, 0))
	elif Input.is_action_pressed("downwards"):
		self.add_force_local(Vector3(0, -THRUST, 0), Vector3(0, -0.05, 0))

	if Input.is_action_pressed("rotate_left"):
		self.apply_torque(self.transform.basis * Vector3(0, THRUST, 0))
	elif Input.is_action_pressed("rotate_right"):
		self.apply_torque(self.transform.basis * Vector3(0, -THRUST, 0))

	if Input.is_action_pressed("camera_up"):
		camera_tilt = min(camera_tilt + 0.5, 45)
	elif Input.is_action_pressed("camera_down"):
		camera_tilt = max(camera_tilt - 0.5, -45)

	if Input.is_action_pressed("gripper_open"):
		ljoint.open()
		rjoint.open()
	elif Input.is_action_pressed("gripper_close"):
		ljoint.close()
		rjoint.close()
	#else:
		#ljoint.set_motor_target_velocity(0)
		#rjoint.set_motor_target_velocity(0)
