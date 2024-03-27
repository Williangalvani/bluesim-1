extends RigidBody3D

const THRUST = 50

var udp = PacketPeerUDP.new()  # UDP socket for fdm in (server)
var peer = false
var start_time = Time.get_ticks_msec()

var last_velocity = Vector3(0, 0, 0)
var calculated_acceleration = Vector3(0, 0, 0)

var buoyancy = 10* self.mass * self.gravity_scale  # Newtons
var _initial_position = 0
var phys_time = 0
var last_com_time = Time.get_ticks_msec()

@onready var light_glows = [$light_glow, $light_glow2, $light_glow3, $light_glow4]

@onready var ljoint = get_tree().get_root().find_child("ljoint", true, false)
@onready var rjoint = get_tree().get_root().find_child("rjoint", true, false)
@onready var wait_SITL = Globals.wait_SITL
@onready var ping1d = $ping1d

func connect_fmd_in():
	if udp.bind(9002) != OK:
		print("Failed to connect fdm_in")


func get_servos():
	if udp.get_available_packet_count() == 0:
		if Time.get_ticks_msec() - last_com_time > 1000:
			peer = false
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
	last_com_time = Time.get_ticks_msec()
	if magic != 18458:
		return
	for i in range(0, 17):
		buffer.seek(8 + i * 2)
		actuate_servo(i, (float(buffer.get_u16()) - 1000) / 1000)


func send_fdm():
	var buffer = StreamPeerBuffer.new()

	buffer.put_double((Time.get_ticks_msec() - start_time) / 1000.0)

	var _basis = transform.basis

# These are the same but mean different things, let's keep both for now
	var toNED = Basis(Vector3(-1, 0, 0), Vector3(0, 0, -1), Vector3(1, 0, 0))

	toNED = Basis(Vector3(1, 0, 0), Vector3(0, 0, -1), Vector3(0, 1, 0))

	var toFRD = Basis(Vector3(0, -1, 0), Vector3(0, 0, -1), Vector3(1, 0, 0))

	var _angular_velocity = toFRD * angular_velocity * _basis
	var gyro = [_angular_velocity.x, _angular_velocity.y, _angular_velocity.z]

	var _acceleration = toFRD * calculated_acceleration * _basis

	var accel = [_acceleration.x, _acceleration.y, _acceleration.z]

	# var orientation = toFRD * Vector3(-rotation.x, - rotation.y, -rotation.z)
	var quaternon = Basis(-_basis.z, _basis.x, _basis.y).rotated(Vector3(1, 0, 0), PI).rotated(Vector3(1, 0, 0), PI / 2).get_rotation_quaternion()

	var euler = quaternon.get_euler()
	euler = [euler.y, euler.x, euler.z]

	var _velocity = toNED * self.linear_velocity
	var velo = [_velocity.x, _velocity.y, _velocity.z]

	var _position = toNED * self.transform.origin
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
	_initial_position = get_global_transform().origin
	print("setting phyisics process...")
	set_physics_process(true)
	set_process_input(true) 
	if typeof(Globals.active_vehicle) == TYPE_STRING and Globals.active_vehicle == "bluerovheavy":
		print("setting active vehicle....")
		Globals.active_vehicle = self
	else:
		return
	if not Globals.isHTML5:
		print("connecting fdm...")
		connect_fmd_in()
	print("init done")
	print(is_inside_tree())


func _physics_process(delta):
	if Engine.is_editor_hint():
		return
	phys_time = phys_time + 1.0 / Globals.physics_rate
	process_keys()
	if Globals.isHTML5:
		print("html5")
		return
	calculated_acceleration = (self.linear_velocity - last_velocity) / delta
	calculated_acceleration.y += 10
	last_velocity = self.linear_velocity
	get_servos()
	if peer:
		send_fdm()



func add_force_local(force: Vector3, pos: Vector3):
	var pos_local = self.transform.basis * pos
	var force_local = self.transform.basis * force
	self.apply_force(force_local, pos_local)

func add_force_local_if_underwater(force: Vector3, pos: Vector3):
	if (self.transform.basis * pos).z > 3.5:
		return
	var pos_local = self.transform.basis * pos
	var force_local = self.transform.basis * force
	self.apply_force(force_local, pos_local)


func actuate_servo(id, percentage):
	if percentage == 0:
		return

	var force = (percentage - 0.5) * 2 * -THRUST
	match id:
		0:
			self.add_force_local_if_underwater($t1.transform.basis*Vector3(force,0,0), $t1.position)
		1:
			self.add_force_local_if_underwater($t2.transform.basis*Vector3(force,0,0), $t2.position)
		2:
			self.add_force_local_if_underwater($t3.transform.basis*Vector3(force,0,0), $t3.position)
		3:
			self.add_force_local_if_underwater($t4.transform.basis*Vector3(force,0,0), $t4.position)
		4:
			self.add_force_local_if_underwater($t5.transform.basis*Vector3(force,0,0), $t5.position)
		5:
			self.add_force_local_if_underwater($t6.transform.basis*Vector3(force,0,0), $t6.position)
		6:
			self.add_force_local_if_underwater($t7.transform.basis*Vector3(force,0,0), $t7.position)
		7:
			self.add_force_local_if_underwater($t8.transform.basis*Vector3(force,0,0), $t8.position)
		14:
			$Camera3D.rotation_degrees.x = 0.9 * $Camera3D.rotation_degrees.x + 0.1 * (-45 + 120 * percentage)
		12:
			percentage -= 0.1
			percentage = max(0, percentage)
			$light1.light_energy = percentage * 5
			$light2.light_energy = percentage * 5
			$light3.light_energy = percentage * 5
			$light4.light_energy = percentage * 5
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
		$light1.light_energy = percentage
		$light2.light_energy = percentage
		$light3.light_energy = percentage
		$light4.light_energy = percentage
		$scatterlight.light_energy = percentage * 0.00005

	if event.is_action("lights_down"):
		var percentage = min(max(0, $light1.light_energy - 0.1), 5)
		print("lights to", percentage)
		$light1.light_energy = percentage
		$light2.light_energy = percentage
		$light3.light_energy = percentage
		$light4.light_energy = percentage
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
		$Camera3D.rotation_degrees.x = min($Camera3D.rotation_degrees.x + 0.5, 45)
	elif Input.is_action_pressed("camera_down"):
		$Camera3D.rotation_degrees.x = 	max($Camera3D.rotation_degrees.x - 0.5, -45)

	if Input.is_action_pressed("gripper_open"):
		ljoint.open()
		rjoint.open()
	elif Input.is_action_pressed("gripper_close"):
		ljoint.close()
		rjoint.close()
	#else:
		#ljoint.set_motor_target_velocity(0)
		#rjoint.set_motor_target_velocity(0)
