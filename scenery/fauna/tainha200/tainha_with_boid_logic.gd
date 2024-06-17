extends RigidBody3D

@export var max_neighbor_distance: float = 0.8
@export var forward_force: float = 15.0
@export var separation_gain: float = 1.8
@export var cohesion: float = 12.0
@export var alignment: float = 4.0
@export var avoidance: float = 10.0
@export var upright: float = 0.1
@export var max_force: float = 20.0
@export var debug: bool = false

var head_position = Vector3.ZERO
var neighbor_nodes = []
var num_neighbors = 0
var raycasts = []
var force_sum = Vector3.ZERO
var randomized_force = 0

# Called when the node enters the scene tree for the first time.
func _ready():
	self.randomized_force = self.forward_force + randf()*5.0
	for child in get_children():
		if child is RayCast3D:
			raycasts.append(child)

func find_neighbors():
	var boidNodes = get_tree().get_nodes_in_group("boid")
	var closeNodes = []
	
	# Process the boid nodes
	for node in boidNodes:
		if node == self:
			continue
		var distance = self.global_transform.origin.distance_to(node.global_transform.origin)
		if distance < max_neighbor_distance:
			closeNodes.append(node)
	return closeNodes

func steer_towards(global_target, gain=1.0):
	if global_target == Vector3.ZERO or is_nan(global_target.x):
		return
	#$target.global_transform.origin = global_target
	var relative_distance = global_target - self.head_position
	self.force_sum += relative_distance.normalized() * gain
	#self.apply_force(, self.head_position - self.global_transform.origin)

func steer_away(global_target, gain=1.0):
	var away_vector = self.global_transform.origin - global_target
	var new_target = self.global_transform.origin + away_vector
	
	self.steer_towards(new_target, gain)
	
## Encourage nearby boids to stick together (near each other)
func apply_cohesion():
	if self.num_neighbors == 0:
		return
	#var average_position = Vector3.ZERO
	var scale = cohesion / self.num_neighbors
	for node in self.neighbor_nodes:
		self.steer_towards(node.global_transform.origin, scale)

## Encourage nearby boids to move in the same direction
func apply_alignment_and_separation():
	if self.num_neighbors == 0:
		return
	var average_velocity = Vector3.ZERO
	for node in self.neighbor_nodes:
		var distance_sq = self.global_transform.origin.distance_squared_to(node.global_transform.origin)+0.01
		# alignment
		self.steer_towards(self.global_transform.origin + node.linear_velocity.normalized()*100,alignment/(distance_sq * self.num_neighbors))
		# separation
		self.steer_away(node.global_transform.origin, separation_gain/(distance_sq * distance_sq * self.num_neighbors))

func random_vector():
	return Vector3(randf_range(-0.1,0.1),randf_range(-0.1,0.1), randf())

func apply_stay_in_water():
	var water_level = 4.3
	var water_force = Vector3.ZERO
	if self.global_transform.origin.y > water_level:
		water_force = Vector3(0, -5, 0)
		self.apply_force(water_force, Vector3(0,0,0.2))

func apply_keep_upright():
	var roll = -self.upright * self.global_transform.basis.get_euler()[2]
	self.apply_torque(self.global_transform.basis.z * roll)

func apply_regular_obstshacle_avoidance():
	if not $forward.is_colliding() :
		$target.global_transform.origin = Vector3(100,0,0)
		return false	

	var collision_imminence = 0.8 - $forward.get_collision_point(0).distance_to(self.global_transform.origin)

	var furthest = 0
	var furthest_point = null

	# if not avoiding yet, pick a free raycast
	for raycast in raycasts:
		if not raycast.is_colliding():
			var point = self.global_transform.origin + raycast.global_transform.basis.y*-1
			$target.global_transform.origin = point
			self.steer_towards(point, avoidance*collision_imminence)
			return true
		else:
			var point = raycast.get_collision_point()
			var distance = point.distance_to(self.global_transform.origin)
			if distance > furthest:
				furthest_point = point
				furthest = distance
	self.steer_towards(furthest_point)
	return true

func get_head_position():
	return $forcepoint.global_transform.origin

func apply_forward():
	var force = self.global_transform.basis.z * forward_force
	var rand = 3.0
	var random_offset = + Vector3(randf_range(-rand,rand),randf_range(-rand,rand),randf_range(-rand,rand))
	self.apply_force(force + random_offset, self.global_transform.basis.z * 0.2)
	

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(_delta):
	self.neighbor_nodes = self.find_neighbors()
	self.num_neighbors = self.neighbor_nodes.size()
	#print(self.num_neighbors)
	self.head_position = self.get_head_position()
	self.force_sum = Vector3.ZERO # no force to start with
	# sum all the forces
	var avoiding = self.apply_regular_obstshacle_avoidance()
	self.apply_alignment_and_separation()
	self.apply_cohesion()
	self.apply_stay_in_water()
	self.apply_keep_upright()
	if not avoiding:

		self.apply_forward()
	# apply the force
	self.force_sum = self.force_sum.normalized() * max(min(self.force_sum.length(), self.max_force), self.max_force/2)
	self.apply_force(self.force_sum, self.head_position - self.global_transform.origin)
