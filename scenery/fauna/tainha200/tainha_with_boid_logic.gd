extends RigidBody3D

@export var max_neighbor_distance: int = 1
@export var forward_force: float = 10
@export var separation_gain: float = 3.0
@export var cohesion: float = 0.2
@export var alignment: float = 0.3
@export var avoidance: float = 6

var neighbor_nodes = []
var avoidance_acumulator = Vector3.ZERO
var raycasts = []

# Called when the node enters the scene tree for the first time.
func _ready():
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
	$target.global_transform.origin = global_target
	var relative_distance = global_target - self.get_head_position()

	self.apply_force(relative_distance.normalized() * gain, $forcepoint.global_transform.origin - self.global_transform.origin)

func steer_away(global_target, gain=1.0):
	var new_target = self.global_transform.origin - global_target
	self.steer_towards(new_target, gain)
	
func apply_cohesion():
	if self.neighbor_nodes.size() == 0:
		return
	var average_position = Vector3.ZERO
	for node in self.neighbor_nodes:
		self.steer_towards(node.global_transform.origin, cohesion)

func apply_alignment():
	if self.neighbor_nodes.size() == 0:
		return
	var average_velocity = Vector3.ZERO
	for node in self.neighbor_nodes:
		var distance = self.global_transform.origin.distance_to(node.global_transform.origin)+1
		self.steer_towards(self.global_transform.origin + node.linear_velocity.normalized()*100,alignment/distance)
	
func apply_separation():
	if self.neighbor_nodes.size() == 0:
		return
	for node in self.neighbor_nodes:
		var distance = self.global_transform.origin.distance_to(node.global_transform.origin)+1
		self.steer_away(node.global_transform.origin, separation_gain/distance)

func apply_near_obstacle_avoidance():
	self.steer_away(self.avoidance_acumulator)
	self.avoidance_acumulator = Vector3.ZERO

func random_vector():
	return Vector3(randf_range(-0.1,0.1),randf_range(-0.1,0.1), randf())

func apply_stay_in_water():
	var water_level = 3.5
	var water_force = Vector3.ZERO
	if self.global_transform.origin.y > water_level:
		water_force = Vector3(0, -5, 0)
		self.apply_force(water_force, Vector3(0,0,0.2))



func apply_regular_obstacle_avoidance():
	if not $forward.is_colliding() :
		$target.global_transform.origin = Vector3(100,0,0)
		return
	
	var furthest = 0
	var furthest_point = null
	for raycast in raycasts:
		if not raycast.is_colliding():
			var point = self.global_transform.origin + raycast.global_transform.basis.y*-1
			$target.global_transform.origin = point
			self.steer_towards(point, avoidance)
			return
		else:
			var point = raycast.get_collision_point()
			var distance = point.distance_to(self.global_transform.origin)
			if distance > furthest:
				furthest_point = point
				furthest = distance
	self.steer_towards(furthest_point)
			
			

func get_head_position():
	return $forcepoint.global_transform.origin

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(_delta):
	self.neighbor_nodes = self.find_neighbors()
	self.apply_cohesion()
	self.apply_alignment()
	self.apply_separation()
	self.apply_near_obstacle_avoidance()
	self.apply_regular_obstacle_avoidance()
	self.apply_stay_in_water()
	var force = self.global_transform.basis.z * forward_force
	force.y *= 0.5
	self.apply_force(force + Vector3(randf_range(-0.5,0.5),randf_range(-0.5,0.5),randf_range(-0.5,0.5)), self.global_transform.basis.z * 0.2)
