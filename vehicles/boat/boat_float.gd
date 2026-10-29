extends AnimatableBody3D
## Survey/support boat floating on BlueSim's water.
##
## BlueSim's sea is a flat plane at Globals.surface_altitude (the same height scripts/environment.gd
## computes buoyancy against), so the boat doesn't need a buoyancy solve: it holds its design
## waterline (this node's origin) on that plane and adds a gentle swell of heave, pitch and roll
## from a few sines. It is kinematic, so the ROV and the tether can push against it without moving
## it, and the TetherAnchor child is a steady topside end for the tether.

## metres the boat's origin (its design waterline) sits above the water surface; lower to load the
## boat deeper
@export var waterline_offset := 0.0
@export var heave_amplitude := 0.05  ## m
@export var heave_period := 4.3  ## s
@export var pitch_degrees := 1.0
@export var pitch_period := 5.3  ## s
@export var roll_degrees := 1.8
@export var roll_period := 3.7  ## s
## 0 keeps the boat still at the surface (a static pose); 1 is the default swell
@export_range(0.0, 3.0) var swell := 1.0

## the transform _float last set: until the next physics step the body's own transform still reads
## the previous one (AnimatableBody3D syncs to physics), so the ground tackle pins this one
var pose := Transform3D()
var _base: Basis
var _time := 0.0
var _phase := 0.0


func _ready():
	# the placement's heading is kept; x/z stay where the boat was put and y follows the water
	_base = global_basis.orthonormalized()
	_phase = float(hash(get_path()) % 1000) / 1000.0 * TAU  # boats side by side don't bob in step
	_float(0.0)


func _physics_process(delta):
	_time += delta
	_float(_time)


func _float(t):
	var wave1 = sin(TAU * t / heave_period + _phase)
	var wave2 = 0.35 * sin(TAU * t / (heave_period * 0.43) + 2.0 * _phase)
	var heave = heave_amplitude * (wave1 + wave2) / 1.35
	var pitch = deg_to_rad(pitch_degrees) * sin(TAU * t / pitch_period + 1.3 * _phase)
	var roll = deg_to_rad(roll_degrees) * sin(TAU * t / roll_period + 0.7 * _phase)
	var origin = global_position
	origin.y = water_height() + waterline_offset + heave * swell
	pose = Transform3D(_base * Basis.from_euler(Vector3(pitch * swell, 0, roll * swell)), origin)
	global_transform = pose


## world height of the water surface
func water_height() -> float:
	var globals = get_node_or_null("/root/Globals")
	return globals.surface_altitude if globals else 0.0


## where the tether leaves the boat, in world space; it moves with the swell
func tether_anchor_position() -> Vector3:
	return $TetherAnchor.global_position
