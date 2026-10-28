extends Node
# Checks the simulated Ping360's scan against a flat floor 3 m under the transducer and a wall 8 m
# north of it, and its placement of the scan on the map.
#   godot4.7 --headless --path . res://tests/ping360/ping360_scan_test.tscn
# Prints one line per check and exits non-zero if any fails.

const Ping360 = preload("res://vehicles/components/ping360.gd")
const PINGS := 40
const FLOOR_DEPTH := 3.0
const WALL_NORTH := 8.0

var failures := 0
var sonar: Ping360
var space: PhysicsDirectSpaceState3D


func _ready() -> void:
	_run.call_deferred()


func _run() -> void:
	var floor_body := _box(Vector3(0, -FLOOR_DEPTH - 0.5, 0), Vector3(200, 1, 200))
	var wall := _box(Vector3(WALL_NORTH + 0.5, 0, 0), Vector3(1, 40, 60))
	var far_wall := _box(Vector3(25.5, 0, 0), Vector3(1, 40, 60))
	sonar = Ping360.new()
	# scanned by hand below, not by its own physics process
	sonar.process_mode = Node.PROCESS_MODE_DISABLED
	add_child(sonar)
	space = sonar.get_world_3d().direct_space_state
	var north := Basis(Vector3.UP, PI / 2)  # forward (+Z) turned to face north (+X)

	_check("ping period at 20 m", absf(sonar.ping_period() - (40.0 / 1500 + 0.012)) < 1e-6, "")
	sonar.range_m = 1
	_check("ping period capped at 50 Hz", is_equal_approx(sonar.ping_period(), 0.02), "")
	sonar.range_m = 20

	await _set_bodies([floor_body, wall], [far_wall])
	var bins := _mean_scan(north, 0)
	var peak := _argmax(bins)
	var expected := WALL_NORTH / sonar.range_m * sonar.samples
	_check(
		"wall echo, head 0 facing north",
		absf(peak - expected) <= 2,
		"peak bin %d, expected %.1f, intensity %.2f" % [peak, expected, bins[peak]]
	)
	# head angles turn clockwise seen from above, so with the vehicle facing east north is 300
	bins = _mean_scan(Basis.IDENTITY, 300)
	peak = _argmax(bins)
	_check(
		"wall echo, head 300 facing east",
		absf(peak - expected) <= 2,
		"peak bin %d, expected %.1f" % [peak, expected]
	)
	bins = _mean_scan(Basis.IDENTITY, 100)
	var echo := _first_echo(bins)
	_check(
		"nothing before the floor, head 100 facing east (south)", echo > 130, "first bin %d" % echo
	)

	# the floor first answers where the lower edge of the beam, 12.5 degrees down, reaches it
	sonar.noise_floor = 0
	var hits := 0
	var first := sonar.samples
	for i in PINGS:
		bins = sonar.scan(space, Vector3.ZERO, north, 200)
		hits += sonar.last_hits
		echo = _first_echo(bins)
		if echo >= 0:
			first = mini(first, echo)
	var slant := FLOOR_DEPTH / sin(deg_to_rad(12.5))
	expected = slant / sonar.range_m * sonar.samples
	_check(
		"floor echo, head 200 facing south",
		hits > 0 and absf(first - expected) <= 3,
		"first echo bin %d, expected %.1f (%.2f m slant), %d hits" % [first, expected, slant, hits]
	)
	sonar.range_m = 10
	_check("floor out of range at 10 m", _hits(north, 200) == 0, "")
	sonar.range_m = 20

	await _set_bodies([far_wall], [floor_body, wall])
	_check("wall 25 m away out of range at 20 m", _hits(north, 0) == 0, "")
	sonar.range_m = 30
	bins = _mean_scan(north, 0)
	peak = _argmax(bins)
	expected = 25.0 / sonar.range_m * sonar.samples
	_check("wall 25 m away in range at 30 m", absf(peak - expected) <= 2, "peak bin %d" % peak)
	sonar.range_m = 20

	await _set_bodies([], [far_wall])
	sonar.noise_floor = 0.03
	hits = 0
	var loudest := 0.0
	for angle in range(0, 400, 10):
		bins = sonar.scan(space, Vector3.ZERO, north, angle)
		hits += sonar.last_hits
		for value in bins:
			loudest = maxf(loudest, value)
	_check(
		"open water, full turn",
		hits == 0 and loudest <= sonar.noise_floor,
		"%d hits, loudest bin %.3f" % [hits, loudest]
	)

	_check_map(north)

	for body: Node in [floor_body, wall, far_wall]:
		body.free()
	print("PING360TEST %s: %d failed" % ["FAIL" if failures else "PASS", failures])
	get_tree().quit(1 if failures else 0)


func _check_map(north: Basis) -> void:
	var lat := Ping360.DEFAULT_HOME_LAT
	var lon := Ping360.DEFAULT_HOME_LON
	var lonlat := Ping360.world_to_lonlat(Vector3(Ping360.METRES_PER_DEGREE / 1000, 0, 0), lat, lon)
	_check("1/1000 degree north", lonlat == [-118.393867, 33.811313], str(lonlat))
	# a degree of longitude is cos(lat) shorter
	var east := Ping360.METRES_PER_DEGREE * cos(deg_to_rad(lat)) / 1000
	lonlat = Ping360.world_to_lonlat(Vector3(0, 0, east), lat, lon)
	_check("1/1000 degree east is +Z", lonlat == [-118.392867, 33.810313], str(lonlat))

	sonar.basis = north
	var bins := PackedFloat32Array()
	bins.resize(sonar.samples)
	bins.fill(1.0)
	var cells := sonar.wedge_cells(bins, 0)
	var ring: Array = cells[10].geometry.coordinates[0]
	_check(
		"map cell, head 0 facing north",
		(
			cells.size() == sonar.display_bins
			and ring.size() == 5
			and ring[0] == ring[4]
			and ring[1][1] > ring[0][1]
			and ring[3][0] > ring[0][0]
		),
		"%d cells, ring %s" % [cells.size(), ring]
	)
	cells = sonar.wedge_cells(bins, 100)
	ring = cells[10].geometry.coordinates[0]
	_check("map cell, head 100 facing north is east", ring[1][0] > ring[0][0], str(ring))
	sonar.rotate_y(-PI / 2)  # yawed right, to face east
	cells = sonar.wedge_cells(bins, 0)
	ring = cells[10].geometry.coordinates[0]
	_check("map cell, head 0 facing east", ring[1][0] > ring[0][0], str(ring))
	bins.fill(0.0)
	_check("no map cells for silence", sonar.wedge_cells(bins, 0).is_empty(), "")


func _box(centre: Vector3, size: Vector3) -> StaticBody3D:
	var body := StaticBody3D.new()
	var shape := CollisionShape3D.new()
	shape.shape = BoxShape3D.new()
	shape.shape.size = size
	body.add_child(shape)
	body.position = centre
	return body


func _set_bodies(present: Array, absent: Array) -> void:
	for body: Node in absent:
		if body.is_inside_tree():
			remove_child(body)
	for body: Node in present:
		if not body.is_inside_tree():
			add_child(body)
	for i in 2:
		await get_tree().physics_frame


func _mean_scan(axes: Basis, angle: int) -> PackedFloat32Array:
	var mean := PackedFloat32Array()
	mean.resize(sonar.samples)
	for i in PINGS:
		var bins := sonar.scan(space, Vector3.ZERO, axes, angle)
		for b in bins.size():
			mean[b] += bins[b] / PINGS
	return mean


func _hits(axes: Basis, angle: int) -> int:
	var hits := 0
	for i in PINGS:
		sonar.scan(space, Vector3.ZERO, axes, angle)
		hits += sonar.last_hits
	return hits


func _argmax(bins: PackedFloat32Array) -> int:
	var best := 0
	for b in bins.size():
		if bins[b] > bins[best]:
			best = b
	return best


# First bin louder than the noise could make it, or -1
func _first_echo(bins: PackedFloat32Array) -> int:
	for b in bins.size():
		if bins[b] > sonar.noise_floor:
			return b
	return -1


func _check(check_name: String, ok: bool, detail: String) -> void:
	if not ok:
		failures += 1
	print("PING360TEST %s %s %s" % ["PASS" if ok else "FAIL", check_name, detail])
