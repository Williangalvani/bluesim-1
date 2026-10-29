extends Node
# Loads a reef level pack and checks what the simulated Ping360 makes of it: the seabed under where
# the vehicle starts and around it, and whole turns of the head from the sonar dived at the spawn,
# 2 m above the seabed at the spawn, and 2 m above the seabed far out where only the newer levels
# have one. Turns from the sonar surfaced are measured but not checked. Also checks the vehicle's
# hull is on the layer the level's kelp reacts to.
#   godot4.7 --headless --path . res://tests/ping360/reef_level_ping_test.tscn -- pck=PCK out=JSON
# pck defaults to res://levels/reef.pck; out, if given, gets the results as JSON. Prints one line
# per check and exits non-zero if any fails.

const Ping360 = preload("res://vehicles/components/ping360.gd")
const DEFAULT_PCK := "res://levels/reef.pck"
const LEVEL := "res://custom_level.tscn"
const VEHICLE := "res://vehicles/bluerovheavy/BlueRovHeavy.tscn"
const HULL := "BlueRov"
# the vehicle where levels/baselevel.tscn starts it, surfaced, and its sonar surfaced and 2 m down
const SPAWN := Vector3(0, 3.41, 0)
const SURFACED := Vector3(0, 3.645, 0.2)
const DIVED := Vector3(0, 1.4, 0.2)
const GRID_REACH := 240
const GRID_STEP := 40
# m above the surface the rays looking for the seabed around the spawn start
const RAY_HEIGHT := 100.0
const RAY_LENGTH := 1000.0
const ANGLE_STEP := 4  # gradians
const RANGES := [20.0, 50.0]
const NEAREST_ECHO := 0.5  # m
const SETTLE_FRAMES := 5
const ROV_LAYER := 2
const KELP_LAYER := 8
# how high above the seabed the sonar looks for it, at the spawn and far out
const ABOVE_SEABED := 2.0
const SEABED_RANGE := 20.0
# x, z far out, where the older packs have no seabed
const OUTER := [Vector2(120, 120), Vector2(-180, 60)]
# the seabed assumed far out when the ray finds none
const OUTER_FALLBACK_Y := -7.5

var failures := 0
var results := {"checks": []}
var sonar: Ping360
var space: PhysicsDirectSpaceState3D
var level: Node


func _ready() -> void:
	_run.call_deferred()


func _run() -> void:
	var args := _args()
	var pck: String = args.get("pck", DEFAULT_PCK)
	results.pck = pck
	_check_hull_layers()
	var loaded := ProjectSettings.load_resource_pack(pck)
	_check("pack loaded", loaded, pck)
	if loaded and ResourceLoader.exists(LEVEL):
		level = (load(LEVEL) as PackedScene).instantiate()
		add_child(level)
		for i in SETTLE_FRAMES:
			await get_tree().physics_frame
		_measure()
	else:
		_check("level in the pack", false, LEVEL)
	results.failures = failures
	_write(args.get("out", ""))
	print("REEFPINGTEST %s: %d failed" % ["FAIL" if failures else "PASS", failures])
	get_tree().quit(1 if failures else 0)


# The hull as the vehicle's scene saves it, without making one
func _check_hull_layers() -> void:
	var state := (load(VEHICLE) as PackedScene).get_state()
	var layer := 1
	var mask := 1
	for node in state.get_node_count():
		if state.get_node_name(node) != HULL:
			continue
		for property in state.get_node_property_count(node):
			match state.get_node_property_name(node, property):
				"collision_layer":
					layer = state.get_node_property_value(node, property)
				"collision_mask":
					mask = state.get_node_property_value(node, property)
	results.hull = {"collision_layer": layer, "collision_mask": mask}
	_check(
		"vehicle hull on layers 1 and 2 (ROV), mask 1",
		layer == (1 | 1 << (ROV_LAYER - 1)) and mask == 1,
		"layer %d, mask %d" % [layer, mask]
	)


func _measure() -> void:
	sonar = Ping360.new()
	# scanned by hand below, not by its own physics process
	sonar.process_mode = Node.PROCESS_MODE_DISABLED
	add_child(sonar)
	space = sonar.get_world_3d().direct_space_state

	results.layer_names = {}
	for layer: int in [ROV_LAYER, KELP_LAYER]:
		var layer_name: String = ProjectSettings.get_setting(
			"layer_names/3d_physics/layer_%d" % layer, ""
		)
		results.layer_names[str(layer)] = layer_name
		print('REEFPINGTEST INFO 3D physics layer %d is named "%s"' % [layer, layer_name])

	var zones := 0
	for area: Area3D in level.find_children("*", "Area3D", true, false):
		if area.collision_mask & (1 << (ROV_LAYER - 1)):
			zones += 1
	results.kelp_zones = zones
	print("REEFPINGTEST INFO %d areas react to layer %d (ROV)" % [zones, ROV_LAYER])

	var surface: float = Globals.surface_altitude
	var spawn := _down(SPAWN.x, SPAWN.z, SPAWN.y)
	results.seabed_spawn = spawn
	_check(
		"seabed under the spawn",
		spawn.hit,
		(
			(
				"y %.2f, %.2f m below the surface, %.2f m under the vehicle, %s"
				% [spawn.y, surface - spawn.y, SPAWN.y - spawn.y, spawn.collider]
			)
			if spawn.hit
			else "nothing within %d m" % RAY_LENGTH
		)
	)

	var points := []
	var misses := 0
	for x in range(-GRID_REACH, GRID_REACH + 1, GRID_STEP):
		for z in range(-GRID_REACH, GRID_REACH + 1, GRID_STEP):
			var point := _down(x, z, surface + RAY_HEIGHT)
			points.append(point)
			if point.hit:
				print(
					(
						"REEFPINGTEST INFO seabed x %4d z %4d: y %8.2f %s"
						% [x, z, point.y, point.collider]
					)
				)
			else:
				misses += 1
				print("REEFPINGTEST INFO seabed x %4d z %4d: miss" % [x, z])
	results.grid = {"reach": GRID_REACH, "step": GRID_STEP, "misses": misses, "points": points}
	print("REEFPINGTEST INFO seabed grid: %d of %d missed" % [misses, points.size()])

	results.scans = []
	for range_m: float in RANGES:
		results.scans.append(_scan("surfaced", SURFACED, range_m))
	# at 20 m the beam passes over the seabed from here, so only measured
	results.scans.append(_scan("dived", DIVED, 20.0))
	_check_echoes(_scan("dived", DIVED, 50.0), "")

	var near := "%.0f m above the seabed at the spawn" % ABOVE_SEABED
	if spawn.hit:
		var origin := Vector3(SURFACED.x, spawn.y + ABOVE_SEABED, SURFACED.z)
		_check_echoes(_scan(near, origin, SEABED_RANGE), "seabed y %.2f" % spawn.y)
	else:
		_check(_echo_check_name(near, SEABED_RANGE), false, "no seabed under the spawn")

	print(
		(
			"REEFPINGTEST INFO the outer seabed checks fail on packs with no seabed out there"
			+ " (v8) and pass on those with one (v17)"
		)
	)
	for xz: Vector2 in OUTER:
		var bed := _down(xz.x, xz.y, surface + RAY_HEIGHT)
		var bed_y: float = bed.y if bed.hit else OUTER_FALLBACK_Y
		var found := (
			"seabed y %.2f, %s" % [bed_y, bed.collider]
			if bed.hit
			else "no seabed found, used y %.2f" % bed_y
		)
		print("REEFPINGTEST INFO outer seabed x %d z %d: %s" % [xz.x, xz.y, found])
		var origin := Vector3(xz.x, bed_y + ABOVE_SEABED, xz.y)
		var scan := _scan(
			"%.0f m above the outer seabed at x %d z %d" % [ABOVE_SEABED, xz.x, xz.y],
			origin,
			SEABED_RANGE
		)
		scan.seabed = bed
		scan.seabed_y_used = bed_y
		_check_echoes(scan, found)


func _echo_check_name(where: String, range_m: float) -> String:
	return "%s, %d m: most pings echo at %.1f..%d m" % [where, range_m, NEAREST_ECHO, range_m]


# Records `scan` and checks more than half its pings echo in range
func _check_echoes(scan: Dictionary, detail: String) -> void:
	var check_name := _echo_check_name(scan.where, scan.range_m)
	scan.check = check_name
	results.scans.append(scan)
	var counts := "%d of %d pings" % [scan.pings_plausible, scan.pings]
	_check(
		check_name,
		scan.pings_plausible * 2 > scan.pings,
		counts + (", " + detail if detail else "")
	)


# Straight down from `top` at `x`, `z`, with the default mask
func _down(x: float, z: float, top: float) -> Dictionary:
	var from := Vector3(x, top, z)
	var hit := space.intersect_ray(
		PhysicsRayQueryParameters3D.create(from, from + Vector3.DOWN * RAY_LENGTH)
	)
	if hit.is_empty():
		return {"x": x, "z": z, "hit": false}
	var y: float = hit.position.y
	return {"x": x, "z": z, "hit": true, "y": y, "collider": str((hit.collider as Node).name)}


# A whole turn of the head, every ANGLE_STEP gradians, from `origin` with the level's axes
func _scan(where: String, origin: Vector3, range_m: float) -> Dictionary:
	sonar.range_m = range_m
	var bin_size := range_m / sonar.samples
	var pings := 0
	var hit_pings := 0
	var hits := 0
	var plausible := 0
	var firsts: Array[float] = []
	var peak_sum := 0.0
	for angle in range(0, Ping360.GRADIANS, ANGLE_STEP):
		var bins := sonar.scan(space, origin, Basis.IDENTITY, angle)
		pings += 1
		hits += sonar.last_hits
		hit_pings += 1 if sonar.last_hits > 0 else 0
		var first := _first_echo(bins)
		if first < 0:
			continue
		var first_m := (first + 0.5) * bin_size
		firsts.append(first_m)
		peak_sum += (_argmax(bins) + 0.5) * bin_size
		if first_m >= NEAREST_ECHO and first_m <= range_m:
			plausible += 1
	firsts.sort()
	var echoes := firsts.size()
	var median: Variant = null
	if echoes:
		median = (firsts[(echoes - 1) / 2] + firsts[echoes / 2]) / 2
	var scan := {
		"where": where,
		"origin": [origin.x, origin.y, origin.z],
		"range_m": range_m,
		"angle_step": ANGLE_STEP,
		"pings": pings,
		"pings_hit": hit_pings,
		"pings_echo": echoes,
		"pings_plausible": plausible,
		"hits": hits,
		"first_echo_median_m": median,
		"first_echo_min_m": firsts[0] if echoes else null,
		"first_echo_max_m": firsts[-1] if echoes else null,
		"peak_mean_m": peak_sum / echoes if echoes else null,
	}
	var counts := (
		"%d pings, %d hit, %d echo (%d at %.1f..%d m)"
		% [pings, hit_pings, echoes, plausible, NEAREST_ECHO, range_m]
	)
	var ranges := (
		"first echo median %s min %s max %s m, mean peak %s m"
		% [
			_metres(scan.first_echo_median_m),
			_metres(scan.first_echo_min_m),
			_metres(scan.first_echo_max_m),
			_metres(scan.peak_mean_m),
		]
	)
	print(
		(
			"REEFPINGTEST INFO scan %s %d m from %s: %s, %s, %d hits"
			% [where, range_m, origin, counts, ranges, hits]
		)
	)
	return scan


static func _metres(value: Variant) -> String:
	return "-" if value == null else "%.2f" % value


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


# The user arguments after --, as key=value
static func _args() -> Dictionary:
	var args := {}
	for arg in OS.get_cmdline_user_args():
		var parts := arg.split("=", true, 1)
		if parts.size() == 2:
			args[parts[0]] = parts[1]
	return args


func _write(path: String) -> void:
	if path.is_empty():
		return
	var file := FileAccess.open(path, FileAccess.WRITE)
	if file == null:
		_check(
			"results written", false, "%s: %s" % [path, error_string(FileAccess.get_open_error())]
		)
		return
	file.store_string(JSON.stringify(results, "  ", false))
	print("REEFPINGTEST INFO results written to %s" % path)


func _check(check_name: String, ok: bool, detail: String) -> void:
	if not ok:
		failures += 1
	results.checks.append({"name": check_name, "ok": ok, "detail": detail})
	print("REEFPINGTEST %s %s %s" % ["PASS" if ok else "FAIL", check_name, detail])
