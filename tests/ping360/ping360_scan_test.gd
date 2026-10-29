extends Node
# Checks the simulated Ping360's scan against a flat floor 3 m under the transducer and a wall 8 m
# north of it, the quads it lays on Cockpit's map for its pings, the map's palette and ring of
# slots, as a model of Cockpit (cockpit_mesh_model.gd) takes them, and its log of quantized pings.
#   godot4.7 --headless --path . res://tests/ping360/ping360_scan_test.tscn
# Prints one line per check and exits non-zero if any fails.

const Ping360 = preload("res://vehicles/components/ping360.gd")
const Model = preload("res://tests/ping360/cockpit_mesh_model.gd")
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
	sonar.range_m = 20
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
	_check_palette()
	_check_capacity()
	_check_record()
	_check_pose()
	_check_map_commands()
	_check_ring()
	_check_log()
	await _set_bodies([floor_body, wall, far_wall], [])
	_check_turn(north)

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

	var origin := Vector3(3, -1, -2)
	sonar.transform = Transform3D(north, origin)
	sonar.range_m = 20
	var levels := PackedByteArray()
	for b in sonar.samples:
		levels.append(b + 50)
	levels[100] = 0
	levels[sonar.samples - 1] = 255
	var sent := levels.duplicate()
	var quad := sonar.ping_quad(levels, 0)
	var corners: PackedFloat64Array = quad.corners
	var transducer := Ping360.world_to_lonlat_exact(origin, sonar.home_lat, sonar.home_lon)
	var a := _offset(corners, 2)
	var b := _offset(corners, 3)
	var half := Ping360.WEDGE_STEPS * sonar.angle_step / 2.0 * TAU / Ping360.GRADIANS
	_check(
		"map quad [O, O, A, B]: O at the transducer, A and B at the range either side of north",
		(
			corners.size() == 8
			and corners.slice(0, 2) == transducer
			and corners.slice(2, 4) == transducer
			and absf(a.length() - sonar.range_m) < 1e-3
			and absf(b.length() - sonar.range_m) < 1e-3
			and absf(atan2(a.y, a.x) + half) < 1e-4
			and absf(atan2(b.y, b.x) - half) < 1e-4
		),
		"A %s, B %s m north, east of O" % [a, b]
	)
	sonar.angle_step = 2
	var wide := sonar.ping_quad(levels, 100)
	a = _offset(wide.corners, 2)
	b = _offset(wide.corners, 3)
	var bisector := rad_to_deg(atan2(a.y + b.y, a.x + b.x))
	var spread := rad_to_deg(a.angle_to(b))
	_check(
		"map quad widened to 1.5 head steps, head 100 facing north is east",
		absf(bisector - 90) < 1e-3 and absf(spread - 1.5 * 2 * 0.9) < 1e-3,
		"bisector %.4f, %.4f degrees wide" % [bisector, spread]
	)
	sonar.angle_step = 1
	sonar.rotate_y(-PI / 2)  # yawed right, to face east
	a = _offset(sonar.ping_quad(levels, 0).corners, 2)
	_check("map quad, head 0 facing east", absf(rad_to_deg(atan2(a.y, a.x)) - 90) < 1, str(a))
	var texels: PackedByteArray = quad.texels
	var blanked := ceili(sonar.map_blank_m / sonar.range_m * sonar.samples)
	var indexed := true
	for bin in range(blanked, sonar.samples):
		indexed = indexed and texels[bin] == Ping360.map_index(levels[bin])
	_check(
		"map quad uv runs 0 to the bins along the range, the bins' map indices its texels, ringdown 0",
		(
			quad.uv == PackedFloat32Array([0, 0.5, 0, 0.5, 200, 0.5, 200, 0.5])
			and texels.size() == sonar.samples
			and blanked > 0
			and texels.slice(0, blanked).count(0) == blanked
			and indexed
			and Array(texels.slice(blanked)).min() == 1
			and texels[100] == 1
			and texels[sonar.samples - 1] == 255
			and levels == sent
			and Marshalls.base64_to_raw(sonar.ping_record(levels, 0, 0, sonar.pose()).i) == sent
		),
		"%d bins blanked, texels %s" % [blanked, texels.slice(0, blanked + 3)]
	)
	var indices := []
	for level in 256:
		indices.append(Ping360.map_index(level))
	var rising := true
	for level in range(1, 256):
		rising = rising and indices[level] >= indices[level - 1]
	_check(
		"map index: silence 1, the strongest 255, rising, 0 left for no data",
		indices[0] == 1 and indices[255] == 255 and rising and indices.min() == 1,
		str(indices.slice(0, 8))
	)
	sonar.transform = Transform3D.IDENTITY


# The map's palette: 0, no data, transparent, and every echo strength drawn, from black for
# silence brightening to white, its alpha its index, from 1 for silence to opaque for the strongest
func _check_palette() -> void:
	var palette := Ping360.map_palette()
	var alphas := []
	var brightness := []
	for index in 256:
		var entry := palette.slice(index * 4, index * 4 + 4)
		alphas.append(entry[3])
		brightness.append(0.2126 * entry[0] + 0.7152 * entry[1] + 0.0722 * entry[2])
	var brightening := true
	for index in range(2, 256):
		brightening = brightening and brightness[index] >= brightness[index - 1]
	var alpha_is_index := true
	for index in range(1, 256):
		alpha_is_index = alpha_is_index and alphas[index] == index
	_check(
		"palette: 0 transparent, alpha the index, 1 for silence to 255, 1 black, brightening to white",
		(
			palette.size() == 1024
			and alphas[0] == 0
			and alpha_is_index
			and palette.slice(4, 8) == PackedByteArray([0, 0, 0, 1])
			and brightening
			and brightness[255] > brightness[1]
			and palette.slice(1020, 1024) == PackedByteArray([255, 255, 255, 255])
		),
		(
			"index 0..2 %s, alpha %s, brightness %s"
			% [palette.slice(0, 12), alphas.slice(1, 256, 32), brightness.slice(1, 256, 32)]
		)
	)


func _check_capacity() -> void:
	var capacities := []
	for range_m: float in [1.0, 10.0, 30.0, 50.0]:
		sonar.range_m = range_m
		capacities.append(sonar.map_capacity())
	sonar.map_history_s = 3600
	var most := sonar.map_capacity()
	sonar.map_history_s = 210
	sonar.range_m = 20
	_check(
		"map capacity: 3.5 min of pings, 4096 at 30 m",
		capacities == [11264, 9216, 4096, 3072] and most == Ping360.MAX_MAP_CAPACITY,
		str(capacities)
	)


# North and east, in m, of corner `corner` of `corners` from the first
static func _offset(corners: PackedFloat64Array, corner: int) -> Vector2:
	var north := (corners[corner * 2 + 1] - corners[1]) * Ping360.METRES_PER_DEGREE
	var east := (
		(corners[corner * 2] - corners[0])
		* Ping360.METRES_PER_DEGREE
		* cos(deg_to_rad(Ping360.DEFAULT_HOME_LAT))
	)
	return Vector2(north, east)


func _check_record() -> void:
	var bins := PackedFloat32Array()
	bins.resize(sonar.samples)
	# from below silence to beyond saturation
	for b in bins.size():
		bins[b] = b / 150.0 - 0.1
	sonar.pings = 77
	var record := sonar.ping_record(Ping360.quantize(bins), 123, 1.25, sonar.pose())
	var decoded := Marshalls.base64_to_raw(record.i)
	var worst := 0.0
	for b in mini(decoded.size(), bins.size()):
		worst = maxf(worst, absf(decoded[b] / 255.0 - clampf(bins[b], 0, 1)))
	_check(
		"record bins round-trip",
		decoded.size() == sonar.samples and worst <= 1.0 / 255,
		"%d bytes, worst error %.5f, %d base64 chars" % [decoded.size(), worst, record.i.length()]
	)
	_check(
		"record fields",
		(
			record.k == 77
			and record.t == 1250
			and record.a == 123
			and record.r == roundi(sonar.range_m * 100)
			and record.g == roundi(sonar.gain * 100)
			and record.n == sonar.samples
		),
		str(record)
	)


func _check_pose() -> void:
	var surface: float = Globals.surface_altitude
	# scaled like the sonar on the vehicle
	var scale := Vector3.ONE * 0.0614213
	sonar.transform = Transform3D(Basis.IDENTITY.scaled(scale), Vector3(12.5, surface - 4.2, -7.25))
	var levels := Ping360.quantize(PackedFloat32Array([0.5]))
	var record := sonar.ping_record(levels, 0, 0, sonar.pose())
	var lonlat := Ping360.world_to_lonlat(sonar.global_position, sonar.home_lat, sonar.home_lon)
	_check(
		"record position and depth",
		(
			record.lat == roundi(lonlat[1] * 1e7)
			and record.lon == roundi(lonlat[0] * 1e7)
			and record.d == 420
		),
		"%d %d %d, %s" % [record.lat, record.lon, record.d, lonlat]
	)
	var attitudes := [
		["yaw facing east", Basis.IDENTITY, [900, 0, 0]],
		["yaw facing north", Basis(Vector3.UP, PI / 2), [0, 0, 0]],
		["yaw facing west", Basis(Vector3.UP, PI), [2700, 0, 0]],
		["pitch 10 degrees nose up, facing east", Basis(Vector3.RIGHT, -PI / 18), [900, 100, 0]],
		[
			"roll 15 degrees right side down, facing east",
			Basis(Vector3.BACK, PI / 12),
			[900, 0, 150]
		],
	]
	for attitude: Array in attitudes:
		sonar.basis = (attitude[1] as Basis).scaled(scale)
		record = sonar.ping_record(levels, 0, 0, sonar.pose())
		var got := [record.yaw, record.pitch, record.roll]
		_check(attitude[0], got == attitude[2], "yaw, pitch, roll %s" % [got])
	sonar.transform = Transform3D.IDENTITY


# The map made, emptied, cleared and removed as Cockpit takes it, the log kept through a clear
func _check_map_commands() -> void:
	var posted: Array[Dictionary] = []
	sonar.poster = func(message: Dictionary) -> void: posted.append(message)
	sonar.range_m = 30
	sonar.start_map()
	var types := posted.map(func(message: Dictionary) -> String: return message.type)
	var mesh: Dictionary = posted[1].get("mesh", {})
	_check(
		"map start: capabilities asked, setMapMesh, then clearMapMesh, as Cockpit's API v1 has them",
		(
			types == ["cockpit:getMapCapabilities", "cockpit:setMapMesh", "cockpit:clearMapMesh"]
			and posted.all(func(message: Dictionary) -> bool: return message.has("requestId"))
			and mesh.id == "bluesim-ping360-map"
			and mesh.capacity == 4096
			and mesh.texels == {"format": "r8", "width": 200, "height": 1}
			and mesh.filter == "linear"
			and (mesh.palette as PackedByteArray).size() == 1024
			and mesh.opacity == Ping360.MAP_OPACITY
		),
		str(types)
	)
	var levels := PackedByteArray()
	levels.resize(sonar.samples)
	levels.fill(200)
	sonar.log_cap = 50
	sonar.clear_log()
	for n in 3:
		sonar.log_ping(sonar.ping_record(levels, n, n, sonar.pose()), levels)
		sonar.map_ping(levels, n)
	sonar.map_mesh.flush()
	sonar.command({"type": "bluesim:ping360", "command": "clear"})
	sonar.map_ping(levels, 3)
	sonar.map_mesh.flush()
	var model := Model.new()
	var problems := model.apply_all(posted)
	_check(
		"clear command empties the map and restarts its ring, keeping the log",
		(
			problems.is_empty()
			and posted[-2].type == "cockpit:clearMapMesh"
			and posted[-1].first == 0
			and model.slot_texels(Ping360.MAP_ID).keys() == [0]
			and sonar.stats().logPings == 3
		),
		"%s %s" % [problems, posted.map(func(message: Dictionary) -> String: return message.type)]
	)
	# BlueSim reloaded in the same Cockpit: made again and emptied, the old pings gone
	sonar.start_map()
	model.apply_all(posted)
	sonar.stop_map()
	model.apply_all(posted.slice(-1))
	_check(
		"map start again empties Cockpit's copy, stop removes it",
		(
			model.problems.is_empty()
			and posted[-1].type == "cockpit:removeMapMesh"
			and posted[-1].id == Ping360.MAP_ID
			and model.meshes.is_empty()
			and sonar.map_mesh == null
		),
		str(model.problems)
	)
	sonar.command({"type": "bluesim:ping360", "command": "clear"})
	_check("clear with the sonar off sends nothing", posted[-1].type == "cockpit:removeMapMesh", "")
	sonar.log_cap = 50000


# More pings than the map holds: the oldest slots rewritten, never a write past the ring's end
func _check_ring() -> void:
	var posted: Array[Dictionary] = []
	sonar.poster = func(message: Dictionary) -> void: posted.append(message)
	sonar.range_m = 30
	sonar.map_history_s = 10
	sonar.start_map()
	var capacity: int = sonar.map_mesh.capacity
	var levels := PackedByteArray()
	levels.resize(sonar.samples)
	for n in capacity + 10:
		levels.fill(n % 256)
		sonar.map_ping(levels, n % Ping360.GRADIANS)
		if n % 7 == 6:
			sonar.map_mesh.flush()
	sonar.map_mesh.flush()
	var model := Model.new()
	var problems := model.apply_all(posted)
	var held := model.slot_texels(Ping360.MAP_ID)
	var newest := true
	for slot in 10:
		newest = newest and held[slot][100] == Ping360.map_index((capacity + slot) % 256)
	var largest := 0
	for message in posted:
		if message.has("corners"):
			largest = maxi(largest, message.corners.size() / 8)
	_check(
		"ring wraps: %d slots hold the newest pings" % capacity,
		(
			problems.is_empty()
			and capacity == Ping360.MIN_MAP_CAPACITY
			and held.size() == capacity
			and newest
			and held[10][100] == Ping360.map_index(10)
			and largest <= 7
		),
		str(problems)
	)
	sonar.stop_map()
	sonar.map_history_s = 210


func _check_log() -> void:
	var posted: Array[Dictionary] = []
	sonar.poster = func(message: Dictionary) -> void: posted.append(message)
	sonar.log_cap = 5
	sonar.clear_log()
	for n in range(1, 13):
		sonar.pings = n
		var levels := PackedByteArray()
		levels.resize(sonar.samples)
		levels.fill(n)
		sonar.log_ping(sonar.ping_record(levels, n, n, sonar.pose()), levels)
	var pings: Array = sonar.ping_log().pings
	var numbers := pings.map(func(record: Dictionary) -> int: return record.k)
	var intact := true
	for record: Dictionary in pings:
		var levels := Marshalls.base64_to_raw(record.i)
		if levels.size() != sonar.samples or levels.count(record.k) != sonar.samples:
			intact = false
	var stats := sonar.stats()
	_check(
		"log keeps the last log_cap pings, oldest first",
		(
			numbers == [8, 9, 10, 11, 12]
			and intact
			and stats.logPings == 5
			and stats.logBytes == 5 * (Ping360.RECORD_KEYS.size() * 4 + sonar.samples)
		),
		"%s, %d bytes" % [numbers, stats.logBytes]
	)
	sonar.command({"type": "bluesim:ping360", "command": "exportLog", "requestId": "r1"})
	var reply: Dictionary = posted[-1] if posted else {}
	_check(
		"exportLog command answers with the log",
		(
			reply.get("type") == "bluesim:ping360Log"
			and reply.requestId == "r1"
			and reply.log.format == "bluesim-ping360-log/1"
			and reply.log.pings.size() == 5
		),
		str(reply.keys())
	)
	sonar.command({"type": "bluesim:ping360", "command": "clearLog"})
	_check("clearLog command empties the log", sonar.stats().logPings == 0, "")
	sonar.log_cap = 50000


# A whole turn with the floor and both walls in range, mapped and logged as in Cockpit
func _check_turn(north: Basis) -> void:
	var posted: Array[Dictionary] = []
	sonar.poster = func(message: Dictionary) -> void: posted.append(message)
	sonar.transform = Transform3D(north, Vector3.ZERO)
	sonar.range_m = 30
	sonar.noise_floor = 0.03
	sonar.head_angle = 0
	sonar.start_map()
	var before := sonar.stats()
	var period := sonar.ping_period()
	var started := Time.get_ticks_usec()
	for n in Ping360.GRADIANS:
		sonar.ping(n * period)
		sonar.map_mesh.tick(period)
	var elapsed := Time.get_ticks_usec() - started
	var stats := sonar.stats()
	var map: Dictionary = stats.map
	var writes := posted.filter(
		func(message: Dictionary) -> bool: return message.type == "cockpit:setMapMeshQuads"
	)
	var typed := 0
	var json := 0
	for message in writes:
		var sizes := Ping360.MapMesh.message_bytes(message)
		typed += sizes.x
		json += sizes.y
	var model := Model.new()
	var problems := model.apply_all(posted)
	# the wall 8 m north, in the ping at head 0
	var wall_bin := roundi(WALL_NORTH / sonar.range_m * sonar.samples)
	var wall: PackedByteArray = model.slot_texels(Ping360.MAP_ID)[0].slice(
		wall_bin - 3, wall_bin + 4
	)
	var log_json := JSON.stringify(sonar.ping_log()).length()
	var logged := Marshalls.base64_to_raw(sonar.ping_log().pings[0].i)
	var mapped: PackedByteArray = model.slot_texels(Ping360.MAP_ID)[0]
	var blanked := ceili(sonar.map_blank_m / sonar.range_m * sonar.samples)
	var silent := 0
	var as_logged := mapped.slice(0, blanked).count(0) == blanked
	for b in range(blanked, sonar.samples):
		as_logged = as_logged and mapped[b] == Ping360.map_index(logged[b])
		silent += 1 if logged[b] == 0 else 0
	_check(
		"a turn's first ping: raw bytes in the log, their map indices on the map, silence drawn",
		as_logged and silent > 0 and Array(logged).max() == 255 and Array(mapped).max() == 255,
		"%d silent bins, log %s, map %s" % [silent, logged.slice(0, 12), mapped.slice(0, 12)]
	)
	_check(
		"a turn mapped and logged",
		(
			problems.is_empty()
			and stats.revolutions == before.revolutions + 1
			and stats.logPings == Ping360.GRADIANS
			and map.slotsWritten == Ping360.GRADIANS
			and writes.size() == Ping360.GRADIANS
			and Array(wall).max() > 200
			and absi(stats.pingBytesMeshTyped - typed / writes.size()) <= 4
			and stats.pingBytesBinary == Ping360.RECORD_KEYS.size() * 4 + sonar.samples
		),
		(
			(
				"%d messages, %d pings in %d ms (scan and map); posted per ping: %.0f bytes"
				+ " typed, %.0f as base64 JSON; per ping elsewhere: float JSON %d, quantized"
				+ " JSON %d, binary %d, exported log JSON %.0f; log %d bytes; wall %s %s"
			)
			% [
				writes.size(),
				Ping360.GRADIANS,
				elapsed / 1000,
				float(typed) / Ping360.GRADIANS,
				float(json) / Ping360.GRADIANS,
				stats.pingBytesFloatJson,
				stats.pingBytesQuantJson,
				stats.pingBytesBinary,
				float(log_json) / Ping360.GRADIANS,
				stats.logBytes,
				wall,
				problems,
			]
		)
	)
	sonar.stop_map()
	sonar.transform = Transform3D.IDENTITY


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
