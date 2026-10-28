extends Node
# Checks the simulated Ping360's scan against a flat floor 3 m under the transducer and a wall 8 m
# north of it, its placement of the scan on the map, the chunks it keeps the map in and its log of
# quantized pings.
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
	_check_runs(north)
	_check_record()
	_check_pose()
	_check_chunks()
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

	sonar.basis = north
	# a level for each display bin unlike its neighbours', so each is a cell of its own
	var bins := PackedFloat32Array()
	bins.resize(sonar.samples)
	for b in bins.size():
		bins[b] = 1.0 if b * sonar.display_bins / sonar.samples % 2 else 0.5
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
	sonar.draw_empty = false
	_check("no map cells for silence", sonar.wedge_cells(bins, 0).is_empty(), "")
	sonar.draw_empty = true
	cells = sonar.wedge_cells(bins, 0)
	_check(
		"silence drawn as one dark cell with draw_empty",
		(
			cells.size() == 1
			and cells[0].properties.b0 == 0
			and cells[0].properties.b1 == sonar.samples
			and cells[0].properties.fill == "#" + Ping360.level_colour(0).to_html(false)
		),
		str(cells.map(func(cell: Dictionary) -> Dictionary: return cell.properties))
	)
	sonar.transform = Transform3D.IDENTITY


func _check_runs(north: Basis) -> void:
	var strengths := PackedFloat32Array([0.0, 0.01, 0.5, 0.52, 0.9, 1.0, 1.0, 0.2, 0.0, 0.0])
	var runs := Ping360.level_runs(strengths)
	var expected := PackedInt32Array([0, 2, 0, 2, 4, 8, 4, 5, 14, 5, 7, 15, 7, 8, 3, 8, 10, 0])
	_check("level runs", runs == expected, str(runs))

	var display_bins := sonar.display_bins
	sonar.display_bins = strengths.size()
	sonar.basis = north
	var bins := PackedFloat32Array()
	bins.resize(sonar.samples)
	for b in bins.size():
		bins[b] = strengths[b * strengths.size() / sonar.samples]
	var record := {"k": 1}
	var cells := sonar.wedge_cells(bins, 0, record)
	var covered: bool = (
		cells.size() == expected.size() / 3
		and cells[0].properties.b0 == 0
		and cells[-1].properties.b1 == sonar.samples
	)
	var styled := true
	for n in cells.size():
		var properties: Dictionary = cells[n].properties
		if n > 0 and properties.b0 != cells[n - 1].properties.b1:
			covered = false
		var colour := "#" + Ping360.level_colour(expected[n * 3 + 2]).to_html(false)
		if properties.fill != colour or properties.ping != record:
			styled = false
	var spans := cells.map(
		func(cell: Dictionary) -> Array: return [cell.properties.b0, cell.properties.b1]
	)
	_check("map cells, one per run, cover every bin", covered, str(spans))
	_check(
		"map cells coloured from the palette, carrying the ping",
		styled and cells[3].properties.v == 255 and cells[1].properties.v == 133,
		str(cells.map(func(cell: Dictionary) -> String: return cell.properties.fill))
	)
	# the second run, display bins 2 to 4, reaches from 2 to 4 cell depths north
	var ring: Array = cells[1].geometry.coordinates[0]
	var depth := sonar.range_m / sonar.display_bins
	var inner: float = (ring[0][1] - Ping360.DEFAULT_HOME_LAT) * Ping360.METRES_PER_DEGREE
	var outer: float = (ring[1][1] - Ping360.DEFAULT_HOME_LAT) * Ping360.METRES_PER_DEGREE
	_check(
		"merged map cell spans its run",
		absf(inner - 2 * depth) < 0.02 and absf(outer - 4 * depth) < 0.02,
		"%.3f m to %.3f m" % [inner, outer]
	)
	sonar.draw_empty = false
	cells = sonar.wedge_cells(bins, 0)
	_check(
		"no cells for the weakest level without draw_empty",
		cells.size() == 4,
		"%d cells" % cells.size()
	)
	sonar.draw_empty = true
	sonar.display_bins = display_bins
	sonar.transform = Transform3D.IDENTITY


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


func _check_chunks() -> void:
	var posted: Array[Dictionary] = []
	sonar.poster = func(message: Dictionary) -> void: posted.append(message)
	sonar.max_map_cells = 40
	sonar.map_chunks = 4
	var capacity := 10
	sonar.clear_map()
	var cells: Array[Dictionary] = []
	for n in 7:
		cells.append({"type": "Feature", "n": n})
	var batches := 25
	for batch in batches:
		sonar.add_map_cells(cells)
		sonar.flush_map()

	# what Cockpit would make of the messages, which refuses an append to an overlay it hasn't got
	var on_map := {}
	var last_append := {}
	var problems := []
	var sets := 0
	var id_pattern := RegEx.create_from_string("^[a-z0-9]+(-[a-z0-9]+)*$")
	for index in posted.size():
		var message := posted[index]
		var id: String = message.overlay.id if message.has("overlay") else message.id
		if id_pattern.search(id) == null:
			problems.append("bad id %s" % id)
		if message.type == "cockpit:setMapOverlay":
			if on_map.get(id, 0) > 0:
				# a chunk is only reused for the oldest cells on the map
				for other: String in on_map:
					if on_map[other] > 0 and last_append[other] < last_append[id]:
						problems.append("%d: emptied %s before %s" % [index, id, other])
			on_map[id] = message.overlay.geojson.features.size()
			sets += 1
		elif not on_map.has(id):
			problems.append("%d: append to %s before setting it" % [index, id])
		else:
			on_map[id] += message.features.size()
			last_append[id] = index
			if on_map[id] > capacity:
				problems.append("%d: %s holds %d cells" % [index, id, on_map[id]])
	var total := 0
	for id: String in on_map:
		total += on_map[id]
	var refills := ceili(batches * cells.size() / float(capacity)) - sonar.map_chunks
	_check(
		"map chunks: oldest emptied before reuse, never appended to before set",
		problems.is_empty() and sets == sonar.map_chunks + refills,
		"%d sets, %s" % [sets, problems]
	)
	var stats := sonar.stats()
	_check(
		"map holds the newest cells",
		(
			total == stats.mapCells
			and total <= sonar.max_map_cells
			and total > sonar.max_map_cells - capacity
			and stats.sentFeatures == batches * cells.size()
		),
		"%d on the map, %s" % [total, on_map]
	)

	posted.clear()
	sonar.command({"type": "bluesim:ping360", "command": "clear"})
	var emptied := posted.filter(
		func(message: Dictionary) -> bool:
			return (
				message.type == "cockpit:setMapOverlay"
				and message.overlay.geojson.features.is_empty()
			)
	)
	sonar.add_map_cells(cells)
	sonar.flush_map()
	_check(
		"clear command empties every chunk and restarts at the first",
		(
			emptied.size() == sonar.map_chunks
			and posted.size() == sonar.map_chunks + 1
			and posted[-1].id == "bluesim-ping360-map-0"
			and sonar.stats().mapCells == cells.size()
		),
		str(posted.map(func(message: Dictionary) -> String: return message.type))
	)
	sonar.max_map_cells = 40000
	sonar.clear_map()


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


# A whole turn with the floor and both walls in range, published and logged as in Cockpit
func _check_turn(north: Basis) -> void:
	var posted: Array[Dictionary] = []
	sonar.poster = func(message: Dictionary) -> void: posted.append(message)
	sonar.transform = Transform3D(north, Vector3.ZERO)
	sonar.range_m = 30
	sonar.noise_floor = 0.03
	sonar.head_angle = 0
	sonar.clear_map()
	var before := sonar.stats()
	for n in Ping360.GRADIANS:
		sonar.ping(n * sonar.ping_period())
		if n % 4 == 3:
			sonar.flush_map()
	var stats := sonar.stats()
	var sent: int = stats.sentFeatures - before.sentFeatures
	var sent_bytes: int = stats.sentBytes - before.sentBytes
	var log_json := JSON.stringify(sonar.ping_log()).length()
	_check(
		"a turn mapped and logged",
		(
			stats.revolutions == before.revolutions + 1
			and stats.logPings == Ping360.GRADIANS
			and stats.mapCells == sent
			and stats.avgCellsPerPing >= 1
			and stats.pingBytesBinary == Ping360.RECORD_KEYS.size() * 4 + sonar.samples
		),
		(
			(
				"%d cells (%.1f a ping, %d last), %d bytes posted (%.0f a cell, %.0f a ping);"
				+ " per ping: float JSON %d, quantized JSON %d, binary %d, exported log JSON %.0f;"
				+ " log %d bytes"
			)
			% [
				sent,
				stats.avgCellsPerPing,
				stats.cellsLastPing,
				sent_bytes,
				float(sent_bytes) / sent,
				float(sent_bytes) / Ping360.GRADIANS,
				stats.pingBytesFloatJson,
				stats.pingBytesQuantJson,
				stats.pingBytesBinary,
				float(log_json) / Ping360.GRADIANS,
				stats.logBytes,
			]
		)
	)
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
