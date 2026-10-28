extends MeshInstance3D
# Simulated Blue Robotics Ping360 scanning sonar. The head turns in gradians, 400 to a turn, from 0
# at the vehicle's forward and clockwise seen from above, and pings a 2 x 25 degree beam laid in the
# vehicle's horizontal plane. Each ping casts a fan of rays across the beam and turns what they hit
# into echo strength per range bin. The scan feeds the Ping360 HUD panel and, in the web build
# embedded in Cockpit, a sonar map on Cockpit's map that builds up as the vehicle moves, through the
# map overlay commands Cockpit takes from the pages it embeds (cockpit PR #3271).
#
# On the web, ?ping360 in the page URL turns it on, and ping360_range, ping360_gain, ping360_step,
# ping360_bins, ping360_max_cells and ping360_draw_empty override range_m, gain, angle_step,
# display_bins, max_map_cells and draw_empty. home=lat,lon places the scan on the map.

# gdlint: ignore=signal-name
signal updatePing360Display(angle: int, points: Array)

const GRADIANS := 400
const SOUND_SPEED := 1500.0  # m/s
# what the Ping360 spends on each ping besides waiting for the farthest echo
const PING_OVERHEAD := 0.012  # s
const MAX_PING_RATE := 50.0  # Hz
const MAX_PINGS_PER_TICK := 3
# half-power widths, so the edge rays of the fan count half
const BEAM_WIDTH := deg_to_rad(2.0)
const BEAM_HEIGHT := deg_to_rad(25.0)
const RAYS_ACROSS := 3
const RAYS_UP := 9
# echo of one ray striking square on, before its losses: a wall across the beam saturates, while
# the seafloor where the edge of the beam grazes it reads about a third
const ECHO_SCALE := 4.0
const ABSORPTION := 0.02  # per m
const RAYLEIGH_MEAN := 1.2533141  # sqrt(PI / 2)
const HUD_BINS := 100
# ArduPilot's LATLON_TO_M, as tools/map_tiles.gd uses to georeference the level
const METRES_PER_DEGREE := 111318.84502145034
# the reef level's map tiles are rendered for this home
const DEFAULT_HOME_LAT := 33.810313
const DEFAULT_HOME_LON := -118.393867
const PUBLISH_INTERVAL := 0.2  # s
const MAP_ID := "bluesim-ping360-map-%d"
const MAP_NAME := "Ping360 map %d"
const BEAM_ID := "bluesim-ping360-beam"
const BEAM_COLOUR := "#00ffff"
const FALSE_VALUES := ["0", "false", "off", "no"]
# echo strengths the map tells apart
const LEVELS := 16
const MAP_OPACITY := 0.9

@export_range(1, 50) var range_m := 30.0
@export var gain := 1.0
@export_range(1, 10) var angle_step := 1  # gradians
@export var samples := 200  # range bins per ping
@export var display_bins := 100  # range bins of the map cells
@export var noise_floor := 0.03
@export var noise_seed := 360
# map cells kept on Cockpit's map, in map_chunks overlays: when they are all full the oldest is
# emptied for the newest cells
@export var max_map_cells := 40000
@export var map_chunks := 4
# draws the weakest echoes too, so where the sonar found nothing reads as scanned
@export var draw_empty := true

var head_angle := 0  # gradians
var pings := 0
var revolutions := 0
# rays of the last ping that hit something in range
var last_hits := 0
# bodies the rays pass through: the vehicle carrying the sonar
var exclude: Array[RID] = []
var rng := RandomNumberGenerator.new()
# where the world origin is on the map, in double precision unlike a Vector2
var home_lat := DEFAULT_HOME_LAT
var home_lon := DEFAULT_HOME_LON
# takes the messages meant for the page embedding BlueSim instead of it, when set
var poster := Callable()

var _active := false
var _ping_clock := 0.0
var _palette := PackedStringArray()
var _web := false
var _embedded := false
var _window: JavaScriptObject
var _parent: JavaScriptObject
var _json: JavaScriptObject
var _publish_clock := 0.0
var _pending: Array[Dictionary] = []
# cells on the map in each chunk, empty while there is no map
var _chunk_cells := PackedInt32Array()
var _chunk := 0
var _sent_features := 0
var _sent_bytes := 0
var _mapped_pings := 0
var _mapped_cells := 0
var _cells_last_ping := 0
var _last_max := 0.0


func _init() -> void:
	rng.seed = noise_seed
	for level in LEVELS:
		_palette.append("#" + level_colour(level).to_html(false))


func _ready() -> void:
	rng.seed = noise_seed
	_web = OS.has_feature("web")
	if _web:
		_read_url()


func _physics_process(delta: float) -> void:
	if not Globals.ping360_enabled:
		if _active:
			_stop()
		return
	if not _active:
		_start()
	_ping_clock += delta
	var period := ping_period()
	var count := 0
	while _ping_clock >= period and count < MAX_PINGS_PER_TICK:
		_ping_clock -= period
		ping()
		count += 1
	# a slow frame skips pings rather than catching up on them later
	_ping_clock = minf(_ping_clock, period)
	if not _web:
		return
	_publish_clock += delta
	if _publish_clock >= PUBLISH_INTERVAL:
		_publish_clock = fmod(_publish_clock, PUBLISH_INTERVAL)
		_publish()


# The time between pings: the farthest echo's round trip plus the Ping360's own overhead, at most
# 50 a second
func ping_period() -> float:
	return maxf(2 * clampf(range_m, 1, 50) / SOUND_SPEED + PING_OVERHEAD, 1 / MAX_PING_RATE)


# One ping with the head where it is, which then steps on: shown on the HUD and queued for the map
func ping() -> void:
	var bins := scan(get_world_3d().direct_space_state, global_position, global_basis, head_angle)
	pings += 1
	_emit_hud(bins)
	_last_max = 0.0
	for value in bins:
		_last_max = maxf(_last_max, value)
	if _publishing():
		var cells := wedge_cells(bins, head_angle)
		_pending.append_array(cells)
		_cells_last_ping = cells.size()
		_mapped_cells += cells.size()
		_mapped_pings += 1
	var next := head_angle + angle_step
	if next >= GRADIANS:
		revolutions += 1
		# the vehicle may have picked something up, or dropped it
		exclude = _vehicle_rids()
	head_angle = next % GRADIANS


# One ping with the head at `angle` gradians, from a transducer at `origin` that scans the XZ plane
# of `axes`, 0 along +Z: echo strength 0..1 in each of `samples` range bins out to range_m
func scan(
	space: PhysicsDirectSpaceState3D, origin: Vector3, axes: Basis, angle: float
) -> PackedFloat32Array:
	var bins := PackedFloat32Array()
	bins.resize(samples)
	axes = axes.orthonormalized()
	var bin_size := range_m / samples
	var query := PhysicsRayQueryParameters3D.new()
	query.from = origin
	query.exclude = exclude
	last_hits = 0
	for i in RAYS_ACROSS:
		# one ray at a random spot in each slice of the beam, so pings fill it in rather than draw
		# the fan's rays
		var across := ((i + rng.randf()) / RAYS_ACROSS - 0.5) * BEAM_WIDTH
		var bearing := angle * TAU / GRADIANS + across
		var across_weight := pow(cos(across / BEAM_WIDTH * PI / 2), 2)
		for j in RAYS_UP:
			var up := ((j + rng.randf()) / RAYS_UP - 0.5) * BEAM_HEIGHT
			var horizontal := Vector3(-sin(bearing), 0, cos(bearing)) * cos(up)
			var direction := axes * (horizontal + Vector3(0, sin(up), 0))
			query.to = origin + direction * range_m
			var hit := space.intersect_ray(query)
			if hit.is_empty():
				continue
			var distance := origin.distance_to(hit.position)
			var bin := int(distance / bin_size)
			if bin >= samples:
				continue
			last_hits += 1
			var incidence := absf((hit.normal as Vector3).dot(direction))
			var echo := (
				ECHO_SCALE
				* gain
				* incidence
				/ (1 + ABSORPTION * distance)
				* across_weight
				* pow(cos(up / BEAM_HEIGHT * PI / 2), 2)
			)
			bins[bin] += echo
			if bin > 0:
				bins[bin - 1] += echo / 2
			if bin < samples - 1:
				bins[bin + 1] += echo / 2
	for b in samples:
		var speckle := 0.0
		if bins[b] > 0:
			speckle = sqrt(-2 * log(1 - rng.randf())) / RAYLEIGH_MEAN
		bins[b] = clampf(bins[b] * speckle + noise_floor * rng.randf(), 0, 1)
	return bins


# Strongest echo in each of `count` equal stretches of `bins`
static func resample(bins: PackedFloat32Array, count: int) -> PackedFloat32Array:
	var out := PackedFloat32Array()
	out.resize(count)
	for k in count:
		var first := k * bins.size() / count
		var last := maxi((k + 1) * bins.size() / count, first + 1)
		for b in range(first, last):
			out[k] = maxf(out[k], bins[b])
	return out


# Which of the map's LEVELS an echo strength falls in
static func strength_level(strength: float) -> int:
	return clampi(int(strength * LEVELS), 0, LEVELS - 1)


# The stretches of `strengths` at the same level, three numbers each: first index, index after the
# last, and level
static func level_runs(strengths: PackedFloat32Array) -> PackedInt32Array:
	var runs := PackedInt32Array()
	for k in strengths.size():
		var level := strength_level(strengths[k])
		if runs.is_empty() or runs[runs.size() - 1] != level:
			runs.append(k)
			runs.append(k + 1)
			runs.append(level)
		else:
			runs[runs.size() - 2] = k + 1
	return runs


# Near black for silence, through deep blue, copper and yellow, to white for the strongest echoes,
# like SonarView, so the scan stands out against the seafloor's bright satellite imagery
static func sonar_colour(intensity: float) -> Color:
	var stops := [
		Color(0.0, 0.01, 0.05),
		Color(0.05, 0.15, 0.6),
		Color(0.8, 0.42, 0.15),
		Color(1, 0.85, 0.1),
		Color(1, 1, 1),
	]
	var x := clampf(intensity, 0, 1) * (stops.size() - 1)
	var k := mini(int(x), stops.size() - 2)
	return (stops[k] as Color).lerp(stops[k + 1], x - k)


static func level_colour(level: int) -> Color:
	return sonar_colour(float(level) / (LEVELS - 1))


# [lon, lat] of a world position, world +X being north and +Z east in metres from the home location
static func world_to_lonlat(position: Vector3, lat0: float, lon0: float) -> Array:
	var lat := lat0 + position.x / METRES_PER_DEGREE
	var lon := lon0 + position.z / (METRES_PER_DEGREE * cos(deg_to_rad(lat0)))
	return [snappedf(lon, 1e-7), snappedf(lat, 1e-7)]


# The world direction the head points at `angle` gradians, in the horizontal plane: the sonar's
# yaw, without its roll and pitch, which the map has no room for
func horizontal_direction(angle: float) -> Vector3:
	var forward := Vector3(global_basis.z.x, 0, global_basis.z.z)
	if forward.length_squared() < 1e-6:
		# pointing straight up or down, the top of the vehicle faces where it was heading
		forward = Vector3(global_basis.y.x, 0, global_basis.y.z)
	return forward.normalized().rotated(Vector3.UP, -angle * TAU / GRADIANS)


# The map cells of one ping's echoes: a wedge angle_step wide for each run of display bins at the
# same level out to range_m. They are laid from where the transducer is and the way it is heading
# now, at the ping, so the map builds up as the vehicle moves and turns.
func wedge_cells(bins: PackedFloat32Array, angle: float) -> Array[Dictionary]:
	var cells: Array[Dictionary] = []
	var strengths := resample(bins, display_bins)
	var runs := level_runs(strengths)
	var centre := global_position
	var left := horizontal_direction(angle - angle_step / 2.0)
	var right := horizontal_direction(angle + angle_step / 2.0)
	var depth := range_m / display_bins
	for n in range(0, runs.size(), 3):
		var first := runs[n]
		var last := runs[n + 1]
		var level := runs[n + 2]
		if level == 0 and not draw_empty:
			continue
		var inner_left := _lonlat(centre + left * first * depth)
		var ring := [
			inner_left,
			_lonlat(centre + left * last * depth),
			_lonlat(centre + right * last * depth),
			_lonlat(centre + right * first * depth),
			inner_left,
		]
		var loudest := 0.0
		for k in range(first, last):
			loudest = maxf(loudest, strengths[k])
		var properties := {
			"fill": _palette[level],
			"fill-opacity": MAP_OPACITY,
			"stroke-width": 0,
			"b0": first * bins.size() / display_bins,
			"b1": last * bins.size() / display_bins,
			"v": roundi(loudest * 255),
		}
		cells.append(_feature("Polygon", [ring], properties))
	return cells


# Queues cells for the next batch posted to the map
func add_map_cells(cells: Array[Dictionary]) -> void:
	_pending.append_array(cells)


# Empties the map, all its chunks, and starts filling it again from the first
func clear_map() -> void:
	_pending.clear()
	_reset_map()


# Posts the cells queued since the last batch to the newest chunk of the map until it holds its
# share of max_map_cells, then to the next, emptied first of the oldest cells
func flush_map() -> void:
	if _pending.is_empty():
		return
	if _chunk_cells.size() != map_chunks:
		_reset_map()
	var capacity := maxi(ceili(float(max_map_cells) / map_chunks), 1)
	var start := 0
	while start < _pending.size():
		if _chunk_cells[_chunk] >= capacity:
			_chunk = (_chunk + 1) % map_chunks
			# an empty chunk is still empty on the map from _reset_map()
			if _chunk_cells[_chunk] > 0:
				_empty_chunk(_chunk)
		var count := mini(capacity - _chunk_cells[_chunk], _pending.size() - start)
		var append := {
			"type": "cockpit:appendMapOverlayFeatures",
			"id": MAP_ID % _chunk,
			"features": _pending.slice(start, start + count),
		}
		_post(append)
		_chunk_cells[_chunk] += count
		_sent_features += count
		start += count
	_pending = []


func stats() -> Dictionary:
	var map_cells := 0
	for cells in _chunk_cells:
		map_cells += cells
	return {
		"pings": pings,
		"revolutions": revolutions,
		"sentFeatures": _sent_features,
		"mapCells": map_cells,
		"chunk": _chunk,
		"lastAngle": head_angle,
		"lastMaxIntensity": snappedf(_last_max, 0.001),
		"cellsLastPing": _cells_last_ping,
		"avgCellsPerPing": snappedf(float(_mapped_cells) / maxi(_mapped_pings, 1), 0.01),
		"sentBytes": _sent_bytes,
	}


func _lonlat(position: Vector3) -> Array:
	return world_to_lonlat(position, home_lat, home_lon)


func _read_url() -> void:
	_window = JavaScriptBridge.get_interface("window")
	_json = JavaScriptBridge.get_interface("JSON")
	_embedded = bool(JavaScriptBridge.eval("window.parent !== window"))
	if _embedded:
		_parent = _window.parent
	var search := "JSON.stringify(Object.fromEntries(new URLSearchParams(location.search)))"
	var params: Variant = JSON.parse_string(str(JavaScriptBridge.eval(search)))
	if typeof(params) != TYPE_DICTIONARY:
		return
	if params.has("ping360") and String(params.ping360).to_lower() not in FALSE_VALUES:
		Globals.ping360_enabled = true
	if String(params.get("ping360_range", "")).is_valid_float():
		range_m = clampf(float(params.ping360_range), 1, 50)
	if String(params.get("ping360_gain", "")).is_valid_float():
		gain = maxf(float(params.ping360_gain), 0)
	if String(params.get("ping360_step", "")).is_valid_int():
		angle_step = clampi(int(params.ping360_step), 1, 10)
	if String(params.get("ping360_bins", "")).is_valid_int():
		display_bins = clampi(int(params.ping360_bins), 1, samples)
	if String(params.get("ping360_max_cells", "")).is_valid_int():
		max_map_cells = clampi(int(params.ping360_max_cells), 1000, 500000)
	if params.has("ping360_draw_empty"):
		draw_empty = String(params.ping360_draw_empty).to_lower() not in FALSE_VALUES
	var home := String(params.get("home", "")).split(",")
	if home.size() >= 2 and home[0].is_valid_float() and home[1].is_valid_float():
		home_lat = float(home[0])
		home_lon = float(home[1])


func _publishing() -> bool:
	return _embedded or poster.is_valid()


func _start() -> void:
	_active = true
	_ping_clock = 0.0
	exclude = _vehicle_rids()
	if _publishing():
		clear_map()


func _stop() -> void:
	_active = false
	_pending.clear()
	if _publishing():
		for chunk in _chunk_cells.size():
			_post({"type": "cockpit:removeMapOverlay", "id": MAP_ID % chunk})
		_post({"type": "cockpit:removeMapOverlay", "id": BEAM_ID})
	_chunk_cells.clear()


# Every physics body of the vehicle the sonar is on: its hull, claws and anything else in its scene
func _vehicle_rids() -> Array[RID]:
	var rids: Array[RID] = []
	var body := get_parent()
	while body != null and not body is CollisionObject3D:
		body = body.get_parent()
	if body == null:
		return rids
	var vehicle: Node = body.owner if body.owner != null else body
	if vehicle is CollisionObject3D:
		rids.append((vehicle as CollisionObject3D).get_rid())
	for node in vehicle.find_children("*", "CollisionObject3D", true, false):
		rids.append((node as CollisionObject3D).get_rid())
	return rids


func _emit_hud(bins: PackedFloat32Array) -> void:
	var points := []
	var strengths := resample(bins, HUD_BINS)
	for k in HUD_BINS:
		points.append([k, strengths[k]])
	updatePing360Display.emit(roundi(head_angle * 360.0 / GRADIANS) % 360, points)


func _publish() -> void:
	if _publishing():
		flush_map()
		_publish_beam()
	_window.bluesimPing360 = _json.parse(JSON.stringify(stats()))


# Creates every chunk of the map empty, or empties it
func _reset_map() -> void:
	_chunk_cells.resize(map_chunks)
	_chunk = 0
	for chunk in map_chunks:
		_empty_chunk(chunk)


# Cockpit only appends to an overlay it has, so a chunk is always set before it is appended to
func _empty_chunk(chunk: int) -> void:
	_chunk_cells[chunk] = 0
	_set_overlay(MAP_ID % chunk, MAP_NAME % (chunk + 1), [])


func _publish_beam() -> void:
	var transducer := _lonlat(global_position)
	var end := _lonlat(global_position + horizontal_direction(head_angle) * range_m)
	var line := {"stroke": BEAM_COLOUR, "stroke-width": 2, "stroke-opacity": 0.8}
	var circle := {
		"radius": range_m, "stroke": BEAM_COLOUR, "stroke-opacity": 0.3, "fill-opacity": 0
	}
	var features := [
		_feature("LineString", [transducer, end], line), _feature("Point", transducer, circle)
	]
	_set_overlay(BEAM_ID, "Ping360 beam", features)


func _set_overlay(id: String, overlay_name: String, features: Array) -> void:
	var collection := {"type": "FeatureCollection", "features": features}
	var overlay := {"id": id, "name": overlay_name, "live": true, "geojson": collection}
	_post({"type": "cockpit:setMapOverlay", "overlay": overlay})


static func _feature(type: String, coordinates: Array, properties: Dictionary) -> Dictionary:
	var geometry := {"type": type, "coordinates": coordinates}
	return {"type": "Feature", "geometry": geometry, "properties": properties}


# Posted as an object, not a string: Cockpit picks commands out by their `type`
func _post(message: Dictionary) -> void:
	var text := JSON.stringify(message)
	_sent_bytes += text.length()
	if poster.is_valid():
		poster.call(message)
	else:
		_parent.postMessage(_json.parse(text), "*")
