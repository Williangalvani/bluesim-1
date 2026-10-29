extends MeshInstance3D
# Simulated Blue Robotics Ping360 scanning sonar. The head turns in gradians, 400 to a turn, from 0
# at the vehicle's forward and clockwise seen from above, and pings a 2 x 25 degree beam laid in the
# vehicle's horizontal plane. Each ping casts a fan of rays across the beam and turns what they hit
# into echo strength per range bin. The scan feeds the Ping360 HUD panel and, in the web build
# embedded in Cockpit, a sonar map on Cockpit's map that builds up as the vehicle moves. Every ping
# is also kept, quantized, in a log the page can read.
#
# The map is a mesh drawn by Cockpit (cockpit_map_mesh.gd sends it): each ping is one quad, a wedge
# from the transducer out to the range carrying the ping's quantized bins, which Cockpit colours
# through the mesh's palette, opaque out to the range as in Ping Viewer, black for silence, but for
# the first map_blank_m. It keeps the last map_history_s of pings, the oldest overwritten by the
# newest. The beam and the range circle are a small GeoJSON overlay (cockpit PR #3271).
#
# On the web, ?ping360 in the page URL turns it on, and ping360_range, ping360_gain, ping360_step,
# ping360_log_cap, ping360_history_s, ping360_blank_m and ping360_encoding override range_m, gain,
# angle_step, log_cap, map_history_s, map_blank_m and mesh_encoding. home=lat,lon places the scan
# on the map.
#
# window.bluesimPing360Log() returns the log, and the page embedding BlueSim can post it
# {type: "bluesim:ping360", command: "clear" | "clearLog" | "exportLog", requestId}: "exportLog"
# is answered with {type: "bluesim:ping360Log", requestId, log}.

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
const MapMesh := preload("res://vehicles/components/cockpit_map_mesh.gd")
const MAP_ID := "bluesim-ping360-map"
const MAP_NAME := "Ping360 sonar map"
const MAP_OPACITY := 0.9
# slots of the map, whatever the ping rate makes of map_history_s
const MIN_MAP_CAPACITY := 1024
const MAX_MAP_CAPACITY := 16384
# how wide each ping's wedge is on the map, in head steps: wider than a step, so a vehicle turning
# with the head leaves no gaps between them, and still inside the beam
const WEDGE_STEPS := 1.5
# how mesh_encoding gets the map's arrays from Godot, which can only hand JavaScript strings, to
# Cockpit: decoded from base64 here into typed arrays it is given, or as the base64 strings
const TYPED := "typed"
const BASE64 := "base64"
const BEAM_ID := "bluesim-ping360-beam"
const BEAM_COLOUR := "#00ffff"
const FALSE_VALUES := ["0", "false", "off", "no"]
const COMMAND_TYPE := "bluesim:ping360"
const LOG_TYPE := "bluesim:ping360Log"
const LOG_FORMAT := "bluesim-ping360-log/1"
# the whole numbers of a ping's record, in the order the log packs them, before its bins in "i"
const RECORD_KEYS := ["k", "t", "a", "r", "g", "n", "lat", "lon", "d", "yaw", "pitch", "roll"]
const RECORD_UNITS := {
	"k": "ping number",
	"t": "ms of simulation since the sonar started",
	"a": "head angle in gradians, 400 a turn, 0 at the vehicle's forward, clockwise from above",
	"r": "range in cm",
	"g": "gain x 100",
	"n": "range bins",
	"lat": "transducer latitude in 1e-7 degrees",
	"lon": "transducer longitude in 1e-7 degrees",
	"d": "transducer depth below the surface in cm",
	"yaw": "0.1 degrees, 0 north, clockwise",
	"pitch": "0.1 degrees, nose up",
	"roll": "0.1 degrees, right side down",
	"i": "base64 of a byte per range bin out to the range, echo strength 0..1 as 0..255",
}

@export_range(1, 50) var range_m := 30.0
@export var gain := 1.0
@export_range(1, 10) var angle_step := 1  # gradians
@export var samples := 200  # range bins per ping
@export var noise_floor := 0.03
@export var noise_seed := 360
@export var log_cap := 50000  # pings
# s of pings the map keeps, at the ping rate the sonar starts with
@export var map_history_s := 210.0
# left out of the map nearest the transducer, where the real Ping360 still rings from its ping
@export var map_blank_m := 0.5
@export_enum("typed", "base64") var mesh_encoding := TYPED

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
# the sonar map, from start_map() until stop_map()
var map_mesh: MapMesh

var _active := false
var _ping_clock := 0.0
var _clock := 0.0  # s of simulation since the sonar started
var _web := false
var _embedded := false
var _window: JavaScriptObject
var _parent: JavaScriptObject
var _json: JavaScriptObject
# JavaScript can only call back into Godot while these are held
var _callbacks: Array[JavaScriptObject] = []
var _publish_clock := 0.0
var _sent_bytes := 0
var _last_max := 0.0
var _last_bins := PackedFloat32Array()
var _last_time := 0.0
var _last_pose := {}
var _last_record := {}
var _last_quad := {}
# the log: a ring of log_cap pings, their whole numbers and their bins packed one after another
var _log_ints := PackedInt32Array()
var _log_bins := PackedByteArray()
var _log_count := 0
var _log_oldest := 0
var _log_samples := 0


func _init() -> void:
	rng.seed = noise_seed


func _ready() -> void:
	rng.seed = noise_seed
	_web = OS.has_feature("web")
	if _web:
		_read_url()
		_listen()


func _physics_process(delta: float) -> void:
	if not Globals.ping360_enabled:
		if _active:
			_stop()
		return
	if not _active:
		_start()
	_clock += delta
	_ping_clock += delta
	var period := ping_period()
	var count := 0
	while _ping_clock >= period and count < MAX_PINGS_PER_TICK:
		_ping_clock -= period
		# it went out _ping_clock ago, partway through this tick
		ping(_clock - _ping_clock)
		count += 1
	# a slow frame skips pings rather than catching up on them later
	_ping_clock = minf(_ping_clock, period)
	if map_mesh != null:
		map_mesh.tick(delta)
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


# One ping at `time` s since the sonar started, with the head where it is, which then steps on:
# shown on the HUD, logged and queued for the map
func ping(time: float) -> void:
	var bins := scan(get_world_3d().direct_space_state, global_position, global_basis, head_angle)
	pings += 1
	_emit_hud(bins)
	_last_max = 0.0
	for value in bins:
		_last_max = maxf(_last_max, value)
	var levels := quantize(bins)
	_last_bins = bins
	_last_time = time
	_last_pose = pose()
	_last_record = ping_record(levels, head_angle, time, _last_pose)
	log_ping(_last_record, levels)
	if map_mesh != null:
		map_ping(levels, head_angle)
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


# A byte per bin, 0..255 for echo strength 0..1
static func quantize(bins: PackedFloat32Array) -> PackedByteArray:
	var bytes := PackedByteArray()
	bytes.resize(bins.size())
	for b in bins.size():
		bytes[b] = roundi(clampf(bins[b], 0, 1) * 255)
	return bytes


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


# The map texel of a byte of echo strength, 0..255 onto 1..255: 0 is left for no data
static func map_index(level: int) -> int:
	return 1 + roundi(level * 254 / 255.0)


# The map's palette, 256 RGBA entries, straight alpha: 0 transparent, where there is no data, and
# 1..255 opaque, like Ping Viewer's sonar view, black for silence and then sonar_colour() for the
# echo strength map_index() gave them
static func map_palette() -> PackedByteArray:
	var palette := PackedByteArray([0, 0, 0, 0])
	for index in range(1, 256):
		var colour := Color.BLACK if index == 1 else sonar_colour((index - 1) / 254.0)
		palette.append_array([colour.r8, colour.g8, colour.b8, 255])
	return palette


# [lon, lat] of a world position, world +X being north and +Z east in metres from the home
# location, to 1e-7 degrees
static func world_to_lonlat(position: Vector3, lat0: float, lon0: float) -> Array:
	var lonlat := world_to_lonlat_exact(position, lat0, lon0)
	return [snappedf(lonlat[0], 1e-7), snappedf(lonlat[1], 1e-7)]


static func world_to_lonlat_exact(
	position: Vector3, lat0: float, lon0: float
) -> PackedFloat64Array:
	var lat := lat0 + position.x / METRES_PER_DEGREE
	var lon := lon0 + position.z / (METRES_PER_DEGREE * cos(deg_to_rad(lat0)))
	return PackedFloat64Array([lon, lat])


# Where the transducer is and how it is turned: lat and lon in degrees, depth below the surface in
# m, and yaw, pitch and roll in degrees of its forward-right-down axes in north-east-down, as
# BlueROV2Heavy.gd gives ArduSub the vehicle's
func pose() -> Dictionary:
	var lonlat := _lonlat(global_position)
	var axes := global_basis.orthonormalized()
	var surface: float = Globals.surface_altitude
	return {
		"lat": lonlat[1],
		"lon": lonlat[0],
		"d": surface - global_position.y,
		# forward is +Z, right -X and down -Y, and north +X, east +Z and down -Y
		"yaw": fposmod(rad_to_deg(atan2(axes.z.z, axes.z.x)), 360),
		"pitch": rad_to_deg(asin(clampf(axes.z.y, -1, 1))),
		"roll": rad_to_deg(atan2(axes.x.y, axes.y.y)),
	}


# What the log and the map keep of a ping: its quantized bins `levels`, at head `angle` and `time`
# s, from a transducer at `where`, a pose(), all in whole numbers of the units of RECORD_UNITS
func ping_record(levels: PackedByteArray, angle: int, time: float, where: Dictionary) -> Dictionary:
	return {
		"k": pings,
		"t": roundi(time * 1000),
		"a": angle,
		"r": roundi(range_m * 100),
		"g": roundi(gain * 100),
		"n": levels.size(),
		"lat": roundi(float(where.lat) * 1e7),
		"lon": roundi(float(where.lon) * 1e7),
		"d": roundi(float(where.d) * 100),
		"yaw": posmod(roundi(float(where.yaw) * 10), 3600),
		"pitch": roundi(float(where.pitch) * 10),
		"roll": roundi(float(where.roll) * 10),
		"i": Marshalls.raw_to_base64(levels),
	}


# The world direction the head points at `angle` gradians, in the horizontal plane: the sonar's
# yaw, without its roll and pitch, which the map has no room for
func horizontal_direction(angle: float) -> Vector3:
	var forward := Vector3(global_basis.z.x, 0, global_basis.z.z)
	if forward.length_squared() < 1e-6:
		# pointing straight up or down, the top of the vehicle faces where it was heading
		forward = Vector3(global_basis.y.x, 0, global_basis.y.z)
	return forward.normalized().rotated(Vector3.UP, -angle * TAU / GRADIANS)


# Slots for map_history_s of pings at the ping rate now, rounded up to a whole 1024
func map_capacity() -> int:
	var pings_kept := ceili(map_history_s / ping_period() / 1024) * 1024
	return clampi(pings_kept, MIN_MAP_CAPACITY, MAX_MAP_CAPACITY)


# Has Cockpit make the sonar map, empty: the sonar started again, so its ring did
func start_map() -> void:
	map_mesh = MapMesh.new(MAP_ID, MAP_NAME, map_capacity(), "r8", samples)
	map_mesh.filter = "linear"
	map_mesh.palette = map_palette()
	map_mesh.opacity = MAP_OPACITY
	map_mesh.post = _post_mesh
	map_mesh.create()


# Empties the sonar map and starts filling it again from its first slot
func clear_map() -> void:
	if map_mesh != null:
		map_mesh.clear()


func stop_map() -> void:
	if map_mesh != null:
		map_mesh.remove()
	map_mesh = null


# The map quad of one ping, its quantized bins `levels` at head `angle`: a wedge WEDGE_STEPS head
# steps wide from where the transducer is, the way it is heading now, out to range_m, as corners
# [O, O, A, B] in lon, lat, uv along the strip of bins from 0 at the transducer to samples at the
# range, so each bin lies over its own stretch of range, and the bins as map_index() texels, the
# first map_blank_m left out as 0.
func ping_quad(levels: PackedByteArray, angle: float) -> Dictionary:
	var origin := global_position
	var half := WEDGE_STEPS * angle_step / 2.0
	var transducer := _lonlat_exact(origin)
	var corners := PackedFloat64Array()
	corners.append_array(transducer)
	corners.append_array(transducer)
	corners.append_array(_lonlat_exact(origin + horizontal_direction(angle - half) * range_m))
	corners.append_array(_lonlat_exact(origin + horizontal_direction(angle + half) * range_m))
	var width := levels.size()
	var uv := PackedFloat32Array([0, 0.5, 0, 0.5, width, 0.5, width, 0.5])
	var texels := PackedByteArray()
	texels.resize(width)
	var blanked := mini(ceili(map_blank_m / range_m * width), width)
	for b in range(blanked, width):
		texels[b] = map_index(levels[b])
	return {"corners": corners, "uv": uv, "texels": texels}


# Queues a ping's quad, from ping_quad(), for the map
func map_ping(levels: PackedByteArray, angle: float) -> void:
	_last_quad = ping_quad(levels, angle)
	map_mesh.write_quad(_last_quad.corners, _last_quad.uv, _last_quad.texels)


# Adds a ping to the log, its record from ping_record() and its quantized bins, dropping the oldest
# beyond log_cap
func log_ping(record: Dictionary, levels: PackedByteArray) -> void:
	if log_cap <= 0:
		return
	if levels.size() != _log_samples or _log_count > log_cap:
		clear_log()
		_log_samples = levels.size()
	if _log_count < log_cap:
		for key: String in RECORD_KEYS:
			_log_ints.append(record[key])
		_log_bins.append_array(levels)
		_log_count += 1
		return
	var slot := _log_oldest
	_log_oldest = (_log_oldest + 1) % log_cap
	for j in RECORD_KEYS.size():
		_log_ints[slot * RECORD_KEYS.size() + j] = record[RECORD_KEYS[j]]
	for b in _log_samples:
		_log_bins[slot * _log_samples + b] = levels[b]


func clear_log() -> void:
	_log_ints = PackedInt32Array()
	_log_bins = PackedByteArray()
	_log_count = 0
	_log_oldest = 0


# The log, oldest ping first, as window.bluesimPing360Log() gives it
func ping_log() -> Dictionary:
	var records := []
	for n in _log_count:
		var slot := (_log_oldest + n) % _log_count
		var record := {}
		for j in RECORD_KEYS.size():
			record[RECORD_KEYS[j]] = _log_ints[slot * RECORD_KEYS.size() + j]
		var levels := _log_bins.slice(slot * _log_samples, (slot + 1) * _log_samples)
		record.i = Marshalls.raw_to_base64(levels)
		records.append(record)
	return {
		"format": LOG_FORMAT,
		"home": {"lat": home_lat, "lon": home_lon},
		"samples": _log_samples if _log_count else samples,
		"units": RECORD_UNITS,
		"pings": records,
	}


# Carries out a command from the page embedding BlueSim
func command(message: Dictionary) -> void:
	match String(message.get("command", "")):
		"clear":
			# with the sonar off there is no map to clear
			clear_map()
		"clearLog":
			clear_log()
		"exportLog":
			_post({"type": LOG_TYPE, "requestId": message.get("requestId"), "log": ping_log()})


func stats() -> Dictionary:
	var quantized := 0 if _last_record.is_empty() else JSON.stringify(_last_record).length()
	var ping_bytes := Vector2i.ZERO
	if not _last_quad.is_empty():
		var message := {"type": "cockpit:setMapMeshQuads", "id": MAP_ID, "first": 0}
		message.merge(_last_quad)
		ping_bytes = MapMesh.message_bytes(message)
	return {
		"pings": pings,
		"revolutions": revolutions,
		"lastAngle": head_angle,
		"lastMaxIntensity": snappedf(_last_max, 0.001),
		"map": map_mesh.stats() if map_mesh != null else {},
		"pingBytesMeshTyped": ping_bytes.x,
		"pingBytesMeshJson": ping_bytes.y,
		"logPings": _log_count,
		"logBytes": _log_ints.size() * 4 + _log_bins.size(),
		"sentBytes": _sent_bytes,
		"pingBytesFloatJson": _float_record_bytes(),
		"pingBytesQuantJson": quantized,
		"pingBytesBinary": RECORD_KEYS.size() * 4 + _last_bins.size(),
	}


# JSON length of the last ping as it would be without quantizing: float bins and pose
func _float_record_bytes() -> int:
	if _last_bins.is_empty():
		return 0
	var record := {
		"k": pings,
		"t": _last_time,
		"a": _last_record.a,
		"r": range_m,
		"g": gain,
		"n": _last_bins.size(),
	}
	record.merge(_last_pose)
	record.i = Array(_last_bins)
	return JSON.stringify(record, "", false, true).length()


func _lonlat(position: Vector3) -> Array:
	return world_to_lonlat(position, home_lat, home_lon)


func _lonlat_exact(position: Vector3) -> PackedFloat64Array:
	return world_to_lonlat_exact(position, home_lat, home_lon)


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
	if String(params.get("ping360_log_cap", "")).is_valid_int():
		log_cap = clampi(int(params.ping360_log_cap), 0, 500000)
	if String(params.get("ping360_history_s", "")).is_valid_float():
		map_history_s = clampf(float(params.ping360_history_s), 10, 3600)
	if String(params.get("ping360_blank_m", "")).is_valid_float():
		map_blank_m = clampf(float(params.ping360_blank_m), 0, 5)
	if params.has("ping360_encoding"):
		mesh_encoding = BASE64 if String(params.ping360_encoding).to_lower() == BASE64 else TYPED
	var home := String(params.get("home", "")).split(",")
	if home.size() >= 2 and home[0].is_valid_float() and home[1].is_valid_float():
		home_lat = float(home[0])
		home_lon = float(home[1])


# Gives the page window.bluesimPing360Log() and, embedded, takes commands from its parent
func _listen() -> void:
	var log_request := JavaScriptBridge.create_callback(_on_log_request)
	_callbacks.append(log_request)
	_window.bluesimPing360LogRequest = log_request
	# Godot's callbacks return nothing to JavaScript, so the log is left for this to pick up
	var log_function := """
		window.bluesimPing360Log = function () {
			window.bluesimPing360LogRequest();
			const log = window.bluesimPing360LogResult;
			delete window.bluesimPing360LogResult;
			return log;
		};
		"""
	JavaScriptBridge.eval(log_function)
	if not _embedded:
		return
	var on_command := JavaScriptBridge.create_callback(_on_command)
	_callbacks.append(on_command)
	_window.bluesimPing360Command = on_command
	var listener := """
		window.addEventListener("message", (event) => {
			if (event.source === window.parent && event.data && event.data.type === "%s") {
				window.bluesimPing360Command(JSON.stringify(event.data));
			}
		});
		"""
	JavaScriptBridge.eval(listener % COMMAND_TYPE)
	_callbacks.append(MapMesh.install_web(_on_mesh_reply))


func _on_log_request(_args: Array) -> void:
	_window.bluesimPing360LogResult = _json.parse(JSON.stringify(ping_log()))


func _on_command(args: Array) -> void:
	var message: Variant = JSON.parse_string(str(args[0]))
	if typeof(message) == TYPE_DICTIONARY:
		command(message)


func _on_mesh_reply(message: Dictionary) -> void:
	if map_mesh != null:
		map_mesh.handle_reply(message)


func _publishing() -> bool:
	return _embedded or poster.is_valid()


func _start() -> void:
	_active = true
	_ping_clock = 0.0
	_clock = 0.0
	exclude = _vehicle_rids()
	if _publishing():
		start_map()


func _stop() -> void:
	_active = false
	if _publishing():
		stop_map()
		_post({"type": "cockpit:removeMapOverlay", "id": BEAM_ID})
	map_mesh = null


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
		_publish_beam()
	_window.bluesimPing360 = _json.parse(JSON.stringify(stats()))


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


# Posted with its arrays as typed arrays or base64, by mesh_encoding
func _post_mesh(message: Dictionary) -> void:
	var typed := mesh_encoding == TYPED
	_sent_bytes += map_mesh.last_bytes_typed if typed else map_mesh.last_bytes_json
	if poster.is_valid():
		poster.call(message)
	else:
		MapMesh.post_web(message, typed)
