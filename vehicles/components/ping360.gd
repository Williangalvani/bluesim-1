extends MeshInstance3D
# Simulated Blue Robotics Ping360 scanning sonar. The head turns in gradians, 400 to a turn, from 0
# at the vehicle's forward and clockwise seen from above, and pings a 2 x 25 degree beam laid in the
# vehicle's horizontal plane. Each ping casts a fan of rays across the beam and turns what they hit
# into echo strength per range bin. The scan feeds the Ping360 HUD panel and, in the web build
# embedded in Cockpit, Cockpit's map, through the map overlay commands Cockpit takes from the pages
# it embeds (cockpit PR #3271).
#
# On the web, ?ping360 in the page URL turns it on, and ping360_range, ping360_gain, ping360_step
# and ping360_bins override range_m, gain, angle_step and display_bins. home=lat,lon places the
# scan on the map.

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
const SCAN_IDS := ["bluesim-ping360-scan-a", "bluesim-ping360-scan-b"]
const SCAN_NAMES := ["Ping360 scan A", "Ping360 scan B"]
const BEAM_ID := "bluesim-ping360-beam"
const BEAM_COLOUR := "#00ffff"
const FALSE_VALUES := ["0", "false", "off", "no"]

@export_range(1, 50) var range_m := 20.0
@export var gain := 1.0
@export_range(1, 10) var angle_step := 1  # gradians
@export var samples := 200  # range bins per ping
@export var display_bins := 60  # range bins of the map cells
@export var threshold := 0.08  # weakest echo drawn on the map
@export var noise_floor := 0.03
@export var noise_seed := 360

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
var _sent_features := 0
var _last_max := 0.0


func _init() -> void:
	rng.seed = noise_seed
	for i in 256:
		_palette.append("#" + sonar_colour(i / 255.0).to_html(false))


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
		_ping()
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


# Dark blue for silence, through cyan and yellow, to white for the strongest echoes
static func sonar_colour(intensity: float) -> Color:
	var stops := [Color(0.02, 0.05, 0.3), Color(0, 1, 1), Color(1, 1, 0), Color(1, 1, 1)]
	var x := clampf(intensity, 0, 1) * (stops.size() - 1)
	var k := mini(int(x), stops.size() - 2)
	return (stops[k] as Color).lerp(stops[k + 1], x - k)


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


# The map cells of one ping's echoes: wedges `angle_step` wide and range_m / display_bins deep
func wedge_cells(bins: PackedFloat32Array, angle: float) -> Array[Dictionary]:
	var cells: Array[Dictionary] = []
	var strengths := resample(bins, display_bins)
	var centre := global_position
	var left := horizontal_direction(angle - angle_step / 2.0)
	var right := horizontal_direction(angle + angle_step / 2.0)
	var depth := range_m / display_bins
	for k in display_bins:
		var strength := strengths[k]
		if strength < threshold:
			continue
		var inner_left := _lonlat(centre + left * k * depth)
		var ring := [
			inner_left,
			_lonlat(centre + left * (k + 1) * depth),
			_lonlat(centre + right * (k + 1) * depth),
			_lonlat(centre + right * k * depth),
			inner_left,
		]
		var properties := {
			"fill": _palette[int(strength * 255)],
			"fill-opacity": snappedf(0.35 + 0.6 * strength, 0.01),
			"stroke-width": 0,
		}
		cells.append(_feature("Polygon", [ring], properties))
	return cells


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
	var home := String(params.get("home", "")).split(",")
	if home.size() >= 2 and home[0].is_valid_float() and home[1].is_valid_float():
		home_lat = float(home[0])
		home_lon = float(home[1])


func _start() -> void:
	_active = true
	_ping_clock = 0.0
	_pending.clear()
	exclude = _vehicle_rids()
	if _embedded:
		for buffer in SCAN_IDS.size():
			_set_overlay(SCAN_IDS[buffer], SCAN_NAMES[buffer], [])


func _stop() -> void:
	_active = false
	if _embedded:
		for id: String in SCAN_IDS + [BEAM_ID]:
			_post({"type": "cockpit:removeMapOverlay", "id": id})


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


func _ping() -> void:
	var bins := scan(get_world_3d().direct_space_state, global_position, global_basis, head_angle)
	pings += 1
	_emit_hud(bins)
	_last_max = 0.0
	for value in bins:
		_last_max = maxf(_last_max, value)
	if _embedded:
		_pending.append_array(wedge_cells(bins, head_angle))
	var next := head_angle + angle_step
	if next >= GRADIANS:
		revolutions += 1
		if _embedded:
			# the buffer two revolutions back is emptied for the new one, while the last one stays
			# on the map
			_flush(SCAN_IDS[(revolutions + 1) % 2])
			var buffer := revolutions % 2
			_set_overlay(SCAN_IDS[buffer], SCAN_NAMES[buffer], [])
		# the vehicle may have picked something up, or dropped it
		exclude = _vehicle_rids()
	head_angle = next % GRADIANS


func _emit_hud(bins: PackedFloat32Array) -> void:
	var points := []
	var strengths := resample(bins, HUD_BINS)
	for k in HUD_BINS:
		points.append([k, strengths[k]])
	updatePing360Display.emit(roundi(head_angle * 360.0 / GRADIANS) % 360, points)


func _publish() -> void:
	if _embedded:
		_flush(SCAN_IDS[revolutions % 2])
		_publish_beam()
	var stats := {
		"pings": pings,
		"revolutions": revolutions,
		"sentFeatures": _sent_features,
		"lastAngle": head_angle,
		"lastMaxIntensity": snappedf(_last_max, 0.001),
	}
	_window.bluesimPing360 = _json.parse(JSON.stringify(stats))


func _flush(id: String) -> void:
	if _pending.is_empty():
		return
	_post({"type": "cockpit:appendMapOverlayFeatures", "id": id, "features": _pending})
	_sent_features += _pending.size()
	_pending = []


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
	_parent.postMessage(_json.parse(JSON.stringify(message)), "*")
