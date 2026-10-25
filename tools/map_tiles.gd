extends Node

# Renders the active level straight down into XYZ map tiles, georeferenced the way ArduPilot's SITL places the
# vehicle: world +X is north and +Z is east, in metres from the home location. tools/render_map_tiles.mjs opens
# the web build with ?maptiles&lat=..&lon=.. and collects the tiles this hands to the page.

const TILE_SIZE = 256
# tiles per rendered side; rendering several at once keeps shadows continuous across tile edges
const CHUNK_TILES = 4
# ArduPilot's LATLON_TO_M, so tiles line up with where the autopilot reports the vehicle
const METRES_PER_DEGREE = 111318.84502145034
# open water around the level, also filling lower-zoom tiles past its edge
const BACKGROUND = Color("#0b3a53")

var home_lat: float
var home_lon: float
var max_zoom: int
var min_zoom: int
var tiles = {}


func _ready():
	var params = JSON.parse_string(JavaScriptBridge.eval("JSON.stringify(Object.fromEntries(new URLSearchParams(location.search)))"))
	home_lat = float(params.get("lat", "33.810313"))
	home_lon = float(params.get("lon", "-118.393867"))
	max_zoom = int(params.get("zoom", "22"))
	min_zoom = int(params.get("minzoom", "15"))
	JavaScriptBridge.eval("window.mapTiles = []")
	var level = params.get("level", "reef").get_file()
	if not ProjectSettings.load_resource_pack("res://levels/" + level + ".pck"):
		JavaScriptBridge.eval("window.mapTilesError = %s" % JSON.stringify("level pack %s.pck not found" % level))
		return
	_render.call_deferred()


func _render():
	var viewport = SubViewport.new()
	viewport.own_world_3d = true
	viewport.size = Vector2i.ONE * TILE_SIZE * CHUNK_TILES
	viewport.msaa_3d = Viewport.MSAA_4X
	# the level's tonemapping would shift a rendered background away from the one filling lower zooms
	viewport.transparent_bg = true
	viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	add_child(viewport)

	var level = load("res://custom_level.tscn").instantiate()
	# the underwater effects are what a diver sees; from above they only hide the seafloor
	var effects = level.get_node_or_null("UnderwaterFX")
	if effects:
		level.remove_child(effects)
		effects.free()
	for body in level.find_children("*", "RigidBody3D", true, false):
		body.freeze = true
	viewport.add_child(level)

	for world_environment in level.find_children("*", "WorldEnvironment", true, false):
		var environment = world_environment.environment.duplicate()
		environment.fog_enabled = false
		environment.volumetric_fog_enabled = false
		world_environment.environment = environment
	var bounds = _level_bounds(level)
	for sun in level.find_children("*", "DirectionalLight3D", true, false):
		sun.directional_shadow_max_distance = bounds.size.y + 40

	var camera = Camera3D.new()
	camera.projection = Camera3D.PROJECTION_ORTHOGONAL
	camera.keep_aspect = Camera3D.KEEP_HEIGHT
	# looking down, with north up and east to the right of the image
	camera.basis = Basis(Vector3(0, 0, 1), Vector3(1, 0, 0), Vector3(0, 1, 0))
	camera.position.y = bounds.end.y + 10
	camera.far = bounds.size.y + 20
	viewport.add_child(camera)
	camera.make_current()

	var north_west = _tile_at(bounds.end.x, bounds.position.z, max_zoom)
	var south_east = _tile_at(bounds.position.x, bounds.end.z, max_zoom)
	for chunk_y in range(north_west.y, south_east.y + 1, CHUNK_TILES):
		for chunk_x in range(north_west.x, south_east.x + 1, CHUNK_TILES):
			var north = (_tile_lat(chunk_y, max_zoom) - home_lat) * METRES_PER_DEGREE
			var south = (_tile_lat(chunk_y + CHUNK_TILES, max_zoom) - home_lat) * METRES_PER_DEGREE
			var west = (_tile_lon(chunk_x, max_zoom) - home_lon) * _metres_per_degree_lon()
			var east = (_tile_lon(chunk_x + CHUNK_TILES, max_zoom) - home_lon) * _metres_per_degree_lon()
			camera.position.x = (north + south) / 2
			camera.position.z = (west + east) / 2
			camera.size = north - south
			# one frame to move the camera and one for the shadow maps to follow it
			for i in 2:
				await RenderingServer.frame_post_draw
			var render = viewport.get_texture().get_image()
			render.convert(Image.FORMAT_RGBA8)
			var image = Image.create_empty(render.get_width(), render.get_height(), false, Image.FORMAT_RGBA8)
			image.fill(BACKGROUND)
			image.blend_rect(render, Rect2i(Vector2i.ZERO, render.get_size()), Vector2i.ZERO)
			for y in CHUNK_TILES:
				for x in CHUNK_TILES:
					var tile = image.get_region(Rect2i(x * TILE_SIZE, y * TILE_SIZE, TILE_SIZE, TILE_SIZE))
					_add_tile(max_zoom, chunk_x + x, chunk_y + y, tile)

	for zoom in range(max_zoom - 1, min_zoom - 1, -1):
		_add_parent_tiles(zoom)
	JavaScriptBridge.eval("window.mapTilesDone = true")


func _add_parent_tiles(zoom):
	var parents = {}
	for key in tiles:
		if key.x == zoom + 1:
			parents[Vector2i(key.y >> 1, key.z >> 1)] = true
	for parent in parents:
		var image = Image.create_empty(TILE_SIZE * 2, TILE_SIZE * 2, false, Image.FORMAT_RGB8)
		image.fill(BACKGROUND)
		for y in 2:
			for x in 2:
				var child = tiles.get(Vector3i(zoom + 1, parent.x * 2 + x, parent.y * 2 + y))
				if child:
					image.blit_rect(child, Rect2i(Vector2i.ZERO, child.get_size()), Vector2i(x, y) * TILE_SIZE)
		image.resize(TILE_SIZE, TILE_SIZE, Image.INTERPOLATE_LANCZOS)
		_add_tile(zoom, parent.x, parent.y, image)


func _add_tile(zoom, x, y, image):
	image.convert(Image.FORMAT_RGB8)
	tiles[Vector3i(zoom, x, y)] = image
	var jpeg = Marshalls.raw_to_base64(image.save_jpg_to_buffer(0.85))
	JavaScriptBridge.eval("window.mapTiles.push(['%d/%d/%d.jpg', '%s'])" % [zoom, x, y, jpeg])


func _level_bounds(level):
	var bounds = null
	for node in level.find_children("*", "GeometryInstance3D", true, false):
		if not node.visible:
			continue
		var node_bounds = node.global_transform * node.get_aabb()
		bounds = node_bounds if bounds == null else bounds.merge(node_bounds)
	return bounds


func _metres_per_degree_lon():
	return METRES_PER_DEGREE * cos(deg_to_rad(home_lat))


func _tile_at(north, east, zoom):
	var lat = deg_to_rad(home_lat + north / METRES_PER_DEGREE)
	var lon = home_lon + east / _metres_per_degree_lon()
	var n = 1 << zoom
	return Vector2i(floori((lon + 180) / 360 * n), floori((1 - log(tan(lat) + 1 / cos(lat)) / PI) / 2 * n))


func _tile_lon(x, zoom):
	return float(x) / (1 << zoom) * 360 - 180


func _tile_lat(y, zoom):
	return rad_to_deg(atan(sinh(PI - 2 * PI * float(y) / (1 << zoom))))
