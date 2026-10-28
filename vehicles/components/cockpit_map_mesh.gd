extends RefCounted
# Sends one mesh of textured quads to Cockpit's maps through its map mesh commands. It knows nothing
# of what the quads show: a sonar's pings, a grid or an image are all quads with lon/lat corners, uv
# and a strip of texels each. Written to be ported line by line to the JavaScript or Python of a
# sender outside BlueSim, such as a BlueOS extension serving the same commands on its WebSocket.
#
# Message flow, every message a JSON object with a "type", from the sender to Cockpit:
#   1. cockpit:setMapMesh {mesh: {id, name, capacity, texels: {format, width, height}, filter,
#      palette, opacity}} creates the mesh. Sent again with the same capacity and texels it keeps
#      the quads and only changes name, filter, palette and opacity.
#   2. cockpit:clearMapMesh {id} empties every slot: the sender's ring restarted, so must Cockpit's.
#   3. cockpit:setMapMeshQuads {id, first, corners, uv, texels, tint?} for every batch of quads, at
#      most 60 a second: corners are 8 float64 per quad (lon, lat of 4 corners in ring order), uv 8
#      float32 per quad in texels of the quad's own strip (texel centres at +0.5), texels
#      width * height * bytes per texel per quad, tint 4 RGBA bytes per quad. Each message writes
#      consecutive slots first .. first + n - 1, at most MAX_QUADS_PER_MESSAGE quads and
#      MAX_MESSAGE_BYTES of arrays, never across the end of the ring. Without first Cockpit writes
#      after its previous write, wrapping at capacity, but a Cockpit that joins late only converges
#      on explicit slots, so they are sent unless explicit_first is false.
#   4. cockpit:removeMapMeshQuads {id, first, count} empties slots; cockpit:setMapMeshPalette {id,
#      palette} recolours an r8 mesh; cockpit:removeMapMesh {id} removes the mesh.
#   Cockpit keeps meshes in memory only, so a Cockpit that connects later (or reloads without the
#   sender) gets replay(): steps 1 and 2, then every slot this side wrote, oldest first.
# The arrays go as typed arrays (Float64Array, Float32Array, Uint8Array) through postMessage with
# their buffers transferred, or as base64 of their little-endian bytes in JSON, as over a WebSocket.
# Godot hands JavaScript strings only, so on the web both go as base64 in a JSON string, and
# JS_POST decodes the typed ones in the page into typed arrays to post with a transfer list.
#
# Replies (TODO: not in Cockpit's mesh spike yet, so nothing asks for them by default). A command
# with a requestId is to be answered with cockpit:mapMeshResult {requestId, ok, error?}, and
# cockpit:getMapCapabilities {requestId} with cockpit:mapCapabilities {requestId, meshes: {version,
# drawn, maxCapacity, maxQuadsPerMessage, ...}}. handle_reply() takes both; ask_capabilities and
# request_results turn on asking.

const FORMAT_BYTES := {"r8": 1, "rgba8": 4}
const MAX_CAPACITY := 1 << 20
const MAX_QUADS_PER_MESSAGE := 4096
const MAX_MESSAGE_BYTES := 2 * 1024 * 1024
const MAX_STRIP_SIDE := 2048
const PALETTE_BYTES := 256 * 4
const CORNER_BYTES := 8 * 8
const UV_BYTES := 8 * 4
const TINT_BYTES := 4
const WHITE := [255, 255, 255, 255]
const RESULT_TYPE := "cockpit:mapMeshResult"
const CAPABILITIES_TYPE := "cockpit:mapCapabilities"
# kinds of the arrays a message carries, by the packed array type holding them
const KINDS := {TYPE_PACKED_FLOAT64_ARRAY: "f64", TYPE_PACKED_FLOAT32_ARRAY: "f32"}
# posts a message given as JSON with its typed arrays in base64, decoded at the dotted paths given
# as JSON pairs of path and kind
const JS_POST := """
	window.cockpitMapMeshPost = function (text, paths) {
		const message = JSON.parse(text);
		const types = {f64: Float64Array, f32: Float32Array, u8: Uint8Array};
		const transfer = [];
		for (const [path, kind] of JSON.parse(paths)) {
			const keys = path.split(".");
			const last = keys.pop();
			const owner = keys.reduce((object, key) => object[key], message);
			let bytes;
			if (Uint8Array.fromBase64) {
				bytes = Uint8Array.fromBase64(owner[last]);
			} else {
				const binary = atob(owner[last]);
				bytes = new Uint8Array(binary.length);
				for (let i = 0; i < binary.length; i++) {
					bytes[i] = binary.charCodeAt(i);
				}
			}
			const Type = types[kind];
			owner[last] = new Type(bytes.buffer, 0, bytes.byteLength / Type.BYTES_PER_ELEMENT);
			transfer.push(bytes.buffer);
		}
		window.parent.postMessage(message, "*", transfer);
	};
	"""
# hands the replies from the embedding page to window.cockpitMapMeshReply, as JSON
const JS_LISTEN := """
	window.addEventListener("message", (event) => {
		const type = event.data && event.data.type;
		if (event.source === window.parent && (type === "%s" || type === "%s")) {
			window.cockpitMapMeshReply(JSON.stringify(event.data));
		}
	});
	"""

var id: String
var mesh_name: String
var capacity: int
var format: String
var width: int
var height: int
var filter := "linear"
# 256 RGBA entries, straight alpha, for r8 texels; empty for Cockpit's grey ramp
var palette := PackedByteArray()
var opacity := 1.0
# sends `first` with every write, so a Cockpit that joined late converges
var explicit_first := true
# keeps every slot's last write for replay()
var keep_history := true
# a write is held at most this long, s, to go out with the next ones
var flush_interval := 1.0 / 60
var max_quads_per_message := MAX_QUADS_PER_MESSAGE
# TODO: ask once Cockpit answers cockpit:getMapCapabilities and gives requestIds a result
var ask_capabilities := false
var request_results := false
# takes each message, its arrays as PackedFloat64Array, PackedFloat32Array and PackedByteArray
var post := Callable()
# quads written to the ring since the mesh was last emptied; the next goes in slot seq % capacity
var seq := 0
# what Cockpit said it can do, from cockpit:mapCapabilities
var capabilities := {}
var messages := 0
var quads_sent := 0
# bytes posted: as typed arrays (JSON envelope plus raw arrays) and as base64 JSON
var bytes_typed := 0
var bytes_json := 0
var last_bytes_typed := 0
var last_bytes_json := 0
var last_quads := 0
var replays := 0
var results := 0
var errors := 0
var last_error := ""

var _strip_bytes := 0
var _since_flush := 0.0
var _next_request := 0
# quads waiting for flush(), in the order written
var _slots := PackedInt32Array()
var _corners := PackedFloat64Array()
var _uv := PackedFloat32Array()
var _texels := PackedByteArray()
var _tints := PackedByteArray()
var _tinted := false
# the last write of each slot, and the order it was written in, -1 for an empty slot
var _history_corners := PackedFloat64Array()
var _history_uv := PackedFloat32Array()
var _history_texels := PackedByteArray()
var _history_tints := PackedByteArray()
var _history_order := PackedInt64Array()
var _order := 0
var _written := 0
var _white := PackedByteArray(WHITE)


func _init(
	mesh_id: String,
	display_name: String,
	slots: int,
	texel_format: String,
	texel_width: int,
	texel_height := 1
) -> void:
	assert(FORMAT_BYTES.has(texel_format))
	id = mesh_id
	mesh_name = display_name
	capacity = clampi(slots, 1, MAX_CAPACITY)
	format = texel_format
	width = clampi(texel_width, 1, MAX_STRIP_SIDE)
	height = clampi(texel_height, 1, MAX_STRIP_SIDE)
	_strip_bytes = width * height * FORMAT_BYTES[format]
	_reset_history()


# Bytes of one quad's strip of texels
func strip_bytes() -> int:
	return _strip_bytes


# The mesh as cockpit:setMapMesh carries it
func definition() -> Dictionary:
	var mesh := {
		"id": id,
		"name": mesh_name,
		"capacity": capacity,
		"texels": {"format": format, "width": width, "height": height},
		"filter": filter,
		"opacity": opacity,
	}
	if format == "r8" and palette.size() == PALETTE_BYTES:
		mesh.palette = palette
	return mesh


# Has Cockpit make the mesh, empty, and starts the ring again from slot 0
func create() -> void:
	_drop_pending()
	_reset_history()
	seq = 0
	_send({"type": "cockpit:setMapMesh", "mesh": definition()})
	_send({"type": "cockpit:clearMapMesh", "id": id})
	if ask_capabilities:
		request_capabilities()


# Empties the mesh, here and in Cockpit, and starts the ring again from slot 0
func clear() -> void:
	_drop_pending()
	_reset_history()
	seq = 0
	_send({"type": "cockpit:clearMapMesh", "id": id})


func remove() -> void:
	_drop_pending()
	_send({"type": "cockpit:removeMapMesh", "id": id})


func set_palette(colours: PackedByteArray) -> void:
	assert(colours.size() == PALETTE_BYTES)
	palette = colours
	_send({"type": "cockpit:setMapMeshPalette", "id": id, "palette": palette})


# Queues one quad for the next flush(): 8 corner numbers lon, lat in ring order, 8 uv numbers and
# strip_bytes() texels, with an optional RGBA tint. It goes in the ring's next slot, or in `slot`
# for a sender keying slots itself, and the slot is returned.
func write_quad(
	corners: PackedFloat64Array,
	uv: PackedFloat32Array,
	texels: PackedByteArray,
	tint := PackedByteArray(),
	slot := -1
) -> int:
	assert(corners.size() == 8 and uv.size() == 8 and texels.size() == _strip_bytes)
	if slot < 0:
		slot = seq % capacity
		seq += 1
	assert(slot < capacity)
	_slots.append(slot)
	_corners.append_array(corners)
	_uv.append_array(uv)
	_texels.append_array(texels)
	if tint.size() == TINT_BYTES:
		_tints.append_array(tint)
		_tinted = _tinted or tint != _white
	else:
		_tints.append_array(_white)
	if keep_history:
		_remember(slot, _slots.size() - 1)
	return slot


# Empties `count` slots from `first`, after sending what is queued
func remove_quads(first: int, count: int) -> void:
	flush()
	first = clampi(first, 0, capacity - 1)
	count = clampi(count, 1, capacity - first)
	for slot in range(first, first + count):
		if _history_order.size() and _history_order[slot] >= 0:
			_history_order[slot] = -1
			_written -= 1
	_send({"type": "cockpit:removeMapMeshQuads", "id": id, "first": first, "count": count})


# Sends the queued quads once flush_interval has passed since the last time, at most
func tick(delta: float) -> void:
	_since_flush += delta
	if not _slots.is_empty() and _since_flush >= flush_interval:
		flush()


# Sends every queued quad now, in as few messages as Cockpit takes
func flush() -> void:
	_since_flush = 0.0
	var start := 0
	for run in batches(_slots, _strip_bytes + (TINT_BYTES if _tinted else 0)):
		_send(_quads_message(start, run))
		start += run
	_drop_pending()


# How many quads go in each message, in order, for quads in `slots`: consecutive slots, at most
# max_quads_per_message and MAX_MESSAGE_BYTES of arrays at `quad_bytes` of texels and tint each
func batches(slots: PackedInt32Array, quad_bytes: int) -> PackedInt32Array:
	var by_bytes := maxi(MAX_MESSAGE_BYTES / (CORNER_BYTES + UV_BYTES + quad_bytes), 1)
	var most := mini(max_quads_per_message, by_bytes)
	var runs := PackedInt32Array()
	var count := 0
	for n in slots.size():
		if count > 0 and (count >= most or slots[n] != slots[n - 1] + 1):
			runs.append(count)
			count = 0
		count += 1
	if count > 0:
		runs.append(count)
	return runs


# Sends the mesh afresh, as to a Cockpit that has none of it: made and emptied, then every slot
# written since the last create() or clear(), oldest first
func replay() -> void:
	replays += 1
	_drop_pending()
	_send({"type": "cockpit:setMapMesh", "mesh": definition()})
	_send({"type": "cockpit:clearMapMesh", "id": id})
	var order := []
	for slot in _history_order.size():
		if _history_order[slot] >= 0:
			order.append(slot)
	order.sort_custom(func(a: int, b: int) -> bool: return _history_order[a] < _history_order[b])
	for slot: int in order:
		_slots.append(slot)
		_corners.append_array(_history_corners.slice(slot * 8, slot * 8 + 8))
		_uv.append_array(_history_uv.slice(slot * 8, slot * 8 + 8))
		_texels.append_array(_history_texels.slice(slot * _strip_bytes, (slot + 1) * _strip_bytes))
		var tint := _history_tints.slice(slot * TINT_BYTES, (slot + 1) * TINT_BYTES)
		_tints.append_array(tint)
		_tinted = _tinted or tint != _white
	flush()


func request_capabilities() -> void:
	_send({"type": "cockpit:getMapCapabilities", "requestId": _request_id()})


# Takes a reply from Cockpit, true if it was one for map meshes
func handle_reply(message: Dictionary) -> bool:
	match String(message.get("type", "")):
		CAPABILITIES_TYPE:
			var meshes: Variant = message.get("meshes")
			if typeof(meshes) != TYPE_DICTIONARY:
				return true
			capabilities = meshes
			if meshes.has("maxQuadsPerMessage"):
				max_quads_per_message = clampi(
					int(meshes.maxQuadsPerMessage), 1, MAX_QUADS_PER_MESSAGE
				)
			if not meshes.get("drawn", true):
				push_warning("Cockpit cannot draw map meshes, so %s will not show" % id)
			if int(meshes.get("maxCapacity", capacity)) < capacity:
				push_warning(
					(
						"Cockpit holds %d quads a mesh, %s needs %d"
						% [meshes.maxCapacity, id, capacity]
					)
				)
			return true
		RESULT_TYPE:
			if not String(message.get("requestId", "")).begins_with(id + ":"):
				return false
			results += 1
			if not message.get("ok", false):
				errors += 1
				last_error = str(message.get("error", ""))
				# TODO: replay() once Cockpit says which error means it lost the mesh
			return true
	return false


# Gives the page JS_POST and hands `on_reply` each reply from its parent, as a Dictionary for
# handle_reply(). Returns the callback JavaScript calls, to be held while replies are wanted.
static func install_web(on_reply: Callable) -> JavaScriptObject:
	JavaScriptBridge.eval(JS_POST)
	var reply := JavaScriptBridge.create_callback(
		func(args: Array) -> void:
			var message: Variant = JSON.parse_string(str(args[0]))
			if typeof(message) == TYPE_DICTIONARY:
				on_reply.call(message)
	)
	JavaScriptBridge.get_interface("window").cockpitMapMeshReply = reply
	JavaScriptBridge.eval(JS_LISTEN % [RESULT_TYPE, CAPABILITIES_TYPE])
	return reply


# Posts `message` to the page embedding this one, through JS_POST with its arrays as typed arrays,
# or else as JSON with them in base64, as a WebSocket would carry it
static func post_web(message: Dictionary, typed := true) -> void:
	var paths := []
	var text := JSON.stringify(encode(message, paths))
	var window := JavaScriptBridge.get_interface("window")
	if typed:
		window.cockpitMapMeshPost(text, JSON.stringify(paths))
	else:
		window.parent.postMessage(JavaScriptBridge.get_interface("JSON").parse(text), "*")


# `message` as JSON takes it, its arrays as base64 strings, or as "" with `blank`. `paths` gets the
# dotted path and kind of each.
static func encode(message: Dictionary, paths := [], blank := false, prefix := "") -> Dictionary:
	var out := {}
	for key: String in message:
		var value: Variant = message[key]
		var path := prefix + key
		match typeof(value):
			TYPE_DICTIONARY:
				out[key] = encode(value, paths, blank, path + ".")
			TYPE_PACKED_FLOAT64_ARRAY, TYPE_PACKED_FLOAT32_ARRAY, TYPE_PACKED_BYTE_ARRAY:
				paths.append([path, KINDS.get(typeof(value), "u8")])
				out[key] = "" if blank else Marshalls.raw_to_base64(array_bytes(value))
			_:
				out[key] = value
	return out


# The little-endian bytes of a packed array
static func array_bytes(array: Variant) -> PackedByteArray:
	if array is PackedByteArray:
		return array
	return array.to_byte_array()


# Bytes `message` takes as typed arrays (JSON envelope plus the arrays) and as base64 JSON
static func message_bytes(message: Dictionary) -> Vector2i:
	var paths := []
	var envelope := JSON.stringify(encode(message, paths, true)).length()
	var raw := 0
	var base64 := 0
	for pair: Array in paths:
		var value: Variant = message
		for key in String(pair[0]).split("."):
			value = value[key]
		var size := array_bytes(value).size()
		raw += size
		base64 += ceili(size / 3.0) * 4
	return Vector2i(envelope + raw, envelope + base64)


func stats() -> Dictionary:
	return {
		"id": id,
		"capacity": capacity,
		"seq": seq,
		"slotsWritten": _written,
		"pending": _slots.size(),
		"messages": messages,
		"quadsSent": quads_sent,
		"bytesTyped": bytes_typed,
		"bytesJson": bytes_json,
		"lastBytesTyped": last_bytes_typed,
		"lastBytesJson": last_bytes_json,
		"lastQuads": last_quads,
		"replays": replays,
		"results": results,
		"errors": errors,
		"lastError": last_error,
		"capabilities": capabilities,
	}


# The cockpit:setMapMeshQuads of the `count` queued quads from the `start`th
func _quads_message(start: int, count: int) -> Dictionary:
	var message := {"type": "cockpit:setMapMeshQuads", "id": id}
	if explicit_first:
		message.first = _slots[start]
	message.corners = _corners.slice(start * 8, (start + count) * 8)
	message.uv = _uv.slice(start * 8, (start + count) * 8)
	message.texels = _texels.slice(start * _strip_bytes, (start + count) * _strip_bytes)
	if _tinted:
		message.tint = _tints.slice(start * TINT_BYTES, (start + count) * TINT_BYTES)
	return message


func _send(message: Dictionary) -> void:
	if request_results and message.type != "cockpit:getMapCapabilities":
		message.requestId = _request_id()
	var sizes := message_bytes(message)
	messages += 1
	last_bytes_typed = sizes.x
	last_bytes_json = sizes.y
	bytes_typed += sizes.x
	bytes_json += sizes.y
	if message.has("corners"):
		last_quads = (message.corners as PackedFloat64Array).size() / 8
		quads_sent += last_quads
	if post.is_valid():
		post.call(message)


func _request_id() -> String:
	_next_request += 1
	return "%s:%d" % [id, _next_request]


func _remember(slot: int, queued: int) -> void:
	if _history_order[slot] < 0:
		_written += 1
	_history_order[slot] = _order
	_order += 1
	for k in 8:
		_history_corners[slot * 8 + k] = _corners[queued * 8 + k]
		_history_uv[slot * 8 + k] = _uv[queued * 8 + k]
	for k in _strip_bytes:
		_history_texels[slot * _strip_bytes + k] = _texels[queued * _strip_bytes + k]
	for k in TINT_BYTES:
		_history_tints[slot * TINT_BYTES + k] = _tints[queued * TINT_BYTES + k]


func _reset_history() -> void:
	_written = 0
	_order = 0
	if not keep_history:
		_history_order = PackedInt64Array()
		return
	_history_corners.resize(capacity * 8)
	_history_uv.resize(capacity * 8)
	_history_texels.resize(capacity * _strip_bytes)
	_history_tints.resize(capacity * TINT_BYTES)
	_history_order.resize(capacity)
	_history_order.fill(-1)


func _drop_pending() -> void:
	_slots = PackedInt32Array()
	_corners = PackedFloat64Array()
	_uv = PackedFloat32Array()
	_texels = PackedByteArray()
	_tints = PackedByteArray()
	_tinted = false
