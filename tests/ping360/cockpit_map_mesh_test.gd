extends Node
# Checks the Cockpit map mesh sender on its own: the messages it posts, as Cockpit's schema and
# store (cockpit_mesh_model.gd) take them, its ring of slots, its batching, its encoding and replay.
#   godot4.7 --headless --path . res://tests/ping360/cockpit_map_mesh_test.tscn
# Prints one line per check and exits non-zero if any fails.

const MapMesh = preload("res://vehicles/components/cockpit_map_mesh.gd")
const Model = preload("res://tests/ping360/cockpit_mesh_model.gd")
const ID := "test-mesh"

var failures := 0


func _ready() -> void:
	_check_create()
	_check_ring()
	_check_batches()
	_check_encoding()
	_check_replay()
	_check_remove_and_tint()
	_check_replies()
	print("MESHTEST %s: %d failed" % ["FAIL" if failures else "PASS", failures])
	get_tree().quit(1 if failures else 0)


func _check_create() -> void:
	var posted: Array[Dictionary] = []
	var mesh := _mesh(8, posted)
	mesh.palette.resize(MapMesh.PALETTE_BYTES)
	mesh.create()
	var definition: Dictionary = posted[0].get("mesh", {}) if posted else {}
	_check(
		"create: setMapMesh then clearMapMesh",
		(
			(
				posted.map(func(m: Dictionary) -> String: return m.type)
				== ["cockpit:setMapMesh", "cockpit:clearMapMesh"]
			)
			and definition.texels == {"format": "r8", "width": 3, "height": 1}
			and definition.capacity == 8
			and definition.palette is PackedByteArray
			and posted[1].id == ID
		),
		str(definition.keys())
	)
	_check("create accepted by Cockpit", Model.new().apply_all(posted).is_empty(), "")
	var rgba := MapMesh.new(ID, "Test", 4, "rgba8", 2, 2)
	rgba.palette.resize(MapMesh.PALETTE_BYTES)
	_check(
		"no palette on an rgba8 mesh, strip of 2 x 2 x 4 bytes",
		not rgba.definition().has("palette") and rgba.strip_bytes() == 16,
		""
	)


func _check_ring() -> void:
	var posted: Array[Dictionary] = []
	var mesh := _mesh(5, posted)
	mesh.create()
	var slots := []
	for n in 12:
		slots.append(_write(mesh, n))
		if n % 2:
			mesh.flush()
	var model := Model.new()
	var problems := model.apply_all(posted)
	var held := model.slot_texels(ID)
	var newest := true
	for slot: int in held:
		# slot s holds the newest quad n with n % 5 == s, its texels all n
		var n := 10 + slot if slot < 2 else 5 + slot
		if held[slot] != _texels(n):
			newest = false
	_check(
		"ring wraps: slot = seq % capacity, Cockpit keeps the newest in each",
		(
			slots == [0, 1, 2, 3, 4, 0, 1, 2, 3, 4, 0, 1]
			and problems.is_empty()
			and held.size() == 5
			and newest
			and mesh.seq == 12
			and mesh.stats().slotsWritten == 5
		),
		"%s %s" % [slots, problems]
	)
	posted.clear()
	mesh.clear()
	_write(mesh, 99)
	mesh.flush()
	model.apply_all(posted)
	_check(
		"clear empties Cockpit and restarts at slot 0",
		(
			posted[0].type == "cockpit:clearMapMesh"
			and posted[1].first == 0
			and mesh.seq == 1
			and model.slot_texels(ID).keys() == [0]
		),
		""
	)
	# the same ring with Cockpit picking the slots gives the same slots
	posted.clear()
	var auto := _mesh(5, posted)
	auto.explicit_first = false
	auto.create()
	for n in 12:
		_write(auto, n)
		if n % 3 == 2:
			auto.flush()
	auto.flush()
	var auto_model := Model.new()
	var auto_problems := auto_model.apply_all(posted)
	var firsts := posted.filter(func(m: Dictionary) -> bool: return m.has("first"))
	_check(
		"auto index: no first, Cockpit's slots match the explicit ring",
		auto_problems.is_empty() and firsts.is_empty() and auto_model.slot_texels(ID) == held,
		str(auto_problems)
	)


func _check_batches() -> void:
	var posted: Array[Dictionary] = []
	var mesh := MapMesh.new(ID, "Test", 10000, "r8", 200)
	mesh.post = func(message: Dictionary) -> void: posted.append(message)
	mesh.keep_history = false
	var slots := PackedInt32Array()
	for n in 9000:
		slots.append((n + 9000) % 10000)
	# 9000 quads from slot 9000: 1000 before the wrap, then 4096, 3904
	var runs := mesh.batches(slots, 200)
	_check(
		"batches split at the ring's end and at 4096 quads",
		runs == PackedInt32Array([1000, 4096, 3904]),
		str(runs)
	)
	runs = mesh.batches(PackedInt32Array([3, 4, 5, 9, 10, 2]), 200)
	_check("batches split where slots jump", runs == PackedInt32Array([3, 2, 1]), str(runs))
	# 2048 x 1 rgba8 strips are 8 KiB each, so 2 MiB holds 253 with their corners and uv
	var big := MapMesh.new(ID, "Test", 1000, "rgba8", 2048)
	runs = big.batches(_range(600), big.strip_bytes())
	var per_quad := MapMesh.CORNER_BYTES + MapMesh.UV_BYTES + big.strip_bytes()
	_check(
		"batches keep each message's arrays under 2 MiB",
		(
			runs == PackedInt32Array([253, 253, 94])
			and 253 * per_quad <= MapMesh.MAX_MESSAGE_BYTES
			and 254 * per_quad > MapMesh.MAX_MESSAGE_BYTES
		),
		str(runs)
	)
	mesh.max_quads_per_message = 1000
	runs = mesh.batches(_range(2500), 200)
	_check(
		"batches follow a lower maxQuadsPerMessage",
		runs == PackedInt32Array([1000, 1000, 500]),
		str(runs)
	)
	mesh.max_quads_per_message = MapMesh.MAX_QUADS_PER_MESSAGE
	# a long stall, then one flush: what goes out is what Cockpit takes
	posted.clear()
	mesh.create()
	for n in 9000:
		mesh.write_quad(_corners(n), _uv(200), _texels(n, 200))
	mesh.flush()
	var model := Model.new()
	var problems := model.apply_all(posted)
	var sizes := posted.slice(2).map(
		func(m: Dictionary) -> int: return (m.corners as PackedFloat64Array).size() / 8
	)
	_check(
		"9000 quads in one flush: 3 messages Cockpit takes",
		problems.is_empty() and sizes == [4096, 4096, 808] and model.slot_texels(ID).size() == 9000,
		"%s %s" % [sizes, problems]
	)
	posted.clear()
	mesh.create()
	mesh.write_quad(_corners(1), _uv(200), _texels(1, 200))
	mesh.tick(0.01)
	var held := posted.size()
	mesh.tick(0.01)
	_check(
		"tick holds a write at most flush_interval",
		held == 2 and posted.size() == 3 and mesh.stats().pending == 0,
		"%d then %d" % [held, posted.size()]
	)


func _check_encoding() -> void:
	var posted: Array[Dictionary] = []
	var mesh := MapMesh.new(ID, "Test", 4096, "r8", 200)
	mesh.post = func(message: Dictionary) -> void: posted.append(message)
	mesh.create()
	mesh.write_quad(_corners(7), _uv(200), _texels(7, 200))
	mesh.flush()
	var message: Dictionary = posted[-1]
	var paths := []
	var json := MapMesh.encode(message, paths)
	var corners := Marshalls.base64_to_raw(json.corners).to_float64_array()
	var uv := Marshalls.base64_to_raw(json.uv).to_float32_array()
	var texels := Marshalls.base64_to_raw(json.texels)
	_check(
		"base64 round-trips little-endian float64, float32 and bytes",
		(
			corners == message.corners
			and uv == message.uv
			and texels == message.texels
			and paths == [["corners", "f64"], ["uv", "f32"], ["texels", "u8"]]
		),
		str(paths)
	)
	_check(
		"base64 sizes: 88 + 44 + 268 characters for one 200-bin quad",
		[json.corners.length(), json.uv.length(), json.texels.length()] == [88, 44, 268],
		str([json.corners.length(), json.uv.length(), json.texels.length()])
	)
	var sizes := MapMesh.message_bytes(message)
	var envelope := JSON.stringify(MapMesh.encode(message, [], true)).length()
	_check(
		"message bytes: envelope + 296 typed, envelope + 400 as base64 JSON",
		(
			sizes == Vector2i(envelope + 296, envelope + 400)
			and sizes.y == JSON.stringify(json).length()
			and mesh.last_bytes_typed == sizes.x
		),
		"%s, envelope %d" % [sizes, envelope]
	)
	var palette := PackedByteArray()
	palette.resize(MapMesh.PALETTE_BYTES)
	paths = []
	MapMesh.encode({"type": "cockpit:setMapMesh", "mesh": {"palette": palette}}, paths)
	_check("nested palette encoded at mesh.palette", paths == [["mesh.palette", "u8"]], str(paths))


func _check_replay() -> void:
	var posted: Array[Dictionary] = []
	var mesh := _mesh(6, posted)
	mesh.create()
	for n in 9:
		_write(mesh, n)
	mesh.flush()
	var live := Model.new()
	live.apply_all(posted)
	posted.clear()
	mesh.replay()
	var late := Model.new()
	var problems := late.apply_all(posted)
	var firsts := posted.slice(2).map(func(m: Dictionary) -> int: return m.first)
	_check(
		"replay converges a late Cockpit on the live one, oldest first",
		(
			problems.is_empty()
			and posted[0].type == "cockpit:setMapMesh"
			and posted[1].type == "cockpit:clearMapMesh"
			and firsts == [3, 0]
			and late.slot_texels(ID) == live.slot_texels(ID)
		),
		"%s %s" % [firsts, problems]
	)
	# replayed into the live Cockpit too, nothing changes
	live.apply_all(posted)
	_check("replay is idempotent", live.slot_texels(ID) == late.slot_texels(ID), "")


func _check_remove_and_tint() -> void:
	var posted: Array[Dictionary] = []
	var mesh := _mesh(6, posted)
	mesh.create()
	for n in 4:
		_write(mesh, n)
	mesh.write_quad(_corners(4), _uv(3), _texels(4), PackedByteArray([255, 0, 0, 128]))
	mesh.remove_quads(1, 2)
	var model := Model.new()
	var problems := model.apply_all(posted)
	var write: Dictionary = posted[2]
	_check(
		"a tinted quad sends a tint for every quad, white for the rest",
		(
			problems.is_empty()
			and (
				write.tint
				== PackedByteArray(
					[
						255,
						255,
						255,
						255,
						255,
						255,
						255,
						255,
						255,
						255,
						255,
						255,
						255,
						255,
						255,
						255,
						255,
						0,
						0,
						128
					]
				)
			)
		),
		str(problems)
	)
	posted.clear()
	mesh.replay()
	var late := Model.new()
	late.apply_all(posted)
	_check(
		"removed slots stay empty, here and after replay",
		(
			model.slot_texels(ID).keys() == [0, 3, 4]
			and late.slot_texels(ID) == model.slot_texels(ID)
			and mesh.stats().slotsWritten == 3
		),
		str(model.slot_texels(ID).keys())
	)


func _check_replies() -> void:
	var posted: Array[Dictionary] = []
	var mesh := _mesh(6, posted)
	mesh.ask_capabilities = true
	mesh.request_results = true
	mesh.create()
	var asked: Dictionary = posted[-1]
	var capabilities := {
		"type": "cockpit:mapCapabilities",
		"requestId": asked.get("requestId"),
		"meshes": {"version": 1, "drawn": true, "maxCapacity": 1 << 20, "maxQuadsPerMessage": 512},
	}
	var result := {"type": "cockpit:mapMeshResult", "requestId": posted[0].requestId, "ok": false}
	result.error = "nope"
	_check(
		"capabilities and results taken, their limits followed",
		(
			asked.type == "cockpit:getMapCapabilities"
			and posted[0].requestId != posted[1].requestId
			and mesh.handle_reply(capabilities)
			and mesh.max_quads_per_message == 512
			and mesh.handle_reply(result)
			and mesh.errors == 1
			and mesh.last_error == "nope"
			and not mesh.handle_reply({"type": "cockpit:mapMeshResult", "requestId": "other:1"})
			and not mesh.handle_reply({"type": "bluesim:ping360"})
		),
		str(mesh.stats())
	)


func _mesh(capacity: int, posted: Array[Dictionary]) -> MapMesh:
	var mesh := MapMesh.new(ID, "Test", capacity, "r8", 3)
	mesh.post = func(message: Dictionary) -> void: posted.append(message)
	return mesh


func _write(mesh: MapMesh, n: int) -> int:
	return mesh.write_quad(_corners(n), _uv(3), _texels(n))


# A small quad near home, a little further east for each n
static func _corners(n: int) -> PackedFloat64Array:
	var lon := -118.39 + n * 1e-6
	var lat := 33.81
	return PackedFloat64Array([lon, lat, lon, lat, lon + 1e-5, lat + 1e-5, lon + 2e-5, lat])


static func _uv(width: int) -> PackedFloat32Array:
	return PackedFloat32Array([0, 0.5, 0, 0.5, width, 0.5, width, 0.5])


static func _texels(n: int, width := 3) -> PackedByteArray:
	var texels := PackedByteArray()
	texels.resize(width)
	texels.fill(n % 256)
	return texels


static func _range(count: int) -> PackedInt32Array:
	var slots := PackedInt32Array()
	for n in count:
		slots.append(n)
	return slots


func _check(check_name: String, ok: bool, detail: String) -> void:
	if not ok:
		failures += 1
	print("MESHTEST %s %s %s" % ["PASS" if ok else "FAIL", check_name, detail])
