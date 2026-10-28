extends RefCounted
# What Cockpit makes of the map mesh commands, after src/libs/map/external-overlay-api.ts (schema),
# external-map-meshes.ts (store) and composables/map/useExternalMapOverlays.ts (answers) in
# Cockpit's mesh spike, API v1: apply() returns the reason Cockpit would refuse a message, or "",
# keeps each mesh's slots as Cockpit would and queues the answers in `replies`.

# gdlint: disable=max-returns
const MAX_CAPACITY := 1 << 20
const MAX_WRITE_QUADS := 4096
const MAX_MESSAGE_BYTES := 2 * 1024 * 1024
const MAX_STRIP_SIDE := 2048
const MAX_LATITUDE := 85.05
const MAX_QUAD_DIAGONAL_M := 20000.0
const PALETTE_BYTES := 1024
const METRES_PER_DEGREE := PI / 180 * 6378137
const MESH_TYPES := [
	"cockpit:setMapMesh",
	"cockpit:setMapMeshQuads",
	"cockpit:removeMapMeshQuads",
	"cockpit:clearMapMesh",
	"cockpit:removeMapMesh",
	"cockpit:setMapMeshPalette",
]

# id: {mesh, head, slots: {slot: {corners, uv, texels, tint}}}
var meshes := {}
var problems: Array[String] = []
# answers to the commands with a requestId, in order
var replies: Array[Dictionary] = []
# what cockpit:mapCapabilities says
var version := 1
var drawn := true
var max_capacity := MAX_CAPACITY
var _id_pattern := RegEx.create_from_string("^[a-z0-9]+(-[a-z0-9]+)*$")


# Applies every mesh message of `posted`, collecting what Cockpit would refuse in `problems`
func apply_all(posted: Array) -> Array[String]:
	for index in posted.size():
		var problem := apply(posted[index])
		if problem:
			problems.append("%d %s: %s" % [index, posted[index].get("type"), problem])
	return problems


func apply(message: Dictionary) -> String:
	var type: String = message.get("type", "")
	if type == "cockpit:getMapCapabilities":
		var limits := {"maxCapacity": max_capacity, "maxQuadsPerMessage": MAX_WRITE_QUADS}
		limits.merge({"maxMessageBytes": MAX_MESSAGE_BYTES, "maxTexelSide": MAX_STRIP_SIDE})
		limits.merge({"version": version, "drawn": drawn, "formats": ["r8", "rgba8"]})
		var answer := {"type": "cockpit:mapCapabilities", "requestId": message.requestId}
		answer.meshes = limits
		replies.append(answer)
		return ""
	if type not in MESH_TYPES:
		return ""
	var problem := _apply(message)
	if message.has("requestId"):
		var id: String = message.mesh.id if message.has("mesh") else message.get("id", "")
		var reply := {"type": "cockpit:mapMeshResult", "requestId": message.requestId, "id": id}
		reply.ok = problem == ""
		if problem:
			reply.error = problem
		if meshes.has(id):
			reply.capacity = meshes[id].mesh.capacity
			reply.head = meshes[id].head % reply.capacity
		replies.append(reply)
	return problem


func _apply(message: Dictionary) -> String:
	var type: String = message.type
	if type == "cockpit:setMapMesh":
		return _set_mesh(message.mesh)
	if not _id_pattern.search(String(message.get("id", ""))):
		return "bad id"
	if not meshes.has(message.id):
		return "no mesh %s" % message.id
	var mesh: Dictionary = meshes[message.id]
	match type:
		"cockpit:setMapMeshQuads":
			return _write(mesh, message)
		"cockpit:removeMapMeshQuads":
			if message.first + message.count > mesh.mesh.capacity or message.count < 1:
				return "slots past capacity"
			for slot in range(message.first, message.first + message.count):
				mesh.slots.erase(slot)
		"cockpit:clearMapMesh":
			mesh.slots.clear()
			mesh.head = 0
		"cockpit:removeMapMesh":
			meshes.erase(message.id)
		"cockpit:setMapMeshPalette":
			if not message.palette is PackedByteArray or message.palette.size() != PALETTE_BYTES:
				return "palette of %d bytes" % (message.palette as PackedByteArray).size()
			mesh.mesh.palette = message.palette
	return ""


# The texels of each slot holding a quad, by slot
func slot_texels(id: String) -> Dictionary:
	var out := {}
	for slot: int in meshes[id].slots:
		out[slot] = meshes[id].slots[slot].texels
	return out


func _set_mesh(mesh: Dictionary) -> String:
	if not _id_pattern.search(String(mesh.get("id", ""))):
		return "bad id"
	if not (mesh.get("name") is String and mesh.name != ""):
		return "no name"
	if not (mesh.get("capacity") is int and mesh.capacity >= 1 and mesh.capacity <= max_capacity):
		return "capacity %s" % mesh.get("capacity")
	var texels: Dictionary = mesh.get("texels", {})
	if texels.get("format") not in ["r8", "rgba8"]:
		return "format %s" % texels.get("format")
	for side: String in ["width", "height"]:
		if not (texels.get(side) is int and texels[side] >= 1 and texels[side] <= MAX_STRIP_SIDE):
			return "%s %s" % [side, texels.get(side)]
	if mesh.get("filter", "nearest") not in ["nearest", "linear"]:
		return "filter %s" % mesh.filter
	if mesh.has("palette"):
		if texels.format != "r8":
			return "palette on rgba8"
		if not mesh.palette is PackedByteArray or mesh.palette.size() != PALETTE_BYTES:
			return "palette size"
	if not (float(mesh.get("opacity", 1)) >= 0 and float(mesh.get("opacity", 1)) <= 1):
		return "opacity"
	var previous: Dictionary = meshes.get(mesh.id, {})
	var same: bool = (
		not previous.is_empty()
		and previous.mesh.capacity == mesh.capacity
		and previous.mesh.texels == texels
	)
	if same:
		previous.mesh = mesh
	else:
		meshes[mesh.id] = {"mesh": mesh, "head": 0, "slots": {}}
	return ""


func _write(mesh: Dictionary, message: Dictionary) -> String:
	var corners: Variant = message.get("corners")
	var uv: Variant = message.get("uv")
	var texels: Variant = message.get("texels")
	var tint: Variant = message.get("tint", PackedByteArray())
	if not corners is PackedFloat64Array or not uv is PackedFloat32Array:
		return "corners or uv of the wrong type"
	if not texels is PackedByteArray or not tint is PackedByteArray:
		return "texels or tint of the wrong type"
	var count: int = corners.size() / 8
	if corners.size() % 8 or count < 1:
		return "corners not 8 a quad"
	if count > MAX_WRITE_QUADS:
		return "%d quads in one message" % count
	if uv.size() != count * 8:
		return "uv not 8 a quad"
	if message.has("tint") and tint.size() != count * 4:
		return "tint not 4 a quad"
	var bytes: int = corners.size() * 8 + uv.size() * 4 + texels.size() + tint.size()
	if bytes > MAX_MESSAGE_BYTES:
		return "%d bytes in one message" % bytes
	var spec: Dictionary = mesh.mesh.texels
	var strip: int = spec.width * spec.height * (1 if spec.format == "r8" else 4)
	if texels.size() != count * strip:
		return "texels not %d a quad" % strip
	var capacity: int = mesh.mesh.capacity
	if count > capacity:
		return "more quads than capacity"
	var first: int = message.get("first", mesh.head % capacity)
	if message.has("first") and (not message.first is int or first < 0 or first + count > capacity):
		return "slots %s + %d past capacity %d" % [message.first, count, capacity]
	for quad in count:
		var problem := _corner_problem(corners, quad)
		if problem:
			return "quad %d %s" % [quad, problem]
		for k in 8:
			if not is_finite(uv[quad * 8 + k]):
				return "quad %d uv not finite" % quad
	for quad in count:
		mesh.slots[(first + quad) % capacity] = {
			"corners": corners.slice(quad * 8, quad * 8 + 8),
			"uv": uv.slice(quad * 8, quad * 8 + 8),
			"texels": texels.slice(quad * strip, (quad + 1) * strip),
			"tint": tint.slice(quad * 4, quad * 4 + 4),
		}
	var end := first + count
	mesh.head = ((end - 1) % capacity) + 1
	return ""


static func _corner_problem(corners: PackedFloat64Array, quad: int) -> String:
	var at := quad * 8
	for corner in 4:
		var lon := corners[at + corner * 2]
		var lat := corners[at + corner * 2 + 1]
		if not is_finite(lon) or not is_finite(lat):
			return "corner not finite"
		if absf(lon) > 180 or absf(lat) > MAX_LATITUDE:
			return "corner off the map"
	var scale := cos(deg_to_rad((corners[at + 1] + corners[at + 5]) / 2))
	for pair: Array in [[0, 2], [1, 3]]:
		var east: float = (corners[at + pair[1] * 2] - corners[at + pair[0] * 2]) * scale
		var north: float = corners[at + pair[1] * 2 + 1] - corners[at + pair[0] * 2 + 1]
		if Vector2(east, north).length() * METRES_PER_DEGREE > MAX_QUAD_DIAGONAL_M:
			return "too big"
	return ""
