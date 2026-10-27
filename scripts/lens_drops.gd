extends RefCounted
# The water drops on the camera's dome after it comes out of the water, placed by the meridian they are on and how far
# down it from the dome's top, in radians: the heavy ones run down the dome, wandering and catching on it, and the ones
# after them run into the wet trails the first ones left

const MAX_DROPS = 48
# rad, how big a drop has to be to run, and how much of its radius it leaves behind per radian it runs
const RUNNING_RADIUS = 0.03
const LOSS = 0.005
# rad, either side of the camera's forward, that the map of where drops ran covers
const MAP_AROUND = PI / 2
const MAP_SIZE = 256


class Drop:
	var position: Vector2  # rad, around the dome and down it
	var radius: float  # rad
	var heading = 0.0  # rad, off straight down the dome
	var speed = 0.0  # rad/s
	var ran = false
	var hold: float  # s, before it runs on
	var dries_at: float  # wetness

	func _init(at: Vector2, size: float, delay: float, dries: float):
		position = at
		radius = size
		hold = delay
		dries_at = dries


var drops: Array[Drop] = []
# where water left on the dome above the image keeps feeding drops down it, and when it releases the next
var sources: Array[Vector3] = []
var time = 0.0
# how much a drop swept each spot, and the wetness then, which the shader dries its trail from
var trails = Image.create_empty(MAP_SIZE, MAP_SIZE, false, Image.FORMAT_RG8)
var texture = ImageTexture.create_from_image(trails)


func wet(top: float, bottom: float):
	time = 0.0
	trails.fill(Color.BLACK)
	texture.update(trails)
	drops.clear()
	sources.clear()
	# mostly small drops, which cling until they dry
	for i in 28:
		var at = Vector2(randf_range(-1.1, 1.1), randf_range(top, bottom))
		drops.append(Drop.new(at, 0.008 + 0.05 * randf() ** 2, randf() * 2, randf() * 0.9))
	for i in 4:
		sources.append(Vector3(randf_range(-0.8, 0.8), top, randf()))


func step(delta: float, wetness: float):
	time += delta
	for i in sources.size():
		if wetness > 0.4 and time > sources[i].z and drops.size() < MAX_DROPS:
			var at = Vector2(sources[i].x + randfn(0, 0.01), sources[i].y)
			drops.append(Drop.new(at, randf_range(0.035, 0.06), 0, 0))
			sources[i].z = time + randf_range(0.4, 1.6)
	var painted = false
	for drop in drops:
		if drop.radius >= RUNNING_RADIUS:
			painted = run(drop, delta, wetness) or painted
	merge()
	drops = drops.filter(func(drop): return drop.radius > 0 and wetness > drop.dries_at and on_map(drop.position))
	if painted:
		texture.update(trails)


# returns whether it moved, painting its trail
func run(drop: Drop, delta: float, wetness: float) -> bool:
	if drop.hold > 0:
		drop.hold -= delta
		return false
	var at = drop.position
	var squeeze = maxf(sin(at.y), 0.2)
	var wet_here = swept(at)
	var forward = Vector2(sin(drop.heading) / squeeze, cos(drop.heading))
	var side = Vector2(cos(drop.heading) / squeeze, -sin(drop.heading))
	var reach = drop.radius * 2
	# it wanders off its way, and turns into whichever side ahead of it is wetter
	var pull = swept(at + (forward + side) * reach) - swept(at + (forward - side) * reach)
	drop.heading += randfn(0, 2) * sqrt(delta) + (8 * pull - 2 * drop.heading) * delta
	drop.heading = clampf(drop.heading, -0.6, 0.6)
	# it catches on the dome and slips off again, less so where water already ran
	if randf() < delta * (3 - 2.5 * wet_here):
		drop.hold = randf_range(0.05, 0.3)
		drop.speed = 0
		return false
	var target = 1.2 * sin(at.y) * (drop.radius / RUNNING_RADIUS - 0.7 + wet_here)
	drop.speed = lerpf(drop.speed, target, minf(6 * delta, 1))
	drop.ran = true
	drop.position += forward * drop.speed * delta
	drop.radius -= LOSS * drop.speed * delta
	paint(at, drop.position, drop.radius, 1 - wetness)
	return true


# a running drop gathers up the drops it runs into
func merge():
	for drop in drops:
		if drop.radius < RUNNING_RADIUS or drop.hold > 0:
			continue
		for other in drops:
			if other == drop or other.radius == 0:
				continue
			var apart = drop.position - other.position
			apart.x *= sin(drop.position.y)
			if apart.length() < drop.radius + other.radius:
				drop.radius = pow(drop.radius ** 3 + other.radius ** 3, 1.0 / 3)
				other.radius = 0


func on_map(at: Vector2) -> bool:
	return abs(at.x) < MAP_AROUND and at.y > 0 and at.y < PI - 0.2


func to_map(at: Vector2) -> Vector2:
	return Vector2((at.x / MAP_AROUND + 1) / 2, at.y / PI) * MAP_SIZE


func swept(at: Vector2) -> float:
	if not on_map(at):
		return 0.0
	return trails.get_pixelv(Vector2i(to_map(at))).r


func paint(from: Vector2, to: Vector2, radius: float, when: float):
	var a = to_map(from)
	var b = to_map(to)
	var reach = Vector2(radius / maxf(sin(to.y), 0.2), radius) / (PI / MAP_SIZE)
	var box = Rect2i(Vector2i(a.min(b) - reach), Vector2i((a - b).abs() + reach * 2) + Vector2i(2, 2))
	box = box.intersection(Rect2i(0, 0, MAP_SIZE, MAP_SIZE))
	var along = b - a
	for y in range(box.position.y, box.end.y):
		for x in range(box.position.x, box.end.x):
			var q = Vector2(x + 0.5, y + 0.5) - a
			var nearest = along * clampf(q.dot(along) / maxf(along.length_squared(), 1e-6), 0, 1)
			var swept_here = 1 - ((q - nearest) / reach).length()
			if swept_here > 0:
				trails.set_pixel(x, y, Color(maxf(trails.get_pixel(x, y).r, swept_here), when, 0))


func packed() -> PackedVector4Array:
	var packed = PackedVector4Array()
	packed.resize(MAX_DROPS)
	for i in drops.size():
		var drop = drops[i]
		# in head radii, longer the faster it runs
		var tail = 1.5 + 0.8 * drop.speed if drop.ran else 0.0
		packed[i] = Vector4(drop.position.x, drop.position.y, drop.radius, tail)
	return packed
