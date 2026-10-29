extends "res://rope/pbd_tether.gd"
# Benchmark alternative: no simulated cable at all. The vehicle gets a lumped force at the attachment point,
# quadratic drag on a fixed share of the tether as if it moved with the vehicle, plus a spring once the
# vehicle is further from the topside end than the tether is long. The cable is drawn (draw_shape) as a
# parabola with the sag the slack would give, bowed up as a buoyant tether floats, through the same tube mesh.

@export var drag_fraction := 0.5  # share of the deployed length that moves with the vehicle
@export var taut_stiffness := 2000.0  # N/m
@export var taut_damping := 200.0  # N/(m/s)
@export var draw_shape := true


func _solve(_dt):
	var a = attach_point()
	var offset = a - rov.global_position
	var vel = rov.linear_velocity + rov.angular_velocity.cross(offset)
	var chord = anchor - a
	var c = chord.length()
	var dir = chord / c if c > 1e-6 else Vector3.UP
	var l_drag = drag_fraction * length
	var submerged = a.y < Globals.surface_altitude
	var f = Vector3.ZERO
	if submerged:
		var vt = dir * vel.dot(dir)
		var vn = vel - vt
		f -= 0.5 * WATER_DENSITY * normal_cd * diameter * l_drag * vn.length() * vn
		f -= 0.5 * WATER_DENSITY * axial_cf * PI * diameter * l_drag * vt.length() * vt
	if c > length:
		f += dir * (taut_stiffness * (c - length) + taut_damping * max(0.0, -vel.dot(dir)))
	force = f.limit_length(max_force)
	tension = force.length()
	if not draw_shape:
		return
	# arc length of a parabola: L ~ c + 8 s^2 / (3 c)
	var sag = sqrt(max(0.0, 3.0 * c * (length - c) / 8.0)) if c < length else 0.0
	var n = segments
	for i in range(n + 1):
		var t = float(i) / n
		var q = a.lerp(anchor, t)
		q.y = min(q.y + 4.0 * sag * t * (1.0 - t), Globals.surface_altitude)
		x[i] = q
