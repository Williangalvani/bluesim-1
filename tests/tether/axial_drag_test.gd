extends SceneTree
# Axial vs perpendicular drag of rope/spool_tether.gd, from its own per-particle kn/kt (_update_masses) and the same
# implicit quadratic drag update _solve uses. A straight cable moving as a whole (along or across its axis) has no
# constraint forces, so one interior particle (1 m of cable) stands for all of it.
#   godot4.7 --headless --path . -s res://tests/tether/axial_drag_test.gd -- cf=0.01,0.005 [hz=120] [pull=0.02] [v0=1]
# Prints one JSON line per cf: kn, kt, ratio, terminal speed under a constant pull (N per m) along / across, and the
# coast from v0: time to half speed and distance in 10 s.

func _args() -> Dictionary:
	var d := {"cf": "0.01,0.005", "hz": "120", "pull": "0.02", "v0": "1.0"}
	for a in OS.get_cmdline_user_args():
		var kv = a.split("=", true, 1)
		if kv.size() == 2:
			d[kv[0]] = kv[1]
	return d


func _initialize():
	var args := _args()
	var h := 1.0 / float(args.hz)
	var pull := float(args.pull)
	var v0 := float(args.v0)
	for cf_s in String(args.cf).split(","):
		var t = load("res://rope/spool_tether.gd").new()
		t.axial_cf = float(cf_s)
		t.rest = PackedFloat64Array([1.0, 1.0, 1.0])
		t._update_masses()
		var i := 1  # interior: half of each 1 m segment next to it
		var im: float = t.inv_m[i]
		var kn: float = t.kn[i]
		var kt: float = t.kt[i]
		var res := {"axial_cf": t.axial_cf, "normal_cd": t.normal_cd, "kn": kn, "kt": kt, "ratio": kn / kt, "hz": 1.0 / h}
		for dir in ["along", "across"]:
			var k := kt if dir == "along" else kn
			# terminal speed under `pull` N per m: integrate until it settles
			var v := 0.0
			for s in range(int(600.0 / h)):
				v += h * pull * im
				v /= 1.0 + h * k * abs(v) * im
			res["vterm_" + dir] = v
			res["vterm_" + dir + "_analytic"] = sqrt(pull / k)
			# coast from v0, no pull
			v = v0
			var x := 0.0
			var t_half := -1.0
			for s in range(int(10.0 / h)):
				v /= 1.0 + h * k * abs(v) * im
				x += v * h
				if t_half < 0 and v <= 0.5 * v0:
					t_half = (s + 1) * h
			res["half_" + dir] = t_half
			res["coast10_" + dir] = x
			res["v10_" + dir] = v
			# drag force at 1 m/s per m of cable
			res["f1_" + dir] = k
		res["vterm_ratio"] = res.vterm_along / res.vterm_across
		res["half_ratio"] = res.half_along / res.half_across if res.half_across > 0 else -1
		t.free()
		print("AXIALDRAG " + JSON.stringify(res))
	quit()
