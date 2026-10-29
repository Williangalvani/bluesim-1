# Survey / ROV-support boat (v11.1: open grey skiff, with pilot)

A low-poly open fibreglass skiff of about 4.0 m, modelled on Willian's reference photo of the team's boat (photo kept internal).
It has a white hull with a black rub rail, a plain light grey liner, wide flat gunwale caps, a blunt
rounded bow with a bow deck, a thwart bench about 70 % of the way forward, a black rubber mat on the aft floor, a small grey Yamaha
tiller outboard on the transom with a kicker tilted up beside it, aluminium gunwale rails with black clamp blocks and rod holders,
and an oar on the port gunwale. A Blue Robotics Fathom Spool on the aft mat carries the ROV tether's topside end. v11 replaces
the 5.9 m white centre-console of v2-v10; the v10 hull is kept off-repo as a backup.

| file | what |
|---|---|
| `boat.glb` | the model: one mesh, 2140 triangles, 1366 vertices, 8 flat materials (no textures, no UVs). Built procedurally by `build_boat.py` (Blender 4.2: `blender -b --python build_boat.py -- boat.glb`) |
| `boat.tscn` | instanceable boat: `AnimatableBody3D` (group `boats`) + model + two box colliders + the `TetherAnchor` marker + the `Pilot` + the `GroundTackle` |
| `pilot.glb` | the ROV pilot (unchanged v10): 3017 triangles in two meshes, `PilotBody` (2897) and `PilotLaptop` (120), 14 flat materials. Built by `build_pilot.py` (`blender -b --python build_pilot.py -- pilot.glb`) |
| `boat_float.gd` | floating: holds the waterline on `Globals.surface_altitude` and adds heave/pitch/roll; `pose` is the transform it last set |
| `boat_anchor.gd` | ground tackle (`GroundTackle` child in `boat.tscn`): anchor on the seabed ahead of the bow, chain up to the bow eye, pins the boat's horizontal position |
| `boat_demo.tscn` / `.gd` | the sea level (baselevel + reef pack) with its boat astern of the ROV; `-- --boat-shot=/path/prefix` saves screenshots (views framed for the 4 m boat; prints the draft) |

Materials: `boat_hull` (white gelcoat outside, white non-skid bow pads), `boat_liner` (light grey interior,
gunwale caps, bench), `boat_black` (floor mat, rub rail, clamp blocks,
rod holders, tiller, spool frame and flanges), `boat_metal` (aluminium rails, brackets, roller cheeks),
`boat_engine` (Yamaha grey cowls, leg), `boat_wood` (oar blade and grip), `boat_blue` (anodized spool parts),
`boat_tether` (yellow). All are opaque, so v10's alpha-blended windshield is gone.

v11.1 (Willian: "we dont need the veins or the ropes"): the vein strips (and the `boat_vein` material), the coiled blue
bow line and its loose end, the black bungee, the two white rope coils and line on the mat/gunwale, and the black cables
along the transom and port gunwale are removed. The yellow tether from the spool over the transom roller to `TetherAnchor`
and the short yellow extension from the slip ring to the laptop case stay. Nothing else changed: the hull, nodes,
colliders, anchor, pilot placement and waterline are the same as v11. 2577 -> 2140 triangles.

## Frame and layout

The origin is the design waterline on the centreline, amidships. The bow points to -Z (Godot forward), the stern to +Z, and
starboard is +X. Transom at z = +2.0, bow tip at z = -2.0. Beam 1.40 m at the sheer (1.45 m over the rub rail); 1.2 m across the transom.
The hull is a modified V with about 14 degrees of deadrise: keel at y = -0.30 over the aft 55 %, rising to the stem; chine
with a small spray-rail step; flared topsides. Sheer at 0.46, rising to 0.58 at the bow. In plan the bow is a blunt
rounded U.

Inside, from bow to stern:
- a rounded bow deck at 0.28 with white non-skid pads on both bow corners, a dark footwell square starboard of centre and a small cleat;
- a bow eye on the stem at about (0, 0.52, -2.03), where the `GroundTackle`'s chain is made fast;
- the thwart bench across the boat at z = -0.78..-0.98 (top 0.40);
- the cockpit floor at 0.10, with a black rubber mat from the bench to 0.62 m short of the transom (a grey transom step behind it);
- gunwale caps 0.10 m wide, widening to 0.30 m in the bow;
- all interior surfaces in one flat light grey (v11.1: no vein strips).

Starboard gunwale: an aluminium rail with two black clamp blocks, three rod holders and an electronics box at the quarter.
Port gunwale: a rail with two clamp blocks and the oar (handle aft, wooden blade forward).

Outboards:
- **Main:** a small grey Yamaha tiller four-stroke on the transom, 0.2 m to port, tiller forward. Short shaft, so the skeg (-0.35) is only just below the keel.
- **Kicker:** a small grey Yamaha tilted fully up on a bracket on the starboard quarter.

## Fathom Spool

Modelled on Blue Robotics' Fathom Spool (standard, 150 m) product photos and dimension drawing
(bluerobotics.com/store/cables-connectors/tether-management/tms-asm-kit-vp/): 340 mm black HDPE flanges with
five cut-outs, yellow tether wound almost to the rim over blue standoffs, X-shaped black side panels on
feet (375 mm tall, 366 mm footprint), blue handle bar across the top, blue crank arm with a black knob on
one side and the blue slip-ring cap with the Binder 770 connector on the other. It is the same real-size model as v10, with
the flanges cut to 12 segments and 4 wrap ridges to stay inside the triangle budget.

It sits on the starboard side of the aft mat: axle centre at Godot (0.25, 0.31, 1.05), feet on the mat, axle across the boat so the
tether pays off aft, crank outboard. The yellow extension cable runs from the slip ring along the mat to the pilot's laptop case.

## TetherAnchor and the tether run

`Marker3D` named `TetherAnchor` (group `tether_anchors`), child of the boat root:

- local position `(0.25, 0.12, 2.16)`: 16 cm aft of the transom, starboard of centre, 12 cm above the waterline,
  clear of the port-side outboard and the kicker on the starboard quarter (v10: `(0.6, 0.12, 3.06)`).
- orientation unchanged: its -Z axis is the pay-out direction, aft and 60 degrees down, local `(0, -0.866, 0.5)`
  (rotation of -120 degrees about X): `Transform3D(1,0,0, 0,-0.5,0.866, 0,-0.866,-0.5, 0.25,0.12,2.16)`.

The model includes the static topside run of tether (thin yellow tube, part of `boat.glb`):
top of the spool drum (0.25, 0.47, 1.05) -> aft over a black roller between aluminium cheeks on the transom top
(0.25, ~0.52, 1.96) -> down the outside of the transom -> `TetherAnchor` (0.25, 0.12, 2.16). The simulated tether starts at the anchor.

`boat.tether_anchor_position()` returns the anchor's world position (it moves a few cm with the swell).
`rope/spool_tether.gd` pins its last particle to the first node in `tether_anchors` every solve (a placeholder spool if
there is none), and leaves the boat out of the contact queries of the particles next to the spool and of its segment
queries, since the cable leaves the spool through the hull.

In baselevel the boat is at `Transform3D(0,0,1, 0,1,0, -1,0,0, -7.16,3.53,0.25)` (bow towards -X, stern towards the ROV),
which puts the `TetherAnchor` at (-5.00, 3.65, 0.00): 4.97 m behind the ROV's start (0, 3.41, 0), where v10's anchor was.

## Colliders

- `HullCollision`: box 1.45 x 0.88 x 4.05 m at (0, 0.14, 0), from -0.30 to 0.58.
- `ConsoleCollision`: the name is kept so existing references still work, but it is now fitted to the thwart bench: box 1.04 x 0.30 x 0.20 at (0, 0.25, -0.88).

## Pilot

`Pilot` (the instanced `pilot.glb`, a Node3D) is a child of the boat root with two MeshInstance3D children:

- `PilotBody`: a friendly stylised low-poly operator loosely inspired by Blake (not a likeness). Black
  trucker cap with a lower crown, a flat, nearly upright front panel and a flat, slightly curved brim; on
  the front panel a pink fish silhouette facing left (forked tail, black eye notch) above five thin white
  dashes standing in for the lettering. Fair, warm skin; broad round head and jaw, ears, a small nose, open
  eyes (large whites, brown irises, dark pupils, an upper-lid line), brows, an upturned smile with lips, a
  hint of stubble on the chin and jaw; head lifted to look ~17 degrees down at the screen. Tousled
  sandy-brown hair with lighter sun-bleached tips: a volume under the cap plus 20 chunky overlapping wedges
  in two layers whose length, angle and outward flick vary, so the back reads messy rather than a bob.
  Bulky, rumpled grey/white marbled neck gaiter bunched up to the jaw line (below the chin, up to the hair
  at the back) so no bare neck shows, sagging slightly to his left, over a high black jacket collar; a small
  draped, folded flap with a ragged hem over the top of the chest, with irregular darker grey patches. Rounded red foul-weather jacket
  with black trim only (a slim band at the collar, cuffs, front zip) and slimmer red shoulders; two folds
  over the belly; rounded dark trousers with knees, boots. Hands with four fingers wrapped round each
  gamepad grip and thumbs on the sticks. He sits on a grey hard case, bent forward.
- `PilotLaptop`: a second hard case used as a table, with a laptop showing a glowing screen under a black
  fabric sun-shade hood that is open towards the pilot (the laptop-under-a-hood setup used on deck).

Placement: `Transform3D(-1,0,0, 0,1,0, 0,0,-1, -0.18, 0.114, -0.15)`: seated on his case on the black mat, port of
centre between the thwart and the spool, facing aft towards the laptop case, the spool and the transom where the tether goes in
(checked in Blender with a BVH overlap test: no intersection with the floor, mat, bench, spool or tether extension; the cases sit 2 mm above the mat). No collider (the hull box already
covers the deck), so it doesn't change floating or the ROV's collisions. Hide with `$Pilot.visible = false`,
or just the laptop with `$Pilot/PilotLaptop.visible = false`.

Pilot materials: `pilot_skin`, `pilot_lip`, `pilot_stubble`, `pilot_jacket`, `pilot_trousers`, `pilot_hair`, `pilot_hair_tip`,
`pilot_iris`, `pilot_black`
(cap, jacket trim, boots, gamepad, shade hood, pupils), `pilot_case` (hard cases, laptop shell, mouth and lid lines),
`pilot_gaiter` (grey camo), `pilot_white` (white camo, eye whites, logo dashes, buttons), `pilot_pink` (logo),
`pilot_screen` (emissive). Boat + pilot: 5157 triangles; 24 material surfaces (boat 8, PilotBody 13, PilotLaptop 3).

## Floating

BlueSim's sea is a flat plane at `Globals.surface_altitude` (3.53 m; the `water` mesh in baselevel and
`scripts/environment.gd`'s buoyancy both use it). The boat is kinematic: every physics tick
`boat_float.gd` sets y = surface + `waterline_offset` + heave and tilts by pitch/roll, each a sum of sines
(defaults 5 cm, 1.0 deg, 1.8 deg; periods 4.3/5.3/3.7 s), keeping the placed x/z and heading. `swell = 0`
gives a static pose at the surface. Being an AnimatableBody3D the ROV collides with it (hull box
1.45 x 0.88 x 4.05 m, bench box) and can't shove it, so the anchor stays put. (It is not in the `buoyant`
group: environment.gd's buoyancy is a linear spring per buoy with no damping, which suits a submerged ROV
better than a surface hull.)

Measured in `boat_demo.tscn` (WillPC, Godot 4.7.2, gl_compatibility, v11.1): `BOAT_WATERLINE surface=3.53
boat_origin_y=3.557 hull_bottom_y=3.17 hull_top_y=4.474 draft=0.36`. The origin is the surface plus heave at that moment (+2.7 cm), and
`waterline_offset` is 0. The lowest point is the outboard skeg at -0.35 m (keel -0.30 m); the swell's pitch and roll widen the
world AABB a little, so the printed draft depends on the swell phase (with the boat in baselevel, anchored, two runs gave
`boat_origin_y=3.525 hull_bottom_y=3.126 draft=0.404` and `boat_origin_y=3.482 hull_bottom_y=3.08 draft=0.45`, at -0.5 and
-4.8 cm of heave; with the swell off it is the glb's 0.35). So the boat floats at the same level as v10 (0.36 m), and the water meets the hull at the design waterline.
The cockpit floor is 0.10 m above the waterline, so the flat sea plane never shows inside the boat with the default swell.
In baselevel over 70 s the origin stayed between y 3.480 and 3.580 with pitch within +-1.0 and roll within +-1.8 degrees.

`boat_demo.gd`'s screenshot camera is a plain Camera3D. BlueSim only swaps environments for the ROV's
camera (`scripts/camera_waterline.gd`: sky and no fog above the surface), so before v8 the demo shots taken
above the water still used the reef level's underwater `WorldEnvironment` (solid blue background, water fog,
god rays) and looked as if the boat were under water. Each shot now gets
`CameraWaterline.make_surface_environment()` and `underwater_fx.set_fx(false)` when its camera is above the
surface, and the level's environment with the FX on when it is below (`under`). There is also a `waterline`
shot low along the side. The demo prints `BOAT_WATERLINE surface=... boat_origin_y=... hull_bottom_y=... draft=...`.

In the v17 reef none of the 473 kelp strands reaches the surface, and no kelp, rock or coral comes within 4 m of the
hull above its keel; the seabed under the boat is 7.5 m down.

## Ground tackle (anchor and chain)

`GroundTackle` (a `Node3D` child of the boat, script `boat_anchor.gd`) lays an anchor on the seabed and keeps the
boat on it:

- Its `fairlead` (boat frame) is the bow eye, `(0, 0.52, -2.03)`: the export's default and the value set in `boat.tscn`.
- At start (deferred, after `boat_float.gd` has floated the boat) it casts a ray down ahead of the bow to find the
  seabed (level colliders, the boat excluded; `max_depth` 60 m if nothing is hit) and puts the anchor
  `scope` (3) x depth ahead of the bow. In baselevel / reef v17 next to the ROV: 7.5 m deep, 22.5 m ahead of the bow
  eye, 26.7 m from the `TetherAnchor`.
- Every physics tick after `boat_float.gd` (it is the parent, so it runs first) it sets the boat's x/z back to its
  place relative to the anchor on the pose `boat_float.gd` set that tick (`pose`: an AnimatableBody3D's own transform
  reads the last physics step's until the next one, and pinning that froze the swell). Heave, pitch and roll stay,
  horizontal drift is zero (in baselevel the origin moved 0.5 micrometres horizontally over 70 s; the TetherAnchor,
  off the origin, sways 7.9 mm with pitch and roll).
- `swing_degrees` (default 0) lets the boat yaw slowly round the anchor (`swing_period` 90 s), bow on the chain's
  lead. Keep it at 1 degree or less with a tether out: the stern, ~27 m from the anchor, moves ~0.5 m per degree,
  and with the 5.9 m boat 5 degrees (6.5 m of TetherAnchor travel) made the tether solve 7x costlier and peaked it at 184 N.
- Drawing: the anchor is one procedural `ArrayMesh` (stockless anchor: shank, crown, two flukes, shackle and ring,
  148 triangles, ~1 m, `anchor_scale`); the chain is one `MultiMeshInstance3D` of stretched low-poly torus links
  (48 triangles, `link_length` 0.22 m, alternately turned 90 degrees) along a catenary from the bow eye
  (`fairlead`) that lies on the seabed for `chain_on_seabed` (30%) of the run: 113 links / ~5.4k
  instanced triangles in one draw call at 7.5 m. Computed once (again every 6 ticks only while swinging).
- Collision (`chain_collision`, default on): no rigid bodies, but one static `StaticBody3D` `ChainCollider` on
  layer 1 with `chain_capsules` (10) capsules, `chain_radius` 0.03 m, over equal arc-length slices of the chain from
  the bow eye to the anchor, plus a `anchor_radius` 0.35 m sphere at the anchor. Placed with the chain (rebuilt
  only while swinging). The tether rests on the chain and the ROV bumps into it; the kelp still has no collision,
  so the chain passes through kelp.
- Remove the `GroundTackle` node (or `anchor=0` in the spool bench) for a free-floating boat.
