# Survey / ROV-support boat (v10: with pilot)

Low-poly ~5.9 m white open centre-console, the layout of the white centre-console the team launches the
BlueROV2 from, with a Blue Robotics Fathom Spool on the aft deck carrying the ROV tether's topside end.

| file | what |
|---|---|
| `boat.glb` | the model: one mesh, 2384 triangles, 1536 vertices, 7 flat materials (no textures, no UVs). Built procedurally by `build_boat.py` (Blender 4.2: `blender -b --python build_boat.py -- boat.glb`) |
| `boat.tscn` | instanceable boat: `AnimatableBody3D` (group `boats`) + model + two box colliders + the `TetherAnchor` marker + the `Pilot` |
| `pilot.glb` | the ROV pilot: 3017 triangles in two meshes, `PilotBody` (2897) and `PilotLaptop` (120), 14 flat materials. Built by `build_pilot.py` (`blender -b --python build_pilot.py -- pilot.glb`) |
| `boat_float.gd` | floating: holds the waterline on `Globals.surface_altitude` and adds heave/pitch/roll |
| `boat_anchor.gd` | ground tackle (`GroundTackle` child in `boat.tscn`): anchor on the seabed ahead of the bow, chain up to the bow roller, pins the boat's horizontal position |
| `boat_demo.tscn` / `.gd` | the sea level (baselevel + reef pack) with its boat astern of the ROV; `-- --boat-shot=/path/prefix` saves screenshots |

In the game, `levels/baselevel.tscn` has a `Boat` 5 m astern of where the ROV starts, so the spool tether pays out
from its `TetherAnchor`; `baselevel.gd` drops it for the pool level and the tether benches, which place their own.

Materials: `boat_hull` (white gelcoat), `boat_blue` (bottom paint, sheer stripe, anodized spool parts),
`boat_deck` (grey non-skid pads, cushions), `boat_metal` (stainless), `boat_black` (outboard, rub rail,
spool frame and flanges), `boat_tether` (yellow), `boat_glass` (acrylic windshield, alpha-blended).

## Frame and layout

Origin = design waterline on the centreline, amidships. Bow points to -Z (Godot forward), stern to +Z,
starboard +X. Deep-V hull, keel at y = -0.33, sheer 0.75 (stern) to 1.07 (bow), self-bailing deck at 0.32,
raised bow casting deck at 0.60.

From bow to stern: stainless bow rail and grey casting-deck pad; cooler seat; centre console (z = -0.3)
with a wrap-around acrylic windshield, stainless grab rail, wheel and chartplotter; leaning post with four
rod holders; open aft work deck with a grey non-skid pad and the Fathom Spool; stern grab rails; a
stainless tether roller on the starboard quarter of the transom; outboard on the centreline.

## Fathom Spool

Modelled on Blue Robotics' Fathom Spool (standard, 150 m) product photos and dimension drawing
(bluerobotics.com/store/cables-connectors/tether-management/tms-asm-kit-vp/): 340 mm black HDPE flanges with
five cut-outs, yellow tether wound almost to the rim over blue standoffs, X-shaped black side panels on
feet (375 mm tall, 366 mm footprint), blue handle bar across the top, blue crank arm with a black knob on
one side and the blue slip-ring cap with the Binder 770 connector on the other.

Mounted at Godot (0.6, 0.52, 2.1) (axle centre), axle across the boat so the tether pays off aft, crank
outboard (starboard). The yellow extension cable runs from the slip ring along the deck to the console.

## TetherAnchor and the tether run

`Marker3D` named `TetherAnchor` (group `tether_anchors`), child of the boat root, unchanged from v1:

- local position `(0.6, 0.12, 3.06)`: just aft of the transom on the starboard quarter, 12 cm above the
  waterline, clear of the centreline outboard.
- orientation: its -Z axis is the pay-out direction, aft and 60 degrees down, local `(0, -0.866, 0.5)`
  (rotation of -120 degrees about X): `Transform3D(1,0,0, 0,-0.5,0.866, 0,-0.866,-0.5, 0.6,0.12,3.06)`.

The model includes the static topside run of tether (thin yellow tube, part of `boat.glb`):
top of the spool drum (0.6, 0.68, 2.1) -> aft over the transom roller (0.6, 0.82, 2.88) -> down the
outside of the transom -> `TetherAnchor` (0.6, 0.12, 3.06). The simulated tether starts at the anchor.

`boat.tether_anchor_position()` returns the anchor's world position (it moves a few cm with the swell).

`rope/pbd_tether.gd` currently places its anchored end `anchor_distance` from the ROV at start. To start
it from the boat, look the marker up and pin the last particle to it, for example in `_ready` and in each solve:

```gdscript
var boat_anchor := get_tree().get_first_node_in_group("tether_anchors") as Node3D
if boat_anchor:
    anchor = boat_anchor.global_position   # and x[segments] = anchor for the pinned end
```

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

Placement: `Transform3D(-1,0,0, 0,1,0, 0,0,-1, -0.45, 0.32, 1.55)`: on the port side of the aft deck,
facing aft towards the spool and the transom where the tether goes in. No collider (the hull box already
covers the deck), so it doesn't change floating or the ROV's collisions. Hide with `$Pilot.visible = false`,
or just the laptop with `$Pilot/PilotLaptop.visible = false`.

Pilot materials: `pilot_skin`, `pilot_lip`, `pilot_stubble`, `pilot_jacket`, `pilot_trousers`, `pilot_hair`, `pilot_hair_tip`,
`pilot_iris`, `pilot_black`
(cap, jacket trim, boots, gamepad, shade hood, pupils), `pilot_case` (hard cases, laptop shell, mouth and lid lines),
`pilot_gaiter` (grey camo), `pilot_white` (white camo, eye whites, logo dashes, buttons), `pilot_pink` (logo),
`pilot_screen` (emissive). Boat + pilot: 5401 triangles; 23 material surfaces (boat 7, PilotBody 13, PilotLaptop 3).

## Floating

BlueSim's sea is a flat plane at `Globals.surface_altitude` (3.53 m; the `water` mesh in baselevel and
`scripts/environment.gd`'s buoyancy both use it). The boat is kinematic: every physics tick
`boat_float.gd` sets y = surface + `waterline_offset` + heave and tilts by pitch/roll, each a sum of sines
(defaults 5 cm, 1.0 deg, 1.8 deg; periods 4.3/5.3/3.7 s), keeping the placed x/z and heading. `swell = 0`
gives a static pose at the surface. Being an AnimatableBody3D the ROV collides with it (hull box
2.3 x 1.1 x 5.85 m, console box) and can't shove it, so the anchor stays put. (It is not in the `buoyant`
group: environment.gd's buoyancy is a linear spring per buoy with no damping, which suits a submerged ROV
better than a surface hull.)

Measured in `boat_demo.tscn` (WillPC, Godot 4.7.2): surface 3.53, `water` mesh y 3.53, boat origin y 3.53 +
heave (3.561 at the probe, 3.558 at the shots), `waterline_offset` 0, hull bottom y 3.169, so the keel is
0.36 m under the surface and the water cuts the hull at the design waterline (bottom paint below, white
topsides above). The boat is not sunk.

`boat_demo.gd`'s screenshot camera is a plain Camera3D. BlueSim only swaps environments for the ROV's
camera (`scripts/camera_waterline.gd`: sky and no fog above the surface), so before v8 the demo shots taken
above the water still used the reef level's underwater `WorldEnvironment` (solid blue background, water fog,
god rays) and looked as if the boat were under water. Each shot now gets
`CameraWaterline.make_surface_environment()` and `underwater_fx.set_fx(false)` when its camera is above the
surface, and the level's environment with the FX on when it is below (`under`). There is also a `waterline`
shot low along the side. The demo prints `BOAT_WATERLINE surface=... boat_origin_y=... hull_bottom_y=...`.

Known level issue (not the boat's): in the reef pack, 146 of the 473 kelp strands are taller than the water
is deep and stick out of the surface (up to 5.75 m, median 1.8 m), 18 of them within 12 m of the boat.
From above the water they look like plants standing in the air. The fix belongs in the reef level build
(clamp strand height to the local depth, or bend the top into a surface canopy).

## Ground tackle (anchor and chain)

`GroundTackle` (a `Node3D` child of the boat, script `boat_anchor.gd`) lays an anchor on the seabed and keeps the
boat on it:

- At start (deferred, after `boat_float.gd` has floated the boat) it casts a ray down ahead of the bow to find the
  seabed (level colliders, the boat excluded; `max_depth` 60 m if nothing is hit) and puts the anchor
  `scope` (3) x depth ahead of the bow. In baselevel / reef next to the ROV: 7.5 m deep, 22.5 m ahead.
- Every physics tick after `boat_float.gd` (it is the parent, so it runs first) it sets the boat's x/z back to its
  place relative to the anchor: heave, pitch and roll stay, horizontal drift is zero (the TetherAnchor moved 0.000 m
  horizontally over 100 s; without the anchor it wobbles ~7 mm with pitch/roll).
- `swing_degrees` (default 0) lets the boat yaw slowly round the anchor (`swing_period` 90 s), bow on the chain's
  lead. Keep it at 1 degree or less with a tether out: the stern, ~30 m from the anchor, moves ~0.5 m per degree,
  and 5 degrees (6.5 m of TetherAnchor travel) made the tether solve 7x costlier and peaked it at 184 N.
- Drawing: the anchor is one procedural `ArrayMesh` (stockless anchor: shank, crown, two flukes, shackle and ring,
  148 triangles, ~1 m, `anchor_scale`); the chain is one `MultiMeshInstance3D` of stretched low-poly torus links
  (48 triangles, `link_length` 0.22 m, alternately turned 90 degrees) along a catenary from the bow roller
  (`fairlead`, boat frame) that lies on the seabed for `chain_on_seabed` (30%) of the run: 113 links / ~5.4k
  instanced triangles in one draw call at 7.5 m. Computed once (again every 6 ticks only while swinging). No rigid
  bodies and no colliders: the ROV and the tether pass through it.
- Remove the `GroundTackle` node (or `anchor=0` in the spool bench) for a free-floating boat.
