# BlueSim survey / ROV-support boat, v2: a ~6 m white open centre-console (the layout of the white centre-console
# ROV-launch boat in the team's reference photos) with a Blue Robotics Fathom Spool on the aft deck.
# Procedural, low poly, flat colours. Run: blender -b --python build_boat.py -- [out.glb]
# Blender coords: +Y = bow, +X = starboard, +Z = up, Z=0 = design waterline. The glTF export maps
# Blender (x,y,z) -> Godot (x,z,-y): bow to -Z, starboard +X, waterline y=0.
import bpy, bmesh, math, sys
from mathutils import Vector, Matrix

OUT = sys.argv[sys.argv.index("--") + 1] if "--" in sys.argv else "boat.glb"
bpy.ops.wm.read_factory_settings(use_empty=True)

def mat(name, rgb, rough=0.6, metal=0.0, alpha=1.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    p = m.node_tree.nodes["Principled BSDF"]
    p.inputs["Base Color"].default_value = (*rgb, 1)
    p.inputs["Roughness"].default_value = rough
    p.inputs["Metallic"].default_value = metal
    if alpha < 1:
        p.inputs["Alpha"].default_value = alpha
        m.blend_method = "BLEND"
    m.use_backface_culling = alpha == 1
    return m

M = {
 "hull":   mat("boat_hull",   (0.62, 0.63, 0.61), 0.4),        # white gelcoat: hull, deck, console, seats
 "blue":   mat("boat_blue",   (0.01, 0.16, 0.48), 0.5),        # bottom paint, sheer stripe, anodized spool parts
 "deck":   mat("boat_deck",   (0.17, 0.18, 0.19), 0.9),        # grey non-skid pads and cushions
 "metal":  mat("boat_metal",  (0.70, 0.71, 0.72), 0.3, 0.85),  # stainless rails, rod holders, roller
 "black":  mat("boat_black",  (0.03, 0.03, 0.035), 0.55),      # outboard, rub rail, wheel, spool HDPE frame
 "yellow": mat("boat_tether", (0.98, 0.82, 0.05), 0.6),        # Fathom tether
 "glass":  mat("boat_glass",  (0.55, 0.65, 0.70), 0.05, 0.0, alpha=0.35),  # acrylic windshield
}
order = list(M.keys())
bm = bmesh.new()

def face(verts, m):
    f = bm.faces.new(verts); f.material_index = order.index(m); f.smooth = False; return f

# ------------------------------------------------------------------ hull
stations = [-2.9, -2.5, -1.8, -1.0, -0.2, 0.6, 1.3, 1.9, 2.35, 2.7, 2.95]
L = 5.85
def section(y):
    t = (y + 2.9) / L
    bow = max(0.0, (t - 0.5) / 0.5)
    b = 1.15 * (1 - bow ** 1.9) + 0.03            # half beam at the sheer
    sheer = 0.75 + 0.32 * t ** 2.4                # sheer line, rising to the bow
    keel = -0.33 + 0.85 * bow ** 1.7              # deep-V keel rising into a raked stem
    chine_x = 0.9 * b
    chine_z = max(keel + 0.32 * (1 - bow), 0.0) + bow * 0.3   # ~20 deg deadrise, chine near the waterline
    pts = [(0.0, keel), (0.5 * chine_x, keel + 0.5 * (chine_z - keel) * 0.95), (chine_x, chine_z),
           (chine_x + 0.05 * b, chine_z + 0.02), (0.985 * b, sheer - 0.16), (b, sheer)]
    return pts, sheer
def deck_z(y):
    return 0.32 if y < 1.3 else 0.32 + 0.28 * min(1.0, (y - 1.3) / 0.6)   # raised bow casting deck

rings = []
for y in stations:
    pts, sheer = section(y)
    half = [Vector((x, y, z)) for x, z in pts]
    full = [half[0]] + half[1:] + [Vector((-v.x, v.y, v.z)) for v in reversed(half[1:])]
    rings.append([bm.verts.new(v) for v in full])
n = len(rings[0])          # keel, s1..s5 (s5 = sheer stbd), p5..p1; s4->s5 is the sheer stripe
S, P = 5, 6                # sheer stbd / port indices
for i in range(len(rings) - 1):
    a, b = rings[i], rings[i + 1]
    for k in range(n):
        k2 = (k + 1) % n
        if k == S: continue
        zc = (a[k].co.z + a[k2].co.z + b[k].co.z + b[k2].co.z) / 4
        face([a[k], a[k2], b[k2], b[k]], "blue" if zc < 0.03 or k in (S - 1, P) else "hull")
face(list(reversed(rings[-1])), "hull")
face(rings[0], "hull")                                   # transom
deck_rings = []
for i, y in enumerate(stations[:-1]):
    r = rings[i]; s, p = r[S].co, r[P].co
    inset = 0.08; dz = deck_z(y)
    deck_rings.append((r[S], bm.verts.new((s.x - inset, y, s.z)), bm.verts.new((s.x - inset, y, dz)),
                       bm.verts.new((p.x + inset, y, dz)), bm.verts.new((p.x + inset, y, p.z)), r[P]))
for i in range(len(deck_rings) - 1):
    A, B = deck_rings[i], deck_rings[i + 1]
    face([A[0], B[0], B[1], A[1]], "hull")   # gunwale caps
    face([A[1], B[1], B[2], A[2]], "hull")   # liner
    face([A[2], B[2], B[3], A[3]], "hull")   # deck
    face([A[3], B[3], B[4], A[4]], "hull")
    face([A[4], B[4], B[5], A[5]], "hull")
last = deck_rings[-1]; bowr = rings[-1]
face([last[0], last[1], last[2], last[3], last[4], last[5], bowr[P], bowr[S]], "hull")
first = deck_rings[0]
face([first[5], first[4], first[3], first[2], first[1], first[0]], "hull")

# ------------------------------------------------------------------ primitives
def box(c, s, m, rot=None):
    c = Vector(c); vs = []
    for dx in (-1, 1):
        for dy in (-1, 1):
            for dz in (-1, 1):
                v = Vector((dx * s[0] / 2, dy * s[1] / 2, dz * s[2] / 2))
                vs.append(bm.verts.new((rot @ v if rot else v) + c))
    for f in [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]:
        face([vs[i] for i in f], m)

def bar(p0, p1, w, t, m, normal=(1, 0, 0)):
    """flat bar of width w (in the plane normal to `normal`) and thickness t along `normal`"""
    p0, p1 = Vector(p0), Vector(p1); ax = p1 - p0; nrm = Vector(normal).normalized()
    side = ax.cross(nrm).normalized()
    rot = Matrix((side, ax.normalized(), nrm)).transposed()
    box((p0 + p1) / 2, (w, ax.length, t), m, rot)

def tube(p0, p1, r, m, seg=6, caps=True):
    p0, p1 = Vector(p0), Vector(p1); ax = (p1 - p0).normalized()
    up = Vector((0, 0, 1)) if abs(ax.z) < 0.9 else Vector((1, 0, 0))
    u = ax.cross(up).normalized(); w = ax.cross(u); r0, r1 = [], []
    for i in range(seg):
        a = 2 * math.pi * i / seg; o = (u * math.cos(a) + w * math.sin(a)) * r
        r0.append(bm.verts.new(p0 + o)); r1.append(bm.verts.new(p1 + o))
    for i in range(seg):
        j = (i + 1) % seg; face([r0[i], r0[j], r1[j], r1[i]], m)
    if caps: face(list(reversed(r0)), m); face(r1, m)

def polyline(pts, r, m, seg=5):
    for a, b in zip(pts, pts[1:]): tube(a, b, r, m, seg, caps=False)

def lathe_x(cx, cy, cz, profile, m, seg=12):
    """surface of revolution about an X-parallel axis; profile = [(x_offset, radius), ...]"""
    rings_ = []
    for xo, r in profile:
        rings_.append([bm.verts.new((cx + xo, cy + r * math.cos(2 * math.pi * i / seg), cz + r * math.sin(2 * math.pi * i / seg))) for i in range(seg)])
    for a, b in zip(rings_, rings_[1:]):
        for i in range(seg):
            j = (i + 1) % seg; face([a[i], a[j], b[j], b[i]], m)
    face(list(reversed(rings_[0])), m); face(rings_[-1], m)

def ring_x(cx, cy, cz, r_in, r_out, t, m, seg=16):
    """flat annulus (flange rim) about an X-parallel axis, thickness t"""
    V = [[[bm.verts.new((cx + sx * t / 2, cy + r * math.cos(2 * math.pi * i / seg), cz + r * math.sin(2 * math.pi * i / seg)))
           for i in range(seg)] for r in (r_in, r_out)] for sx in (-1, 1)]
    for i in range(seg):
        j = (i + 1) % seg
        face([V[0][0][i], V[0][1][i], V[0][1][j], V[0][0][j]], m)   # -x side
        face([V[1][0][j], V[1][1][j], V[1][1][i], V[1][0][i]], m)   # +x side
        face([V[0][1][i], V[1][1][i], V[1][1][j], V[0][1][j]], m)   # outer rim
        face([V[0][0][j], V[1][0][j], V[1][0][i], V[0][0][i]], m)   # inner rim

# ------------------------------------------------------------------ deck layout (white centre-console)
DZ = deck_z(0)
# grey non-skid pads: aft work deck, either side of the console, bow casting deck
box((0, -1.75, DZ + 0.006), (1.75, 1.6, 0.012), "deck")
for sx in (-0.78, 0.78): box((sx, 0.25, DZ + 0.006), (0.4, 1.6, 0.012), "deck")
box((0, 2.05, deck_z(2.05) + 0.006), (0.95, 0.6, 0.012), "deck")
# centre console with an acrylic windshield wrapped in a stainless grab rail
CY = 0.3
box((0, CY, DZ + 0.43), (0.95, 0.85, 0.86), "hull")
dash = [bm.verts.new(v) for v in [(-0.47, CY - 0.43, DZ + 0.86), (0.47, CY - 0.43, DZ + 0.86), (0.47, CY + 0.1, DZ + 1.0), (-0.47, CY + 0.1, DZ + 1.0)]]
face(dash, "black")
dash_f = [bm.verts.new(v) for v in [(-0.47, CY + 0.1, DZ + 1.0), (0.47, CY + 0.1, DZ + 1.0), (0.47, CY + 0.43, DZ + 0.86), (-0.47, CY + 0.43, DZ + 0.86)]]
face(dash_f, "hull")
for sx in (-1, 1):   # console side fillets up to the dash
    face([bm.verts.new(v) for v in ([(sx * 0.475, CY - 0.43, DZ + 0.86), (sx * 0.475, CY + 0.43, DZ + 0.86), (sx * 0.475, CY + 0.1, DZ + 1.0)] if sx > 0 else
                                     [(sx * 0.475, CY + 0.1, DZ + 1.0), (sx * 0.475, CY + 0.43, DZ + 0.86), (sx * 0.475, CY - 0.43, DZ + 0.86)])], "hull")
ws = [(-0.47, CY + 0.43, DZ + 0.86), (0.47, CY + 0.43, DZ + 0.86), (0.42, CY + 0.2, DZ + 1.32), (-0.42, CY + 0.2, DZ + 1.32)]
face([bm.verts.new(v) for v in ws], "glass")
for sx in (-1, 1):
    face([bm.verts.new(v) for v in [(sx * 0.47, CY + 0.43, DZ + 0.86), (sx * 0.47, CY - 0.05, DZ + 0.92), (sx * 0.42, CY - 0.05, DZ + 1.3), (sx * 0.42, CY + 0.2, DZ + 1.32)]], "glass")
polyline([(-0.44, CY - 0.05, DZ + 1.34), (-0.44, CY + 0.2, DZ + 1.36), (0.44, CY + 0.2, DZ + 1.36), (0.44, CY - 0.05, DZ + 1.34)], 0.018, "metal", 5)
for sx in (-0.44, 0.44): tube((sx, CY - 0.05, DZ + 0.92), (sx, CY - 0.05, DZ + 1.34), 0.016, "metal", 4, caps=False)
tube((0.1, CY - 0.28, DZ + 0.92), (0.1, CY - 0.31, DZ + 0.95), 0.17, "black", 10)       # wheel
box((-0.18, CY - 0.05, DZ + 0.99), (0.3, 0.05, 0.2), "black", Matrix.Rotation(math.radians(-25), 3, "X"))  # chartplotter
# leaning post with rod holders, and the cooler seat in front of the console
box((0, -0.65, DZ + 0.33), (0.8, 0.45, 0.66), "hull")
box((0, -0.62, DZ + 0.72), (0.85, 0.42, 0.12), "deck")
box((0, -0.82, DZ + 0.92), (0.8, 0.1, 0.32), "deck")
for sx in (-0.3, -0.1, 0.1, 0.3): tube((sx, -0.9, DZ + 0.66), (sx, -0.96, DZ + 1.15), 0.025, "metal", 5)
box((0, CY + 0.72, DZ + 0.24), (0.8, 0.5, 0.48), "hull")
box((0, CY + 0.72, DZ + 0.5), (0.82, 0.52, 0.06), "deck")
# stainless bow rail, stern grab rails, rod holders in the gunwale
def sheer_pt(y, side, up=0.0, inn=0.04):
    pts, sh = section(y); return (side * (pts[-1][0] - inn), y, sh + up)
for side in (-1, 1):
    ys = [1.2, 1.9, 2.4, 2.8]
    top = [sheer_pt(y, side, 0.32, 0.06 + 0.02 * k) for k, y in enumerate(ys)]
    polyline(top, 0.016, "metal", 4)
    for y, p in zip(ys, top): tube((p[0], p[1], p[2] - 0.32), p, 0.014, "metal", 4, caps=False)
    rail = [sheer_pt(y, side, 0.14, 0.06) for y in (-2.55, -1.95, -1.35)]
    polyline(rail, 0.016, "metal", 4)
    for p in rail: tube((p[0], p[1], p[2] - 0.14), p, 0.014, "metal", 4, caps=False)
    for y in (-0.3, -1.1):
        p = sheer_pt(y, side, 0.0, 0.05); tube((p[0], p[1], p[2] - 0.12), (p[0], p[1] - 0.04, p[2] + 0.05), 0.026, "metal", 5)
# rub rail along the sheer
for i in range(len(stations) - 2):
    for side in (1, -1):
        (p0, s0), (p1, s1) = section(stations[i]), section(stations[i + 1])
        tube((side * (p0[-1][0] + 0.02), stations[i], s0 - 0.03), (side * (p1[-1][0] + 0.02), stations[i + 1], s1 - 0.03), 0.035, "black", 4, caps=False)
# outboard on the centreline transom (a ~150 hp four-stroke): cowl with a smaller cap, midsection, cavitation plate, skeg
box((0, -3.2, 0.98), (0.44, 0.56, 0.42), "black")
box((0, -3.22, 1.24), (0.36, 0.46, 0.1), "black")
box((0, -3.12, 0.5), (0.18, 0.3, 0.56), "black")
box((0, -3.1, 0.13), (0.32, 0.42, 0.05), "black")
box((0, -3.08, -0.1), (0.1, 0.3, 0.4), "black")
box((0, -3.0, 0.66), (0.32, 0.14, 0.18), "metal")
# nav light pole on the transom
tube((-0.85, -2.82, 0.75), (-0.85, -2.85, 1.55), 0.015, "metal", 4)
box((-0.85, -2.85, 1.57), (0.05, 0.05, 0.06), "hull")

# ------------------------------------------------------------------ Blue Robotics Fathom Spool (standard, 150 m)
# real proportions: flanges R170 mm, 240 mm between the frame side panels, 375 mm tall, feet span 366 mm.
# axle across the boat (X) so the tether pays off aft; on the starboard aft deck, crank outboard.
SX, SY, SZ = 0.6, -2.1, DZ + 0.2          # axle centre
FL = 0.105                                # flange offset from the centre (flanges ~210 mm apart)
for sx in (-FL, FL):
    ring_x(SX + sx, SY, SZ, 0.125, 0.17, 0.008, "black", 16)       # flange rim (kidney cut-outs between spokes)
    for k in range(5):                                              # five wide spokes to the hub
        a = 2 * math.pi * k / 5 + 0.3
        bar((SX + sx, SY + 0.03 * math.cos(a), SZ + 0.03 * math.sin(a)), (SX + sx, SY + 0.13 * math.cos(a), SZ + 0.13 * math.sin(a)), 0.05, 0.008, "black")
# yellow tether wound to ~12 mm below the flange rim, with ridges for the wraps; the blue standoff core inside
prof = [(-FL + 0.006, 0.09)]
for k in range(8):
    x0 = -FL + 0.006 + k * (2 * FL - 0.012) / 8
    prof += [(x0 + 0.002, 0.152), (x0 + (2 * FL - 0.012) / 16, 0.158)]
prof += [(FL - 0.008, 0.152), (FL - 0.006, 0.09)]
lathe_x(SX, SY, SZ, prof, "yellow", 12)
for k in range(6):    # blue standoffs showing past the tether at the flanges
    a = 2 * math.pi * k / 6
    tube((SX - FL - 0.004, SY + 0.095 * math.cos(a), SZ + 0.095 * math.sin(a)), (SX - FL + 0.01, SY + 0.095 * math.cos(a), SZ + 0.095 * math.sin(a)), 0.009, "blue", 4)
# frame side panels: X-shaped 12 mm black HDPE, the long upper arm carries the blue handle bar
for px in (-0.125, 0.125):
    x = SX + px
    bar((x, SY - 0.18, SZ - 0.2), (x, SY + 0.17, SZ + 0.17), 0.045, 0.012, "black")   # long arm, foot aft -> handle fwd-top
    bar((x, SY + 0.18, SZ - 0.2), (x, SY - 0.1, SZ + 0.1), 0.045, 0.012, "black")     # short arm
    box((x, SY, SZ), (0.013, 0.07, 0.07), "black")                                     # boss at the axle
box((SX, SY + 0.17, SZ + 0.17), (0.25, 0.03, 0.03), "blue")                           # handle bar
tube((SX - 0.125, SY + 0.17, SZ + 0.17), (SX + 0.125, SY + 0.17, SZ + 0.17), 0.016, "blue", 6)
for fy in (-0.17, 0.17): box((SX, SY + fy, SZ - 0.19), (0.26, 0.025, 0.02), "black")   # foot bars
# crank on the starboard (outboard) side: blue arm, black knob
box((SX + 0.135, SY, SZ - 0.045), (0.012, 0.035, 0.12), "blue")
tube((SX + 0.14, SY, SZ - 0.1), (SX + 0.22, SY, SZ - 0.1), 0.017, "black", 6)
# slip ring on the inboard side: blue cap flange, Binder 770 connector and the yellow extension to the console
lathe_x(SX - 0.13, SY, SZ, [(-0.0, 0.04), (-0.025, 0.04), (-0.025, 0.03), (-0.06, 0.03)], "blue", 10)
tube((SX - 0.19, SY, SZ), (SX - 0.24, SY, SZ), 0.014, "black", 6)
polyline([(SX - 0.24, SY, SZ), (SX - 0.3, SY + 0.02, DZ + 0.02), (0.05, -1.0, DZ + 0.02), (0.05, -0.2, DZ + 0.02), (0.05, -0.12, DZ + 0.3)], 0.006, "yellow", 4)

# ------------------------------------------------------------------ tether run: spool -> transom roller -> TetherAnchor
# Godot: drum top (0.6, 0.67, 2.1) -> roller (0.6, 0.83, 2.88) -> over the transom -> TetherAnchor (0.6, 0.12, 3.06)
RY, RZ = -2.88, section(-2.9)[1] + 0.06
box((SX - 0.09, RY, RZ - 0.04), (0.012, 0.08, 0.1), "metal"); box((SX + 0.09, RY, RZ - 0.04), (0.012, 0.08, 0.1), "metal")
tube((SX - 0.085, RY, RZ - 0.03), (SX + 0.085, RY, RZ - 0.03), 0.028, "black", 8)       # roller
polyline([(SX, SY, SZ + 0.158), (SX, RY + 0.02, RZ + 0.005), (SX, RY - 0.06, RZ - 0.01), (SX, -2.99, 0.6), (SX, -3.06, 0.12)], 0.007, "yellow", 5)

bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-5)
bmesh.ops.recalc_face_normals(bm, faces=[f for f in bm.faces if f.material_index != order.index("glass")])
me = bpy.data.meshes.new("boat"); bm.to_mesh(me); bm.free()
for m in M.values(): me.materials.append(m)
ob = bpy.data.objects.new("boat_hull", me); bpy.context.collection.objects.link(ob)
tris = sum(len(p.vertices) - 2 for p in me.polygons)
print("TRIS", tris, "VERTS", len(me.vertices), "MATS", len(me.materials))
bpy.ops.export_scene.gltf(filepath=OUT, export_format="GLB", export_apply=True, export_normals=True,
                          export_texcoords=False, export_materials="EXPORT")
