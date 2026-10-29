# BlueSim survey / ROV-support boat, v11: a ~4 m (13 ft) open grey fibreglass skiff, the layout of the boat in
# Willian's reference photo: white hull with a black rub rail, plain light grey liner (v11.1: no veins, no ropes), wide
# flat gunwale caps, a blunt rounded bow with a bow deck (white non-skid pads), a thwart bench ~70 % forward,
# black rubber mat on the aft floor, a small grey tiller outboard on the transom (port of centre), a small kicker
# tilted up on the starboard quarter, aluminium gunwale rails with black clamp blocks / rod holders, an oar on the
# port gunwale. Plus the Blue Robotics Fathom Spool on the aft mat and its tether over a transom roller.
# Procedural, low poly, flat colours, no textures. Run: blender -b --python build_boat.py -- [out.glb]
# Blender coords: +Y = bow, +X = starboard, +Z = up, Z=0 = design waterline. The glTF export maps
# Blender (x,y,z) -> Godot (x,z,-y): bow to -Z, starboard +X, waterline y=0.
import bpy, bmesh, math, sys
from mathutils import Vector, Matrix

OUT = sys.argv[sys.argv.index("--") + 1] if "--" in sys.argv else "boat.glb"
bpy.ops.wm.read_factory_settings(use_empty=True)

def mat(name, rgb, rough=0.6, metal=0.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    p = m.node_tree.nodes["Principled BSDF"]
    p.inputs["Base Color"].default_value = (*rgb, 1)
    p.inputs["Roughness"].default_value = rough
    p.inputs["Metallic"].default_value = metal
    m.use_backface_culling = True
    return m

M = {
 "hull":   mat("boat_hull",   (0.66, 0.67, 0.65), 0.45),       # white gelcoat outside, non-skid pads, white rope
 "liner":  mat("boat_liner",  (0.31, 0.33, 0.36), 0.6),        # light grey speckled interior, gunwale caps
 "black":  mat("boat_black",  (0.025, 0.025, 0.028), 0.9),     # floor mat, rub rail, clamp blocks, cables
 "metal":  mat("boat_metal",  (0.62, 0.63, 0.64), 0.35, 0.8),  # aluminium rails, brackets, roller cheeks
 "engine": mat("boat_engine", (0.17, 0.175, 0.185), 0.4, 0.2),  # Yamaha grey cowls
 "wood":   mat("boat_wood",   (0.56, 0.33, 0.12), 0.7),        # oar blade
 "blue":   mat("boat_blue",   (0.02, 0.17, 0.62), 0.5),        # bow rope, anodized spool parts
 "yellow": mat("boat_tether", (0.98, 0.82, 0.05), 0.6),        # Fathom tether
}
order = list(M.keys())
bm = bmesh.new()

def face(verts, m):
    f = bm.faces.new(verts); f.material_index = order.index(m); f.smooth = False; return f

# ------------------------------------------------------------------ hull shape
YT, YB = -2.0, 2.0          # transom, bow tip
L = YB - YT
def half_beam(y):             # at the sheer: wide transom, max amidships, blunt rounded (U) bow in plan
    if y <= 0: return 0.60 + 0.10 * (1 - (y / YT) ** 2)
    u = min(1.0, y / YB)
    return max(0.03, 0.70 * max(0.0, 1 - u ** 2.6) ** (1 / 2.2))
def sheer_z(y):
    return 0.46 + 0.12 * max(0.0, (y - 0.3) / (YB - 0.3)) ** 2
def keel_z(y):
    t = (y - YT) / L
    return -0.30 + 0.36 * max(0.0, (t - 0.55) / 0.45) ** 1.8
def section(y):
    b, sh, k = half_beam(y), sheer_z(y), keel_z(y)
    xc = 0.86 * b
    zc = k + 0.25 * xc                                   # ~14 deg deadrise to the chine
    pts = [(0.0, k), (0.45 * xc, k + 0.25 * 0.45 * xc - 0.01), (xc, zc), (xc + 0.03 * b, zc + 0.02),
           (0.96 * b, zc + 0.55 * (sh - zc)), (b, sh)]
    return pts, sh
def cap_w(y):                 # gunwale cap width; widens into the rounded bow deck
    return 0.10 + 0.20 * min(1.0, max(0.0, (y - 1.0) / 0.75))
FLOOR, BOWDECK, THW0, THW1 = 0.10, 0.28, 0.78, 0.98
def deck_z(y):
    return FLOOR if y <= THW0 + 1e-6 else BOWDECK
def floor_x(y):               # half width of the floor where the liner meets it
    return max(0.03, half_beam(y) - cap_w(y) - 0.07)

stations = [YT, -1.7, -1.2, -0.6, 0.0, 0.5, THW0, THW1, 1.25, 1.5, 1.72, 1.88, 1.96, YB]
rings = []
for y in stations:
    pts, sh = section(y)
    half = [Vector((x, y, z)) for x, z in pts]
    full = [half[0]] + half[1:] + [Vector((-v.x, v.y, v.z)) for v in reversed(half[1:])]
    rings.append([bm.verts.new(v) for v in full])
n = len(rings[0])
S, P = 5, 6                   # sheer stbd / port indices
for i in range(len(rings) - 1):
    a, b = rings[i], rings[i + 1]
    for k in range(n):
        k2 = (k + 1) % n
        if k == S: continue
        face([a[k], a[k2], b[k2], b[k]], "hull")
face(list(reversed(rings[-1])), "hull")
face(rings[0], "hull")                                   # transom
deck_rings = []
for i, y in enumerate(stations[:-1]):
    r = rings[i]; s, p = r[S].co, r[P].co
    ci = max(0.04, s.x - cap_w(y)); fx = min(floor_x(y), ci - 0.01); dz = deck_z(y)
    deck_rings.append((r[S], bm.verts.new((ci, y, s.z)), bm.verts.new((fx, y, dz)),
                       bm.verts.new((-fx, y, dz)), bm.verts.new((-ci, y, p.z)), r[P]))
for i in range(len(deck_rings) - 1):
    A, B = deck_rings[i], deck_rings[i + 1]
    face([A[0], B[0], B[1], A[1]], "liner")   # gunwale caps
    face([A[1], B[1], B[2], A[2]], "liner")   # liner sides
    face([A[2], B[2], B[3], A[3]], "liner")   # floor / bow deck
    face([A[3], B[3], B[4], A[4]], "liner")
    face([A[4], B[4], B[5], A[5]], "liner")
last = deck_rings[-1]; bowr = rings[-1]
face([last[0], last[1], last[2], last[3], last[4], last[5], bowr[P], bowr[S]], "liner")   # breasthook
first = deck_rings[0]
face([first[5], first[4], first[3], first[2], first[1], first[0]], "liner")             # transom inside

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

def flat_strip(pts, w, m):
    """thin flat ribbon lying on a horizontal-ish surface (points already carry z), facing up"""
    P_ = [Vector(p) for p in pts]; L_, R_ = [], []
    for i, p in enumerate(P_):
        d = (P_[min(i + 1, len(P_) - 1)] - P_[max(i - 1, 0)]); d.z = 0; d.normalize()
        o = Vector((-d.y, d.x, 0)) * (w / 2)
        L_.append(bm.verts.new(p + o)); R_.append(bm.verts.new(p - o))
    for i in range(len(P_) - 1):
        face([R_[i], R_[i + 1], L_[i + 1], L_[i]], m)

def lathe_x(cx, cy, cz, profile, m, seg=12):
    rings_ = []
    for xo, r in profile:
        rings_.append([bm.verts.new((cx + xo, cy + r * math.cos(2 * math.pi * i / seg), cz + r * math.sin(2 * math.pi * i / seg))) for i in range(seg)])
    for a, b in zip(rings_, rings_[1:]):
        for i in range(seg):
            j = (i + 1) % seg; face([a[i], a[j], b[j], b[i]], m)
    face(list(reversed(rings_[0])), m); face(rings_[-1], m)

def ring_x(cx, cy, cz, r_in, r_out, t, m, seg=16):
    V = [[[bm.verts.new((cx + sx * t / 2, cy + r * math.cos(2 * math.pi * i / seg), cz + r * math.sin(2 * math.pi * i / seg)))
           for i in range(seg)] for r in (r_in, r_out)] for sx in (-1, 1)]
    for i in range(seg):
        j = (i + 1) % seg
        face([V[0][0][i], V[0][1][i], V[0][1][j], V[0][0][j]], m)
        face([V[1][0][j], V[1][1][j], V[1][1][i], V[1][0][i]], m)
        face([V[0][1][i], V[1][1][i], V[1][1][j], V[0][1][j]], m)
        face([V[0][0][j], V[1][0][j], V[1][0][i], V[0][0][i]], m)

def cap_pt(y, side, frac=0.5, up=0.0):
    """point on the gunwale cap: frac 0 = outer edge (sheer), 1 = inner edge"""
    b = half_beam(y); return (side * (b - frac * cap_w(y)), y, sheer_z(y) + up)

# ------------------------------------------------------------------ interior layout
# thwart bench across the boat ~70 % forward (top 0.40), its aft face hides the step up to the bow deck
tw = 2 * (half_beam(0.88) - cap_w(0.88) - 0.04)
box((0, 0.88, (FLOOR + 0.40) / 2), (tw, THW1 - THW0, 0.40 - FLOOR), "liner")
box((0, 0.88, 0.404), (tw - 0.02, THW1 - THW0 - 0.02, 0.008), "liner")
# dark footwell square on the bow deck just forward of the bench, starboard of centre
box((0.1, 1.2, BOWDECK + 0.004), (0.34, 0.3, 0.008), "black")
# black rubber mat over the aft floor (transom step left grey)
mys = [-1.38, -1.0, -0.5, 0.0, 0.5, THW0 - 0.01]
mz = FLOOR + 0.012
ml = [bm.verts.new((-(floor_x(y) - 0.02), y, mz)) for y in mys]
mr = [bm.verts.new(((floor_x(y) - 0.02), y, mz)) for y in mys]
for i in range(len(mys) - 1): face([ml[i], mr[i], mr[i + 1], ml[i + 1]], "black")
face([ml[0], ml[-1], mr[-1], mr[0]][:0] or [ml[0], mr[0], bm.verts.new((floor_x(-1.38) - 0.02, -1.38, FLOOR + 0.001)), bm.verts.new((-(floor_x(-1.38) - 0.02), -1.38, FLOOR + 0.001))], "black")
# white non-skid pads on the bow deck corners
for side in (1, -1):
    ys = [1.1, 1.35, 1.6, 1.8]
    flat_strip([cap_pt(y, side, 0.5, 0.004) for y in ys], 0.12, "hull")
# bow eye on the stem (GroundTackle fairlead) and a small bow cleat
tube((0, YB - 0.02, sheer_z(YB) - 0.06), (0, YB + 0.04, sheer_z(YB) - 0.06), 0.02, "metal", 6)
box((0.08, 1.68, BOWDECK + 0.02), (0.05, 0.12, 0.03), "metal")

# rub rail around the sheer and the transom top
for i in range(len(stations) - 1):
    for side in (1, -1):
        (p0, s0), (p1, s1) = section(stations[i]), section(stations[i + 1])
        tube((side * (p0[-1][0] + 0.012), stations[i], s0 - 0.012), (side * (p1[-1][0] + 0.012), stations[i + 1], s1 - 0.012), 0.022, "black", 4, caps=False)
tube((half_beam(YT), YT - 0.012, sheer_z(YT) - 0.012), (-half_beam(YT), YT - 0.012, sheer_z(YT) - 0.012), 0.022, "black", 4, caps=False)

# aluminium gunwale rails with black clamp blocks (starboard: rowlock/clamp blocks, rod holders, electronics)
for side, (y0, y1), blocks in ((1, (-1.8, -0.15), (-1.25, -0.55)), (-1, (-1.75, 0.25), (-1.3, -0.45))):
    a, b = cap_pt(y0, side, 0.45, 0.05), cap_pt(y1, side, 0.45, 0.05)
    tube(a, b, 0.011, "metal", 5)
    for y in (y0, y1):
        p = cap_pt(y, side, 0.45); tube(p, (p[0], p[1], p[2] + 0.05), 0.01, "metal", 4, caps=False)
    for y in blocks:
        p = cap_pt(y, side, 0.45, 0.06); box(p, (0.07, 0.12, 0.07), "black")
for k, y in enumerate((-0.42, -0.32, -0.22)):   # rod holders on the starboard gunwale
    p = cap_pt(y, 1, 0.6); tube((p[0] - 0.02, p[1], p[2] - 0.02), (p[0] - 0.06, p[1] - 0.03, p[2] + 0.08), 0.022, "black", 5)
p = cap_pt(-1.75, 1, 0.5); box((p[0], p[1], p[2] + 0.05), (0.1, 0.12, 0.08), "black")   # electronics box on a bracket
# oar lying on the port gunwale: handle aft, wooden blade forward
oy0, oy1 = -1.3, 0.05
a, b = cap_pt(oy0, -1, 0.4, 0.085), cap_pt(oy1, -1, 0.4, 0.085)
tube(a, b, 0.017, "black", 6)
c = cap_pt(oy1 + 0.22, -1, 0.4, 0.085)
box(((b[0] + c[0]) / 2, (b[1] + c[1]) / 2, c[2]), (0.13, 0.42, 0.014), "wood")
tube((a[0], a[1] - 0.12, a[2]), a, 0.02, "wood", 6)

# ------------------------------------------------------------------ outboards
# main: small grey Yamaha tiller four-stroke (~9.9-20 hp, short shaft) on the transom, port of centre
OX, OY = -0.2, YT - 0.22
TZ = sheer_z(YT)
box((OX, YT - 0.05, TZ - 0.06), (0.16, 0.12, 0.16), "black")                    # transom clamp bracket
box((OX, OY, TZ + 0.12), (0.30, 0.42, 0.16), "black")                         # lower cowl
box((OX, OY - 0.01, TZ + 0.29), (0.32, 0.46, 0.18), "engine")                 # upper cowl
box((OX, OY - 0.02, TZ + 0.40), (0.26, 0.36, 0.05), "engine")                 # rounded top
box((OX, OY + 0.24, TZ + 0.12), (0.06, 0.06, 0.05), "black")                  # tiller pivot
bar((OX, OY + 0.26, TZ + 0.12), (OX - 0.05, YT + 0.42, TZ + 0.18), 0.05, 0.05, "black", (0, 0, 1))   # tiller arm forward
box((OX, OY + 0.02, (TZ + 0.04 - 0.22) / 2), (0.11, 0.18, TZ + 0.04 + 0.22), "engine")   # midsection
box((OX, OY - 0.02, -0.22), (0.26, 0.34, 0.025), "engine")                     # cavitation plate
tube((OX, OY + 0.12, -0.27), (OX, OY - 0.14, -0.27), 0.045, "engine", 6)       # gearcase torpedo
box((OX, OY + 0.0, -0.32), (0.02, 0.16, 0.06), "engine")                       # skeg (bottom -0.35)
for k in range(3):                                                              # 3-blade prop
    a = 2 * math.pi * k / 3 + 0.4
    bar((OX, OY - 0.15, -0.27), (OX + 0.085 * math.cos(a), OY - 0.15, -0.27 + 0.075 * math.sin(a)), 0.05, 0.008, "black", (0, 1, 0))
# kicker: a small grey Yamaha tilted fully up on a bracket on the starboard quarter
KX, KY, KZ = 0.5, YT - 0.12, TZ + 0.06
box((KX, YT - 0.04, TZ - 0.03), (0.1, 0.08, 0.1), "metal")
box((KX, KY - 0.08, KZ), (0.17, 0.24, 0.15), "engine")
box((KX, KY - 0.33, KZ - 0.02), (0.05, 0.28, 0.06), "black")                    # shaft, horizontal (tilted up)
box((KX, KY - 0.48, KZ - 0.02), (0.012, 0.1, 0.13), "black")                   # prop

# ------------------------------------------------------------------ Blue Robotics Fathom Spool (standard, 150 m)
# real proportions: flanges R170 mm, 240 mm between the frame side panels, 375 mm tall, feet span 366 mm.
# axle across the boat (X) so the tether pays off aft; on the starboard side of the aft mat, crank outboard.
SX, SY, SZ = 0.25, -1.05, mz + 0.2
FL = 0.105
for sx in (-FL, FL):
    ring_x(SX + sx, SY, SZ, 0.125, 0.17, 0.008, "black", 12)
    for k in range(5):
        a = 2 * math.pi * k / 5 + 0.3
        bar((SX + sx, SY + 0.03 * math.cos(a), SZ + 0.03 * math.sin(a)), (SX + sx, SY + 0.13 * math.cos(a), SZ + 0.13 * math.sin(a)), 0.05, 0.008, "black")
prof = [(-FL + 0.006, 0.09)]
for k in range(4):     # ridges for the wraps
    x0 = -FL + 0.006 + k * (2 * FL - 0.012) / 4
    prof += [(x0 + 0.002, 0.152), (x0 + (2 * FL - 0.012) / 8, 0.158)]
prof += [(FL - 0.008, 0.152), (FL - 0.006, 0.09)]
lathe_x(SX, SY, SZ, prof, "yellow", 12)
for k in range(3):
    a = 2 * math.pi * k / 3
    tube((SX - FL - 0.004, SY + 0.095 * math.cos(a), SZ + 0.095 * math.sin(a)), (SX - FL + 0.01, SY + 0.095 * math.cos(a), SZ + 0.095 * math.sin(a)), 0.009, "blue", 4)
for px in (-0.125, 0.125):
    x = SX + px
    bar((x, SY - 0.18, SZ - 0.2), (x, SY + 0.17, SZ + 0.17), 0.045, 0.012, "black")
    bar((x, SY + 0.18, SZ - 0.2), (x, SY - 0.1, SZ + 0.1), 0.045, 0.012, "black")
    box((x, SY, SZ), (0.013, 0.07, 0.07), "black")
box((SX, SY + 0.17, SZ + 0.17), (0.25, 0.03, 0.03), "blue")
tube((SX - 0.125, SY + 0.17, SZ + 0.17), (SX + 0.125, SY + 0.17, SZ + 0.17), 0.016, "blue", 6)
for fy in (-0.17, 0.17): box((SX, SY + fy, SZ - 0.19), (0.26, 0.025, 0.02), "black")
box((SX + 0.135, SY, SZ - 0.045), (0.012, 0.035, 0.12), "blue")
tube((SX + 0.14, SY, SZ - 0.1), (SX + 0.2, SY, SZ - 0.1), 0.017, "black", 6)
# slip ring on the inboard side, Binder connector, and the yellow extension along the mat to the pilot's laptop case
lathe_x(SX - 0.13, SY, SZ, [(-0.0, 0.04), (-0.025, 0.04), (-0.025, 0.03), (-0.06, 0.03)], "blue", 10)
tube((SX - 0.19, SY, SZ), (SX - 0.24, SY, SZ), 0.014, "black", 6)
polyline([(SX - 0.24, SY, SZ), (SX - 0.24, SY + 0.03, mz + 0.006), (0.1, -0.95, mz + 0.006), (0.1, -0.73, mz + 0.006), (0.062, -0.73, mz + 0.3)], 0.006, "yellow", 4)

# ------------------------------------------------------------------ tether run: spool -> transom roller -> TetherAnchor
# Godot: drum top (0.25, 0.47, 1.05) -> roller on the transom top (0.25, ~0.52, 1.96) -> down the transom -> TetherAnchor (0.25, 0.12, 2.16)
RY, RZ = YT + 0.04, TZ + 0.06
box((SX - 0.09, RY, RZ - 0.04), (0.012, 0.08, 0.1), "metal"); box((SX + 0.09, RY, RZ - 0.04), (0.012, 0.08, 0.1), "metal")
tube((SX - 0.085, RY, RZ - 0.03), (SX + 0.085, RY, RZ - 0.03), 0.028, "black", 8)
polyline([(SX, SY, SZ + 0.158), (SX, RY + 0.02, RZ + 0.005), (SX, RY - 0.06, RZ - 0.01), (SX, YT - 0.1, 0.35), (SX, YT - 0.16, 0.12)], 0.007, "yellow", 5)

bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-5)
bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
me = bpy.data.meshes.new("boat"); bm.to_mesh(me); bm.free()
for m in M.values(): me.materials.append(m)
ob = bpy.data.objects.new("boat_hull", me); bpy.context.collection.objects.link(ob)
tris = sum(len(p.vertices) - 2 for p in me.polygons)
import collections
per = collections.Counter()
for p in me.polygons: per[order[p.material_index]] += len(p.vertices) - 2
print("THWART_W", round(tw, 3), "HB_MAX", round(max(half_beam(y / 100) for y in range(-175, 176)), 3))
print("TRIS", tris, "VERTS", len(me.vertices), "MATS", len(me.materials), dict(per))
zs = [v.co.z for v in me.vertices]; print("ZRANGE", round(min(zs), 3), round(max(zs), 3))
bpy.ops.export_scene.gltf(filepath=OUT, export_format="GLB", export_apply=True, export_normals=True,
                          export_texcoords=False, export_materials="EXPORT")
