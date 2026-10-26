# Low-poly ROV pilot for the BlueSim boat (v10, ~3.1k tris): a stylised figure loosely inspired by Blake (trucker cap, sandy
# tousled hair flicking out under the cap, KATCH-style fish logo, smile and stubble, red-and-black
# foul-weather jacket, grey/white camo neck gaiter), seated on a hard case, bent forward over a
# laptop under a dark sun-shade hood, gamepad in both hands. Two objects: PilotBody (figure, seat case,
# gamepad) and PilotLaptop (table case, laptop, shade hood). Run: blender -b --python build_pilot.py -- [out.glb]
# Blender coords, origin on the deck under the seat, figure faces +Y (Godot -Z). boat.tscn turns it to face aft.
import bpy, bmesh, math, sys
from mathutils import Vector, Matrix

OUT = sys.argv[sys.argv.index("--") + 1] if "--" in sys.argv else "pilot.glb"
bpy.ops.wm.read_factory_settings(use_empty=True)

def mat(name, rgb, rough=0.7, emit=None, cull=True):
    m = bpy.data.materials.new(name); m.use_nodes = True
    p = m.node_tree.nodes["Principled BSDF"]
    p.inputs["Base Color"].default_value = (*rgb, 1); p.inputs["Roughness"].default_value = rough
    if emit:
        p.inputs["Emission Color"].default_value = (*emit, 1); p.inputs["Emission Strength"].default_value = 1.5
    m.use_backface_culling = cull
    return m
M = {
 "skin":     mat("pilot_skin",     (0.87, 0.60, 0.45), 0.7),    # fair, warm peach skin
 "lip":      mat("pilot_lip",      (0.80, 0.40, 0.36), 0.6),    # lips of the smile
 "jacket":   mat("pilot_jacket",   (0.55, 0.045, 0.03), 0.6),   # red foul-weather jacket
 "trousers": mat("pilot_trousers", (0.06, 0.065, 0.07), 0.8),   # dark rain trousers
 "hair":     mat("pilot_hair",     (0.22, 0.135, 0.058), 0.8), # darker sandy-brown hair and brows
 "hair_tip": mat("pilot_hair_tip", (0.46, 0.32, 0.155), 0.8),    # sun-bleached lighter tips of the hair
 "iris":     mat("pilot_iris",     (0.10, 0.05, 0.022), 0.4),   # brown eyes
 "black":    mat("pilot_black",    (0.025, 0.025, 0.03), 0.6),  # cap, jacket panels, boots, gamepad, sun-shade hood
 "case":     mat("pilot_case",     (0.13, 0.135, 0.14), 0.5),   # hard cases, laptop shell
 "gaiter":   mat("pilot_gaiter",   (0.20, 0.21, 0.23), 0.9),    # grey camo patches of the neck gaiter
 "white":    mat("pilot_white",    (0.78, 0.78, 0.76), 0.8),    # white camo patches, cap logo lettering, gamepad buttons
 "pink":     mat("pilot_pink",     (0.85, 0.28, 0.42), 0.6),    # cap logo fish
 "stubble":  mat("pilot_stubble",  (0.80, 0.555, 0.42), 0.85),    # light stubble on jaw, chin and upper lip
 "screen":   mat("pilot_screen",   (0.02, 0.1, 0.16), 0.2, emit=(0.08, 0.3, 0.45)),  # ROV video on the laptop
}
order = list(M.keys())

def new_obj(name, build):
    global bm
    bm = bmesh.new(); build()
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-5)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    me = bpy.data.meshes.new(name); bm.to_mesh(me); bm.free()
    for m in M.values(): me.materials.append(m)
    ob = bpy.data.objects.new(name, me); bpy.context.collection.objects.link(ob)
    return ob

def face(verts, m):
    f = bm.faces.new(verts); f.material_index = order.index(m); return f

def box(c, s, m, rot=None):
    c = Vector(c); vs = []
    for dx in (-1, 1):
        for dy in (-1, 1):
            for dz in (-1, 1):
                v = Vector((dx * s[0] / 2, dy * s[1] / 2, dz * s[2] / 2))
                vs.append(bm.verts.new((rot @ v if rot else v) + c))
    for f in [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]:
        face([vs[i] for i in f], m)

def frustum(p0, p1, w0, d0, w1, d1, m, fwd=(0, 1, 0)):
    """tapered box from p0 to p1; w = width along X, d = depth across the axis in the fwd direction"""
    p0, p1 = Vector(p0), Vector(p1); ax = (p1 - p0).normalized()
    x = Vector((1, 0, 0)); y = ax.cross(x).normalized() * -1
    def rect(c, w, d): return [bm.verts.new(c + x * sx * w / 2 + y * sy * d / 2) for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    a, b = rect(p0, w0, d0), rect(p1, w1, d1)
    face(list(reversed(a)), m); face(b, m)
    for i in range(4):
        j = (i + 1) % 4; face([a[i], a[j], b[j], b[i]], m)

def tube(p0, p1, r0, r1, m, seg=6, caps=True):
    p0, p1 = Vector(p0), Vector(p1); ax = (p1 - p0).normalized()
    up = Vector((0, 0, 1)) if abs(ax.z) < 0.9 else Vector((1, 0, 0))
    u = ax.cross(up).normalized(); w = ax.cross(u); A, B = [], []
    for i in range(seg):
        a = 2 * math.pi * i / seg; o = u * math.cos(a) + w * math.sin(a)
        A.append(bm.verts.new(p0 + o * r0)); B.append(bm.verts.new(p1 + o * r1))
    for i in range(seg):
        j = (i + 1) % seg; face([A[i], A[j], B[j], B[i]], m)
    if caps: face(list(reversed(A)), m); face(B, m)

def blob(c, s, m, useg=8, vseg=6, rot=None, zmin=-1.0, deform=None, matfn=None):
    """low-poly ellipsoid (optionally only the part above zmin in unit coords, closed flat)"""
    c = Vector(c); rings_ = []
    for j in range(1, vseg):
        phi = math.pi * j / vseg; z = math.cos(phi)
        if z < zmin: break
        rings_.append([(math.sin(phi) * math.cos(2 * math.pi * i / useg), math.sin(phi) * math.sin(2 * math.pi * i / useg), z) for i in range(useg)])
    def V(p):
        if deform: p = deform(p)
        v = Vector((p[0] * s[0], p[1] * s[1], p[2] * s[2])); return bm.verts.new((rot @ v if rot else v) + c)
    top = V((0, 0, 1)); R = [[V(p) for p in r] for r in rings_]
    for i in range(useg): face([top, R[0][i], R[0][(i + 1) % useg]], m)
    for j, (a, b) in enumerate(zip(R, R[1:])):
        for i in range(useg):
            k = (i + 1) % useg
            cu = [sum(q[d] for q in (rings_[j][i], rings_[j + 1][i], rings_[j + 1][k], rings_[j][k])) / 4 for d in range(3)]
            face([a[i], b[i], b[k], a[k]], matfn(cu) if matfn else m)
    if zmin > -1.0: face(list(reversed(R[-1])), m)
    else:
        bot = V((0, 0, -1))
        for i in range(useg): face([R[-1][(i + 1) % useg], R[-1][i], bot], m)


def loft(rings_pts, mats, cap0=True, cap1=True):
    R = [[bm.verts.new(p) for p in r] for r in rings_pts]
    n = len(R[0])
    for k, (a, b) in enumerate(zip(R, R[1:])):
        for i in range(n):
            j = (i + 1) % n; face([a[i], a[j], b[j], b[i]], mats[k])
    if cap0: face(list(reversed(R[0])), mats[0])
    if cap1: face(R[-1], mats[-1])

def superellipse(c, xv, yv, w, d, n=12, e=2.6):
    pts = []
    for i in range(n):
        a = 2 * math.pi * i / n; ca, sa = math.cos(a), math.sin(a)
        x = math.copysign(abs(ca) ** (2 / e), ca) * w / 2; y = math.copysign(abs(sa) ** (2 / e), sa) * d / 2
        pts.append(c + xv * x + yv * y)
    return pts

def lock(pts, widths, thicks, wdir, m, sides=4):
    """a lock of hair: flattened tapered strand along pts, width along wdir"""
    rings_ = []
    for i, p in enumerate(pts):
        t = (pts[min(i + 1, len(pts) - 1)] - pts[max(i - 1, 0)]).normalized()
        w = (wdir - t * wdir.dot(t)).normalized(); nrm = t.cross(w).normalized()
        rings_.append([p + w * math.cos(2 * math.pi * k / sides) * widths[i] / 2 + nrm * math.sin(2 * math.pi * k / sides) * thicks[i] / 2 for k in range(sides)])
    loft(rings_, m if isinstance(m, list) else [m] * (len(rings_) - 1))

def bezier(p0, p1, p2, n):
    return [p0 * (1 - t) ** 2 + p1 * 2 * t * (1 - t) + p2 * t * t for t in [k / (n - 1) for k in range(n)]]

def body():
    # seat: a grey hard case on the deck
    box((0, -0.06, 0.2), (0.52, 0.36, 0.4), "case")
    box((0, -0.06, 0.405), (0.2, 0.05, 0.02), "black")
    # legs: rounded trousers with knees, boots with shaft, foot, sole
    blob((0, 0.0, 0.5), (0.19, 0.15, 0.09), "trousers", 8, 4)
    for sx in (-1, 1):
        hip, knee, ankle = Vector((sx * 0.1, 0.0, 0.48)), Vector((sx * 0.12, 0.4, 0.5)), Vector((sx * 0.13, 0.47, 0.13))
        tube(hip, knee, 0.088, 0.068, "trousers", 8, False)
        blob(knee, (0.068, 0.068, 0.068), "trousers", 6, 4)
        tube(knee, ankle, 0.062, 0.052, "trousers", 8, False)
        tube((sx * 0.13, 0.47, 0.18), (sx * 0.13, 0.48, 0.05), 0.058, 0.058, "black", 8)        # boot shaft
        loft([superellipse(Vector((sx * 0.13, y, z)), Vector((1, 0, 0)), Vector((0, 0, 1)), w, h, 8) for y, z, w, h in
              ((0.42, 0.055, 0.1, 0.09), (0.5, 0.05, 0.105, 0.085), (0.6, 0.042, 0.1, 0.065), (0.655, 0.04, 0.075, 0.05))]
             if False else [[Vector((sx * 0.13, y, z)) + Vector((1, 0, 0)) * x + Vector((0, 0, 1)) * zz for x, zz in
                             [(w / 2 * math.cos(2 * math.pi * k / 8), h / 2 * math.sin(2 * math.pi * k / 8)) for k in range(8)]]
                            for y, z, w, h in ((0.42, 0.055, 0.1, 0.09), (0.5, 0.05, 0.105, 0.085), (0.6, 0.045, 0.1, 0.07), (0.655, 0.042, 0.07, 0.05))],
             ["black"] * 3)
        box((sx * 0.13, 0.54, 0.008), (0.115, 0.26, 0.016), "case")                              # sole
    # torso: rounded jacket loft, bent ~30 degrees forward; two folds over the belly; black yoke; zip
    p0, p1 = Vector((0, 0.0, 0.55)), Vector((0, 0.24, 1.0)); ax = (p1 - p0).normalized()
    xv = Vector((1, 0, 0)); yv = ax.cross(xv).normalized() * -1
    prof = [(0.0, 0.36, 0.26, "jacket"), (0.18, 0.37, 0.255, "jacket"), (0.3, 0.375, 0.25, "jacket"), (0.32, 0.39, 0.265, "jacket"),
            (0.34, 0.378, 0.252, "jacket"), (0.44, 0.385, 0.252, "jacket"), (0.46, 0.4, 0.268, "jacket"), (0.48, 0.39, 0.254, "jacket"),
            (0.72, 0.42, 0.255, "jacket"), (0.88, 0.42, 0.24, "jacket"), (0.95, 0.395, 0.225, "jacket"), (0.99, 0.35, 0.195, "black"), (1.02, 0.31, 0.165, "black"), (1.06, 0.25, 0.15, "black")]
    rings_ = [superellipse(p0 + ax * (t * (p1 - p0).length), xv, yv, w, d, 12) for t, w, d, _ in prof]
    loft(rings_, [pm for _, _, _, pm in prof[1:]])
    for t0, t1 in ((0.04, 0.7),):
        a = p0 + ax * (t0 * (p1 - p0).length) - yv * 0.133; b = p0 + ax * (t1 * (p1 - p0).length) - yv * 0.13
        tube(a, b, 0.007, 0.007, "black", 4)
    # high jacket collar, and a bulky, rumpled gaiter bunched up to the jaw line so no bare neck shows; it
    # stays below the chin and sags a little to the pilot's left
    tube((0, 0.232, 0.975), (0, 0.262, 1.05), 0.1, 0.096, "black", 12, False)
    MARBLE = "WGGWGGWWGWGGWGWGGW"
    marble = lambda i, j: "white" if MARBLE[(i * 5 + j * 7 + (i // 3) * j) % len(MARBLE)] == "W" else "gaiter"
    g0, g1 = Vector((0, 0.245, 0.975)), Vector((0, 0.277, 1.056)); gax = (g1 - g0).normalized()
    gu = Vector((1, 0, 0)); gw = gax.cross(gu)
    tilt_, H_ = Matrix.Rotation(math.radians(-17), 3, "X"), Vector((0, 0.35, 1.18))   # the head's frame (see below)
    N = 12
    def top_pt(i):
        # under the jaw in the head's frame: low at the front (under the chin), high at the back (to the hair)
        a = 2 * math.pi * i / N; ca, sa = math.cos(a), math.sin(a)
        z = -0.096 - 0.03 * max(0.0, sa) + 0.03 * max(0.0, -sa)
        return H_ + tilt_ @ Vector((0.092 * ca, -0.022 + 0.084 * sa, z))
    def bot_pt(i):
        a = 2 * math.pi * i / N
        return g0 + Vector((0, 0, 0.012)) + (gu * math.cos(a) + gw * math.sin(a)) * 0.094
    def rumple(i, j):
        return 1.0 + 0.07 * math.sin(3.0 * i + 1.7 * j) + 0.045 * math.sin(5.0 * i + 0.6 * j + 1.0)
    GL = []
    for j, (t, bulge) in enumerate(((0.0, 0.0), (0.35, 0.03), (0.68, 0.034), (1.0, 0.0))):
        row = []
        for i in range(N):
            b_, t_ = bot_pt(i), top_pt(i)
            c_ = g0.lerp(H_ + tilt_ @ Vector((0, -0.022, -0.1)), t)
            v = b_.lerp(t_, t)
            out = (v - c_); out = out - gax * out.dot(gax); out.normalize()
            a = 2 * math.pi * i / N
            sag = 0.012 * max(0.0, -math.cos(a)) * math.sin(math.pi * t)       # drape to the pilot's left (-x)
            v = v + out * bulge * rumple(i, j) + Vector((-0.008 * math.sin(math.pi * t), 0, -sag))
            row.append(bm.verts.new(v))
        GL.append(row)
    for j in range(3):
        for i in range(N):
            k = (i + 1) % N
            blotch = math.sin(1.3 * i + 2.1 * j) + 0.6 * math.sin(2.9 * i - j + 0.5)     # clustered grey blotches
            face([GL[j][i], GL[j][k], GL[j + 1][k], GL[j + 1][i]], "gaiter" if blotch > 0.35 else "white")
    face(list(reversed(GL[0])), "gaiter")
    ctr = bm.verts.new(H_ + tilt_ @ Vector((0, -0.02, -0.07)))                       # hidden under the head
    for i in range(N): face([GL[3][i], GL[3][(i + 1) % N], ctr], "gaiter")
    TC = g0.lerp(g1, 0.6) + gw * 0.03
    tp0, tp1 = Vector((0, 0.0, 0.55)), Vector((0, 0.24, 1.0)); tax = (tp1 - tp0).normalized(); fwd = -(tax.cross(Vector((1, 0, 0))).normalized() * -1)
    # loose flap of the gaiter draped down the chest: a folded sheet with a ragged hem, marbled grey/white
    flap = [TC + fwd * 0.085 + Vector((0, 0.0, -0.004))] + [tp0 + tax * ((tp1 - tp0).length * t) + fwd * 0.138 for t in (0.9, 0.83, 0.76)]
    fw = [0.105, 0.11, 0.1, 0.078]
    us = [-1.0, -0.5, 0.0, 0.5, 1.0]
    F, Bk = [], []
    for r, c in enumerate(flap):
        dv = (flap[min(r + 1, 3)] - flap[max(r - 1, 0)]).normalized()
        nv = Vector((1, 0, 0)).cross(dv).normalized()
        if nv.dot(fwd) < 0: nv = -nv
        row_f, row_b = [], []
        for ci, u in enumerate(us):
            fold = 0.008 * math.sin(u * math.pi * 1.25 + 0.9 * r) * min(1.0, r * 0.6) - 0.009 * u * u
            hem = (0.009 if ci % 2 else -0.005) * (r == 3) + (0.006 * u if r == 3 else 0.0)
            v = c + Vector((u * fw[r] / 2 + 0.006 * r * math.sin(2.1 * u + r), 0, 0)) + nv * fold + dv * hem
            row_f.append(bm.verts.new(v)); row_b.append(bm.verts.new(v - nv * 0.006))
        F.append(row_f); Bk.append(row_b)
    FLAP = ["WWWW", "GWWW", "WWWG"]
    for r in range(3):
        for ci in range(4):
            m = "gaiter" if FLAP[r][ci] == "G" else "white"
            face([F[r][ci], F[r + 1][ci], F[r + 1][ci + 1], F[r][ci + 1]], m)
            face([Bk[r][ci + 1], Bk[r + 1][ci + 1], Bk[r + 1][ci], Bk[r][ci]], "gaiter")
    for r in range(3):
        for ci in (0, 4):
            q = [F[r][ci], F[r + 1][ci], Bk[r + 1][ci], Bk[r][ci]]
            face(q if ci == 4 else q[::-1], "white")
    for ci in range(4):
        face([F[3][ci], Bk[3][ci], Bk[3][ci + 1], F[3][ci + 1]], "white")
    # irregular darker grey patches over the white cells of the flap and the roll
    def patch(r, ci, fu, fv, sz, rot):
        a_, b_, c_ = F[r][ci].co, F[r][ci + 1].co, F[r + 1][ci].co
        o = a_ + (b_ - a_) * fu + (c_ - a_) * fv
        e1 = (b_ - a_).normalized(); e2 = (c_ - a_).normalized(); n_ = e1.cross(e2).normalized()
        shape = [(1.0, 0.1), (0.4, 0.8), (-0.5, 0.7), (-1.0, -0.2), (-0.3, -0.9), (0.6, -0.6)]
        cr, sr = math.cos(rot), math.sin(rot)
        q = [o + (e1 * (x * cr - y * sr) + e2 * (x * sr + y * cr)) * sz * 0.8 + n_ * 0.0018 for x, y in shape]
        face([bm.verts.new(v) for v in q], "gaiter")
    patch(0, 0, 0.55, 0.5, 0.016, 0.3); patch(0, 2, 0.7, 0.45, 0.02, 1.1); patch(1, 1, 0.6, 0.55, 0.017, 2.0)
    patch(1, 3, 0.35, 0.6, 0.014, 0.7); patch(2, 1, 0.5, 0.35, 0.015, 1.6); patch(2, 2, 0.6, 0.6, 0.012, 2.6)
    patch(0, 3, 0.3, 0.3, 0.01, 0.4)
    # head looking at the screen ~17 degrees down: broad round face and jaw, ears, small nose, eyes with whites
    # and pupils, brows, a curved smile with lips, subtle stubble
    tilt = Matrix.Rotation(math.radians(-17), 3, "X")
    H = Vector((0, 0.35, 1.18))
    SX, SY, SZ = 0.1, 0.106, 0.122
    def jaw(p):
        x, y, z = p
        if z < 0:
            k = -z; x *= 1 - 0.16 * k; y = y * (1 - 0.1 * k) + (0.08 * k if y > 0 else 0.0)
        return (x, y, z)
    def surf(x, z, off=0.003):
        zu = z / SZ; k = max(0.0, -zu); xu = x / (SX * (1 - 0.16 * k))
        yu = math.sqrt(max(0.0, 1 - xu * xu - zu * zu))
        return (yu * (1 - 0.1 * k) + 0.08 * k) * SY + off
    def stubble(u):
        x, y, z = u
        return "stubble" if (z < -0.7 and y > 0.3) else "skin"
    blob(H, (SX, SY, SZ), "skin", 12, 8, tilt, deform=jaw, matfn=stubble)
    L = lambda v: H + tilt @ Vector(v)
    def flat(x, z, w, h, m, off, roll=0.0):
        """a small flat quad lying on the face surface"""
        r = Matrix.Rotation(math.radians(roll), 3, "Y")
        q = [Vector((x, 0, z)) + r @ Vector((dx * w / 2, 0, dz * h / 2)) for dx, dz in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        q = [Vector((v.x, surf(v.x, v.z, off), v.z)) for v in q]
        face([bm.verts.new(L(v)) for v in q][::-1], m)
    for sx in (-1, 1):
        blob(L((sx * 0.103, -0.01, -0.012)), (0.017, 0.026, 0.036), "skin", 6, 4, tilt)            # ear
        flat(sx * 0.035, 0.008, 0.03, 0.019, "white", 0.004)                                     # eye white
        flat(sx * 0.033, 0.0075, 0.013, 0.015, "iris", 0.0052)                                   # brown iris
        flat(sx * 0.033, 0.0075, 0.005, 0.006, "black", 0.0058)                                  # pupil
        flat(sx * 0.035, 0.019, 0.034, 0.0035, "case", 0.0056, sx * 6)                           # upper lid line
        flat(sx * 0.037, 0.034, 0.032, 0.006, "hair", 0.004, sx * -8)                              # brow
    blob(L((0, surf(0, -0.016, -0.004), -0.016)), (0.012, 0.016, 0.02), "skin", 6, 4, tilt)       # small nose
    # smile: an upturned arc of lip quads with a dark line at the opening
    xs = [-0.03 + 0.01 * k for k in range(7)]
    zc = lambda x: -0.053 + 0.012 * (x / 0.03) ** 2
    for x0, x1 in zip(xs, xs[1:]):
        for (dz0, dz1, m, off) in ((-0.0045, 0.0015, "lip", 0.004), (0.0, 0.0018, "case", 0.0052)):
            q = [Vector((x0, surf(x0, zc(x0), off), zc(x0) + dz0)), Vector((x1, surf(x1, zc(x1), off), zc(x1) + dz0)),
                 Vector((x1, surf(x1, zc(x1), off), zc(x1) + dz1)), Vector((x0, surf(x0, zc(x0), off), zc(x0) + dz1))]
            face([bm.verts.new(L(v)) for v in q][::-1], m)
    # trucker cap: low-profile crown with a flat, nearly upright front panel; flat, slightly curved brim
    CB, CT = 0.028, 0.108                         # crown base and top heights (head-local z)
    front = lambda z: 0.114 - (z - CB) * 0.35     # the flat front panel leans back slightly
    def cap_ring(z, sc, n=16):
        pts = []
        for i in range(n):
            a = 2 * math.pi * i / n; ca, sa = math.cos(a), math.sin(a)
            x = math.copysign(abs(ca) ** 0.8, ca) * 0.107 * sc
            y = math.copysign(abs(sa) ** 0.8, sa) * 0.12 * sc - 0.006
            pts.append(Vector((x, min(y, front(z)), z)))
        return pts
    crings = [cap_ring(z, sc) for z, sc in ((CB, 1.0), (0.055, 0.99), (0.08, 0.93), (0.098, 0.8), (CT, 0.55))]
    CR = [[bm.verts.new(L(v)) for v in r] for r in crings]
    for j in range(len(CR) - 1):
        for i in range(16):
            k = (i + 1) % 16; face([CR[j][i], CR[j][k], CR[j + 1][k], CR[j + 1][i]], "black")
    ctop = bm.verts.new(L((0, -0.006, 0.13)))
    for i in range(16): face([CR[-1][i], CR[-1][(i + 1) % 16], ctop], "black")
    face(list(reversed(CR[0])), "black")
    # logo on the front panel: a pink fish silhouette facing left (from the front) with a tail and an eye
    # notch, and thin white dashes for the lettering underneath
    def FP(u, v, d=0.0025):
        u, v = u * 1.2, v * 1.2; z = 0.074 + v; return L((u, front(z) + d, z))
    body = [(0.024, 0.0), (0.016, 0.008), (0.004, 0.011), (-0.008, 0.009), (-0.016, 0.003), (-0.016, -0.003), (-0.008, -0.008), (0.004, -0.01), (0.016, -0.007)]
    cvt = bm.verts.new(FP(0.002, 0.0)); bv = [bm.verts.new(FP(u, v)) for u, v in body]
    for i in range(len(bv)):
        face([cvt, bv[i], bv[(i + 1) % len(bv)]], "pink")
    face([bm.verts.new(FP(u, v)) for u, v in ((-0.015, 0.0), (-0.029, 0.011), (-0.025, 0.0), (-0.029, -0.011))], "pink")   # forked tail
    face([bm.verts.new(FP(u, v, 0.0032)) for u, v in ((0.012, 0.001), (0.017, 0.001), (0.017, 0.005), (0.012, 0.005))], "black")  # eye notch
    for k in range(5):
        u = -0.024 + k * 0.012
        face([bm.verts.new(FP(u + du, v)) for du, v in ((-0.0045, -0.02), (0.0045, -0.02), (0.0045, -0.0155), (-0.0045, -0.0155))], "white")
    # brim: flat, slightly curved down at the sides, pitched a little down
    angs = [math.radians(a) for a in range(-75, 76, 15)]
    inner = [Vector((0.107 * math.sin(a) ** 1, min(0.12 * math.cos(a) - 0.006, front(CB)), CB)) for a in angs]
    outer = [v + Vector((0.012 * math.sin(a), 0.075 * math.cos(a) ** 0.6, -0.012 - 0.014 * abs(math.sin(a)))) for v, a in zip(inner, angs)]
    for lay, dz in ((0, 0.0), (1, -0.007)):
        I = [bm.verts.new(L(v + Vector((0, 0, dz)))) for v in inner]; O = [bm.verts.new(L(v + Vector((0, 0, dz)))) for v in outer]
        for i in range(len(I) - 1):
            q = [I[i], I[i + 1], O[i + 1], O[i]]
            face(q if lay == 0 else q[::-1], "black")
        if lay == 0: top = (I, O)
        else: bot = (I, O)
    for i in range(len(angs) - 1):
        face([top[1][i], top[1][i + 1], bot[1][i + 1], bot[1][i]], "black")
    # hair: a fuller light sandy mop: volume under the cap down to the ears and nape plus chunky overlapping
    # wedges that flick out at the ends around the ears and the back of the neck
    blob(L((0, -0.03, -0.012)), (0.096, 0.1, 0.124), "hair", 10, 6, tilt, zmin=-0.55)
    def add_lock(theta, drop, flick, wid, thick, jit, yaw=0.0, side=0.0):
        th = math.radians(theta)
        root = Vector((0.1 * math.sin(th), 0.11 * math.cos(th) - 0.02, 0.026))
        to = math.radians(theta + yaw)
        out = Vector((math.sin(to), math.cos(to), 0.0))
        sv = Vector((0, 0, 1)).cross(out).normalized()
        back = max(0.0, -math.cos(th))
        mid = root + out * (0.02 + 0.012 * jit) + sv * side * 0.25 + Vector((0, 0, -0.04 - 0.3 * drop))
        tip = root + out * (0.02 + 0.01 * jit) + sv * side * 0.4 + Vector((0, 0, -drop - 0.03 * back))
        end = tip + out * flick * 0.6 + sv * side * 0.6 + Vector((0, 0, 0.008 + 0.01 * jit))
        pts = [L(q) for q in bezier(root, mid, tip, 4)] + [L(end)]
        wdir = tilt @ Vector((0, 0, 1)).cross(out).normalized()
        lock(pts, [wid, wid * 1.05, wid * 0.95, wid * 0.75, wid * 0.4], [thick, thick * 0.95, thick * 0.85, thick * 0.65, thick * 0.35],
             wdir, ["hair", "hair", "hair_tip", "hair_tip"])
    jits = [0.3, -0.6, 0.9, -0.2, 0.5, -0.9, 0.1, 0.7, -0.4, 0.8, -0.7, 0.2]
    # (angle, length, flick, width, yaw, sideways) per wedge; varied so the back reads tousled, not a bob
    R = [(100, 0.07, 0.04, 0.07, -18, 0.03), (116, 0.115, 0.022, 0.066, 22, -0.035), (132, 0.08, 0.05, 0.075, -25, 0.04),
         (148, 0.13, 0.02, 0.068, 15, -0.03), (163, 0.085, 0.045, 0.078, -20, 0.035), (178, 0.125, 0.03, 0.072, 26, -0.04)]
    for k, (th, dr, fl, wd, yw, sd) in enumerate(R):
        for sgn in (-1, 1):
            j = jits[(k * 2 + (sgn > 0)) % len(jits)]
            if th == 178 and sgn > 0: th, yw, sd, dr = 184, -yw, -sd, dr * 0.7
            add_lock(sgn * th, dr * (1 + 0.25 * j), fl * (1 + 0.4 * j), wd, 0.03 * (1 + 0.3 * j), j, sgn * yw * (1 + 0.3 * j), sgn * sd)
    for th, dr, yw, sd in ((108, 0.1, 28, 0.03), (140, 0.075, -24, -0.035), (170, 0.095, 30, 0.03)):   # second, shorter layer
        for sgn in (-1, 1):
            add_lock(sgn * th, dr * (1.2 if sgn > 0 else 0.85), 0.045, 0.078, 0.036, 0.5 * sgn, sgn * yw, sgn * sd)
    for sgn in (-1, 1):                                    # short tufts above the ears, in front of the locks
        add_lock(sgn * 86, 0.045, 0.026, 0.05, 0.026, 0.4 * sgn, sgn * 14, 0.01)
    # arms: rounded sleeves with elbows and black cuffs; hands with fingers wrapped round the gamepad grips
    gp = Matrix.Rotation(math.radians(-25), 3, "X")
    for sx in (-1, 1):
        sh, el, wr = Vector((sx * 0.2, 0.2, 0.945)), Vector((sx * 0.25, 0.32, 0.7)), Vector((sx * 0.085, 0.5, 0.725))
        blob(sh, (0.06, 0.06, 0.056), "jacket", 6, 4)
        tube(sh, el, 0.062, 0.054, "jacket", 8, False); blob(el, (0.055, 0.055, 0.055), "jacket", 6, 3)
        tube(el, wr, 0.054, 0.046, "jacket", 8, False)
        tube(el.lerp(wr, 0.8), wr, 0.051, 0.048, "black", 8, False)
        g0, g1 = Vector((sx * 0.05, 0.55, 0.75)), Vector((sx * 0.066, 0.52, 0.708))
        box((sx * 0.083, 0.53, 0.728), (0.028, 0.07, 0.06), "skin", gp)                          # palm on the outside of the grip
        for k, t in enumerate((0.2, 0.42, 0.64, 0.86)):
            g = g0.lerp(g1, t); r = 0.0085 - 0.0008 * k
            a, b, c = g + Vector((sx * 0.026, 0.0, -0.004)), g + Vector((sx * 0.004, 0.006, -0.026)), g + Vector((-sx * 0.018, 0.004, -0.008))
            tube(a, b, r, r, "skin", 4, False); tube(b, c, r, r * 0.9, "skin", 4)
        tube((sx * 0.083, 0.535, 0.752), (sx * 0.05, 0.56, 0.775), 0.01, 0.009, "skin", 4, False)   # thumb
        tube((sx * 0.05, 0.56, 0.775), (sx * 0.032, 0.567, 0.786), 0.009, 0.008, "skin", 4)       # thumb tip on the stick
    # gamepad: body, grips, thumb sticks, buttons
    box((0, 0.56, 0.75), (0.15, 0.065, 0.03), "black", gp)
    for sx in (-1, 1):
        tube((sx * 0.05, 0.55, 0.75), (sx * 0.066, 0.52, 0.708), 0.021, 0.018, "black", 8)
        tube((sx * 0.03, 0.565, 0.765), (sx * 0.03, 0.562, 0.782), 0.01, 0.01, "case", 6)
    for bx, by in ((0.0, 0.575), (0.012, 0.565)):
        box((bx, by, 0.768), (0.008, 0.008, 0.006), "white", gp)

def laptop():
    # laptop on a second hard case in front of the pilot
    box((0, 0.88, 0.29), (0.46, 0.34, 0.58), "case")
    box((0, 0.86, 0.59), (0.34, 0.24, 0.018), "case")                         # laptop base
    box((0, 0.86, 0.601), (0.3, 0.14, 0.004), "black")                        # keyboard
    hinge = Vector((0, 0.975, 0.6)); lean = Matrix.Rotation(math.radians(15), 3, "X")
    box(hinge + lean @ Vector((0, 0.006, 0.115)), (0.34, 0.012, 0.23), "case", lean)          # lid
    box(hinge + lean @ Vector((0, -0.001, 0.115)), (0.31, 0.002, 0.2), "screen", lean)        # screen
    # sun shade: a dark fabric hood over the screen, open towards the pilot
    sh = Vector((0, 0.86, 0.6)); t = 0.008
    box(sh + Vector((0, 0.03, 0.31)), (0.4, 0.4, t), "black", Matrix.Rotation(math.radians(-6), 3, "X"))   # roof
    for sx in (-1, 1):
        side = [Vector(v) for v in [(sx * 0.2, 0.69, 0.6), (sx * 0.2, 1.06, 0.6), (sx * 0.2, 1.06, 0.92), (sx * 0.2, 0.66, 0.88)]]
        vs = [bm.verts.new(v) for v in side]; vs2 = [bm.verts.new(v + Vector((sx * t, 0, 0))) for v in side]
        face(vs if sx < 0 else list(reversed(vs)), "black"); face(list(reversed(vs2)) if sx < 0 else vs2, "black")
        for i in range(4):
            j = (i + 1) % 4; face([vs[i], vs[j], vs2[j], vs2[i]] if sx > 0 else [vs[j], vs[i], vs2[i], vs2[j]], "black")
    box((0, 1.065, 0.76), (0.41, t, 0.32), "black")                          # back
    box((0, 0.67, 0.875), (0.4, 0.03, 0.03), "black")                        # stiffened front lip of the hood

new_obj("PilotBody", body)
new_obj("PilotLaptop", laptop)
tot = 0
for ob in bpy.data.objects:
    n = sum(len(p.vertices) - 2 for p in ob.data.polygons); tot += n; print("TRIS", ob.name, n)
print("TRIS total", tot)
bpy.ops.export_scene.gltf(filepath=OUT, export_format="GLB", export_apply=True, export_normals=True,
                          export_texcoords=False, export_materials="EXPORT")
