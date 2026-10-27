// Heartfelt - by Martijn Steinrucken aka BigWings - 2017 (Shadertoy ltffzl)
// Email:countfrolic@gmail.com Twitter:@The_ArtOfCode
// License Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License.
//
// Surfacing drops v2 (BlueSim ROV camera dome): the v11 rain lens (scripts/fx/rain_lens_heartfelt.gdshader, dome
// flow, no lightning) with its drop, trail, static droplet, refraction and blur code unchanged, plus only this:
//  * the rain is "pre-warmed": at surfacing the drop field is Heartfelt's field at a random late time t0, i.e. the
//    glass as it looks after seconds of rain;
//  * no drop is added after that: a running-drop cell lives only if its drop was on the image at t0 (cells that
//    would scroll in from above the image stay dry), and the static droplets' fade cycle is clamped so a cell never
//    re-triggers (each one just fades out once);
//  * the drops keep running down/outward over the dome at Heartfelt's speed and leave the image; whatever is
//    left dries off per cell before `clear_time`, the fogged glass, tint and lens clear with it;
//  * a short sheet of water (extra blur) right at surfacing.
// v2.2, emergence mask: with `use_waterline` each pixel works out whether it is out of the water exactly as the
// waterline split does (scripts/camera_waterline.gd, from its line_uniforms() as the wl_* uniforms), else `emerge_tex`
// g says so for the whole image. `emerge_tex` r has, per cell of the image, how long since it came out (0 while in the
// water). Drops, fog, tint, lens and the sheet are drawn only where it is out, and dry by that age, so they show up
// where the image comes out of the water; the field still moves on the global `age`.
// Port edits as in v11: Godot built-ins, y flipped, rect-local coordinates (the rect can be a part of the window).
// Include after defining BASE_UV / BASE_SCALE / BASE_LOD and the `scene_tex` sampler.
#define S(a, b, t) smoothstep(a, b, t)

uniform float age = 0.0;                 // s since surfacing
uniform float clear_time = 9.0;          // s until the dome is dry
uniform sampler2D emerge_tex : filter_linear, repeat_disable;   // v2.2, over the rect's UV: r = s out / clear_time, g = out
uniform float cell_age_shift = 0.0;      // v2.2: s every cell has been out since emerge_tex was written
uniform bool use_waterline = false;      // v2.2: out of the water by the split's line below, not by emerge_tex g
uniform mat3 wl_basis;                   // the camera's global basis
uniform float wl_tan_half_fov = 0.5;
uniform float wl_aspect = 1.7778;
uniform float wl_line = 0.0;             // how far up a view direction points where it meets the water on the dome
uniform bool wl_cap = false;             // above the water: the water's underside shows above the horizon
uniform float wl_time = 0.0;             // s, the split's clock for its waves
uniform float fade = 1.0;                // overall blend (dive fade)
uniform vec2 seed = vec2(0.0);           // new per surfacing: picks the pre-warmed time t0
uniform vec2 rect_size = vec2(1152.0, 648.0);   // size of the part of the rect that is on screen (drop geometry)
uniform vec4 vis_uv = vec4(0.0, 0.0, 1.0, 1.0); // that part in the rect's UV: offset.xy, size.zw (v2.1)
uniform vec2 full_size = vec2(1152.0, 648.0);   // whole rect size (screen-texture offsets)
uniform float rain_amount = 0.9;         // Heartfelt rainAmount of the pre-warmed field (v11 lens: 0.9)
uniform float drop_speed = 2.0;          // Heartfelt t = T*.2*drop_speed (v11 lens: 2)
uniform float slide_speed = 1.0;         // multiplies how fast the field moves on after surfacing
uniform float zoom = 0.6;                // <1 = bigger drops (v11 drop_scale)
uniform float dome = 1.0;
uniform vec2 dome_center = vec2(0.0, 0.9);
uniform float dome_lens = 0.12;
uniform float fog = 0.4;                 // 1 = original heavy fogged glass
uniform float tint = 0.5;
uniform float burst_time = 0.3;          // s of the water sheet at surfacing
uniform float refraction = 1.0;
uniform bool debug_mask = false;
uniform int debug_part = 0;              // tests with debug_mask: 1 = running drops only (no beads/static), 2 = trail beads only
uniform bool no_respawn = true;          // tests: false = the original continuous Heartfelt rain (control run)

vec3 N13(float p) {
    //  from DAVE HOSKINS
   vec3 p3 = fract(vec3(p) * vec3(.1031,.11369,.13787));
   p3 += dot(p3, p3.yzx + 19.19);
   return fract(vec3((p3.x + p3.y)*p3.z, (p3.x+p3.z)*p3.y, (p3.y+p3.z)*p3.x));
}
float N(float t) {
    return fract(sin(t*12345.564)*7658.76);
}
float Saw(float b, float t) {
	return S(0., b, t)*S(1., b, t);
}
// v2.1: Heartfelt's running-drop cycle with a slightly slower rise (0.92 instead of 0.85 of the cycle), so that the
// drop never climbs faster in its cell than the cell runs down: the drop holds or moves outward, never toward the apex
// (with the apex above the image, outward always has a downward component). The fall is unchanged.
float RunSaw(float t) {
	return S(0., .92, t)*S(1., .85, t);
}

float T0() { return 2.0 + seed.x * 0.4 + seed.y * 0.03; }   // pre-warmed Heartfelt time (t units; small: hash precision)

// v2.1: centred image uv (y up) of a point given in a drop layer's coordinates (inverse of DomeWarp * zoom * scl)
vec2 LayerToImage(vec2 L, float scl) {
    vec2 w = L / (zoom * scl);
    float K = length(dome_center);
    vec2 domed = dome_center + K * exp(-w.y / K) * vec2(sin(w.x / K), -cos(w.x / K));
    return mix(w, domed, dome);                     // exact for dome 0 and 1, close in between
}
bool OnImage(vec2 p) {
    return abs(p.x) <= 0.5 * rect_size.x / max(rect_size.y, 1.0) && abs(p.y) <= 0.5;
}

// Heartfelt DropLayer2, unchanged except: cell alive mask (no drops from beyond the image after t0) and drying (by
// `la`, s since this part of the image came out of the water)
vec2 DropLayer2(vec2 uv, float t, float t0, float la, float scl) {
    vec2 UV0 = uv;
    uv.y += t*0.75;
    vec2 a = vec2(6., 1.);
    vec2 grid = a*2.;
    vec2 id = floor(uv*grid);
    float colShift = N(id.x);
    uv.y += colShift;
    id = floor(uv*grid);
    vec3 n = N13(id.x*35.2+id.y*2376.1);
    vec2 st = fract(uv*grid)-vec2(.5, 0);
    float x = n.x-.5;
    float y = UV0.y*20.;
    float wiggle = sin(y+sin(y));
    x += wiggle*(.5-abs(x))*(n.z-.5);
    x *= .7;
    float ti = fract(t+n.z);
    y = (RunSaw(ti)-.5)*.9+.5;
    vec2 p = vec2(x, y);
    float d = length((st-p)*a.yx);
    float mainDrop = S(.4, .0, d);
    float r = sqrt(S(1., y, st.y));
    float cd = abs(st.x-x);
    float trail = S(.23*r, .15*r*r, cd);
    float trailFront = S(-.02, .02, st.y-y);
    trail *= trailFront*r*r;
    y = UV0.y;
    float trail2 = S(.2*r, .0, cd);
    float droplets = max(0., (sin(y*(1.-y)*120.)-st.y))*trail2*trailFront*n.z;
    y = fract(y*10.)+(st.y-.5);
    float dd = length(st-vec2(x, y));
    droplets = S(.3, 0., dd);
    float m = mainDrop+droplets*r*trailFront;
    if (debug_part == 1) m = mainDrop;
    if (debug_part == 2) m = droplets*r*trailFront;
    // --- surfacing: this cell's drop at t0 (same formula); dead if it was above the top of the image then
    float y0 = (RunSaw(fract(t0+n.z))-.5)*.9+.5;
    float drop_y0 = (id.y + y0) / grid.y - t0*0.75 - colShift;         // layer y of the drop at t0
    // v2.1: alive only if the drop's own centre was on the image at t0 (all four edges, exact inverse dome warp):
    // nothing can come in from beyond the top edge (or creep in from the sides/bottom in Heartfelt's stick phase)
    vec2 drop_l0 = vec2((id.x + 0.5 + (n.x - .5) * .7) / grid.x, drop_y0);
    float alive = no_respawn ? (OnImage(LayerToImage(drop_l0, scl)) ? 1.0 : 0.0) : 1.0;
    float life = clear_time * mix(0.65, 0.95, n.y);                     // whatever has not run off dries by then
    if (no_respawn) alive *= 1.0 - S(life * 0.6, life, la);
    return vec2(m, trail) * alive;
}

// Heartfelt StaticDrops with the fade cycle clamped: phase at t0, then one pass to the end, never re-triggering
float StaticDrops(vec2 uv, float t0, float la) {
	uv *= 40.;
    vec2 id = floor(uv);
    uv = fract(uv)-.5;
    vec3 n = N13(id.x*107.45+id.y*3543.654);
    vec2 p = (n.xy-.5)*.7;
    float d = length(uv-p);
    float ph = mix(0.03, 1.0, fract(t0+n.z));                           // already landed at surfacing
    float k = clamp(la / max(clear_time * 0.9, 0.1), 0.0, 1.0);
    float ti = mix(ph, 1.0, k * k);                                     // slow at first, each one gone by 0.9*clear_time
    float fade_ = ti < 1.0 ? Saw(.025, ti) : 0.0;
    if (!no_respawn) fade_ = Saw(.025, fract(t0 + age*.2*drop_speed + n.z));   // original: re-triggers every cycle
    float c = S(.3, 0., d)*fract(n.z*10.)*fade_;
    return c;
}

vec2 DomeWarp(vec2 uv) {
    vec2 d = uv - dome_center;
    float r = max(length(d), 1e-3);
    float ang = atan(d.x, -d.y);                    // 0 = straight down from the apex
    float K = length(dome_center);                  // keeps drop scale ~unchanged at the screen centre
    vec2 w = vec2(ang * K, -log(r / K) * K);        // conformal: Heartfelt "down" (-y) == radially outward
    return mix(uv, w, dome) * zoom;
}

vec2 Drops(vec2 uv, float t, float t0, float la, float l0, float l1, float l2) {
    float s = debug_part > 0 ? 0.0 : StaticDrops(uv, t0, la)*l0;
    vec2 m1 = DropLayer2(uv, t, t0, la, 1.0)*l1;
    vec2 m2 = DropLayer2(uv*1.85, t, t0, la, 1.85)*l2;
    float c = s+m1.x+m2.x;
    c = S(.3, 1., c);
    return vec2(c, max(m1.y*l0, m2.y*l1));
}

// v2.2: 1 where the camera's image (uv over the whole rect, y down) is out of the water, as SPLIT_SHADER draws it
float OutOfWater(vec2 uv) {
    vec2 ndc = uv * 2.0 - 1.0;
    vec3 view = normalize(wl_basis * vec3(ndc.x * wl_tan_half_fov * wl_aspect, -ndc.y * wl_tan_half_fov, -1.0));
    float heading = atan(view.x, view.z);
    float line = wl_line + 0.03 * sin(heading * 7.0 + wl_time * 2.3) + 0.015 * sin(heading * 19.0 - wl_time * 3.7);
    if (wl_cap) line = min(line, 0.0);
    return S(-0.006, 0.006, view.y - line);
}

vec3 Base(vec2 buv, float lod) {
#ifdef BASE_LOD
    return textureLod(scene_tex, buv, lod).rgb;
#else
    // no mipmaps on a SubViewport texture: small disc blur with the same radius as mip level `lod`
    vec3 c = texture(scene_tex, buv).rgb;
    float rad = exp2(lod) - 1.0;
    if (rad < 0.3) return c;
    vec2 px = rad / rect_size;
    for (int i = 0; i < 8; i++) {
        float an = float(i) * 2.39996 + 0.5;
        float rr = sqrt((float(i) + 0.5) / 8.0);
        c += texture(scene_tex, buv + vec2(cos(an), sin(an)) * rr * px).rgb;
    }
    return c / 9.0;
#endif
}

void fragment() {
    vec2 u_resolution = rect_size;
    vec2 U = (UV - vis_uv.xy) / vis_uv.zw;                              // 0..1 over the on-screen part
    vec2 fc = vec2(U.x, 1.0 - U.y) * u_resolution;                      // GLSL bottom-left origin
	vec2 uv = (fc-.5*u_resolution.xy) / u_resolution.y;
    vec2 UVn = fc/u_resolution.xy;
    float t0 = T0();
    float t = t0 + age*.2*drop_speed*slide_speed;
    // v2.2: s since this part of the image came out of the water, and whether it is out
    vec4 emerge = texture(emerge_tex, UV);
    float local_age = emerge.r * clear_time + cell_age_shift;
    float out_mask = use_waterline ? OutOfWater(UV) : S(.35, .65, emerge.g);
    float rainAmount = rain_amount;
    float wet = 1.0 - S(clear_time * 0.25, clear_time * 0.85, local_age);    // glass fog / tint / lens clear with the drops
    if (!no_respawn) wet = 1.0;
    wet *= out_mask;
    float sheet = (1.0 - S(0.0, max(burst_time, 1e-3), local_age)) * out_mask;

    float maxBlur = mix(3., 6., rainAmount);
    float minBlur = 2.;
    float staticDrops = S(-.5, 1., rainAmount)*2.;
    float layer1 = S(.25, .75, rainAmount);
    float layer2 = S(.0, .5, rainAmount);
    vec2 c = Drops(DomeWarp(uv), t, t0, local_age, staticDrops, layer1, layer2) * out_mask;
   	vec2 e = vec2(.001, 0.);
   	float cx = Drops(DomeWarp(uv+e), t, t0, local_age, staticDrops, layer1, layer2).x * out_mask;
   	float cy = Drops(DomeWarp(uv+e.yx), t, t0, local_age, staticDrops, layer1, layer2).x * out_mask;
   	vec2 n = vec2(cx-c.x, cy-c.x) * refraction;		// expensive normals
    float focus = mix(maxBlur-c.y, minBlur, S(.1, .2, c.x)) * fog * wet + sheet * 3.0;
    vec2 suv = vec2(UVn.x, 1.0 - UVn.y);   // back to rect UV (top-left)
    vec2 q = suv - 0.5; suv = 0.5 + q * (1.0 - dome * dome_lens * wet * dot(q, q) * 2.0);   // dome lens (barrel)
    vec2 sn = vec2(n.x, -n.y);
    vec3 col = Base(BASE_UV + (suv + sn - U) * vis_uv.zw * BASE_SCALE, focus);
    // USE_POST_PROCESSING (original Shadertoy, lightning removed), scaled by how wet the glass still is
    float T = t0 / (.2 * drop_speed) + age;
    float tt = (T+3.)*.5;
    float colFade = sin(tt*.2)*.5+.5;
    float tn = tint * wet;
    col *= mix(vec3(1.), mix(vec3(1.), vec3(.8, .9, 1.3), colFade), tn);
    vec2 vv = UVn - .5;
    col *= 1.-dot(vv, vv)*tn;
    col = mix(texture(scene_tex, BASE_UV).rgb, col, fade);
    if (debug_mask) col = vec3(S(.1, .2, c.x));
    COLOR = vec4(col, 1.);
}
