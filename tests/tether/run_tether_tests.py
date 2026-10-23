#!/usr/bin/env python3
"""Run the tether validation scenarios headless and report stretch / clipping.

usage: GODOT=/path/to/godot tests/tether/run_tether_tests.py [--scenarios a,b] [--links N]
                                                            [--drag-axis z|y|both] [--out DIR] [-j N]

Each scenario runs in its own Godot process (tests/tether/tether_test.tscn) and writes a
JSON result; this script applies the thresholds below and exits non-zero on any FAIL.
"""
import argparse, concurrent.futures, json, os, subprocess, sys, tempfile

SCENARIOS = ["rest", "tow", "sprint", "spin", "circle", "edge_drag", "thin_plate", "floor_drag"]
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# (metric, warn, fail) in metres / fraction / degrees. Link spacing is 0.145 m and the
# collision capsule radius is 0.023 m, so 2 cm of gap or penetration is clearly visible.
LIMITS = [
    ("link_err", 0.005, 0.02),        # gap opening at any joint
    ("length_stretch", 0.005, 0.02),  # total tether elongation
    ("pen_static", 0.005, 0.015),     # sinking into floor / walls / pillar
    ("pen_rov", 0.005, 0.015),        # passing through the ROV hull
    ("self_clip", 0.005, 0.015),      # tether passing through itself
    ("bend_x", 12.0, 25.0),           # rope/joint.tscn limits: bend about x 10 deg,
    ("bend_z", 4.0, 10.0),            # bend about z 2 deg, twist about the long axis 10 deg
    ("twist", 12.0, 25.0),
]


def run_one(godot, scenario, drag_axis, links, steps, attach, outdir):
    tag = f"{scenario}-{drag_axis}-{attach}" + (f"-s{steps[0]}-{steps[1]}" if steps else "")
    out = os.path.join(outdir, f"{tag}.json")
    log = os.path.join(outdir, f"{tag}.log")
    cmd = [godot, "--headless", "--fixed-fps", "120", "--path", ROOT, "res://tests/tether/tether_test.tscn", "--",
           f"--scenario={scenario}", f"--out={out}", f"--links={links}", f"--drag-axis={drag_axis}", f"--attach={attach}"]
    if steps:
        cmd += [f"--velocity-steps={steps[0]}", f"--position-steps={steps[1]}"]
    with open(log, "w") as f:
        rc = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, timeout=1800).returncode
    errors = [l.strip() for l in open(log) if "SCRIPT ERROR" in l or l.startswith("ERROR")]
    if not os.path.exists(out):
        return {"scenario": scenario, "drag_axis": drag_axis, "crashed": True, "rc": rc, "errors": errors[:5]}
    r = json.load(open(out))
    r["errors"] = [e for e in errors if "leaked" not in e and "still in use" not in e][:5]
    return r


def grade(r):
    if r.get("crashed"):
        return "FAIL", ["did not finish"]
    notes, level = [], "PASS"
    def bump(l, msg):
        nonlocal level
        notes.append(msg)
        if l == "FAIL" or level == "PASS":
            level = l if level != "FAIL" else level
    for key, warn, fail in LIMITS:
        v = r["worst"][key]["value"]
        if v > fail:
            bump("FAIL", f"{key}={v:g}")
        elif v > warn:
            bump("WARN", f"{key}={v:g}")
    if r["through_count"]:
        bump("FAIL", f"through_solid x{r['through_count']}")
    if r["exploded"] or not r["completed"]:
        bump("FAIL", "exploded" if r["exploded"] else "incomplete")
    if any(s["below_floor"] for s in r["series"]):
        bump("FAIL", "sections below floor")
    if r["errors"]:
        bump("FAIL", "script errors")
    return level, notes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenarios", default=",".join(SCENARIOS))
    ap.add_argument("--links", type=int, default=200)
    ap.add_argument("--drag-axis", default="z", choices=["z", "y", "both"])
    ap.add_argument("--steps", action="append", default=[],
                    help="VEL/POS Jolt solver steps to test, e.g. 100/100; repeatable (default: project settings)")
    ap.add_argument("--attach", default="clamp", choices=["clamp", "ball", "both"])
    ap.add_argument("--out", default=None)
    ap.add_argument("-j", type=int, default=min(8, os.cpu_count() or 1))
    a = ap.parse_args()
    godot = os.environ.get("GODOT", "godot")
    outdir = a.out or tempfile.mkdtemp(prefix="tether-tests-")
    os.makedirs(outdir, exist_ok=True)
    axes = ["z", "y"] if a.drag_axis == "both" else [a.drag_axis]
    steps = [tuple(int(x) for x in s.split("/")) for s in a.steps] or [None]
    attaches = ["clamp", "ball"] if a.attach == "both" else [a.attach]
    jobs = [(s, ax, st, at) for st in steps for at in attaches for ax in axes for s in a.scenarios.split(",")]
    with concurrent.futures.ThreadPoolExecutor(a.j) as ex:
        results = list(ex.map(lambda j: run_one(godot, j[0], j[1], a.links, j[2], j[3], outdir), jobs))

    cols = ["link_err", "length_stretch", "pen_static", "pen_rov", "self_clip", "bend_x", "bend_z", "twist"]
    print(f"{'scenario':12} {'drag':4} {'attach':6} {'steps':>7} {'result':6} " + " ".join(f"{c:>14}" for c in cols) + f" {'through':>8} {'phys_ms':>8}  notes")
    failed = False
    for r in results:
        level, notes = grade(r)
        failed |= level == "FAIL"
        if r.get("crashed"):
            print(f"{r['scenario']:12} {r['drag_axis']:4} {'':6} {'':>7} {level:6} crashed rc={r['rc']} {r['errors']}")
            continue
        vals = " ".join(f"{r['worst'][c]['value']:>14g}" for c in cols)
        st = f"{r['velocity_steps']}/{r['position_steps']}"
        ms = [x["phys_ms"] for x in r["series"] if x["phys_ms"] > 0]
        r["phys_ms_mean"] = round(sum(ms) / len(ms), 2) if ms else 0
        print(f"{r['scenario']:12} {r['drag_axis']:4} {r['attach']:6} {st:>7} {level:6} {vals} {r['through_count']:>8} {r['phys_ms_mean']:>8}  {', '.join(notes)}")
    summary = {"results": [{k: v for k, v in r.items() if k != "series"} | {"grade": grade(r)} for r in results]}
    json.dump(summary, open(os.path.join(outdir, "summary.json"), "w"), indent=1)
    print(f"\nresults in {outdir}")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
