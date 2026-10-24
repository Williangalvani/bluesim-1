#!/usr/bin/env python3
"""Record videos of the tether scenarios for visual validation.

usage: GODOT=/path/to/godot tests/tether/record_tether_videos.py [--scenarios a,b] [--out DIR] [-j N]
           [--camera wide|side|attach] [--steps V/P] [--bend-limits X,Y,Z] [--seed N] [--no-combine]

Each scenario runs tests/tether/tether_test.tscn with --render under Godot's Movie Maker
(120 fps, so physics matches the headless tests exactly) and is encoded to a real-time MP4
with the live metrics overlay. Outputs in --out:

  tether-<scenario>.mp4      one video per scenario
  tether-<scenario>.png      contact sheet (6 frames)
  tether-all.mp4             every scenario back to back with a title card (unless --no-combine)
  <scenario>.json / .log     the raw test result and Godot log

Display: by default every recording gets its own private Xvfb server (install xorg-server-xvfb
/ xvfb), so recordings work with the screen locked or off and can run in parallel. GNOME and
other compositors stop redrawing windows while the screen is locked or covered, and Movie
Maker then silently repeats the last frame. Without Xvfb, --display desktop records on the
current session (keep the window visible and the screen unlocked).

Every video is checked for frozen frames (a corner box that changes colour on every physics
frame must keep changing) and for its duration against the
simulated time; a recording that fails either check is reported as INVALID and the script exits
non-zero.
"""
import argparse, concurrent.futures, json, os, re, shutil, subprocess, sys, tempfile, time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
from run_tether_tests import SCENARIOS, add_sustained, grade  # noqa: E402

FONT_CANDIDATES = ["/usr/share/fonts/noto/NotoSans-Bold.ttf", "/usr/share/fonts/TTF/DejaVuSans-Bold.ttf",
                   "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"]
MOVIE_FPS = 120  # must match the physics rate so every physics step is one movie frame


def font():
    return next((f for f in FONT_CANDIDATES if os.path.exists(f)), None)


class Xvfb:
    """A private X server for one recording."""

    def __init__(self, binary, display):
        self.display = display
        self.proc = subprocess.Popen([binary, f":{display}", "-screen", "0", "1280x720x24", "-nolisten", "tcp",
                                      "-noreset"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for _ in range(50):
            if os.path.exists(f"/tmp/.X11-unix/X{display}"):
                return
            time.sleep(0.1)
        raise RuntimeError(f"Xvfb :{display} did not start")

    def env(self):
        # Xvfb has no DRI3, so have Mesa present Vulkan frames with a CPU copy
        return dict(os.environ, DISPLAY=f":{self.display}", MESA_VK_WSI_DEBUG="sw")

    def close(self):
        self.proc.terminate()
        self.proc.wait(timeout=10)


def free_display(start=90):
    d = start
    while os.path.exists(f"/tmp/.X11-unix/X{d}") or os.path.exists(f"/tmp/.X{d}-lock"):
        d += 1
    return d


def screen_locked():
    try:
        out = subprocess.run(["gdbus", "call", "--session", "--dest", "org.gnome.ScreenSaver", "--object-path",
                              "/org/gnome/ScreenSaver", "--method", "org.gnome.ScreenSaver.GetActive"],
                             capture_output=True, text=True, timeout=5).stdout
        return "true" in out
    except (OSError, subprocess.TimeoutExpired):
        return False


def frozen_segments(video, duration):
    """Stalls of >= 0.5 s in the frame-tick box (top right), which changes colour every physics frame."""
    r = subprocess.run(["ffmpeg", "-hide_banner", "-i", video, "-vf",
                        "crop=iw*0.02:iw*0.02:iw-iw*0.02:0,freezedetect=n=-30dB:d=0.5", "-map", "0:v", "-f", "null", "-"],
                       capture_output=True, text=True)
    starts = [float(x) for x in re.findall(r"freeze_start: ([\d.]+)", r.stderr)]
    durs = [float(x) for x in re.findall(r"freeze_duration: ([\d.]+)", r.stderr)]
    # a freeze still open at the end of the file has no duration line
    durs += [duration - s for s in starts[len(durs):]]
    return [(round(s, 2), round(d, 2)) for s, d in zip(starts, durs)]


def probe_duration(video):
    out = subprocess.run(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0", video],
                         capture_output=True, text=True).stdout
    return float(out.strip() or 0)


def record(args, scenario, display_lock):
    out = args.out
    avi, log, js = f"{out}/{scenario}.avi", f"{out}/{scenario}.log", f"{out}/{scenario}.json"
    mp4, sheet = f"{out}/tether-{scenario}.mp4", f"{out}/tether-{scenario}.png"
    cmd = [args.godot, "--resolution", "1280x720", "--path", ROOT, "res://tests/tether/tether_test.tscn",
           "--write-movie", avi, "--fixed-fps", str(MOVIE_FPS), "--",
           f"--scenario={scenario}", f"--out={js}", f"--render={args.camera}", f"--seed={args.seed}"]
    if args.steps:
        v, p = args.steps.split("/")
        cmd += [f"--velocity-steps={v}", f"--position-steps={p}"]
    if args.bend_limits:
        cmd += [f"--bend-limits={args.bend_limits}"]
    xvfb = None
    if args.display == "xvfb":
        with display_lock:
            xvfb = Xvfb(args.xvfb, free_display())
        cmd[1:1] = ["--display-driver", "x11"]
        env = xvfb.env()
    else:
        env = dict(os.environ)
    try:
        with open(log, "w") as f:
            subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, env=env, timeout=3600)
    finally:
        if xvfb:
            xvfb.close()
    text = open(log).read()
    m = re.search(r"first_frame=(\d+)", text)
    if not m or not os.path.exists(avi) or not os.path.exists(js):
        return {"scenario": scenario, "valid": False, "problem": "recording failed, see " + log}
    first = int(m.group(1))
    subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-ss", f"{first / MOVIE_FPS:.3f}", "-i", avi,
                    "-vf", "fps=30", "-c:v", "libx264", "-preset", "slow", "-crf", "27", "-pix_fmt", "yuv420p",
                    "-movflags", "+faststart", mp4], check=True)
    os.remove(avi)

    r = json.load(open(js))
    add_sustained(r)
    r["errors"] = [l for l in text.splitlines() if "SCRIPT ERROR" in l][:5]
    level, notes = grade(r)
    dur = probe_duration(mp4)
    freezes = frozen_segments(mp4, dur)
    problems = []
    if freezes:
        problems.append(f"frozen video at {freezes}")
    if abs(dur - r["duration"]) > 0.5:
        problems.append(f"video {dur:.1f}s vs simulated {r['duration']:.1f}s")

    # contact sheet: 6 frames spread over the run, labelled with their time
    f = font()
    tmp = tempfile.mkdtemp(prefix=f"sheet-{scenario}-")
    for i in range(6):
        t = dur * (i + 0.5) / 6
        label = f",drawtext=fontfile={f}:text='{t:.1f} s':x=w-tw-16:y=h-th-16:fontsize=32:fontcolor=white:box=1:boxcolor=black@0.6" if f else ""
        subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-ss", f"{t:.2f}", "-i", mp4, "-frames:v", "1",
                        "-vf", "scale=640:-2" + label, f"{tmp}/{i}.png"], check=True)
    subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-i", f"{tmp}/%d.png", "-vf", "tile=2x3", sheet], check=True)
    shutil.rmtree(tmp)
    return {"scenario": scenario, "valid": not problems, "problem": "; ".join(problems), "grade": level,
            "notes": notes, "video": mp4, "sheet": sheet, "duration": dur, "worst": r["worst"],
            "through_count": r["through_count"]}


def combine(results, out):
    """All scenarios back to back, each preceded by a 2 s title card with its grade."""
    f = font()
    parts = []
    tmp = tempfile.mkdtemp(prefix="combine-")
    for i, r in enumerate(results):
        if not r.get("video"):
            continue
        w, h = map(int, subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v", "-show_entries",
                                        "stream=width,height", "-of", "csv=p=0", r["video"]],
                                       capture_output=True, text=True).stdout.strip().split(","))
        title = f"{tmp}/{i}-title.mp4"
        text = f"{r['scenario']}    {r['grade']}" + ("" if r["valid"] else "    (INVALID RECORDING)")
        draw = f"drawtext=fontfile={f}:text='{text}':x=(w-tw)/2:y=(h-th)/2:fontsize=56:fontcolor=white" if f else "null"
        subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-f", "lavfi", "-i", f"color=c=0x1d3b53:s={w}x{h}:d=2:r=30",
                        "-vf", draw, "-c:v", "libx264", "-pix_fmt", "yuv420p", title], check=True)
        parts += [title, r["video"]]
    lst = f"{tmp}/list.txt"
    with open(lst, "w") as fh:
        fh.writelines(f"file '{p}'\n" for p in parts)
    dst = f"{out}/tether-all.mp4"
    subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-f", "concat", "-safe", "0", "-i", lst, "-c:v", "libx264",
                    "-preset", "slow", "-crf", "27", "-pix_fmt", "yuv420p", "-movflags", "+faststart", dst], check=True)
    shutil.rmtree(tmp)
    return dst


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenarios", default=",".join(SCENARIOS))
    ap.add_argument("--out", default=None)
    ap.add_argument("-j", type=int, default=4, help="parallel recordings (xvfb only; desktop is always 1)")
    ap.add_argument("--camera", default="wide", choices=["wide", "side", "attach"])
    ap.add_argument("--steps", help="override Jolt solver steps, e.g. 100/50")
    ap.add_argument("--bend-limits", help="override joint limits X,Y,Z in degrees")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--display", choices=["xvfb", "desktop"], default=None,
                    help="default: xvfb if available, otherwise desktop")
    ap.add_argument("--xvfb", default=os.environ.get("XVFB", shutil.which("Xvfb")), help="Xvfb binary ($XVFB)")
    ap.add_argument("--no-combine", action="store_true")
    a = ap.parse_args()
    a.godot = os.environ.get("GODOT", "godot")
    a.out = os.path.abspath(a.out or tempfile.mkdtemp(prefix="tether-videos-"))
    os.makedirs(a.out, exist_ok=True)
    a.display = a.display or ("xvfb" if a.xvfb else "desktop")
    if a.display == "xvfb" and not a.xvfb:
        sys.exit("Xvfb not found: install xorg-server-xvfb (or xvfb), set $XVFB, or use --display desktop")
    if a.display == "desktop":
        a.j = 1
        print("recording on the desktop session: keep the Godot window visible and the screen unlocked")
        if screen_locked():
            print("WARNING: the screen is locked; the compositor will not redraw and the videos will freeze")
    for tool in ("ffmpeg", "ffprobe"):
        if not shutil.which(tool):
            sys.exit(f"{tool} not found")

    import threading
    display_lock = threading.Lock()
    scenarios = a.scenarios.split(",")
    with concurrent.futures.ThreadPoolExecutor(a.j) as ex:
        results = list(ex.map(lambda s: record(a, s, display_lock), scenarios))

    print(f"{'scenario':12} {'video':8} {'grade':6} {'length':>7}  notes")
    for r in results:
        status = "ok" if r["valid"] else "INVALID"
        print(f"{r['scenario']:12} {status:8} {r.get('grade', '-'):6} {r.get('duration', 0):6.1f}s  "
              f"{r['problem'] or ', '.join(r.get('notes', []))}")
    if not a.no_combine:
        print("combined:", combine(results, a.out))
    json.dump(results, open(f"{a.out}/videos.json", "w"), indent=1)
    print(f"videos in {a.out}")
    sys.exit(0 if all(r["valid"] for r in results) else 1)


if __name__ == "__main__":
    main()
