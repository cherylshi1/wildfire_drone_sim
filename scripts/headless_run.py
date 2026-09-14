"""Headless driver: stub missing visual assets, then measure or profile.

Usage:
  python3 scripts/headless_run.py measure 1 2 3 4 --runs 3
  python3 scripts/headless_run.py profile 4 --frames 1500
"""
import cProfile
import os
import pstats
import statistics
import sys

SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

os.environ["REAL_SIM_HEADLESS"] = "1"
os.environ["REAL_SIM_AUTOSTART"] = "1"
os.environ["REAL_SIM_AUTOSTART_FULL_AUTO"] = "1"
os.environ.setdefault("REAL_SIM_PARTICIPANT", "baseline")
os.environ["REAL_SIM_MANUAL_FRAME_PACING"] = "0"
os.environ["REAL_SIM_HEADLESS_FRAMES"] = "2"

sys.path.insert(0, SIM_DIR)
os.chdir(SIM_DIR)

# --- stub the loader so missing textures/models do not abort the load -------
from direct.showbase import Loader as _LoaderMod  # noqa: E402
from panda3d.core import NodePath, Texture, ModelRoot  # noqa: E402

_orig_load_texture = _LoaderMod.Loader.loadTexture
_orig_load_model = _LoaderMod.Loader.loadModel


def _safe_load_texture(self, *a, **k):
    try:
        return _orig_load_texture(self, *a, **k)
    except Exception:
        return Texture()


def _safe_load_model(self, *a, **k):
    try:
        return _orig_load_model(self, *a, **k)
    except Exception:
        return NodePath(ModelRoot("stub"))


_LoaderMod.Loader.loadTexture = _safe_load_texture
_LoaderMod.Loader.loadModel = _safe_load_model

from panda3d.core import ClockObject  # noqa: E402
from sim_sections import load_simulation  # noqa: E402

g = {"__file__": os.path.join(SIM_DIR, "main.py"), "__name__": "__main__"}
try:
    load_simulation(g)
except SystemExit:
    pass

clock = ClockObject.getGlobalClock()
clock.setMode(ClockObject.MNonRealTime)
clock.setDt(0.05)

live = g["update"].__globals__
step = g["update_simulation_frame"]


class FakeTask:
    cont = 1


def start_stage(stage):
    live["restart_simulation"]()
    live["show_pregame_setup_after_round"]()
    live["select_experiment_stage"](int(stage))
    live["start_game_from_pregame"]()
    live["set_full_automation"]()


def run_one(stage, dt=0.25, max_steps=6000):
    start_stage(stage)
    steps = 0
    while steps < max_steps and not live["game_over_triggered"]:
        step(FakeTask(), dt)
        steps += 1
    perf = live["compute_performance_matrix"](live["motion_state"].sim_time_seconds)
    hotspots = live["fire_hotspots"]
    out_state = live["FIRE_STATE_OUT"]
    total = len(hotspots)
    det = sum(1 for h in hotspots if h.detection_time_seconds is not None)
    out = sum(1 for h in hotspots if h.suppression_state == out_state)
    collab = live["compute_collaboration_score"]()
    return {
        "score": perf["performance_score"] * 100.0,
        "p1": perf["p1_score"] * 100.0,
        "p2": perf["p2_score"] * 100.0,
        "p3": perf["p3_score"] * 100.0,
        "p4": perf["p4_score"] * 100.0,
        "collab": collab["score"],
        "base": collab["base"],
        "success": (100.0 * out / total) if total else 0.0,
        "detect": (100.0 * det / total) if total else 0.0,
        "fires": total,
        "burn": live["burn_ratio"]() * 100.0,
        "drones": perf["team_total_drones"],
    }


def cmd_measure(argv):
    runs = 3
    stages = []
    i = 0
    while i < len(argv):
        if argv[i] == "--runs":
            i += 1
            runs = int(argv[i])
        elif argv[i].isdigit():
            stages.append(int(argv[i]))
        i += 1
    if not stages:
        stages = [1, 2, 3, 4]
    print("full-auto baselines: %d run(s)/stage" % runs)
    print("%-7s %-6s %-7s %-7s %-7s %-7s %-7s %-7s %-7s %-7s" % (
        "stage", "drones", "score", "collab", "succ%", "det%", "p1", "p2", "p3", "p4"))
    for s in stages:
        res = [run_one(s) for _ in range(runs)]

        def mean(k):
            return statistics.mean(r[k] for r in res)

        def sd(k):
            return statistics.pstdev([r[k] for r in res]) if len(res) > 1 else 0.0
        print("%-7d %-6d %6.2f%s %6.2f %6.1f %6.1f %6.1f %6.1f %6.1f %6.1f  fires=%.0f burn=%.0f" % (
            s, res[0]["drones"], mean("score"),
            ("~%.1f" % sd("score")) if sd("score") else "     ",
            mean("collab"), mean("success"), mean("detect"),
            mean("p1"), mean("p2"), mean("p3"), mean("p4"),
            mean("fires"), mean("burn")))


def cmd_profile(argv):
    frames = 1500
    stage = 4
    i = 0
    while i < len(argv):
        if argv[i] == "--frames":
            i += 1
            frames = int(argv[i])
        elif argv[i].isdigit():
            stage = int(argv[i])
        i += 1
    start_stage(stage)
    task = FakeTask()

    def loop():
        for _ in range(frames):
            if live["game_over_triggered"]:
                break
            step(task, 0.05)
    pr = cProfile.Profile()
    pr.enable()
    loop()
    pr.disable()
    st = pstats.Stats(pr)
    st.sort_stats("cumulative")
    print("=== stage %d, %d frames @ dt=0.05 (cumulative) ===" % (stage, frames))
    st.print_stats(35)
    st.sort_stats("tottime")
    print("=== by tottime ===")
    st.print_stats(35)


def cmd_calibrate(argv):
    """Measure the full-auto baseline for every stage on THIS machine and write
    calibration_baselines.json next to main.py. The sim loads that file, so
    after this runs, full automation lands on the anchor on this machine. Run it
    again whenever you retune the fire, resources or team."""
    import datetime
    import json
    stages = [int(a) for a in argv if a.isdigit()] or [0, 1, 2, 3, 4]
    baselines = {}
    print("measuring full-auto baselines on this machine...")
    for s in stages:
        r = run_one(s)
        baselines[str(s)] = round(r["score"], 2)
        print("  stage %d (%d drones): raw full-auto score %.2f" % (s, r["drones"], r["score"]))
    scored = [baselines[str(s)] for s in (1, 2, 3, 4) if str(s) in baselines]
    anchor = round(sum(scored) / len(scored), 2) if scored else None
    round_minutes = round(live["round_duration_seconds"] / 60.0, 1)
    print("(measured at %.0f-minute round length)" % round_minutes)
    payload = {
        "baselines": baselines,
        "anchor": anchor,
        "round_minutes": round_minutes,
        "measured_at": datetime.date.today().isoformat(),
        "note": "Full-auto raw P1-P4 x100 per stage, measured on this machine at a "
                "%.0f-minute round. Anchor = mean of stages 1-4. RE-MEASURE if you "
                "change the round length (3 vs 10 min) or retune." % round_minutes,
    }
    out_path = os.path.join(SIM_DIR, "calibration_baselines.json")
    with open(out_path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
    print("\nanchor (mean of stages 1-4) = %s" % anchor)
    print("wrote %s" % out_path)
    print("Full automation will now land on the anchor on this machine.")


def cmd_seedscan(argv):
    """Search fire-scenario seeds for one stage and pin the map whose full-auto
    baseline best matches the OTHER stages, then write scenario_seeds.json. Use
    it to pull a stage whose automation baseline is an outlier (e.g. stage 2 too
    low) in line with the rest, without touching any other setting.

    python3 scripts/headless_run.py seedscan 2 [--count 20]
    """
    import json
    stage = next((int(a) for a in argv if a.isdigit()), 2)
    count = 20
    if "--count" in argv:
        count = int(argv[argv.index("--count") + 1])

    # Target = mean of the OTHER scored stages' current baselines, so this stage
    # ends up consistent with them. Falls back to the anchor if none is stored.
    target = live["CALIBRATION_ANCHOR_SCORE"]
    baseline_path = os.path.join(SIM_DIR, "calibration_baselines.json")
    if os.path.exists(baseline_path):
        with open(baseline_path) as handle:
            stored = (json.load(handle).get("baselines") or {})
        others = [float(v) for k, v in stored.items()
                  if k.isdigit() and int(k) in (1, 2, 3, 4) and int(k) != stage]
        if others:
            target = sum(others) / len(others)

    stride = live["STAGE_SCENARIO_SEED_STRIDE"]
    base = live["STAGE_SCENARIO_SEED_BASE"] + stage * stride
    candidates = [base + i * 1009 for i in range(count)]
    override = live["STAGE_SCENARIO_SEED_OVERRIDE"]

    print("seedscan stage %d: matching the other stages' mean baseline %.2f" % (stage, target))
    print("trying %d candidate maps..." % count)
    results = []
    for seed in candidates:
        override[stage] = seed
        r = run_one(stage)
        results.append((seed, r["score"], r["success"], r["detect"]))
        print("  seed %d -> raw %5.2f  (succ %3.0f%%  det %3.0f%%)" % (
            seed, r["score"], r["success"], r["detect"]))
    override.pop(stage, None)

    best = min(results, key=lambda row: abs(row[1] - target))
    print("\nbest map for stage %d: seed %d -> raw %.2f (target %.2f)" % (
        stage, best[0], best[1], target))

    seeds_path = os.path.join(SIM_DIR, "scenario_seeds.json")
    payload = {"seeds": {}}
    if os.path.exists(seeds_path):
        try:
            with open(seeds_path) as handle:
                payload = json.load(handle)
                payload.setdefault("seeds", {})
        except Exception:
            payload = {"seeds": {}}
    payload["seeds"][str(stage)] = best[0]
    with open(seeds_path, "w") as handle:
        json.dump(payload, handle, indent=2)
    print("wrote %s (stage %d pinned to seed %d)" % (seeds_path, stage, best[0]))
    print("Now re-run:  python3 scripts/headless_run.py calibrate   to refresh the baselines.")


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "measure"
    if mode == "profile":
        cmd_profile(sys.argv[2:])
    elif mode == "calibrate":
        cmd_calibrate(sys.argv[2:])
    elif mode == "seedscan":
        cmd_seedscan(sys.argv[2:])
    else:
        cmd_measure(sys.argv[2:])
