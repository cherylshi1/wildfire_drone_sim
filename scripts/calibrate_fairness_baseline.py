"""Fairness calibration (July 9 protocol): hands-off full-auto baseline.

Runs each official team size (2/4/6 drones) under FULL AUTOMATION with zero
operator input and prints the raw composite score, so the per-team-size fire
multipliers (FIRE_TEAM_SIZE_SPREAD_RATE_MULTIPLIERS in section 01) can be
tuned until the do-nothing score matches across team sizes.

Run from the real sim folder:
    python3 scripts/calibrate_fairness_baseline.py            # default seeds
    python3 scripts/calibrate_fairness_baseline.py 1 2 3 4 5  # your seeds

Each (stage, seed) pair is a separate process so runs cannot leak state.
Full rounds take a few minutes each; results append to
scripts/fairness_baseline_results.txt so overnight batches are safe.
"""
import os
import subprocess
import sys

SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS_PATH = os.path.join(SIM_DIR, "scripts", "fairness_baseline_results.txt")

SINGLE_RUN_TEMPLATE = r"""
import os, sys, types, random
os.environ["REAL_SIM_HEADLESS"] = "1"
os.environ["REAL_SIM_HEADLESS_FRAMES"] = "1"
os.chdir({sim_dir!r})
sys.path.insert(0, {sim_dir!r})
from sim_sections import load_simulation
g = {{"__file__": os.path.join({sim_dir!r}, "main.py"), "__name__": "__main__"}}
try:
    load_simulation(g)
except SystemExit:
    pass
g["REAL_SIM_HEADLESS_FRAME_LIMIT"] = 0
g["REAL_SIM_MANUAL_FRAME_PACING"] = False
random.seed({seed})
g["confirm_pregame_or_restart"]()
g["select_experiment_stage"]({stage})
g["confirm_pregame_or_restart"]()
g["set_full_automation"]()
from panda3d.core import ClockObject
clock = ClockObject.getGlobalClock()
clock.setMode(ClockObject.MNonRealTime)
clock.setDt(0.05)
fake_task = types.SimpleNamespace(cont=1, again=2, done=0, time=0.0)
while (g["motion_state"].sim_time_seconds < g["round_duration_seconds"]
       and not g["game_over_triggered"]):
    g["update"](fake_task)
perf = g["compute_performance_matrix"](g["motion_state"].sim_time_seconds)
grade = g["compute_final_grade"]()
hotspots = g["fire_hotspots"]
detected = sum(1 for h in hotspots if h.detection_time_seconds is not None)
out = sum(1 for h in hotspots if h.suppression_state == g["FIRE_STATE_OUT"])
print("BASELINE stage={stage} team=%d seed={seed} mult=%s raw=%.3f grade=%.1f "
      "burn=%.3f fires=%d det=%d out=%d t=%.0f" % (
          g["team_total_drone_count"],
          g["FIRE_TEAM_SIZE_SPREAD_RATE_MULTIPLIERS"],
          perf["raw_performance_score"], grade["score"], g["burn_ratio"](),
          len(hotspots), detected, out, g["motion_state"].sim_time_seconds))
"""


def main():
    seeds = [int(argument) for argument in sys.argv[1:]] or [11, 12, 13, 14, 15]
    with open(RESULTS_PATH, "a") as results_file:
        for stage in (2, 3, 4):
            for seed in seeds:
                code = SINGLE_RUN_TEMPLATE.format(
                    sim_dir=SIM_DIR, stage=stage, seed=seed
                )
                completed = subprocess.run(
                    [sys.executable, "-c", code],
                    capture_output=True, text=True,
                )
                for line in completed.stdout.splitlines():
                    if line.startswith("BASELINE"):
                        print(line)
                        results_file.write(line + "\n")
                        results_file.flush()
                        break
                else:
                    print(f"RUN FAILED stage={stage} seed={seed}")
                    print(completed.stderr[-500:])


if __name__ == "__main__":
    main()
