"""Measure the full-automation baseline score for each experiment stage.

    python3 scripts/measure_baselines.py                 stages 1-4, 15 runs each
    python3 scripts/measure_baselines.py 2 3 4 --runs 25
    python3 scripts/measure_baselines.py --csv out.csv

For calibration we need to know two things about a hands-off (full automation)
round in every config:

  1. where its performance score lands on average (the automation baseline), and
  2. how much that number wobbles from run to run.

Calibration maps each config's automation baseline onto one common anchor, so
the baseline has to be STABLE (low spread) for the anchor to mean anything. This
script runs each stage full-auto many times, records the P1-P4 performance score
and its parts, and prints mean / std / min / max so we can see the spread.

Note: this drives update_simulation_frame directly. Do NOT call update(), whose
pace_main_loop() waits on real time that never advances under MNonRealTime.
"""

import os
import statistics
import sys

SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

runs = 15
csv_path = None
args = sys.argv[1:]
stages = []
index = 0
while index < len(args):
    token = args[index]
    if token == "--runs":
        index += 1
        runs = int(args[index])
    elif token == "--csv":
        index += 1
        csv_path = args[index]
    elif token.isdigit():
        stages.append(int(token))
    index += 1
if not stages:
    stages = [1, 2, 3, 4]

os.environ["REAL_SIM_HEADLESS"] = "1"
os.environ["REAL_SIM_AUTOSTART"] = "1"
os.environ["REAL_SIM_AUTOSTART_FULL_AUTO"] = "1"
os.environ.setdefault("REAL_SIM_PARTICIPANT", "baseline")
os.environ["REAL_SIM_MANUAL_FRAME_PACING"] = "0"
os.environ["REAL_SIM_HEADLESS_FRAMES"] = "2"
os.environ["REAL_SIM_STAGE"] = str(stages[0])

sys.path.insert(0, SIM_DIR)
os.chdir(SIM_DIR)

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
step_simulation = g["update_simulation_frame"]


class FakeTask:
    cont = 1


def run_one(stage):
    live["restart_simulation"]()
    live["show_pregame_setup_after_round"]()
    live["select_experiment_stage"](int(stage))
    live["start_game_from_pregame"]()
    live["set_full_automation"]()
    steps = 0
    while steps < 6000 and not live["game_over_triggered"]:
        step_simulation(FakeTask(), 0.25)
        steps += 1
    performance = live["compute_performance_matrix"](
        live["motion_state"].sim_time_seconds
    )
    hotspots = live["fire_hotspots"]
    fires_total = len(hotspots)
    fires_detected = sum(
        1 for h in hotspots if h.detection_time_seconds is not None
    )
    fire_state_out = live["FIRE_STATE_OUT"]
    fires_out = sum(1 for h in hotspots if h.suppression_state == fire_state_out)
    success_rate = (100.0 * fires_out / fires_total) if fires_total else 0.0
    detect_rate = (100.0 * fires_detected / fires_total) if fires_total else 0.0
    return {
        "success_rate": success_rate,
        "detect_rate": detect_rate,
        "fires_total": fires_total,
        "fires_detected": fires_detected,
        "fires_out": fires_out,
        "active_trucks": live["active_fire_truck_count"],
        "tank_liters": live["WATER_DRONE_TANK_CAPACITY_LITERS"],
        "score": performance["performance_score"] * 100.0,
        "p1": performance["p1_score"] * 100.0,
        "p2": performance["p2_score"] * 100.0,
        "p3": performance["p3_score"] * 100.0,
        "p4": performance["p4_score"] * 100.0,
        "burned": live["burn_ratio"]() * 100.0,
        "total_drones": performance["team_total_drones"],
        "survey_drones": performance["team_survey_drones"],
        "water_drones": performance["team_water_drones"],
    }


def summarize(label, values):
    mean = statistics.mean(values)
    std = statistics.pstdev(values) if len(values) > 1 else 0.0
    return "%-8s mean %6.2f  std %5.2f  min %6.2f  max %6.2f" % (
        label,
        mean,
        std,
        min(values),
        max(values),
    )


rows = []
print("full-auto baseline: %d run(s) per stage\n" % runs)
for stage in stages:
    results = [run_one(stage) for _ in range(runs)]
    for run_index, result in enumerate(results):
        rows.append((stage, run_index, result))
    scores = [result["score"] for result in results]
    label = "stage %d" % stage
    drones = results[0]["total_drones"]
    first = results[0]
    print("=== %s  (%d drones)  trucks=%d  tank=%.0fL ===" % (
        label, drones, first["active_trucks"], first["tank_liters"]))
    print("  " + summarize("SUCCESS%", [r["success_rate"] for r in results])
          + "   (fires out / total)")
    print("  " + summarize("detect%", [r["detect_rate"] for r in results]))
    print("  fires: total=%d detected=%d out=%d" % (
        first["fires_total"], first["fires_detected"], first["fires_out"]))
    print("  " + summarize("score", scores))
    for part in ("p1", "p2", "p3", "p4", "burned"):
        print("  " + summarize(part, [result[part] for result in results]))
    print()

if csv_path:
    import csv

    with open(csv_path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            ["stage", "run", "total_drones", "survey_drones", "water_drones",
             "score", "p1", "p2", "p3", "p4", "burned"]
        )
        for stage, run_index, result in rows:
            writer.writerow([
                stage, run_index, result["total_drones"],
                result["survey_drones"], result["water_drones"],
                "%.3f" % result["score"], "%.3f" % result["p1"],
                "%.3f" % result["p2"], "%.3f" % result["p3"],
                "%.3f" % result["p4"], "%.3f" % result["burned"],
            ])
    print("wrote %s (%d rows)" % (csv_path, len(rows)))
