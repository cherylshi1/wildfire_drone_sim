"""Headless check of the July 29 trust matrix work.

Runs two short fully-automatic rounds, then asserts the trust matrix numbers,
the run log, the per-run tables (with the dS/dW/dTotal columns) and the exports.

    python3 scripts/verify_trust_matrix.py 1   (then 2)
"""

import os
import sys

SIM_DIR = "/sessions/vibrant-dazzling-einstein/mnt/Desktop/Summer26 FSC/human trust w auto sys/real sim"

os.environ["REAL_SIM_HEADLESS"] = "1"
os.environ["REAL_SIM_AUTOSTART"] = "1"
os.environ["REAL_SIM_AUTOSTART_FULL_AUTO"] = "1"
os.environ["REAL_SIM_STAGE"] = "3"
os.environ.setdefault("REAL_SIM_PARTICIPANT", "trustcheck")
os.environ["REAL_SIM_ROUND_MINUTES"] = "1"
os.environ["REAL_SIM_TIME_SCALE"] = "6"
os.environ["REAL_SIM_MAX_SUBSTEP"] = "0.4"
os.environ["REAL_SIM_MANUAL_FRAME_PACING"] = "0"
os.environ["REAL_SIM_HEADLESS_FRAMES"] = "2"  # let app.run() exit, then drive update() by hand

PHASE = sys.argv[1] if len(sys.argv) > 1 else "1"

sys.path.insert(0, SIM_DIR)
os.chdir(SIM_DIR)

from panda3d.core import ClockObject  # noqa: E402

from sim_sections import load_simulation  # noqa: E402

g = {"__file__": os.path.join(SIM_DIR, "main.py"), "__name__": "__main__"}
try:
    load_simulation(g)
except SystemExit:
    pass

checks = []


def check(label, condition, detail=""):
    checks.append((label, bool(condition), detail))
    print(("  OK   " if condition else "  FAIL ") + label + (("  " + detail) if detail else ""))


clock = ClockObject.getGlobalClock()
clock.setMode(ClockObject.MNonRealTime)
clock.setDt(0.05)

update = g["update"]
live = update.__globals__
# NOTE: do not call update() in a scripted run. Its pace_main_loop() waits on
# real time, which never advances under ClockObject.MNonRealTime, so it hangs.
# Drive the inner step directly, exactly as the fast profile does per substep.
step_simulation = g["update_simulation_frame"]
STEP_SECONDS = 0.25


class FakeTask:
    cont = 1


def run_round(max_frames=4000):
    frames = 0
    while frames < max_frames and not live["game_over_triggered"]:
        step_simulation(FakeTask(), STEP_SECONDS)
        frames += 1
    return frames


def report_trust(tag):
    rows = live["compute_trust_metrics"]()
    print("\n%s trust matrix:" % tag)
    for line in live["compose_trust_matrix_lines"](rows):
        print("   " + line)
    return rows


if PHASE == "1":
    print("\n--- round 1 ---")
    frames = run_round()
    print("frames %d, sim time %.1f s, over=%s"
          % (frames, live["motion_state"].sim_time_seconds, live["game_over_triggered"]))
    check("round 1 reached game over", live["game_over_triggered"],
          "%.1f s of sim" % live["motion_state"].sim_time_seconds)

    rows1 = report_trust("round 1")
    check("a row per drone", len(rows1) == 4, "rows=%s" % sorted(rows1))
    check(
        "flown distance recorded",
        all(row["flown_m"] > 1.0 for row in rows1.values()),
        str({k: round(v["flown_m"], 1) for k, v in rows1.items()}),
    )
    check(
        "survey planned progress recorded",
        all(rows1[label]["planned_m"] > 10.0 for label in rows1 if label.startswith("S")),
        str({k: round(v["planned_m"], 1) for k, v in rows1.items()}),
    )
    check(
        "M1 sigma' present for survey drones",
        all(rows1[label]["m1_sigma"] is not None for label in rows1 if label.startswith("S")),
        str({k: (round(v["m1_sigma"], 2) if v["m1_sigma"] else None) for k, v in rows1.items()}),
    )
    check(
        "M4 Hausdorff present for survey drones",
        all(
            rows1[label]["m4_hausdorff_m"] is not None
            for label in rows1
            if label.startswith("S")
        ),
        str({k: (round(v["m4_hausdorff_m"], 1) if v["m4_hausdorff_m"] else None)
             for k, v in rows1.items()}),
    )
    check(
        "M3 auto pct is ~100 in a full-auto round",
        all(row["m3_auto_pct"] > 90.0 for row in rows1.values()),
        str({k: round(v["m3_auto_pct"]) for k, v in rows1.items()}),
    )
    check(
        "M2 has no manual samples in a full-auto round",
        all(row["m2_samples"] == 0 for row in rows1.values()),
    )

    run_number = live["record_trust_round"]()
    check("round 1 logged as run 1", run_number == 1, "run=%s" % run_number)
    check("second record call does not duplicate", live["record_trust_round"]() == 1)

    data_dir = live["export_round_data_files"]()
    pdf_path = live["export_round_report_pdf"]()
    print("exports -> %s" % data_dir)
    print("pdf     -> %s" % pdf_path)
    check("per-round trust csv written", any(
        name.endswith("_trust.csv") for name in os.listdir(data_dir)
    ))
    check("run log written", os.path.exists(live["get_trust_history_csv_path"]()))
    check("figure log written", os.path.exists(live["get_trust_history_figure_path"]()))
    check("pdf written", os.path.exists(pdf_path) and os.path.getsize(pdf_path) > 4000,
          "%d bytes" % os.path.getsize(pdf_path))

else:
    # --- second round, with a manual stretch so M2/M3 have something to say -----
    print("\n--- round 2 (manual survey lead for part of the round) ---")
    live["reset_trust_metrics_tracking"]()
    check("reset clears the trust records", not live["trust_drone_records"])
    live["set_full_automation"]()
    live["set_control_mode"](live["CONTROL_MODE_MANUAL"])
    live["key_map"]["forward"] = True
    frames = 0
    while frames < 4000 and not live["game_over_triggered"]:
        step_simulation(FakeTask(), STEP_SECONDS)
        frames += 1
        if frames == 60:
            live["key_map"]["forward"] = False
            live["set_control_mode"](live["CONTROL_MODE_AUTOMATION"])
    print("frames %d, sim time %.1f s" % (frames, live["motion_state"].sim_time_seconds))

    rows2 = report_trust("round 2")
    check(
        "manual flight recorded on the survey lead",
        rows2["S1"]["flown_manual_m"] > 0.0,
        "manual %.1f m" % rows2["S1"]["flown_manual_m"],
    )
    check(
        "M2 clearance sampled during manual flight",
        rows2["S1"]["m2_samples"] > 0 and rows2["S1"]["m2_clearance_mean_m"] is not None,
        "samples=%s mean=%s" % (rows2["S1"]["m2_samples"], rows2["S1"]["m2_clearance_mean_m"]),
    )
    check(
        "M3 manual pct above zero",
        rows2["S1"]["m3_manual_pct"] > 0.0,
        "manual %.1f%%" % rows2["S1"]["m3_manual_pct"],
    )

    run_number = live["record_trust_round"]()
    check("round 2 logged as run 2", run_number == 2, "run=%s" % run_number)
    data_dir = live["export_round_data_files"]()

    history = live["load_trust_run_history"]()
    check("history has two runs", len(history) == 2, "runs=%s" % [run["run"] for run in history])

    for metric_key, metric_label, _fmt, _higher in live["TRUST_METRIC_DEFINITIONS"]:
        columns, table_rows = live["build_trust_run_table"](metric_key, history)
        print("\nPER RUN: %s" % metric_label)
        print("   " + " | ".join("%-8s" % column for column in columns))
        for row in table_rows:
            print("   " + " | ".join("%-8s" % cell for cell in row))
        check(
            "%s table has dS/dW/dTotal" % metric_key,
            columns[-3:] == ["dS", "dW", "dTotal"],
        )
        check("%s table has one row per run" % metric_key, len(table_rows) == 2)

    columns, table_rows = live["build_trust_run_table"]("m3_manual_pct", history)
    check(
        "run 2 shows a delta against run 1",
        any(cell not in ("-",) for cell in table_rows[1][-3:]),
        str(table_rows[1]),
    )

    pdf_path = live["export_round_report_pdf"]()
    size = os.path.getsize(pdf_path)
    check("second pdf written with the figure pages", size > 6000, "%d bytes" % size)
    with open(pdf_path, "rb") as pdf_file:
        body = pdf_file.read()
    check("pdf declares more than 4 pages", b"/Count" in body,
          body[body.find(b"/Count"):body.find(b"/Count") + 12].decode("latin-1"))

    # formula sheet must document both families
    formula = "\n".join(live["compose_metric_formula_lines"]())
    for needle in (
        "0. TWO FAMILIES OF METRIC",
        "6. P1 DETECTION SPEED",
        "12. TRUST MATRIX M1-M4",
        "13. M1 PATH LENGTH RATIO",
        "14. M2 OBSTACLE CLEARANCE",
        "15. M3 MODE USAGE",
        "16. M4 HAUSDORFF DISTANCE",
    ):
        check("formula sheet contains %r" % needle, needle in formula)

    summary_path = [
        name for name in os.listdir(data_dir) if name.endswith("_summary.csv")
    ]
    check("summary csv uses p1..p4 column names", bool(summary_path))
    if summary_path:
        header = open(os.path.join(data_dir, sorted(summary_path)[-1])).readline()
        check("p1_detection_pct in summary header", "p1_detection_pct" in header, header[:120])
        check("no m1_detection_pct left", "m1_detection_pct" not in header)


failed = [label for label, ok, _ in checks if not ok]
print("\n%d checks, %d failed" % (len(checks), len(failed)))
for label in failed:
    print("  FAILED: " + label)
sys.exit(1 if failed else 0)
