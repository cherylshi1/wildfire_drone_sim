"""Check that a hands-off full-automation round always scores the same.

    python3 scripts/verify_collaboration_score.py 3        one stage
    python3 scripts/verify_collaboration_score.py 3 4      several stages

A full-auto round must land on exactly the collaboration base. Since Aug 9,
2026 the base is the round's P1-P4 mission performance out of 100 (the fixed
40/67 water tier was dropped), so the base varies by round, but a hands-off run
must still equal it exactly. Anything else means an operator credit or cost
leaked into an automation run.

Note: this drives update_simulation_frame directly. Do NOT call update(), whose
pace_main_loop() waits on real time that never advances under MNonRealTime.
"""

import os
import sys

no_water = "--no-water" in sys.argv

SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

os.environ["REAL_SIM_HEADLESS"] = "1"
os.environ["REAL_SIM_AUTOSTART"] = "1"
if "--no-water" not in sys.argv:
    # Full automation from the first frame. Deliberately NOT set for --no-water,
    # because full automation also arms the water dispatch.
    os.environ["REAL_SIM_AUTOSTART_FULL_AUTO"] = "1"
os.environ.setdefault("REAL_SIM_PARTICIPANT", "collabcheck")
os.environ["REAL_SIM_MANUAL_FRAME_PACING"] = "0"
os.environ["REAL_SIM_HEADLESS_FRAMES"] = "2"

stages = [argument for argument in sys.argv[1:] if argument.isdigit()] or ["3"]
os.environ["REAL_SIM_STAGE"] = stages[0]

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


failures = []
for stage in stages:
    live["restart_simulation"]()
    live["show_pregame_setup_after_round"]()
    live["select_experiment_stage"](int(stage))
    live["start_game_from_pregame"]()
    if no_water:
        # The realistic 40-point round: the operator automates the survey team
        # and never dispatches water, so the water drones sit where they were
        # born and only the survey drones and ground crews do the work.
        for view in live["get_all_drone_views"]():
            if view["role"] == "survey":
                live["set_drone_view_control_mode"](view, live["CONTROL_MODE_AUTOMATION"])
    else:
        live["set_full_automation"]()
    steps = 0
    while steps < 4000 and not live["game_over_triggered"]:
        step_simulation(FakeTask(), 0.25)
        steps += 1
    grade = live["compute_collaboration_score"]()
    performance = live["compute_performance_matrix"](live["motion_state"].sim_time_seconds)
    print(
        "stage %s: score %.1f  base %.0f (%s)  operator %+.1f  "
        "outcome P1-P4 %.0f%%  burned %.0f%%"
        % (
            stage,
            grade["score"],
            grade["base"],
            grade["base_label"],
            grade["operator_delta"],
            performance["performance_score"] * 100.0,
            live["burn_ratio"]() * 100.0,
        )
    )
    if abs(grade["score"] - grade["base"]) > 1e-6:
        failures.append(
            "stage %s scored %.2f, expected the base %.0f"
            % (stage, grade["score"], grade["base"])
        )
    if grade["credit_total"] or grade["penalty_total"]:
        failures.append(
            "stage %s: automation run picked up operator credit %s / cost %s"
            % (stage, grade["credits"], grade["penalties"])
        )

print("\n%d stage(s) checked, %d problem(s)" % (len(stages), len(failures)))
for failure in failures:
    print("  " + failure)
sys.exit(1 if failures else 0)
