"""Launch any experiment stage fast-forwarded to a few seconds.

Same mission as main_10min.py: same participant ID screen, same stage screen,
same ignition schedule, same suppression rates, same scoring. Two differences
and only two:

  1. the wall clock is compressed, so a round plays out in seconds
  2. every round starts in full automation, hands off

    python3 main_fast.py        opens on stage 0
    python3 main_fast.py 1      opens on stage 1, 2 drones, water only
    python3 main_fast.py 2      opens on stage 2, 2 drones, full control
    python3 main_fast.py 3      opens on stage 3, 4 drones
    python3 main_fast.py 4      opens on stage 4, 6 drones

The stage argument only preselects the stage; you can still change it on the
setup screen, and every round after that starts in full automation too.

You type your name on the ID screen exactly as in a real round, and the report
and CSVs go to reports/<your name>/ with that person's other files.

Tune it from the shell if you want:
  REAL_SIM_TIME_SCALE=80 python3 main_fast.py 4     faster wall clock
  REAL_SIM_MAX_SUBSTEP=0.15 python3 main_fast.py 4  finer physics, slower run

Full automation from the first frame belongs to this launcher and only this
launcher. main.py, main_3min.py and main_10min.py start in MANUAL, exactly as
a real study round runs.

The round is over in simulated time either way; the wall clock depends on how
many integration substeps your machine can push per second.
"""

import os
import sys

VALID_STAGES = ("0", "1", "2", "3", "4")

stage = "0"
for argument in sys.argv[1:]:
    cleaned = argument.strip().lower().replace("stage", "").strip()
    if cleaned in VALID_STAGES:
        stage = cleaned
        break
    print("main_fast.py: stage must be one of 0 1 2 3 4 (got %r)" % argument)
    raise SystemExit(2)

os.environ["REAL_SIM_ROUND_MINUTES"] = "10"
# 60x: one rendered frame advances about 2 simulated seconds.
os.environ.setdefault("REAL_SIM_TIME_SCALE", "60")
# Coarser integration is what buys the speed. Normal play uses 0.05.
os.environ.setdefault("REAL_SIM_MAX_SUBSTEP", "0.25")
# Never sleep to hold a frame rate in this profile.
os.environ.setdefault("REAL_SIM_MANUAL_FRAME_PACING", "0")
# Preselect the stage. The ID and stage screens still come up, so the round is
# filed under the real participant name like any other round.
os.environ["REAL_SIM_STAGE"] = stage
# THIS LAUNCHER ONLY (Cheryl, July 28): every round starts in full automation.
# Not setdefault, and set nowhere else in the project.
os.environ["REAL_SIM_AUTOSTART_FULL_AUTO"] = "1"

print("[main_fast] stage %s, fast-forward x%s, substep %ss, full automation"
      % (stage, os.environ["REAL_SIM_TIME_SCALE"], os.environ["REAL_SIM_MAX_SUBSTEP"]))

import main  # noqa: F401,E402
