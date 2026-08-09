"""Launch the preserved 3-minute wildfire mission profile."""

import os

os.environ["REAL_SIM_ROUND_MINUTES"] = "3"

import main  # noqa: F401,E402
