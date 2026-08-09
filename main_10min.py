"""Launch the extended 10-minute, three-episode mission profile."""

import os

os.environ["REAL_SIM_ROUND_MINUTES"] = "10"

import main  # noqa: F401,E402
