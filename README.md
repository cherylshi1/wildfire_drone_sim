# Human Trust in Autonomous Wildfire Response Simulation

This repository contains a Panda3D wildfire response simulation for studying
human trust, automation, and collaboration in multi-drone forest fire scenarios.
It models survey drones, water drones, fire trucks, fire spread, participant
interaction, automation modes, and post-run trust/performance metrics.

## Quick Start

Download the whole repository. Do not download only `main.py`, because the
simulation also needs `sim_sections/`, `models/`, `audio/`, and the JSON
configuration files.

```bash
git clone https://github.com/cherylshi1/wildfire_drone_sim.git
cd wildfire_drone_sim
python3 -m pip install panda3d
python3 main.py
```

If you do not use Git, click GitHub's green **Code** button, choose
**Download ZIP**, unzip the folder, then run `python3 main.py` from inside the
unzipped folder.

## What To Keep When Sharing Or Running

Required for the main simulation:

- `main.py`
- `sim_sections/`
- `simulation_helpers.py`
- `models/`
- `audio/`
- `calibration_baselines.json`
- `scenario_seeds.json`

Useful but optional:

- `main_fast.py`, `main_3min.py`, `main_10min.py` - convenience launch modes
- `scripts/` - headless measurement, calibration, and verification tools
- `CITATION.cff` - machine-readable citation metadata
- `LICENSE.md` - reuse restrictions

Local-only folders such as old experiments, raw fire-model sources, notes, and
participant reports are intentionally not part of the public runnable release.

## Repository Layout

- `main.py` - simulation entry point
- `sim_sections/` - numbered modules loaded in order by `main.py`
- `simulation_helpers.py` - shared math and canopy helper functions
- `models/` - runtime visual assets used by the release
- `audio/` - runtime audio assets
- `scripts/` - command-line tools for headless runs and calibration

## Running

Standard run:

```bash
python3 main.py
```

Shortcut modes:

```bash
python3 main_fast.py
python3 main_3min.py
python3 main_10min.py
```

Headless measurement examples:

```bash
python3 scripts/headless_run.py measure 1 2 3 4 --runs 3
python3 scripts/headless_run.py profile 4 --frames 1500
```

## Citation

If you quote, discuss, compare against, or otherwise rely on this project in a
paper, presentation, poster, repository, report, or derivative research artifact,
please cite it. GitHub can read `CITATION.cff` and show a **Cite this
repository** button.

Suggested short citation:

> Cheryl. (2026). Human Trust in Autonomous Wildfire Response Simulation
> [Computer software]. https://github.com/cherylshi1/wildfire_drone_sim

Update `CITATION.cff` before a formal release if you add a DOI, institution,
paper title, version number, or full author name.

## License And Reuse

Copyright (c) 2026 Cheryl. All rights reserved.

This repository is source-available for review, citation, and scholarly reference
only. No license is granted to copy, redistribute, modify, publish, sublicense,
sell, or incorporate this code or its assets into another project without written
permission from the copyright holder.

Short quotations from the documentation or code comments are permitted for
scholarly discussion, review, and citation when attributed clearly to this
repository. For any other reuse, contact the author first.
