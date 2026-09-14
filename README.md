# Human Trust in Autonomous Wildfire Response Simulation

This repository contains a Panda3D-based wildfire response simulation for studying
human trust, automation, and collaboration in multi-drone forest fire scenarios.
The project models survey drones, water drones, fire trucks, fire spread,
participant interaction, automation modes, and post-run trust/performance
metrics.

## Project Status

This is research software. It is shared so others can inspect, cite, and discuss
the work, but it is not released as open-source software. See [License and reuse](#license-and-reuse).

## Repository Layout

- `main.py` - simulation entry point
- `sim_sections/` - numbered modules loaded in order by `main.py`
- `models/` - runtime visual assets
- `audio/` - runtime audio assets
- `scripts/` - headless measurement, calibration, and verification scripts
- `notes/` - research and implementation notes
- `random_files/` - earlier experiments and reference code

The section loader keeps the simulation source navigable while preserving the
shared state and callbacks expected by Panda3D.

## Requirements

- Python 3
- Panda3D
- NumPy, for selected asset-baking scripts
- Blender Python, only for Blender-specific baking scripts that import `bpy`

Participant reports and other study outputs should remain local and are ignored
by Git through `reports/`.

## Running the Simulation

From the repository root:

```bash
python3 main.py
```

For faster startup modes:

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
please cite it. A machine-readable citation is provided in `CITATION.cff`, which
GitHub can display through its "Cite this repository" button.

Suggested short citation text:

> Cheryl. (2026). Human Trust in Autonomous Wildfire Response Simulation
> [Computer software].

If a DOI, paper, institution, or release version becomes available, update
`CITATION.cff` before publishing a tagged release.

## License and Reuse

Copyright (c) 2026 Cheryl. All rights reserved.

This repository is source-available for review, citation, and scholarly reference
only. No license is granted to copy, redistribute, modify, publish, sublicense,
sell, or incorporate this code or its assets into another project without written
permission from the copyright holder.

Short quotations from the documentation or code comments are permitted for
scholarly discussion, review, and citation when attributed clearly to this
repository. For any other reuse, contact the author first.

## Notes for Contributors

Because this project is not open source, please do not submit code contributions
unless you have already agreed on contribution and reuse terms with the author.
