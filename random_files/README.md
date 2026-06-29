# Real Sim, folder guide

Quick map of what each file and folder is. All `.py` files stay at the top level on purpose: they import each other as siblings and load assets with paths like `models/...`, so moving them into subfolders would break the sim.

## Run these (entry points)

- `main.py` — the full wildfire drone simulation
- `case1.py` — Case 1: full coverage, no fire (clean baseline)
- `case2.py` — Case 2: ignition plus swarm response, with manual takeover

## Shared code (keep at root, the entry points import these)

- `simulation_helpers.py` — core helpers; imported by `main.py` and `swarm_demo_shared.py`
- `swarm_demo_shared.py` — shared swarm logic; imported by `case1.py` and `case2.py`

## Assets (loaded by relative path, keep next to the code)

- `models/` — 3D models: terrain, trees, drone, fire truck, fire render set
- `audio/` — sound assets
- `fire model/` — fire model assets

## Support

- `scripts/` — offline baking tools (fire mesh and sprite baking)
- `notes/` — documentation; start at `notes/NOTES_INDEX.md`
