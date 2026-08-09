# Simulation sections

`main.py` loads these files in numeric order into one shared namespace. The
shared namespace preserves the existing Panda3D callbacks and mutable simulation
state while keeping the source navigable.

- `01_app_setup_and_configuration.py` — imports, settings, app setup, assets, terrain, and spawn bounds
- `02_world_objects_and_visuals.py` — fire, vegetation, drone artwork, and scene construction
- `03_drone_controls_and_cameras.py` — controller input, cameras, motion state, and view control
- `04_mission_rules_and_dispatch.py` — damage, dispatch, game setup, and keyboard actions
- `04b_trust_matrix.py` — mission score P1-P4 (wildfire outcome, renamed from M1-M4 on July 29, 2026) and the truck-vs-drone extinguish credit
- `04c_trust_metrics.py` — trust matrix M1-M4: path length ratio sigma', obstacle clearance, mode usage, Hausdorff distance, plus the per-participant run log
- `04d_collaboration_score.py` — the 100-point collaboration score: 40 survey + ground crews, 67 with water drones, plus operator credit and cost
- `05_hud_maps_and_reports.py` — HUD, metrics, reports, results, and fire-map UI
- `05b_participant_summary.py` — the all-stages participant report: every round, per-stage averages, trust matrix run by run, trend graphs
- `06_drone_and_fire_simulation.py` — navigation, drone behavior, collisions, detection, and fire lifecycle
- `07_frame_update.py` — per-frame update and application callback registration
- `08_headless_self_tests.py` — opt-in headless integration tests
- `09_start_application.py` — starts the Panda3D application loop

The numeric prefixes are part of the dependency order. A section may use names
created by any earlier section.
