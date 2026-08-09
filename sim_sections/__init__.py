"""Load the ordered implementation sections for the wildfire simulation."""

from pathlib import Path


SECTION_FILES = (
    "01_app_setup_and_configuration.py",
    "02_world_objects_and_visuals.py",
    "03_drone_controls_and_cameras.py",
    "04_mission_rules_and_dispatch.py",
    "04b_trust_matrix.py",
    "04c_trust_metrics.py",
    "04d_collaboration_score.py",
    "05_hud_maps_and_reports.py",
    "05b_participant_summary.py",
    "06_drone_and_fire_simulation.py",
    "07_frame_update.py",
    "08_headless_self_tests.py",
    "09_start_application.py",
)


def load_simulation(namespace):
    """Load every section in dependency order into the caller's namespace.

    The shared namespace preserves the existing Panda3D callbacks and mutable
    runtime state while the source is kept in focused, navigable files.
    """
    section_directory = Path(__file__).resolve().parent
    for filename in SECTION_FILES:
        section_path = section_directory / filename
        source = section_path.read_text(encoding="utf-8")
        exec(compile(source, str(section_path), "exec"), namespace)
