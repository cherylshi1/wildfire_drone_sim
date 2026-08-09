import json
import os
import sys
import time
from math import atan2, ceil, copysign, cos, degrees, exp, log1p, radians, sin, tau
from pathlib import Path
import random
from collections import deque
from dataclasses import dataclass, field

RANDOM_FILES_DIR = Path(__file__).resolve().parent / "random_files"
if RANDOM_FILES_DIR.exists():
    sys.path.insert(0, str(RANDOM_FILES_DIR))

from direct.gui.OnscreenText import OnscreenText
from direct.showbase.ShowBase import ShowBase
from panda3d.core import (
    AmbientLight,
    CardMaker,
    ClockObject,
    CollisionHandlerQueue,
    CollisionNode,
    CollisionRay,
    CollisionTraverser,
    DirectionalLight,
    GeomNode,
    GeoMipTerrain,
    InputDevice,
    LineSegs,
    NodePath,
    PandaNode,
    Plane,
    PlaneNode,
    Point2,
    Point3,
    PNMImage,
    SamplerState,
    Texture,
    TextureStage,
    TextNode,
    TransparencyAttrib,
    Vec3,
    loadPrcFileData,
)
from simulation_helpers import (
    approach,
    build_tree_canopy_samples,
    compute_desired_velocity,
    compute_local_canopy_height,
    count_detected_hotspots,
)


def read_nonnegative_int_env(name, default_value):
    try:
        return max(0, int(os.environ.get(name, str(default_value))))
    except (TypeError, ValueError):
        return default_value


def read_positive_int_env(name, default_value):
    return max(1, read_nonnegative_int_env(name, default_value))


def read_int_env(name, default_value):
    try:
        return int(os.environ.get(name, str(default_value)))
    except (TypeError, ValueError):
        return default_value


def read_nonnegative_float_env(name, default_value):
    try:
        return max(0.0, float(os.environ.get(name, str(default_value))))
    except (TypeError, ValueError):
        return default_value


def read_bool_env(name, default_value):
    value = os.environ.get(name)
    if value is None:
        return bool(default_value)
    return value.strip().lower() not in ("0", "false", "no", "off", "")


def read_even_drone_count_env(name, default_value):
    drone_count = max(2, read_positive_int_env(name, default_value))
    if drone_count % 2 != 0:
        drone_count += 1
    return drone_count


# ------------------------------
# World and visual configuration
# ------------------------------
# Finalized project scale convention (May 14, 2026):
# 1 simulation world unit equals 1 meter.
REAL_SIM_MAX_FPS = max(15, read_positive_int_env("REAL_SIM_MAX_FPS", 30))
REAL_SIM_FIRE_VISUAL_HZ = read_positive_int_env("REAL_SIM_FIRE_VISUAL_HZ", 8)
REAL_SIM_MAP_HZ = read_positive_int_env("REAL_SIM_MAP_HZ", 4)
REAL_SIM_ALERT_HZ = read_positive_int_env("REAL_SIM_ALERT_HZ", 10)
REAL_SIM_HUD_HZ = read_positive_int_env("REAL_SIM_HUD_HZ", 2)
REAL_SIM_BURN_ETA_HZ = read_positive_int_env("REAL_SIM_BURN_ETA_HZ", 1)
REAL_SIM_PREGAME_HZ = read_positive_int_env("REAL_SIM_PREGAME_HZ", 1)
REAL_SIM_CLIENT_SLEEP_SECONDS = (
    read_nonnegative_float_env("REAL_SIM_CLIENT_SLEEP_MS", 2.0) / 1000.0
)
REAL_SIM_MANUAL_FRAME_PACING = read_bool_env("REAL_SIM_MANUAL_FRAME_PACING", True)
REAL_SIM_GROUND_CACHE_CELL_METERS = read_nonnegative_float_env(
    "REAL_SIM_GROUND_CACHE_CELL_METERS",
    0.5,
)
REAL_SIM_GROUND_CACHE_LIMIT = read_positive_int_env("REAL_SIM_GROUND_CACHE_LIMIT", 12000)

SIM_METERS_PER_UNIT = 1.0
SIM_UNITS_PER_METER = 1.0 / SIM_METERS_PER_UNIT

WORLD_X_LIMIT = 50
WORLD_Y_LIMIT = 50
GRASS_COUNT = read_nonnegative_int_env("REAL_SIM_GRASS_COUNT", 240)
GRASS_CUTOUT_TEXTURE_MAX_WIDTH = read_positive_int_env(
    "REAL_SIM_GRASS_TEXTURE_MAX_WIDTH",
    1024,
)

# Ecological baseline and gameplay scaling:
# - Baseline density target: 400 trees/hectare
# - Playability factor keeps the environment manageable for control tasks
TREE_DENSITY_PER_HECTARE_BASELINE = 400.0
TREE_PLAYABILITY_FACTOR = 0.5
OBSTACLE_DEMO_TREE_MAX_COUNT = 150

NOMINAL_PLAY_AREA_SQM = (
    (WORLD_X_LIMIT * 2 * SIM_METERS_PER_UNIT)
    * (WORLD_Y_LIMIT * 2 * SIM_METERS_PER_UNIT)
)
NOMINAL_PLAY_AREA_HECTARES = NOMINAL_PLAY_AREA_SQM / 10_000.0
ECOLOGICAL_TREE_COUNT_BASELINE = TREE_DENSITY_PER_HECTARE_BASELINE * NOMINAL_PLAY_AREA_HECTARES
TREE_COUNT = max(1, int(round(ECOLOGICAL_TREE_COUNT_BASELINE * TREE_PLAYABILITY_FACTOR)))

TREE_HEIGHT_MULTIPLIER = 2.0
TREE_RANDOM_SCALE_MIN = 1.35
TREE_RANDOM_SCALE_MAX = 2.35
LEAF_START_HEIGHT_RATIO = 0.6

GRASS_PATCH_CLUMPS_MIN = 4
GRASS_PATCH_CLUMPS_MAX = 9
GRASS_PATCH_RADIUS = 1.9
GRASS_EXCLUSION_RADIUS = 3.0

USE_TERRAIN_MODEL = False
USE_PROCEDURAL_MOUNTAIN_TERRAIN = True
TERRAIN_SCALE = 1
TERRAIN_HEIGHTFIELD_SIZE = 129
TERRAIN_WORLD_X = WORLD_X_LIMIT * 2.8
TERRAIN_WORLD_Y = WORLD_Y_LIMIT * 2.8
TERRAIN_WORLD_Z = 13.5

ENFORCE_MIN_FLYING_ALTITUDE = False
DRONE_MODEL_SCALE = 0.3
DRONE_SPEED_UNITS_PER_SECOND = 13.0
CONTROL_INPUT_LAG_SECONDS = 0.0
DRONE_ACCEL_UNITS_PER_SECOND_SQ = 18.0
DRONE_DECEL_UNITS_PER_SECOND_SQ = 22.0
DRONE_SPEED_PRESET_SLOW = "slow"
DRONE_SPEED_PRESET_NORMAL = "normal"
DRONE_SPEED_PRESET_FAST = "fast"
DRONE_SPEED_PRESET_MULTIPLIERS = {
    DRONE_SPEED_PRESET_SLOW: 0.35,
    DRONE_SPEED_PRESET_NORMAL: 0.48,
    DRONE_SPEED_PRESET_FAST: 0.90,
}
# Tilt limits are per speed preset and symmetric (pitch = roll), tuned so
# the achievable steady-state speed matches each preset's displayed speed
# (terminal speed ~ g * tan(tilt) / horizontal damping):
#   slow 6 deg ~ 4.3 m/s, normal 8 deg ~ 6.0 m/s, fast 15 deg ~ 11.0 m/s.
DRONE_TILT_LIMIT_DEGREES_BY_PRESET = {
    DRONE_SPEED_PRESET_SLOW: 6.0,
    DRONE_SPEED_PRESET_NORMAL: 8.0,
    DRONE_SPEED_PRESET_FAST: 15.0,
}
TX12_CONTROLLER_MANUAL_SPEED_PRESET = DRONE_SPEED_PRESET_FAST
TX12_CONTROLLER_MANUAL_SPEED_UNITS_PER_SECOND = (
    DRONE_SPEED_UNITS_PER_SECOND
    * DRONE_SPEED_PRESET_MULTIPLIERS[TX12_CONTROLLER_MANUAL_SPEED_PRESET]
)
DRONE_ROLL_RESPONSE_DEGREES_PER_SECOND = 70.0
DRONE_PITCH_RESPONSE_DEGREES_PER_SECOND = 82.0
DRONE_BASE_PITCH_DEGREES = 90.0
DRONE_MODEL_HEADING_OFFSET_DEGREES = 180.0
DRONE_HEADING_X_SIGN = -1.0
DRONE_HEADING_Y_SIGN = 1.0
DRONE_HEADING_RESPONSE_DEGREES_PER_SECOND = 90.0
DRONE_HORIZONTAL_ACTIVE_DRAG_PER_SECOND = 0.45
DRONE_HORIZONTAL_PASSIVE_DRAG_PER_SECOND = 3.2
DRONE_VERTICAL_ACTIVE_DRAG_PER_SECOND = 0.35
DRONE_VERTICAL_PASSIVE_DRAG_PER_SECOND = 2.6
DRONE_VERTICAL_STOP_SETTLE_SPEED_METERS_PER_SECOND = 0.18
DRONE_VERTICAL_STOP_SETTLE_ALTITUDE_METERS = 0.12
DRONE_ROLL_FROM_ACCEL_GAIN = 0.28
DRONE_ROLL_REBOUND_FROM_SPEED_GAIN = 0.06
DRONE_ROLL_ACCEL_FILTER_TIME_SECONDS = 0.22
DRONE_ROLL_SWAY_MAX_SPEED = 1.0
DRONE_ROLL_SWAY_FILTER_TIME_SECONDS = 0.26
DRONE_ROLL_ANTICIPATION_DELTA_GAIN = 0.12
DRONE_ROLL_ANTICIPATION_MAX_SPEED = 1.8
DRONE_ROLL_ANTICIPATION_DECAY_TIME_SECONDS = 0.3
DRONE_PITCH_FROM_SPEED_GAIN = 0.14
DRONE_YAW_BANK_MAX_DEGREES = 3.0
DRONE_YAW_BANK_FROM_TURN_RATE_GAIN = 0.018
DRONE_ROLL_SHAKE_AMPLITUDE_DEGREES = 0.06
DRONE_ROLL_SHAKE_FREQUENCY_HZ = 2.3
DRONE_ROLL_SHAKE_MIN_SPEED = 2.0
DRONE_GRAVITY_METERS_PER_SECOND_SQ = 9.81
# Two drone types differ by mass and inertia matrix ONLY (design decision).
# Survey (S): lightweight mapping quadrotor; Ixx = Iyy enforced (symmetric
#   airframe), values in line with identified parameters for small research
#   quadrotors (~1.5-2 kg class).
# Water (W): heavy-lift firefighting quadrotor; mass and inertia taken from
#   the widely cited 4.34 kg quadrotor platform in T. Lee, M. Leok,
#   N. H. McClamroch, "Geometric Tracking Control of a Quadrotor UAV on
#   SE(3)", IEEE CDC 2010 (J = diag[0.0820, 0.0845, 0.1377] kg m^2).
#   No water-shooting reaction forces are modeled.
SURVEY_DRONE_MASS_KG = 1.6
SURVEY_DRONE_INERTIA_XX_KG_M2 = 0.045
SURVEY_DRONE_INERTIA_YY_KG_M2 = 0.045  # Ix = Iy for the surveying drone
SURVEY_DRONE_INERTIA_ZZ_KG_M2 = 0.085
WATER_DRONE_MASS_KG = 4.34
WATER_DRONE_INERTIA_XX_KG_M2 = 0.0820
WATER_DRONE_INERTIA_YY_KG_M2 = 0.0845
WATER_DRONE_INERTIA_ZZ_KG_M2 = 0.1377
# Survey values double as the module-level defaults.
DRONE_MASS_KG = SURVEY_DRONE_MASS_KG
DRONE_INERTIA_XX_KG_M2 = SURVEY_DRONE_INERTIA_XX_KG_M2
DRONE_INERTIA_YY_KG_M2 = SURVEY_DRONE_INERTIA_YY_KG_M2
DRONE_INERTIA_ZZ_KG_M2 = SURVEY_DRONE_INERTIA_ZZ_KG_M2
DRONE_ARM_LENGTH_METERS = 0.28
DRONE_ROTOR_THRUST_COEFFICIENT = 2.2e-5
DRONE_ROTOR_YAW_MOMENT_RATIO = 0.018
DRONE_MOTOR_TIME_CONSTANT_SECONDS = 0.075
DRONE_HOVER_THRUST_PER_ROTOR_NEWTONS = (
    DRONE_MASS_KG * DRONE_GRAVITY_METERS_PER_SECOND_SQ
) / 4.0
DRONE_HOVER_ROTOR_OMEGA_RAD_PER_SECOND = (
    DRONE_HOVER_THRUST_PER_ROTOR_NEWTONS / DRONE_ROTOR_THRUST_COEFFICIENT
) ** 0.5
DRONE_MAX_HORIZONTAL_ACCEL_METERS_PER_SECOND_SQ = 8.4
DRONE_MAX_VERTICAL_ACCEL_METERS_PER_SECOND_SQ = 5.2
DRONE_HOVER_GUIDANCE_GAIN = 4.2
DRONE_HOVER_VERTICAL_RATE_GAIN = 3.4
DRONE_HORIZONTAL_VELOCITY_GAIN = 3.7
DRONE_HORIZONTAL_BASE_DAMPING_PER_SECOND = 0.22
DRONE_HORIZONTAL_BRAKE_DAMPING_PER_SECOND = 1.9
DRONE_HORIZONTAL_STOP_SETTLE_SPEED_METERS_PER_SECOND = 0.24
DRONE_WIND_COMMAND_BLEND_MANUAL = 0.0
DRONE_WIND_COMMAND_BLEND_AUTOMATION = 0.14
DRONE_PITCH_ANGLE_P_GAIN = 9.5
DRONE_PITCH_RATE_D_GAIN = 4.8
DRONE_ROLL_ANGLE_P_GAIN = 9.5
DRONE_ROLL_RATE_D_GAIN = 4.8
DRONE_YAW_ANGLE_P_GAIN = 6.0
DRONE_YAW_RATE_D_GAIN = 3.0
DRONE_YAW_HOLD_DEADBAND_RADIANS = radians(0.9)
DRONE_YAW_RATE_SETTLE_RADIANS_PER_SECOND = radians(2.0)
DRONE_MANUAL_YAW_INPUT_SPEED_METERS_PER_SECOND = 0.35
DRONE_MAX_PITCH_TORQUE_NM = 0.42
DRONE_MAX_ROLL_TORQUE_NM = 0.42
DRONE_MAX_YAW_TORQUE_NM = 0.18
DRONE_MAX_COLLECTIVE_THRUST_NEWTONS = (
    DRONE_MASS_KG * DRONE_GRAVITY_METERS_PER_SECOND_SQ * 2.4
)
DRONE_MIN_COLLECTIVE_THRUST_NEWTONS = (
    DRONE_MASS_KG * DRONE_GRAVITY_METERS_PER_SECOND_SQ * 0.35
)
DRONE_PROPELLER_BLADE_HALF_LENGTH = 1.35
DRONE_PROPELLER_BLADE_HALF_WIDTH = 0.12
DRONE_PROPELLER_BLADE_ALPHA = 0.56
DRONE_PROPELLER_X_OFFSET_FACTOR = 0.34
DRONE_PROPELLER_Z_OFFSET_FACTOR = 0.33
DRONE_PROPELLER_Y_TOP_MARGIN_FACTOR = 0.28
DRONE_PROPELLER_SPIN_BASE_DEGREES_PER_SECOND = 1150.0
DRONE_PROPELLER_SPIN_MAX_DEGREES_PER_SECOND = 1650.0  # cap so the blades don't strobe
DRONE_PROPELLER_SPIN_GAIN_DEGREES_PER_SECOND = 58.0
CAMERA_DRAG_X_DEGREES_PER_UNIT = -60.0
THIRD_PERSON_DRAG_Y_DEGREES_PER_UNIT = -60.0
FIRST_PERSON_DRAG_Y_DEGREES_PER_UNIT = 60.0
CAMERA_DRAG_DELTA_CLAMP = 0.05
AUTOMATION_CAMERA_ALIGN_SPEED_DEGREES_PER_SECOND = 85.0
AUTOMATION_CAMERA_ALIGN_MIN_COMMAND_METERS_PER_SECOND = 0.55
AUTOMATION_CAMERA_ALIGN_MIN_SPEED_METERS_PER_SECOND = 1.65
AUTOMATION_CAMERA_LOOK_CONE_HALF_ANGLE_DEGREES = 55.0

THIRD_PERSON_CAMERA_DISTANCE = 13.5
THIRD_PERSON_CAMERA_HEIGHT = 4.2
THIRD_PERSON_CAMERA_PITCH_DEGREES = 22.0
FIRST_PERSON_CAMERA_PITCH_DEGREES = 0.0
# A steep launch angle makes the nearby ground visible immediately. At -42
# degrees, the first-person ray drops roughly 20 m over its 30 m look distance.
MISSION_START_CAMERA_PITCH_DEGREES = -42.0
THIRD_PERSON_CAMERA_MIN_PITCH_DEGREES = -72.0
THIRD_PERSON_CAMERA_MAX_PITCH_DEGREES = 62.0
FIRST_PERSON_CAMERA_MIN_PITCH_DEGREES = -88.0
FIRST_PERSON_CAMERA_MAX_PITCH_DEGREES = 68.0
THIRD_PERSON_CAMERA_MIN_DISTANCE = 4.5
THIRD_PERSON_CAMERA_OBSTRUCTION_DISTANCE_SCALE = 0.72
THIRD_PERSON_CAMERA_MAX_OBSTRUCTION_PASSES = 3
THIRD_PERSON_CAMERA_CANOPY_QUERY_RADIUS_METERS = 1.8
THIRD_PERSON_CAMERA_CANOPY_BUFFER_METERS = 1.0
THIRD_PERSON_CAMERA_MAX_VERTICAL_LIFT_METERS = 9.0
THIRD_PERSON_CAMERA_PATH_PROBE_FRACTIONS = (0.2, 0.4, 0.6, 0.8, 1.0)
FIRST_PERSON_CAMERA_CANOPY_QUERY_RADIUS_METERS = 1.3
FIRST_PERSON_CAMERA_CANOPY_BUFFER_METERS = 1.2
FIRST_PERSON_CAMERA_FORWARD_OFFSET_METERS = 0.85

CONTROL_MODE_MANUAL = "manual"
CONTROL_MODE_AUTOMATION = "automation"

AUTOMATION_PROFILE_CANOPY_CRUISE = "canopy_cruise"
AUTOMATION_PROFILE_OBSTACLE_AVOIDANCE = "obstacle_avoidance"
AUTOMATION_PROFILE_ALIASES = {
    AUTOMATION_PROFILE_CANOPY_CRUISE: AUTOMATION_PROFILE_CANOPY_CRUISE,
    "canopy": AUTOMATION_PROFILE_CANOPY_CRUISE,
    "default": AUTOMATION_PROFILE_CANOPY_CRUISE,
    AUTOMATION_PROFILE_OBSTACLE_AVOIDANCE: AUTOMATION_PROFILE_OBSTACLE_AVOIDANCE,
    "obstacle": AUTOMATION_PROFILE_OBSTACLE_AVOIDANCE,
    "avoidance": AUTOMATION_PROFILE_OBSTACLE_AVOIDANCE,
}
AUTOMATION_PROFILE = AUTOMATION_PROFILE_ALIASES.get(
    os.environ.get(
        "REAL_SIM_AUTOMATION_PROFILE",
        AUTOMATION_PROFILE_CANOPY_CRUISE,
    ).strip().lower(),
    AUTOMATION_PROFILE_CANOPY_CRUISE,
)
AUTOMATION_PROFILE_LABEL = (
    "OBSTACLE"
    if AUTOMATION_PROFILE == AUTOMATION_PROFILE_OBSTACLE_AVOIDANCE
    else "CANOPY"
)
AUTOMATION_STARTS_IN_AUTOMATION = False
AUTOMATION_START_SPEED_PRESET = DRONE_SPEED_PRESET_NORMAL

TEAM_ALGORITHM_BASE = "base"
TEAM_ALGORITHM_MAP_THEN_RESUME = "map_then_resume"
TEAM_ALGORITHM_ROLLING_PATROL = "rolling_patrol"
TEAM_ALGORITHM_ORDER = (
    TEAM_ALGORITHM_BASE,
    TEAM_ALGORITHM_MAP_THEN_RESUME,
    TEAM_ALGORITHM_ROLLING_PATROL,
)
TEAM_ALGORITHM_LABELS = {
    TEAM_ALGORITHM_BASE: "BASE",
    TEAM_ALGORITHM_MAP_THEN_RESUME: "MAP-RESUME",
    TEAM_ALGORITHM_ROLLING_PATROL: "ROLLING",
}
# Study cases: 2 drones (1S:1W base case), 4 drones, or 6 drones. The setup
# screen lets the operator shift the S:W split while keeping at least one of
# each role.
TEAM_TOTAL_DRONE_OPTIONS = (2, 4, 6)
_team_total_default_request = read_even_drone_count_env("REAL_SIM_TOTAL_DRONES", 4)
TEAM_TOTAL_DRONE_COUNT_DEFAULT = min(
    TEAM_TOTAL_DRONE_OPTIONS,
    key=lambda option: abs(option - _team_total_default_request),
)
TEAM_WATER_DRONE_COUNT_DEFAULT = min(
    max(1, TEAM_TOTAL_DRONE_COUNT_DEFAULT - 1),
    max(
        1,
        read_nonnegative_int_env(
            "REAL_SIM_WATER_DRONES",
            max(1, TEAM_TOTAL_DRONE_COUNT_DEFAULT // 2),
        ),
    ),
)
# Experiment protocol (Cheryl, July 9 2026, see Experiment Documentation
# folder): every participant plays the stages in order. Stage 0 is a practice
# demo; stages 1-4 are the official games. Stage 1 uses the 2-drone team but
# the survey drone is LOCKED to automation - the operator only flies water.
EXPERIMENT_STAGES = {
    0: {"label": "DEMO practice (2 drones)", "total": 2, "water": 1,
        "water_only": False, "demo": True},
    1: {"label": "2 drones - you fly the WATER drone only", "total": 2,
        "water": 1, "water_only": True, "demo": False},
    2: {"label": "2 drones - full control", "total": 2, "water": 1,
        "water_only": False, "demo": False},
    3: {"label": "4 drones", "total": 4, "water": 2,
        "water_only": False, "demo": False},
    4: {"label": "6 drones", "total": 6, "water": 3,
        "water_only": False, "demo": False},
}
PARTICIPANT_ID_MAX_CHARS = 24
# Fairness across team sizes (July 9 decision): a hands-off full-auto round
# must score the same for 2/4/6 drones, achieved by making the fire harder
# for bigger teams instead of normalizing statistically. The multiplier
# divides the spread-check interval (higher = faster fire).
# FIRST-PASS calibration July 9 (100 sim-s windows, seeds 11/12, raw
# composite): team 2 = 0.313, team 4 = 0.311, team 6 = 0.392 -> only the
# 6-drone game was too easy; 1.3 pulled it to ~0.384-0.38. Seed noise is
# +-0.1, so treat 1.3 as PROVISIONAL: rerun scripts/
# calibrate_fairness_baseline.py with many seeds + full rounds before the
# study, and record the final values in the decision log.
FIRE_TEAM_SIZE_SPREAD_RATE_MULTIPLIERS = {2: 1.0, 4: 1.0, 6: 1.3}
FOLLOWER_PATROL_EDGE_MARGIN_METERS = 6.0
# Anti-stall: a patrol leg is considered stuck only if it takes much longer
# than the distance/speed suggests (tree or drone avoidance can deadlock a
# drone near corners). The allowance scales with the remaining distance so
# long sweep legs are never skipped mid-flight.
PATROL_STALL_BASE_ALLOWANCE_SECONDS = 10.0
PATROL_STALL_DISTANCE_TIME_FACTOR = 2.5
FOLLOWER_SURVEY_ORBIT_SLOT_SPACING_DEGREES = 58.0
FOLLOWER_WATER_HOVER_SLOT_SPACING_DEGREES = 58.0
SURVEY_SECTOR_MARGIN_METERS = 5.0
SURVEY_SECTOR_SWEEP_SPACING_METERS = 11.0
SURVEY_SECTOR_START_CLEARANCE_METERS = 6.0
TEAM_ROLE_MIN_EFFECTIVENESS = 0.35
TEAM_ROLE_EXTRA_DRONE_EFFECTIVENESS = 0.16
TEAM_ROLE_MAX_EFFECTIVENESS = 1.45
TEAM_EXTRA_DRONE_SCORE_BONUS = 0.018
TEAM_EXTRA_DRONE_MAX_SCORE_BONUS = 0.12
SURVEY_MAP_BUILD_TIME_SECONDS = 12.0
SURVEY_ORBIT_RADIUS_METERS = 10.5
SURVEY_ORBIT_CAPTURE_BAND_METERS = 3.0
SURVEY_ORBIT_SPEED_METERS_PER_SECOND = 7.5
SURVEY_ORBIT_ALTITUDE_OFFSET_METERS = 7.0
SURVEY_ROLLING_PATROL_ORBIT_SECONDS = 7.0
SURVEY_ROLLING_PATROL_SWEEP_SECONDS = 5.0
WATER_DRONE_CRUISE_SPEED_METERS_PER_SECOND = 12.5
WATER_DRONE_VERTICAL_SPEED_METERS_PER_SECOND = 5.5
WATER_DRONE_SUPPRESSION_RADIUS_METERS = 6.0
# More forgiving reach for MANUAL [J] spray so it reliably hits a fire you fly
# near (the tight auto tolerances made J feel like it did nothing).
WATER_MANUAL_SPRAY_RADIUS_METERS = 11.0
WATER_MANUAL_SPRAY_ALTITUDE_TOLERANCE_METERS = 8.0
WATER_DRONE_SUPPRESSION_ALTITUDE_METERS = 5.8
WATER_DRONE_IDLE_ALTITUDE_METERS = 4.6
WATER_DRONE_HOVER_OFFSET_METERS = 3.2
WATER_DRONE_MIN_ALTITUDE_ABOVE_GROUND_METERS = 8.0
WATER_DRONE_CANOPY_CLEARANCE_METERS = 8.0
WATER_DRONE_HOME_ALTITUDE_METERS = 24.0
WATER_DRONE_HOME_CORNER_MARGIN_METERS = 4.0
WATER_DRONE_STREAM_POINT_COUNT = 9
WATER_DRONE_STREAM_WOBBLE_METERS = 0.38
WATER_DRONE_STREAM_IMPACT_HEIGHT_METERS = 1.1
WATER_DRONE_STREAM_OUTER_THICKNESS = 8.0
WATER_DRONE_STREAM_INNER_THICKNESS = 3.2
WATER_DRONE_STREAM_MIST_CARD_COUNT = 4
WATER_DRONE_STREAM_MIST_SCALE_METERS = 0.92
WATER_DRONE_HOVER_HOLD_RADIUS_METERS = 4.6
WATER_DRONE_AUTO_VISUAL_TILT_SCALE = 0.12
WATER_DRONE_AUTO_VISUAL_TILT_TIME_CONSTANT_SECONDS = 0.18
WATER_DRONE_MODEL_TINT = (0.36, 0.72, 1.0, 1.0)
WATER_DRONE_TANK_CAPACITY_LITERS = 40.0
WATER_DRONE_SPRAY_FLOW_LITERS_PER_SECOND = 0.35
WATER_DRONE_EMPTY_EPSILON_LITERS = 0.05
WATER_REFILL_HOLD_SECONDS = 8.0
WATER_REFILL_RADIUS_METERS = 6.0
WATER_REFILL_CORNER_MARGIN_METERS = 6.0
WATER_TARGET_RETICLE_RADIUS_METERS = 6.4
WATER_TARGET_RETICLE_SEGMENTS = 48
WATER_TARGET_RETICLE_GROUND_OFFSET_METERS = 0.18
WATER_HUD_TARGET_RETICLE_RADIUS = 0.078
WATER_HUD_TARGET_RETICLE_ARC_SEGMENTS = 8
WATER_HUD_TARGET_RETICLE_PULSE = 0.012
WATER_HUD_TARGET_RETICLE_TICK = 0.038
WATER_HUD_TARGET_RETICLE_CLAMP_MARGIN = 0.09
SURVEY_DRONE_MODEL_TINT = (1.0, 0.94, 0.88, 1.0)
FIRE_MAP_POPUP_GRID_SIZE = 12
FIRE_MAP_POPUP_MAX_LISTED_HOTSPOTS = 3
REWARD_DOLLARS = 100
REWARD_MAX_ADDITIONAL_BURNED_CELLS = 0
PERFORMANCE_WEIGHT_P1 = 0.3
PERFORMANCE_WEIGHT_P2 = 0.2
PERFORMANCE_WEIGHT_P3 = 0.2
PERFORMANCE_WEIGHT_P4 = 0.3
PERFORMANCE_REWARD_THRESHOLD = 0.72
PERFORMANCE_FIREFIGHTER_SUPPRESSION_BASE_SCORE = 0.18
PERFORMANCE_CONTAINED_FIRE_CONTROL_CREDIT = 0.65
# --- Collaboration score out of 100 (Cheryl, July 29, 2026) ------------------
# The 100-point score is now built from WHO COLLABORATED, so a hands-off
# full-automation round always lands on the same number instead of drifting
# with team size and luck:
#   survey drones + ground crews working, no water drone suppression  -> 40
#   water drones contributing suppression as well                     -> 67
# Everything above or below that comes from what the OPERATOR does. A round
# where the operator never touches a drone scores exactly the base.
COLLABORATION_BASE_SURVEY_GROUND_POINTS = 40.0
COLLABORATION_BASE_WITH_WATER_POINTS = 67.0
# Water counts as involved once its drones have done this much suppression work
# (work-seconds, the same unit the fire lifecycle accumulates).
COLLABORATION_WATER_INVOLVEMENT_MIN_WORK_SECONDS = 0.5
# Operator credit: fires the operator found while flying a survey drone by
# hand, and suppression the operator did with manual [J] spray.
COLLABORATION_MANUAL_DETECTION_POINTS = 4.0
COLLABORATION_MANUAL_SUPPRESSION_POINTS_PER_FIRE = 6.0
# Operator cost: drones lost (manual flying has no separation assist, and the
# kill switch is an operator action), and fires that burned or went unfound
# while the responsible survey drone was in the operator's hands.
COLLABORATION_DRONE_LOSS_PENALTY_POINTS = 8.0
COLLABORATION_MISSED_FIRE_PENALTY_POINTS = 5.0
# A run counts as a success when the operator at least matched the automation
# baseline it was given.
COLLABORATION_SUCCESS_MARGIN_POINTS = 0.0
CAMERA_TARGET_SURVEY = "survey"
CAMERA_TARGET_WATER = "water"
HUD_LEFT_MARGIN = 0.012
HUD_TOP_MARGIN = 0.055
WATER_TANK_HUD_WIDTH = 0.46
WATER_TANK_HUD_SLOT_WIDTH = 0.28
WATER_TANK_HUD_SLOT_GAP = 0.018
WATER_TANK_HUD_HEIGHT = 0.046
WATER_TANK_HUD_TOP_Z = -0.415
WATER_TANK_HUD_MAX_SLOTS = max(1, max(TEAM_TOTAL_DRONE_OPTIONS) - 1)
HUD_AUTO_MESSAGE_TOP_OFFSET = 0.30
HUD_AUTO_MESSAGE_WIDTH = 1.02
HUD_AUTO_MESSAGE_HEIGHT = 0.34
HUD_AUTO_MESSAGE_TEXT_SCALE = 0.027
HUD_AUTO_MESSAGE_WORDWRAP = 40.0
# Per-drone accent colors so each section is instantly scannable.
HUD_AUTO_MESSAGE_HEADER_COLOR = (0.99, 0.84, 0.40, 1.0)
HUD_AUTO_MESSAGE_STATUS_COLOR = (0.78, 0.88, 0.96, 1.0)
HUD_AUTO_MESSAGE_SURVEY_COLOR = (0.55, 0.92, 0.98, 1.0)
HUD_AUTO_MESSAGE_WATER_COLOR = (0.62, 0.80, 1.0, 1.0)
HUD_AUTO_MESSAGE_DETAIL_COLOR = (0.86, 0.91, 0.96, 1.0)
PERFORMANCE_PANEL_WIDTH = 0.86
PERFORMANCE_PANEL_HEIGHT = 0.64
PERFORMANCE_PANEL_LEFT_MARGIN = 0.03
PERFORMANCE_PANEL_BOTTOM_MARGIN = 0.08
PERFORMANCE_PANEL_GRAPH_LEFT = 0.035
PERFORMANCE_PANEL_GRAPH_BOTTOM = 0.055
PERFORMANCE_PANEL_GRAPH_WIDTH = 0.78
PERFORMANCE_PANEL_GRAPH_HEIGHT = 0.1
PERFORMANCE_HISTORY_MAX_SAMPLES = 90
PERFORMANCE_GRAPH_LOG_STRENGTH = 9.0
PERFORMANCE_METER_FILL_GREEN = (0.20, 0.95, 0.42, 0.95)
PERFORMANCE_GRAPH_LINE_GREEN = (0.28, 1.0, 0.48, 1.0)
FIRE_MAP_PANEL_WIDTH = 0.86
# Panel grew taller (view is now SQUARE so map positions match the world 1:1).
FIRE_MAP_PANEL_HEIGHT = 1.26
FIRE_MAP_PANEL_RIGHT_MARGIN = 0.04
FIRE_MAP_PANEL_TOP_MARGIN = 0.03
# Scaled down to compensate for the taller square panel: on screen the map
# takes the same height as the old 0.96-tall panel did.
FIRE_MAP_COLLAPSED_SCALE = 0.76
FIRE_MAP_EXPANDED_SCALE = 1.02
FIRE_MAP_ICON_BUTTON_SIZE = 0.048
FIRE_MAP_ICON_BUTTON_GAP = 0.010
FIRE_MAP_SIZE_BUTTON_WIDTH = FIRE_MAP_ICON_BUTTON_SIZE
FIRE_MAP_SIZE_BUTTON_HEIGHT = FIRE_MAP_ICON_BUTTON_SIZE
FIRE_MAP_SIZE_BUTTON_RIGHT_MARGIN = 0.035
FIRE_MAP_SIZE_BUTTON_TOP_MARGIN = 0.025
FIRE_MAP_VIEW_LEFT = 0.04
FIRE_MAP_VIEW_BOTTOM = 0.40
# The world spawn area is SQUARE (+-WORLD_X_LIMIT x +-WORLD_Y_LIMIT), so the
# map view must be square too or every mark lands visually off (old 0.78x0.48
# squashed the world 1.63:1 vertically - fires/drones/trees never lined up
# with what the drone/bird views showed).
FIRE_MAP_VIEW_WIDTH = 0.78
FIRE_MAP_VIEW_HEIGHT = 0.78
FIRE_MAP_TREE_DOT_HALF_SIZE = 0.0021
FIRE_MAP_DRONE_MARKER_HALF_SIZE = 0.011
FIRE_MAP_SELECTED_MARKER_SCALE = 1.45
FIRE_MAP_DRONE_RING_BIN = 130
FIRE_MAP_DRONE_MARKER_BIN = 134
FIRE_MAP_DRONE_LABEL_BIN = 138
FIRE_MAP_RING_SEGMENTS = 36
FIRE_MAP_GRID_DIVISIONS = 4
FIRE_MAP_FIRE_PULSE_INNER_RADIUS_METERS = 5.0
FIRE_MAP_FIRE_PULSE_OUTER_RADIUS_METERS = 10.0
FIRE_MAP_WIND_WIDGET_X = 0.085
FIRE_MAP_WIND_WIDGET_Z = FIRE_MAP_PANEL_HEIGHT - 0.092
FIRE_MAP_WIND_ARROW_LENGTH = 0.07
FIRE_MAP_WIND_ARROW_HEAD_HALF_WIDTH = 0.012
FIRE_MAP_WIND_ARROW_HEAD_BACK_OFFSET = 0.022
FIRE_MAP_WIND_LABEL_OFFSET_X = 0.036
FIRE_MAP_WIND_LABEL_OFFSET_Z = 0.01
DISPATCH_ALERT_TEXT = (
    "FIRE DETECTED\n"
    "PRESS [P] W1   [L] W2   [K] W3"
)
DISPATCH_ALERT_FLASH_SECONDS = 5.0
OVERVIEW_CAMERA_DEFAULT_DISTANCE = 150.0
OVERVIEW_CAMERA_MIN_DISTANCE = 56.0
OVERVIEW_CAMERA_MAX_DISTANCE = 400.0
OVERVIEW_CAMERA_DEFAULT_PITCH_DEGREES = 54.0
OVERVIEW_CAMERA_MIN_PITCH_DEGREES = 20.0
OVERVIEW_CAMERA_MAX_PITCH_DEGREES = 86.0
OVERVIEW_CAMERA_DRAG_Y_DEGREES_PER_UNIT = -72.0
OVERVIEW_CAMERA_ZOOM_STEP = 20.0
OVERVIEW_CAMERA_FOCUS_HEIGHT_METERS = 8.0
# Zoom further out + tilt more top-down so the bird views frame the WHOLE map.
OVERVIEW_CAMERA_AUTO_DISTANCE_SCALE = 1.95
OVERVIEW_CAMERA_AUTO_DISTANCE_PADDING_METERS = 58.0
OVERVIEW_CAMERA_MAP_EDGE_PADDING_METERS = 16.0
OVERVIEW_CAMERA_RESET_ANGLE_DEGREES = -35.0
OVERVIEW_CAMERA_RESET_PITCH_DEGREES = 78.0

DRONE_INTERACTION_EDGE_MARGIN_METERS = 4.0
DRONE_INTERACTION_MIN_HORIZONTAL_SEPARATION_METERS = 6.5
DRONE_INTERACTION_MIN_VERTICAL_SEPARATION_METERS = 2.6
DRONE_INTERACTION_WATER_ALTITUDE_BIAS_METERS = 3.6
# Unified radii (June 2026, keep these consistent everywhere):
# - Obstacle-avoidance radius: SAME for all drones (drone-drone avoidance).
# - Crash radius: minimum spacing used for spawn separation.
# - Effective fire-detection radius: FIRE_HOTSPOT_DETECTION_RADIUS_METERS (15 m).
DRONE_OBSTACLE_AVOIDANCE_RADIUS_METERS = 18.0
DRONE_CRASH_RADIUS_METERS = 3.0
DRONE_CRASH_VERTICAL_RADIUS_METERS = 2.0
DRONE_SPAWN_PAIR_SEPARATION_MARGIN_METERS = 2.0
# Proactive avoidance: survey drone starts steering around the water drone at this
# range (well before the hard separation push) and routes around it.
DRONE_INTERACTION_AVOIDANCE_RADIUS_METERS = DRONE_OBSTACLE_AVOIDANCE_RADIUS_METERS
DRONE_INTERACTION_AVOIDANCE_GAIN = 1.25
DRONE_INTERACTION_AVOIDANCE_LOOKAHEAD_SECONDS = 0.9
DRONE_INTERACTION_AVOIDANCE_MAX_SPEED_FACTOR = 0.82
DRONE_INTERACTION_STUCK_DISTANCE_METERS = 8.5
DRONE_INTERACTION_STUCK_PASS_AROUND_FACTOR = 0.72

# The original 3-minute study profile remains available.  The extended
# 10-minute profile runs three separate ignition episodes and allows limited
# regrowth after a fire has been put out.
ROUND_DURATION_SHORT_SECONDS = 3.0 * 60.0
ROUND_DURATION_LONG_SECONDS = 10.0 * 60.0
configured_round_minutes = read_nonnegative_float_env(
    "REAL_SIM_ROUND_MINUTES",
    ROUND_DURATION_SHORT_SECONDS / 60.0,
)
round_duration_seconds = (
    ROUND_DURATION_LONG_SECONDS
    if configured_round_minutes >= 6.0
    else ROUND_DURATION_SHORT_SECONDS
)

# Short profile: one random fire source at 0:15, plus the established second
# source at 1:15 for a balanced 6-drone team.
FIRE_HOTSPOT_INITIAL_COUNT = 1
FIRE_IGNITION_DELAY_SECONDS = 15.0
SECOND_FIRE_IGNITION_DELAY_SECONDS = 75.0
SECOND_FIRE_MIN_TEAM_SIZE = 6
FIRE_SOURCE_COUNT_FOUR_DRONE_CASE = 1
FIRE_SOURCE_COUNT_SIX_DRONE_CASE = 2
# Long profile: three incident episodes distributed across the 10-minute run.
LONG_ROUND_FIRE_IGNITION_SCHEDULE_SECONDS = (15.0, 210.0, 405.0)
LONG_ROUND_FIRE_SOURCE_COUNT = len(LONG_ROUND_FIRE_IGNITION_SCHEDULE_SECONDS)
# Cap on simultaneous non-burned hotspots. The old cap of 48 silently stopped
# fire growth mid-round; raised so the fire keeps expanding (watch FPS, each
# hotspot carries a visual effect).
FIRE_HOTSPOT_ACTIVE_LIMIT = 250
FIRE_HOTSPOT_SPAWN_ATTEMPTS = 350
FIRE_HOTSPOT_TREE_CLEARANCE_METERS = 6.0
FIRE_HOTSPOT_DETECTION_RADIUS_METERS = 15.0
FIRE_HOTSPOT_DETECTION_PROBABILITY_PER_SECOND = 1.8
SMALL_TEAM_DETECTION_PROBABILITY_SCALE = 0.58
SMALL_TEAM_FORCED_DETECTION_RADIUS_SCALE = 0.55
FIRE_HOTSPOT_MIN_SCAN_ALTITUDE_METERS = 4.5
# Research-backed timeline references:
# - CAL FIRE: first burn period is approximately the first 2 hours.
# - USFS/NIFC messaging: roughly 98% of wildfires are contained within 24 hours.
# - DOI report: some fires can burn unchecked for more than 12 hours before air support.
FIRE_SIM_REAL_MINUTES_PER_SECOND = 15.0
FIRE_REFERENCE_INITIAL_ATTACK_WINDOW_MINUTES = 120.0
FIRE_REFERENCE_CONTAINMENT_WINDOW_MINUTES = 24.0 * 60.0
FIRE_REFERENCE_UNCHECKED_GROWTH_WINDOW_MINUTES = 12.0 * 60.0

FIRE_SUPPRESSION_CONTAIN_DELAY_SECONDS = (
    FIRE_REFERENCE_INITIAL_ATTACK_WINDOW_MINUTES / FIRE_SIM_REAL_MINUTES_PER_SECOND
)
FIRE_SUPPRESSION_EXTINGUISH_DELAY_SECONDS = max(
    1.0,
    (
        FIRE_REFERENCE_CONTAINMENT_WINDOW_MINUTES / FIRE_SIM_REAL_MINUTES_PER_SECOND
    ) - FIRE_SUPPRESSION_CONTAIN_DELAY_SECONDS,
)
FIRE_UNDETECTED_BURNOUT_DELAY_SECONDS = (
    FIRE_REFERENCE_UNCHECKED_GROWTH_WINDOW_MINUTES / FIRE_SIM_REAL_MINUTES_PER_SECOND
)
PERFORMANCE_DETECTION_TARGET_SECONDS = (
    FIRE_REFERENCE_INITIAL_ATTACK_WINDOW_MINUTES / FIRE_SIM_REAL_MINUTES_PER_SECOND
)
# Prototype pacing targets:
# - A full unattended run should last closer to ~3 minutes than ~1 minute.
# - Ground firefighters are the baseline suppressors; the water drone is a strong boost.
FIRE_TARGET_GAME_DURATION_SECONDS = ROUND_DURATION_SHORT_SECONDS
# Suppression pacing: this training sim treats each fire marker as a small
# initial-attack hotspot, not a full incident perimeter. Real wildfire
# operations distinguish contained/controlled/out, and many small starts are
# knocked down before they become large. At 1 sim-second = 15 real minutes,
# the values below make an engaged ground unit finish a marker in roughly
# 10 sim seconds, while a water drone assisting direct attack can knock one
# down in about 3 sim seconds. With four ground units on this 1-hectare training
# map, a clean 6-drone full-auto run should plausibly reach about 50% fully-out
# during a 3-minute exercise.
GROUND_FIREFIGHTER_SUPPRESSION_RATE_PER_SECOND = 0.95
WATER_DRONE_ASSIST_SUPPRESSION_RATE_PER_SECOND = 2.15
GROUND_FIREFIGHTER_RESPONSE_DELAY_SECONDS = 8.0
FIRE_SUPPRESSION_CONTAIN_DELAY_SECONDS = 7.5
FIRE_SUPPRESSION_EXTINGUISH_DELAY_SECONDS = 14.5
FIRE_UNDETECTED_BURNOUT_DELAY_SECONDS = FIRE_TARGET_GAME_DURATION_SECONDS
FIRE_REIGNITE_PROBABILITY_PER_SECOND = 0.0  # original 3-minute profile
LONG_ROUND_REIGNITE_PROBABILITY_PER_SECOND = 0.001
LONG_ROUND_MAX_REIGNITIONS_PER_HOTSPOT = 2
LONG_ROUND_GROUND_SUPPRESSION_RATE_SCALE = 0.72
LONG_ROUND_WATER_SUPPRESSION_RATE_SCALE = 0.78
FIRE_SPREAD_ENABLED = True
FIRE_FULL_MAP_TARGET_SECONDS = 160.0
LONG_ROUND_FULL_MAP_TARGET_SECONDS = 540.0
# Six-drone full-auto pacing target: dense enough that the team can miss some
# perimeter growth, but calm enough that a clean 3S/3W run can find roughly 80%
# of hotspots and put out roughly 50% with water support.
SIX_DRONE_FULL_AUTO_TARGET_DETECTION_RATE = 0.80
SIX_DRONE_FULL_AUTO_TARGET_EXTINGUISH_RATE = 0.50
FIRE_SPREAD_CHECK_INTERVAL_SECONDS = 1.0
FIRE_SPREAD_EVENTS_PER_TICK = 3
FIRE_SPREAD_EXTRA_EVENT_PROBABILITY = 0.0
FIRE_SPREAD_MIN_DISTANCE_METERS = 4.0
# Moderate max hop: big enough to cover ground quickly (more fire), small enough
# that the fire stays one contiguous front rather than detached "second source"
# spots.
FIRE_SPREAD_MAX_DISTANCE_METERS = 9.0
FIRE_SPREAD_MIN_HOTSPOT_SEPARATION_METERS = 5.8
# Chance a spot already being suppressed still spreads (0 = full containment-as-
# a-race, 1 = original unstoppable spread). Keeps containment meaningful but
# lets the fire grow into a real fight. PLAYTEST.
FIRE_WORKED_SPOT_SPREAD_PROBABILITY = 0.35
FIRE_SPREAD_TREE_CLEARANCE_METERS = 3.5
FIRE_SPREAD_IGNORE_TREE_CLEARANCE = True
FIRE_SPREAD_SPAWN_ATTEMPTS = 34
FIRE_SPREAD_FALLBACK_RANDOM_ATTEMPTS = 20
FIRE_BURN_CELL_SIZE_METERS = 10.0

WIND_ENABLED = True
WIND_BASE_DIRECTION_DEGREES = 35.0
WIND_BASE_SPEED_METERS_PER_SECOND = 4.0
WIND_GUST_DIRECTION_RANGE_DEGREES = 24.0
WIND_GUST_SPEED_RANGE_METERS_PER_SECOND = 1.8
WIND_GUST_MIN_DURATION_SECONDS = 2.5
WIND_GUST_MAX_DURATION_SECONDS = 9.0
WIND_GUST_DIRECTION_RESPONSE_DEGREES_PER_SECOND = 18.0
WIND_GUST_SPEED_RESPONSE_PER_SECOND = 1.5
WIND_DRONE_INFLUENCE_FACTOR = 0.28
# Sim simplification (June 2026): fire expansion is UNIFORM (isotropic).
# Wind still affects drone flight, but no longer biases fire spread.
WIND_FIRE_DIRECTIONAL_BIAS_PER_METER_PER_SECOND = 0.0
WIND_FIRE_BIAS_CONE_DEGREES = 55.0
WIND_FIRE_DOWNWIND_DISTANCE_GAIN_METERS_PER_METER_PER_SECOND = 0.0
WIND_FIRE_UPWIND_DISTANCE_PENALTY_METERS_PER_METER_PER_SECOND = 0.0

# If False (study setting), automation has NO prior knowledge of fire
# locations: survey drones strictly fly their S-shaped search pattern and
# only find fires when the sweep brings them inside the detection radius.
# If True, automation flies toward undetected hotspots (omniscient cheat).
AUTOMATION_KNOWS_UNDETECTED_FIRES = False
AUTOMATION_SPEED_FACTOR = 1.0
AUTOMATION_CRUISE_ALTITUDE_METERS = 22.0
AUTOMATION_ALTITUDE_GAIN = 0.9
AUTOMATION_VERTICAL_SPEED_FACTOR = 0.65
AUTOMATION_TARGET_REACH_RADIUS_METERS = 1.6
AUTOMATION_HOTSPOT_COMMIT_RADIUS_METERS = 0.45
AUTOMATION_HOTSPOT_MIN_APPROACH_SPEED_METERS_PER_SECOND = 3.6
AUTOMATION_SCAN_ALTITUDE_METERS = FIRE_HOTSPOT_MIN_SCAN_ALTITUDE_METERS
AUTOMATION_FORCED_DETECTION_RADIUS_METERS = 1.75
AUTOMATION_HOTSPOT_FLYBY_RADIUS_METERS = 2.4
AUTOMATION_HOTSPOT_REVISIT_DELAY_SECONDS = 6.0
AUTOMATION_MIN_ALTITUDE_ABOVE_GROUND_METERS = 8.0
AUTOMATION_TREE_LOOKAHEAD_SECONDS = 0.8
AUTOMATION_TREE_AVOID_RADIUS_METERS = 12.0
AUTOMATION_TREE_AVOID_GAIN = 0.55
AUTOMATION_CANOPY_QUERY_RADIUS_METERS = 5.0
AUTOMATION_CANOPY_LOOKAHEAD_SECONDS = 2.5
AUTOMATION_CANOPY_CLEARANCE_METERS = 6.0
AUTOMATION_CANOPY_PATH_PROBE_FRACTIONS = (0.0, 0.5, 1.0)
AUTOMATION_ENABLE_CANOPY_CLEARANCE = True
AUTOMATION_MIN_FORWARD_PROGRESS_FACTOR = 0.0
AUTOMATION_SEARCH_REACH_RADIUS_METERS = 5.0
AUTOMATION_SEARCH_SPAWN_RING_MIN_FACTOR = 0.85
AUTOMATION_SEARCH_SPAWN_RING_MAX_FACTOR = 1.35
AUTOMATION_SEARCH_WAYPOINT_ATTEMPTS = 28

if AUTOMATION_PROFILE == AUTOMATION_PROFILE_OBSTACLE_AVOIDANCE:
    # Slower and lower obstacle-demo profile for more deliberate under-canopy motion.
    TREE_COUNT = min(TREE_COUNT, OBSTACLE_DEMO_TREE_MAX_COUNT)
    AUTOMATION_STARTS_IN_AUTOMATION = True
    AUTOMATION_START_SPEED_PRESET = DRONE_SPEED_PRESET_SLOW
    AUTOMATION_SPEED_FACTOR = 0.84
    AUTOMATION_CRUISE_ALTITUDE_METERS = 7.2
    AUTOMATION_ALTITUDE_GAIN = 1.15
    AUTOMATION_VERTICAL_SPEED_FACTOR = 0.42
    AUTOMATION_HOTSPOT_MIN_APPROACH_SPEED_METERS_PER_SECOND = 2.4
    AUTOMATION_SCAN_ALTITUDE_METERS = 3.2
    AUTOMATION_MIN_ALTITUDE_ABOVE_GROUND_METERS = 4.5
    AUTOMATION_TREE_LOOKAHEAD_SECONDS = 1.0
    AUTOMATION_TREE_AVOID_RADIUS_METERS = 10.0
    AUTOMATION_TREE_AVOID_GAIN = 0.62
    AUTOMATION_ENABLE_CANOPY_CLEARANCE = False
    AUTOMATION_MIN_FORWARD_PROGRESS_FACTOR = 0.24

DEFAULT_BACKGROUND_COLOR = (0.45, 0.67, 0.84, 1)
SUN_AMBIENT_LIGHT_COLOR = (0.72, 0.74, 0.78, 1)
SUN_DIRECTIONAL_LIGHT_COLOR = (0.78, 0.76, 0.7, 1)
SUN_HEADING_DEGREES = 128.0
SUN_PITCH_DEGREES = -38.0
THERMAL_BACKGROUND_COLOR = (0.04, 0.05, 0.08, 1)
THERMAL_HIDE_GRASS = True
THERMAL_HOTSPOT_GLOW_SCALE = 2.6
THERMAL_HOTSPOT_BEACON_SCALE = 5.8
THERMAL_HOTSPOT_BIN_ORDER = 60
BAKED_FIRE_FRAME_RATE = 12.0
BAKED_FIRE_CARD_HALF_WIDTH_METERS = 4.2
BAKED_FIRE_CARD_HEIGHT_METERS = 10.2
BAKED_FIRE_CARD_LIFT_METERS = 0.24
BAKED_FIRE_GLOW_HALF_WIDTH_METERS = 2.05
BAKED_FIRE_GLOW_HALF_DEPTH_METERS = 1.08
BAKED_FIRE_ACTIVE_SCALE = 1.08
BAKED_FIRE_DETECTED_SCALE = 1.36
BAKED_FIRE_CONTAINED_SCALE = 0.86
BAKED_FIRE_GROWTH_MIN_SCALE = 0.82
BAKED_FIRE_GROWTH_MAX_SCALE = 1.38
BAKED_FIRE_MESH_TARGET_HEIGHT_METERS = 8.6
BAKED_FIRE_MESH_GROUND_OFFSET_METERS = 0.06
BAKED_FIRE_MESH_HORIZONTAL_SCALE = 0.28
BAKED_FIRE_MESH_ACTIVE_ALPHA = 0.14
BAKED_FIRE_MESH_DETECTED_ALPHA = 0.24
BAKED_FIRE_MESH_CONTAINED_ALPHA = 0.08
# Flame colour by state, so detection flips the fire from deep RED to a
# deliberately artificial cyan/blue-white rescue cue. A natural red-to-amber
# change was too subtle on the baked fire cards.
UNDETECTED_FIRE_GREEN_SCALE = 0.02
UNDETECTED_FIRE_BLUE_SCALE = 0.0
DETECTED_FIRE_GREEN_SCALE = 1.0
DETECTED_FIRE_BLUE_SCALE = 1.0
DETECTED_FIRE_OVERLAY_GLOW_SCALE = 3.55
DETECTED_FIRE_OVERLAY_BEACON_A_SCALE = 2.75
DETECTED_FIRE_OVERLAY_BEACON_B_SCALE = 1.9
FIRE_EFFECT_UPDATE_INTERVAL_SECONDS = 1.0 / REAL_SIM_FIRE_VISUAL_HZ
FIRE_MAP_UPDATE_INTERVAL_SECONDS = 1.0 / REAL_SIM_MAP_HZ
DISPATCH_ALERT_UPDATE_INTERVAL_SECONDS = 1.0 / REAL_SIM_ALERT_HZ
STATUS_UPDATE_INTERVAL_SECONDS = 1.0 / REAL_SIM_HUD_HZ
BURN_ETA_UPDATE_INTERVAL_SECONDS = 1.0 / REAL_SIM_BURN_ETA_HZ
PREGAME_UPDATE_INTERVAL_SECONDS = 1.0 / REAL_SIM_PREGAME_HZ
REAL_SIM_HEADLESS_FRAME_LIMIT = read_nonnegative_int_env("REAL_SIM_HEADLESS_FRAMES", 0)
REAL_SIM_HEADLESS_SUMMARY = read_bool_env("REAL_SIM_HEADLESS_SUMMARY", False)
REAL_SIM_HEADLESS_FIXED_DT_SECONDS = read_nonnegative_float_env(
    "REAL_SIM_HEADLESS_FIXED_DT",
    0.0,
)
REAL_SIM_AUTOSTART = read_bool_env("REAL_SIM_AUTOSTART", False)
REAL_SIM_AUTOSTART_FULL_AUTO = read_bool_env("REAL_SIM_AUTOSTART_FULL_AUTO", False)
# Which experiment stage an autostarted run should play (0-4). -1 keeps
# whatever stage is already selected. Any launcher can use it:
#   REAL_SIM_AUTOSTART=1 REAL_SIM_STAGE=3 python3 main_10min.py
REAL_SIM_AUTOSTART_STAGE = read_int_env("REAL_SIM_STAGE", -1)
# Participant folder for autostarted runs, so test exports do not pile up in
# the same folder as a real participant.
REAL_SIM_AUTOSTART_PARTICIPANT = os.environ.get("REAL_SIM_PARTICIPANT", "").strip()

# --- Fast-forward profile (July 28) ------------------------------------------
# main_fast.py runs the ordinary 10-minute mission with the clock multiplied so
# a full-automation round finishes in seconds. Nothing about the mission
# changes: the ignition schedule, suppression rates and scores are still
# expressed in simulated seconds, only the wall clock is compressed.
#
#   sim seconds advanced per rendered frame = frame_dt * SIMULATION_TIME_SCALE
#   substeps this frame = ceil(that / SIMULATION_MAX_SUBSTEP_SECONDS)
#
# The substep cap is what keeps the flight physics stable, and it is also the
# speed limit: the whole round costs (round_seconds / substep) integration
# steps no matter how large the scale is. 0.05 s is the normal-play value;
# the fast profile trades integration detail for wall-clock time.
SIMULATION_TIME_SCALE = max(
    1.0,
    read_nonnegative_float_env("REAL_SIM_TIME_SCALE", 1.0),
)
SIMULATION_MAX_SUBSTEP_SECONDS = min(
    0.5,
    max(0.01, read_nonnegative_float_env("REAL_SIM_MAX_SUBSTEP", 0.05)),
)
# Per-frame clamp on real elapsed time, so a stall does not teleport anything.
SIMULATION_MAX_FRAME_DT_SECONDS = 0.05


def round_is_long_profile():
    return round_duration_seconds >= ROUND_DURATION_LONG_SECONDS - 0.5


def current_ground_suppression_rate_per_second():
    scale = LONG_ROUND_GROUND_SUPPRESSION_RATE_SCALE if round_is_long_profile() else 1.0
    return GROUND_FIREFIGHTER_SUPPRESSION_RATE_PER_SECOND * scale


def current_water_suppression_rate_per_second():
    scale = LONG_ROUND_WATER_SUPPRESSION_RATE_SCALE if round_is_long_profile() else 1.0
    return WATER_DRONE_ASSIST_SUPPRESSION_RATE_PER_SECOND * scale


def current_reignite_probability_per_second():
    if round_is_long_profile():
        return LONG_ROUND_REIGNITE_PROBABILITY_PER_SECOND
    return FIRE_REIGNITE_PROBABILITY_PER_SECOND


def current_undetected_burnout_delay_seconds():
    if round_is_long_profile():
        return ROUND_DURATION_LONG_SECONDS
    return FIRE_UNDETECTED_BURNOUT_DELAY_SECONDS


def current_full_map_target_seconds():
    if round_is_long_profile():
        return LONG_ROUND_FULL_MAP_TARGET_SECONDS
    return FIRE_FULL_MAP_TARGET_SECONDS


FIRE_STATE_ACTIVE = "active"
FIRE_STATE_CONTAINED = "contained"
FIRE_STATE_OUT = "out"
FIRE_STATE_BURNED = "burned"

thermal_view_enabled = False

SCRIPT_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_ROOT.parent if SCRIPT_ROOT.name == "random_files" else SCRIPT_ROOT
FOREST_SIM_ROOT = PROJECT_ROOT.parent / "forest_sim_demo"
DETECTION_ALARM_SOUND_PATH = PROJECT_ROOT / "audio" / "swarm_alarm.wav"
DETECTION_ALARM_COOLDOWN_SECONDS = 4.0


# Keep startup logs useful without noisy model warnings.
if os.environ.get("REAL_SIM_HEADLESS", "0") == "1":
    loadPrcFileData("", "window-type none")
loadPrcFileData("", "notify-level-assimp error")
loadPrcFileData("", "clock-mode limited")
loadPrcFileData("", f"clock-frame-rate {REAL_SIM_MAX_FPS}")
loadPrcFileData("", "sync-video true")
if REAL_SIM_CLIENT_SLEEP_SECONDS > 0.0:
    loadPrcFileData("", f"client-sleep {REAL_SIM_CLIENT_SLEEP_SECONDS:.4f}")
loadPrcFileData("", f'model-cache-dir "{PROJECT_ROOT / ".panda3d-cache"}"')


def asset_path(relative_path):
    local_path = PROJECT_ROOT / relative_path
    if local_path.exists():
        return str(local_path)
    return str(FOREST_SIM_ROOT / relative_path)


# ---------
# App setup
# ---------
app = ShowBase()  #create 3D program window and sets up the engine
app.disableMouse()  #panda has a defualt ca,era controlled, this turns it off
app.setBackgroundColor(*DEFAULT_BACKGROUND_COLOR)  #set color
alarm_sound = None
last_detection_alarm_time_seconds = -999.0


def load_detection_alarm():
    global alarm_sound
    if not DETECTION_ALARM_SOUND_PATH.exists():
        alarm_sound = None
        return
    try:
        alarm_sound = app.loader.loadSfx(str(DETECTION_ALARM_SOUND_PATH))
        if alarm_sound is not None:
            alarm_sound.setVolume(0.9)
    except Exception:
        alarm_sound = None


load_detection_alarm()


def setup_lights():
    ambient_light = AmbientLight("ambient_light")
    ambient_light.setColor(SUN_AMBIENT_LIGHT_COLOR)
    ambient_np = app.render.attachNewNode(ambient_light)
    app.render.setLight(ambient_np)

    sun_light = DirectionalLight("sun_light")
    sun_light.setColor(SUN_DIRECTIONAL_LIGHT_COLOR)
    sun_np = app.render.attachNewNode(sun_light)
    sun_np.setHpr(SUN_HEADING_DEGREES, SUN_PITCH_DEGREES, 0)
    app.render.setLight(sun_np)


setup_lights()


# ----------------
# Utility helpers
# ----------------
grass_texture_cache = {}
procedural_texture_cache = {}
baked_fire_textures = None
baked_fire_metadata = None
baked_fire_mesh_prototypes = None
baked_fire_mesh_metadata = None
GRASS_CUTOUT_CACHE_VERSION = 3


def configure_clamped_texture(texture):
    if texture is None:
        return None
    texture.setWrapU(SamplerState.WM_clamp)
    texture.setWrapV(SamplerState.WM_clamp)
    return texture


def grass_cutout_cache_path(texture_path):
    source_path = Path(texture_path)
    try:
        source_stat = source_path.stat()
    except OSError:
        return None
    safe_stem = "".join(
        char if char.isalnum() or char in ("-", "_") else "_"
        for char in source_path.stem
    )
    cache_name = (
        f"{safe_stem}_{source_stat.st_size}_{source_stat.st_mtime_ns}"
        f"_v{GRASS_CUTOUT_CACHE_VERSION}.png"
    )
    return PROJECT_ROOT / ".panda3d-cache" / "grass_cutouts" / cache_name


def build_cached_grass_cutout_with_numpy(texture_path, cache_path):
    try:
        import numpy as np
        from PIL import Image
    except ImportError:
        return False

    try:
        source_image = Image.open(texture_path).convert("RGBA")
        if (
            GRASS_CUTOUT_TEXTURE_MAX_WIDTH > 0
            and source_image.width > GRASS_CUTOUT_TEXTURE_MAX_WIDTH
        ):
            resized_height = max(
                1,
                round(
                    source_image.height
                    * (GRASS_CUTOUT_TEXTURE_MAX_WIDTH / source_image.width)
                ),
            )
            source_image = source_image.resize(
                (GRASS_CUTOUT_TEXTURE_MAX_WIDTH, resized_height),
                Image.Resampling.LANCZOS,
            )
        rgba = np.asarray(source_image, dtype=np.float32) / 255.0
    except Exception:
        return False

    red = rgba[..., 0]
    green = rgba[..., 1]
    blue = rgba[..., 2]
    luma = (red + green + blue) / 3.0
    green_boost = np.maximum(0.0, green - np.maximum(red, blue))
    rgba[..., 3] = np.clip((luma - 0.12) * 2.2 + green_boost * 2.5, 0.0, 1.0)
    output = np.clip((rgba * 255.0) + 0.5, 0.0, 255.0).astype(np.uint8)

    try:
        cache_path.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(output).save(cache_path)
    except Exception:
        return False
    return True


def build_radial_sprite_texture(texture_name, size=96):
    cached_texture = procedural_texture_cache.get(texture_name)
    if cached_texture is not None:
        return cached_texture

    image = PNMImage(size, size, 4)
    center = (size - 1) * 0.5
    radius = max(1.0, center)

    for y in range(size):
        for x in range(size):
            dx = (x - center) / radius
            dy = (y - center) / radius
            distance = (dx * dx + dy * dy) ** 0.5
            alpha = max(0.0, 1.0 - distance)
            alpha = alpha * alpha
            image.setXelA(x, y, 1.0, 1.0, 1.0, alpha)

    texture = Texture(texture_name)
    texture.load(image)
    texture.setWrapU(SamplerState.WM_clamp)
    texture.setWrapV(SamplerState.WM_clamp)
    procedural_texture_cache[texture_name] = texture
    return texture


def build_flame_sprite_texture(texture_name, size=96):
    cached_texture = procedural_texture_cache.get(texture_name)
    if cached_texture is not None:
        return cached_texture

    image = PNMImage(size, size, 4)

    for y in range(size):
        v = y / max(1, size - 1)
        base_flare = 0.24 + ((1.0 - v) ** 0.9) * 0.82
        shoulder = max(0.0, 1.0 - abs(v - 0.24) / 0.2) * 0.18
        max_width = base_flare + shoulder
        vertical_core = max(0.0, 1.0 - abs(v - 0.34) / 0.62)
        tip_fade = 1.0 if v <= 0.82 else max(0.0, 1.0 - ((v - 0.82) / 0.18))

        for x in range(size):
            u = ((x / max(1, size - 1)) * 2.0) - 1.0
            width_noise = sin((u * 2.6 + v * 5.4) * tau) * 0.05 * (1.0 - v)
            horizontal = abs(u) / max(0.01, max_width + width_noise)
            if horizontal >= 1.0:
                alpha = 0.0
            else:
                body = (1.0 - horizontal) ** 1.55
                alpha = body * (vertical_core ** 0.75) * tip_fade
                alpha = min(1.0, alpha * 1.45)
            image.setXelA(x, y, 1.0, 1.0, 1.0, alpha)

    texture = Texture(texture_name)
    texture.load(image)
    texture.setWrapU(SamplerState.WM_clamp)
    texture.setWrapV(SamplerState.WM_clamp)
    procedural_texture_cache[texture_name] = texture
    return texture


HOTSPOT_GLOW_TEXTURE = build_radial_sprite_texture("hotspot_glow")
HOTSPOT_FLAME_TEXTURE = build_flame_sprite_texture("hotspot_flame")


def load_baked_fire_textures():
    global baked_fire_textures
    if baked_fire_textures is not None:
        return baked_fire_textures

    baked_frames_dir = PROJECT_ROOT / "models" / "fire_baked_frames"
    if not baked_frames_dir.exists():
        baked_fire_textures = tuple()
        return baked_fire_textures

    frame_paths = sorted(
        frame_path
        for frame_path in baked_frames_dir.iterdir()
        if frame_path.suffix.lower() == ".png"
    )
    if not frame_paths:
        baked_fire_textures = tuple()
        return baked_fire_textures

    loaded_textures = []
    for frame_path in frame_paths:
        texture = app.loader.loadTexture(str(frame_path))
        if texture is None:
            continue
        texture.setWrapU(SamplerState.WM_clamp)
        texture.setWrapV(SamplerState.WM_clamp)
        loaded_textures.append(texture)

    baked_fire_textures = tuple(loaded_textures)
    return baked_fire_textures


def load_baked_fire_metadata():
    global baked_fire_metadata
    if baked_fire_metadata is not None:
        return baked_fire_metadata

    metadata_path = PROJECT_ROOT / "models" / "fire_baked_frames" / "metadata.json"
    if not metadata_path.exists():
        baked_fire_metadata = {}
        return baked_fire_metadata

    try:
        baked_fire_metadata = json.loads(metadata_path.read_text())
    except (OSError, ValueError, TypeError):
        baked_fire_metadata = {}
    return baked_fire_metadata


def baked_fire_frame_rate():
    metadata = load_baked_fire_metadata()
    metadata_frame_rate = metadata.get("frame_rate")
    if isinstance(metadata_frame_rate, (int, float)) and metadata_frame_rate > 0.0:
        return float(metadata_frame_rate)
    return BAKED_FIRE_FRAME_RATE


def load_baked_fire_mesh_prototypes():
    global baked_fire_mesh_prototypes
    if baked_fire_mesh_prototypes is not None:
        return baked_fire_mesh_prototypes

    baked_mesh_dir = PROJECT_ROOT / "models" / "fire_baked_mesh_frames"
    if not baked_mesh_dir.exists():
        baked_fire_mesh_prototypes = tuple()
        return baked_fire_mesh_prototypes

    frame_paths = sorted(
        frame_path
        for frame_path in baked_mesh_dir.iterdir()
        if frame_path.suffix.lower() == ".obj"
    )
    if not frame_paths:
        baked_fire_mesh_prototypes = tuple()
        return baked_fire_mesh_prototypes

    loaded_models = []
    for frame_path in frame_paths:
        try:
            model = app.loader.loadModel(str(frame_path))
        except Exception:
            continue
        if model is None or model.isEmpty():
            continue
        loaded_models.append(model)

    baked_fire_mesh_prototypes = tuple(loaded_models)
    return baked_fire_mesh_prototypes


def load_baked_fire_mesh_metadata():
    global baked_fire_mesh_metadata
    if baked_fire_mesh_metadata is not None:
        return baked_fire_mesh_metadata

    metadata_path = (
        PROJECT_ROOT / "models" / "fire_baked_mesh_frames" / "metadata.json"
    )
    if not metadata_path.exists():
        baked_fire_mesh_metadata = {}
        return baked_fire_mesh_metadata

    try:
        baked_fire_mesh_metadata = json.loads(metadata_path.read_text())
    except (OSError, ValueError, TypeError):
        baked_fire_mesh_metadata = {}
    return baked_fire_mesh_metadata


def baked_fire_mesh_frame_rate():
    metadata = load_baked_fire_mesh_metadata()
    metadata_frame_rate = metadata.get("frame_rate")
    if isinstance(metadata_frame_rate, (int, float)) and metadata_frame_rate > 0.0:
        return float(metadata_frame_rate)
    return 3.0


def load_cutout_grass_texture(texture_path):
    cached_texture = grass_texture_cache.get(texture_path)
    if cached_texture is not None:
        return cached_texture

    cache_path = grass_cutout_cache_path(texture_path)
    if cache_path is not None:
        if cache_path.exists() or build_cached_grass_cutout_with_numpy(
            texture_path,
            cache_path,
        ):
            texture = app.loader.loadTexture(str(cache_path))
            if texture is not None:
                texture = configure_clamped_texture(texture)
                grass_texture_cache[texture_path] = texture
                return texture

    image = PNMImage()
    if not image.read(texture_path):
        texture = app.loader.loadTexture(texture_path)
        texture = configure_clamped_texture(texture)
        grass_texture_cache[texture_path] = texture
        return texture

    if not image.hasAlpha():
        image.addAlpha()

    x_size = image.getXSize()
    y_size = image.getYSize()

    for y in range(y_size):
        for x in range(x_size):
            r = image.getRed(x, y)
            g = image.getGreen(x, y)
            b = image.getBlue(x, y)

            # Remove dark photo backgrounds while preserving green blades.
            luma = (r + g + b) / 3.0
            green_boost = max(0.0, g - max(r, b))
            alpha = (luma - 0.12) * 2.2 + green_boost * 2.5
            alpha = max(0.0, min(1.0, alpha))
            image.setAlpha(x, y, alpha)

    if cache_path is not None:
        try:
            cache_path.parent.mkdir(parents=True, exist_ok=True)
            image.write(str(cache_path))
        except Exception:
            pass

    fallback_texture_path = (
        str(cache_path)
        if cache_path is not None and cache_path.exists()
        else texture_path
    )
    texture = app.loader.loadTexture(fallback_texture_path)
    if texture is None:
        texture = Texture(Path(texture_path).stem)
    texture.load(image)
    texture = configure_clamped_texture(texture)
    grass_texture_cache[texture_path] = texture
    return texture


def apply_repeating_texture(node_path, texture_path, u_scale, v_scale):
    texture = app.loader.loadTexture(asset_path(texture_path))
    texture.setWrapU(SamplerState.WM_repeat)
    texture.setWrapV(SamplerState.WM_repeat)
    node_path.setTexture(texture, 1)
    node_path.setTexScale(TextureStage.getDefault(), u_scale, v_scale)


def snap_bottom_to_ground(node_path):
    bounds = node_path.getTightBounds()
    if bounds is None:
        return

    lowest_point = bounds[0]
    lowest_z = lowest_point.z
    current_z = node_path.getZ()
    node_path.setZ(current_z - lowest_z)


def align_to_slope(node_path, normal):
    if normal.lengthSquared() <= 0.00001:
        return

    random_forward = Vec3(random.uniform(-1, 1), random.uniform(-1, 1), 0)
    if random_forward.lengthSquared() < 0.001:
        random_forward = Vec3(1, 0, 0)

    random_forward.normalize()
    pos = node_path.getPos()
    node_path.headsUp(pos + random_forward, normal)


def transform_bounds(bounds, transform_matrix):
    if bounds is None:
        return None

    minimum_point, maximum_point = bounds
    corners = (
        Point3(minimum_point.x, minimum_point.y, minimum_point.z),
        Point3(minimum_point.x, minimum_point.y, maximum_point.z),
        Point3(minimum_point.x, maximum_point.y, minimum_point.z),
        Point3(minimum_point.x, maximum_point.y, maximum_point.z),
        Point3(maximum_point.x, minimum_point.y, minimum_point.z),
        Point3(maximum_point.x, minimum_point.y, maximum_point.z),
        Point3(maximum_point.x, maximum_point.y, minimum_point.z),
        Point3(maximum_point.x, maximum_point.y, maximum_point.z),
    )
    transformed_points = [transform_matrix.xformPoint(corner) for corner in corners]
    return (
        Point3(
            min(point.x for point in transformed_points),
            min(point.y for point in transformed_points),
            min(point.z for point in transformed_points),
        ),
        Point3(
            max(point.x for point in transformed_points),
            max(point.y for point in transformed_points),
            max(point.z for point in transformed_points),
        ),
    )


# ---------------------
# Ground / terrain mesh
# ---------------------
procedural_mountain_heightfield = None


def build_mountain_heightfield(size):
    image = PNMImage(size, size, 1)

    mountain_peaks = [
        (-0.34, -0.12, 0.42, 0.40),
        (0.18, 0.10, 0.35, 0.36),
        (0.04, -0.30, 0.24, 0.28),
        (-0.02, 0.34, 0.2, 0.24),
    ]

    for y in range(size):
        for x in range(size):
            u = x / (size - 1)
            v = y / (size - 1)
            nx = (u - 0.5) * 2.0
            ny = (v - 0.5) * 2.0

            height = 0.11
            height += 0.02 * sin(u * 9.5)
            height += 0.018 * cos(v * 8.0)

            for peak_x, peak_y, peak_height, peak_radius in mountain_peaks:
                dx = nx - peak_x
                dy = ny - peak_y
                dist_sq = dx * dx + dy * dy
                radial = max(0.0, 1.0 - dist_sq / (peak_radius * peak_radius))
                height += peak_height * (radial * radial)

            edge = max(abs(nx), abs(ny))
            height -= max(0.0, edge - 0.82) * 0.5

            image.setGray(x, y, max(0.0, min(1.0, height)))

    return image


def build_procedural_mountain_terrain():
    global procedural_mountain_heightfield

    terrain = GeoMipTerrain("procedural_mountains")
    procedural_mountain_heightfield = build_mountain_heightfield(TERRAIN_HEIGHTFIELD_SIZE)
    terrain.setHeightfield(procedural_mountain_heightfield)
    terrain.setBruteforce(True)

    root = terrain.getRoot()
    root.reparentTo(app.render)
    root.setScale(
        TERRAIN_WORLD_X / (TERRAIN_HEIGHTFIELD_SIZE - 1),
        TERRAIN_WORLD_Y / (TERRAIN_HEIGHTFIELD_SIZE - 1),
        TERRAIN_WORLD_Z,
    )
    root.setPos(-TERRAIN_WORLD_X * 0.5, -TERRAIN_WORLD_Y * 0.5, -1.0)

    terrain.generate()
    apply_repeating_texture(root, "models/Grass/Grass1.jpg", 2.4, 2.0)
    return root


def build_ground():
    if USE_PROCEDURAL_MOUNTAIN_TERRAIN:
        return build_procedural_mountain_terrain(), True

    if USE_TERRAIN_MODEL:
        terrain = app.loader.loadModel(asset_path("models/89-terrain/uploads_files_2708212_terrain.fbx"))
        terrain.reparentTo(app.render)
        terrain.setP(90)
        terrain.setScale(TERRAIN_SCALE)
        snap_bottom_to_ground(terrain)
        terrain.setZ(-0.35)
        apply_repeating_texture(terrain, "models/Grass/Grass1.jpg", 1.0, 1.0)
        return terrain, True

    ground_maker = CardMaker("ground")
    ground_maker.setFrame(-40, 40, -40, 40)  #sets it to be a rectangle

    ground_card = app.render.attachNewNode(ground_maker.generate())
    ground_card.setP(-90)
    ground_card.setZ(0)
    ground_card.setColor(0.14, 0.52, 0.2, 1)
    ground_card.setLightOff()
    return ground_card, False


ground, using_terrain = build_ground()


# -------------------------
# Ground sampling collision
# -------------------------
GROUND_COLLIDE_MASK = GeomNode.getDefaultCollideMask()
ground.setCollideMask(GROUND_COLLIDE_MASK)

ground_probe_traverser = CollisionTraverser("ground_probe_traverser")
ground_probe_queue = CollisionHandlerQueue()

ground_probe_ray = CollisionRay(0, 0, 150, 0, 0, -1)
ground_probe_node = CollisionNode("ground_probe")
ground_probe_node.addSolid(ground_probe_ray)
ground_probe_node.setFromCollideMask(GROUND_COLLIDE_MASK)
ground_probe_node.setIntoCollideMask(0)
ground_probe_np = app.render.attachNewNode(ground_probe_node)
ground_probe_traverser.addCollider(ground_probe_np, ground_probe_queue)
ground_sample_cache = {}
GROUND_SAMPLE_CACHE_MISS = object()


def ground_sample_cache_key(x, y):
    if REAL_SIM_GROUND_CACHE_CELL_METERS <= 0.0:
        return None
    return (
        int(round(x / REAL_SIM_GROUND_CACHE_CELL_METERS)),
        int(round(y / REAL_SIM_GROUND_CACHE_CELL_METERS)),
    )


def _heightfield_gray_bilinear(heightfield, x, y):
    size_x = heightfield.getXSize()
    size_y = heightfield.getYSize()
    x0 = max(0, min(size_x - 1, int(x)))
    y0 = max(0, min(size_y - 1, int(y)))
    x1 = min(size_x - 1, x0 + 1)
    y1 = min(size_y - 1, y0 + 1)
    tx = max(0.0, min(1.0, x - x0))
    ty = max(0.0, min(1.0, y - y0))

    h00 = heightfield.getGray(x0, y0)
    h10 = heightfield.getGray(x1, y0)
    h01 = heightfield.getGray(x0, y1)
    h11 = heightfield.getGray(x1, y1)
    h0 = h00 + (h10 - h00) * tx
    h1 = h01 + (h11 - h01) * tx
    return h0 + (h1 - h0) * ty


def sample_procedural_mountain_ground(x, y):
    if procedural_mountain_heightfield is None:
        return None

    size = TERRAIN_HEIGHTFIELD_SIZE
    terrain_x_scale = TERRAIN_WORLD_X / max(1, size - 1)
    terrain_y_scale = TERRAIN_WORLD_Y / max(1, size - 1)
    grid_x = (x + (TERRAIN_WORLD_X * 0.5)) / terrain_x_scale
    grid_y = (y + (TERRAIN_WORLD_Y * 0.5)) / terrain_y_scale
    if grid_x < 0.0 or grid_x > size - 1 or grid_y < 0.0 or grid_y > size - 1:
        return None

    height_gray = _heightfield_gray_bilinear(
        procedural_mountain_heightfield,
        grid_x,
        grid_y,
    )
    z = -1.0 + (height_gray * TERRAIN_WORLD_Z)

    left_gray = _heightfield_gray_bilinear(
        procedural_mountain_heightfield,
        max(0.0, grid_x - 1.0),
        grid_y,
    )
    right_gray = _heightfield_gray_bilinear(
        procedural_mountain_heightfield,
        min(size - 1.0, grid_x + 1.0),
        grid_y,
    )
    down_gray = _heightfield_gray_bilinear(
        procedural_mountain_heightfield,
        grid_x,
        max(0.0, grid_y - 1.0),
    )
    up_gray = _heightfield_gray_bilinear(
        procedural_mountain_heightfield,
        grid_x,
        min(size - 1.0, grid_y + 1.0),
    )
    dz_dx = ((right_gray - left_gray) * TERRAIN_WORLD_Z) / (2.0 * terrain_x_scale)
    dz_dy = ((up_gray - down_gray) * TERRAIN_WORLD_Z) / (2.0 * terrain_y_scale)
    normal = Vec3(-dz_dx, -dz_dy, 1.0)
    if normal.lengthSquared() > 0:
        normal.normalize()
    else:
        normal = Vec3(0, 0, 1)
    return Point3(x, y, z), normal


def sample_ground(x, y):
    cache_key = ground_sample_cache_key(x, y)
    sample_x = x
    sample_y = y
    if cache_key is not None:
        cached_sample = ground_sample_cache.get(cache_key, GROUND_SAMPLE_CACHE_MISS)
        if cached_sample is not GROUND_SAMPLE_CACHE_MISS:
            return cached_sample
        sample_x = cache_key[0] * REAL_SIM_GROUND_CACHE_CELL_METERS
        sample_y = cache_key[1] * REAL_SIM_GROUND_CACHE_CELL_METERS

    if USE_PROCEDURAL_MOUNTAIN_TERRAIN:
        result = sample_procedural_mountain_ground(sample_x, sample_y)
        if cache_key is not None:
            if len(ground_sample_cache) >= REAL_SIM_GROUND_CACHE_LIMIT:
                ground_sample_cache.clear()
            ground_sample_cache[cache_key] = result
        return result

    ground_probe_np.setPos(sample_x, sample_y, 150)
    ground_probe_queue.clearEntries()
    ground_probe_traverser.traverse(ground)

    if ground_probe_queue.getNumEntries() == 0:
        result = None
        if cache_key is not None:
            if len(ground_sample_cache) >= REAL_SIM_GROUND_CACHE_LIMIT:
                ground_sample_cache.clear()
            ground_sample_cache[cache_key] = result
        return result

    ground_probe_queue.sortEntries()
    entry = ground_probe_queue.getEntry(0)
    point = entry.getSurfacePoint(app.render)
    normal = entry.getSurfaceNormal(app.render)

    if normal.lengthSquared() > 0:
        normal.normalize()
    else:
        normal = Vec3(0, 0, 1)

    result = point, normal
    if cache_key is not None:
        if len(ground_sample_cache) >= REAL_SIM_GROUND_CACHE_LIMIT:
            ground_sample_cache.clear()
        ground_sample_cache[cache_key] = result
    return result


# --------------------------
# Play area and spawn bounds
# --------------------------
def compute_spawn_bounds():
    bounds = ground.getTightBounds()
    if bounds is None:
        return (-WORLD_X_LIMIT, WORLD_X_LIMIT, -WORLD_Y_LIMIT, WORLD_Y_LIMIT)

    minimum_point = bounds[0]
    maximum_point = bounds[1]
    margin = 1.5

    usable_width = maximum_point.x - minimum_point.x
    usable_depth = maximum_point.y - minimum_point.y
    if usable_width <= margin * 2 or usable_depth <= margin * 2:
        return (-WORLD_X_LIMIT, WORLD_X_LIMIT, -WORLD_Y_LIMIT, WORLD_Y_LIMIT)

    min_x = minimum_point.x + margin
    max_x = maximum_point.x - margin
    min_y = minimum_point.y + margin
    max_y = maximum_point.y - margin

    if using_terrain:
        center_x = (min_x + max_x) * 0.5
        center_y = (min_y + max_y) * 0.5
        min_x = max(min_x, center_x - WORLD_X_LIMIT)
        max_x = min(max_x, center_x + WORLD_X_LIMIT)
        min_y = max(min_y, center_y - WORLD_Y_LIMIT)
        max_y = min(max_y, center_y + WORLD_Y_LIMIT)

    if min_x >= max_x or min_y >= max_y:
        return (-WORLD_X_LIMIT, WORLD_X_LIMIT, -WORLD_Y_LIMIT, WORLD_Y_LIMIT)

    return (min_x, max_x, min_y, max_y)


def apply_play_area_clip(node_path, x_min, x_max, y_min, y_max):
    plane_specs = [
        ("clip_x_min", Vec3(1, 0, 0), Point3(x_min, 0, 0)),
        ("clip_x_max", Vec3(-1, 0, 0), Point3(x_max, 0, 0)),
        ("clip_y_min", Vec3(0, 1, 0), Point3(0, y_min, 0)),
        ("clip_y_max", Vec3(0, -1, 0), Point3(0, y_max, 0)),
    ]

    clip_planes = []
    for plane_spec in plane_specs:
        plane_name = plane_spec[0]
        plane_normal = plane_spec[1]
        plane_point = plane_spec[2]

        plane_np = app.render.attachNewNode(
            PlaneNode(plane_name, Plane(plane_normal, plane_point))
        )
        node_path.setClipPlane(plane_np)
        clip_planes.append(plane_np)

    return clip_planes


SPAWN_X_MIN, SPAWN_X_MAX, SPAWN_Y_MIN, SPAWN_Y_MAX = compute_spawn_bounds()


def get_water_refill_station_xy():
    """Map-only refill station in the northeast/top-right play-area corner."""
    return (
        SPAWN_X_MAX - WATER_REFILL_CORNER_MARGIN_METERS,
        SPAWN_Y_MAX - WATER_REFILL_CORNER_MARGIN_METERS,
    )

if using_terrain:
    ground_clip_planes = apply_play_area_clip(
        ground,
        SPAWN_X_MIN,
        SPAWN_X_MAX,
        SPAWN_Y_MIN,
        SPAWN_Y_MAX,
    )
else:
    ground_clip_planes = []


def random_world_position(exclusion_radius=8):
    for _ in range(500):
        x = random.uniform(SPAWN_X_MIN, SPAWN_X_MAX)
        y = random.uniform(SPAWN_Y_MIN, SPAWN_Y_MAX)

        if (x * x + (y - 5) * (y - 5)) < exclusion_radius * exclusion_radius:
            continue

        ground_sample = sample_ground(x, y)
        if ground_sample is not None:
            ground_point = ground_sample[0]
            ground_normal = ground_sample[1]
            return x, y, ground_point.z, ground_normal

    return 0, 0, 0, Vec3(0, 0, 1)


def _team_counts_for_spawn():
    """Read live team counts with safe fallbacks during module init."""
    total = globals().get("team_total_drone_count", TEAM_TOTAL_DRONE_COUNT_DEFAULT)
    water = globals().get("team_water_drone_count", TEAM_WATER_DRONE_COUNT_DEFAULT)
    survey = max(1, int(total) - int(water))
    return survey, max(1, int(water))


DRONE_SPAWN_CORNER_MARGIN_METERS = 6.0


def get_strip_first_sweep_xy(slot, strip_count):
    """First sweep-like point in a specific map strip."""
    strip_count = max(1, int(strip_count))
    sector_width = (SPAWN_X_MAX - SPAWN_X_MIN) / strip_count
    index = max(0, min(strip_count - 1, int(slot) - 1))
    x_min = SPAWN_X_MIN + (sector_width * index)
    x_max = SPAWN_X_MAX if index == strip_count - 1 else x_min + sector_width
    margin_x = min(SURVEY_SECTOR_MARGIN_METERS, max(2.0, (x_max - x_min) * 0.18))
    margin_y = min(SURVEY_SECTOR_MARGIN_METERS, max(3.0, (SPAWN_Y_MAX - SPAWN_Y_MIN) * 0.08))
    return x_min + margin_x, SPAWN_Y_MIN + margin_y


def get_survey_first_sweep_xy(slot):
    """Exact (x, y) of the FIRST waypoint of a survey drone's S-sweep, derived
    with the same math as build_survey_search_sectors + build_survey_sector_
    waypoints. Used so a survey drone's birth point IS its search start (#10)."""
    survey_count, _ = _team_counts_for_spawn()
    return get_strip_first_sweep_xy(slot, survey_count)


def get_fixed_drone_spawn_xy(role, slot):
    """Fixed drone birth spots (June 2026 design).

    Each survey drone is born at the first waypoint of its own S-shaped search
    sector. Water drones with matching survey slots spawn near their paired
    survey; extra water drones use their own water strips and row offsets so
    high-water ratios cannot stack at the same point.
    """
    survey_count, water_count = _team_counts_for_spawn()
    if role == "survey":
        return get_survey_first_sweep_xy(slot)

    pair_separation_meters = (
        max(
            DRONE_CRASH_RADIUS_METERS,
            DRONE_INTERACTION_MIN_HORIZONTAL_SEPARATION_METERS,
        )
        + DRONE_SPAWN_PAIR_SEPARATION_MARGIN_METERS
    )
    if int(slot) <= survey_count:
        corner_x, corner_y = get_survey_first_sweep_xy(slot)
        spawn_x = max(
            SPAWN_X_MIN + DRONE_SPAWN_CORNER_MARGIN_METERS,
            min(
                SPAWN_X_MAX - DRONE_SPAWN_CORNER_MARGIN_METERS,
                corner_x + pair_separation_meters,
            ),
        )
        return spawn_x, corner_y

    spawn_x, spawn_y = get_strip_first_sweep_xy(slot, water_count)
    extra_row = max(1, int(slot) - survey_count)
    row_spacing = DRONE_INTERACTION_MIN_HORIZONTAL_SEPARATION_METERS + 1.8
    spawn_y = max(
        SPAWN_Y_MIN + DRONE_SPAWN_CORNER_MARGIN_METERS,
        min(
            SPAWN_Y_MAX - DRONE_SPAWN_CORNER_MARGIN_METERS,
            spawn_y + (extra_row * row_spacing),
        ),
    )
    return spawn_x, spawn_y


def get_water_drone_home_xy(water_slot):
    return get_fixed_drone_spawn_xy("water", water_slot)


FIRE_BURN_GRID_WIDTH = max(
    1,
    int((SPAWN_X_MAX - SPAWN_X_MIN) / FIRE_BURN_CELL_SIZE_METERS) + 1,
)
FIRE_BURN_GRID_HEIGHT = max(
    1,
    int((SPAWN_Y_MAX - SPAWN_Y_MIN) / FIRE_BURN_CELL_SIZE_METERS) + 1,
)


def world_to_fire_cell(x, y):
    cell_x = int((x - SPAWN_X_MIN) / FIRE_BURN_CELL_SIZE_METERS)
    cell_y = int((y - SPAWN_Y_MIN) / FIRE_BURN_CELL_SIZE_METERS)
    cell_x = max(0, min(FIRE_BURN_GRID_WIDTH - 1, cell_x))
    cell_y = max(0, min(FIRE_BURN_GRID_HEIGHT - 1, cell_y))
    return cell_x, cell_y


def fire_cell_to_world(cell_x, cell_y):
    x = SPAWN_X_MIN + (cell_x + 0.5) * FIRE_BURN_CELL_SIZE_METERS
    y = SPAWN_Y_MIN + (cell_y + 0.5) * FIRE_BURN_CELL_SIZE_METERS
    jitter = FIRE_BURN_CELL_SIZE_METERS * 0.35
    return (
        x + random.uniform(-jitter, jitter),
        y + random.uniform(-jitter, jitter),
    )


def build_fire_burnable_cells():
    burnable_cells = set()
    for cell_x in range(FIRE_BURN_GRID_WIDTH):
        for cell_y in range(FIRE_BURN_GRID_HEIGHT):
            sample_x = SPAWN_X_MIN + (cell_x + 0.5) * FIRE_BURN_CELL_SIZE_METERS
            sample_y = SPAWN_Y_MIN + (cell_y + 0.5) * FIRE_BURN_CELL_SIZE_METERS
            if sample_ground(sample_x, sample_y) is None:
                continue
            burnable_cells.add((cell_x, cell_y))
    return burnable_cells


def burn_ratio():
    burnable_total = len(fire_burnable_cells)
    if burnable_total <= 0:
        return 0.0
    return min(1.0, len(fire_burned_cells) / burnable_total)


