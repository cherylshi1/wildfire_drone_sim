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
MISSION_START_CAMERA_PITCH_DEGREES = -8.0
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
PERFORMANCE_WEIGHT_M1 = 0.3
PERFORMANCE_WEIGHT_M2 = 0.2
PERFORMANCE_WEIGHT_M3 = 0.2
PERFORMANCE_WEIGHT_M4 = 0.3
PERFORMANCE_REWARD_THRESHOLD = 0.72
PERFORMANCE_FIREFIGHTER_SUPPRESSION_BASE_SCORE = 0.18
PERFORMANCE_CONTAINED_FIRE_CONTROL_CREDIT = 0.65
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
FIRE_MAP_PANEL_HEIGHT = 0.96
FIRE_MAP_PANEL_RIGHT_MARGIN = 0.04
FIRE_MAP_PANEL_TOP_MARGIN = 0.03
FIRE_MAP_COLLAPSED_SCALE = 1.0
FIRE_MAP_EXPANDED_SCALE = 1.34
FIRE_MAP_ICON_BUTTON_SIZE = 0.048
FIRE_MAP_ICON_BUTTON_GAP = 0.010
FIRE_MAP_SIZE_BUTTON_WIDTH = FIRE_MAP_ICON_BUTTON_SIZE
FIRE_MAP_SIZE_BUTTON_HEIGHT = FIRE_MAP_ICON_BUTTON_SIZE
FIRE_MAP_SIZE_BUTTON_RIGHT_MARGIN = 0.035
FIRE_MAP_SIZE_BUTTON_TOP_MARGIN = 0.075
FIRE_MAP_VIEW_LEFT = 0.04
FIRE_MAP_VIEW_BOTTOM = 0.35
FIRE_MAP_VIEW_WIDTH = 0.78
FIRE_MAP_VIEW_HEIGHT = 0.53
FIRE_MAP_TREE_DOT_HALF_SIZE = 0.0021
FIRE_MAP_DRONE_MARKER_HALF_SIZE = 0.011
FIRE_MAP_SELECTED_MARKER_SCALE = 1.45
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

# Sim simplification (June 2026): ONE random fire source per run, igniting
# at t = 15 s after launch. No other random ignitions, EXCEPT in 6-drone
# runs, which get a second random fire at t = 75 s (1:15).
FIRE_HOTSPOT_INITIAL_COUNT = 1
FIRE_IGNITION_DELAY_SECONDS = 15.0
SECOND_FIRE_IGNITION_DELAY_SECONDS = 75.0
SECOND_FIRE_MIN_TEAM_SIZE = 6
FIRE_SOURCE_COUNT_FOUR_DRONE_CASE = 1
FIRE_SOURCE_COUNT_SIX_DRONE_CASE = 2
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
FIRE_TARGET_GAME_DURATION_SECONDS = 180.0
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
FIRE_REIGNITE_PROBABILITY_PER_SECOND = 0.0  # disabled: single fire source only
FIRE_SPREAD_ENABLED = True
FIRE_FULL_MAP_TARGET_SECONDS = 160.0
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
AUTOMATION_ENABLES_THERMAL_VIEW = True
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


# ---------------------------
# Drone, trees, and grass art
# ---------------------------
TREE_BASE_MODEL_PATH = asset_path("models/Tree 02/Tree.obj")

TREE_VARIANTS = [
    {
        "name": "tree_1",
        "scale": 1.6,
        "pitch": 90,
        "bark_texture": asset_path("models/Tree 02/bark_0004.jpg"),
        "leaf_texture": asset_path("models/Tree 02/DB2X2_L01.png"),
    },
    {
        "name": "tree_2",
        "scale": 1.9,
        "pitch": 90,
        "bark_texture": asset_path("models/Tree1/BarkDecidious0143_5_S.jpg"),
        "leaf_texture": asset_path("models/Tree 02/DB2X2_L02.png"),
    },
    {
        "name": "tree_3",
        "scale": 1.45,
        "pitch": 90,
        "bark_texture": asset_path("models/Tree1/BarkDecidious0194_7_S.jpg"),
        "leaf_texture": asset_path("models/Tree1/Leaves0156_1_S.png"),
    },
]

GRASS_VARIANTS = [
    {
        "name": "short_grass",
        "path": asset_path("models/Grass/Grass.blend"),
        "texture": asset_path("models/Grass/Grass1.jpg"),
        "base_scale": 0.13,
        "base_pitch": 90,
        "scale_min": 0.65,
        "scale_max": 1.2,
        "tint_min": 0.92,
        "tint_max": 1.05,
    },
    {
        "name": "tall_grass",
        "path": asset_path("models/Grass2/Grass.blend"),
        "texture": asset_path("models/Grass2/Grass2.jpg"),
        "base_scale": 0.2,
        "base_pitch": 90,
        "scale_min": 0.95,
        "scale_max": 1.8,
        "tint_min": 0.9,
        "tint_max": 1.02,
    },
]

FIRE_TRUCK_MODEL_CANDIDATE_PATHS = [
    "models/Tonka_Fire_Truck_V2_L3.123c53c9356e-736e-40f5-90ab-d3e9600dba04/10608_Tonka_Fire_Truck_SG_v1_L3.obj",
]
FIRE_TRUCK_TARGET_LENGTH_METERS = 4.0
FIRE_TRUCK_MODEL_PITCH_DEGREES = 0
FIRE_TRUCK_TREE_CLEARANCE_METERS = 8.0
FIRE_TRUCK_SPAWN_ATTEMPTS = 300
FIRE_TRUCK_GROUND_OFFSET_METERS = 0.12
FIRE_TRUCKS_PER_HECTARE = 4.0
FIRE_TRUCK_MIN_COUNT = 1
FIRE_TRUCK_MAX_COUNT = 4
FIRE_TRUCK_COUNT = max(
    FIRE_TRUCK_MIN_COUNT,
    min(
        FIRE_TRUCK_MAX_COUNT,
        int(ceil(max(0.01, NOMINAL_PLAY_AREA_HECTARES) * FIRE_TRUCKS_PER_HECTARE)),
    ),
)
FIRE_TRUCK_SPEED_METERS_PER_SECOND = 7.0
FIRE_TRUCK_FIRELINE_STANDOFF_METERS = 8.0
FIRE_TRUCK_ARRIVAL_RADIUS_METERS = 2.4
FIRE_TRUCK_STAGING_CORNER_MARGIN_METERS = 7.0
FIRE_TRUCK_STAGING_SPACING_METERS = 9.5
FIRE_TRUCK_MAP_MARKER_SCALE = 1.18
FIRE_TRUCK_MAP_ICON_ACTIVE_SCALE = 1.42
FIRE_TRUCK_MAP_ICON_IDLE_SCALE = 1.12
FIRE_TRUCK_MAP_ICON_ACTIVE_BOB = 0.006
FIRE_TRUCK_MAP_ICON_IDLE_BOB = 0.002
FIRE_TRUCK_SUPPRESSING_COLOR = (0.78, 0.08, 1.0, 0.98)
FIRE_TRUCK_SUPPRESSING_GLOW_COLOR = (0.58, 0.05, 1.0, 0.70)
FIRE_TRUCK_SUPPRESSING_CORE_COLOR = (0.96, 0.42, 1.0, 1.0)
USE_DOWNLOADED_FIRE_MODEL = False
FIRE_EFFECT_MODEL_CANDIDATE_PATHS = [
    "models/fire_render_set/VDB_Render_set_1.fbx",
    "models/fire_render_set/Render set .obj",
    "models/fire_render_set/VDB_Render_set_1.obj",
]
FIRE_EFFECT_TARGET_HEIGHT_METERS = 2.35
FIRE_EFFECT_GROUND_OFFSET_METERS = 0.08
FIRE_EFFECT_ACTIVE_SCALE = 1.0
FIRE_EFFECT_DETECTED_SCALE = 1.03
FIRE_EFFECT_CONTAINED_SCALE = 0.55
FIRE_EFFECT_PULSE_AMPLITUDE = 0.04
FIRE_EFFECT_PULSE_SPEED_HZ = 0.85
FIRE_EFFECT_ROTATION_DEGREES_PER_SECOND = 18.0


def _add_flat_fire_truck_fallback(truck_root):
    body = CardMaker("fire_truck_body")
    body.setFrame(-1.7, 1.7, -0.9, 0.9)
    body_np = truck_root.attachNewNode(body.generate())
    body_np.setP(-90)
    body_np.setColor(0.86, 0.08, 0.08, 1)
    body_np.setTwoSided(True)

    stripe = CardMaker("fire_truck_stripe")
    stripe.setFrame(-1.4, 1.4, -0.22, 0.22)
    stripe_np = truck_root.attachNewNode(stripe.generate())
    stripe_np.setP(-90)
    stripe_np.setZ(0.02)
    stripe_np.setColor(0.95, 0.95, 0.95, 1)
    stripe_np.setTwoSided(True)

    cab = CardMaker("fire_truck_cab")
    cab.setFrame(-0.7, 0.7, -0.55, 0.55)
    cab_np = truck_root.attachNewNode(cab.generate())
    cab_np.setP(-90)
    cab_np.setY(0.75)
    cab_np.setZ(0.03)
    cab_np.setColor(0.78, 0.05, 0.05, 1)
    cab_np.setTwoSided(True)


def _load_fire_truck_model():
    for relative_path in FIRE_TRUCK_MODEL_CANDIDATE_PATHS:
        candidate_path = Path(asset_path(relative_path))
        if not candidate_path.exists():
            continue

        try:
            loaded_model = app.loader.loadModel(str(candidate_path))
        except Exception:
            continue

        if not loaded_model.isEmpty():
            return loaded_model

    return None


def _scale_model_to_target_length(model_np, target_length_meters):
    bounds = model_np.getTightBounds()
    if bounds is None:
        return

    minimum_point = bounds[0]
    maximum_point = bounds[1]
    size_x = abs(maximum_point.x - minimum_point.x)
    size_y = abs(maximum_point.y - minimum_point.y)
    dominant_horizontal_size = max(size_x, size_y)

    if dominant_horizontal_size <= 0.0001:
        return

    model_np.setScale(target_length_meters / dominant_horizontal_size)


def _load_fire_effect_model():
    if not USE_DOWNLOADED_FIRE_MODEL:
        return None

    for relative_path in FIRE_EFFECT_MODEL_CANDIDATE_PATHS:
        candidate_path = Path(asset_path(relative_path))
        if not candidate_path.exists():
            continue

        try:
            loaded_model = app.loader.loadModel(str(candidate_path))
        except Exception:
            continue

        if not loaded_model.isEmpty():
            filtered_model = NodePath(PandaNode("fire_effect_filtered"))
            for geometry_np in loaded_model.findAllMatches("**/+GeomNode"):
                if geometry_np.node().getName().startswith("SurfPatch"):
                    geometry_np.copyTo(filtered_model)

            if not filtered_model.find("**/+GeomNode").isEmpty():
                return filtered_model

            return loaded_model

    return None


def _scale_model_to_target_height(model_np, target_height_meters):
    bounds = model_np.getTightBounds()
    if bounds is None:
        return

    minimum_point = bounds[0]
    maximum_point = bounds[1]
    model_height = abs(maximum_point.z - minimum_point.z)

    if model_height <= 0.0001:
        return

    model_np.setScale(target_height_meters / model_height)


def _build_fire_effect_model_instance(hotspot_root):
    if fire_effect_prototype is None:
        return None, 1.0

    effect_model = fire_effect_prototype.copyTo(hotspot_root)
    effect_model.setTransparency(TransparencyAttrib.MAlpha)
    effect_model.setTwoSided(True)
    _scale_model_to_target_height(effect_model, FIRE_EFFECT_TARGET_HEIGHT_METERS)
    snap_bottom_to_ground(effect_model)
    effect_model.setZ(effect_model.getZ() + FIRE_EFFECT_GROUND_OFFSET_METERS)
    effect_model.setH(random.uniform(0.0, 360.0))

    base_scale = effect_model.getScale().x
    if abs(base_scale) <= 0.0001:
        base_scale = 1.0

    return effect_model, base_scale


def _is_position_clear_of_trees(x, y, min_clearance_meters):
    if "trees" not in globals():
        return True

    minimum_clearance_sq = min_clearance_meters * min_clearance_meters
    for tree_node in trees:
        dx = x - tree_node.getX()
        dy = y - tree_node.getY()
        if (dx * dx + dy * dy) < minimum_clearance_sq:
            return False

    return True


def get_fire_truck_staging_spawn(slot):
    margin = max(
        FIRE_TRUCK_STAGING_CORNER_MARGIN_METERS,
        FIRE_TRUCK_TREE_CLEARANCE_METERS * 0.5,
    )
    slot_index = max(0, slot - 1)
    grid_col = slot_index % 2
    grid_row = slot_index // 2
    base_x = SPAWN_X_MIN + margin
    base_y = SPAWN_Y_MIN + margin
    spacing = FIRE_TRUCK_STAGING_SPACING_METERS

    candidate_offsets = (
        (grid_col * spacing, grid_row * spacing),
        ((grid_col + 0.5) * spacing, grid_row * spacing),
        (grid_col * spacing, (grid_row + 0.5) * spacing),
        ((grid_col + 0.5) * spacing, (grid_row + 0.5) * spacing),
        ((grid_col + 1.0) * spacing, grid_row * spacing),
        (grid_col * spacing, (grid_row + 1.0) * spacing),
    )
    for offset_x, offset_y in candidate_offsets:
        x = clamp(base_x + offset_x, SPAWN_X_MIN + margin, SPAWN_X_MAX - margin)
        y = clamp(base_y + offset_y, SPAWN_Y_MIN + margin, SPAWN_Y_MAX - margin)
        ground_sample = sample_ground(x, y)
        if ground_sample is None:
            continue
        if _is_position_clear_of_trees(x, y, FIRE_TRUCK_TREE_CLEARANCE_METERS):
            return x, y, ground_sample[0].z, ground_sample[1]

    x = clamp(
        base_x + grid_col * spacing,
        SPAWN_X_MIN + margin,
        SPAWN_X_MAX - margin,
    )
    y = clamp(
        base_y + grid_row * spacing,
        SPAWN_Y_MIN + margin,
        SPAWN_Y_MAX - margin,
    )
    ground_sample = sample_ground(x, y)
    if ground_sample is None:
        return x, y, 0.0, Vec3(0, 0, 1)
    return x, y, ground_sample[0].z, ground_sample[1]


def spawn_fire_truck_marker(slot=1):
    """Spawn a fire truck model on the ground; fallback to a flat marker if unavailable."""
    x, y, z, _ = get_fire_truck_staging_spawn(slot)

    truck_root = app.render.attachNewNode("fire_truck")
    truck_root.setPos(x, y, z + 0.08)
    truck_root.setH(45.0)

    truck_model = _load_fire_truck_model()
    if truck_model is None:
        _add_flat_fire_truck_fallback(truck_root)
        return truck_root

    truck_model.reparentTo(truck_root)
    truck_model.setP(FIRE_TRUCK_MODEL_PITCH_DEGREES)
    _scale_model_to_target_length(truck_model, FIRE_TRUCK_TARGET_LENGTH_METERS)
    snap_bottom_to_ground(truck_model)
    truck_model.setZ(truck_model.getZ() + FIRE_TRUCK_GROUND_OFFSET_METERS)
    truck_model.flattenLight()

    return truck_root


fire_effect_prototype = _load_fire_effect_model()


@dataclass
class FireHotspot:
    root: object
    base_glow: object
    glow: object
    beacon_a: object
    beacon_b: object
    flame_nodes: tuple[object, ...]
    flame_base_scales: tuple[float, ...]
    mesh_frame_nodes: tuple[object, ...]
    mesh_frame_base_scales: tuple[Vec3, ...]
    scar_core: object
    scar_cross_a: object
    scar_cross_b: object
    ground_z: float
    effect_model: object | None = None
    effect_model_base_scale: float = 1.0
    effect_scale_multiplier: float = 1.0
    effect_heading_degrees: float = field(default_factory=lambda: random.uniform(0.0, 360.0))
    effect_pulse_phase_radians: float = field(default_factory=lambda: random.uniform(0.0, tau))
    fire_cluster_scale_multiplier: float = 1.0
    base_glow_scale_multiplier: float = 1.0
    fire_cluster_color: tuple[float, float, float, float] = (1.0, 0.52, 0.1, 0.86)
    flame_flicker_phase_radians: float = field(default_factory=lambda: random.uniform(0.0, tau))
    flame_texture_frame_offset: int = field(default_factory=lambda: random.randint(0, 23))
    current_flame_texture_frame_index: int = -1
    current_mesh_frame_index: int = -1
    detected: bool = False
    suppression_state: str = FIRE_STATE_ACTIVE
    detection_age_seconds: float = 0.0
    last_flyby_time_seconds: float = -9999.0
    burn_age_seconds: float = 0.0
    ignition_time_seconds: float = 0.0
    detection_time_seconds: float | None = None
    detected_by_survey_slot: int | None = None
    mapping_progress_seconds: float = 0.0
    suppression_work_seconds: float = 0.0
    water_drone_requested: bool = False
    water_drone_assigned: bool = False
    water_drone_engaged: bool = False
    water_drone_assignment_count: int = 0
    water_drone_engagement_count: int = 0
    ground_firefighter_engaged: bool = False
    current_suppression_rate_per_second: float = 0.0
    # Which ignition event this hotspot belongs to (spread children inherit
    # it). Used for fire ownership: the survey drone that first detects any
    # hotspot of an event owns the whole fire; other surveys keep sweeping.
    fire_event_id: int = 1


@dataclass
class FireTruckState:
    slot: int
    root: object
    home_x: float
    home_y: float
    home_z: float
    target_hotspot: object | None = None
    engaged: bool = False


def spawn_fire_truck_fleet():
    trucks = []
    for slot in range(1, FIRE_TRUCK_COUNT + 1):
        truck_root = spawn_fire_truck_marker(slot)
        truck_root.setName(f"fire_truck_{slot}")
        trucks.append(
            FireTruckState(
                slot=slot,
                root=truck_root,
                home_x=truck_root.getX(),
                home_y=truck_root.getY(),
                home_z=truck_root.getZ(),
            )
        )
    return trucks


def hotspot_burn_progress(hotspot):
    if FIRE_UNDETECTED_BURNOUT_DELAY_SECONDS <= 0.0:
        return 1.0
    return max(
        0.0,
        min(1.0, hotspot.burn_age_seconds / FIRE_UNDETECTED_BURNOUT_DELAY_SECONDS),
    )


def hotspot_fire_growth_scale(hotspot):
    growth_reference_seconds = max(
        1.0,
        min(
            FIRE_UNDETECTED_BURNOUT_DELAY_SECONDS,
            FIRE_SUPPRESSION_CONTAIN_DELAY_SECONDS * 1.15,
        ),
    )
    active_growth_seconds = hotspot.burn_age_seconds + min(
        hotspot.detection_age_seconds,
        FIRE_SUPPRESSION_CONTAIN_DELAY_SECONDS,
    )
    active_growth_progress = min(1.0, active_growth_seconds / growth_reference_seconds)
    growth_scale = BAKED_FIRE_GROWTH_MIN_SCALE + (
        (BAKED_FIRE_GROWTH_MAX_SCALE - BAKED_FIRE_GROWTH_MIN_SCALE)
        * active_growth_progress
    )

    if hotspot.suppression_state == FIRE_STATE_CONTAINED:
        contained_elapsed_seconds = max(
            0.0,
            hotspot.detection_age_seconds - FIRE_SUPPRESSION_CONTAIN_DELAY_SECONDS,
        )
        contained_progress = min(
            1.0,
            contained_elapsed_seconds / max(1.0, FIRE_SUPPRESSION_EXTINGUISH_DELAY_SECONDS),
        )
        growth_scale *= 1.0 - (contained_progress * 0.34)

    return growth_scale


def set_hotspot_visual(hotspot):
    using_baked_fire = bool(load_baked_fire_textures())
    using_baked_fire_mesh = bool(hotspot.mesh_frame_nodes)
    hotspot_parts = (
        hotspot.glow,
        hotspot.beacon_a,
        hotspot.beacon_b,
        hotspot.scar_core,
        hotspot.scar_cross_a,
        hotspot.scar_cross_b,
    )
    hotspot_state = hotspot.suppression_state
    state_is_active = hotspot_state == FIRE_STATE_ACTIVE
    state_is_contained = hotspot_state == FIRE_STATE_CONTAINED
    state_is_burned = hotspot_state == FIRE_STATE_BURNED
    burn_progress = hotspot_burn_progress(hotspot)
    firetruck_response_active = (
        hotspot.detected
        and hotspot_has_fire_truck_response(hotspot)
        and hotspot_state in (FIRE_STATE_ACTIVE, FIRE_STATE_CONTAINED)
    )

    if state_is_burned:
        hotspot.scar_core.show()
        hotspot.scar_cross_a.show()
        hotspot.scar_cross_b.show()
        hotspot.scar_cross_a.setScale(1.35)
        hotspot.scar_cross_b.setScale(1.35)
    else:
        hotspot.scar_core.hide()
        hotspot.scar_cross_a.hide()
        hotspot.scar_cross_b.hide()

    if thermal_view_enabled:
        hotspot.base_glow.hide()
        for flame_node in hotspot.flame_nodes:
            flame_node.hide()
        for mesh_node in hotspot.mesh_frame_nodes:
            mesh_node.hide()
    elif state_is_active:
        hotspot.base_glow.show()
        for flame_node in hotspot.flame_nodes:
            flame_node.show()
        if hotspot.current_mesh_frame_index >= 0:
            hotspot.mesh_frame_nodes[hotspot.current_mesh_frame_index].show()
        if firetruck_response_active:
            hotspot.fire_cluster_color = FIRE_TRUCK_SUPPRESSING_COLOR
            hotspot.fire_cluster_scale_multiplier = (
                BAKED_FIRE_DETECTED_SCALE if using_baked_fire else 1.1
            )
            hotspot.base_glow_scale_multiplier = 1.78 if using_baked_fire else 1.6
            hotspot.base_glow.setColor(*FIRE_TRUCK_SUPPRESSING_GLOW_COLOR)
        elif hotspot.detected:
            hotspot.fire_cluster_color = (0.18, 0.94, 1.0, 0.98)
            hotspot.fire_cluster_scale_multiplier = (
                BAKED_FIRE_DETECTED_SCALE if using_baked_fire else 1.08
            )
            hotspot.base_glow_scale_multiplier = 1.72 if using_baked_fire else 1.58
            hotspot.base_glow.setColor(
                0.0,
                0.96,
                1.0,
                0.34 if using_baked_fire_mesh else (0.52 if using_baked_fire else 0.72),
            )
        else:
            # Undetected fire reads as a deep, alarming RED so it stands out
            # clearly from the orange/amber of a detected (being-handled) fire.
            hotspot.fire_cluster_color = (1.0, 0.18, 0.05, 0.95)
            hotspot.fire_cluster_scale_multiplier = (
                BAKED_FIRE_ACTIVE_SCALE if using_baked_fire else 1.0
            )
            hotspot.base_glow_scale_multiplier = 1.04 if using_baked_fire else 1.14
            hotspot.base_glow.setColor(
                1.0,
                0.12,
                0.03,
                0.06 if using_baked_fire_mesh else (0.12 if using_baked_fire else 0.3),
            )
    elif state_is_contained:
        hotspot.base_glow.show()
        for flame_node in hotspot.flame_nodes:
            flame_node.show()
        if hotspot.current_mesh_frame_index >= 0:
            hotspot.mesh_frame_nodes[hotspot.current_mesh_frame_index].show()
        if firetruck_response_active:
            hotspot.fire_cluster_color = FIRE_TRUCK_SUPPRESSING_COLOR
            hotspot.base_glow.setColor(*FIRE_TRUCK_SUPPRESSING_GLOW_COLOR)
        else:
            hotspot.fire_cluster_color = (0.95, 0.34, 0.12, 0.5)
            hotspot.base_glow.setColor(
                0.82,
                0.18,
                0.06,
                0.03 if using_baked_fire_mesh else (0.05 if using_baked_fire else 0.14),
            )
        hotspot.fire_cluster_scale_multiplier = (
            BAKED_FIRE_CONTAINED_SCALE if using_baked_fire else 0.6
        )
        hotspot.base_glow_scale_multiplier = 0.82 if using_baked_fire else 0.72
    else:
        hotspot.base_glow.hide()
        for flame_node in hotspot.flame_nodes:
            flame_node.hide()
        for mesh_node in hotspot.mesh_frame_nodes:
            mesh_node.hide()

    if hotspot.effect_model is not None:
        effect_model_visible = (
            (not thermal_view_enabled)
            and hotspot_state not in (FIRE_STATE_OUT, FIRE_STATE_BURNED)
        )
        if effect_model_visible:
            hotspot.effect_model.show()
            if state_is_active:
                if firetruck_response_active:
                    hotspot.effect_model.setColorScale(*FIRE_TRUCK_SUPPRESSING_COLOR)
                    hotspot.effect_scale_multiplier = FIRE_EFFECT_DETECTED_SCALE
                elif hotspot.detected:
                    hotspot.effect_model.setColorScale(0.28, 0.96, 1.0, 1.0)
                    hotspot.effect_scale_multiplier = FIRE_EFFECT_DETECTED_SCALE
                else:
                    hotspot.effect_model.setColorScale(1.0, 0.42, 0.08, 0.94)
                    hotspot.effect_scale_multiplier = FIRE_EFFECT_ACTIVE_SCALE
            elif state_is_contained:
                if firetruck_response_active:
                    hotspot.effect_model.setColorScale(*FIRE_TRUCK_SUPPRESSING_COLOR)
                else:
                    hotspot.effect_model.setColorScale(0.82, 0.3, 0.12, 0.68)
                hotspot.effect_scale_multiplier = FIRE_EFFECT_CONTAINED_SCALE
            else:
                hotspot.effect_model.hide()
        else:
            hotspot.effect_model.hide()

    if thermal_view_enabled:
        hotspot.glow.show()
        hotspot.beacon_a.show()
        hotspot.beacon_b.show()
        hotspot.glow.setScale(THERMAL_HOTSPOT_GLOW_SCALE)
        hotspot.beacon_a.setScale(THERMAL_HOTSPOT_GLOW_SCALE * 0.72)
        hotspot.beacon_b.setScale(THERMAL_HOTSPOT_GLOW_SCALE * 0.46)
        for part in hotspot_parts:
            part.setBin("fixed", THERMAL_HOTSPOT_BIN_ORDER)
            part.setDepthTest(False)
            part.setDepthWrite(False)

        if state_is_active:
            if firetruck_response_active:
                hotspot.glow.setColor(*FIRE_TRUCK_SUPPRESSING_COLOR)
                hotspot.beacon_a.setColor(*FIRE_TRUCK_SUPPRESSING_CORE_COLOR)
                hotspot.beacon_b.setColor(*FIRE_TRUCK_SUPPRESSING_COLOR)
            elif hotspot.detected:
                hotspot.glow.setColor(0.0, 0.92, 1.0, 0.98)
                hotspot.beacon_a.setColor(0.12, 1.0, 1.0, 0.98)
                hotspot.beacon_b.setColor(0.72, 1.0, 1.0, 0.98)
            else:
                green_channel = max(0.06, 0.26 - burn_progress * 0.2)
                blue_channel = max(0.01, 0.08 - burn_progress * 0.06)
                hotspot.glow.setColor(1.0, green_channel, blue_channel, 0.98)
                hotspot.beacon_a.setColor(1.0, min(1.0, green_channel + 0.24), min(1.0, blue_channel + 0.1), 0.98)
                hotspot.beacon_b.setColor(1.0, min(1.0, green_channel + 0.24), min(1.0, blue_channel + 0.1), 0.98)
        elif state_is_contained:
            if firetruck_response_active:
                hotspot.glow.setColor(*FIRE_TRUCK_SUPPRESSING_COLOR)
                hotspot.beacon_a.setColor(*FIRE_TRUCK_SUPPRESSING_CORE_COLOR)
                hotspot.beacon_b.setColor(*FIRE_TRUCK_SUPPRESSING_COLOR)
            else:
                hotspot.glow.setColor(1.0, 0.74, 0.26, 0.94)
                hotspot.beacon_a.setColor(1.0, 0.86, 0.45, 0.96)
                hotspot.beacon_b.setColor(1.0, 0.86, 0.45, 0.96)
        elif state_is_burned:
            hotspot.glow.setColor(0.48, 0.07, 0.05, 0.82)
            hotspot.beacon_a.setColor(0.62, 0.1, 0.07, 0.86)
            hotspot.beacon_b.setColor(0.74, 0.14, 0.1, 0.88)
            hotspot.scar_core.setColor(0.02, 0.02, 0.02, 0.98)
            hotspot.scar_cross_a.setColor(0.95, 0.12, 0.08, 0.96)
            hotspot.scar_cross_b.setColor(0.95, 0.12, 0.08, 0.96)
        else:
            hotspot.glow.setColor(0.25, 0.45, 0.34, 0.55)
            hotspot.beacon_a.setColor(0.38, 0.65, 0.5, 0.62)
            hotspot.beacon_b.setColor(0.38, 0.65, 0.5, 0.62)
    else:
        for part in hotspot_parts:
            part.clearBin()
            part.setDepthTest(True)
            part.setDepthWrite(True)
        if firetruck_response_active:
            hotspot.glow.show()
            hotspot.beacon_a.show()
            hotspot.beacon_b.show()
            hotspot.glow.setScale(DETECTED_FIRE_OVERLAY_GLOW_SCALE * 1.18)
            hotspot.beacon_a.setScale(DETECTED_FIRE_OVERLAY_BEACON_A_SCALE * 1.12)
            hotspot.beacon_b.setScale(DETECTED_FIRE_OVERLAY_BEACON_B_SCALE * 1.12)
            hotspot.glow.setColor(*FIRE_TRUCK_SUPPRESSING_GLOW_COLOR)
            hotspot.beacon_a.setColor(*FIRE_TRUCK_SUPPRESSING_CORE_COLOR)
            hotspot.beacon_b.setColor(*FIRE_TRUCK_SUPPRESSING_COLOR)
            for marker_part in (hotspot.glow, hotspot.beacon_a, hotspot.beacon_b):
                marker_part.setDepthTest(False)
                marker_part.setDepthWrite(False)
        elif state_is_active and hotspot.detected:
            hotspot.glow.show()
            hotspot.beacon_a.show()
            hotspot.beacon_b.show()
            hotspot.glow.setScale(DETECTED_FIRE_OVERLAY_GLOW_SCALE)
            hotspot.beacon_a.setScale(DETECTED_FIRE_OVERLAY_BEACON_A_SCALE)
            hotspot.beacon_b.setScale(DETECTED_FIRE_OVERLAY_BEACON_B_SCALE)
            hotspot.glow.setColor(0.0, 0.9, 1.0, 0.82)
            hotspot.beacon_a.setColor(0.1, 1.0, 1.0, 0.9)
            hotspot.beacon_b.setColor(0.78, 1.0, 1.0, 0.96)
            for marker_part in (hotspot.glow, hotspot.beacon_a, hotspot.beacon_b):
                marker_part.setDepthTest(False)
                marker_part.setDepthWrite(False)
        else:
            hotspot.glow.hide()
            hotspot.beacon_a.hide()
            hotspot.beacon_b.hide()
        if state_is_burned:
            hotspot.scar_core.setColor(0.01, 0.01, 0.01, 0.98)
            hotspot.scar_cross_a.setColor(0.78, 0.06, 0.03, 0.96)
            hotspot.scar_cross_b.setColor(0.78, 0.06, 0.03, 0.96)

    if firetruck_response_active:
        hotspot.scar_core.show()
        hotspot.scar_cross_a.show()
        hotspot.scar_cross_b.show()
        hotspot.scar_core.setScale(1.28)
        hotspot.scar_cross_a.setScale(1.68)
        hotspot.scar_cross_b.setScale(1.68)
        hotspot.scar_core.setColor(
            FIRE_TRUCK_SUPPRESSING_COLOR[0],
            FIRE_TRUCK_SUPPRESSING_COLOR[1],
            FIRE_TRUCK_SUPPRESSING_COLOR[2],
            0.34,
        )
        hotspot.scar_cross_a.setColor(*FIRE_TRUCK_SUPPRESSING_COLOR)
        hotspot.scar_cross_b.setColor(*FIRE_TRUCK_SUPPRESSING_CORE_COLOR)
        for outline_part in (
            hotspot.scar_core,
            hotspot.scar_cross_a,
            hotspot.scar_cross_b,
        ):
            outline_part.setBin("fixed", THERMAL_HOTSPOT_BIN_ORDER + 1)
            outline_part.setDepthTest(False)
            outline_part.setDepthWrite(False)


def update_hotspot_effect_animation(dt):
    if not fire_hotspots:
        return

    animated_fire_textures = load_baked_fire_textures()
    animated_fire_frame_rate = baked_fire_frame_rate()
    animated_fire_mesh_frame_rate = baked_fire_mesh_frame_rate()
    for hotspot in fire_hotspots:
        fire_growth_scale = hotspot_fire_growth_scale(hotspot)
        suppression_visual_progress = 0.0
        if hotspot.suppression_state == FIRE_STATE_ACTIVE:
            suppression_visual_progress = clamp(
                hotspot.suppression_work_seconds
                / max(1.0, FIRE_SUPPRESSION_CONTAIN_DELAY_SECONDS),
                0.0,
                1.0,
            )
        elif hotspot.suppression_state == FIRE_STATE_CONTAINED:
            contained_elapsed = max(
                0.0,
                hotspot.suppression_work_seconds
                - FIRE_SUPPRESSION_CONTAIN_DELAY_SECONDS,
            )
            suppression_visual_progress = clamp(
                contained_elapsed
                / max(1.0, FIRE_SUPPRESSION_EXTINGUISH_DELAY_SECONDS),
                0.0,
                1.0,
            )
        suppression_visual_intensity = 0.0
        if hotspot.current_suppression_rate_per_second > 0.0:
            suppression_visual_intensity = clamp(
                hotspot.current_suppression_rate_per_second
                / (
                    GROUND_FIREFIGHTER_SUPPRESSION_RATE_PER_SECOND
                    + WATER_DRONE_ASSIST_SUPPRESSION_RATE_PER_SECOND
                ),
                0.0,
                1.0,
            )
        if hotspot.suppression_state in (FIRE_STATE_ACTIVE, FIRE_STATE_CONTAINED):
            fire_growth_scale *= 1.0 - (
                suppression_visual_progress
                * (0.22 if hotspot.suppression_state == FIRE_STATE_ACTIVE else 0.35)
            )
        if (
            (not thermal_view_enabled)
            and hotspot.suppression_state not in (FIRE_STATE_OUT, FIRE_STATE_BURNED)
        ):
            if hotspot.mesh_frame_nodes:
                raw_mesh_frame_index = int(
                    motion_state.sim_time_seconds * animated_fire_mesh_frame_rate
                ) + hotspot.flame_texture_frame_offset
                mesh_frame_index = raw_mesh_frame_index % len(hotspot.mesh_frame_nodes)
                if mesh_frame_index != hotspot.current_mesh_frame_index:
                    if hotspot.current_mesh_frame_index >= 0:
                        hotspot.mesh_frame_nodes[
                            hotspot.current_mesh_frame_index
                        ].hide()
                    hotspot.current_mesh_frame_index = mesh_frame_index
                    hotspot.mesh_frame_nodes[mesh_frame_index].show()

            if animated_fire_textures:
                raw_frame_index = int(
                    motion_state.sim_time_seconds * animated_fire_frame_rate
                ) + hotspot.flame_texture_frame_offset
                frame_index = raw_frame_index % len(animated_fire_textures)
                if frame_index != hotspot.current_flame_texture_frame_index:
                    hotspot.current_flame_texture_frame_index = frame_index
                    frame_texture = animated_fire_textures[frame_index]
                    for flame_node in hotspot.flame_nodes:
                        flame_node.setTexture(frame_texture, 1)

            hotspot.flame_flicker_phase_radians = (
                hotspot.flame_flicker_phase_radians + (tau * 1.75 * dt)
            ) % tau

            # Flame tint: deep RED while undetected, bright AMBER once detected,
            # so the detection flip is unmistakable. Undetected fires ALSO throb
            # in size (undetected_pulse) so an unseen fire visibly pulses/alarms
            # while a detected (being-handled) fire burns steady.
            undetected_pulse = 1.0
            if hotspot.suppression_state == FIRE_STATE_ACTIVE and not hotspot.detected:
                flame_g_fac, flame_b_fac = UNDETECTED_FIRE_GREEN_SCALE, UNDETECTED_FIRE_BLUE_SCALE
                undetected_pulse = 1.0 + 0.26 * sin(
                    hotspot.flame_flicker_phase_radians * 2.6
                )
            elif hotspot.suppression_state == FIRE_STATE_ACTIVE and hotspot.detected:
                flame_g_fac, flame_b_fac = DETECTED_FIRE_GREEN_SCALE, DETECTED_FIRE_BLUE_SCALE
            else:
                flame_g_fac, flame_b_fac = 1.0, 1.0
            for index, flame_node in enumerate(hotspot.flame_nodes):
                phase = hotspot.flame_flicker_phase_radians + (index * 0.82)
                flicker = 1.0 + (sin(phase) * 0.12) + (sin(phase * 2.15 + 0.6) * 0.05)
                flicker = max(0.78, flicker)
                flame_support_scale = 0.68 if hotspot.mesh_frame_nodes else 1.0
                flame_node.setScale(
                    hotspot.flame_base_scales[index]
                    * hotspot.fire_cluster_scale_multiplier
                    * flame_support_scale
                    * fire_growth_scale
                    * flicker
                    * undetected_pulse
                )

                brightness = max(0.72, 1.0 - (index * 0.08))
                flame_alpha_multiplier = 1.0 - (
                    (suppression_visual_progress * 0.32)
                    + (suppression_visual_intensity * 0.2)
                )
                flame_is_detected = (
                    hotspot.suppression_state == FIRE_STATE_ACTIVE
                    and hotspot.detected
                )
                flame_is_firetruck_suppressed = (
                    flame_is_detected
                    and hotspot_has_fire_truck_response(hotspot)
                )
                if animated_fire_textures:
                    if flame_is_firetruck_suppressed:
                        flame_node.setColor(
                            max(0.0, FIRE_TRUCK_SUPPRESSING_COLOR[0] * brightness),
                            max(0.0, FIRE_TRUCK_SUPPRESSING_COLOR[1] * brightness),
                            FIRE_TRUCK_SUPPRESSING_COLOR[2],
                            FIRE_TRUCK_SUPPRESSING_COLOR[3]
                            * (0.88 if hotspot.mesh_frame_nodes else 1.0)
                            * max(0.72, 1.0 - (index * 0.07))
                            * flame_alpha_multiplier,
                        )
                    elif flame_is_detected:
                        flame_node.setColor(
                            0.16 + brightness * 0.18,
                            min(1.0, 0.82 + brightness * 0.2),
                            1.0,
                            hotspot.fire_cluster_color[3]
                            * (0.88 if hotspot.mesh_frame_nodes else 1.0)
                            * max(0.72, 1.0 - (index * 0.07))
                            * flame_alpha_multiplier,
                        )
                    else:
                        flame_node.setColor(
                            max(0.68, brightness - suppression_visual_intensity * 0.16),
                            min(1.0, (brightness * 0.98 + suppression_visual_intensity * 0.08) * flame_g_fac),
                            min(1.0, (brightness * 0.92 + suppression_visual_intensity * 0.16) * flame_b_fac),
                            hotspot.fire_cluster_color[3]
                            * (0.76 if hotspot.mesh_frame_nodes else 1.0)
                            * max(0.62, 1.0 - (index * 0.1))
                            * flame_alpha_multiplier,
                        )
                else:
                    if flame_is_firetruck_suppressed:
                        flame_node.setColor(
                            max(0.0, FIRE_TRUCK_SUPPRESSING_COLOR[0] * brightness),
                            max(0.0, FIRE_TRUCK_SUPPRESSING_COLOR[1] * brightness),
                            FIRE_TRUCK_SUPPRESSING_COLOR[2],
                            FIRE_TRUCK_SUPPRESSING_COLOR[3]
                            * max(0.72, 1.0 - (index * 0.06))
                            * flame_alpha_multiplier,
                        )
                    elif flame_is_detected:
                        flame_node.setColor(
                            0.12 + brightness * 0.2,
                            min(1.0, 0.84 + brightness * 0.2),
                            1.0,
                            hotspot.fire_cluster_color[3]
                            * max(0.72, 1.0 - (index * 0.06))
                            * flame_alpha_multiplier,
                        )
                    else:
                        flame_node.setColor(
                            1.0,
                            min(
                                1.0,
                                hotspot.fire_cluster_color[1] * brightness
                                + 0.08
                                + suppression_visual_intensity * 0.1,
                            ),
                            min(
                                1.0,
                                hotspot.fire_cluster_color[2] * brightness
                                + suppression_visual_intensity * 0.22,
                            ),
                            hotspot.fire_cluster_color[3]
                            * max(0.55, 1.0 - (index * 0.08))
                            * flame_alpha_multiplier,
                        )

            glow_pulse = 1.0 + (sin(hotspot.flame_flicker_phase_radians * 0.72) * 0.08)
            hotspot.base_glow.setScale(
                hotspot.base_glow_scale_multiplier
                * max(0.72, fire_growth_scale * 0.82)
                * glow_pulse
                * undetected_pulse
            )

            if hotspot.mesh_frame_nodes and hotspot.current_mesh_frame_index >= 0:
                mesh_phase = hotspot.flame_flicker_phase_radians * 0.6
                mesh_pulse = 0.96 + (sin(mesh_phase) * 0.045)
                mesh_node = hotspot.mesh_frame_nodes[hotspot.current_mesh_frame_index]
                mesh_base_scale = hotspot.mesh_frame_base_scales[
                    hotspot.current_mesh_frame_index
                ]
                mesh_scale_multiplier = (
                    hotspot.fire_cluster_scale_multiplier
                    * fire_growth_scale
                    * mesh_pulse
                    * undetected_pulse
                )
                mesh_node.setScale(
                    mesh_base_scale.x * mesh_scale_multiplier,
                    mesh_base_scale.y * mesh_scale_multiplier,
                    mesh_base_scale.z * mesh_scale_multiplier,
                )
                if hotspot.suppression_state == FIRE_STATE_CONTAINED:
                    mesh_alpha = BAKED_FIRE_MESH_CONTAINED_ALPHA
                elif hotspot.detected:
                    mesh_alpha = BAKED_FIRE_MESH_DETECTED_ALPHA
                else:
                    mesh_alpha = BAKED_FIRE_MESH_ACTIVE_ALPHA
                if hotspot.detected and hotspot_has_fire_truck_response(hotspot):
                    mesh_node.setColorScale(
                        FIRE_TRUCK_SUPPRESSING_COLOR[0],
                        FIRE_TRUCK_SUPPRESSING_COLOR[1],
                        FIRE_TRUCK_SUPPRESSING_COLOR[2],
                        mesh_alpha * (1.0 - (suppression_visual_progress * 0.25)),
                    )
                elif hotspot.suppression_state == FIRE_STATE_ACTIVE and hotspot.detected:
                    mesh_node.setColorScale(
                        0.2,
                        0.96,
                        1.0,
                        mesh_alpha * (1.0 - (suppression_visual_progress * 0.25)),
                    )
                else:
                    mesh_node.setColorScale(
                        1.0,
                        min(1.0, (hotspot.fire_cluster_color[1] + 0.08 + suppression_visual_intensity * 0.1) * flame_g_fac),
                        min(1.0, (hotspot.fire_cluster_color[2] + 0.05 + suppression_visual_intensity * 0.22) * flame_b_fac),
                        mesh_alpha * (1.0 - (suppression_visual_progress * 0.45)),
                    )

        if hotspot.effect_model is None:
            continue
        if thermal_view_enabled:
            continue
        if hotspot.suppression_state in (FIRE_STATE_OUT, FIRE_STATE_BURNED):
            continue

        hotspot.effect_pulse_phase_radians = (
            hotspot.effect_pulse_phase_radians
            + (tau * FIRE_EFFECT_PULSE_SPEED_HZ * dt)
        ) % tau

        pulse_amplitude = FIRE_EFFECT_PULSE_AMPLITUDE
        rotation_speed = FIRE_EFFECT_ROTATION_DEGREES_PER_SECOND
        if hotspot.suppression_state == FIRE_STATE_CONTAINED:
            pulse_amplitude *= 0.55
            rotation_speed *= 0.45

        pulse_scale = 1.0 + (sin(hotspot.effect_pulse_phase_radians) * pulse_amplitude)
        hotspot.effect_heading_degrees = (
            hotspot.effect_heading_degrees + rotation_speed * dt
        ) % 360.0

        hotspot.effect_model.setScale(
            hotspot.effect_model_base_scale
            * hotspot.effect_scale_multiplier
            * pulse_scale
        )
        hotspot.effect_model.setH(hotspot.effect_heading_degrees)


def build_ground_fire_visual(root):
    animated_fire_textures = load_baked_fire_textures()
    mesh_prototypes = load_baked_fire_mesh_prototypes()
    base_fire_texture = (
        animated_fire_textures[0]
        if animated_fire_textures
        else HOTSPOT_FLAME_TEXTURE
    )

    base_glow_card = CardMaker("hotspot_base_glow")
    if animated_fire_textures:
        base_glow_card.setFrame(
            -BAKED_FIRE_GLOW_HALF_WIDTH_METERS,
            BAKED_FIRE_GLOW_HALF_WIDTH_METERS,
            -BAKED_FIRE_GLOW_HALF_DEPTH_METERS,
            BAKED_FIRE_GLOW_HALF_DEPTH_METERS,
        )
    else:
        base_glow_card.setFrame(-1.35, 1.35, -0.82, 0.82)
    base_glow = root.attachNewNode(base_glow_card.generate())
    base_glow.setP(-90)
    base_glow.setZ(0.025)
    base_glow.setTransparency(TransparencyAttrib.MAlpha)
    base_glow.setTwoSided(True)
    base_glow.setTexture(HOTSPOT_GLOW_TEXTURE, 1)
    base_glow.setDepthWrite(False)

    if animated_fire_textures:
        flame_layouts = (
            (
                (0.0, 0.02, BAKED_FIRE_CARD_LIFT_METERS),
                (
                    -BAKED_FIRE_CARD_HALF_WIDTH_METERS,
                    BAKED_FIRE_CARD_HALF_WIDTH_METERS,
                    0.0,
                    BAKED_FIRE_CARD_HEIGHT_METERS,
                ),
                1.0,
                0.0,
            ),
            (
                (-0.36, -0.16, BAKED_FIRE_CARD_LIFT_METERS + 0.08),
                (
                    -(BAKED_FIRE_CARD_HALF_WIDTH_METERS * 0.82),
                    BAKED_FIRE_CARD_HALF_WIDTH_METERS * 0.82,
                    0.0,
                    BAKED_FIRE_CARD_HEIGHT_METERS * 0.96,
                ),
                0.98,
                -28.0,
            ),
            (
                (0.38, 0.18, BAKED_FIRE_CARD_LIFT_METERS + 0.06),
                (
                    -(BAKED_FIRE_CARD_HALF_WIDTH_METERS * 0.8),
                    BAKED_FIRE_CARD_HALF_WIDTH_METERS * 0.8,
                    0.0,
                    BAKED_FIRE_CARD_HEIGHT_METERS * 0.94,
                ),
                0.96,
                28.0,
            ),
            (
                (-0.18, 0.34, BAKED_FIRE_CARD_LIFT_METERS + 0.12),
                (
                    -(BAKED_FIRE_CARD_HALF_WIDTH_METERS * 0.64),
                    BAKED_FIRE_CARD_HALF_WIDTH_METERS * 0.64,
                    0.0,
                    BAKED_FIRE_CARD_HEIGHT_METERS * 0.8,
                ),
                0.88,
                -58.0,
            ),
            (
                (0.24, -0.32, BAKED_FIRE_CARD_LIFT_METERS + 0.1),
                (
                    -(BAKED_FIRE_CARD_HALF_WIDTH_METERS * 0.62),
                    BAKED_FIRE_CARD_HALF_WIDTH_METERS * 0.62,
                    0.0,
                    BAKED_FIRE_CARD_HEIGHT_METERS * 0.76,
                ),
                0.84,
                58.0,
            ),
        )
    else:
        flame_layouts = (
            ((0.0, 0.16, 0.05), (-0.62, 0.62, 0.0, 0.94), 1.0, None),
            ((-0.54, -0.04, 0.03), (-0.48, 0.48, 0.0, 0.78), 0.88, None),
            ((0.54, 0.0, 0.03), (-0.5, 0.5, 0.0, 0.8), 0.9, None),
            ((-0.92, 0.1, 0.02), (-0.38, 0.38, 0.0, 0.58), 0.72, None),
            ((0.92, 0.08, 0.02), (-0.4, 0.4, 0.0, 0.6), 0.74, None),
            ((0.0, -0.3, 0.02), (-0.52, 0.52, 0.0, 0.72), 0.82, None),
        )

    flame_nodes = []
    flame_base_scales = []
    for flame_index, (offset, frame, base_scale, billboard_axis_offset) in enumerate(flame_layouts):
        flame_card = CardMaker(f"hotspot_flame_{flame_index}")
        flame_card.setFrame(*frame)
        flame_node = root.attachNewNode(flame_card.generate())
        flame_node.setPos(*offset)
        flame_node.setTransparency(TransparencyAttrib.MAlpha)
        flame_node.setTwoSided(True)
        flame_node.setTexture(base_fire_texture, 1)
        if billboard_axis_offset is None:
            flame_node.setBillboardPointEye()
        else:
            flame_node.setBillboardAxis(billboard_axis_offset)
        flame_node.setDepthWrite(False)
        flame_node.setScale(base_scale)
        flame_nodes.append(flame_node)
        flame_base_scales.append(base_scale)

    mesh_frame_nodes = []
    mesh_frame_base_scales = []
    if mesh_prototypes:
        for mesh_prototype in mesh_prototypes:
            mesh_node = mesh_prototype.copyTo(root)
            mesh_node.hide()
            mesh_node.setTransparency(TransparencyAttrib.MAlpha)
            mesh_node.setTwoSided(False)
            mesh_node.setDepthWrite(True)
            mesh_node.setP(90)
            _scale_model_to_target_height(
                mesh_node,
                BAKED_FIRE_MESH_TARGET_HEIGHT_METERS,
            )
            mesh_scale = mesh_node.getScale()
            mesh_node.setScale(
                mesh_scale.x * BAKED_FIRE_MESH_HORIZONTAL_SCALE,
                mesh_scale.y,
                mesh_scale.z * BAKED_FIRE_MESH_HORIZONTAL_SCALE,
            )
            snap_bottom_to_ground(mesh_node)
            mesh_node.setZ(mesh_node.getZ() + BAKED_FIRE_MESH_GROUND_OFFSET_METERS)
            mesh_node.setColorScale(1.0, 0.5, 0.14, BAKED_FIRE_MESH_ACTIVE_ALPHA)
            base_scale = Vec3(mesh_node.getScale())
            mesh_frame_nodes.append(mesh_node)
            mesh_frame_base_scales.append(base_scale)

    return (
        base_glow,
        tuple(flame_nodes),
        tuple(flame_base_scales),
        tuple(mesh_frame_nodes),
        tuple(mesh_frame_base_scales),
    )


def build_hotspot_node():
    root = app.render.attachNewNode("fire_hotspot")

    (
        base_glow,
        flame_nodes,
        flame_base_scales,
        mesh_frame_nodes,
        mesh_frame_base_scales,
    ) = build_ground_fire_visual(root)

    glow_card = CardMaker("hotspot_glow")
    glow_card.setFrame(-1.1, 1.1, -1.1, 1.1)
    glow_np = root.attachNewNode(glow_card.generate())
    glow_np.setP(-90)
    glow_np.setTransparency(TransparencyAttrib.MAlpha)
    glow_np.setTwoSided(True)
    glow_np.setTexture(HOTSPOT_GLOW_TEXTURE, 1)

    beacon_card = CardMaker("hotspot_beacon")
    beacon_card.setFrame(-0.7, 0.7, -0.7, 0.7)
    beacon_a = root.attachNewNode(beacon_card.generate())
    beacon_a.setP(-90)
    beacon_a.setZ(0.015)
    beacon_a.setTransparency(TransparencyAttrib.MAlpha)
    beacon_a.setTwoSided(True)
    beacon_a.setTexture(HOTSPOT_GLOW_TEXTURE, 1)

    beacon_b = root.attachNewNode(beacon_card.generate())
    beacon_b.setP(-90)
    beacon_b.setZ(0.03)
    beacon_b.setTransparency(TransparencyAttrib.MAlpha)
    beacon_b.setTwoSided(True)
    beacon_b.setTexture(HOTSPOT_GLOW_TEXTURE, 1)

    scar_core_card = CardMaker("hotspot_scar_core")
    scar_core_card.setFrame(-0.48, 0.48, -0.48, 0.48)
    scar_core = root.attachNewNode(scar_core_card.generate())
    scar_core.setP(-90)
    scar_core.setZ(0.05)
    scar_core.setTransparency(TransparencyAttrib.MAlpha)
    scar_core.setTwoSided(True)

    scar_cross_card = CardMaker("hotspot_scar_cross")
    scar_cross_card.setFrame(-1.45, 1.45, -0.16, 0.16)
    scar_cross_a = root.attachNewNode(scar_cross_card.generate())
    scar_cross_a.setP(-90)
    scar_cross_a.setH(45)
    scar_cross_a.setZ(0.055)
    scar_cross_a.setTransparency(TransparencyAttrib.MAlpha)
    scar_cross_a.setTwoSided(True)

    scar_cross_b = root.attachNewNode(scar_cross_card.generate())
    scar_cross_b.setP(-90)
    scar_cross_b.setH(-45)
    scar_cross_b.setZ(0.058)
    scar_cross_b.setTransparency(TransparencyAttrib.MAlpha)
    scar_cross_b.setTwoSided(True)

    return (
        root,
        base_glow,
        glow_np,
        beacon_a,
        beacon_b,
        flame_nodes,
        flame_base_scales,
        mesh_frame_nodes,
        mesh_frame_base_scales,
        scar_core,
        scar_cross_a,
        scar_cross_b,
    )


def build_fire_hotspot_at(x, y, z):
    (
        root,
        base_glow,
        glow_np,
        beacon_a,
        beacon_b,
        flame_nodes,
        flame_base_scales,
        mesh_frame_nodes,
        mesh_frame_base_scales,
        scar_core,
        scar_cross_a,
        scar_cross_b,
    ) = build_hotspot_node()
    root.setPos(x, y, z + 0.03)
    effect_model, effect_model_base_scale = _build_fire_effect_model_instance(root)

    hotspot = FireHotspot(
        root=root,
        base_glow=base_glow,
        glow=glow_np,
        beacon_a=beacon_a,
        beacon_b=beacon_b,
        flame_nodes=flame_nodes,
        flame_base_scales=flame_base_scales,
        mesh_frame_nodes=mesh_frame_nodes,
        mesh_frame_base_scales=mesh_frame_base_scales,
        scar_core=scar_core,
        scar_cross_a=scar_cross_a,
        scar_cross_b=scar_cross_b,
        ground_z=z,
        effect_model=effect_model,
        effect_model_base_scale=effect_model_base_scale,
    )
    set_hotspot_visual(hotspot)
    if "motion_state" in globals():
        hotspot.ignition_time_seconds = motion_state.sim_time_seconds
    hotspot_cell = world_to_fire_cell(x, y)
    if "fire_burnable_cells" in globals():
        fire_burnable_cells.add(hotspot_cell)
    fire_burned_cells.add(hotspot_cell)
    return hotspot


def _is_position_clear_of_hotspots(x, y, min_clearance_meters):
    if "fire_hotspots" not in globals():
        return True

    min_clearance_sq = min_clearance_meters * min_clearance_meters
    for hotspot in fire_hotspots:
        dx = hotspot.root.getX() - x
        dy = hotspot.root.getY() - y
        if dx * dx + dy * dy < min_clearance_sq:
            return False
    return True


def spawn_fire_hotspots(count):
    hotspots = []

    spawn_count = max(0, int(count))
    for _ in range(spawn_count):
        chosen_spawn = None
        for _ in range(FIRE_HOTSPOT_SPAWN_ATTEMPTS):
            candidate_spawn = random_world_position(exclusion_radius=18)
            candidate_x = candidate_spawn[0]
            candidate_y = candidate_spawn[1]
            if _is_position_clear_of_trees(
                candidate_x,
                candidate_y,
                FIRE_HOTSPOT_TREE_CLEARANCE_METERS,
            ):
                chosen_spawn = candidate_spawn
                break

        if chosen_spawn is None:
            chosen_spawn = random_world_position(exclusion_radius=18)

        x = chosen_spawn[0]
        y = chosen_spawn[1]
        z = chosen_spawn[2]

        hotspots.append(build_fire_hotspot_at(x, y, z))

    return hotspots


def trim_active_fire_hotspots():
    non_burned_count = sum(
        1 for hotspot in fire_hotspots
        if hotspot.suppression_state != FIRE_STATE_BURNED
    )
    if non_burned_count <= FIRE_HOTSPOT_ACTIVE_LIMIT:
        return

    overflow_count = non_burned_count - FIRE_HOTSPOT_ACTIVE_LIMIT
    while overflow_count > 0:
        trim_index = next(
            (
                index
                for index, hotspot in enumerate(fire_hotspots)
                if hotspot.suppression_state != FIRE_STATE_BURNED
            ),
            None,
        )
        if trim_index is None:
            break
        hotspot = fire_hotspots.pop(trim_index)
        hotspot.root.removeNode()
        overflow_count -= 1


def build_tree_prototypes():
    prototypes = []

    bark_textures = {}
    leaf_textures = {}
    for variant in TREE_VARIANTS:
        bark_texture_path = variant["bark_texture"]
        leaf_texture_path = variant["leaf_texture"]
        bark_textures[bark_texture_path] = app.loader.loadTexture(bark_texture_path)
        leaf_textures[leaf_texture_path] = app.loader.loadTexture(leaf_texture_path)

    for variant in TREE_VARIANTS:
        prototype = app.loader.loadModel(TREE_BASE_MODEL_PATH)
        prototype.setName(variant["name"])
        prototype.setP(variant["pitch"])
        prototype.setScale(variant["scale"] * TREE_HEIGHT_MULTIPLIER)

        trunk = prototype.find("**/g1")
        leaves = prototype.find("**/g2")

        if not trunk.isEmpty():
            trunk.setTexture(bark_textures[variant["bark_texture"]], 1)

        if not leaves.isEmpty():
            leaves.setTexture(leaf_textures[variant["leaf_texture"]], 1)
            leaves.setTransparency(TransparencyAttrib.MDual, 1)
            leaves.setTwoSided(True)

            # Move leaves up so canopies start higher along the trunk.
            trunk_bounds = trunk.getTightBounds(prototype)
            leaves_bounds = leaves.getTightBounds(prototype)
            if trunk_bounds is not None and leaves_bounds is not None:
                trunk_min = trunk_bounds[0]
                trunk_max = trunk_bounds[1]
                leaves_min = leaves_bounds[0]
                desired_leaf_base_z = trunk_min.z + (trunk_max.z - trunk_min.z) * LEAF_START_HEIGHT_RATIO
                z_shift = desired_leaf_base_z - leaves_min.z
                if z_shift > 0:
                    leaves.setZ(leaves.getZ() + z_shift)

        prototype.setTransparency(TransparencyAttrib.MDual, 1)
        prototype.setTwoSided(True)
        snap_bottom_to_ground(prototype)
        canopy_bounds_in_prototype = (
            leaves.getTightBounds(prototype)
            if not leaves.isEmpty()
            else prototype.getTightBounds(prototype)
        )
        canopy_bounds = transform_bounds(
            canopy_bounds_in_prototype,
            prototype.getMat(),
        )
        if canopy_bounds is not None:
            minimum_point, maximum_point = canopy_bounds
            prototype.setPythonTag(
                "canopy_local_bounds",
                (
                    minimum_point.x,
                    maximum_point.x,
                    minimum_point.y,
                    maximum_point.y,
                    minimum_point.z,
                    maximum_point.z,
                ),
            )
        prototype.flattenLight()
        prototypes.append(prototype)

    return prototypes


def build_grass_prototypes():
    if GRASS_COUNT <= 0:
        return []

    prototypes = []

    for variant in GRASS_VARIANTS:
        prototype = app.loader.loadModel(variant["path"])

        for junk_name in ("**/Camera", "**/Plane", "**/ground"):
            junk = prototype.find(junk_name)
            if not junk.isEmpty():
                junk.removeNode()

        prototype.setTransparency(TransparencyAttrib.MDual, 1)
        prototype.setTwoSided(True)
        prototype.setTexture(load_cutout_grass_texture(variant["texture"]), 1)
        prototype.setScale(variant["base_scale"])
        prototype.setP(variant["base_pitch"])
        snap_bottom_to_ground(prototype)
        prototype.flattenLight()

        prototypes.append(
            {
                "name": variant["name"],
                "model": prototype,
                "scale_min": variant["scale_min"],
                "scale_max": variant["scale_max"],
                "tint_min": variant["tint_min"],
                "tint_max": variant["tint_max"],
            }
        )

    return prototypes


def spawn_trees(tree_prototypes):
    tree_nodes = []

    for _ in range(TREE_COUNT):
        tree_root = app.render.attachNewNode("tree")
        tree_prototype = random.choice(tree_prototypes)
        tree_model = tree_prototype.instanceTo(tree_root)

        tree_root.setScale(random.uniform(TREE_RANDOM_SCALE_MIN, TREE_RANDOM_SCALE_MAX))

        spawn_result = random_world_position(exclusion_radius=10)
        x = spawn_result[0]
        y = spawn_result[1]
        z = spawn_result[2]
        normal = spawn_result[3]
        tree_root.setPos(x, y, z)
        tree_root.setH(random.uniform(0, 360))

        if using_terrain and normal.z < 0.98:
            tree_root.setP(-normal.x * 8)
            tree_root.setR(normal.y * 8)

        canopy_bounds = tree_prototype.getPythonTag("canopy_local_bounds")
        if canopy_bounds:
            (
                local_min_x,
                local_max_x,
                local_min_y,
                local_max_y,
                local_min_z,
                local_top_z,
            ) = canopy_bounds
            canopy_world_bounds = transform_bounds(
                (
                    Point3(local_min_x, local_min_y, local_min_z),
                    Point3(local_max_x, local_max_y, local_top_z),
                ),
                tree_root.getMat(app.render),
            )
            if canopy_world_bounds is not None:
                world_minimum, world_maximum = canopy_world_bounds
                tree_root.setPythonTag(
                    "canopy_sample",
                    {
                        "min_x": world_minimum.x,
                        "max_x": world_maximum.x,
                        "min_y": world_minimum.y,
                        "max_y": world_maximum.y,
                        "top_z": world_maximum.z,
                    },
                )

        tint = random.uniform(0.94, 1.1)
        tree_model.setColorScale(tint, tint, tint, 1)
        tree_nodes.append(tree_root)
        #stores them in the list

    return tree_nodes


def build_spawned_tree_canopy_samples(tree_nodes):
    canopy_samples = [
        tree_node.getPythonTag("canopy_sample")
        for tree_node in tree_nodes
        if tree_node.hasPythonTag("canopy_sample")
    ]
    if len(canopy_samples) == len(tree_nodes):
        return canopy_samples
    return build_tree_canopy_samples(tree_nodes, app.render)


def spawn_grass(grass_prototypes):
    grass_nodes = []
    if not grass_prototypes:
        return grass_nodes

    for _ in range(GRASS_COUNT):
        patch_spawn = random_world_position(exclusion_radius=GRASS_EXCLUSION_RADIUS)
        patch_x = patch_spawn[0]
        patch_y = patch_spawn[1]
        patch_normal = patch_spawn[3]
        patch_size = random.randint(GRASS_PATCH_CLUMPS_MIN, GRASS_PATCH_CLUMPS_MAX)

        for _ in range(patch_size):
            grass_variant = random.choice(grass_prototypes)
            grass_root = app.render.attachNewNode("grass")
            grass_model = grass_variant["model"].instanceTo(grass_root)

            x = patch_x + random.uniform(-GRASS_PATCH_RADIUS, GRASS_PATCH_RADIUS)
            y = patch_y + random.uniform(-GRASS_PATCH_RADIUS, GRASS_PATCH_RADIUS)
            ground_sample = sample_ground(x, y)

            if ground_sample is None:
                grass_root.removeNode()
                continue

            z = ground_sample[0].z
            normal = ground_sample[1] if using_terrain else patch_normal
            grass_root.setPos(x, y, z)

            if using_terrain:
                align_to_slope(grass_root, normal)
            else:
                grass_root.setHpr(
                    random.uniform(0, 360),
                    random.uniform(-8, 8),
                    random.uniform(-8, 8),
                )

            grass_root.setScale(
                random.uniform(grass_variant["scale_min"], grass_variant["scale_max"])
            )

            tint = random.uniform(grass_variant["tint_min"], grass_variant["tint_max"])
            grass_model.setColorScale(tint, tint, tint, 1)
            grass_nodes.append(grass_root)
            #stores them in the list

    return grass_nodes


TREE_AVOIDANCE_CACHE_CELL_METERS = max(1.0, AUTOMATION_TREE_AVOID_RADIUS_METERS)


def tree_avoidance_cell_coords(x, y):
    return (
        int(x // TREE_AVOIDANCE_CACHE_CELL_METERS),
        int(y // TREE_AVOIDANCE_CACHE_CELL_METERS),
    )


def build_tree_avoidance_spatial_index(tree_positions):
    cells = {}
    for index, (tree_x, tree_y) in enumerate(tree_positions):
        cells.setdefault(tree_avoidance_cell_coords(tree_x, tree_y), []).append(index)
    return cells


def iter_nearby_tree_avoidance_positions(probe_points, radius_meters):
    if not tree_avoidance_samples:
        return
    if not tree_avoidance_cells:
        for tree_position in tree_avoidance_samples:
            yield tree_position
        return

    cell_radius = max(
        1,
        int(ceil(radius_meters / TREE_AVOIDANCE_CACHE_CELL_METERS)),
    )
    seen_indices = set()
    for probe_x, probe_y, _ in probe_points:
        center_cell_x, center_cell_y = tree_avoidance_cell_coords(probe_x, probe_y)
        for offset_x in range(-cell_radius, cell_radius + 1):
            for offset_y in range(-cell_radius, cell_radius + 1):
                cell_key = (center_cell_x + offset_x, center_cell_y + offset_y)
                for tree_index in tree_avoidance_cells.get(cell_key, ()):
                    if tree_index in seen_indices:
                        continue
                    seen_indices.add(tree_index)
                    yield tree_avoidance_samples[tree_index]


def build_drone_propellers(drone_model):
    bounds = drone_model.getTightBounds()
    if bounds is None:
        return []

    min_bound, max_bound = bounds
    span_x = max_bound.x - min_bound.x
    span_y = max_bound.y - min_bound.y
    span_z = max_bound.z - min_bound.z
    center_x = (min_bound.x + max_bound.x) * 0.5
    center_z = (min_bound.z + max_bound.z) * 0.5
    rotor_y = max_bound.y - (span_y * DRONE_PROPELLER_Y_TOP_MARGIN_FACTOR)
    rotor_offset_x = span_x * DRONE_PROPELLER_X_OFFSET_FACTOR
    rotor_offset_z = span_z * DRONE_PROPELLER_Z_OFFSET_FACTOR
    blade_color = (0.08, 0.09, 0.11, DRONE_PROPELLER_BLADE_ALPHA)
    rotor_specs = (
        ((center_x - rotor_offset_x, rotor_y, center_z - rotor_offset_z), 1.0),
        ((center_x + rotor_offset_x, rotor_y, center_z - rotor_offset_z), -1.0),
        ((center_x - rotor_offset_x, rotor_y, center_z + rotor_offset_z), -1.0),
        ((center_x + rotor_offset_x, rotor_y, center_z + rotor_offset_z), 1.0),
    )
    rotor_nodes = []

    for rotor_index, (rotor_position, spin_direction) in enumerate(rotor_specs):
        rotor_root = drone_model.attachNewNode(f"propeller_{rotor_index}")
        rotor_root.setPos(*rotor_position)

        for blade_index, blade_angle in enumerate((0.0, 90.0)):
            blade_maker = CardMaker(f"propeller_blade_{rotor_index}_{blade_index}")
            blade_maker.setFrame(
                -DRONE_PROPELLER_BLADE_HALF_LENGTH,
                DRONE_PROPELLER_BLADE_HALF_LENGTH,
                -DRONE_PROPELLER_BLADE_HALF_WIDTH,
                DRONE_PROPELLER_BLADE_HALF_WIDTH,
            )
            blade = rotor_root.attachNewNode(blade_maker.generate())
            blade.setR(blade_angle)
            blade.setColor(*blade_color)
            blade.setTransparency(TransparencyAttrib.MAlpha)
            blade.setTwoSided(True)

        rotor_nodes.append((rotor_root, spin_direction))

    return rotor_nodes

def build_drone_rig(root_name, tint_rgba=None):
    drone_root = app.render.attachNewNode(root_name)
    drone_visual = drone_root.attachNewNode(f"{root_name}_visual")
    drone_model = app.loader.loadModel(asset_path("models/Drone_Costum/Material/drone_costum.obj"))
    drone_model.reparentTo(drone_visual)
    drone_propellers = build_drone_propellers(drone_model)
    drone_model.setScale(DRONE_MODEL_SCALE)
    drone_model.setP(DRONE_BASE_PITCH_DEGREES)
    drone_model.setH(DRONE_MODEL_HEADING_OFFSET_DEGREES)
    if tint_rgba is not None:
        drone_model.setColorScale(*tint_rgba)
    return drone_root, drone_visual, drone_model, drone_propellers


def build_water_suppression_visual():
    stream_root = app.render.attachNewNode("water_suppression_stream")
    stream_root.hide()

    mist_nodes = []
    for mist_index in range(WATER_DRONE_STREAM_MIST_CARD_COUNT):
        mist_card = CardMaker(f"water_stream_mist_{mist_index}")
        mist_card.setFrame(-0.5, 0.5, -0.5, 0.5)
        mist_node = stream_root.attachNewNode(mist_card.generate())
        mist_node.setTexture(HOTSPOT_GLOW_TEXTURE, 1)
        mist_node.setTransparency(TransparencyAttrib.MAlpha)
        mist_node.setTwoSided(True)
        mist_node.setBillboardPointEye()
        mist_node.setDepthWrite(False)
        mist_node.hide()
        mist_nodes.append(mist_node)

    return stream_root, tuple(mist_nodes)


drone, drone_visual, drone_model, drone_propellers = build_drone_rig(
    "survey_drone",
    SURVEY_DRONE_MODEL_TINT,
)
drone.setPos(*get_fixed_drone_spawn_xy("survey", 1), 2)
water_drone, water_drone_visual, water_drone_model, water_drone_propellers = build_drone_rig(
    "water_drone",
    WATER_DRONE_MODEL_TINT,
)

tree_prototypes = build_tree_prototypes()
grass_prototypes = build_grass_prototypes()
trees = spawn_trees(tree_prototypes)
tree_avoidance_samples = tuple((tree_node.getX(), tree_node.getY()) for tree_node in trees)
tree_avoidance_cells = build_tree_avoidance_spatial_index(tree_avoidance_samples)
tree_canopy_samples = build_spawned_tree_canopy_samples(trees)

start_ground = sample_ground(drone.getX(), drone.getY())
start_altitude = AUTOMATION_CRUISE_ALTITUDE_METERS
if start_ground is not None:
    start_altitude = max(
        start_altitude,
        start_ground[0].z + AUTOMATION_MIN_ALTITUDE_ABOVE_GROUND_METERS,
    )

if AUTOMATION_ENABLE_CANOPY_CLEARANCE:
    start_canopy = compute_local_canopy_height(
        tree_canopy_samples,
        drone.getX(),
        drone.getY(),
        AUTOMATION_CANOPY_QUERY_RADIUS_METERS,
    )
    if start_canopy is not None:
        start_altitude = max(
            start_altitude,
            start_canopy + AUTOMATION_CANOPY_CLEARANCE_METERS,
        )
drone.setZ(start_altitude)

grass_clumps = spawn_grass(grass_prototypes)
fire_truck_states = spawn_fire_truck_fleet()
water_home_x, water_home_y = get_water_drone_home_xy(1)
water_drone_ground = sample_ground(water_home_x, water_home_y)
water_drone_start_z = max(
    WATER_DRONE_IDLE_ALTITUDE_METERS,
    WATER_DRONE_HOME_ALTITUDE_METERS,
)
if water_drone_ground is not None:
    water_drone_start_z = max(
        water_drone_start_z,
        water_drone_ground[0].z + WATER_DRONE_MIN_ALTITUDE_ABOVE_GROUND_METERS,
    )
if tree_canopy_samples:
    water_start_canopy = compute_local_canopy_height(
        tree_canopy_samples,
        water_home_x,
        water_home_y,
        AUTOMATION_CANOPY_QUERY_RADIUS_METERS,
    )
    if water_start_canopy is not None:
        water_drone_start_z = max(
            water_drone_start_z,
            water_start_canopy + WATER_DRONE_CANOPY_CLEARANCE_METERS,
        )
water_drone.setPos(
    water_home_x,
    water_home_y,
    water_drone_start_z,
)
fire_burned_cells = set()
fire_burnable_cells = build_fire_burnable_cells()
# Delayed ignition: the run starts with no fire; scheduled random sources
# ignite later (see maybe_ignite_initial_fire / get_fire_ignition_schedule).
fire_hotspots = []
fire_ignitions_done = 0
# fire_event_id -> survey slot that first detected that fire (fire ownership)
fire_event_owner_slot = {}
initial_burned_cell_count = len(fire_burned_cells)
fire_spread_timer_seconds = 0.0
fire_effect_update_accumulator_seconds = 0.0
fire_map_update_accumulator_seconds = 0.0
dispatch_alert_update_accumulator_seconds = 0.0
status_update_accumulator_seconds = 0.0
burn_eta_update_accumulator_seconds = 0.0
pregame_update_accumulator_seconds = 0.0
headless_frame_counter = 0
frame_pace_last_time = time.perf_counter()
game_over_triggered = False
game_end_reason = None
burn_ratio_history = deque(maxlen=360)
projected_full_burn_eta_seconds = None
incident_ignition_time_seconds = 0.0
incident_first_detection_time_seconds = None


def pace_main_loop():
    global frame_pace_last_time
    if not REAL_SIM_MANUAL_FRAME_PACING:
        frame_pace_last_time = time.perf_counter()
        return

    target_frame_seconds = 1.0 / REAL_SIM_MAX_FPS
    now = time.perf_counter()
    elapsed = now - frame_pace_last_time
    remaining = target_frame_seconds - elapsed
    if remaining > 0.0:
        time.sleep(remaining)
        now = time.perf_counter()
    frame_pace_last_time = now


# -------------------------
# Input and camera behavior
# -------------------------
key_map = {
    "left": False,
    "right": False,
    "forward": False,
    "backward": False,
    "up": False,
    "down": False,
}
REAL_SIM_RADIO_ENABLED = read_bool_env("REAL_SIM_RADIO_ENABLED", True)
TX12_CONTROLLER_DEADZONE = 0.08
TX12_CONTROLLER_YAW_DEGREES_PER_SECOND = 92.0
REAL_SIM_CONTROLLER_DEBUG = read_bool_env("REAL_SIM_CONTROLLER_DEBUG", False)
TX12_CONTROLLER_KILL_CHANNEL = read_nonnegative_int_env(
    "REAL_SIM_TX12_KILL_CHANNEL",
    5,
)
TX12_CONTROLLER_KILL_CHANGE_THRESHOLD = min(
    2.0,
    read_nonnegative_float_env("REAL_SIM_TX12_KILL_CHANGE_THRESHOLD", 0.75),
)
tx12_controller_device = None
tx12_controller_name = ""
tx12_controller_axes = {
    "right": 0.0,
    "forward": 0.0,
    "vertical": 0.0,
    "yaw": 0.0,
}
tx12_controller_was_connected = False
tx12_controller_last_debug_print_time = 0.0
tx12_controller_pressed_buttons = set()
tx12_controller_input_enabled = REAL_SIM_RADIO_ENABLED
tx12_controller_axis_centers = {
    "right": 0.0,
    "forward": 0.0,
    "vertical": 0.0,
    "yaw": 0.0,
}
tx12_controller_channel_baselines = {
    5: 0.0,
}
tx12_controller_channel_kill_active = False
tx12_controller_kill_channel_last_value = None
tx12_controller_recenter_pending = True

#this is a dictionalry that stores wheter each movement is curretly being
#held(like the whatever keys u defined)
camera_angle = 0
camera_pitch = FIRST_PERSON_CAMERA_PITCH_DEGREES
camera_distance = THIRD_PERSON_CAMERA_DISTANCE
camera_height = THIRD_PERSON_CAMERA_HEIGHT
# First person is the default operator view; [V] switches to the chase
# camera and [SPACE] to the map overview.
first_person_view = True
first_person_eye_height = 1.2
camera_target = CAMERA_TARGET_SURVEY
camera_target_index = 0
camera_target_node_override = None
overview_camera_enabled = False
# Two overhead modes share the orbit camera (overview_camera_enabled):
#  - DEV full-info bird view (hidden key [\]): shows EVERYTHING incl. undetected
#    fire. For the developer only.
#  - OPERATOR satellite view ([Tab]): realistic top-down, shows ONLY detected
#    fire; the little detected-fire map hides while it is active.
overview_is_operator = False
overview_camera_angle = -34.0
overview_camera_pitch = OVERVIEW_CAMERA_DEFAULT_PITCH_DEGREES
overview_camera_distance = OVERVIEW_CAMERA_DEFAULT_DISTANCE
control_mode = (
    CONTROL_MODE_AUTOMATION
    if AUTOMATION_STARTS_IN_AUTOMATION
    else CONTROL_MODE_MANUAL
)
water_control_mode = CONTROL_MODE_MANUAL
current_speed_preset = AUTOMATION_START_SPEED_PRESET
team_total_drone_count = TEAM_TOTAL_DRONE_COUNT_DEFAULT
team_water_drone_count = TEAM_WATER_DRONE_COUNT_DEFAULT
pregame_active = True
round_restart_pending_from_setup = False
pregame_root = None
pregame_backdrop_np = None
pregame_accent_np = None
pregame_title = None
pregame_text = None
follower_drones = []
automation_target_hotspot = None
automation_search_waypoint = None
survey_lead_patrol_sector_label = None
survey_lead_patrol_waypoints = tuple()
survey_lead_patrol_waypoint_index = 0
survey_lead_waypoint_timer_seconds = 0.0

dragging_camera = False
last_mouse_x = 0
last_mouse_y = 0
camera_manual_override_active = False
camera_yaw_offset_degrees = 0.0


@dataclass
class AxisState:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class DroneMotionState:
    drone_role: str = "survey"
    mass_kg: float = SURVEY_DRONE_MASS_KG
    inertia_xx_kg_m2: float = SURVEY_DRONE_INERTIA_XX_KG_M2
    inertia_yy_kg_m2: float = SURVEY_DRONE_INERTIA_YY_KG_M2
    inertia_zz_kg_m2: float = SURVEY_DRONE_INERTIA_ZZ_KG_M2
    min_collective_thrust_newtons: float = DRONE_MIN_COLLECTIVE_THRUST_NEWTONS
    max_collective_thrust_newtons: float = DRONE_MAX_COLLECTIVE_THRUST_NEWTONS
    hover_thrust_per_rotor_newtons: float = DRONE_HOVER_THRUST_PER_ROTOR_NEWTONS
    hover_rotor_omega_rad_per_second: float = DRONE_HOVER_ROTOR_OMEGA_RAD_PER_SECOND
    sim_time_seconds: float = 0.0
    input_command_buffer: deque = field(default_factory=deque)
    applied_cmd: AxisState = field(default_factory=AxisState)
    velocity: AxisState = field(default_factory=AxisState)
    pitch_radians: float = 0.0
    roll_radians: float = 0.0
    yaw_radians: float = 0.0
    pitch_rate_rad_per_second: float = 0.0
    roll_rate_rad_per_second: float = 0.0
    yaw_rate_rad_per_second: float = 0.0
    heading_hold_radians: float | None = None
    target_altitude_meters: float | None = None
    visual_pitch: float = 0.0
    visual_roll: float = 0.0
    propeller_spin_degrees: float = 0.0
    rotor_omegas_rad_per_second: list[float] = field(
        default_factory=lambda: [DRONE_HOVER_ROTOR_OMEGA_RAD_PER_SECOND] * 4
    )
    rotor_thrusts_newtons: list[float] = field(
        default_factory=lambda: [DRONE_HOVER_THRUST_PER_ROTOR_NEWTONS] * 4
    )
    last_body_right_speed: float = 0.0
    filtered_body_right_accel: float = 0.0
    roll_sway_body_speed: float = 0.0
    last_body_right_command_speed: float = 0.0
    roll_anticipation_body_speed: float = 0.0
    vertical_command_was_active: bool = False


def apply_drone_physical_parameters(drone_motion_state, drone_role):
    """Set per-type physics on a motion state.

    Survey and water drones differ by mass and inertia matrix only; thrust
    coefficient, arm length, and motor dynamics are shared.
    """
    if drone_role == "water":
        mass_kg = WATER_DRONE_MASS_KG
        inertia_xx = WATER_DRONE_INERTIA_XX_KG_M2
        inertia_yy = WATER_DRONE_INERTIA_YY_KG_M2
        inertia_zz = WATER_DRONE_INERTIA_ZZ_KG_M2
    else:
        drone_role = "survey"
        mass_kg = SURVEY_DRONE_MASS_KG
        inertia_xx = SURVEY_DRONE_INERTIA_XX_KG_M2
        inertia_yy = SURVEY_DRONE_INERTIA_YY_KG_M2
        inertia_zz = SURVEY_DRONE_INERTIA_ZZ_KG_M2
    weight_newtons = mass_kg * DRONE_GRAVITY_METERS_PER_SECOND_SQ
    hover_thrust_per_rotor = weight_newtons / 4.0
    drone_motion_state.drone_role = drone_role
    drone_motion_state.mass_kg = mass_kg
    drone_motion_state.inertia_xx_kg_m2 = inertia_xx
    drone_motion_state.inertia_yy_kg_m2 = inertia_yy
    drone_motion_state.inertia_zz_kg_m2 = inertia_zz
    drone_motion_state.min_collective_thrust_newtons = weight_newtons * 0.35
    drone_motion_state.max_collective_thrust_newtons = weight_newtons * 2.4
    drone_motion_state.hover_thrust_per_rotor_newtons = hover_thrust_per_rotor
    drone_motion_state.hover_rotor_omega_rad_per_second = (
        hover_thrust_per_rotor / DRONE_ROTOR_THRUST_COEFFICIENT
    ) ** 0.5
    drone_motion_state.rotor_omegas_rad_per_second = [
        drone_motion_state.hover_rotor_omega_rad_per_second
    ] * 4
    drone_motion_state.rotor_thrusts_newtons = [hover_thrust_per_rotor] * 4
    return drone_motion_state


motion_state = DroneMotionState()
apply_drone_physical_parameters(motion_state, "survey")


@dataclass
class TeamCoordinationState:
    algorithm_mode: str = TEAM_ALGORITHM_ROLLING_PATROL
    rolling_patrol_timer_seconds: float = 0.0
    survey_orbit_phase_radians: float = 0.0
    survey_focus_hotspot: object | None = None
    water_target_hotspot: object | None = None
    pending_water_dispatch_hotspot: object | None = None


team_state = TeamCoordinationState()


@dataclass
class SurveySearchSector:
    slot: int
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    label: str

    def center(self):
        return (self.x_min + self.x_max) * 0.5, (self.y_min + self.y_max) * 0.5

    def contains(self, x, y):
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max


@dataclass
class WaterDroneState:
    home_x: float = 0.0
    home_y: float = 0.0
    home_z: float = WATER_DRONE_IDLE_ALTITUDE_METERS
    stream_root: object | None = None
    stream_outer_node: object | None = None
    stream_inner_node: object | None = None
    mist_nodes: tuple[object, ...] = field(default_factory=tuple)
    tracked_hotspot_id: int | None = None
    hover_offset_angle_radians: float = 0.0
    mission_active: bool = False
    water_liters: float = WATER_DRONE_TANK_CAPACITY_LITERS
    # When a fire is detected, the operator gets the dispatch message and
    # calls the FIRST water drone with [P]; the second water drone is called
    # separately with [L].
    auto_dispatch_enabled: bool = False


water_drone_state = WaterDroneState()
water_drone_state.home_x = water_drone.getX()
water_drone_state.home_y = water_drone.getY()
water_drone_state.home_z = water_drone.getZ()
(
    water_drone_state.stream_root,
    water_drone_state.mist_nodes,
) = build_water_suppression_visual()
water_motion_state = DroneMotionState()
apply_drone_physical_parameters(water_motion_state, "water")

fire_map_panel = None
fire_map_view_root = None
fire_map_marker_root = None
fire_map_header_text = None
fire_map_prompt_text = None
fire_map_status_text = None
fire_map_detected_hotspot_nodes = {}
fire_map_drone_nodes = {}
fire_map_truck_nodes = {}
fire_map_wind_arrow_root = None
fire_map_wind_label = None
fire_map_trajectory_root = None
fire_map_expanded = False
fire_map_size_button_frame = None
fire_map_size_button_icon_root = None
survey_map_marker_node = None
survey_map_ring_node = None
survey_map_label = None
water_map_marker_node = None
water_map_ring_node = None
water_map_label = None
performance_panel_root = None
performance_tab_text = None
performance_score_text = None
performance_meter_fill = None
performance_graph_root = None
performance_graph_line_node = None
performance_metrics_hidden = False
performance_toggle_button_root = None
performance_toggle_button_card = None
performance_toggle_button_text = None
performance_toggle_button_frame = None
performance_history = deque(maxlen=PERFORMANCE_HISTORY_MAX_SAMPLES)
operator_lifetime_round_count = 0
operator_lifetime_success_count = 0
operator_lifetime_score_sum = 0.0
performance_round_recorded = False
dispatch_alert_root = None
dispatch_alert_backdrop = None
dispatch_alert_text = None
dispatch_alert_body_text = None
dispatch_alert_hotspot = None
dispatch_alert_flash_timer_seconds = 0.0
dispatch_prompt_cycle_active = False
dispatch_prompt_has_shown_this_round = False
automation_message_root = None
automation_message_backdrop = None
automation_message_text = None
automation_message_header_text = None
automation_message_status_text = None
automation_message_survey_text = None
automation_message_water_text = None
automation_message_accent_bar = None
water_tank_status_root = None
water_tank_status_fill = None
water_tank_status_text = None
water_tank_status_rows = []
collision_alert_root = None
collision_alert_backdrop = None
collision_alert_text = None
collision_alert_body_text = None


@dataclass
class WindState:
    base_direction_degrees: float = WIND_BASE_DIRECTION_DEGREES
    base_speed_mps: float = WIND_BASE_SPEED_METERS_PER_SECOND
    gust_direction_degrees: float = 0.0
    gust_speed_mps: float = 0.0
    target_gust_direction_degrees: float = 0.0
    target_gust_speed_mps: float = 0.0
    gust_hold_timer_seconds: float = 0.0


wind_state = WindState()


def _normalize_signed_degrees(angle_degrees):
    return (angle_degrees + 180.0) % 360.0 - 180.0


def _shortest_angle_delta_degrees(current_degrees, target_degrees):
    return _normalize_signed_degrees(target_degrees - current_degrees)


def approach_angle_degrees(current_degrees, target_degrees, rate_per_second, dt):
    delta_degrees = _shortest_angle_delta_degrees(current_degrees, target_degrees)
    max_step = rate_per_second * dt
    if abs(delta_degrees) <= max_step:
        return current_degrees + delta_degrees
    return current_degrees + max_step if delta_degrees > 0.0 else current_degrees - max_step


def apply_axis_drag(
    current_velocity,
    commanded_velocity,
    dt,
    max_command_speed,
    active_drag_per_second,
    passive_drag_per_second,
):
    speed = abs(current_velocity)
    if speed <= 0.0001:
        return 0.0

    throttle_ratio = 0.0
    if max_command_speed > 0.0001:
        throttle_ratio = min(1.0, abs(commanded_velocity) / max_command_speed)

    drag_rate = passive_drag_per_second + (
        (active_drag_per_second - passive_drag_per_second) * throttle_ratio
    )
    drag_step = speed * drag_rate * dt
    if drag_step >= speed:
        return 0.0
    return current_velocity - copysign(drag_step, current_velocity)


def low_pass_value(current_value, target_value, dt, time_constant_seconds):
    if time_constant_seconds <= 0.0:
        return target_value
    blend = 1.0 - exp(-dt / time_constant_seconds)
    return current_value + (target_value - current_value) * blend


def clamp(value, minimum_value, maximum_value):
    return max(minimum_value, min(maximum_value, value))


def get_team_survey_drone_count():
    return max(0, team_total_drone_count - team_water_drone_count)


def set_team_water_drone_count(new_water_count):
    global team_water_drone_count
    if not pregame_active:
        return
    # Keep at least one of each role: the two lead drones are 1 survey + 1 water.
    lower_bound = 1 if team_total_drone_count >= 2 else 0
    upper_bound = max(lower_bound, team_total_drone_count - 1)
    team_water_drone_count = int(
        clamp(new_water_count, lower_bound, upper_bound)
    )
    if "status_text" in globals():
        update_status_overlay()
    if fire_map_panel is not None:
        update_fire_map_popup()


def set_team_total_drone_count(new_total):
    global team_total_drone_count, team_water_drone_count
    if not pregame_active:
        return
    previous_total = max(2, team_total_drone_count)
    previous_water_fraction = team_water_drone_count / previous_total
    new_total = int(new_total)
    if new_total not in TEAM_TOTAL_DRONE_OPTIONS:
        new_total = min(
            TEAM_TOTAL_DRONE_OPTIONS,
            key=lambda option: abs(option - new_total),
        )
    team_total_drone_count = new_total
    # Preserve the operator's chosen ratio as total team size changes.
    team_water_drone_count = int(
        clamp(
            round(team_total_drone_count * previous_water_fraction),
            1,
            team_total_drone_count - 1,
        )
    )
    if "status_text" in globals():
        update_status_overlay()
    if fire_map_panel is not None:
        update_fire_map_popup()


def cycle_total_drone_count():
    if not pregame_active:
        return
    options = TEAM_TOTAL_DRONE_OPTIONS
    try:
        current_index = options.index(team_total_drone_count)
    except ValueError:
        current_index = 0
    set_team_total_drone_count(options[(current_index + 1) % len(options)])
    update_pregame_overlay()


def choose_more_survey_drones():
    if not pregame_active:
        return
    set_team_water_drone_count(team_water_drone_count - 1)
    update_pregame_overlay()


def choose_more_water_drones():
    if not pregame_active:
        return
    set_team_water_drone_count(team_water_drone_count + 1)
    update_pregame_overlay()


def format_team_ratio_menu():
    if pregame_active:
        return (
            f"TEAM: {team_total_drone_count} drones"
            f" | Survey {get_team_survey_drone_count()} / Water {team_water_drone_count}"
            " | [G] total 2/4/6 | [1] +survey | [2] +water"
        )
    return (
        f"TEAM: {team_total_drone_count} drones"
        f" | Survey {get_team_survey_drone_count()} / Water {team_water_drone_count}"
        " | [1-6] views | map click any | [M/O] manual/auto | [H] controls"
    )


def team_role_effectiveness(role_drone_count):
    if role_drone_count <= 0:
        return TEAM_ROLE_MIN_EFFECTIVENESS
    return clamp(
        1.0 + (
            TEAM_ROLE_EXTRA_DRONE_EFFECTIVENESS
            * max(0, role_drone_count - 1)
        ),
        TEAM_ROLE_MIN_EFFECTIVENESS,
        TEAM_ROLE_MAX_EFFECTIVENESS,
    )


def survey_detection_probability_scale():
    if get_team_survey_drone_count() >= 3:
        return 1.0
    return SMALL_TEAM_DETECTION_PROBABILITY_SCALE


def survey_forced_detection_radius_scale():
    if get_team_survey_drone_count() >= 3:
        return 1.0
    return SMALL_TEAM_FORCED_DETECTION_RADIUS_SCALE


def team_extra_drone_score_bonus():
    extra_drone_count = max(0, team_total_drone_count - 2)
    return min(
        TEAM_EXTRA_DRONE_MAX_SCORE_BONUS,
        extra_drone_count * TEAM_EXTRA_DRONE_SCORE_BONUS,
    )


def iter_active_follower_drones(role=None):
    for follower in follower_drones:
        if role is not None and follower.get("role") != role:
            continue
        rig_root = follower.get("root")
        if rig_root is None or rig_root.isEmpty():
            continue
        yield follower


def get_survey_drone_root_control_mode(survey_root):
    if survey_root is drone:
        return control_mode
    for follower in iter_active_follower_drones("survey"):
        if follower["root"] is survey_root:
            return follower.get("control_mode", CONTROL_MODE_AUTOMATION)
    return CONTROL_MODE_AUTOMATION


def get_survey_slot_for_root(survey_root):
    if survey_root is drone:
        return 1
    for follower in iter_active_follower_drones("survey"):
        if follower["root"] is survey_root:
            return max(1, follower.get("slot", 1) + 1)
    return None


def get_survey_follower_for_slot(survey_slot):
    for follower in iter_active_follower_drones("survey"):
        if max(1, follower.get("slot", 1) + 1) == survey_slot:
            return follower
    return None


def survey_follower_is_actively_scanning(follower):
    if follower is None or follower.get("role") != "survey":
        return False
    control_mode_for_follower = follower.get("control_mode", CONTROL_MODE_AUTOMATION)
    if control_mode_for_follower == CONTROL_MODE_AUTOMATION:
        return bool(follower.get("automation_started", False))
    return control_mode_for_follower == CONTROL_MODE_MANUAL


def get_active_survey_drone_roots():
    survey_roots = []
    if (
        not is_drone_damaged("survey", 1)
        and control_mode in (CONTROL_MODE_MANUAL, CONTROL_MODE_AUTOMATION)
    ):
        survey_roots.append(drone)
    survey_roots.extend(
        follower["root"]
        for follower in iter_active_follower_drones("survey")
        if not is_drone_damaged("survey", max(1, follower.get("slot", 1) + 1))
        and survey_follower_is_actively_scanning(follower)
    )
    return survey_roots


def prime_survey_fire_response(survey_slot, hotspot):
    """Switch the detecting survey drone from its sweep leg into fire response."""
    global automation_target_hotspot, automation_search_waypoint
    global survey_lead_waypoint_timer_seconds

    if hotspot is None or survey_slot is None:
        return
    survey_slot = int(survey_slot)
    if survey_slot == 1:
        if control_mode == CONTROL_MODE_AUTOMATION:
            automation_target_hotspot = hotspot
            automation_search_waypoint = None
            survey_lead_waypoint_timer_seconds = 0.0
        return

    follower = get_survey_follower_for_slot(survey_slot)
    if follower is None:
        return
    if follower.get("control_mode", CONTROL_MODE_AUTOMATION) == CONTROL_MODE_AUTOMATION:
        follower["automation_started"] = True
    follower["target_hotspot"] = hotspot
    follower["search_waypoint"] = None
    follower["patrol_waypoint_timer_seconds"] = 0.0
    follower["orbit_hotspot_id"] = None


def reset_hotspot_water_drone_contact():
    for hotspot in fire_hotspots:
        hotspot.water_drone_assigned = False
        hotspot.water_drone_engaged = False
        hotspot.water_drone_assignment_count = 0
        hotspot.water_drone_engagement_count = 0


def register_hotspot_water_drone_contact(hotspot, engaged=False):
    if hotspot is None:
        return
    hotspot.water_drone_assigned = True
    hotspot.water_drone_assignment_count += 1
    if engaged:
        hotspot.water_drone_engaged = True
        hotspot.water_drone_engagement_count += 1


def build_survey_search_sectors():
    survey_count = max(1, get_team_survey_drone_count())
    sector_width = (SPAWN_X_MAX - SPAWN_X_MIN) / survey_count
    sectors = []
    for index in range(survey_count):
        x_min = SPAWN_X_MIN + (sector_width * index)
        x_max = SPAWN_X_MAX if index == survey_count - 1 else x_min + sector_width
        sectors.append(
            SurveySearchSector(
                slot=index + 1,
                x_min=x_min,
                x_max=x_max,
                y_min=SPAWN_Y_MIN,
                y_max=SPAWN_Y_MAX,
                label=chr(ord("A") + index),
            )
        )
    return sectors


def get_survey_search_sector(slot):
    sectors = build_survey_search_sectors()
    if not sectors:
        return None
    slot_index = int(clamp(slot - 1, 0, len(sectors) - 1))
    return sectors[slot_index]


def get_hotspot_survey_sector_slot(hotspot):
    sectors = build_survey_search_sectors()
    if not sectors:
        return 1
    hotspot_x = hotspot.root.getX()
    hotspot_y = hotspot.root.getY()
    for sector in sectors:
        if sector.contains(hotspot_x, hotspot_y):
            return sector.slot
    nearest_sector = min(
        sectors,
        key=lambda sector: (
            (sector.center()[0] - hotspot_x) ** 2
            + (sector.center()[1] - hotspot_y) ** 2
        ),
    )
    return nearest_sector.slot


def get_hotspot_primary_survey_slot(hotspot):
    detected_slot = getattr(hotspot, "detected_by_survey_slot", None)
    survey_count = max(1, get_team_survey_drone_count())
    if detected_slot is not None:
        return int(clamp(detected_slot, 1, survey_count))
    return get_hotspot_survey_sector_slot(hotspot)


def choose_survey_sector_spawn(sector, preferred_positions=None):
    if sector is None:
        return None

    center_x, center_y = sector.center()
    margin_x = min(
        SURVEY_SECTOR_MARGIN_METERS,
        max(2.0, (sector.x_max - sector.x_min) * 0.22),
    )
    margin_y = min(
        SURVEY_SECTOR_MARGIN_METERS,
        max(2.0, (sector.y_max - sector.y_min) * 0.12),
    )
    x_min = sector.x_min + margin_x
    x_max = sector.x_max - margin_x
    y_min = sector.y_min + margin_y
    y_max = sector.y_max - margin_y
    if x_max <= x_min:
        x_min = x_max = center_x
    if y_max <= y_min:
        y_min = y_max = center_y

    candidates = []
    if preferred_positions:
        for candidate_x, candidate_y in preferred_positions:
            candidates.append(
                (
                    clamp(candidate_x, x_min, x_max),
                    clamp(candidate_y, y_min, y_max),
                )
            )
    candidates.extend(
        (
            (x_min, y_min),
            (x_min, y_max),
            (x_max, y_min),
            (x_max, y_max),
            (center_x, center_y),
        )
    )

    for candidate_x, candidate_y in candidates:
        if not _is_position_clear_of_trees(
            candidate_x,
            candidate_y,
            SURVEY_SECTOR_START_CLEARANCE_METERS,
        ):
            continue
        ground_sample = sample_ground(candidate_x, candidate_y)
        if ground_sample is None:
            continue
        return patrol_waypoint_with_safe_altitude(
            (candidate_x, candidate_y, ground_sample[0].z)
        )

    for _ in range(120):
        candidate_x = random.uniform(x_min, x_max)
        candidate_y = random.uniform(y_min, y_max)
        if not _is_position_clear_of_trees(
            candidate_x,
            candidate_y,
            SURVEY_SECTOR_START_CLEARANCE_METERS,
        ):
            continue
        ground_sample = sample_ground(candidate_x, candidate_y)
        if ground_sample is None:
            continue
        return patrol_waypoint_with_safe_altitude(
            (candidate_x, candidate_y, ground_sample[0].z)
        )

    ground_sample = sample_ground(center_x, center_y)
    center_z = ground_sample[0].z if ground_sample is not None else 0.0
    return patrol_waypoint_with_safe_altitude((center_x, center_y, center_z))


def choose_survey_sector_spawn_for_slot(slot, preferred_positions=None):
    return choose_survey_sector_spawn(
        get_survey_search_sector(slot),
        preferred_positions=preferred_positions,
    )


def build_survey_sector_waypoints(sector):
    if sector is None:
        return tuple()
    margin_x = min(
        SURVEY_SECTOR_MARGIN_METERS,
        max(2.0, (sector.x_max - sector.x_min) * 0.18),
    )
    margin_y = min(
        SURVEY_SECTOR_MARGIN_METERS,
        max(3.0, (sector.y_max - sector.y_min) * 0.08),
    )
    start_x = sector.x_min + margin_x
    end_x = sector.x_max - margin_x
    start_y = sector.y_min + margin_y
    end_y = sector.y_max - margin_y

    if end_x <= start_x:
        center_x, _ = sector.center()
        start_x = center_x
        end_x = center_x
    if end_y <= start_y:
        _, center_y = sector.center()
        start_y = center_y
        end_y = center_y

    sweep_x_positions = []
    sweep_x = start_x
    while sweep_x <= end_x + 0.001:
        sweep_x_positions.append(sweep_x)
        sweep_x += SURVEY_SECTOR_SWEEP_SPACING_METERS
    if not sweep_x_positions:
        sweep_x_positions.append((start_x + end_x) * 0.5)
    elif abs(sweep_x_positions[-1] - end_x) > SURVEY_SECTOR_SWEEP_SPACING_METERS * 0.4:
        sweep_x_positions.append(end_x)

    waypoints = []
    current_y = start_y
    moving_up = True
    for sweep_x in sweep_x_positions:
        waypoints.append((sweep_x, current_y, 0.0))
        target_y = end_y if moving_up else start_y
        waypoints.append((sweep_x, target_y, 0.0))
        current_y = target_y
        moving_up = not moving_up
    return tuple(waypoints)


def rotate_waypoints_to_nearest(waypoints, reference_x, reference_y):
    if not waypoints:
        return tuple()
    closest_index = min(
        range(len(waypoints)),
        key=lambda waypoint_index: (
            (waypoints[waypoint_index][0] - reference_x) ** 2
            + (waypoints[waypoint_index][1] - reference_y) ** 2
        ),
    )
    return tuple(waypoints[closest_index:] + waypoints[:closest_index])


def patrol_waypoint_with_safe_altitude(waypoint):
    waypoint_x, waypoint_y, waypoint_z = waypoint
    return (
        waypoint_x,
        waypoint_y,
        compute_safe_altitude_at(
            waypoint_x,
            waypoint_y,
            max(AUTOMATION_CRUISE_ALTITUDE_METERS, waypoint_z),
            AUTOMATION_MIN_ALTITUDE_ABOVE_GROUND_METERS,
            canopy_clearance_meters=(
                AUTOMATION_CANOPY_CLEARANCE_METERS
                if AUTOMATION_ENABLE_CANOPY_CLEARANCE
                else 0.0
            ),
        ),
    )


def is_search_waypoint_exhausted(waypoint):
    if waypoint is None:
        return True
    waypoint_cell = world_to_fire_cell(waypoint[0], waypoint[1])
    # Do not let hidden fire alter the S-pattern. A waypoint is "exhausted"
    # only when the operator/automation already knows about fire in that cell.
    for hotspot in fire_hotspots:
        if hotspot.root.isEmpty():
            continue
        if world_to_fire_cell(hotspot.root.getX(), hotspot.root.getY()) != waypoint_cell:
            continue
        if hotspot.detected or hotspot.suppression_state in (FIRE_STATE_OUT, FIRE_STATE_BURNED):
            return True
    return False


def performance_graph_y_fraction(score):
    score = clamp(score, 0.0, 1.0)
    log_strength = max(0.0, PERFORMANCE_GRAPH_LOG_STRENGTH)
    if log_strength <= 0.0:
        return score
    return log1p(score * log_strength) / log1p(log_strength)


def wrap_angle_radians(angle_radians):
    return (angle_radians + (tau * 0.5)) % tau - (tau * 0.5)


def wrap_angle_degrees(angle_degrees):
    return (angle_degrees + 180.0) % 360.0 - 180.0


def bounded_guidance(error_value):
    return error_value / (1.0 + abs(error_value))


def body_axes_from_yaw(yaw_radians):
    cos_yaw = cos(yaw_radians)
    sin_yaw = sin(yaw_radians)
    body_right = (cos_yaw, sin_yaw)
    body_forward = (-sin_yaw, cos_yaw)
    return body_right, body_forward


# Thrust mixing for an X-configuration quadrotor, following
# M. Faessler, D. Falanga, D. Scaramuzza, "Thrust Mixing, Saturation, and
# Body-Rate Control for Accurate Aggressive Quadrotor Flight",
# IEEE RA-L 2(2), 2017 (UTIAS slide via Longhao).
#   kappa = C_Q / C_T, so each rotor's reaction torque is tau_i = kappa * f_i.
#   [f, tau_x, tau_y, tau_z]^T = K [f1, f2, f3, f4]^T with
#   K row 2 = (sqrt(2)/2) l [ 1, -1, -1,  1 ]   (tau_x, roll, x forward)
#   K row 3 = (sqrt(2)/2) l [-1, -1,  1,  1 ]   (tau_y, pitch, y left)
#   K row 4 = kappa        [ 1, -1,  1, -1 ]    (tau_z, yaw)
# Rotor order: f1 front-left, f2 front-right, f3 rear-right, f4 rear-left.
# Sim torque conventions kept: roll_torque = tau_x; pitch_torque (positive =
# nose up) = -tau_y; yaw_torque = tau_z.
SQRT2_OVER_2 = (2.0 ** 0.5) / 2.0


def mix_controls_to_rotor_thrusts(
    collective_thrust_newtons,
    pitch_torque_nm,
    roll_torque_nm,
    yaw_torque_nm,
):
    lever = max(0.001, SQRT2_OVER_2 * DRONE_ARM_LENGTH_METERS)
    kappa = max(0.0001, DRONE_ROTOR_YAW_MOMENT_RATIO)

    tau_x = roll_torque_nm
    tau_y = -pitch_torque_nm
    tau_z = yaw_torque_nm

    quarter_collective = collective_thrust_newtons * 0.25
    roll_term = tau_x / (4.0 * lever)
    pitch_term = tau_y / (4.0 * lever)
    yaw_term = tau_z / (4.0 * kappa)

    thrust_1 = quarter_collective + roll_term - pitch_term + yaw_term
    thrust_2 = quarter_collective - roll_term - pitch_term - yaw_term
    thrust_3 = quarter_collective - roll_term + pitch_term + yaw_term
    thrust_4 = quarter_collective + roll_term + pitch_term - yaw_term

    return [
        max(0.0, thrust_1),
        max(0.0, thrust_2),
        max(0.0, thrust_3),
        max(0.0, thrust_4),
    ]


def rotor_thrusts_to_control_inputs(rotor_thrusts_newtons):
    thrust_1, thrust_2, thrust_3, thrust_4 = rotor_thrusts_newtons
    lever = SQRT2_OVER_2 * DRONE_ARM_LENGTH_METERS
    kappa = DRONE_ROTOR_YAW_MOMENT_RATIO
    collective_thrust = thrust_1 + thrust_2 + thrust_3 + thrust_4
    tau_x = lever * (thrust_1 - thrust_2 - thrust_3 + thrust_4)
    tau_y = lever * (-thrust_1 - thrust_2 + thrust_3 + thrust_4)
    tau_z = kappa * (thrust_1 - thrust_2 + thrust_3 - thrust_4)
    pitch_torque_nm = -tau_y
    roll_torque_nm = tau_x
    yaw_torque_nm = tau_z
    return collective_thrust, pitch_torque_nm, roll_torque_nm, yaw_torque_nm


def update_rotor_dynamics_for_state(
    drone_motion_state,
    desired_rotor_thrusts_newtons,
    dt,
):
    actual_rotor_thrusts = []
    updated_omegas = []
    for rotor_index, desired_thrust in enumerate(desired_rotor_thrusts_newtons):
        desired_omega = (
            max(0.0, desired_thrust) / DRONE_ROTOR_THRUST_COEFFICIENT
        ) ** 0.5
        current_omega = drone_motion_state.rotor_omegas_rad_per_second[rotor_index]
        updated_omega = low_pass_value(
            current_omega,
            desired_omega,
            dt,
            DRONE_MOTOR_TIME_CONSTANT_SECONDS,
        )
        updated_omegas.append(updated_omega)
        actual_rotor_thrusts.append(
            DRONE_ROTOR_THRUST_COEFFICIENT * updated_omega * updated_omega
        )

    drone_motion_state.rotor_omegas_rad_per_second = updated_omegas
    drone_motion_state.rotor_thrusts_newtons = actual_rotor_thrusts
    return actual_rotor_thrusts


def update_rotor_dynamics(desired_rotor_thrusts_newtons, dt):
    return update_rotor_dynamics_for_state(
        motion_state,
        desired_rotor_thrusts_newtons,
        dt,
    )


def initialize_drone_motion_state_from_pose(drone_root, drone_motion_state):
    drone_motion_state.yaw_radians = radians(drone_root.getH())
    drone_motion_state.heading_hold_radians = drone_motion_state.yaw_radians
    drone_motion_state.pitch_radians = 0.0
    drone_motion_state.roll_radians = 0.0
    drone_motion_state.pitch_rate_rad_per_second = 0.0
    drone_motion_state.roll_rate_rad_per_second = 0.0
    drone_motion_state.yaw_rate_rad_per_second = 0.0
    drone_motion_state.target_altitude_meters = drone_root.getZ()
    drone_motion_state.visual_pitch = 0.0
    drone_motion_state.visual_roll = 0.0
    drone_motion_state.propeller_spin_degrees = 0.0
    drone_motion_state.velocity = AxisState()
    drone_motion_state.applied_cmd = AxisState()
    drone_motion_state.input_command_buffer.clear()
    drone_motion_state.rotor_omegas_rad_per_second = [
        drone_motion_state.hover_rotor_omega_rad_per_second
    ] * 4
    drone_motion_state.rotor_thrusts_newtons = [
        drone_motion_state.hover_thrust_per_rotor_newtons
    ] * 4
    drone_motion_state.vertical_command_was_active = False
    drone_motion_state.last_body_right_speed = 0.0
    drone_motion_state.filtered_body_right_accel = 0.0
    drone_motion_state.roll_sway_body_speed = 0.0
    drone_motion_state.last_body_right_command_speed = 0.0
    drone_motion_state.roll_anticipation_body_speed = 0.0


def get_drone_heading_hold_radians(drone_motion_state):
    if drone_motion_state.heading_hold_radians is None:
        drone_motion_state.heading_hold_radians = drone_motion_state.yaw_radians
    return drone_motion_state.heading_hold_radians


def set_drone_heading_hold_degrees(drone_motion_state, heading_degrees):
    drone_motion_state.heading_hold_radians = radians(
        wrap_angle_degrees(heading_degrees)
    )


def initialize_motion_state_from_drone_pose():
    initialize_drone_motion_state_from_pose(drone, motion_state)


def update_guided_drone_motion(
    drone_root,
    drone_visual_root,
    drone_propellers,
    drone_motion_state,
    desired_world_vx,
    desired_world_vy,
    target_altitude_meters,
    horizontal_speed_limit,
    vertical_speed_limit,
    dt,
    wind_command_blend=DRONE_WIND_COMMAND_BLEND_AUTOMATION,
    apply_drone_avoidance=True,
    target_heading_degrees=None,
    visual_tilt_scale=1.0,
    visual_tilt_time_constant_seconds=0.0,
    max_tilt_radians=None,
):
    if max_tilt_radians is None:
        max_tilt_radians = get_current_max_tilt_radians()

    if apply_drone_avoidance:
        avoid_vx, avoid_vy = compute_drone_avoidance_velocity(
            drone_root,
            drone_root.getX(),
            drone_root.getY(),
            desired_world_vx,
            desired_world_vy,
            horizontal_speed_limit,
        )
        desired_world_vx += avoid_vx
        desired_world_vy += avoid_vy
        desired_speed = (desired_world_vx * desired_world_vx + desired_world_vy * desired_world_vy) ** 0.5
        if desired_speed > horizontal_speed_limit and desired_speed > 0.001:
            desired_world_vx = (desired_world_vx / desired_speed) * horizontal_speed_limit
            desired_world_vy = (desired_world_vy / desired_speed) * horizontal_speed_limit

    wind_vx, wind_vy, _, _ = get_wind_velocity()
    commanded_vx = desired_world_vx + wind_vx * wind_command_blend
    commanded_vy = desired_world_vy + wind_vy * wind_command_blend
    drone_motion_state.applied_cmd.x = desired_world_vx
    drone_motion_state.applied_cmd.y = desired_world_vy
    drone_motion_state.applied_cmd.z = 0.0
    drone_motion_state.target_altitude_meters = target_altitude_meters

    body_right, body_forward = body_axes_from_yaw(drone_motion_state.yaw_radians)
    desired_body_right_speed = commanded_vx * body_right[0] + commanded_vy * body_right[1]
    desired_body_forward_speed = commanded_vx * body_forward[0] + commanded_vy * body_forward[1]
    current_body_right_speed = (
        drone_motion_state.velocity.x * body_right[0]
        + drone_motion_state.velocity.y * body_right[1]
    )
    current_body_forward_speed = (
        drone_motion_state.velocity.x * body_forward[0]
        + drone_motion_state.velocity.y * body_forward[1]
    )

    desired_body_right_accel = clamp(
        (desired_body_right_speed - current_body_right_speed) * DRONE_HORIZONTAL_VELOCITY_GAIN,
        -DRONE_MAX_HORIZONTAL_ACCEL_METERS_PER_SECOND_SQ,
        DRONE_MAX_HORIZONTAL_ACCEL_METERS_PER_SECOND_SQ,
    )
    desired_body_forward_accel = clamp(
        (desired_body_forward_speed - current_body_forward_speed) * DRONE_HORIZONTAL_VELOCITY_GAIN,
        -DRONE_MAX_HORIZONTAL_ACCEL_METERS_PER_SECOND_SQ,
        DRONE_MAX_HORIZONTAL_ACCEL_METERS_PER_SECOND_SQ,
    )

    desired_roll_radians = clamp(
        desired_body_right_accel / DRONE_GRAVITY_METERS_PER_SECOND_SQ,
        -max_tilt_radians,
        max_tilt_radians,
    )
    desired_pitch_radians = clamp(
        -desired_body_forward_accel / DRONE_GRAVITY_METERS_PER_SECOND_SQ,
        -max_tilt_radians,
        max_tilt_radians,
    )

    if target_heading_degrees is not None:
        set_drone_heading_hold_degrees(drone_motion_state, target_heading_degrees)
    target_yaw_radians = get_drone_heading_hold_radians(drone_motion_state)

    altitude_error = drone_root.getZ() - drone_motion_state.target_altitude_meters
    desired_vertical_rate = clamp(
        -DRONE_HOVER_GUIDANCE_GAIN * bounded_guidance(altitude_error),
        -vertical_speed_limit,
        vertical_speed_limit,
    )
    desired_vertical_accel = clamp(
        (desired_vertical_rate - drone_motion_state.velocity.z) * DRONE_HOVER_VERTICAL_RATE_GAIN,
        -DRONE_MAX_VERTICAL_ACCEL_METERS_PER_SECOND_SQ,
        DRONE_MAX_VERTICAL_ACCEL_METERS_PER_SECOND_SQ,
    )
    lift_projection = max(
        0.3,
        cos(drone_motion_state.pitch_radians) * cos(drone_motion_state.roll_radians),
    )
    collective_thrust_command = clamp(
        (
            drone_motion_state.mass_kg
            * (DRONE_GRAVITY_METERS_PER_SECOND_SQ + desired_vertical_accel)
        ) / lift_projection,
        drone_motion_state.min_collective_thrust_newtons,
        drone_motion_state.max_collective_thrust_newtons,
    )

    pitch_torque_command = clamp(
        drone_motion_state.inertia_xx_kg_m2
        * (
            DRONE_PITCH_ANGLE_P_GAIN * (desired_pitch_radians - drone_motion_state.pitch_radians)
            - DRONE_PITCH_RATE_D_GAIN * drone_motion_state.pitch_rate_rad_per_second
        ),
        -DRONE_MAX_PITCH_TORQUE_NM,
        DRONE_MAX_PITCH_TORQUE_NM,
    )
    roll_torque_command = clamp(
        drone_motion_state.inertia_yy_kg_m2
        * (
            DRONE_ROLL_ANGLE_P_GAIN * (desired_roll_radians - drone_motion_state.roll_radians)
            - DRONE_ROLL_RATE_D_GAIN * drone_motion_state.roll_rate_rad_per_second
        ),
        -DRONE_MAX_ROLL_TORQUE_NM,
        DRONE_MAX_ROLL_TORQUE_NM,
    )
    yaw_error_radians = wrap_angle_radians(target_yaw_radians - drone_motion_state.yaw_radians)
    if (
        abs(yaw_error_radians) <= DRONE_YAW_HOLD_DEADBAND_RADIANS
        and abs(drone_motion_state.yaw_rate_rad_per_second)
        <= DRONE_YAW_RATE_SETTLE_RADIANS_PER_SECOND
    ):
        drone_motion_state.yaw_radians = target_yaw_radians
        drone_motion_state.yaw_rate_rad_per_second = 0.0
        yaw_error_radians = 0.0
    yaw_torque_command = clamp(
        drone_motion_state.inertia_zz_kg_m2
        * (
            DRONE_YAW_ANGLE_P_GAIN * yaw_error_radians
            - DRONE_YAW_RATE_D_GAIN * drone_motion_state.yaw_rate_rad_per_second
        ),
        -DRONE_MAX_YAW_TORQUE_NM,
        DRONE_MAX_YAW_TORQUE_NM,
    )

    desired_rotor_thrusts = mix_controls_to_rotor_thrusts(
        collective_thrust_command,
        pitch_torque_command,
        roll_torque_command,
        yaw_torque_command,
    )
    actual_rotor_thrusts = update_rotor_dynamics_for_state(
        drone_motion_state,
        desired_rotor_thrusts,
        dt,
    )
    (
        collective_thrust_newtons,
        pitch_torque_nm,
        roll_torque_nm,
        yaw_torque_nm,
    ) = rotor_thrusts_to_control_inputs(actual_rotor_thrusts)

    pitch_accel = (
        (
            (drone_motion_state.inertia_yy_kg_m2 - drone_motion_state.inertia_zz_kg_m2)
            / drone_motion_state.inertia_xx_kg_m2
        )
        * drone_motion_state.roll_rate_rad_per_second
        * drone_motion_state.yaw_rate_rad_per_second
    ) + (pitch_torque_nm / drone_motion_state.inertia_xx_kg_m2)
    roll_accel = (
        (
            (drone_motion_state.inertia_zz_kg_m2 - drone_motion_state.inertia_xx_kg_m2)
            / drone_motion_state.inertia_yy_kg_m2
        )
        * drone_motion_state.pitch_rate_rad_per_second
        * drone_motion_state.yaw_rate_rad_per_second
    ) + (roll_torque_nm / drone_motion_state.inertia_yy_kg_m2)
    yaw_accel = yaw_torque_nm / drone_motion_state.inertia_zz_kg_m2

    drone_motion_state.pitch_rate_rad_per_second += pitch_accel * dt
    drone_motion_state.roll_rate_rad_per_second += roll_accel * dt
    drone_motion_state.yaw_rate_rad_per_second += yaw_accel * dt

    drone_motion_state.pitch_radians = clamp(
        drone_motion_state.pitch_radians + drone_motion_state.pitch_rate_rad_per_second * dt,
        -max_tilt_radians,
        max_tilt_radians,
    )
    drone_motion_state.roll_radians = clamp(
        drone_motion_state.roll_radians + drone_motion_state.roll_rate_rad_per_second * dt,
        -max_tilt_radians,
        max_tilt_radians,
    )
    drone_motion_state.yaw_radians = wrap_angle_radians(
        drone_motion_state.yaw_radians + drone_motion_state.yaw_rate_rad_per_second * dt
    )

    cos_yaw = cos(drone_motion_state.yaw_radians)
    sin_yaw = sin(drone_motion_state.yaw_radians)
    cos_pitch = cos(drone_motion_state.pitch_radians)
    sin_pitch = sin(drone_motion_state.pitch_radians)
    cos_roll = cos(drone_motion_state.roll_radians)
    sin_roll = sin(drone_motion_state.roll_radians)
    thrust_accel = collective_thrust_newtons / drone_motion_state.mass_kg

    world_accel_x = thrust_accel * (
        (cos_yaw * sin_roll) + (sin_yaw * sin_pitch * cos_roll)
    )
    world_accel_y = thrust_accel * (
        (sin_yaw * sin_roll) - (cos_yaw * sin_pitch * cos_roll)
    )
    world_accel_z = thrust_accel * cos_pitch * cos_roll - DRONE_GRAVITY_METERS_PER_SECOND_SQ

    command_horizontal_intent = (
        desired_world_vx * desired_world_vx
        + desired_world_vy * desired_world_vy
    ) ** 0.5
    brake_factor = clamp(
        1.0 - (command_horizontal_intent / max(0.001, horizontal_speed_limit)),
        0.0,
        1.0,
    )
    horizontal_damping_rate = (
        DRONE_HORIZONTAL_BASE_DAMPING_PER_SECOND
        + DRONE_HORIZONTAL_BRAKE_DAMPING_PER_SECOND * brake_factor
    )
    vertical_command_intent = abs(desired_vertical_rate)
    vertical_brake_factor = clamp(
        1.0 - (vertical_command_intent / max(0.001, vertical_speed_limit)),
        0.0,
        1.0,
    )
    vertical_damping_rate = (
        DRONE_VERTICAL_ACTIVE_DRAG_PER_SECOND
        + DRONE_VERTICAL_PASSIVE_DRAG_PER_SECOND * vertical_brake_factor
    )
    world_accel_x -= drone_motion_state.velocity.x * horizontal_damping_rate
    world_accel_y -= drone_motion_state.velocity.y * horizontal_damping_rate
    world_accel_z -= drone_motion_state.velocity.z * vertical_damping_rate

    drone_motion_state.velocity.x += world_accel_x * dt
    drone_motion_state.velocity.y += world_accel_y * dt
    drone_motion_state.velocity.z += world_accel_z * dt

    horizontal_speed = (
        drone_motion_state.velocity.x * drone_motion_state.velocity.x
        + drone_motion_state.velocity.y * drone_motion_state.velocity.y
    ) ** 0.5
    max_horizontal_speed = horizontal_speed_limit * 1.15
    if horizontal_speed > max_horizontal_speed and horizontal_speed > 0.001:
        horizontal_speed_scale = max_horizontal_speed / horizontal_speed
        drone_motion_state.velocity.x *= horizontal_speed_scale
        drone_motion_state.velocity.y *= horizontal_speed_scale
        horizontal_speed = max_horizontal_speed
    if (
        brake_factor > 0.92
        and horizontal_speed < DRONE_HORIZONTAL_STOP_SETTLE_SPEED_METERS_PER_SECOND
    ):
        drone_motion_state.velocity.x = 0.0
        drone_motion_state.velocity.y = 0.0
    drone_motion_state.velocity.z = clamp(
        drone_motion_state.velocity.z,
        -vertical_speed_limit * 1.2,
        vertical_speed_limit * 1.2,
    )
    if (
        vertical_brake_factor > 0.92
        and abs(drone_motion_state.velocity.z) < DRONE_VERTICAL_STOP_SETTLE_SPEED_METERS_PER_SECOND
        and abs(drone_root.getZ() - drone_motion_state.target_altitude_meters)
        < DRONE_VERTICAL_STOP_SETTLE_ALTITUDE_METERS
    ):
        drone_motion_state.velocity.z = 0.0
        drone_motion_state.target_altitude_meters = drone_root.getZ()

    drone_root.setX(drone_root.getX() + drone_motion_state.velocity.x * dt)
    drone_root.setY(drone_root.getY() + drone_motion_state.velocity.y * dt)
    drone_root.setZ(drone_root.getZ() + drone_motion_state.velocity.z * dt)

    drone_root.setH(degrees(drone_motion_state.yaw_radians))
    target_visual_pitch = degrees(drone_motion_state.pitch_radians) * visual_tilt_scale
    target_visual_roll = degrees(drone_motion_state.roll_radians) * visual_tilt_scale
    if visual_tilt_time_constant_seconds > 0.0:
        drone_motion_state.visual_pitch = low_pass_value(
            drone_motion_state.visual_pitch,
            target_visual_pitch,
            dt,
            visual_tilt_time_constant_seconds,
        )
        drone_motion_state.visual_roll = low_pass_value(
            drone_motion_state.visual_roll,
            target_visual_roll,
            dt,
            visual_tilt_time_constant_seconds,
        )
    else:
        drone_motion_state.visual_pitch = target_visual_pitch
        drone_motion_state.visual_roll = target_visual_roll
    drone_visual_root.setP(drone_motion_state.visual_pitch)
    drone_visual_root.setR(drone_motion_state.visual_roll)

    average_rotor_omega = sum(drone_motion_state.rotor_omegas_rad_per_second) / max(
        1,
        len(drone_motion_state.rotor_omegas_rad_per_second),
    )
    propeller_spin_speed = min(
        DRONE_PROPELLER_SPIN_MAX_DEGREES_PER_SECOND,
        DRONE_PROPELLER_SPIN_BASE_DEGREES_PER_SECOND + (average_rotor_omega * 2.35),
    )
    drone_motion_state.propeller_spin_degrees = (
        drone_motion_state.propeller_spin_degrees + propeller_spin_speed * dt
    ) % 360.0
    for rotor_node, spin_direction in drone_propellers:
        rotor_node.setR(drone_motion_state.propeller_spin_degrees * spin_direction)

    edge_margin = 4
    clamped_x = max(SPAWN_X_MIN + edge_margin, min(SPAWN_X_MAX - edge_margin, drone_root.getX()))
    clamped_y = max(SPAWN_Y_MIN + edge_margin, min(SPAWN_Y_MAX - edge_margin, drone_root.getY()))
    if clamped_x != drone_root.getX():
        drone_motion_state.velocity.x = 0.0
    if clamped_y != drone_root.getY():
        drone_motion_state.velocity.y = 0.0
    drone_root.setX(clamped_x)
    drone_root.setY(clamped_y)

    min_altitude = compute_safe_altitude_at(
        drone_root.getX(),
        drone_root.getY(),
        drone_root.getZ(),
        1.6,
    )
    if drone_root.getZ() < min_altitude:
        drone_root.setZ(min_altitude)
        drone_motion_state.velocity.z = max(0.0, drone_motion_state.velocity.z)


def get_current_speed_preset_multiplier():
    return DRONE_SPEED_PRESET_MULTIPLIERS[current_speed_preset]


def get_current_drone_speed_units_per_second():
    return DRONE_SPEED_UNITS_PER_SECOND * get_current_speed_preset_multiplier()


def controller_manual_input_active():
    return controller_motion_input_should_apply() and not raw_keyboard_movement_command_active()


def get_manual_drone_speed_units_per_second():
    if controller_manual_input_active():
        return TX12_CONTROLLER_MANUAL_SPEED_UNITS_PER_SECOND
    return get_current_drone_speed_units_per_second()


def patrol_waypoint_stall_timeout_seconds(distance_meters):
    cruise_speed = max(
        1.0,
        get_current_drone_speed_units_per_second() * AUTOMATION_SPEED_FACTOR,
    )
    return PATROL_STALL_BASE_ALLOWANCE_SECONDS + (
        PATROL_STALL_DISTANCE_TIME_FACTOR * distance_meters / cruise_speed
    )


def get_speed_preset_max_tilt_radians(speed_preset):
    return radians(
        DRONE_TILT_LIMIT_DEGREES_BY_PRESET.get(
            speed_preset,
            DRONE_TILT_LIMIT_DEGREES_BY_PRESET[DRONE_SPEED_PRESET_NORMAL],
        )
    )


def get_current_max_tilt_radians():
    return get_speed_preset_max_tilt_radians(current_speed_preset)


def get_manual_max_tilt_radians():
    if controller_manual_input_active():
        return get_speed_preset_max_tilt_radians(TX12_CONTROLLER_MANUAL_SPEED_PRESET)
    return get_current_max_tilt_radians()


def get_current_speed_preset_label():
    return current_speed_preset.upper()


def format_speed_mode_menu():
    controller_label = format_controller_hud_label()
    if controller_controls_active():
        return (
            f"CONTROLLER: sticks 0-{TX12_CONTROLLER_MANUAL_SPEED_UNITS_PER_SECOND:.1f} m/s"
            " | CH3 middle hover (+up/-down)"
            f"{controller_label}\n"
            "KEYBOARD: Z/X/C speed only when controller is off"
        )
    return (
        f"CURRENT SPEED: {get_current_speed_preset_label()} {get_current_drone_speed_units_per_second():.1f} m/s"
        f" | [Z] Slow {DRONE_SPEED_PRESET_MULTIPLIERS[DRONE_SPEED_PRESET_SLOW] * DRONE_SPEED_UNITS_PER_SECOND:.1f} m/s"
        f" | [X] Normal {DRONE_SPEED_PRESET_MULTIPLIERS[DRONE_SPEED_PRESET_NORMAL] * DRONE_SPEED_UNITS_PER_SECOND:.1f} m/s"
        f" | [C] Fast {DRONE_SPEED_PRESET_MULTIPLIERS[DRONE_SPEED_PRESET_FAST] * DRONE_SPEED_UNITS_PER_SECOND:.1f} m/s"
        f"{controller_label}"
    )


def choose_next_wind_gust():
    wind_state.target_gust_direction_degrees = random.uniform(
        -WIND_GUST_DIRECTION_RANGE_DEGREES,
        WIND_GUST_DIRECTION_RANGE_DEGREES,
    )
    wind_state.target_gust_speed_mps = random.uniform(
        -WIND_GUST_SPEED_RANGE_METERS_PER_SECOND,
        WIND_GUST_SPEED_RANGE_METERS_PER_SECOND,
    )
    wind_state.gust_hold_timer_seconds = random.uniform(
        WIND_GUST_MIN_DURATION_SECONDS,
        WIND_GUST_MAX_DURATION_SECONDS,
    )


def update_wind(dt):
    if not WIND_ENABLED:
        wind_state.gust_direction_degrees = 0.0
        wind_state.gust_speed_mps = 0.0
        return

    wind_state.gust_hold_timer_seconds -= dt
    if wind_state.gust_hold_timer_seconds <= 0.0:
        choose_next_wind_gust()

    gust_heading_delta = _shortest_angle_delta_degrees(
        wind_state.gust_direction_degrees,
        wind_state.target_gust_direction_degrees,
    )
    max_heading_step = WIND_GUST_DIRECTION_RESPONSE_DEGREES_PER_SECOND * dt
    if abs(gust_heading_delta) <= max_heading_step:
        wind_state.gust_direction_degrees = wind_state.target_gust_direction_degrees
    else:
        heading_step = max_heading_step if gust_heading_delta > 0 else -max_heading_step
        wind_state.gust_direction_degrees = _normalize_signed_degrees(
            wind_state.gust_direction_degrees + heading_step
        )

    wind_state.gust_speed_mps = approach(
        wind_state.gust_speed_mps,
        wind_state.target_gust_speed_mps,
        WIND_GUST_SPEED_RESPONSE_PER_SECOND,
        dt,
    )


def get_wind_direction_degrees():
    return (wind_state.base_direction_degrees + wind_state.gust_direction_degrees) % 360.0


def get_wind_speed_mps():
    if not WIND_ENABLED:
        return 0.0
    return max(0.0, wind_state.base_speed_mps + wind_state.gust_speed_mps)


def get_wind_velocity():
    wind_direction_degrees = get_wind_direction_degrees()
    wind_speed_mps = get_wind_speed_mps()
    wind_heading = radians(wind_direction_degrees)
    wind_vx = sin(wind_heading) * wind_speed_mps
    wind_vy = cos(wind_heading) * wind_speed_mps
    return wind_vx, wind_vy, wind_direction_degrees, wind_speed_mps


def get_wind_compass_label(direction_degrees):
    compass_labels = ("N", "NE", "E", "SE", "S", "SW", "W", "NW")
    label_index = int(((direction_degrees + 22.5) % 360.0) // 45.0)
    return compass_labels[label_index]


choose_next_wind_gust()
initialize_motion_state_from_drone_pose()
initialize_drone_motion_state_from_pose(water_drone, water_motion_state)


def set_key(key, value):
    #this is a helper fuhction
    key_map[key] = value


def zero_tx12_controller_axes():
    for axis_name in tx12_controller_axes:
        tx12_controller_axes[axis_name] = 0.0


def raw_keyboard_movement_command_active():
    return any(
        key_map.get(action_name, False)
        for action_name in ("left", "right", "forward", "backward", "up", "down")
    )


def keyboard_movement_command_active():
    return raw_keyboard_movement_command_active()


def controller_controls_active():
    return (
        REAL_SIM_RADIO_ENABLED
        and tx12_controller_input_enabled
        and tx12_controller_is_connected()
    )


def keyboard_movement_controls_allowed():
    return True


def clear_keyboard_movement_keys():
    for action_name in ("left", "right", "forward", "backward", "up", "down"):
        key_map[action_name] = False


def toggle_tx12_controller_input():
    global tx12_controller_input_enabled
    if not REAL_SIM_RADIO_ENABLED:
        tx12_controller_input_enabled = False
        zero_tx12_controller_axes()
        clear_keyboard_movement_keys()
        print("[real-sim] Radio/controller input is disabled in this build")
        return
    tx12_controller_input_enabled = not tx12_controller_input_enabled
    zero_tx12_controller_axes()
    clear_keyboard_movement_keys()
    state_label = "enabled" if tx12_controller_input_enabled else "disabled"
    print(f"[real-sim] Controller input {state_label}")
    if "status_text" in globals():
        update_status_overlay()


def request_tx12_controller_recenter():
    global tx12_controller_recenter_pending
    if not REAL_SIM_RADIO_ENABLED:
        return
    tx12_controller_recenter_pending = True
    zero_tx12_controller_axes()
    print("[real-sim] Controller recenter requested")
    if "status_text" in globals():
        update_status_overlay()


def tx12_controller_is_connected():
    return (
        REAL_SIM_RADIO_ENABLED
        and
        tx12_controller_device is not None
        and getattr(tx12_controller_device, "connected", False)
    )


def _tx12_axis_value(device, axis_candidates):
    for axis in axis_candidates:
        axis_state = device.findAxis(axis)
        if getattr(axis_state, "known", False):
            return float(axis_state.value)
    return 0.0


def _tx12_channel_values(device):
    try:
        axis_states = list(device.axes)
    except Exception:
        return ()
    return tuple(
        float(axis_state.value)
        for axis_state in axis_states
        if getattr(axis_state, "known", False)
    )


def _tx12_channel_value(channel_values, channel_number, default=0.0):
    channel_index = channel_number - 1
    if channel_index < 0 or channel_index >= len(channel_values):
        return default
    return float(channel_values[channel_index])


def format_tx12_channel_snapshot(channel_values):
    if not channel_values:
        return ""
    shown_channel_count = min(
        len(channel_values),
        max(5, TX12_CONTROLLER_KILL_CHANNEL),
    )
    return " ".join(
        f"CH{channel_index + 1}={channel_value:+.2f}"
        for channel_index, channel_value in enumerate(
            channel_values[:shown_channel_count],
        )
    )


def _tx12_apply_deadzone(value):
    value = clamp(value, -1.0, 1.0)
    magnitude = abs(value)
    if magnitude <= TX12_CONTROLLER_DEADZONE:
        return 0.0
    scaled = (magnitude - TX12_CONTROLLER_DEADZONE) / (
        1.0 - TX12_CONTROLLER_DEADZONE
    )
    return copysign(clamp(scaled, 0.0, 1.0), value)


def _tx12_apply_centered_axis(value, center):
    value = clamp(value, -1.0, 1.0)
    center = clamp(center, -1.0, 1.0)
    if value >= center:
        normalized = (value - center) / max(0.001, 1.0 - center)
    else:
        normalized = (value - center) / max(0.001, center + 1.0)
    return _tx12_apply_deadzone(normalized)


def _is_tx12_named_device(device):
    device_name = (getattr(device, "name", "") or "").lower()
    return any(
        token in device_name
        for token in ("tx12", "radiomaster", "radio master", "edgetx")
    )


def _device_has_axes(device):
    try:
        return len(device.axes) > 0
    except Exception:
        return False


def format_controller_device_name():
    if not tx12_controller_is_connected():
        return ""
    return tx12_controller_name or "USB joystick"


def format_controller_state_label():
    if not tx12_controller_input_enabled:
        return "disabled"
    if not tx12_controller_is_connected():
        return "not detected"
    return "ready"


def format_controller_axis_snapshot():
    return (
        f"Pitch {tx12_controller_axes['forward']:+.2f}  "
        f"Roll {tx12_controller_axes['right']:+.2f}  "
        f"Alt {tx12_controller_axes['vertical']:+.2f}  "
        f"Yaw {tx12_controller_axes['yaw']:+.2f}"
    )


def format_controller_center_snapshot():
    return (
        f"Pitch {tx12_controller_axis_centers['forward']:+.2f} "
        f"Roll {tx12_controller_axis_centers['right']:+.2f} "
        f"Alt {tx12_controller_axis_centers['vertical']:+.2f} "
        f"Yaw {tx12_controller_axis_centers['yaw']:+.2f}"
    )


def format_controller_hud_label():
    device_name = format_controller_device_name()
    if not device_name:
        return ""
    device_name_lower = device_name.lower()
    device_label = (
        "TX12"
        if "tx12" in device_name_lower or "radiomaster" in device_name_lower
        else "USB"
    )
    return f" | Controller: {device_label} {format_controller_state_label()}"


def format_controller_setup_line():
    if not REAL_SIM_RADIO_ENABLED:
        return ""
    device_name = format_controller_device_name()
    if not tx12_controller_input_enabled:
        return "Controller: disabled ([U] enable)"
    if not device_name:
        return "Controller: not detected"
    return f"Controller: TX12 {format_controller_state_label()}"


def format_radio_pregame_controls():
    if not REAL_SIM_RADIO_ENABLED:
        return ""
    return (
        f"{format_controller_setup_line()} | [U] on/off [N] recenter\n"
        "TX12: CH1 roll CH2 pitch CH3 alt CH4 yaw; CH5 kill\n"
    )


def _button_handle_name(button_handle):
    if button_handle is None:
        return ""
    try:
        return button_handle.getName()
    except Exception:
        return getattr(button_handle, "name", "") or ""


def get_pressed_controller_button_names(device):
    pressed_names = set()
    try:
        button_states = list(device.buttons)
    except Exception:
        button_states = ()

    for button_index, button_state in enumerate(button_states):
        if not getattr(button_state, "known", False):
            continue
        if not getattr(button_state, "pressed", False):
            continue
        handle = getattr(button_state, "handle", None)
        handle_name = _button_handle_name(handle).strip().lower()
        if handle_name:
            pressed_names.add(handle_name)
        pressed_names.add(f"button{button_index}")

    return pressed_names


def update_tx12_controller_buttons(device):
    global tx12_controller_pressed_buttons
    pressed_names = get_pressed_controller_button_names(device)
    tx12_controller_pressed_buttons = pressed_names


def update_tx12_channel_switches(channel_values):
    global tx12_controller_channel_kill_active, tx12_controller_kill_channel_last_value
    if TX12_CONTROLLER_KILL_CHANNEL <= 0:
        tx12_controller_channel_kill_active = False
        tx12_controller_kill_channel_last_value = None
        return
    kill_value = _tx12_channel_value(channel_values, TX12_CONTROLLER_KILL_CHANNEL)

    if tx12_controller_kill_channel_last_value is None:
        tx12_controller_kill_channel_last_value = kill_value
        return

    switch_changed = (
        abs(kill_value - tx12_controller_kill_channel_last_value)
        >= TX12_CONTROLLER_KILL_CHANGE_THRESHOLD
    )
    if switch_changed and not tx12_controller_channel_kill_active:
        trigger_kill_switch()
        tx12_controller_channel_kill_active = True
    if switch_changed:
        tx12_controller_kill_channel_last_value = kill_value


def update_tx12_channel_mode(channel_values):
    # TX12 channels are limited to sticks plus CH5 kill.
    return


def update_tx12_channel_actions(channel_values):
    # No controller full-auto or hover-all actions.
    return


def format_known_controller_axes(device):
    axis_parts = []
    try:
        axis_states = list(device.axes)
    except Exception:
        axis_states = ()
    for axis_state in axis_states:
        if not getattr(axis_state, "known", False):
            continue
        axis = getattr(axis_state, "axis", None)
        axis_name = getattr(axis, "name", str(axis)).lower()
        axis_parts.append(f"{axis_name}={float(axis_state.value):+.2f}")
    return " ".join(axis_parts)


def maybe_print_controller_debug(
    device,
    raw_right,
    raw_forward,
    raw_vertical,
    raw_yaw,
    channel_values=(),
):
    global tx12_controller_last_debug_print_time
    if not REAL_SIM_CONTROLLER_DEBUG:
        return
    now = time.perf_counter()
    if now - tx12_controller_last_debug_print_time < 1.0:
        return
    tx12_controller_last_debug_print_time = now
    device_name = format_controller_device_name() or "USB joystick"
    print(
        "[real-sim] controller "
        f"{device_name}: "
        f"raw Pitch {raw_forward:+.3f} Roll {raw_right:+.3f} "
        f"Alt {raw_vertical:+.3f} Yaw {raw_yaw:+.3f} | "
        f"mapped {format_controller_axis_snapshot()} | "
        f"centers {format_controller_center_snapshot()} | "
        f"channels {format_tx12_channel_snapshot(channel_values)} | "
        f"buttons {sorted(tx12_controller_pressed_buttons)} | "
        f"axes {format_known_controller_axes(device)}"
    )


def refresh_tx12_controller_device():
    global tx12_controller_device, tx12_controller_name, tx12_controller_was_connected
    global tx12_controller_pressed_buttons
    global tx12_controller_recenter_pending, tx12_controller_channel_kill_active
    global tx12_controller_kill_channel_last_value
    if not REAL_SIM_RADIO_ENABLED:
        tx12_controller_device = None
        tx12_controller_name = ""
        tx12_controller_was_connected = False
        tx12_controller_pressed_buttons = set()
        tx12_controller_recenter_pending = True
        tx12_controller_channel_kill_active = False
        tx12_controller_kill_channel_last_value = None
        zero_tx12_controller_axes()
        return
    if tx12_controller_is_connected():
        return

    tx12_controller_device = None
    tx12_controller_name = ""
    tx12_controller_pressed_buttons = set()
    tx12_controller_recenter_pending = True
    tx12_controller_channel_kill_active = False
    tx12_controller_kill_channel_last_value = None
    zero_tx12_controller_axes()

    if "app" not in globals() or not hasattr(app, "devices"):
        return

    devices = list(app.devices.getDevices())
    preferred_devices = [
        device
        for device in devices
        if getattr(device, "connected", False)
        and _device_has_axes(device)
        and _is_tx12_named_device(device)
    ]
    fallback_devices = [
        device
        for device in devices
        if getattr(device, "connected", False)
        and _device_has_axes(device)
        and device.device_class
        in (
            InputDevice.DeviceClass.flight_stick,
            InputDevice.DeviceClass.gamepad,
            InputDevice.DeviceClass.unknown,
        )
    ]
    for device in preferred_devices + fallback_devices:
        tx12_controller_device = device
        tx12_controller_name = getattr(device, "name", "") or "USB joystick"
        try:
            app.attachInputDevice(device, prefix="tx12")
        except Exception:
            pass
        if not tx12_controller_was_connected:
            print(f"[real-sim] TX12/controller connected: {tx12_controller_name}")
            clear_keyboard_movement_keys()
            tx12_controller_recenter_pending = True
        tx12_controller_was_connected = True
        return

    if tx12_controller_was_connected:
        print("[real-sim] TX12/controller disconnected")
    tx12_controller_was_connected = False


def update_tx12_controller_input(dt):
    global camera_angle, camera_manual_override_active
    global tx12_controller_recenter_pending, tx12_controller_channel_kill_active
    global tx12_controller_kill_channel_last_value
    if not REAL_SIM_RADIO_ENABLED:
        zero_tx12_controller_axes()
        return
    refresh_tx12_controller_device()
    if not tx12_controller_input_enabled or not tx12_controller_is_connected():
        zero_tx12_controller_axes()
        return

    device = tx12_controller_device
    try:
        device.poll()
    except Exception:
        pass
    update_tx12_controller_buttons(device)

    tx12_like = (
        _is_tx12_named_device(device)
        or device.device_class != InputDevice.DeviceClass.gamepad
    )
    channel_values = _tx12_channel_values(device) if tx12_like else ()
    if tx12_like:
        raw_right = _tx12_channel_value(channel_values, 1)         # right stick: roll
        raw_forward = -_tx12_channel_value(channel_values, 2)      # right stick: pitch
        raw_vertical = _tx12_channel_value(channel_values, 3)      # left stick: altitude
        raw_yaw = -_tx12_channel_value(channel_values, 4)          # left stick: yaw
    else:
        raw_right = _tx12_axis_value(
            device,
            (InputDevice.Axis.right_x, InputDevice.Axis.x),
        )
        raw_forward = -_tx12_axis_value(
            device,
            (InputDevice.Axis.right_y, InputDevice.Axis.y),
        )
        raw_vertical = -_tx12_axis_value(
            device,
            (InputDevice.Axis.left_y, InputDevice.Axis.throttle),
        )
        raw_yaw = _tx12_axis_value(
            device,
            (InputDevice.Axis.left_x, InputDevice.Axis.yaw),
        )

    if tx12_controller_recenter_pending:
        tx12_controller_axis_centers["right"] = clamp(raw_right, -1.0, 1.0)
        tx12_controller_axis_centers["forward"] = clamp(raw_forward, -1.0, 1.0)
        tx12_controller_axis_centers["vertical"] = clamp(raw_vertical, -1.0, 1.0)
        tx12_controller_axis_centers["yaw"] = clamp(raw_yaw, -1.0, 1.0)
        for channel_number in tx12_controller_channel_baselines:
            tx12_controller_channel_baselines[channel_number] = _tx12_channel_value(
                channel_values,
                channel_number,
            )
        tx12_controller_channel_kill_active = False
        tx12_controller_kill_channel_last_value = (
            _tx12_channel_value(channel_values, TX12_CONTROLLER_KILL_CHANNEL)
            if TX12_CONTROLLER_KILL_CHANNEL > 0
            else None
        )
        tx12_controller_recenter_pending = False
        zero_tx12_controller_axes()
        print(
            "[real-sim] Controller centered: "
            f"{format_controller_center_snapshot()} | "
            f"{format_tx12_channel_snapshot(channel_values)}"
        )
        if "status_text" in globals():
            update_status_overlay()
        return

    if tx12_like:
        update_tx12_channel_switches(channel_values)

    tx12_controller_axes["right"] = _tx12_apply_centered_axis(
        raw_right,
        tx12_controller_axis_centers["right"],
    )
    tx12_controller_axes["forward"] = _tx12_apply_centered_axis(
        raw_forward,
        tx12_controller_axis_centers["forward"],
    )
    tx12_controller_axes["vertical"] = _tx12_apply_centered_axis(
        raw_vertical,
        tx12_controller_axis_centers["vertical"],
    )
    tx12_controller_axes["yaw"] = _tx12_apply_centered_axis(
        raw_yaw,
        tx12_controller_axis_centers["yaw"],
    )
    maybe_print_controller_debug(
        device,
        raw_right,
        raw_forward,
        raw_vertical,
        raw_yaw,
        channel_values,
    )

    selected_view = get_selected_drone_view()
    if (
        selected_view is None
        or overview_camera_enabled
        or is_view_damaged(selected_view)
        or get_drone_view_control_mode(selected_view) != CONTROL_MODE_MANUAL
    ):
        return

    yaw_input = tx12_controller_axes["yaw"]
    if abs(yaw_input) > 0.001:
        camera_angle = wrap_angle_degrees(
            camera_angle + yaw_input * TX12_CONTROLLER_YAW_DEGREES_PER_SECOND * dt
        )
        selected_motion_state = get_drone_view_motion_state(selected_view)
        if selected_motion_state is not None:
            set_drone_heading_hold_degrees(selected_motion_state, camera_angle)
        camera_manual_override_active = True


def controller_movement_command_active():
    if not controller_motion_input_should_apply():
        return False
    return any(
        abs(tx12_controller_axes[axis_name]) > 0.001
        for axis_name in ("right", "forward", "vertical", "yaw")
    )


def controller_motion_input_should_apply():
    return controller_controls_active()


def compute_manual_desired_velocity(camera_heading_degrees, key_state, speed):
    if raw_keyboard_movement_command_active():
        forward_input = int(key_state["forward"]) - int(key_state["backward"])
        right_input = int(key_state["right"]) - int(key_state["left"])
        vertical_input = int(key_state["up"]) - int(key_state["down"])
    elif controller_motion_input_should_apply():
        forward_input = tx12_controller_axes["forward"]
        right_input = tx12_controller_axes["right"]
        vertical_input = tx12_controller_axes["vertical"]
    else:
        forward_input = int(key_state["forward"]) - int(key_state["backward"])
        right_input = int(key_state["right"]) - int(key_state["left"])
        vertical_input = int(key_state["up"]) - int(key_state["down"])

    horizontal_length = (forward_input * forward_input + right_input * right_input) ** 0.5
    if horizontal_length > 1.0:
        forward_input /= horizontal_length
        right_input /= horizontal_length
    vertical_input = clamp(vertical_input, -1.0, 1.0)

    angle = radians(camera_heading_degrees)
    return (
        (right_input * cos(angle) - forward_input * sin(angle)) * speed,
        (right_input * sin(angle) + forward_input * cos(angle)) * speed,
        vertical_input * speed,
    )


def get_all_drone_views():
    """Ordered camera/control targets: all survey drones, then all water drones."""
    views = [
        {
            "root": drone,
            "label": "Survey drone 1",
            "role": "survey",
            "slot": 1,
            "follower": None,
        }
    ]
    survey_followers = sorted(
        (
            follower
            for follower in follower_drones
            if follower.get("role") == "survey"
            and follower.get("root") is not None
            and not follower["root"].isEmpty()
        ),
        key=lambda follower: follower.get("slot", 0),
    )
    for follower in survey_followers:
        slot = follower.get("slot", 0) + 1
        views.append(
            {
                "root": follower["root"],
                "label": f"Survey drone {slot}",
                "role": "survey",
                "slot": slot,
                "follower": follower,
            }
        )

    views.append(
        {
            "root": water_drone,
            "label": "Water drone 1",
            "role": "water",
            "slot": 1,
            "follower": None,
        }
    )
    water_followers = sorted(
        (
            follower
            for follower in follower_drones
            if follower.get("role") == "water"
            and follower.get("root") is not None
            and not follower["root"].isEmpty()
        ),
        key=lambda follower: follower.get("slot", 0),
    )
    for follower in water_followers:
        slot = follower.get("slot", 0) + 1
        views.append(
            {
                "root": follower["root"],
                "label": f"Water drone {slot}",
                "role": "water",
                "slot": slot,
                "follower": follower,
            }
        )
    return views


def get_drone_view_for_role_slot(role, slot):
    for view in get_all_drone_views():
        if view["role"] == role and view["slot"] == slot:
            return view
    return None


def get_selected_drone_view():
    views = get_all_drone_views()
    if not views:
        return None
    target_node = get_camera_target_node()
    for view in views:
        if view["root"] is target_node:
            return view
    clamped_index = int(clamp(camera_target_index, 0, len(views) - 1))
    return views[clamped_index]


def get_drone_view_control_mode(view):
    if view is None:
        return CONTROL_MODE_AUTOMATION
    if view["follower"] is not None:
        return view["follower"].get("control_mode", CONTROL_MODE_AUTOMATION)
    if view["role"] == "water":
        return water_control_mode
    return control_mode


def get_drone_view_motion_state(view):
    if view is None:
        return None
    if view["follower"] is not None:
        return view["follower"].get("motion_state")
    if view["role"] == "water":
        return water_motion_state
    return motion_state


def heading_from_world_vector(vx, vy):
    return degrees(atan2(-vx, vy))


def selected_automation_travel_heading_degrees():
    view = get_selected_drone_view()
    if view is None or is_view_damaged(view):
        return None
    if get_drone_view_control_mode(view) != CONTROL_MODE_AUTOMATION:
        return None
    drone_motion_state = get_drone_view_motion_state(view)
    if drone_motion_state is None:
        return None

    cmd_vx = drone_motion_state.applied_cmd.x
    cmd_vy = drone_motion_state.applied_cmd.y
    command_speed = (cmd_vx * cmd_vx + cmd_vy * cmd_vy) ** 0.5
    if command_speed >= AUTOMATION_CAMERA_ALIGN_MIN_COMMAND_METERS_PER_SECOND:
        return heading_from_world_vector(cmd_vx, cmd_vy)

    velocity_vx = drone_motion_state.velocity.x
    velocity_vy = drone_motion_state.velocity.y
    horizontal_speed = (velocity_vx * velocity_vx + velocity_vy * velocity_vy) ** 0.5
    if horizontal_speed >= AUTOMATION_CAMERA_ALIGN_MIN_SPEED_METERS_PER_SECOND:
        return heading_from_world_vector(velocity_vx, velocity_vy)

    root = view.get("root")
    if root is not None and not root.isEmpty():
        return root.getH()
    return None


def auto_align_camera_to_selected_drone(dt):
    global camera_angle
    if overview_camera_enabled:
        return
    target_heading = selected_automation_travel_heading_degrees()
    if target_heading is None:
        return
    target_heading += camera_yaw_offset_degrees
    camera_angle = wrap_angle_degrees(
        approach_angle_degrees(
            camera_angle,
            target_heading,
            AUTOMATION_CAMERA_ALIGN_SPEED_DEGREES_PER_SECOND,
            dt,
        )
    )


def snap_camera_angle_to_selected_automation_heading():
    global camera_angle
    target_heading = selected_automation_travel_heading_degrees()
    if target_heading is None:
        return
    camera_angle = wrap_angle_degrees(target_heading + camera_yaw_offset_degrees)


def clamp_camera_pitch_for_current_mode():
    global camera_pitch
    if first_person_view:
        camera_pitch = clamp(
            camera_pitch,
            FIRST_PERSON_CAMERA_MIN_PITCH_DEGREES,
            FIRST_PERSON_CAMERA_MAX_PITCH_DEGREES,
        )
    else:
        camera_pitch = clamp(
            camera_pitch,
            THIRD_PERSON_CAMERA_MIN_PITCH_DEGREES,
            THIRD_PERSON_CAMERA_MAX_PITCH_DEGREES,
        )


def apply_mission_start_camera():
    global first_person_view, camera_pitch
    global overview_camera_enabled, overview_is_operator
    global camera_manual_override_active, camera_yaw_offset_degrees

    first_person_view = True
    camera_pitch = MISSION_START_CAMERA_PITCH_DEGREES
    overview_camera_enabled = False
    overview_is_operator = False
    camera_manual_override_active = False
    camera_yaw_offset_degrees = 0.0
    clamp_camera_pitch_for_current_mode()
    apply_camera_mode()


def get_camera_target_node():
    if camera_target_node_override is not None and not camera_target_node_override.isEmpty():
        return camera_target_node_override
    return water_drone if camera_target == CAMERA_TARGET_WATER else drone


def set_camera_target_index(new_index):
    global camera_target, camera_target_index, camera_target_node_override
    global view_switch_count, camera_manual_override_active, camera_yaw_offset_degrees
    views = get_all_drone_views()
    if not views:
        return
    new_index = int(clamp(new_index, 0, len(views) - 1))
    if new_index != camera_target_index and not pregame_active:
        view_switch_count += 1
        view_switch_log.append(
            (motion_state.sim_time_seconds, views[new_index]["label"])
        )
    if overview_camera_enabled and not pregame_active:
        # Choosing a specific drone drops out of any overhead view and brings
        # the little detected-fire map back (#14).
        set_overhead_view(False, overview_is_operator)
    camera_target_index = new_index
    target_view = views[new_index]
    target_node = target_view["root"]
    if target_view["role"] == "survey" and target_view["slot"] == 1:
        camera_target = CAMERA_TARGET_SURVEY
        camera_target_node_override = None
    elif target_view["role"] == "water" and target_view["slot"] == 1:
        camera_target = CAMERA_TARGET_WATER
        camera_target_node_override = None
    else:
        camera_target = target_view["role"]
        camera_target_node_override = target_node
    camera_manual_override_active = False
    camera_yaw_offset_degrees = 0.0
    snap_camera_angle_to_selected_automation_heading()
    apply_camera_mode()
    if "status_text" in globals():
        update_status_overlay()


def set_camera_target_role_slot(role, slot):
    target_view = get_drone_view_for_role_slot(role, slot)
    if target_view is None:
        return
    views = get_all_drone_views()
    for index, view in enumerate(views):
        if view["root"] is target_view["root"]:
            set_camera_target_index(index)
            return


def handle_number_key(key_number):
    # During setup, [1]/[2] shift the team ratio. In flight, number keys switch
    # by fixed role slots: 1-3 survey, 4-6 water.
    if pregame_active:
        if key_number == 1:
            choose_more_survey_drones()
        elif key_number == 2:
            choose_more_water_drones()
        return
    if 1 <= key_number <= 3:
        set_camera_target_role_slot("survey", key_number)
    elif 4 <= key_number <= 6:
        set_camera_target_role_slot("water", key_number - 3)


def apply_camera_mode():
    for view in get_all_drone_views():
        view["root"].show()
    if overview_camera_enabled:
        return
    if first_person_view:
        get_camera_target_node().hide()


def set_first_person_mode():
    global first_person_view, camera_pitch
    first_person_view = True
    camera_pitch = FIRST_PERSON_CAMERA_PITCH_DEGREES
    clamp_camera_pitch_for_current_mode()
    apply_camera_mode()


def set_third_person_mode():
    global first_person_view, camera_pitch
    first_person_view = False
    camera_pitch = THIRD_PERSON_CAMERA_PITCH_DEGREES
    clamp_camera_pitch_for_current_mode()
    apply_camera_mode()


def toggle_camera_mode():
    global view_switch_count
    if not pregame_active:
        view_switch_count += 1
        record_view_switch_event("camera_mode_toggle")
        view_switch_log.append(
            (
                motion_state.sim_time_seconds,
                "CHASE CAMERA" if first_person_view else "FIRST PERSON",
            )
        )
    if first_person_view:
        set_third_person_mode()
    else:
        set_first_person_mode()


def set_camera_target_survey():
    set_camera_target_role_slot("survey", 1)


def set_camera_target_water():
    set_camera_target_role_slot("water", 1)


def set_overhead_view(active, is_operator):
    """Enable/disable an overhead orbit camera. is_operator=True is the
    realistic OPERATOR satellite view (only detected fire, little map hidden);
    is_operator=False is the DEV full-info bird view (sees everything)."""
    global overview_camera_enabled, overview_is_operator
    global overview_camera_angle, overview_camera_pitch, view_switch_count
    global overview_camera_distance
    was_active = overview_camera_enabled
    overview_camera_enabled = bool(active)
    overview_is_operator = bool(is_operator) if active else False
    if active and not was_active and not pregame_active:
        view_switch_count += 1
        record_view_switch_event("overview_toggle")
        view_switch_log.append(
            (
                motion_state.sim_time_seconds,
                "OPERATOR SATELLITE" if is_operator else "DEV BIRD VIEW",
            )
        )
    if active:
        overview_camera_angle = OVERVIEW_CAMERA_RESET_ANGLE_DEGREES
        overview_camera_pitch = clamp(
            OVERVIEW_CAMERA_RESET_PITCH_DEGREES,
            OVERVIEW_CAMERA_MIN_PITCH_DEGREES,
            OVERVIEW_CAMERA_MAX_PITCH_DEGREES,
        )
        # Start framed on the WHOLE map, then the wheel can freely zoom in/out.
        _, _, _, scene_span = get_overview_focus_point()
        overview_camera_distance = clamp(
            (scene_span * OVERVIEW_CAMERA_AUTO_DISTANCE_SCALE)
            + OVERVIEW_CAMERA_AUTO_DISTANCE_PADDING_METERS,
            OVERVIEW_CAMERA_MIN_DISTANCE,
            OVERVIEW_CAMERA_MAX_DISTANCE,
        )
    refresh_operator_fire_visibility()
    apply_camera_mode()
    if "fire_map_panel" in globals() and fire_map_panel is not None:
        update_fire_map_popup()


def toggle_operator_satellite_view():
    """[Tab] operator's realistic satellite top-down. Selecting a single drone
    (keys 1-6) brings the normal view + little map back."""
    if pregame_active or results_page_active:
        return
    if overview_camera_enabled and overview_is_operator:
        set_overhead_view(False, True)
    else:
        set_overhead_view(True, True)


def toggle_dev_bird_view():
    """[\\] DEV-ONLY full-information bird view (shows undetected fire too)."""
    if pregame_active or results_page_active:
        return
    if overview_camera_enabled and not overview_is_operator:
        set_overhead_view(False, False)
    else:
        set_overhead_view(True, False)


def refresh_operator_fire_visibility():
    """In the operator satellite view, undetected fire is invisible (the
    operator must actually detect it with drones). Every other view shows the
    fire world normally."""
    hide_undetected = overview_camera_enabled and overview_is_operator
    for hotspot in fire_hotspots:
        root = getattr(hotspot, "root", None)
        if root is None or root.isEmpty():
            continue
        if hide_undetected and not hotspot.detected:
            root.hide()
        else:
            root.show()


def adjust_overview_camera_zoom(distance_delta):
    global overview_camera_distance
    if not overview_camera_enabled:
        return
    overview_camera_distance = clamp(
        overview_camera_distance + distance_delta,
        OVERVIEW_CAMERA_MIN_DISTANCE,
        OVERVIEW_CAMERA_MAX_DISTANCE,
    )


def zoom_overview_camera_in():
    adjust_overview_camera_zoom(-OVERVIEW_CAMERA_ZOOM_STEP)


def zoom_overview_camera_out():
    adjust_overview_camera_zoom(OVERVIEW_CAMERA_ZOOM_STEP)


def play_detection_alarm(current_time_seconds):
    global last_detection_alarm_time_seconds
    if (
        current_time_seconds - last_detection_alarm_time_seconds
    ) < DETECTION_ALARM_COOLDOWN_SECONDS:
        return
    last_detection_alarm_time_seconds = current_time_seconds
    if alarm_sound is None:
        return
    try:
        alarm_sound.stop()
        alarm_sound.play()
    except Exception:
        pass


def any_water_drone_manual():
    if water_control_mode == CONTROL_MODE_MANUAL:
        return True
    for follower in follower_drones:
        if (
            follower.get("role") == "water"
            and follower.get("control_mode", CONTROL_MODE_AUTOMATION) == CONTROL_MODE_MANUAL
        ):
            return True
    return False


# Which keyboard key dispatches each water drone view slot ([P]=W1, [L]=W2, [K]=W3).
WATER_DISPATCH_KEYS = {1: "P", 2: "L", 3: "K"}


def get_water_view_by_slot(slot):
    """Return the camera view dict for water drone view slot 1/2/3, or None."""
    for view in get_all_drone_views():
        if view["role"] == "water" and view["slot"] == slot:
            return view
    return None


def is_water_slot_automation_controlled(water_slot):
    view = get_water_view_by_slot(water_slot)
    if view is None or is_view_damaged(view):
        return False
    return get_drone_view_control_mode(view) == CONTROL_MODE_AUTOMATION


def water_slot_can_auto_dispatch(water_slot):
    if not water_slot_has_paired_survey(water_slot):
        # Extra water drones are optional support. They do not join the paired
        # survey-water algorithm just because the team enters full auto.
        return water_slot in dispatched_water_slots or is_water_slot_automation_controlled(water_slot)
    return (
        water_drone_state.auto_dispatch_enabled
        or water_slot in dispatched_water_slots
        or is_water_slot_automation_controlled(water_slot)
    )


def any_water_slot_can_auto_dispatch():
    return any(
        water_slot_can_auto_dispatch(slot)
        for slot in range(1, team_water_drone_count + 1)
    )


def is_active_detected_hotspot(hotspot):
    return (
        hotspot is not None
        and not hotspot.root.isEmpty()
        and hotspot.detected
        and hotspot.suppression_state == FIRE_STATE_ACTIVE
    )


def is_water_slot_dispatched(water_slot):
    if not water_slot_has_paired_survey(water_slot):
        return (
            water_slot in dispatched_water_slots
            or get_water_dispatch_target(water_slot) is not None
        )
    return (
        water_drone_state.auto_dispatch_enabled
        or water_slot in dispatched_water_slots
        or get_water_dispatch_target(water_slot) is not None
    )


def water_slot_needs_operator_dispatch(water_slot):
    if not water_slot_has_paired_survey(water_slot):
        return False
    view = get_water_view_by_slot(water_slot)
    if view is None or is_view_damaged(view):
        return False
    return get_drone_view_control_mode(view) != CONTROL_MODE_AUTOMATION


def any_water_slot_needs_operator_dispatch():
    return any(
        water_slot_needs_operator_dispatch(slot)
        for slot in range(1, team_water_drone_count + 1)
    )


def get_alert_dispatch_hotspot():
    if is_active_detected_hotspot(dispatch_alert_hotspot):
        return dispatch_alert_hotspot
    return None


def build_water_dispatch_status_line():
    """Per-drone status for the dispatch prompt, one plain-English line each so
    it is obvious which water drones still need sending and how. AUTO describes
    flight mode; DISPATCHED describes whether that water drone has actually been
    released from holding to the fire."""
    lines = []
    for slot in range(1, team_water_drone_count + 1):
        view = get_water_view_by_slot(slot)
        key = WATER_DISPATCH_KEYS.get(slot, "?")
        if view is not None and is_view_damaged(view):
            damage_reason = format_drone_damage_reason(view["role"], view["slot"])
            lines.append(f"Water {slot}: OFFLINE ({damage_reason})")
            continue
        tank_label = format_water_tank_label_for_view(view)
        water_label = (
            "EMPTY"
            if tank_label == "EMPTY"
            else f"water {tank_label}"
        )
        mode = (
            get_drone_view_control_mode(view)
            if view is not None
            else CONTROL_MODE_AUTOMATION
        )
        reference_x, reference_y = get_water_slot_reference_xy(slot)
        active_target = get_water_dispatch_target(slot)
        if active_target is None and mode == CONTROL_MODE_AUTOMATION:
            active_target = choose_paired_survey_water_hotspot(
                slot,
                reference_x,
                reference_y,
            )

        if mode == CONTROL_MODE_MANUAL:
            pairing_label = (
                f"paired with S{slot}"
                if water_slot_has_paired_survey(slot)
                else "extra water support"
            )
            dispatch_instruction = (
                f"Press [{key}] for AUTO + dispatch."
                if key != "?"
                else "Select it and press [O] for AUTO support."
            )
            redispatch_instruction = (
                f"Press [{key}] for AUTO response."
                if key != "?"
                else "Select it and press [O] for AUTO response."
            )
            if is_water_slot_dispatched(slot):
                lines.append(
                    f"Water {slot}: MANUAL, {pairing_label}, dispatched earlier, {water_label}. {redispatch_instruction}"
                )
            else:
                lines.append(
                    f"Water {slot}: MANUAL, {pairing_label}, holding, {water_label}. {dispatch_instruction}"
                )
        elif active_target is not None or is_water_slot_dispatched(slot):
            lines.append(
                f"Water {slot}: AUTO + DISPATCHED, responding, {water_label}."
            )
        elif water_slot_has_paired_survey(slot):
            lines.append(
                f"Water {slot}: AUTO, waiting for S{slot}'s detected fire, {water_label}."
            )
        else:
            lines.append(
                f"Water {slot}: AUTO, optional extra support, {water_label}."
            )
    return "\n".join(lines)


def refresh_dispatch_after_action():
    """After a dispatch key sets a water drone to auto: re-render the prompt in
    place if any water drone still needs a dispatch decision, otherwise dismiss it."""
    if any_water_slot_needs_operator_dispatch():
        if "dispatch_alert_root" in globals() and dispatch_alert_root is not None:
            update_dispatch_alert_overlay(0.0)
    else:
        clear_dispatch_alert()


def clear_dispatch_alert():
    global dispatch_alert_flash_timer_seconds, dispatch_alert_hotspot
    global dispatch_prompt_cycle_active, dispatch_prompt_has_shown_this_round

    team_state.pending_water_dispatch_hotspot = None
    dispatch_alert_hotspot = None
    dispatch_alert_flash_timer_seconds = 0.0
    dispatch_prompt_cycle_active = False
    if dispatch_alert_root is not None:
        dispatch_alert_root.hide()


def manual_movement_command_active():
    return controller_movement_command_active() or keyboard_movement_command_active()


def mark_hotspot_detected(
    hotspot,
    current_time_seconds,
    highlight_dispatch=True,
    trigger_alarm=True,
    detected_by_survey_slot=None,
):
    global dispatch_alert_flash_timer_seconds, dispatch_alert_hotspot
    global dispatch_prompt_cycle_active
    global dispatch_prompt_has_shown_this_round
    global incident_first_detection_time_seconds
    was_detected = hotspot.detected
    hotspot.detected = True
    hotspot.detection_age_seconds = 0.0
    if (
        detected_by_survey_slot is not None
        and hotspot.detected_by_survey_slot is None
    ):
        hotspot.detected_by_survey_slot = detected_by_survey_slot
    if detected_by_survey_slot is not None:
        # Fire ownership: the survey drone that FIRST detects any hotspot of
        # this ignition event owns the whole fire. Other survey drones keep
        # their S-shaped search pattern until they detect their own fire event.
        if hotspot.fire_event_id not in fire_event_owner_slot:
            fire_event_owner_slot[hotspot.fire_event_id] = detected_by_survey_slot
        if survey_slot_may_target_hotspot(detected_by_survey_slot, hotspot):
            prime_survey_fire_response(detected_by_survey_slot, hotspot)
    set_hotspot_visual(hotspot)
    if was_detected or hotspot.suppression_state != FIRE_STATE_ACTIVE:
        return
    is_first_round_detection = incident_first_detection_time_seconds is None
    if is_first_round_detection:
        incident_first_detection_time_seconds = current_time_seconds
    if hotspot.detection_time_seconds is None:
        hotspot.detection_time_seconds = current_time_seconds
    if trigger_alarm and is_first_round_detection:
        play_detection_alarm(current_time_seconds)
    operator_dispatch_needed = any_water_slot_needs_operator_dispatch()
    if any_water_slot_can_auto_dispatch():
        request_water_support_for_hotspot(hotspot)
    should_prompt_dispatch = (
        highlight_dispatch
        and operator_dispatch_needed
        and not dispatch_prompt_has_shown_this_round
    )
    if should_prompt_dispatch:
        dispatch_prompt_has_shown_this_round = True
        dispatch_alert_flash_timer_seconds = max(
            dispatch_alert_flash_timer_seconds,
            DISPATCH_ALERT_FLASH_SECONDS,
        )
        dispatch_alert_hotspot = hotspot
        dispatch_prompt_cycle_active = True
        team_state.pending_water_dispatch_hotspot = hotspot


def get_overview_focus_point():
    map_min_x = SPAWN_X_MIN - OVERVIEW_CAMERA_MAP_EDGE_PADDING_METERS
    map_max_x = SPAWN_X_MAX + OVERVIEW_CAMERA_MAP_EDGE_PADDING_METERS
    map_min_y = SPAWN_Y_MIN - OVERVIEW_CAMERA_MAP_EDGE_PADDING_METERS
    map_max_y = SPAWN_Y_MAX + OVERVIEW_CAMERA_MAP_EDGE_PADDING_METERS

    focus_x = (map_min_x + map_max_x) * 0.5
    focus_y = (map_min_y + map_max_y) * 0.5
    max_drone_z = max(drone.getZ(), water_drone.getZ())
    focus_z = max(
        OVERVIEW_CAMERA_FOCUS_HEIGHT_METERS,
        max_drone_z * 0.5,
    )
    scene_span = max(
        18.0,
        map_max_x - map_min_x,
        map_max_y - map_min_y,
    )
    return focus_x, focus_y, focus_z, scene_span


def apply_thermal_view():
    app.setBackgroundColor(*(THERMAL_BACKGROUND_COLOR if thermal_view_enabled else DEFAULT_BACKGROUND_COLOR))

    terrain_color = (0.18, 0.24, 0.34, 1.0)
    foliage_color = (0.12, 0.2, 0.18, 1.0)
    drone_color = (0.9, 0.92, 0.95, 1.0)
    truck_color = (0.42, 0.42, 0.42, 1.0)

    if thermal_view_enabled:
        ground.setColorScale(*terrain_color)
        for tree_node in trees:
            tree_node.setColorScale(*foliage_color)
        for grass_node in grass_clumps:
            if THERMAL_HIDE_GRASS:
                grass_node.hide()
            else:
                grass_node.setColorScale(*foliage_color)
        for truck_state in fire_truck_states:
            truck_state.root.setColorScale(*truck_color)
        drone.setColorScale(*drone_color)
        water_drone.setColorScale(*drone_color)
    else:
        ground.clearColorScale()
        for tree_node in trees:
            tree_node.clearColorScale()
        for grass_node in grass_clumps:
            grass_node.show()
            grass_node.clearColorScale()
        for truck_state in fire_truck_states:
            truck_state.root.clearColorScale()
        drone.clearColorScale()
        water_drone.clearColorScale()

    for hotspot in fire_hotspots:
        set_hotspot_visual(hotspot)


def set_thermal_view(enabled):
    global thermal_view_enabled
    thermal_view_enabled = bool(enabled)
    apply_thermal_view()


def toggle_thermal_view():
    set_thermal_view(not thermal_view_enabled)


def reset_motion_state_for_control_mode(drone_root, drone_motion_state):
    drone_motion_state.input_command_buffer.clear()
    drone_motion_state.applied_cmd = AxisState()
    drone_motion_state.target_altitude_meters = drone_root.getZ()
    drone_motion_state.yaw_radians = radians(drone_root.getH())
    drone_motion_state.heading_hold_radians = radians(drone_root.getH())
    drone_motion_state.pitch_rate_rad_per_second = 0.0
    drone_motion_state.roll_rate_rad_per_second = 0.0
    drone_motion_state.yaw_rate_rad_per_second = 0.0
    drone_motion_state.last_body_right_speed = 0.0
    drone_motion_state.filtered_body_right_accel = 0.0
    drone_motion_state.roll_sway_body_speed = 0.0
    drone_motion_state.last_body_right_command_speed = 0.0
    drone_motion_state.roll_anticipation_body_speed = 0.0
    drone_motion_state.vertical_command_was_active = False


# ---------------------------------------------------------------
# Drone damage. MANUAL flying has NO collision-avoidance assist: if a
# manually flown drone gets within the crash radius of any other drone,
# both are damaged and go OFFLINE for the rest of the round.
# (Defined before the module-level startup calls below that use it.)
# ---------------------------------------------------------------
damaged_drone_keys = set()
damaged_drone_reasons = {}
drone_damage_events = []

# Crash animation: a damaged drone falls out of the sky, lands tipped over
# with stopped propellers, and stays there for the rest of the round.
CRASH_FALL_GRAVITY_METERS_PER_SECOND_SQ = 9.81
CRASH_REST_HEIGHT_ABOVE_GROUND_METERS = 0.55
CRASH_TILT_RATE_DEGREES_PER_SECOND = 160.0
COLLISION_ALERT_DURATION_SECONDS = 4.0
DRONE_BRANCH_IMPACT_QUERY_RADIUS_METERS = 1.7
DRONE_BRANCH_IMPACT_VERTICAL_MARGIN_METERS = 0.35
DRONE_GROUND_IMPACT_CLEARANCE_METERS = 0.9
drone_crash_states = {}
collision_alert_hide_at_seconds = -1.0
collision_alert_labels = []
DAMAGE_REASON_COLLISION = "collision"
DAMAGE_REASON_BRANCH_IMPACT = "branch_impact"
DAMAGE_REASON_GROUND_IMPACT = "ground_impact"
DAMAGE_REASON_KILL_SWITCH = "kill_switch"


def is_drone_damaged(role, slot):
    return (role, slot) in damaged_drone_keys


def is_view_damaged(view):
    return view is not None and is_drone_damaged(view["role"], view["slot"])


def get_drone_damage_reason(role, slot):
    return damaged_drone_reasons.get((role, slot), DAMAGE_REASON_COLLISION)


def format_drone_label(role, slot):
    return f"{'Survey' if role == 'survey' else 'Water'} drone {slot}"


def format_drone_damage_reason(role, slot):
    reason = get_drone_damage_reason(role, slot)
    if reason == DAMAGE_REASON_KILL_SWITCH:
        return "kill switch shutdown"
    if reason == DAMAGE_REASON_BRANCH_IMPACT:
        return "branch impact"
    if reason == DAMAGE_REASON_GROUND_IMPACT:
        return "ground impact"
    return "drone collision"


def wrap_alert_body_text(body, max_chars=56):
    wrapped_lines = []
    for raw_line in body.splitlines():
        words = raw_line.split()
        if not words:
            wrapped_lines.append("")
            continue
        line = words[0]
        for word in words[1:]:
            if len(line) + 1 + len(word) > max_chars:
                wrapped_lines.append(line)
                line = word
            else:
                line += " " + word
        wrapped_lines.append(line)
    return "\n".join(wrapped_lines), wrapped_lines


def layout_collision_alert(title, body):
    global collision_alert_backdrop
    if "collision_alert_root" not in globals() or collision_alert_root is None:
        return body

    wrapped_body, body_lines = wrap_alert_body_text(body)
    max_line_chars = max(
        [len(title)] + [len(line) for line in body_lines],
        default=len(title),
    )
    frame_width = clamp(0.44 + (max_line_chars * 0.0155), 0.76, 1.46)
    frame_height = 0.145 + (len(body_lines) * 0.050)
    frame_half_width = frame_width * 0.5
    frame_half_height = frame_height * 0.5

    if collision_alert_backdrop is not None and not collision_alert_backdrop.isEmpty():
        collision_alert_backdrop.removeNode()
    backdrop = CardMaker("collision_alert_backdrop")
    backdrop.setFrame(
        -frame_half_width,
        frame_half_width,
        -frame_half_height,
        frame_half_height,
    )
    collision_alert_backdrop = collision_alert_root.attachNewNode(backdrop.generate())
    collision_alert_backdrop.setColor(0.62, 0.05, 0.04, 0.9)
    collision_alert_backdrop.setTransparency(TransparencyAttrib.MAlpha)
    collision_alert_backdrop.setDepthWrite(False)
    collision_alert_backdrop.setDepthTest(False)
    collision_alert_backdrop.setBin("fixed", 152)
    return wrapped_body


def play_damage_alert_alarm():
    if alarm_sound is None:
        return
    try:
        alarm_sound.stop()
        alarm_sound.play()
    except Exception:
        pass


def show_drone_damage_alert(labels, damage_reason):
    """Big red banner + alarm for a few seconds when a drone is damaged."""
    global collision_alert_hide_at_seconds
    if "collision_alert_root" not in globals() or collision_alert_root is None:
        return
    now = motion_state.sim_time_seconds
    if damage_reason == DAMAGE_REASON_COLLISION:
        if (
            now > collision_alert_hide_at_seconds
            or collision_alert_text.getText() != "DRONE COLLISION"
        ):
            collision_alert_labels.clear()
        for label in labels:
            upper_label = label.upper()
            if upper_label not in collision_alert_labels:
                collision_alert_labels.append(upper_label)
        title = "DRONE COLLISION"
        label_text = " + ".join(collision_alert_labels)
        body = (
            f"{label_text} collided and are OFFLINE for the rest of the round.\n"
            "Only the colliding drones go down; the rest of the team continues."
        )
    else:
        collision_alert_labels.clear()
        label_text = " + ".join(label.upper() for label in labels)
        if damage_reason == DAMAGE_REASON_BRANCH_IMPACT:
            title = "BRANCH IMPACT"
            body = (
                f"{label_text} hit the tree canopy and is OFFLINE for the rest of the round.\n"
                "Only this drone goes down; the rest of the team continues."
            )
        elif damage_reason == DAMAGE_REASON_GROUND_IMPACT:
            title = "GROUND IMPACT"
            body = (
                f"{label_text} hit the ground and is OFFLINE for the rest of the round.\n"
                "Only this drone goes down; the rest of the team continues."
            )
        else:
            title = "DRONE OFFLINE"
            body = (
                f"{label_text} is OFFLINE for the rest of the round.\n"
                "The rest of the team continues."
            )
    body = layout_collision_alert(title, body)
    collision_alert_hide_at_seconds = now + COLLISION_ALERT_DURATION_SECONDS
    collision_alert_text.setText(title)
    collision_alert_body_text.setText(body)
    collision_alert_root.show()
    play_damage_alert_alarm()


def show_collision_alert(labels):
    show_drone_damage_alert(labels, DAMAGE_REASON_COLLISION)


def show_kill_switch_alert(drone_count):
    """Big red banner for the operator-triggered emergency shutdown."""
    global collision_alert_hide_at_seconds
    if "collision_alert_root" not in globals() or collision_alert_root is None:
        return
    collision_alert_labels.clear()
    collision_alert_hide_at_seconds = (
        motion_state.sim_time_seconds + COLLISION_ALERT_DURATION_SECONDS
    )
    collision_alert_text.setText("KILL SWITCH ACTIVATED")
    body = layout_collision_alert(
        "KILL SWITCH ACTIVATED",
        f"Emergency stop sent to {drone_count} drone(s).\n"
        "All drones are OFFLINE and dropping to the ground.",
    )
    collision_alert_body_text.setText(body)
    collision_alert_root.show()
    play_damage_alert_alarm()


def update_collision_alert_overlay():
    if "collision_alert_root" not in globals() or collision_alert_root is None:
        return
    if motion_state.sim_time_seconds > collision_alert_hide_at_seconds:
        collision_alert_root.hide()


def mark_drone_damaged(role, slot, damage_reason=DAMAGE_REASON_COLLISION, show_alert=True):
    key = (role, slot)
    label = format_drone_label(role, slot)
    if key in damaged_drone_keys:
        if damage_reason == DAMAGE_REASON_KILL_SWITCH:
            damaged_drone_reasons[key] = damage_reason
        return False
    damaged_drone_keys.add(key)
    damaged_drone_reasons[key] = damage_reason
    drone_crash_states[key] = {
        "fall_speed": 0.0,
        "resting": False,
        "tilt_p": random.uniform(48.0, 80.0) * random.choice((-1.0, 1.0)),
        "tilt_r": random.uniform(35.0, 70.0) * random.choice((-1.0, 1.0)),
        "prop_r": random.uniform(0.0, 90.0),
    }
    drone_damage_events.append((motion_state.sim_time_seconds, label, damage_reason))
    log_prefix = {
        DAMAGE_REASON_KILL_SWITCH: "KILL SWITCH",
        DAMAGE_REASON_BRANCH_IMPACT: "BRANCH IMPACT",
        DAMAGE_REASON_GROUND_IMPACT: "GROUND IMPACT",
    }.get(damage_reason, "COLLISION")
    print(
        f"[real-sim] {log_prefix}: {label} damaged at "
        f"t={motion_state.sim_time_seconds:.1f}s, OFFLINE for the round."
    )
    if show_alert:
        if damage_reason == DAMAGE_REASON_KILL_SWITCH:
            show_kill_switch_alert(1)
        else:
            show_drone_damage_alert([label], damage_reason)
    if "status_text" in globals():
        update_status_overlay()
    return True


def get_view_crash_rig_parts(view):
    """(visual_node, propellers, motion_state) for any drone view."""
    if view["follower"] is not None:
        follower = view["follower"]
        return follower["visual"], follower["propellers"], follower["motion_state"]
    if view["role"] == "survey":
        return drone_visual, drone_propellers, motion_state
    return water_drone_visual, water_drone_propellers, water_motion_state


def approach_angle(current, target, max_step):
    delta = target - current
    if abs(delta) <= max_step:
        return target
    return current + (max_step if delta > 0 else -max_step)


def update_crashed_drones(dt):
    """Runs after all drone physics: animate damaged drones falling to the
    ground, tipping over, with stopped propellers."""
    if not drone_crash_states:
        return
    for view in get_all_drone_views():
        key = (view["role"], view["slot"])
        state = drone_crash_states.get(key)
        if state is None:
            continue
        root = view.get("root")
        if root is None or root.isEmpty():
            continue
        visual, propellers, drone_motion_state = get_view_crash_rig_parts(view)

        # Propellers frozen at a fixed angle (overrides this frame's spin).
        for rotor_node, _spin_direction in propellers:
            rotor_node.setR(state["prop_r"])

        # Tilt the body toward its crash attitude. Physics rewrites the
        # visual's P/R every frame, so track our own angles in the state.
        if state.get("cur_p") is None:
            state["cur_p"] = visual.getP()
            state["cur_r"] = visual.getR()
        tilt_step = CRASH_TILT_RATE_DEGREES_PER_SECOND * dt
        state["cur_p"] = approach_angle(state["cur_p"], state["tilt_p"], tilt_step)
        state["cur_r"] = approach_angle(state["cur_r"], state["tilt_r"], tilt_step)
        visual.setP(state["cur_p"])
        visual.setR(state["cur_r"])

        # Track our own altitude: the regular physics re-clamps the node's Z
        # to minimum flight altitude every frame, so reading it back would
        # stall the fall just below the clamp.
        ground = sample_ground(root.getX(), root.getY())
        ground_z = ground[0].z if ground is not None else 0.0
        rest_z = ground_z + CRASH_REST_HEIGHT_ABOVE_GROUND_METERS
        if state.get("z") is None:
            state["z"] = root.getZ()
        if state["resting"]:
            root.setZ(rest_z)
        else:
            state["fall_speed"] += CRASH_FALL_GRAVITY_METERS_PER_SECOND_SQ * dt
            state["z"] -= state["fall_speed"] * dt
            if state["z"] <= rest_z:
                state["z"] = rest_z
                state["resting"] = True
            root.setZ(state["z"])

        # Keep the hover/altitude-hold physics from fighting the fall.
        drone_motion_state.target_altitude_meters = root.getZ()
        drone_motion_state.velocity.z = 0.0


def reset_drone_damage():
    global collision_alert_hide_at_seconds
    damaged_drone_keys.clear()
    damaged_drone_reasons.clear()
    drone_damage_events.clear()
    drone_crash_states.clear()
    collision_alert_labels.clear()
    collision_alert_hide_at_seconds = -1.0
    if "collision_alert_root" in globals() and collision_alert_root is not None:
        collision_alert_root.hide()


def any_drone_automation_enabled():
    return any(
        get_drone_view_control_mode(view) == CONTROL_MODE_AUTOMATION
        for view in get_all_drone_views()
    )


def set_drone_view_control_mode(view, new_mode):
    global control_mode, water_control_mode
    global automation_target_hotspot, automation_search_waypoint
    global survey_lead_patrol_sector_label, survey_lead_patrol_waypoints
    global survey_lead_patrol_waypoint_index
    global camera_manual_override_active, camera_yaw_offset_degrees
    if new_mode not in (CONTROL_MODE_MANUAL, CONTROL_MODE_AUTOMATION):
        return
    if view is None:
        return
    if is_view_damaged(view):
        # Offline drones cannot be revived or re-moded this round.
        return

    if view["follower"] is not None:
        view["follower"]["control_mode"] = new_mode
        if new_mode == CONTROL_MODE_AUTOMATION:
            # First [O] on this drone releases it from standby; from then on
            # it sweeps its own sector regardless of the lead's mode.
            view["follower"]["automation_started"] = True
            view["follower"]["target_hotspot"] = None
            view["follower"]["search_waypoint"] = None
            view["follower"]["orbit_hotspot_id"] = None
            view["follower"]["tracked_hotspot_id"] = None
            if view["follower"].get("role") == "survey":
                view["follower"]["patrol_sector_label"] = None
                refresh_follower_survey_patrol(view["follower"])
        else:
            view["follower"]["target_hotspot"] = None
            view["follower"]["tracked_hotspot_id"] = None
            if view["follower"].get("role") == "water":
                clear_water_suppression_visual(view["follower"])
        reset_motion_state_for_control_mode(
            view["root"],
            view["follower"]["motion_state"],
        )
    elif view["role"] == "water":
        water_control_mode = new_mode
        if new_mode == CONTROL_MODE_AUTOMATION:
            water_drone_state.tracked_hotspot_id = None
            team_state.water_target_hotspot = None
        reset_motion_state_for_control_mode(water_drone, water_motion_state)
    else:
        control_mode = new_mode
        automation_target_hotspot = None
        automation_search_waypoint = None
        if new_mode == CONTROL_MODE_AUTOMATION:
            survey_lead_patrol_sector_label = None
            survey_lead_patrol_waypoints = tuple()
            survey_lead_patrol_waypoint_index = 0
        reset_motion_state_for_control_mode(drone, motion_state)

    if view["role"] == "water" and new_mode == CONTROL_MODE_AUTOMATION:
        # Handing a water drone back to automation cancels its manual spray.
        manual_water_spray_slots.discard(view["slot"])
    if view["root"] is get_camera_target_node():
        camera_manual_override_active = False
        camera_yaw_offset_degrees = 0.0
        if new_mode == CONTROL_MODE_AUTOMATION:
            snap_camera_angle_to_selected_automation_heading()
    team_state.rolling_patrol_timer_seconds = 0.0
    team_state.survey_focus_hotspot = None
    team_state.pending_water_dispatch_hotspot = None
    if AUTOMATION_ENABLES_THERMAL_VIEW:
        set_thermal_view(any_drone_automation_enabled())
    if "status_text" in globals():
        update_status_overlay()
    if fire_map_panel is not None:
        update_fire_map_popup()


def set_control_mode(new_mode):
    set_drone_view_control_mode(get_selected_drone_view(), new_mode)


def set_manual_mode():
    set_control_mode(CONTROL_MODE_MANUAL)


def set_automation_mode():
    set_control_mode(CONTROL_MODE_AUTOMATION)


def toggle_control_mode():
    selected_mode = get_drone_view_control_mode(get_selected_drone_view())
    if selected_mode == CONTROL_MODE_MANUAL:
        set_automation_mode()
    else:
        set_manual_mode()


def set_full_automation():
    """Blue screen button: automates the survey team and the paired water team.

    If water drones outnumber survey drones, extra water stays in manual hold
    until the operator explicitly selects/dispatches it.
    """
    if pregame_active or game_over_triggered or results_page_active:
        return
    set_hover_all(False)
    for view in get_all_drone_views():
        if is_view_damaged(view):
            continue
        if view["role"] == "water" and not water_slot_has_paired_survey(view["slot"]):
            set_drone_view_control_mode(view, CONTROL_MODE_MANUAL)
            continue
        set_drone_view_control_mode(view, CONTROL_MODE_AUTOMATION)
    water_drone_state.auto_dispatch_enabled = True
    water_dispatch_targets.clear()
    for water_slot in range(1, team_water_drone_count + 1):
        if water_slot_has_paired_survey(water_slot):
            dispatched_water_slots.add(water_slot)
        else:
            dispatched_water_slots.discard(water_slot)
    auto_request_water_support_for_detected_hotspots()
    if "status_text" in globals():
        update_status_overlay()
    if fire_map_panel is not None:
        update_fire_map_popup()


# --- Hover-all (yellow button) and kill switch (red button) -----------------
hover_all_active = False
_hover_all_poses = {}


def set_hover_all(active):
    """Yellow button: freeze every drone hovering in place. Capture each drone's
    pose on activation so the per-frame freeze can pin them exactly there."""
    global hover_all_active
    hover_all_active = bool(active)
    _hover_all_poses.clear()
    if hover_all_active:
        for view in get_all_drone_views():
            if is_view_damaged(view):
                continue
            if get_drone_view_control_mode(view) != CONTROL_MODE_MANUAL:
                set_drone_view_control_mode(view, CONTROL_MODE_MANUAL)
            if view["role"] == "water":
                manual_water_spray_slots.discard(view["slot"])
            root = view["root"]
            if root is not None and not root.isEmpty():
                _hover_all_poses[id(root)] = (root.getPos(), root.getHpr())
        water_drone_state.auto_dispatch_enabled = False
        dispatched_water_slots.clear()
        water_dispatch_targets.clear()
        clear_dispatch_alert()
    if "status_text" in globals():
        update_status_overlay()


def toggle_hover_all():
    if pregame_active or game_over_triggered or results_page_active:
        return
    set_hover_all(not hover_all_active)


def apply_hover_all_freeze():
    """Pin every (non-crashed) drone to its captured hover pose and kill its
    velocity, so the whole team holds station."""
    for view in get_all_drone_views():
        if is_view_damaged(view):
            continue
        root = view["root"]
        if root is None or root.isEmpty():
            continue
        pose = _hover_all_poses.get(id(root))
        if pose is None:
            _hover_all_poses[id(root)] = (root.getPos(), root.getHpr())
            continue
        root.setPos(pose[0])
        root.setHpr(pose[1])
        drone_motion = get_view_crash_rig_parts(view)[2]
        if drone_motion is not None:
            drone_motion.velocity.x = 0.0
            drone_motion.velocity.y = 0.0
            drone_motion.velocity.z = 0.0
            drone_motion.target_altitude_meters = root.getZ()


def trigger_kill_switch():
    """Red button: emergency stop. Every drone loses power and drops to the
    ground (reuses the collision crash-and-fall animation)."""
    if pregame_active or game_over_triggered or results_page_active:
        return
    set_hover_all(False)
    affected_count = 0
    for view in get_all_drone_views():
        affected_count += 1
        mark_drone_damaged(
            view["role"],
            view["slot"],
            damage_reason=DAMAGE_REASON_KILL_SWITCH,
            show_alert=False,
        )
    show_kill_switch_alert(affected_count)
    if "status_text" in globals():
        update_status_overlay()


def set_speed_preset(new_speed_preset):
    global current_speed_preset
    if new_speed_preset not in DRONE_SPEED_PRESET_MULTIPLIERS:
        return
    current_speed_preset = new_speed_preset
    if "status_text" in globals():
        update_status_overlay()
    if fire_map_panel is not None:
        update_fire_map_popup()


def set_slow_speed():
    set_speed_preset(DRONE_SPEED_PRESET_SLOW)


def set_normal_speed():
    set_speed_preset(DRONE_SPEED_PRESET_NORMAL)


def set_fast_speed():
    set_speed_preset(DRONE_SPEED_PRESET_FAST)


def get_team_algorithm_label():
    return TEAM_ALGORITHM_LABELS.get(team_state.algorithm_mode, team_state.algorithm_mode.upper())


def cycle_team_algorithm():
    try:
        current_index = TEAM_ALGORITHM_ORDER.index(team_state.algorithm_mode)
    except ValueError:
        current_index = 0
    team_state.algorithm_mode = TEAM_ALGORITHM_ORDER[
        (current_index + 1) % len(TEAM_ALGORITHM_ORDER)
    ]
    team_state.rolling_patrol_timer_seconds = 0.0
    team_state.survey_focus_hotspot = None


def additional_burned_cell_count():
    return max(0, len(fire_burned_cells) - initial_burned_cell_count)


def additional_burned_area_square_meters():
    return additional_burned_cell_count() * (FIRE_BURN_CELL_SIZE_METERS ** 2)


def mapping_coverage_score():
    if not fire_hotspots:
        return 0.0
    mapped_equivalent = sum(
        hotspot_mapping_fraction(hotspot)
        for hotspot in fire_hotspots
    )
    return clamp(mapped_equivalent / len(fire_hotspots), 0.0, 1.0)


def suppression_progress_score():
    detected_hotspots = [
        hotspot
        for hotspot in fire_hotspots
        if hotspot.detected
    ]
    if not detected_hotspots:
        return 0.0

    contain_delay = max(1.0, FIRE_SUPPRESSION_CONTAIN_DELAY_SECONDS)
    extinguish_delay = max(1.0, FIRE_SUPPRESSION_EXTINGUISH_DELAY_SECONDS)
    hotspot_progress_values = []
    for hotspot in detected_hotspots:
        if hotspot.suppression_state == FIRE_STATE_OUT:
            hotspot_progress_values.append(1.0)
            continue
        if hotspot.suppression_state == FIRE_STATE_BURNED:
            hotspot_progress_values.append(0.0)
            continue
        if hotspot.suppression_state == FIRE_STATE_CONTAINED:
            contained_progress = clamp(
                (hotspot.suppression_work_seconds - contain_delay) / extinguish_delay,
                0.0,
                1.0,
            )
            hotspot_progress_values.append(0.5 + (contained_progress * 0.5))
            continue
        active_progress = clamp(
            hotspot.suppression_work_seconds / contain_delay,
            0.0,
            1.0,
        )
        if hotspot.ground_firefighter_engaged:
            active_progress = max(
                active_progress,
                PERFORMANCE_FIREFIGHTER_SUPPRESSION_BASE_SCORE,
            )
        hotspot_progress_values.append(active_progress * 0.5)

    if not hotspot_progress_values:
        return 0.0
    return clamp(
        sum(hotspot_progress_values) / len(hotspot_progress_values),
        0.0,
        1.0,
    )


def fire_out_control_score():
    if not fire_hotspots:
        return 0.0

    contain_delay = max(1.0, FIRE_SUPPRESSION_CONTAIN_DELAY_SECONDS)
    extinguish_delay = max(1.0, FIRE_SUPPRESSION_EXTINGUISH_DELAY_SECONDS)
    full_suppression_work = contain_delay + extinguish_delay
    hotspot_control_values = []

    for hotspot in fire_hotspots:
        if hotspot.suppression_state == FIRE_STATE_OUT:
            hotspot_control_values.append(1.0)
            continue
        if hotspot.suppression_state == FIRE_STATE_CONTAINED:
            contained_elapsed = max(
                0.0,
                hotspot.suppression_work_seconds - contain_delay,
            )
            extinguish_progress = clamp(
                contained_elapsed / extinguish_delay,
                0.0,
                1.0,
            )
            hotspot_control_values.append(
                PERFORMANCE_CONTAINED_FIRE_CONTROL_CREDIT
                + ((1.0 - PERFORMANCE_CONTAINED_FIRE_CONTROL_CREDIT) * extinguish_progress)
            )
            continue
        if hotspot.suppression_state == FIRE_STATE_ACTIVE and hotspot.detected:
            hotspot_control_values.append(
                clamp(hotspot.suppression_work_seconds / full_suppression_work, 0.0, 0.35)
            )
            continue
        hotspot_control_values.append(0.0)

    return clamp(
        sum(hotspot_control_values) / max(1, len(hotspot_control_values)),
        0.0,
        1.0,
    )


def compute_performance_matrix(current_time_seconds):
    # Four-part wildfire score:
    # - finding the fire: detection speed + coverage
    # - controlling the fire: suppression progress + putting the fire out
    coverage_score = mapping_coverage_score()
    suppression_score = suppression_progress_score()
    fire_out_score = fire_out_control_score()

    # Detection speed (M1): average how promptly EACH fire was found, across all
    # fires that have ignited. Recomputed every frame so the score keeps updating
    # as new fires ignite and as the drone detects them.
    detection_target_seconds = max(0.1, PERFORMANCE_DETECTION_TARGET_SECONDS)
    per_fire_detection_scores = []
    for hotspot in fire_hotspots:
        if hotspot.detection_time_seconds is not None:
            # Genuinely detected while active: delay = detect time - ignition time.
            fire_delay_seconds = max(
                0.0,
                hotspot.detection_time_seconds - hotspot.ignition_time_seconds,
            )
        else:
            # Not yet detected (still searching, or it burned out undetected):
            # ongoing penalty that grows the longer it stays unfound.
            fire_delay_seconds = max(
                0.0,
                current_time_seconds - hotspot.ignition_time_seconds,
            )
        per_fire_detection_scores.append(
            1.0 - clamp(fire_delay_seconds / detection_target_seconds, 0.0, 1.0)
        )

    if per_fire_detection_scores:
        detection_score = sum(per_fire_detection_scores) / len(per_fire_detection_scores)
        detection_delay_seconds = None
    else:
        detection_score = 0.5
        detection_delay_seconds = None

    survey_drone_count = get_team_survey_drone_count()
    water_drone_count = team_water_drone_count
    survey_effectiveness = team_role_effectiveness(survey_drone_count)
    water_effectiveness = team_role_effectiveness(water_drone_count)

    m1_score = clamp(detection_score * survey_effectiveness, 0.0, 1.0)
    m2_score = clamp(coverage_score * survey_effectiveness, 0.0, 1.0)
    m3_score = clamp(suppression_score * water_effectiveness, 0.0, 1.0)
    m4_score = clamp(fire_out_score * water_effectiveness, 0.0, 1.0)

    performance_weight_total = (
        PERFORMANCE_WEIGHT_M1
        + PERFORMANCE_WEIGHT_M2
        + PERFORMANCE_WEIGHT_M3
        + PERFORMANCE_WEIGHT_M4
    )
    if performance_weight_total <= 0.0:
        performance_weight_total = 1.0
    raw_performance_score = clamp(
        (
            (PERFORMANCE_WEIGHT_M1 * m1_score)
            + (PERFORMANCE_WEIGHT_M2 * m2_score)
            + (PERFORMANCE_WEIGHT_M3 * m3_score)
            + (PERFORMANCE_WEIGHT_M4 * m4_score)
        ) / performance_weight_total,
        0.0,
        1.0,
    )
    extra_drone_bonus = team_extra_drone_score_bonus()
    performance_score = clamp(
        raw_performance_score + extra_drone_bonus,
        0.0,
        1.0,
    )
    return {
        "performance_score": performance_score,
        "raw_performance_score": raw_performance_score,
        "m1_score": m1_score,
        "m2_score": m2_score,
        "m3_score": m3_score,
        "m4_score": m4_score,
        "coverage_score": coverage_score,
        "detection_score": detection_score,
        "detection_delay_seconds": detection_delay_seconds,
        "suppression_score": suppression_score,
        "fire_out_score": fire_out_score,
        "burn_control_score": fire_out_score,
        "team_total_drones": team_total_drone_count,
        "team_survey_drones": survey_drone_count,
        "team_water_drones": water_drone_count,
        "survey_effectiveness": survey_effectiveness,
        "water_effectiveness": water_effectiveness,
        "extra_drone_bonus": extra_drone_bonus,
    }


def reward_eligible():
    performance = compute_performance_matrix(motion_state.sim_time_seconds)
    return performance["performance_score"] >= PERFORMANCE_REWARD_THRESHOLD


def unresolved_detected_hotspots():
    return [
        hotspot
        for hotspot in fire_hotspots
        if hotspot.detected
        and hotspot.suppression_state in (FIRE_STATE_ACTIVE, FIRE_STATE_CONTAINED)
    ]


def active_detected_hotspots():
    return [
        hotspot
        for hotspot in fire_hotspots
        if hotspot.detected
        and hotspot.suppression_state == FIRE_STATE_ACTIVE
    ]


def unresolved_requested_hotspots():
    return [
        hotspot
        for hotspot in active_detected_hotspots()
        if hotspot.water_drone_requested
    ]


def request_water_support_for_hotspot(hotspot):
    if hotspot is None:
        return False
    if hotspot.suppression_state != FIRE_STATE_ACTIVE or not hotspot.detected:
        return False
    if hotspot.water_drone_requested:
        return False
    hotspot.water_drone_requested = True
    hotspot.water_drone_assigned = False
    hotspot.water_drone_engaged = False
    hotspot.water_drone_assignment_count = 0
    hotspot.water_drone_engagement_count = 0
    return True


def auto_request_water_support_for_detected_hotspots():
    if not any_water_slot_can_auto_dispatch():
        return
    for hotspot in active_detected_hotspots():
        request_water_support_for_hotspot(hotspot)


def hotspot_local_burned_cell_count(hotspot, search_radius_meters=20.0):
    hotspot_cell_x, hotspot_cell_y = world_to_fire_cell(
        hotspot.root.getX(),
        hotspot.root.getY(),
    )
    cell_radius = max(
        1,
        int(round(search_radius_meters / max(1.0, FIRE_BURN_CELL_SIZE_METERS))),
    )
    burned_count = 0
    for burned_cell_x, burned_cell_y in fire_burned_cells:
        if abs(burned_cell_x - hotspot_cell_x) > cell_radius:
            continue
        if abs(burned_cell_y - hotspot_cell_y) > cell_radius:
            continue
        burned_count += 1
    return burned_count


def hotspot_water_priority_key(hotspot, reference_x, reference_y):
    local_burned_cells = hotspot_local_burned_cell_count(hotspot)
    return (
        0 if hotspot.suppression_state == FIRE_STATE_ACTIVE else 1,
        -local_burned_cells,
        -hotspot.burn_age_seconds,
        -hotspot.detection_age_seconds,
        (hotspot.root.getX() - reference_x) ** 2
        + (hotspot.root.getY() - reference_y) ** 2,
    )


def choose_requested_hotspot(reference_x, reference_y):
    candidates = unresolved_requested_hotspots()
    if not candidates:
        return None
    return min(
        candidates,
        key=lambda hotspot: hotspot_water_priority_key(
            hotspot,
            reference_x,
            reference_y,
        ),
    )


def get_water_source_candidates():
    auto_request_water_support_for_detected_hotspots()
    if any_water_slot_can_auto_dispatch():
        return active_detected_hotspots()
    return unresolved_requested_hotspots()


def water_source_sort_key(hotspot, reference_x, reference_y):
    return (
        0 if hotspot.water_drone_requested else 1,
        hotspot.ignition_time_seconds,
        -hotspot.burn_age_seconds,
        hotspot_water_priority_key(hotspot, reference_x, reference_y),
    )


def choose_water_support_hotspot_for_slot(water_slot, reference_x, reference_y):
    candidates = get_water_source_candidates()
    if not candidates:
        return None
    candidates = sorted(
        candidates,
        key=lambda hotspot: water_source_sort_key(hotspot, reference_x, reference_y),
    )
    if len(candidates) == 1:
        return candidates[0]
    slot_index = min(max(0, water_slot - 1), len(candidates) - 1)
    return candidates[slot_index]


def choose_water_support_hotspot(reference_x, reference_y):
    return choose_water_support_hotspot_for_slot(1, reference_x, reference_y)


# --- #6: water drone i is paired to survey drone i ---------------------------
WATER_FOLLOWS_PAIRED_SURVEY = True
WATER_FOLLOW_TRAIL_METERS = 16.0


def get_paired_survey_slot(water_slot):
    return water_slot


def water_slot_has_paired_survey(water_slot):
    if water_slot < 1 or water_slot > get_team_survey_drone_count():
        return False
    survey_view = get_drone_view_for_role_slot("survey", water_slot)
    return survey_view is not None and not is_view_damaged(survey_view)


def get_fire_region_slot(hotspot):
    """Which survey REGION (x-strip sector) a fire sits in. Water drone i covers
    region i, matching survey drone i's sector."""
    survey_count = max(1, get_team_survey_drone_count())
    sector_width = (SPAWN_X_MAX - SPAWN_X_MIN) / survey_count
    if sector_width <= 0.0:
        return 1
    index = int((hotspot.root.getX() - SPAWN_X_MIN) / sector_width)
    return max(1, min(survey_count, index + 1))


def water_slot_matches_hotspot_owner(water_slot, hotspot):
    if not is_active_detected_hotspot(hotspot):
        return False
    paired_survey_slot = get_paired_survey_slot(water_slot)
    return survey_slot_owns_hotspot_event(paired_survey_slot, hotspot)


def choose_paired_water_candidate(water_slot, reference_x, reference_y):
    candidates = [
        hotspot
        for hotspot in active_detected_hotspots()
        if water_slot_matches_hotspot_owner(water_slot, hotspot)
    ]
    if not candidates:
        return None
    return min(
        candidates,
        key=lambda hotspot: water_source_sort_key(hotspot, reference_x, reference_y),
    )


def choose_paired_survey_water_hotspot(water_slot, reference_x, reference_y):
    """Pick the fire for this water drone.

    W1 follows S1, W2 follows S2, etc. If there is no same-number survey drone
    for this water slot, the water drone is unpaired: it waits in manual/hold,
    and only becomes general extra firefighting support when automated.
    """
    if not water_slot_can_auto_dispatch(water_slot):
        return None

    if not water_slot_has_paired_survey(water_slot):
        return choose_water_support_hotspot_for_slot(
            water_slot,
            reference_x,
            reference_y,
        )

    dispatched_target = get_water_dispatch_target(water_slot)
    if water_slot_matches_hotspot_owner(water_slot, dispatched_target):
        return dispatched_target
    return choose_paired_water_candidate(water_slot, reference_x, reference_y)


SURVEY_FOCUS_ORBIT_RADIUS_METERS = 14.0


def is_survey_view_sweeping(view):
    """True only when a survey drone is actively automating (so its planned
    route should be drawn). A follower in standby (not yet released with [O]) and
    a manually flown lead are NOT sweeping, so no route is shown for them - this
    keeps the routes consistent (none appear until you enable automation)."""
    if view["role"] != "survey":
        return False
    if get_drone_view_control_mode(view) != CONTROL_MODE_AUTOMATION:
        return False
    follower = view.get("follower")
    if follower is not None:
        return bool(follower.get("automation_started", False))
    return True


def survey_slot_owns_hotspot_event(survey_slot, hotspot):
    """Whether this survey drone owns the detected fire event.

    Prefer the survey that actually detected this hotspot. Event ownership is
    only a fallback for spread children that have not been detected directly.
    """
    detected_slot = getattr(hotspot, "detected_by_survey_slot", None)
    if detected_slot is not None:
        return int(detected_slot) == int(survey_slot)

    fire_event_id = getattr(hotspot, "fire_event_id", 1)
    owner_slot = fire_event_owner_slot.get(fire_event_id)
    if owner_slot is not None:
        return owner_slot == survey_slot
    return get_hotspot_primary_survey_slot(hotspot) == survey_slot


def get_survey_focus_hotspot(survey_slot, reference_x, reference_y):
    """The detected fire a survey drone is currently working. None while the
    drone is still searching its original sector."""
    candidates = []
    for hotspot in unresolved_detected_hotspots():
        if survey_slot_owns_hotspot_event(survey_slot, hotspot):
            candidates.append(hotspot)
    if not candidates:
        return None
    return min(
        candidates,
        key=lambda hotspot: (hotspot.root.getX() - reference_x) ** 2
        + (hotspot.root.getY() - reference_y) ** 2,
    )


def get_survey_route_world_points(view):
    """Planned route for a survey drone as world (x, y) points. While searching
    it is the S-sweep; once it is working a detected fire the route CHANGES to a
    path in to that fire plus an orbit ring around it (#11/#2: route differs
    after the fire is found)."""
    slot = view["slot"]
    survey_xy = get_survey_drone_position_by_slot(slot)
    reference_x = survey_xy[0] if survey_xy else 0.0
    reference_y = survey_xy[1] if survey_xy else 0.0
    focus = get_survey_focus_hotspot(slot, reference_x, reference_y)
    if focus is not None:
        ring = []
        for step_index in range(17):
            angle = tau * step_index / 16.0
            ring.append(
                (
                    focus.root.getX() + cos(angle) * SURVEY_FOCUS_ORBIT_RADIUS_METERS,
                    focus.root.getY() + sin(angle) * SURVEY_FOCUS_ORBIT_RADIUS_METERS,
                )
            )
        points = []
        if survey_xy:
            points.append(survey_xy)
        points.append(ring[0])
        points.extend(ring)
        return points
    return [(wx, wy) for wx, wy, _ in build_survey_sector_waypoints(get_survey_search_sector(slot))]


def is_water_view_automating(view):
    if view["role"] != "water":
        return False
    if is_view_damaged(view):
        return False
    return get_drone_view_control_mode(view) == CONTROL_MODE_AUTOMATION


def get_water_view_home_xy(view):
    if view.get("follower") is not None:
        return (
            view["follower"].get("home_x", view["root"].getX()),
            view["follower"].get("home_y", view["root"].getY()),
        )
    return water_drone_state.home_x, water_drone_state.home_y


def get_water_route_world_points(view):
    """Planned route for an automated water drone. Once assigned, the route
    points to its suppression hover point beside the fire; otherwise it shows
    the home/hold point only if the drone is away from it."""
    if not is_water_view_automating(view):
        return tuple()
    root = view["root"]
    if root is None or root.isEmpty():
        return tuple()
    slot = view["slot"]
    current_xy = (root.getX(), root.getY())
    target_hotspot = get_water_dispatch_target(slot)
    if target_hotspot is None:
        target_hotspot = choose_paired_survey_water_hotspot(
            slot,
            current_xy[0],
            current_xy[1],
        )
    if target_hotspot is not None:
        hover_angle = compute_water_support_hover_angle(
            target_hotspot,
            slot,
            current_xy[0],
            current_xy[1],
        )
        hover_xy = (
            target_hotspot.root.getX()
            + cos(hover_angle) * WATER_DRONE_HOVER_HOLD_RADIUS_METERS,
            target_hotspot.root.getY()
            + sin(hover_angle) * WATER_DRONE_HOVER_HOLD_RADIUS_METERS,
        )
        fire_xy = (target_hotspot.root.getX(), target_hotspot.root.getY())
        return (current_xy, hover_xy, fire_xy)

    home_xy = get_water_view_home_xy(view)
    home_dx = current_xy[0] - home_xy[0]
    home_dy = current_xy[1] - home_xy[1]
    if (home_dx * home_dx + home_dy * home_dy) <= 4.0:
        return tuple()
    return (current_xy, home_xy)


def get_view_automation_route_world_points(view):
    if view["role"] == "survey":
        if not is_survey_view_sweeping(view):
            return tuple()
        return get_survey_route_world_points(view)
    if view["role"] == "water":
        return get_water_route_world_points(view)
    return tuple()


def get_water_follow_anchor(water_slot):
    """Loiter point for a water drone without a fire target.

    Paired water waits at its birth point until the same-number survey finds a
    fire. Extra unpaired water also waits unless the operator/manual/full-auto
    puts it into firefighting automation.
    """
    return None


def choose_pending_water_dispatch_hotspot():
    if not any_water_slot_needs_operator_dispatch():
        return None
    candidates = [
        hotspot
        for hotspot in active_detected_hotspots()
        if not hotspot.water_drone_requested
    ]
    if not candidates:
        return None
    return min(
        candidates,
        key=lambda hotspot: hotspot_water_priority_key(
            hotspot,
            drone.getX(),
            drone.getY(),
        ),
    )


def choose_detected_hotspot(reference_x, reference_y):
    candidates = unresolved_detected_hotspots()
    if not candidates:
        return None
    return min(
        candidates,
        key=lambda hotspot: (
            0 if hotspot.suppression_state == FIRE_STATE_ACTIVE else 1,
            (hotspot.root.getX() - reference_x) ** 2
            + (hotspot.root.getY() - reference_y) ** 2
        ),
    )


def is_unresolved_truck_hotspot(hotspot):
    return (
        hotspot is not None
        and not hotspot.root.isEmpty()
        and hotspot.detected
        and hotspot.suppression_state in (FIRE_STATE_ACTIVE, FIRE_STATE_CONTAINED)
    )


def get_fire_truck_assigned_hotspot_ids(exclude_truck=None):
    assigned_hotspot_ids = set()
    for truck_state in fire_truck_states:
        if exclude_truck is not None and truck_state is exclude_truck:
            continue
        if is_unresolved_truck_hotspot(truck_state.target_hotspot):
            assigned_hotspot_ids.add(id(truck_state.target_hotspot))
    return assigned_hotspot_ids


def hotspot_has_fire_truck_response(hotspot):
    return any(
        truck_state.target_hotspot is hotspot
        for truck_state in globals().get("fire_truck_states", ())
    )


def hotspot_has_engaged_fire_truck(hotspot):
    return any(
        truck_state.target_hotspot is hotspot and truck_state.engaged
        for truck_state in globals().get("fire_truck_states", ())
    )


def fire_truck_priority_key(hotspot, reference_x, reference_y):
    return (
        0 if hotspot.suppression_state == FIRE_STATE_ACTIVE else 1,
        -hotspot_local_burned_cell_count(hotspot),
        -hotspot.burn_age_seconds,
        -hotspot.detection_age_seconds,
        (hotspot.root.getX() - reference_x) ** 2
        + (hotspot.root.getY() - reference_y) ** 2,
    )


def choose_fire_truck_target(truck_state):
    assigned_hotspot_ids = get_fire_truck_assigned_hotspot_ids(
        exclude_truck=truck_state
    )
    candidates = [
        hotspot
        for hotspot in fire_hotspots
        if is_unresolved_truck_hotspot(hotspot)
        and id(hotspot) not in assigned_hotspot_ids
    ]
    if not candidates:
        return None
    return min(
        candidates,
        key=lambda hotspot: fire_truck_priority_key(
            hotspot,
            truck_state.root.getX(),
            truck_state.root.getY(),
        ),
    )


def fire_truck_fireline_target_xy(truck_state, hotspot):
    truck_x = truck_state.root.getX()
    truck_y = truck_state.root.getY()
    dx = truck_x - hotspot.root.getX()
    dy = truck_y - hotspot.root.getY()
    distance = (dx * dx + dy * dy) ** 0.5
    if distance <= 0.001:
        angle = radians((truck_state.slot * 137.0) % 360.0)
        unit_x = cos(angle)
        unit_y = sin(angle)
    else:
        unit_x = dx / distance
        unit_y = dy / distance
    target_x = clamp(
        hotspot.root.getX() + unit_x * FIRE_TRUCK_FIRELINE_STANDOFF_METERS,
        SPAWN_X_MIN + FIRE_TRUCK_TREE_CLEARANCE_METERS * 0.5,
        SPAWN_X_MAX - FIRE_TRUCK_TREE_CLEARANCE_METERS * 0.5,
    )
    target_y = clamp(
        hotspot.root.getY() + unit_y * FIRE_TRUCK_FIRELINE_STANDOFF_METERS,
        SPAWN_Y_MIN + FIRE_TRUCK_TREE_CLEARANCE_METERS * 0.5,
        SPAWN_Y_MAX - FIRE_TRUCK_TREE_CLEARANCE_METERS * 0.5,
    )
    return target_x, target_y


def move_fire_truck_toward(truck_state, target_x, target_y, dt):
    root = truck_state.root
    dx = target_x - root.getX()
    dy = target_y - root.getY()
    distance = (dx * dx + dy * dy) ** 0.5
    if distance <= 0.001:
        return distance
    travel_distance = min(distance, FIRE_TRUCK_SPEED_METERS_PER_SECOND * dt)
    root.setX(root.getX() + (dx / distance) * travel_distance)
    root.setY(root.getY() + (dy / distance) * travel_distance)
    root.setH(degrees(atan2(-dx, dy)))
    ground_sample = sample_ground(root.getX(), root.getY())
    if ground_sample is not None:
        root.setZ(ground_sample[0].z + FIRE_TRUCK_GROUND_OFFSET_METERS)
    return max(0.0, distance - travel_distance)


def update_fire_truck_fleet(dt):
    for truck_state in fire_truck_states:
        previous_target = truck_state.target_hotspot
        if not is_unresolved_truck_hotspot(truck_state.target_hotspot):
            truck_state.target_hotspot = None
            truck_state.engaged = False

        if truck_state.target_hotspot is None:
            truck_state.target_hotspot = choose_fire_truck_target(truck_state)
            truck_state.engaged = False

        if truck_state.target_hotspot is not previous_target:
            if is_unresolved_truck_hotspot(previous_target):
                set_hotspot_visual(previous_target)
            if is_unresolved_truck_hotspot(truck_state.target_hotspot):
                set_hotspot_visual(truck_state.target_hotspot)

        if truck_state.target_hotspot is None:
            remaining_distance = move_fire_truck_toward(
                truck_state,
                truck_state.home_x,
                truck_state.home_y,
                dt,
            )
            if remaining_distance <= FIRE_TRUCK_ARRIVAL_RADIUS_METERS:
                truck_state.root.setZ(truck_state.home_z)
            continue

        target_x, target_y = fire_truck_fireline_target_xy(
            truck_state,
            truck_state.target_hotspot,
        )
        remaining_distance = move_fire_truck_toward(
            truck_state,
            target_x,
            target_y,
            dt,
        )
        truck_state.engaged = remaining_distance <= FIRE_TRUCK_ARRIVAL_RADIUS_METERS


def reset_fire_truck_fleet():
    for truck_state in fire_truck_states:
        truck_state.target_hotspot = None
        truck_state.engaged = False
        if truck_state.root is not None and not truck_state.root.isEmpty():
            truck_state.root.setPos(
                truck_state.home_x,
                truck_state.home_y,
                truck_state.home_z,
            )
            truck_state.root.show()


def choose_nearest_sprayable_hotspot(reference_x, reference_y):
    """For MANUAL [J] spray: the nearest detected fire that still needs water,
    by pure distance (active OR contained). Unlike choose_detected_hotspot this
    does NOT jump to a far-away active fire, so the operator keeps spraying the
    fire they are actually hovering over - and can move on to the next one."""
    candidates = unresolved_detected_hotspots()
    if not candidates:
        return None
    return min(
        candidates,
        key=lambda hotspot: (
            (hotspot.root.getX() - reference_x) ** 2
            + (hotspot.root.getY() - reference_y) ** 2
        ),
    )


def hotspot_mapping_fraction(hotspot):
    if SURVEY_MAP_BUILD_TIME_SECONDS <= 0.0:
        return 1.0
    return min(1.0, hotspot.mapping_progress_seconds / SURVEY_MAP_BUILD_TIME_SECONDS)


def get_survey_drone_position_by_slot(survey_slot):
    if survey_slot == 1:
        if drone is not None and not drone.isEmpty():
            return drone.getX(), drone.getY()
        return None
    for follower in follower_drones:
        if (
            follower.get("role") == "survey"
            and follower.get("slot", 0) + 1 == survey_slot
        ):
            rig = follower.get("root")
            if rig is not None and not rig.isEmpty():
                return rig.getX(), rig.getY()
    return None


def survey_slot_may_target_hotspot(survey_slot, hotspot):
    """Fire ownership gate: only the detector/owner responds to a fire event.

    Other survey drones continue their original search pattern until they detect
    a separate fire event in their own area.
    """
    if getattr(hotspot, "detected_by_survey_slot", None) == survey_slot:
        return True
    if hotspot.detected and get_hotspot_survey_sector_slot(hotspot) == survey_slot:
        return True
    return survey_slot_owns_hotspot_event(survey_slot, hotspot)


def choose_survey_mapping_hotspot_for_slot(survey_slot, reference_x, reference_y):
    def _assignment_priority(hotspot):
        return 0 if survey_slot_owns_hotspot_event(survey_slot, hotspot) else 1

    candidates = [
        hotspot
        for hotspot in unresolved_detected_hotspots()
        if survey_slot_may_target_hotspot(survey_slot, hotspot)
        and (
            survey_slot_owns_hotspot_event(survey_slot, hotspot)
            or hotspot_mapping_fraction(hotspot) < 1.0
        )
    ]
    if not candidates:
        return None
    return min(
        candidates,
        key=lambda hotspot: (
            _assignment_priority(hotspot),
            0 if hotspot.suppression_state == FIRE_STATE_ACTIVE else 1,
            hotspot_mapping_fraction(hotspot),
            (hotspot.root.getX() - reference_x) ** 2
            + (hotspot.root.getY() - reference_y) ** 2,
        ),
    )


def should_survey_orbit_hotspot(hotspot, dt):
    if hotspot is None:
        team_state.rolling_patrol_timer_seconds = 0.0
        return False

    if team_state.algorithm_mode == TEAM_ALGORITHM_BASE:
        return True

    if team_state.algorithm_mode == TEAM_ALGORITHM_MAP_THEN_RESUME:
        return hotspot_mapping_fraction(hotspot) < 1.0

    cycle_duration = (
        SURVEY_ROLLING_PATROL_ORBIT_SECONDS
        + SURVEY_ROLLING_PATROL_SWEEP_SECONDS
    )
    if cycle_duration <= 0.0:
        return True
    team_state.rolling_patrol_timer_seconds = (
        team_state.rolling_patrol_timer_seconds + dt
    ) % cycle_duration
    return team_state.rolling_patrol_timer_seconds < SURVEY_ROLLING_PATROL_ORBIT_SECONDS


def get_water_slot_reference_xy(water_slot):
    view = get_water_view_by_slot(water_slot)
    if view is not None and view["root"] is not None and not view["root"].isEmpty():
        return view["root"].getX(), view["root"].getY()
    return water_drone.getX(), water_drone.getY()


def get_water_dispatch_target(water_slot):
    target_hotspot = water_dispatch_targets.get(water_slot)
    if water_slot_has_paired_survey(water_slot):
        if water_slot_matches_hotspot_owner(water_slot, target_hotspot):
            return target_hotspot
    elif is_active_detected_hotspot(target_hotspot):
        return target_hotspot
    water_dispatch_targets.pop(water_slot, None)
    return None


def choose_water_dispatch_target_for_slot(water_slot):
    reference_x, reference_y = get_water_slot_reference_xy(water_slot)
    if not water_slot_has_paired_survey(water_slot):
        return (
            get_alert_dispatch_hotspot()
            or choose_pending_water_dispatch_hotspot()
            or choose_water_support_hotspot_for_slot(water_slot, reference_x, reference_y)
        )
    for candidate in (
        get_alert_dispatch_hotspot(),
        choose_pending_water_dispatch_hotspot(),
    ):
        if water_slot_matches_hotspot_owner(water_slot, candidate):
            return candidate
    return choose_paired_water_candidate(water_slot, reference_x, reference_y)


def request_water_slot_dispatch(water_slot):
    if pregame_active or game_over_triggered:
        return False
    if water_slot > team_water_drone_count:
        return False
    dispatch_view = get_water_view_by_slot(water_slot)
    if dispatch_view is None or is_view_damaged(dispatch_view):
        return False

    target_hotspot = choose_water_dispatch_target_for_slot(water_slot)
    dispatched_water_slots.add(water_slot)
    if target_hotspot is not None:
        water_dispatch_targets[water_slot] = target_hotspot
        request_water_support_for_hotspot(target_hotspot)
    else:
        water_dispatch_targets.pop(water_slot, None)

    if get_drone_view_control_mode(dispatch_view) != CONTROL_MODE_AUTOMATION:
        set_drone_view_control_mode(dispatch_view, CONTROL_MODE_AUTOMATION)

    if water_slot == 1:
        water_drone_state.tracked_hotspot_id = None
        if target_hotspot is not None or get_water_source_candidates():
            water_drone_state.mission_active = True

    auto_request_water_support_for_detected_hotspots()
    team_state.pending_water_dispatch_hotspot = choose_pending_water_dispatch_hotspot()
    if "status_text" in globals():
        update_status_overlay()
    refresh_dispatch_after_action()
    return True


def request_water_drone_dispatch():
    request_water_slot_dispatch(1)


def restart_simulation():
    global fire_hotspots, fire_burned_cells, fire_burnable_cells
    global fire_ignitions_done
    global initial_burned_cell_count, fire_spread_timer_seconds
    global fire_effect_update_accumulator_seconds, fire_map_update_accumulator_seconds
    global dispatch_alert_update_accumulator_seconds, status_update_accumulator_seconds
    global burn_eta_update_accumulator_seconds
    global pregame_update_accumulator_seconds
    global game_over_triggered, game_end_reason, projected_full_burn_eta_seconds
    global automation_target_hotspot, automation_search_waypoint
    global dispatch_alert_flash_timer_seconds, dispatch_alert_hotspot
    global last_detection_alarm_time_seconds
    global dispatch_prompt_cycle_active
    global incident_ignition_time_seconds, incident_first_detection_time_seconds
    global performance_round_recorded, performance_graph_line_node
    global overview_camera_enabled, overview_is_operator
    global control_mode, water_control_mode
    global camera_manual_override_active, camera_yaw_offset_degrees

    app.taskMgr.remove("update")
    # Every round starts with every drone in MANUAL, so nothing begins sweeping
    # or dispatching until the operator explicitly enables automation.
    control_mode = CONTROL_MODE_MANUAL
    water_control_mode = CONTROL_MODE_MANUAL

    for hotspot in fire_hotspots:
        hotspot.root.removeNode()
    fire_hotspots = []
    reset_fire_truck_fleet()

    for hotspot_id, marker_node in list(fire_map_detected_hotspot_nodes.items()):
        label_text = marker_node.getPythonTag("label_text")
        if label_text is not None:
            label_text.destroy()
        marker_node.removeNode()
        fire_map_detected_hotspot_nodes.pop(hotspot_id, None)

    fire_burned_cells = set()
    fire_burnable_cells = build_fire_burnable_cells()
    fire_hotspots = []
    fire_ignitions_done = 0
    fire_event_owner_slot.clear()
    initial_burned_cell_count = len(fire_burned_cells)
    fire_spread_timer_seconds = 0.0
    fire_effect_update_accumulator_seconds = 0.0
    fire_map_update_accumulator_seconds = 0.0
    dispatch_alert_update_accumulator_seconds = 0.0
    status_update_accumulator_seconds = 0.0
    burn_eta_update_accumulator_seconds = 0.0
    pregame_update_accumulator_seconds = 0.0
    burn_ratio_history.clear()
    projected_full_burn_eta_seconds = None
    game_over_triggered = False
    game_end_reason = None
    incident_ignition_time_seconds = 0.0
    incident_first_detection_time_seconds = None
    performance_round_recorded = False
    performance_history.clear()
    if performance_graph_line_node is not None:
        performance_graph_line_node.removeNode()
        performance_graph_line_node = None
    automation_target_hotspot = None
    automation_search_waypoint = None

    for action_name in key_map:
        key_map[action_name] = False

    survey_start_x, survey_start_y = get_fixed_drone_spawn_xy("survey", 1)
    survey_start_z = compute_safe_altitude_at(
        survey_start_x,
        survey_start_y,
        AUTOMATION_CRUISE_ALTITUDE_METERS,
        AUTOMATION_MIN_ALTITUDE_ABOVE_GROUND_METERS,
        canopy_clearance_meters=(
            AUTOMATION_CANOPY_CLEARANCE_METERS
            if AUTOMATION_ENABLE_CANOPY_CLEARANCE
            else 0.0
        ),
    )
    drone.setPos(survey_start_x, survey_start_y, survey_start_z)
    drone.setH(0.0)
    deploy_survey_lead_to_sector_start()

    water_drone_state.home_x, water_drone_state.home_y = get_water_drone_home_xy(1)
    water_drone_state.home_z = compute_safe_altitude_at(
        water_drone_state.home_x,
        water_drone_state.home_y,
        WATER_DRONE_HOME_ALTITUDE_METERS,
        WATER_DRONE_MIN_ALTITUDE_ABOVE_GROUND_METERS,
        canopy_clearance_meters=WATER_DRONE_CANOPY_CLEARANCE_METERS,
    )
    water_drone.setPos(
        water_drone_state.home_x,
        water_drone_state.home_y,
        water_drone_state.home_z,
    )
    water_drone.setH(0.0)
    water_drone_visual.setP(0.0)
    water_drone_visual.setR(0.0)
    reset_drone_damage()
    respawn_follower_drones()
    water_drone_state.tracked_hotspot_id = None
    water_drone_state.hover_offset_angle_radians = 0.0
    water_drone_state.mission_active = False
    water_drone_state.water_liters = WATER_DRONE_TANK_CAPACITY_LITERS
    water_drone_state.auto_dispatch_enabled = False
    reset_second_water_dispatch()
    manual_water_spray_slots.clear()
    clear_water_suppression_visual()

    reset_view_switch_tracking()
    reset_live_metrics_tracking()
    hide_results_page()
    motion_state.sim_time_seconds = 0.0
    initialize_motion_state_from_drone_pose()
    water_motion_state.sim_time_seconds = 0.0
    initialize_drone_motion_state_from_pose(water_drone, water_motion_state)
    team_state.rolling_patrol_timer_seconds = 0.0
    team_state.survey_orbit_phase_radians = 0.0
    team_state.survey_focus_hotspot = None
    team_state.water_target_hotspot = None
    team_state.pending_water_dispatch_hotspot = None
    dispatch_alert_hotspot = None
    dispatch_alert_flash_timer_seconds = 0.0
    last_detection_alarm_time_seconds = -999.0
    dispatch_prompt_cycle_active = False
    dispatch_prompt_has_shown_this_round = False
    if alarm_sound is not None:
        try:
            alarm_sound.stop()
        except Exception:
            pass
    choose_next_wind_gust()
    wind_state.gust_direction_degrees = 0.0
    wind_state.gust_speed_mps = 0.0
    set_hover_all(False)
    if mode_buttons_root is not None and not pregame_active:
        mode_buttons_root.show()
    clear_planned_trajectories()
    clear_water_target_reticles()
    clear_water_hud_target_reticle()
    overview_camera_enabled = False
    overview_is_operator = False
    camera_manual_override_active = False
    camera_yaw_offset_degrees = 0.0
    apply_mission_start_camera()
    refresh_operator_fire_visibility()
    apply_thermal_view()
    update_fire_map_popup()
    update_dispatch_alert_overlay(0.0)
    update_status_overlay()
    app.taskMgr.add(update, "update")


def apply_startup_demo_profile():
    set_speed_preset(AUTOMATION_START_SPEED_PRESET)
    if AUTOMATION_STARTS_IN_AUTOMATION:
        set_automation_mode()
    else:
        set_manual_mode()


def bind_key_pairs():
    key_bindings = {
        "a": "left",
        "d": "right",
        "w": "forward",
        "s": "backward",
        "q": "up",
        "e": "down",
        "page_down": "down",
    }

    for keyboard_key, action in key_bindings.items():
        app.accept(keyboard_key, set_key, [action, True])
        app.accept(f"{keyboard_key}-up", set_key, [action, False])


def rebuild_pregame_backdrop(frame_x0, frame_x1, frame_z0, frame_z1):
    global pregame_backdrop_np, pregame_accent_np
    if pregame_root is None:
        return
    if pregame_backdrop_np is not None and not pregame_backdrop_np.isEmpty():
        pregame_backdrop_np.removeNode()
    if pregame_accent_np is not None and not pregame_accent_np.isEmpty():
        pregame_accent_np.removeNode()

    backdrop = CardMaker("pregame_backdrop")
    backdrop.setFrame(frame_x0, frame_x1, frame_z0, frame_z1)
    pregame_backdrop_np = pregame_root.attachNewNode(backdrop.generate())
    pregame_backdrop_np.setColor(0.025, 0.045, 0.065, 0.93)
    pregame_backdrop_np.setTransparency(TransparencyAttrib.MAlpha)
    pregame_backdrop_np.setDepthWrite(False)
    pregame_backdrop_np.setDepthTest(False)
    pregame_backdrop_np.setBin("fixed", 200)

    accent = CardMaker("pregame_accent")
    accent.setFrame(frame_x0, frame_x1, frame_z1 - 0.05, frame_z1)
    pregame_accent_np = pregame_root.attachNewNode(accent.generate())
    pregame_accent_np.setColor(1.0, 0.5, 0.18, 0.95)
    pregame_accent_np.setTransparency(TransparencyAttrib.MAlpha)
    pregame_accent_np.setDepthWrite(False)
    pregame_accent_np.setDepthTest(False)
    pregame_accent_np.setBin("fixed", 201)


def layout_pregame_overlay(message_text):
    if pregame_title is None or pregame_text is None:
        return
    lines = message_text.splitlines()
    max_line_chars = max((len(line) for line in lines), default=28)
    frame_width = clamp(0.50 + (max_line_chars * 0.0125), 1.06, 1.14)
    frame_height = clamp(0.20 + (len(lines) * 0.036), 0.48, 0.80)
    frame_x0 = -frame_width * 0.5
    frame_x1 = frame_width * 0.5
    frame_z1 = 0.50
    frame_z0 = frame_z1 - frame_height
    rebuild_pregame_backdrop(frame_x0, frame_x1, frame_z0, frame_z1)


def update_pregame_overlay():
    if pregame_text is None:
        return
    survey_count = get_team_survey_drone_count()
    setup_header = (
        "Round complete. Choose next team, then launch.\n\n"
        if round_restart_pending_from_setup
        else ""
    )
    message_text = (
        f"{setup_header}"
        f"Team: {team_total_drone_count} drones"
        f" ({survey_count}S/{team_water_drone_count}W)\n"
        "[G] size   [1]+S   [2]+W\n\n"
        "HOW TO PLAY\n"
        "Keyboard:\n"
        "W/A/S/D move   Q/E altitude\n"
        f"{format_radio_pregame_controls()}"
        "Views: 1-6 / map click   Mode: M/O\n"
        "Water: P/L/K call   J spray   TAB map\n\n"
        "Survey finds hidden fires.\n"
        "Paired water follows; extras wait for O.\n\n"
        "[ENTER] launch"
    )
    pregame_text.setText(message_text)
    layout_pregame_overlay(message_text)


def show_pregame_setup_after_round():
    global pregame_active, round_restart_pending_from_setup
    pregame_active = True
    round_restart_pending_from_setup = True
    for action_name in key_map:
        key_map[action_name] = False
    if dispatch_alert_root is not None:
        dispatch_alert_root.hide()
    clear_water_suppression_visual()
    clear_water_target_reticles()
    clear_water_hud_target_reticle()
    for follower in follower_drones:
        if follower.get("role") == "water":
            clear_water_suppression_visual(follower)
    if pregame_root is not None:
        pregame_root.show()
    if mode_buttons_root is not None:
        mode_buttons_root.hide()
    update_pregame_overlay()
    if fire_map_panel is not None:
        update_fire_map_popup()
    update_performance_metrics_visibility()


def build_pregame_overlay():
    global pregame_root, pregame_backdrop_np, pregame_accent_np
    global pregame_title, pregame_text

    pregame_root = app.aspect2d.attachNewNode("pregame_root")
    pregame_root.setBin("fixed", 200)

    pregame_title = OnscreenText(
        text="MISSION SETUP",
        parent=pregame_root,
        pos=(0.0, 0.375),
        scale=0.044,
        fg=(1.0, 0.86, 0.5, 1.0),
        align=TextNode.ACenter,
        mayChange=False,
    )
    pregame_title.setBin("fixed", 202)
    pregame_title.setDepthWrite(False)
    pregame_title.setDepthTest(False)

    pregame_text = OnscreenText(
        text="",
        parent=pregame_root,
        pos=(-0.5, 0.315),
        scale=0.030,
        fg=(0.92, 0.95, 1.0, 1.0),
        align=TextNode.ALeft,
        mayChange=True,
        wordwrap=46.0,
    )
    pregame_text.setBin("fixed", 202)
    pregame_text.setDepthWrite(False)
    pregame_text.setDepthTest(False)
    update_pregame_overlay()


def start_game_from_pregame():
    global pregame_active, round_restart_pending_from_setup
    global tx12_controller_channel_kill_active, tx12_controller_kill_channel_last_value
    global tx12_controller_recenter_pending
    if not pregame_active:
        return
    restart_from_setup = round_restart_pending_from_setup
    pregame_active = False
    round_restart_pending_from_setup = False
    tx12_controller_channel_kill_active = False
    tx12_controller_kill_channel_last_value = None
    tx12_controller_recenter_pending = True
    if pregame_root is not None:
        pregame_root.hide()
    if mode_buttons_root is not None:
        mode_buttons_root.show()
    if restart_from_setup:
        restart_simulation()
        return
    deploy_survey_lead_to_sector_start()
    if "respawn_follower_drones" in globals():
        respawn_follower_drones()
    apply_mission_start_camera()
    update_status_overlay()
    if fire_map_panel is not None:
        update_fire_map_popup()
    update_performance_metrics_visibility()


RADIO_CONTROLS_HELP_TEXT = (
    "TX12: CH1 roll  CH2 pitch  CH3 alt  CH4 yaw\n"
    "CH3 middle hover; +up / -down; stick amount = speed\n"
    "CH5 switch position change = kill\n"
    "[U] controller on/off   [N] recenter\n"
    "Keyboard movement overrides controller input\n"
    if REAL_SIM_RADIO_ENABLED
    else ""
)


# In-game controls cheat sheet (bottom right, [H] toggles).
CONTROLS_HELP_TEXT = (
    "CONTROLS   ([H] hide)\n"
    "Keyboard: W/A/S/D move\n"
    "Q up   E/PgDn down\n"
    f"{RADIO_CONTROLS_HELP_TEXT}"
    "Views: 1-6 or map click\n"
    "Map click: switch drone\n"
    "Mode: M manual   O auto\n"
    "Water: P/L/K call   J spray\n"
    "TAB map   V camera   T thermal\n"
    "Z/X/C keyboard speed   B position\n"
    "Buttons: full auto / hover / kill"
)
controls_help_root = None

# Water drones 2 and 3 are called separately ([L] and [K]); each stays at
# its spawn until the operator explicitly dispatches it. [P] arms water 1.
dispatched_water_slots = set()
water_dispatch_targets = {}


def reset_second_water_dispatch():
    dispatched_water_slots.clear()
    water_dispatch_targets.clear()


def request_water_follower_dispatch(water_slot):
    """[L] calls water drone 2, [K] calls water drone 3."""
    request_water_slot_dispatch(water_slot)


# Manual fire suppression ([J]): a water drone flown by hand only sprays the
# fire when the operator toggles its spray ON. Holds the view slots (1/2/3) of
# the water drones currently spraying. Toggle again to stop.
manual_water_spray_slots = set()


def is_water_spray_active(slot):
    return slot in manual_water_spray_slots


def get_water_view_stream_state(view):
    if view is None or view["role"] != "water":
        return None
    if view["follower"] is not None:
        return view["follower"]
    return water_drone_state


def toggle_manual_water_spray():
    """[J] turns the selected water drone's spray on/off. If the drone is in
    automation, pressing [J] first hands it to manual control so the spray
    actually works (this was the usual reason 'J didn't shoot water')."""
    if pregame_active or game_over_triggered or results_page_active:
        return
    view = get_selected_drone_view()
    if view is None or view["role"] != "water" or is_view_damaged(view):
        return
    slot = view["slot"]
    stream_state = get_water_view_stream_state(view)
    if slot in manual_water_spray_slots:
        manual_water_spray_slots.discard(slot)
        clear_water_suppression_visual(stream_state)
    else:
        if not water_tank_has_spray(stream_state):
            manual_water_spray_slots.discard(slot)
            clear_water_suppression_visual(stream_state)
            if "status_text" in globals():
                update_status_overlay()
            return
        if get_drone_view_control_mode(view) != CONTROL_MODE_MANUAL:
            set_drone_view_control_mode(view, CONTROL_MODE_MANUAL)
        manual_water_spray_slots.add(slot)
        clear_dispatch_alert()
    if "status_text" in globals():
        update_status_overlay()


def build_controls_help_overlay():
    global controls_help_root
    controls_help_root = app.a2dBottomRight.attachNewNode("controls_help_root")
    help_lines = CONTROLS_HELP_TEXT.splitlines()
    max_line_chars = max((len(line) for line in help_lines), default=24)
    frame_width = clamp(0.30 + (max_line_chars * 0.011), 0.56, 0.72)
    frame_height = 0.052 + (len(help_lines) * 0.031)
    frame_x1 = -0.02
    frame_x0 = frame_x1 - frame_width
    frame_z0 = 0.02
    frame_z1 = frame_z0 + frame_height
    backdrop = CardMaker("controls_help_backdrop")
    backdrop.setFrame(frame_x0, frame_x1, frame_z0, frame_z1)
    backdrop_np = controls_help_root.attachNewNode(backdrop.generate())
    backdrop_np.setColor(0.015, 0.035, 0.05, 0.90)
    backdrop_np.setTransparency(TransparencyAttrib.MAlpha)
    backdrop_np.setDepthWrite(False)
    backdrop_np.setDepthTest(False)
    backdrop_np.setBin("fixed", 130)
    help_text = OnscreenText(
        text=CONTROLS_HELP_TEXT,
        parent=controls_help_root,
        pos=(frame_x0 + 0.028, frame_z1 - 0.038),
        scale=0.028,
        fg=(0.96, 0.99, 1.0, 1.0),
        shadow=(0.0, 0.0, 0.0, 0.85),
        align=TextNode.ALeft,
        mayChange=False,
        wordwrap=34.0,
    )
    help_text.setBin("fixed", 131)
    help_text.setDepthWrite(False)
    help_text.setDepthTest(False)


def toggle_controls_help():
    if controls_help_root is None:
        return
    if controls_help_root.isHidden():
        controls_help_root.show()
    else:
        controls_help_root.hide()


def confirm_pregame_or_restart():
    if results_page_active:
        hide_results_page()
        show_pregame_setup_after_round()
        return
    if pregame_active:
        start_game_from_pregame()
    else:
        # [ENTER] mid-round now returns to the drone-selection page (so you can
        # change the team) rather than silently restarting the same round.
        show_pregame_setup_after_round()


bind_key_pairs()
app.accept("v", toggle_camera_mode)
for _perspective_key in range(1, 7):
    app.accept(str(_perspective_key), handle_number_key, [_perspective_key])
app.accept("tab", toggle_operator_satellite_view)      # operator satellite (only detected fire)
app.accept("space", toggle_operator_satellite_view)    # alias for the operator view
app.accept("\\", toggle_dev_bird_view)                 # DEV-ONLY full-info bird view
app.accept("f5", set_manual_mode)
app.accept("f6", set_automation_mode)
app.accept("b", set_manual_mode)  # Position mode for the selected drone.
app.accept("m", set_manual_mode)
app.accept("o", set_automation_mode)
app.accept("shift-m", toggle_control_mode)
app.accept("u", toggle_tx12_controller_input)
app.accept("n", request_tx12_controller_recenter)
app.accept("t", toggle_thermal_view)
app.accept("p", request_water_drone_dispatch)
app.accept("l", request_water_follower_dispatch, [2])
app.accept("k", request_water_follower_dispatch, [3])
app.accept("j", toggle_manual_water_spray)
app.accept("h", toggle_controls_help)
app.accept("z", set_slow_speed)
app.accept("x", set_normal_speed)
app.accept("c", set_fast_speed)
app.accept("r", cycle_team_algorithm)
app.accept("g", cycle_total_drone_count)
app.accept("enter", confirm_pregame_or_restart)
app.accept("return", confirm_pregame_or_restart)
app.accept("num_enter", confirm_pregame_or_restart)
app.accept("wheel_up", zoom_overview_camera_in)
app.accept("wheel_down", zoom_overview_camera_out)

apply_startup_demo_profile()
apply_camera_mode()
status_text = OnscreenText(
    text="",
    parent=app.a2dTopLeft,
    pos=(HUD_LEFT_MARGIN, -HUD_TOP_MARGIN),
    align=TextNode.ALeft,
    scale=0.045,
    fg=(0.96, 0.98, 1.0, 1.0),
    shadow=(0.0, 0.0, 0.0, 0.88),
    bg=(0.02, 0.06, 0.11, 0.44),
    mayChange=True,
)
status_text.textNode.setCardAsMargin(0.1, 0.1, 0.2, 0.1)


def build_water_tank_status_overlay():
    global water_tank_status_root, water_tank_status_fill, water_tank_status_text
    global water_tank_status_rows
    water_tank_status_root = app.a2dTopLeft.attachNewNode("water_tank_status_root")
    water_tank_status_rows = []

    for slot in range(1, WATER_TANK_HUD_MAX_SLOTS + 1):
        slot_root = water_tank_status_root.attachNewNode(f"water_tank_slot_{slot}")
        slot_x = HUD_LEFT_MARGIN + (
            (slot - 1) * (WATER_TANK_HUD_SLOT_WIDTH + WATER_TANK_HUD_SLOT_GAP)
        )
        slot_z = WATER_TANK_HUD_TOP_Z - WATER_TANK_HUD_HEIGHT
        slot_root.setPos(slot_x, 0.0, slot_z)

        back = CardMaker(f"water_tank_status_back_{slot}")
        back.setFrame(0.0, WATER_TANK_HUD_SLOT_WIDTH, 0.0, WATER_TANK_HUD_HEIGHT)
        back_np = slot_root.attachNewNode(back.generate())
        back_np.setColor(0.02, 0.06, 0.11, 0.78)
        back_np.setTransparency(TransparencyAttrib.MAlpha)
        back_np.setDepthWrite(False)
        back_np.setDepthTest(False)
        back_np.setBin("fixed", 144)

        fill = CardMaker(f"water_tank_status_fill_{slot}")
        fill.setFrame(0.0, WATER_TANK_HUD_SLOT_WIDTH, 0.0, WATER_TANK_HUD_HEIGHT)
        fill_np = slot_root.attachNewNode(fill.generate())
        fill_np.setScale(1.0, 1.0, 1.0)
        fill_np.setColor(0.24, 0.72, 1.0, 0.86)
        fill_np.setTransparency(TransparencyAttrib.MAlpha)
        fill_np.setDepthWrite(False)
        fill_np.setDepthTest(False)
        fill_np.setBin("fixed", 145)

        label_text = OnscreenText(
            text="",
            parent=slot_root,
            pos=(0.014, 0.011),
            scale=0.025,
            fg=(0.96, 0.99, 1.0, 1.0),
            shadow=(0.0, 0.0, 0.0, 0.78),
            align=TextNode.ALeft,
            mayChange=True,
        )
        label_text.setBin("fixed", 146)
        label_text.setDepthWrite(False)
        label_text.setDepthTest(False)

        water_tank_status_rows.append(
            {
                "root": slot_root,
                "fill": fill_np,
                "text": label_text,
            }
        )

    if water_tank_status_rows:
        water_tank_status_fill = water_tank_status_rows[0]["fill"]
        water_tank_status_text = water_tank_status_rows[0]["text"]
    water_tank_status_root.hide()


def update_water_tank_status_overlay():
    if water_tank_status_root is None:
        return
    if pregame_active or results_page_active:
        water_tank_status_root.hide()
        return

    any_visible = False
    for slot, row in enumerate(water_tank_status_rows, start=1):
        view = get_water_view_by_slot(slot)
        if slot > team_water_drone_count or view is None:
            row["root"].hide()
            continue

        if is_view_damaged(view):
            remaining_liters = 0.0
            tank_text = f"W{slot}  OFFLINE"
            fill_color = (0.9, 0.16, 0.12, 0.88)
        else:
            remaining_liters = get_water_tank_liters(get_water_view_stream_state(view))
            tank_percent = (
                remaining_liters / max(0.001, WATER_DRONE_TANK_CAPACITY_LITERS)
            )
            if remaining_liters <= WATER_DRONE_EMPTY_EPSILON_LITERS:
                tank_text = f"W{slot}  EMPTY"
                fill_color = (0.9, 0.16, 0.12, 0.88)
            elif tank_percent <= 0.25:
                tank_text = f"W{slot}  {remaining_liters:.0f}L"
                fill_color = (1.0, 0.62, 0.16, 0.9)
            else:
                tank_text = f"W{slot}  {remaining_liters:.0f}L"
                fill_color = (0.24, 0.72, 1.0, 0.86)

        fill_percent = clamp(
            remaining_liters / max(0.001, WATER_DRONE_TANK_CAPACITY_LITERS),
            0.001,
            1.0,
        )
        row["fill"].setScale(fill_percent, 1.0, 1.0)
        row["fill"].setColor(*fill_color)
        row["text"].setText(tank_text)
        row["root"].show()
        any_visible = True

    if any_visible:
        water_tank_status_root.show()
    else:
        water_tank_status_root.hide()


FIRE_MAP_DRONE_CLICK_RADIUS = 0.05  # panel units around a drone marker

# ---------------------------------------------------------------
# View-switch tracking (shown post-game only, not in real time):
# how many times the operator switched view, how long they stayed
# on each drone view, and first-person vs bird's-eye time.
# ---------------------------------------------------------------
view_switch_count = 0
view_switch_source_counts = {}
view_time_seconds = {}
camera_mode_time_seconds = {}
view_switch_log = []


def reset_view_switch_tracking():
    global view_switch_count, view_switch_source_counts
    global view_time_seconds, camera_mode_time_seconds, view_switch_log
    view_switch_count = 0
    view_switch_source_counts = {}
    view_time_seconds = {}
    camera_mode_time_seconds = {}
    view_switch_log = []


def record_view_switch_event(source):
    view_switch_source_counts[source] = (
        view_switch_source_counts.get(source, 0) + 1
    )


def current_camera_mode_label():
    if overview_camera_enabled:
        return "OPERATOR SATELLITE" if overview_is_operator else "DEV BIRD VIEW"
    if first_person_view:
        return "FIRST PERSON"
    return "CHASE CAMERA"


def record_view_dwell_time(dt):
    view = get_selected_drone_view()
    label = view["label"] if view is not None else "(none)"
    view_time_seconds[label] = view_time_seconds.get(label, 0.0) + dt
    mode_label = current_camera_mode_label()
    camera_mode_time_seconds[mode_label] = (
        camera_mode_time_seconds.get(mode_label, 0.0) + dt
    )


def get_fire_map_panel_mouse_point():
    if fire_map_panel is None or fire_map_panel.isHidden():
        return None
    if not app.mouseWatcherNode.hasMouse():
        return None
    mouse_x = app.mouseWatcherNode.getMouseX()
    mouse_y = app.mouseWatcherNode.getMouseY()
    panel_point = fire_map_panel.getRelativePoint(
        app.render2d,
        Point3(mouse_x, 0.0, mouse_y),
    )
    panel_x = panel_point.getX()
    panel_y = panel_point.getZ()
    if (
        panel_x < 0.0
        or panel_x > FIRE_MAP_PANEL_WIDTH
        or panel_y < 0.0
        or panel_y > FIRE_MAP_PANEL_HEIGHT
    ):
        return None
    return panel_x, panel_y


def try_click_fire_map_size_button():
    if results_page_active:
        return False
    panel_point = get_fire_map_panel_mouse_point()
    if panel_point is None:
        return False
    panel_x, panel_y = panel_point
    x0, x1, z0, z1 = get_fire_map_size_button_bounds()
    if x0 <= panel_x <= x1 and z0 <= panel_y <= z1:
        toggle_fire_map_size()
        return True
    return False


def try_select_drone_from_map_click():
    """Click a drone on the bird's-eye fire map to switch to it.

    Returns True when the click landed on the map panel (so the camera drag
    should not start). Keyboard switching via keys 1-6 still works.
    """
    if pregame_active or fire_map_panel is None:
        return False
    panel_point = get_fire_map_panel_mouse_point()
    if panel_point is None:
        return False
    panel_x, panel_y = panel_point

    closest_view = None
    closest_distance_sq = FIRE_MAP_DRONE_CLICK_RADIUS * FIRE_MAP_DRONE_CLICK_RADIUS
    for view in get_all_drone_views():
        view_root = view.get("root")
        if view_root is None or view_root.isEmpty():
            continue
        marker_x, marker_y = world_to_fire_map_coords(
            view_root.getX(),
            view_root.getY(),
        )
        distance_sq = (marker_x - panel_x) ** 2 + (marker_y - panel_y) ** 2
        if distance_sq < closest_distance_sq:
            closest_distance_sq = distance_sq
            closest_view = view
    if closest_view is not None:
        set_camera_target_role_slot(closest_view["role"], closest_view["slot"])
        record_view_switch_event("map_click")
    return True


def start_camera_drag():
    global dragging_camera, last_mouse_x, last_mouse_y

    if try_click_results_export():
        return
    if try_click_performance_toggle_button():
        return
    if try_click_mode_button():
        return
    if try_click_fire_map_size_button():
        return
    if try_select_drone_from_map_click():
        return
    dragging_camera = True

    if app.mouseWatcherNode.hasMouse():
        last_mouse_x = app.mouseWatcherNode.getMouseX()
        last_mouse_y = app.mouseWatcherNode.getMouseY()


def stop_camera_drag():
    global dragging_camera
    dragging_camera = False


# --- On-screen mode buttons (#5): blue full-auto / yellow hover / red kill ---
mode_buttons_root = None
mode_buttons = []
MODE_BUTTON_SPECS = (
    ("FULL AUTO", (0.12, 0.45, 0.95, 0.96), "set_full_automation"),
    ("HOVER ALL", (0.95, 0.78, 0.12, 0.96), "toggle_hover_all"),
    ("KILL SWITCH", (0.85, 0.12, 0.12, 0.97), "trigger_kill_switch"),
)


def build_mode_buttons_overlay():
    global mode_buttons_root, mode_buttons
    # Top-left, directly under the 4-line status HUD, laid out as a compact row.
    mode_buttons_root = app.a2dTopLeft.attachNewNode("mode_buttons_root")
    mode_buttons = []
    button_width, button_height = 0.34, 0.07
    button_gap = 0.018
    left_x = HUD_LEFT_MARGIN
    top_z = -0.30          # just below the status block
    z1 = top_z
    z0 = top_z - button_height
    commands = {
        "set_full_automation": set_full_automation,
        "toggle_hover_all": toggle_hover_all,
        "trigger_kill_switch": trigger_kill_switch,
    }
    for index, (label, color, command_name) in enumerate(MODE_BUTTON_SPECS):
        x0 = left_x + index * (button_width + button_gap)
        x1 = x0 + button_width
        card = CardMaker("mode_button_card")
        card.setFrame(x0, x1, z0, z1)
        card_np = mode_buttons_root.attachNewNode(card.generate())
        card_np.setColor(*color)
        card_np.setTransparency(TransparencyAttrib.MAlpha)
        card_np.setDepthWrite(False)
        card_np.setDepthTest(False)
        card_np.setBin("fixed", 145)
        label_text = OnscreenText(
            text=label,
            parent=mode_buttons_root,
            pos=((x0 + x1) * 0.5, z0 + 0.022),
            scale=0.032,
            fg=(1.0, 1.0, 1.0, 1.0),
            shadow=(0.0, 0.0, 0.0, 0.72),
            align=TextNode.ACenter,
            mayChange=False,
        )
        label_text.setBin("fixed", 146)
        label_text.setDepthWrite(False)
        label_text.setDepthTest(False)
        mode_buttons.append(
            {
                "frame": (x0, x1, z0, z1),
                "command": commands[command_name],
            }
        )
    mode_buttons_root.hide()  # shown once the round starts


def try_click_mode_button():
    """Return True if a mode button was clicked (so the camera drag is skipped)."""
    if (
        mode_buttons_root is None
        or mode_buttons_root.isHidden()
        or pregame_active
        or results_page_active
        or game_over_triggered
    ):
        return False
    if not app.mouseWatcherNode.hasMouse():
        return False
    mouse_x = app.mouseWatcherNode.getMouseX()
    mouse_y = app.mouseWatcherNode.getMouseY()
    panel_point = mode_buttons_root.getRelativePoint(
        app.render2d, Point3(mouse_x, 0.0, mouse_y)
    )
    panel_x, panel_z = panel_point.getX(), panel_point.getZ()
    for button in mode_buttons:
        x0, x1, z0, z1 = button["frame"]
        if x0 <= panel_x <= x1 and z0 <= panel_z <= z1:
            button["command"]()
            return True
    return False


def try_click_performance_toggle_button():
    if (
        performance_toggle_button_root is None
        or performance_toggle_button_root.isHidden()
        or performance_toggle_button_frame is None
        or pregame_active
        or results_page_active
        or game_over_triggered
    ):
        return False
    if not app.mouseWatcherNode.hasMouse():
        return False
    mouse_x = app.mouseWatcherNode.getMouseX()
    mouse_y = app.mouseWatcherNode.getMouseY()
    point = performance_toggle_button_root.getRelativePoint(
        app.render2d,
        Point3(mouse_x, 0.0, mouse_y),
    )
    panel_x, panel_z = point.getX(), point.getZ()
    x0, x1, z0, z1 = performance_toggle_button_frame
    if x0 <= panel_x <= x1 and z0 <= panel_z <= z1:
        toggle_performance_metrics_panel()
        return True
    return False


def try_click_results_export():
    """Export the report PDF only when the user clicks the report's export button."""
    if not results_page_active or results_export_button is None:
        return False
    if not app.mouseWatcherNode.hasMouse():
        return False
    mouse_x = app.mouseWatcherNode.getMouseX()
    mouse_y = app.mouseWatcherNode.getMouseY()
    point = results_root.getRelativePoint(app.render2d, Point3(mouse_x, 0.0, mouse_y))
    x0, x1, z0, z1 = results_export_button
    if not (x0 <= point.getX() <= x1 and z0 <= point.getZ() <= z1):
        return False
    try:
        saved_path = export_round_report_pdf()
        if results_export_status_text is not None:
            results_export_status_text.setText(
                "Saved: reports/" + os.path.basename(saved_path)
            )
    except Exception as export_error:
        print(f"[real-sim] PDF export failed: {export_error}")
        if results_export_status_text is not None:
            results_export_status_text.setText("Export failed - see console")
    return True


def build_dispatch_alert_overlay():
    global dispatch_alert_root, dispatch_alert_backdrop, dispatch_alert_text
    global dispatch_alert_body_text

    dispatch_alert_root = app.aspect2d.attachNewNode("dispatch_alert_root")
    # Upper-centre: high enough to notice, clear of the lower map and controls.
    dispatch_alert_root.setPos(0.0, 0.0, 0.22)

    backdrop = CardMaker("dispatch_alert_backdrop")
    backdrop.setFrame(-0.86, 0.86, -0.23, 0.20)
    dispatch_alert_backdrop = dispatch_alert_root.attachNewNode(backdrop.generate())
    dispatch_alert_backdrop.setColor(0.46, 0.08, 0.06, 0.86)
    dispatch_alert_backdrop.setTransparency(TransparencyAttrib.MAlpha)
    dispatch_alert_backdrop.setDepthWrite(False)
    dispatch_alert_backdrop.setDepthTest(False)
    dispatch_alert_backdrop.setBin("fixed", 150)

    dispatch_alert_text = OnscreenText(
        text="FIRE DETECTED",
        parent=dispatch_alert_root,
        pos=(0.0, 0.12),
        scale=0.047,
        fg=(1.0, 0.98, 0.93, 1.0),
        shadow=(0.0, 0.0, 0.0, 0.78),
        align=TextNode.ACenter,
        mayChange=True,
    )
    dispatch_alert_text.setBin("fixed", 151)
    dispatch_alert_text.setDepthWrite(False)
    dispatch_alert_text.setDepthTest(False)

    dispatch_alert_body_text = OnscreenText(
        text="AUTO DISPATCH: [P] W1   [L] W2   [K] W3\n"
        "MANUAL: SELECT WATER + [M] OR [J] SPRAY",
        parent=dispatch_alert_root,
        pos=(0.0, 0.035),
        scale=0.038,
        fg=(1.0, 0.98, 0.93, 1.0),
        shadow=(0.0, 0.0, 0.0, 0.78),
        align=TextNode.ACenter,
        mayChange=True,
    )
    dispatch_alert_body_text.setBin("fixed", 151)
    dispatch_alert_body_text.setDepthWrite(False)
    dispatch_alert_body_text.setDepthTest(False)
    dispatch_alert_root.hide()


def build_collision_alert_overlay():
    global collision_alert_root, collision_alert_backdrop
    global collision_alert_text, collision_alert_body_text

    collision_alert_root = app.aspect2d.attachNewNode("collision_alert_root")
    collision_alert_root.setPos(0.0, 0.0, 0.42)

    backdrop = CardMaker("collision_alert_backdrop")
    backdrop.setFrame(-0.38, 0.38, -0.15, 0.15)
    collision_alert_backdrop = collision_alert_root.attachNewNode(backdrop.generate())
    collision_alert_backdrop.setColor(0.62, 0.05, 0.04, 0.9)
    collision_alert_backdrop.setTransparency(TransparencyAttrib.MAlpha)
    collision_alert_backdrop.setDepthWrite(False)
    collision_alert_backdrop.setDepthTest(False)
    collision_alert_backdrop.setBin("fixed", 152)

    collision_alert_text = OnscreenText(
        text="DRONE COLLISION",
        parent=collision_alert_root,
        pos=(0.0, 0.08),
        scale=0.052,
        fg=(1.0, 0.95, 0.9, 1.0),
        shadow=(0.0, 0.0, 0.0, 0.85),
        align=TextNode.ACenter,
        mayChange=True,
    )
    collision_alert_text.setBin("fixed", 153)
    collision_alert_text.setDepthWrite(False)
    collision_alert_text.setDepthTest(False)

    collision_alert_body_text = OnscreenText(
        text="",
        parent=collision_alert_root,
        pos=(0.0, -0.005),
        scale=0.032,
        fg=(1.0, 0.95, 0.9, 1.0),
        shadow=(0.0, 0.0, 0.0, 0.85),
        align=TextNode.ACenter,
        mayChange=True,
        wordwrap=100.0,
    )
    collision_alert_body_text.setBin("fixed", 153)
    collision_alert_body_text.setDepthWrite(False)
    collision_alert_body_text.setDepthTest(False)
    collision_alert_root.hide()


def build_automation_message_overlay():
    global automation_message_root, automation_message_backdrop, automation_message_text
    global automation_message_header_text, automation_message_status_text
    global automation_message_survey_text, automation_message_water_text
    global automation_message_accent_bar

    automation_message_root = app.a2dTopLeft.attachNewNode("automation_message_root")
    automation_message_root.setPos(HUD_LEFT_MARGIN, 0.0, -HUD_AUTO_MESSAGE_TOP_OFFSET)

    backdrop = CardMaker("automation_message_backdrop")
    backdrop.setFrame(0.0, HUD_AUTO_MESSAGE_WIDTH, -HUD_AUTO_MESSAGE_HEIGHT, 0.02)
    automation_message_backdrop = automation_message_root.attachNewNode(
        backdrop.generate()
    )
    automation_message_backdrop.setColor(0.02, 0.06, 0.11, 0.86)
    automation_message_backdrop.setTransparency(TransparencyAttrib.MAlpha)
    automation_message_backdrop.setDepthWrite(False)
    automation_message_backdrop.setDepthTest(False)
    automation_message_backdrop.setBin("fixed", 130)

    # Accent bar across the top so the panel reads as a live comms feed.
    accent = CardMaker("automation_message_accent")
    accent.setFrame(0.0, HUD_AUTO_MESSAGE_WIDTH, -0.012, 0.02)
    automation_message_accent_bar = automation_message_root.attachNewNode(accent.generate())
    automation_message_accent_bar.setColor(*HUD_AUTO_MESSAGE_HEADER_COLOR)
    automation_message_accent_bar.setTransparency(TransparencyAttrib.MAlpha)
    automation_message_accent_bar.setDepthWrite(False)
    automation_message_accent_bar.setDepthTest(False)
    automation_message_accent_bar.setBin("fixed", 131)

    def _make_text(pos_z, scale, color, wordwrap):
        node = OnscreenText(
            text="",
            parent=automation_message_root,
            pos=(0.022, pos_z),
            scale=scale,
            fg=color,
            shadow=(0.0, 0.0, 0.0, 0.72),
            align=TextNode.ALeft,
            mayChange=True,
            wordwrap=wordwrap,
        )
        node.setBin("fixed", 132)
        node.setDepthWrite(False)
        node.setDepthTest(False)
        return node

    automation_message_header_text = _make_text(-0.045, 0.029, HUD_AUTO_MESSAGE_HEADER_COLOR, 38.0)
    automation_message_status_text = _make_text(-0.084, 0.0215, HUD_AUTO_MESSAGE_STATUS_COLOR, 52.0)
    automation_message_survey_text = _make_text(-0.135, 0.0245, HUD_AUTO_MESSAGE_SURVEY_COLOR, 46.0)
    automation_message_water_text = _make_text(-0.235, 0.0245, HUD_AUTO_MESSAGE_WATER_COLOR, 46.0)

    # Kept for backwards compatibility; no longer rendered directly.
    automation_message_text = automation_message_header_text
    # #11: removed from the HUD -- planned trajectories replace it.
    automation_message_root.hide()


# ---------------------------------------------------------------
# Live mission metrics panel (replaces the old performance-score area).
# Metric 1 (M3): per-drone control-usage bars, color-coded auto/manual.
# Metric 2: fire detection rate (red = total fire, yellow = detected).
# Metric 3: fire extinguish rate (red = total fire, blue = extinguished).
# All graphs update live, freeze at game over, and feed the results page.
# ---------------------------------------------------------------
METRICS_AUTO_COLOR = (0.20, 0.95, 0.42, 0.95)    # automatic control
METRICS_MANUAL_COLOR = (1.0, 0.62, 0.18, 0.95)   # manual control
METRICS_TOTAL_FIRE_COLOR = (0.95, 0.22, 0.16, 0.95)
METRICS_DETECTED_COLOR = (1.0, 0.85, 0.25, 0.95)
METRICS_EXTINGUISHED_COLOR = (0.30, 0.65, 1.0, 0.95)
METRICS_FIRE_BAR_FULL_SCALE_HOTSPOTS = 40.0
METRICS_USAGE_BAR_LEFT = 0.13
METRICS_USAGE_BAR_WIDTH = 0.69
METRICS_USAGE_CELL_COUNT = 48   # time slices across the live usage timeline (#1)
METRICS_OFFLINE_COLOR = (0.5, 0.5, 0.55, 0.92)
METRICS_USAGE_EMPTY_COLOR = (0.12, 0.15, 0.18, 0.92)
METRICS_USAGE_ROW_HEIGHT = 0.030
METRICS_USAGE_ROW_GAP = 0.012
METRICS_USAGE_TOP_Z = 0.475
METRICS_FIRE_BAR_LEFT = 0.13
METRICS_FIRE_BAR_WIDTH = 0.69
METRICS_FIRE_BAR_HEIGHT = 0.030

drone_mode_time_seconds = {}
# #2: per-drone mode timeline -> label: [(start_time, state), ...] where state is
# True (auto), False (manual) or None (offline). One entry per mode change, so
# the report can draw the actual green/orange/grey sequence over time.
drone_mode_timeline = {}
metrics_time_series = deque(maxlen=2400)
metrics_usage_rows = {}
metrics_detection_nodes = {}
metrics_extinguish_nodes = {}


def rebuild_performance_toggle_button():
    global performance_toggle_button_card, performance_toggle_button_frame
    if performance_toggle_button_root is None:
        return
    if performance_toggle_button_card is not None and not performance_toggle_button_card.isEmpty():
        performance_toggle_button_card.removeNode()

    if performance_metrics_hidden:
        width = 0.22
        height = 0.045
        x0 = PERFORMANCE_PANEL_LEFT_MARGIN
        z0 = PERFORMANCE_PANEL_BOTTOM_MARGIN
        label = "SHOW METRICS"
    else:
        width = 0.12
        height = 0.038
        x0 = (
            PERFORMANCE_PANEL_LEFT_MARGIN
            + PERFORMANCE_PANEL_WIDTH
            - width
            - 0.018
        )
        z0 = (
            PERFORMANCE_PANEL_BOTTOM_MARGIN
            + PERFORMANCE_PANEL_HEIGHT
            - height
            - 0.018
        )
        label = "HIDE"

    x1 = x0 + width
    z1 = z0 + height
    card = CardMaker("performance_metrics_toggle_button")
    card.setFrame(x0, x1, z0, z1)
    performance_toggle_button_card = performance_toggle_button_root.attachNewNode(
        card.generate()
    )
    performance_toggle_button_card.setColor(0.05, 0.12, 0.16, 0.94)
    performance_toggle_button_card.setTransparency(TransparencyAttrib.MAlpha)
    performance_toggle_button_card.setDepthWrite(False)
    performance_toggle_button_card.setDepthTest(False)
    performance_toggle_button_card.setBin("fixed", 139)

    if performance_toggle_button_text is not None:
        performance_toggle_button_text.setText(label)
        performance_toggle_button_text.setPos((x0 + x1) * 0.5, z0 + 0.011)
        performance_toggle_button_text.setScale(0.021 if performance_metrics_hidden else 0.020)
    performance_toggle_button_frame = (x0, x1, z0, z1)


def update_performance_metrics_visibility():
    show_toggle = (
        performance_toggle_button_root is not None
        and not pregame_active
        and not results_page_active
        and not game_over_triggered
    )
    if performance_panel_root is not None:
        if show_toggle and not performance_metrics_hidden:
            performance_panel_root.show()
        else:
            performance_panel_root.hide()
    if performance_toggle_button_root is not None:
        if show_toggle:
            rebuild_performance_toggle_button()
            performance_toggle_button_root.show()
        else:
            performance_toggle_button_root.hide()


def toggle_performance_metrics_panel():
    global performance_metrics_hidden
    if pregame_active or results_page_active or game_over_triggered:
        return
    performance_metrics_hidden = not performance_metrics_hidden
    update_performance_metrics_visibility()


def reset_live_metrics_tracking():
    global drone_mode_time_seconds, drone_mode_timeline
    drone_mode_time_seconds = {}
    drone_mode_timeline = {}
    metrics_time_series.clear()
    # Drop per-drone usage rows so they rebuild for the new team layout.
    for nodes in metrics_usage_rows.values():
        _destroy_metrics_usage_row(nodes)
    metrics_usage_rows.clear()


def metrics_short_label(view):
    prefix = "S" if view["role"] == "survey" else "W"
    return f"{prefix}{view['slot']}"


def accumulate_drone_mode_time(dt):
    for view in get_all_drone_views():
        label = metrics_short_label(view)
        timeline = drone_mode_timeline.setdefault(label, [])
        if is_view_damaged(view):
            # Record the offline transition once, then accrue no control time.
            if not timeline or timeline[-1][1] is not None:
                timeline.append((motion_state.sim_time_seconds, None))
            continue
        is_auto = get_drone_view_control_mode(view) != CONTROL_MODE_MANUAL
        if not timeline or timeline[-1][1] != is_auto:
            timeline.append((motion_state.sim_time_seconds, is_auto))
        bucket = drone_mode_time_seconds.setdefault(label, [0.0, 0.0])
        if not is_auto:
            bucket[0] += dt
        else:
            bucket[1] += dt


def mode_state_at_time(timeline, time_seconds):
    """Control state (True auto / False manual / None offline) at a given time,
    or 'empty' if before the drone's first record."""
    if not timeline or time_seconds < timeline[0][0]:
        return "empty"
    state = timeline[0][1]
    for entry_time, entry_state in timeline:
        if entry_time <= time_seconds:
            state = entry_state
        else:
            break
    return state


def count_fire_totals():
    total_count = len(fire_hotspots)
    detected_count = sum(1 for hotspot in fire_hotspots if hotspot.detected)
    extinguished_count = sum(
        1
        for hotspot in fire_hotspots
        if hotspot.suppression_state == FIRE_STATE_OUT
    )
    return total_count, detected_count, extinguished_count


def _build_metrics_fire_bar(z_bottom, overlay_color, name):
    nodes = {}
    back = CardMaker(f"{name}_back")
    back.setFrame(
        METRICS_FIRE_BAR_LEFT,
        METRICS_FIRE_BAR_LEFT + METRICS_FIRE_BAR_WIDTH,
        z_bottom,
        z_bottom + METRICS_FIRE_BAR_HEIGHT,
    )
    back_np = performance_panel_root.attachNewNode(back.generate())
    back_np.setColor(0.12, 0.15, 0.18, 0.92)
    back_np.setTransparency(TransparencyAttrib.MAlpha)
    back_np.setDepthWrite(False)
    back_np.setDepthTest(False)
    back_np.setBin("fixed", 136)

    total_fill = CardMaker(f"{name}_total_fill")
    total_fill.setFrame(0.0, METRICS_FIRE_BAR_WIDTH, 0.0, METRICS_FIRE_BAR_HEIGHT)
    total_np = performance_panel_root.attachNewNode(total_fill.generate())
    total_np.setPos(METRICS_FIRE_BAR_LEFT, 0.0, z_bottom)
    total_np.setScale(0.001, 1.0, 1.0)
    total_np.setColor(*METRICS_TOTAL_FIRE_COLOR)
    total_np.setTransparency(TransparencyAttrib.MAlpha)
    total_np.setDepthWrite(False)
    total_np.setDepthTest(False)
    total_np.setBin("fixed", 137)

    overlay_fill = CardMaker(f"{name}_overlay_fill")
    overlay_fill.setFrame(
        0.0,
        METRICS_FIRE_BAR_WIDTH,
        0.0,
        METRICS_FIRE_BAR_HEIGHT * 0.55,
    )
    overlay_np = performance_panel_root.attachNewNode(overlay_fill.generate())
    overlay_np.setPos(
        METRICS_FIRE_BAR_LEFT,
        0.0,
        z_bottom + METRICS_FIRE_BAR_HEIGHT * 0.225,
    )
    overlay_np.setScale(0.001, 1.0, 1.0)
    overlay_np.setColor(*overlay_color)
    overlay_np.setTransparency(TransparencyAttrib.MAlpha)
    overlay_np.setDepthWrite(False)
    overlay_np.setDepthTest(False)
    overlay_np.setBin("fixed", 138)

    label = OnscreenText(
        text="",
        parent=performance_panel_root,
        pos=(METRICS_FIRE_BAR_LEFT, z_bottom + METRICS_FIRE_BAR_HEIGHT + 0.012),
        scale=0.024,
        fg=(0.92, 0.98, 1.0, 1.0),
        shadow=(0.0, 0.0, 0.0, 0.7),
        align=TextNode.ALeft,
        mayChange=True,
    )
    label.setBin("fixed", 136)
    label.setDepthWrite(False)
    label.setDepthTest(False)

    nodes["total_fill"] = total_np
    nodes["overlay_fill"] = overlay_np
    nodes["label"] = label
    return nodes


def build_operator_performance_overlay():
    global performance_panel_root, performance_tab_text, performance_score_text
    global performance_meter_fill, performance_graph_root, performance_graph_line_node
    global metrics_usage_rows, metrics_detection_nodes, metrics_extinguish_nodes
    global performance_toggle_button_root, performance_toggle_button_text

    performance_panel_root = app.a2dBottomLeft.attachNewNode("operator_performance_root")
    performance_panel_root.setPos(
        PERFORMANCE_PANEL_LEFT_MARGIN,
        0.0,
        PERFORMANCE_PANEL_BOTTOM_MARGIN,
    )

    backdrop = CardMaker("operator_performance_backdrop")
    backdrop.setFrame(0.0, PERFORMANCE_PANEL_WIDTH, 0.0, PERFORMANCE_PANEL_HEIGHT)
    backdrop_np = performance_panel_root.attachNewNode(backdrop.generate())
    backdrop_np.setColor(0.02, 0.05, 0.08, 0.82)
    backdrop_np.setTransparency(TransparencyAttrib.MAlpha)
    backdrop_np.setDepthWrite(False)
    backdrop_np.setDepthTest(False)
    backdrop_np.setBin("fixed", 135)

    performance_tab_text = OnscreenText(
        text="LIVE MISSION METRICS",
        parent=performance_panel_root,
        pos=(0.03, PERFORMANCE_PANEL_HEIGHT - 0.045),
        scale=0.033,
        fg=(0.98, 0.86, 0.42, 1.0),
        shadow=(0.0, 0.0, 0.0, 0.7),
        align=TextNode.ALeft,
        mayChange=True,
    )
    performance_tab_text.setBin("fixed", 136)
    performance_tab_text.setDepthWrite(False)
    performance_tab_text.setDepthTest(False)

    usage_header = OnscreenText(
        text="CONTROL USAGE (green = auto, orange = manual)",
        parent=performance_panel_root,
        pos=(0.03, METRICS_USAGE_TOP_Z + METRICS_USAGE_ROW_HEIGHT + 0.012),
        scale=0.024,
        fg=(0.92, 0.98, 1.0, 1.0),
        shadow=(0.0, 0.0, 0.0, 0.7),
        align=TextNode.ALeft,
        mayChange=False,
    )
    usage_header.setBin("fixed", 136)
    usage_header.setDepthWrite(False)
    usage_header.setDepthTest(False)

    metrics_usage_rows = {}
    metrics_detection_nodes = _build_metrics_fire_bar(
        0.155,
        METRICS_DETECTED_COLOR,
        "metrics_detection_bar",
    )
    metrics_extinguish_nodes = _build_metrics_fire_bar(
        0.05,
        METRICS_EXTINGUISHED_COLOR,
        "metrics_extinguish_bar",
    )

    performance_score_text = OnscreenText(
        text="",
        parent=performance_panel_root,
        pos=(0.03, PERFORMANCE_PANEL_HEIGHT - 0.082),
        scale=0.024,
        fg=(0.92, 0.98, 1.0, 1.0),
        shadow=(0.0, 0.0, 0.0, 0.7),
        align=TextNode.ALeft,
        mayChange=True,
        wordwrap=40.0,
    )
    performance_score_text.setBin("fixed", 136)
    performance_score_text.setDepthWrite(False)
    performance_score_text.setDepthTest(False)

    # Kept for compatibility with older cleanup paths.
    performance_meter_fill = None
    performance_graph_root = performance_panel_root.attachNewNode(
        "operator_performance_graph_dynamic"
    )
    performance_graph_root.setBin("fixed", 137)
    performance_graph_line_node = None

    performance_toggle_button_root = app.a2dBottomLeft.attachNewNode(
        "performance_metrics_toggle_root"
    )
    performance_toggle_button_text = OnscreenText(
        text="HIDE",
        parent=performance_toggle_button_root,
        pos=(0.0, 0.0),
        scale=0.020,
        fg=(0.96, 0.99, 1.0, 1.0),
        shadow=(0.0, 0.0, 0.0, 0.72),
        align=TextNode.ACenter,
        mayChange=True,
    )
    performance_toggle_button_text.setBin("fixed", 140)
    performance_toggle_button_text.setDepthWrite(False)
    performance_toggle_button_text.setDepthTest(False)
    update_performance_metrics_visibility()


def _destroy_metrics_usage_row(nodes):
    label_node = nodes.get("label")
    if label_node is not None:
        label_node.destroy()
    back_np = nodes.get("back")
    if back_np is not None and not back_np.isEmpty():
        back_np.removeNode()
    for cell_np in nodes.get("cells", ()):
        if cell_np is not None and not cell_np.isEmpty():
            cell_np.removeNode()


def _ensure_metrics_usage_row(label, row_index):
    nodes = metrics_usage_rows.get(label)
    if nodes is not None:
        if nodes.get("row_index") == row_index:
            return nodes
        # Row order changed (e.g. followers spawned after the leads were
        # already drawn); rebuild this row at its new position.
        _destroy_metrics_usage_row(nodes)
        metrics_usage_rows.pop(label, None)
    # Compact spacing for 6-drone teams so rows don't crowd the fire bars.
    drone_count = len(get_all_drone_views())
    if drone_count > 4:
        row_height = 0.022
        row_gap = 0.008
        label_scale = 0.022
    else:
        row_height = METRICS_USAGE_ROW_HEIGHT
        row_gap = METRICS_USAGE_ROW_GAP
        label_scale = 0.026
    z_top = METRICS_USAGE_TOP_Z - (row_index * (row_height + row_gap))
    z_bottom = z_top - row_height

    row_label = OnscreenText(
        text=label,
        parent=performance_panel_root,
        pos=(0.03, z_bottom + 0.006),
        scale=label_scale,
        fg=(0.92, 0.98, 1.0, 1.0),
        shadow=(0.0, 0.0, 0.0, 0.7),
        align=TextNode.ALeft,
        mayChange=True,
    )
    row_label.setBin("fixed", 136)
    row_label.setDepthWrite(False)
    row_label.setDepthTest(False)

    back = CardMaker(f"metrics_usage_back_{label}")
    back.setFrame(
        METRICS_USAGE_BAR_LEFT,
        METRICS_USAGE_BAR_LEFT + METRICS_USAGE_BAR_WIDTH,
        z_bottom,
        z_top,
    )
    back_np = performance_panel_root.attachNewNode(back.generate())
    back_np.setColor(0.12, 0.15, 0.18, 0.92)
    back_np.setTransparency(TransparencyAttrib.MAlpha)
    back_np.setDepthWrite(False)
    back_np.setDepthTest(False)
    back_np.setBin("fixed", 136)

    # One thin cell per time slice; coloured each frame by the mode in use at
    # that moment, so the row reads as a green/orange/grey sequence over time.
    cell_width = METRICS_USAGE_BAR_WIDTH / METRICS_USAGE_CELL_COUNT
    cells = []
    for cell_index in range(METRICS_USAGE_CELL_COUNT):
        cell = CardMaker(f"metrics_usage_cell_{label}_{cell_index}")
        cell.setFrame(0.0, cell_width * 1.02, 0.0, row_height)
        cell_np = performance_panel_root.attachNewNode(cell.generate())
        cell_np.setPos(
            METRICS_USAGE_BAR_LEFT + cell_index * cell_width, 0.0, z_bottom
        )
        cell_np.setColor(*METRICS_USAGE_EMPTY_COLOR)
        cell_np.setTransparency(TransparencyAttrib.MAlpha)
        cell_np.setDepthWrite(False)
        cell_np.setDepthTest(False)
        cell_np.setBin("fixed", 137)
        cells.append(cell_np)

    nodes = {
        "label": row_label,
        "back": back_np,
        "cells": cells,
        "z_bottom": z_bottom,
        "row_index": row_index,
    }
    metrics_usage_rows[label] = nodes
    return nodes


def update_live_metrics_panel():
    if performance_panel_root is None:
        return

    # Metric 1 (M3): per-drone usage bars grow with elapsed time.
    views = get_all_drone_views()
    ordered_views = sorted(
        views,
        key=lambda view: (0 if view["role"] == "water" else 1, view["slot"]),
    )
    round_seconds = max(1.0, FIRE_TARGET_GAME_DURATION_SECONDS)
    elapsed_seconds = motion_state.sim_time_seconds
    for row_index, view in enumerate(ordered_views):
        label = metrics_short_label(view)
        nodes = _ensure_metrics_usage_row(label, row_index)
        timeline = drone_mode_timeline.get(label, [])
        for cell_index, cell_np in enumerate(nodes["cells"]):
            cell_time = (cell_index + 0.5) / METRICS_USAGE_CELL_COUNT * round_seconds
            if cell_time > elapsed_seconds:
                cell_np.setColor(*METRICS_USAGE_EMPTY_COLOR)  # not reached yet
                continue
            state = mode_state_at_time(timeline, cell_time)
            if state is True:
                cell_np.setColor(*METRICS_AUTO_COLOR)
            elif state is False:
                cell_np.setColor(*METRICS_MANUAL_COLOR)
            elif state is None:
                cell_np.setColor(*METRICS_OFFLINE_COLOR)
            else:
                cell_np.setColor(*METRICS_USAGE_EMPTY_COLOR)

    # Metrics 2 and 3: total fire vs detected / extinguished.
    total_count, detected_count, extinguished_count = count_fire_totals()
    total_fraction = clamp(
        total_count / METRICS_FIRE_BAR_FULL_SCALE_HOTSPOTS,
        0.0,
        1.0,
    )
    detection_fraction = (
        total_fraction * (detected_count / total_count) if total_count else 0.0
    )
    extinguish_fraction = (
        total_fraction * (extinguished_count / total_count) if total_count else 0.0
    )
    metrics_detection_nodes["total_fill"].setScale(
        max(0.001, total_fraction), 1.0, 1.0
    )
    metrics_detection_nodes["overlay_fill"].setScale(
        max(0.001, detection_fraction), 1.0, 1.0
    )
    detected_percent = (detected_count / total_count * 100.0) if total_count else 0.0
    metrics_detection_nodes["label"].setText(
        f"FIRE DETECTION RATE: {detected_count}/{total_count}"
        f" ({detected_percent:.0f}%)"
    )
    metrics_extinguish_nodes["total_fill"].setScale(
        max(0.001, total_fraction), 1.0, 1.0
    )
    metrics_extinguish_nodes["overlay_fill"].setScale(
        max(0.001, extinguish_fraction), 1.0, 1.0
    )
    extinguished_percent = (
        (extinguished_count / total_count * 100.0) if total_count else 0.0
    )
    metrics_extinguish_nodes["label"].setText(
        f"FIRE EXTINGUISH RATE: {extinguished_count}/{total_count}"
        f" ({extinguished_percent:.0f}%)"
    )

    metrics_time_series.append(
        (
            motion_state.sim_time_seconds,
            total_count,
            detected_count,
            extinguished_count,
        )
    )


def record_completed_round_if_needed(performance):
    global operator_lifetime_round_count, operator_lifetime_success_count
    global operator_lifetime_score_sum, performance_round_recorded

    if not game_over_triggered or performance_round_recorded:
        return

    score = performance["performance_score"]
    operator_lifetime_round_count += 1
    operator_lifetime_score_sum += score
    if score >= PERFORMANCE_REWARD_THRESHOLD:
        operator_lifetime_success_count += 1
    performance_round_recorded = True


def update_operator_performance_overlay(performance):
    global performance_graph_line_node

    update_performance_metrics_visibility()
    if performance_score_text is None or performance_graph_root is None:
        return
    record_completed_round_if_needed(performance)
    if performance_metrics_hidden or pregame_active or results_page_active:
        return

    score = performance["performance_score"]
    performance_history.append(score)

    if game_over_triggered:
        # Freeze all live graphs at game over; the results page displays them.
        return

    performance_score_text.setText(
        f"Composite Score: {score * 100.0:.0f}%"
        f" | Survey {performance['team_survey_drones']}"
        f" / Water {performance['team_water_drones']}"
    )
    update_live_metrics_panel()


# ---------------------------------------------------------------
# End-of-game report page: live graphs freeze at game over and the
# final numbers are shown here, with per-fire-cluster data tables
# and the view-switching summary (post-game only).
# ---------------------------------------------------------------
FIRE_CLUSTER_LINK_DISTANCE_METERS = FIRE_SPREAD_MAX_DISTANCE_METERS * 1.6

results_page_active = False
results_root = None
results_dynamic_nodes = []


def compute_fire_clusters():
    """Group hotspots into spatial clusters (union-find by proximity)."""
    items = [
        hotspot
        for hotspot in fire_hotspots
        if hotspot.root is not None and not hotspot.root.isEmpty()
    ]
    count = len(items)
    parent = list(range(count))

    def find_root(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    link_distance_sq = FIRE_CLUSTER_LINK_DISTANCE_METERS ** 2
    positions = [(h.root.getX(), h.root.getY()) for h in items]
    for first_index in range(count):
        for second_index in range(first_index + 1, count):
            dx = positions[first_index][0] - positions[second_index][0]
            dy = positions[first_index][1] - positions[second_index][1]
            if (dx * dx + dy * dy) <= link_distance_sq:
                parent[find_root(first_index)] = find_root(second_index)

    clusters = {}
    for index, hotspot in enumerate(items):
        clusters.setdefault(find_root(index), []).append(hotspot)
    return list(clusters.values())


def compose_cluster_table_lines():
    lines = ["FIRE CLUSTERS (one cluster = one connected fire area)"]
    clusters = sorted(
        compute_fire_clusters(),
        key=len,
        reverse=True,
    )
    if not clusters:
        lines.append("  (no fire this round)")
        return lines
    for cluster_index, cluster in enumerate(clusters, start=1):
        total = len(cluster)
        surveyed = sum(1 for hotspot in cluster if hotspot.detected)
        extinguished = sum(
            1
            for hotspot in cluster
            if hotspot.suppression_state == FIRE_STATE_OUT
        )
        lines.append(f"  Cluster {cluster_index}: {total} fire spots total")
        lines.append(
            f"    found by drones: {surveyed} of {total}"
            f" ({surveyed / total * 100.0:.0f}%)"
        )
        lines.append(
            f"    put out: {extinguished} of {total}"
            f" ({extinguished / total * 100.0:.0f}%)"
        )
        if surveyed:
            lines.append(
                f"    put out among found: {extinguished} of {surveyed}"
                f" ({extinguished / surveyed * 100.0:.0f}%)"
            )
        else:
            lines.append("    put out among found: none found")
    return lines


def compose_mode_usage_lines():
    lines = ["CONTROL USAGE (final)  manual% / auto%"]
    total_manual = 0.0
    total_auto = 0.0
    for label in sorted(drone_mode_time_seconds.keys()):
        manual_seconds, auto_seconds = drone_mode_time_seconds[label]
        total_manual += manual_seconds
        total_auto += auto_seconds
        drone_total = manual_seconds + auto_seconds
        if drone_total > 0.0:
            lines.append(
                f"  {label}: manual {manual_seconds / drone_total * 100.0:.0f}%"
                f" / auto {auto_seconds / drone_total * 100.0:.0f}%"
            )
    team_total = total_manual + total_auto
    if team_total > 0.0:
        lines.insert(
            1,
            f"  TEAM: manual {total_manual / team_total * 100.0:.0f}%"
            f" / auto {total_auto / team_total * 100.0:.0f}%",
        )
    else:
        lines.append("  (no flight time recorded)")
    return lines


def compose_view_switch_lines():
    lines = [f"VIEW SWITCHING  total switches: {view_switch_count}"]
    total_dwell = sum(view_time_seconds.values())
    if total_dwell > 0.0:
        for label, dwell_seconds in sorted(
            view_time_seconds.items(),
            key=lambda item: -item[1],
        ):
            lines.append(
                f"  {label}: {dwell_seconds / total_dwell * 100.0:.0f}%"
                f" ({dwell_seconds:.0f}s)"
            )
    mode_total = sum(camera_mode_time_seconds.values())
    if mode_total > 0.0:
        for label, dwell_seconds in sorted(
            camera_mode_time_seconds.items(),
            key=lambda item: -item[1],
        ):
            lines.append(
                f"  {label}: {dwell_seconds / mode_total * 100.0:.0f}%"
            )
    return lines


def _results_text(text, pos, scale=0.026, color=(0.92, 0.98, 1.0, 1.0)):
    node = OnscreenText(
        text=text,
        parent=results_root,
        pos=pos,
        scale=scale,
        fg=color,
        shadow=(0.0, 0.0, 0.0, 0.7),
        align=TextNode.ALeft,
        mayChange=False,
    )
    node.setBin("fixed", 211)
    node.setDepthWrite(False)
    node.setDepthTest(False)
    results_dynamic_nodes.append(node)
    return node


def _draw_results_time_series_graph(graph_left, graph_bottom, width, height):
    """Final detection/extinguish graph from the live time series."""
    if len(metrics_time_series) < 2:
        return
    frame = LineSegs("results_graph_frame")
    frame.setThickness(1.2)
    frame.setColor(0.86, 0.92, 0.96, 0.6)
    frame.moveTo(graph_left, 0.0, graph_bottom)
    frame.drawTo(graph_left + width, 0.0, graph_bottom)
    frame.drawTo(graph_left + width, 0.0, graph_bottom + height)
    frame.drawTo(graph_left, 0.0, graph_bottom + height)
    frame.drawTo(graph_left, 0.0, graph_bottom)
    frame_np = results_root.attachNewNode(frame.create())
    frame_np.setBin("fixed", 211)
    results_dynamic_nodes.append(frame_np)

    samples = list(metrics_time_series)
    time_start = samples[0][0]
    time_end = max(samples[-1][0], time_start + 0.001)
    peak_total = max(1, max(sample[1] for sample in samples))

    for series_index, series_color in (
        (1, METRICS_TOTAL_FIRE_COLOR),
        (2, METRICS_DETECTED_COLOR),
        (3, METRICS_EXTINGUISHED_COLOR),
    ):
        line = LineSegs(f"results_graph_series_{series_index}")
        line.setThickness(2.2)
        line.setColor(*series_color)
        for sample_index, sample in enumerate(samples):
            x_fraction = (sample[0] - time_start) / (time_end - time_start)
            y_fraction = sample[series_index] / peak_total
            x = graph_left + x_fraction * width
            z = graph_bottom + clamp(y_fraction, 0.0, 1.0) * height
            if sample_index == 0:
                line.moveTo(x, 0.0, z)
            else:
                line.drawTo(x, 0.0, z)
        line_np = results_root.attachNewNode(line.create())
        line_np.setBin("fixed", 212)
        results_dynamic_nodes.append(line_np)

    # Axis labels (#7): Y = number of fires, X = round time.
    _results_text(
        str(peak_total),
        (graph_left - 0.045, graph_bottom + height - 0.012),
        scale=0.019,
        color=(0.8, 0.86, 0.92, 1.0),
    )
    _results_text(
        "0",
        (graph_left - 0.03, graph_bottom),
        scale=0.019,
        color=(0.8, 0.86, 0.92, 1.0),
    )
    _results_text(
        "fires",
        (graph_left - 0.075, graph_bottom + height * 0.5),
        scale=0.018,
        color=(0.8, 0.86, 0.92, 1.0),
    )
    _results_text(
        "0:00",
        (graph_left, graph_bottom - 0.03),
        scale=0.018,
        color=(0.8, 0.86, 0.92, 1.0),
    )
    _results_text(
        format_round_clock(time_end),
        (graph_left + width - 0.06, graph_bottom - 0.03),
        scale=0.018,
        color=(0.8, 0.86, 0.92, 1.0),
    )
    _results_text(
        "round time -->",
        (graph_left + width * 0.5 - 0.07, graph_bottom - 0.03),
        scale=0.018,
        color=(0.7, 0.78, 0.88, 1.0),
    )


def _draw_results_usage_bars(bars_left, bars_top, width):
    """#2: per-drone control-mode TIMELINE. Each row is a strip across the whole
    round; the colour at each point is the mode in use at that moment (green =
    auto, orange = manual, grey = offline), so switching modes shows up as a
    real green/orange/green sequence instead of one merged bar."""
    if not drone_mode_timeline:
        return
    bar_height = 0.026
    bar_gap = 0.016
    round_seconds = max(1.0, motion_state.sim_time_seconds)
    offline_color = (0.5, 0.5, 0.55, 0.9)
    z = bars_top
    for label in sorted(drone_mode_timeline.keys()):
        timeline = drone_mode_timeline[label]
        if not timeline:
            continue
        # faint background track so the full round length is always visible
        track = CardMaker(f"results_track_{label}")
        track.setFrame(0.0, width, 0.0, bar_height)
        track_np = results_root.attachNewNode(track.generate())
        track_np.setPos(bars_left, 0.0, z - bar_height)
        track_np.setColor(0.16, 0.18, 0.22, 0.7)
        track_np.setTransparency(TransparencyAttrib.MAlpha)
        track_np.setBin("fixed", 211)
        results_dynamic_nodes.append(track_np)
        for index, (segment_start, state) in enumerate(timeline):
            segment_end = (
                timeline[index + 1][0]
                if index + 1 < len(timeline)
                else round_seconds
            )
            x0 = bars_left + width * clamp(segment_start / round_seconds, 0.0, 1.0)
            x1 = bars_left + width * clamp(segment_end / round_seconds, 0.0, 1.0)
            segment_width = x1 - x0
            if segment_width <= 0.0005:
                continue
            if state is True:
                color = METRICS_AUTO_COLOR
            elif state is False:
                color = METRICS_MANUAL_COLOR
            else:
                color = offline_color
            bar = CardMaker(f"results_usage_{label}_{index}")
            bar.setFrame(0.0, segment_width, 0.0, bar_height)
            bar_np = results_root.attachNewNode(bar.generate())
            bar_np.setPos(x0, 0.0, z - bar_height)
            bar_np.setColor(*color)
            bar_np.setTransparency(TransparencyAttrib.MAlpha)
            bar_np.setBin("fixed", 212)
            results_dynamic_nodes.append(bar_np)
        manual_seconds, auto_seconds = drone_mode_time_seconds.get(label, [0.0, 0.0])
        total_seconds = manual_seconds + auto_seconds
        auto_percent = (auto_seconds / total_seconds * 100.0) if total_seconds else 0.0
        _results_text(
            f"{label}  auto {auto_percent:.0f}%",
            (bars_left + width + 0.02, z - bar_height + 0.004),
            scale=0.022,
        )
        z -= bar_height + bar_gap
    # Time axis (#7).
    axis_z = z + 0.004
    _results_text("0:00", (bars_left, axis_z), scale=0.019, color=(0.7, 0.78, 0.88, 1.0))
    _results_text(
        format_round_clock(round_seconds),
        (bars_left + width - 0.06, axis_z),
        scale=0.019,
        color=(0.7, 0.78, 0.88, 1.0),
    )
    _results_text(
        "time into round -->",
        (bars_left + width * 0.5 - 0.08, axis_z),
        scale=0.019,
        color=(0.7, 0.78, 0.88, 1.0),
    )


def _draw_results_view_dwell_bars(bars_left, bars_top, width):
    total_dwell = sum(view_time_seconds.values())
    if total_dwell <= 0.0:
        return
    bar_height = 0.022
    bar_gap = 0.012
    z = bars_top
    for label, dwell_seconds in sorted(
        view_time_seconds.items(),
        key=lambda item: -item[1],
    )[:6]:
        fraction = dwell_seconds / total_dwell
        bar = CardMaker(f"results_view_bar_{label}")
        bar.setFrame(0.0, max(0.004, width * fraction), 0.0, bar_height)
        bar_np = results_root.attachNewNode(bar.generate())
        bar_np.setPos(bars_left, 0.0, z - bar_height)
        bar_np.setColor(0.45, 0.78, 1.0, 0.92)
        bar_np.setTransparency(TransparencyAttrib.MAlpha)
        bar_np.setBin("fixed", 212)
        results_dynamic_nodes.append(bar_np)
        _results_text(
            f"{label} {fraction * 100.0:.0f}%",
            (bars_left + width + 0.02, z - bar_height + 0.004),
            scale=0.022,
        )
        z -= bar_height + bar_gap


def _set_hud_overlays_visible(visible):
    """Hide the in-game HUD while the end-of-round report is on screen."""
    overlays = (
        globals().get("status_text"),
        globals().get("mode_buttons_root"),
        globals().get("performance_panel_root"),
        globals().get("performance_toggle_button_root"),
        globals().get("controls_help_root"),
        globals().get("fire_map_panel"),
        globals().get("dispatch_alert_root"),
        globals().get("collision_alert_root"),
    )
    for overlay in overlays:
        if overlay is None:
            continue
        if visible:
            if overlay is globals().get("performance_panel_root"):
                if globals().get("performance_metrics_hidden", False):
                    overlay.hide()
                    continue
            if overlay is globals().get("dispatch_alert_root") or overlay is globals().get(
                "collision_alert_root"
            ):
                continue  # these alerts manage their own visibility
            overlay.show()
        else:
            overlay.hide()


results_pdf_path = None
results_export_button = None        # (x0, x1, z0, z1) hit-box on the report page
results_export_status_text = None   # updated when the user clicks export


def export_round_report_pdf():
    """#7: write the end-of-round report to a PDF in a 'reports' folder next to
    the sim. Pure-Python PDF (vector text + rectangles + polylines), so it needs
    no extra packages installed on the machine running the sim."""
    import datetime

    try:
        base_dir = os.path.dirname(os.path.abspath(__file__))
    except Exception:
        base_dir = os.getcwd()
    out_dir = os.path.join(base_dir, "reports")

    content = []

    def esc(text_value):
        return (
            str(text_value)
            .replace("\\", "\\\\")
            .replace("(", "\\(")
            .replace(")", "\\)")
        )

    def text(x, y, string, size=10, color=(0.1, 0.1, 0.12)):
        content.append(
            "BT /F1 %d Tf %.3f %.3f %.3f rg %.1f %.1f Td (%s) Tj ET"
            % (size, color[0], color[1], color[2], x, y, esc(string))
        )

    def rect(x, y, w, h, color):
        content.append(
            "%.3f %.3f %.3f rg %.1f %.1f %.1f %.1f re f"
            % (color[0], color[1], color[2], x, y, w, h)
        )

    def rect_stroke(x, y, w, h, color, width=1.0):
        content.append(
            "%.3f %.3f %.3f RG %.2f w %.1f %.1f %.1f %.1f re S"
            % (color[0], color[1], color[2], width, x, y, w, h)
        )

    def polyline(points, color, width=1.6):
        if len(points) < 2:
            return
        segment = "%.3f %.3f %.3f RG %.2f w %.1f %.1f m " % (
            color[0], color[1], color[2], width, points[0][0], points[0][1],
        )
        for px, py in points[1:]:
            segment += "%.1f %.1f l " % (px, py)
        content.append(segment + "S")

    auto_c, manual_c, offline_c = (0.20, 0.78, 0.40), (1.0, 0.6, 0.18), (0.5, 0.5, 0.55)
    red_c, yellow_c, blue_c = (0.9, 0.22, 0.16), (0.95, 0.8, 0.2), (0.3, 0.62, 1.0)
    head_c = (0.12, 0.18, 0.32)

    reason = "Map fully burned" if game_end_reason == "burned_out" else "Round time limit reached"
    text(55, 755, "FSC Fire-Drone Trust Sim - End of Round Report", 16, head_c)
    text(
        55, 736,
        "Outcome: %s   |   Length: %s   |   Burned: %.0f%% of map"
        % (reason, format_round_clock(motion_state.sim_time_seconds), burn_ratio() * 100.0),
        10,
    )
    text(55, 722, "Generated " + datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"), 9, (0.4, 0.4, 0.45))

    cursor_y = 698
    text(55, cursor_y, "CONTROL MODE TIMELINE (green auto / orange manual / grey offline)", 11, head_c)
    round_seconds = max(1.0, motion_state.sim_time_seconds)
    strip_x, strip_w, strip_h = 95, 360, 11
    for label in sorted(drone_mode_timeline.keys()):
        cursor_y -= 20
        timeline = drone_mode_timeline[label]
        rect(strip_x, cursor_y, strip_w, strip_h, (0.85, 0.86, 0.9))
        for index, (segment_start, state) in enumerate(timeline):
            segment_end = timeline[index + 1][0] if index + 1 < len(timeline) else round_seconds
            x0 = strip_x + strip_w * max(0.0, min(1.0, segment_start / round_seconds))
            x1 = strip_x + strip_w * max(0.0, min(1.0, segment_end / round_seconds))
            if x1 - x0 <= 0.3:
                continue
            color = auto_c if state is True else (manual_c if state is False else offline_c)
            rect(x0, cursor_y, x1 - x0, strip_h, color)
        manual_s, auto_s = drone_mode_time_seconds.get(label, [0.0, 0.0])
        total_s = manual_s + auto_s
        auto_pct = (auto_s / total_s * 100.0) if total_s else 0.0
        text(55, cursor_y + 2, label, 10)
        text(strip_x + strip_w + 8, cursor_y + 2, "auto %.0f%%" % auto_pct, 9)
    cursor_y -= 16
    text(strip_x, cursor_y, "0:00", 8, (0.4, 0.4, 0.45))
    text(strip_x + strip_w - 24, cursor_y, format_round_clock(round_seconds), 8, (0.4, 0.4, 0.45))
    text(strip_x + strip_w * 0.5 - 24, cursor_y, "time into round", 8, (0.4, 0.4, 0.45))

    cursor_y -= 26
    text(55, cursor_y, "FIRE TIMELINE (red total / yellow found / blue put out)", 11, head_c)
    graph_x, graph_w, graph_h = 95, 360, 120
    graph_y = cursor_y - 12 - graph_h
    rect_stroke(graph_x, graph_y, graph_w, graph_h, (0.6, 0.62, 0.66))
    samples = list(metrics_time_series)
    if len(samples) >= 2:
        t0 = samples[0][0]
        t1 = max(samples[-1][0], t0 + 0.001)
        peak = max(1, max(sample[1] for sample in samples))
        for series_index, color in ((1, red_c), (2, yellow_c), (3, blue_c)):
            points = []
            for sample in samples:
                fx = (sample[0] - t0) / (t1 - t0)
                fy = sample[series_index] / peak
                points.append((graph_x + fx * graph_w, graph_y + max(0.0, min(1.0, fy)) * graph_h))
            polyline(points, color, 1.6)
        text(graph_x - 22, graph_y + graph_h - 6, str(peak), 8, (0.3, 0.3, 0.35))
        text(graph_x - 14, graph_y, "0", 8, (0.3, 0.3, 0.35))
        text(graph_x - 42, graph_y + graph_h * 0.5, "fires", 8, (0.3, 0.3, 0.35))
        text(graph_x, graph_y - 12, "0:00", 8, (0.3, 0.3, 0.35))
        text(graph_x + graph_w - 24, graph_y - 12, format_round_clock(t1), 8, (0.3, 0.3, 0.35))
        text(graph_x + graph_w * 0.5 - 24, graph_y - 12, "time into round", 8, (0.3, 0.3, 0.35))
    else:
        text(graph_x + 10, graph_y + graph_h * 0.5, "No fire data recorded.", 9, (0.4, 0.4, 0.45))

    cursor_y = graph_y - 30
    text(55, cursor_y, "SUMMARY", 11, head_c)
    summary_lines = (
        compose_mode_usage_lines()
        + [""]
        + compose_cluster_table_lines()
        + [""]
        + compose_view_switch_lines()
    )
    for summary_line in summary_lines:
        cursor_y -= 12
        if cursor_y < 40:
            break
        text(60, cursor_y, summary_line, 9)

    stream = "\n".join(content).encode("latin-1", "replace")
    objects = [
        b"<< /Type /Catalog /Pages 2 0 R >>",
        b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>",
        b"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 612 792] "
        b"/Resources << /Font << /F1 5 0 R >> >> /Contents 4 0 R >>",
        b"<< /Length " + str(len(stream)).encode() + b" >>\nstream\n" + stream + b"\nendstream",
        b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>",
    ]
    pdf = b"%PDF-1.4\n"
    offsets = []
    for index, body in enumerate(objects, start=1):
        offsets.append(len(pdf))
        pdf += ("%d 0 obj\n" % index).encode() + body + b"\nendobj\n"
    xref_pos = len(pdf)
    pdf += ("xref\n0 %d\n" % (len(objects) + 1)).encode() + b"0000000000 65535 f \n"
    for offset in offsets:
        pdf += ("%010d 00000 n \n" % offset).encode()
    pdf += (
        "trailer\n<< /Size %d /Root 1 0 R >>\nstartxref\n%d\n%%%%EOF"
        % (len(objects) + 1, xref_pos)
    ).encode()

    os.makedirs(out_dir, exist_ok=True)
    file_name = "FSC_round_report_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".pdf"
    file_path = os.path.join(out_dir, file_name)
    with open(file_path, "wb") as report_file:
        report_file.write(pdf)
    return file_path


def show_results_page():
    global results_page_active, results_root, results_pdf_path
    global results_export_button, results_export_status_text

    _set_hud_overlays_visible(False)
    if results_root is None:
        results_root = app.aspect2d.attachNewNode("results_page_root")
        results_root.setBin("fixed", 210)
        backdrop = CardMaker("results_backdrop")
        backdrop.setFrame(-1.28, 1.28, -0.92, 0.92)
        backdrop_np = results_root.attachNewNode(backdrop.generate())
        backdrop_np.setColor(0.02, 0.04, 0.07, 0.94)
        backdrop_np.setTransparency(TransparencyAttrib.MAlpha)
        backdrop_np.setDepthWrite(False)
        backdrop_np.setDepthTest(False)
        backdrop_np.setBin("fixed", 210)
    else:
        for node in results_dynamic_nodes:
            if hasattr(node, "destroy"):
                node.destroy()
            elif not node.isEmpty():
                node.removeNode()
        results_dynamic_nodes.clear()

    reason_label = (
        "MAP FULLY BURNED"
        if game_end_reason == "burned_out"
        else "ROUND TIME LIMIT REACHED"
    )
    _results_text(
        f"END OF ROUND REPORT  -  {reason_label}",
        (-1.18, 0.66),
        scale=0.045,
        color=(0.98, 0.86, 0.42, 1.0),
    )
    _results_text(
        f"Round length: {format_round_clock(motion_state.sim_time_seconds)}"
        f" | Burned: {burn_ratio() * 100.0:.0f}% of map",
        (-1.18, 0.58),
    )

    left_lines = []
    left_lines.extend(compose_mode_usage_lines())
    left_lines.append("")
    left_lines.extend(compose_cluster_table_lines())
    left_lines.append("")
    left_lines.extend(compose_view_switch_lines())
    if drone_damage_events:
        left_lines.append("")
        collision_count = sum(
            1
            for _, _, damage_reason in drone_damage_events
            if damage_reason == DAMAGE_REASON_COLLISION
        )
        branch_impact_count = sum(
            1
            for _, _, damage_reason in drone_damage_events
            if damage_reason == DAMAGE_REASON_BRANCH_IMPACT
        )
        ground_impact_count = sum(
            1
            for _, _, damage_reason in drone_damage_events
            if damage_reason == DAMAGE_REASON_GROUND_IMPACT
        )
        kill_switch_count = sum(
            1
            for _, _, damage_reason in drone_damage_events
            if damage_reason == DAMAGE_REASON_KILL_SWITCH
        )
        if collision_count:
            left_lines.append(f"COLLISIONS: {collision_count} drone(s) lost")
        if branch_impact_count:
            left_lines.append(f"BRANCH IMPACTS: {branch_impact_count} drone(s) lost")
        if ground_impact_count:
            left_lines.append(f"GROUND IMPACTS: {ground_impact_count} drone(s) lost")
        if kill_switch_count:
            left_lines.append(
                f"KILL SWITCH: {kill_switch_count} drone(s) shut down"
            )
        for event_time, event_label, damage_reason in drone_damage_events:
            reason_text = {
                DAMAGE_REASON_KILL_SWITCH: "kill switch",
                DAMAGE_REASON_BRANCH_IMPACT: "branch impact",
                DAMAGE_REASON_GROUND_IMPACT: "ground impact",
            }.get(damage_reason, "collision")
            left_lines.append(
                f"  {event_label} offline by {reason_text} at {format_round_clock(event_time)}"
            )
    _results_text("\n".join(left_lines), (-1.18, 0.48), scale=0.027)

    _results_text(
        "CONTROL MODE TIMELINE (green auto / orange manual / grey offline)",
        (0.16, 0.48),
        scale=0.024,
    )
    _draw_results_usage_bars(0.16, 0.445, 0.62)

    _results_text(
        "FIRE TIMELINE (red = total, yellow = found, blue = put out)",
        (0.16, 0.16),
        scale=0.024,
    )
    _draw_results_time_series_graph(0.16, -0.20, 0.98, 0.33)
    _results_text("TIME PER VIEW", (0.16, -0.27), scale=0.024)
    _draw_results_view_dwell_bars(0.16, -0.31, 0.62)

    # #7 (Cheryl June 14): export only when the user clicks the button - no
    # automatic export every round.
    export_x0, export_x1, export_z0, export_z1 = 0.58, 1.12, -0.86, -0.78
    results_export_button = (export_x0, export_x1, export_z0, export_z1)
    export_card = CardMaker("results_export_button")
    export_card.setFrame(export_x0, export_x1, export_z0, export_z1)
    export_card_np = results_root.attachNewNode(export_card.generate())
    export_card_np.setColor(0.16, 0.42, 0.78, 0.96)
    export_card_np.setTransparency(TransparencyAttrib.MAlpha)
    export_card_np.setBin("fixed", 212)
    results_dynamic_nodes.append(export_card_np)
    export_label = OnscreenText(
        text="EXPORT REPORT PDF",
        parent=results_root,
        pos=((export_x0 + export_x1) * 0.5, export_z0 + 0.026),
        scale=0.03,
        fg=(1.0, 1.0, 1.0, 1.0),
        shadow=(0.0, 0.0, 0.0, 0.7),
        align=TextNode.ACenter,
        mayChange=False,
    )
    export_label.setBin("fixed", 213)
    results_dynamic_nodes.append(export_label)
    results_export_status_text = OnscreenText(
        text="",
        parent=results_root,
        pos=(0.58, -0.91),
        scale=0.022,
        fg=(0.6, 0.85, 1.0, 1.0),
        shadow=(0.0, 0.0, 0.0, 0.7),
        align=TextNode.ALeft,
        mayChange=True,
    )
    results_export_status_text.setBin("fixed", 213)
    results_dynamic_nodes.append(results_export_status_text)

    _results_text(
        "Press [ENTER] to continue to mission setup",
        (-1.18, -0.84),
        scale=0.03,
        color=(0.6, 1.0, 0.7, 1.0),
    )

    results_root.show()
    results_page_active = True


def hide_results_page():
    global results_page_active
    results_page_active = False
    if results_root is not None:
        results_root.hide()
    _set_hud_overlays_visible(True)


def format_hotspot_location(hotspot):
    if hotspot is None or hotspot.root.isEmpty():
        return "(unknown)"
    return f"({hotspot.root.getX():.0f}, {hotspot.root.getY():.0f})"


def _drone_block(label, doing, next_action):
    # Three short lines per drone: who, what it's doing (and why), and what's next.
    lines = [label, doing]
    if next_action:
        lines.append(f"Next  >  {next_action}")
    return "\n".join(lines)


def compose_operator_automation_message():
    if game_over_triggered:
        return {
            "header": "AUTOMATION  -  ROUND COMPLETE",
            "status": "Run finished. Automation is paused until you restart.",
            "survey": "",
            "water": "",
        }

    selected_view = get_selected_drone_view()
    selected_mode = get_drone_view_control_mode(selected_view)
    selected_label = selected_view["label"] if selected_view is not None else "Selected drone"
    manual_controls_water = (
        selected_mode == CONTROL_MODE_MANUAL
        and selected_view is not None
        and selected_view["role"] == "water"
    )
    manual_controls_survey = (
        selected_mode == CONTROL_MODE_MANUAL
        and selected_view is not None
        and selected_view["role"] == "survey"
    )

    mode_label = f"{selected_label.upper()}  -  {selected_mode.upper()} selected"

    # Live situational summary so the operator can sanity-check the automation.
    coverage_percent = mapping_coverage_score() * 100.0
    detected_count = sum(
        1 for hotspot in fire_hotspots if hotspot.detected
    )
    active_count = sum(
        1
        for hotspot in fire_hotspots
        if hotspot.suppression_state == FIRE_STATE_ACTIVE
    )
    status_line = (
        f"Area mapped {coverage_percent:.0f}%   |   "
        f"Fires found {detected_count}   |   Active now {active_count}"
    )

    # ---- Survey drone: what / why / next ----
    if manual_controls_survey:
        survey_block = _drone_block(
            "SURVEY DRONE  -  manual",
            "You're flying it. Automation has stepped back and won't steer.",
            "Switch to AUTO to hand control back to me.",
        )
    elif team_state.survey_focus_hotspot is not None:
        focus_hotspot = team_state.survey_focus_hotspot
        focus_location = format_hotspot_location(focus_hotspot)
        mapped_percent = hotspot_mapping_fraction(focus_hotspot) * 100.0
        if team_state.algorithm_mode == TEAM_ALGORITHM_MAP_THEN_RESUME:
            survey_block = _drone_block(
                "SURVEY DRONE  -  mapping",
                f"Circling the fire at {focus_location} to finish its map "
                f"({mapped_percent:.0f}%) so the water drone gets exact coordinates.",
                "Return to area patrol once mapping hits 100%.",
            )
        elif team_state.algorithm_mode == TEAM_ALGORITHM_ROLLING_PATROL:
            survey_block = _drone_block(
                "SURVEY DRONE  -  mapping",
                f"Orbiting the fire at {focus_location} to keep its map fresh "
                f"({mapped_percent:.0f}%).",
                "Break off shortly to sweep for new ignitions.",
            )
        else:
            survey_block = _drone_block(
                "SURVEY DRONE  -  mapping",
                f"Orbiting the fire at {focus_location} to map it "
                f"({mapped_percent:.0f}%) before it can spread.",
                "Map to 100%, then move to the next threat.",
            )
    elif automation_target_hotspot is not None:
        target_location = format_hotspot_location(automation_target_hotspot)
        if automation_target_hotspot.detected:
            survey_block = _drone_block(
                "SURVEY DRONE  -  responding",
                f"Heading to the confirmed fire near {target_location} for a closer map.",
                "Orbit and finish mapping it on arrival.",
            )
        else:
            survey_block = _drone_block(
                "SURVEY DRONE  -  investigating",
                f"Caught a heat signature near {target_location}. Flying over to check it.",
                "Confirm and flag it if it's a real fire.",
            )
    elif automation_search_waypoint is not None:
        waypoint_x, waypoint_y, _ = automation_search_waypoint
        survey_block = _drone_block(
            "SURVEY DRONE  -  patrolling",
            f"Sweeping unburned forest near ({waypoint_x:.0f}, {waypoint_y:.0f}) "
            "to catch fires early.",
            "Divert instantly the moment a fire appears.",
        )
    else:
        survey_block = _drone_block(
            "SURVEY DRONE  -  planning",
            "Picking the next area to scan.",
            "Heading out as soon as a target is chosen.",
        )

    # ---- Water drone: what / why / next ----
    pending_hotspot = team_state.pending_water_dispatch_hotspot
    water_target_hotspot = team_state.water_target_hotspot
    if manual_controls_water:
        if water_target_hotspot is not None and water_target_hotspot.water_drone_engaged:
            water_block = _drone_block(
                "WATER DRONE  -  manual, assisting",
                "You're flying it and in range. Assist spray is boosting the fire-truck crew now.",
                "",
            )
        elif water_target_hotspot is not None:
            water_block = _drone_block(
                "WATER DRONE  -  manual",
                "You're flying it. Get closer to the fire to start the assist spray.",
                "",
            )
        elif pending_hotspot is not None:
            water_block = _drone_block(
                "WATER DRONE  -  manual",
                "A fire needs support. Press P to send water drone 1.",
                "",
            )
        else:
            water_block = _drone_block(
                "WATER DRONE  -  manual",
                "You're flying it. No fire currently needs water.",
                "",
            )
    elif pending_hotspot is not None and any_water_slot_needs_operator_dispatch():
        water_block = _drone_block(
            "WATER DRONE  -  awaiting your go",
            f"Holding for your call. A fire near {format_hotspot_location(pending_hotspot)} "
            "needs support.",
            "Press P/L/K to dispatch the water drones you want.",
        )
    elif water_target_hotspot is not None:
        target_location = format_hotspot_location(water_target_hotspot)
        if water_target_hotspot.water_drone_engaged:
            water_block = _drone_block(
                "WATER DRONE  -  suppressing",
                f"Hovering over {target_location}, boosting the fire-truck crew's suppression.",
                "Hold until the fire is controlled, then return to base.",
            )
        elif water_target_hotspot.water_drone_assigned or water_drone_state.mission_active:
            water_block = _drone_block(
                "WATER DRONE  -  en route",
                f"Flying to {target_location}, the highest-priority active fire.",
                "Begin spraying the moment I arrive.",
            )
        else:
            water_block = _drone_block(
                "WATER DRONE  -  preparing",
                f"Target locked at {target_location}. Lining up my approach.",
                "",
            )
    else:
        water_next_step = (
            "Auto firefighting is armed; launch the instant a detected fire needs water."
            if any_water_slot_can_auto_dispatch()
            else "Press P/L/K when a detected fire needs water."
        )
        water_block = _drone_block(
            "WATER DRONE  -  standby",
            "Waiting at base, fueled and ready.",
            water_next_step,
        )

    return {
        "header": f"AUTOMATION  -  {mode_label}",
        "status": status_line,
        "survey": survey_block,
        "water": water_block,
    }


def update_automation_message_overlay():
    # #11: the auto-message window is removed. The planned-trajectory lines drawn
    # on the map now show what automation is doing, so this panel stays hidden.
    if automation_message_root is not None and not automation_message_root.isHidden():
        automation_message_root.hide()


planned_trajectory_root = None
water_target_reticle_root = None
water_hud_target_reticle_root = None
PLANNED_TRAJECTORY_COLORS = (
    (0.30, 0.85, 1.0, 0.55),   # survey 1 - cyan
    (0.55, 1.0, 0.55, 0.55),   # survey 2 - green
    (1.0, 0.80, 0.35, 0.55),   # survey 3 - amber
)


def update_planned_trajectories():
    """Draw each automated drone's planned route on the terrain. Survey drones
    show their sweep/fire-orbit path; water drones show their suppression path.
    Manual drones get none."""
    global planned_trajectory_root
    if planned_trajectory_root is None:
        planned_trajectory_root = app.render.attachNewNode("planned_trajectory_root")
        planned_trajectory_root.setTransparency(TransparencyAttrib.MAlpha)
        planned_trajectory_root.setDepthWrite(False)
    planned_trajectory_root.node().removeAllChildren()
    if pregame_active or game_over_triggered:
        return
    selected_node = get_camera_target_node()
    for view in get_all_drone_views():
        if is_view_damaged(view):
            continue
        route = get_view_automation_route_world_points(view)
        if len(route) < 2:
            continue
        if view["root"] is selected_node:
            color = (1.0, 0.93, 0.35, 0.95)
        elif view["role"] == "water":
            color = (0.36, 0.72, 1.0, 0.72)
        else:
            color = (0.95, 0.97, 1.0, 0.6)
        path = LineSegs("planned_path")
        path.setThickness(2.4 if view["role"] == "water" else 2.0)
        path.setColor(*color)
        started = False
        for waypoint_x, waypoint_y in route:
            ground = sample_ground(waypoint_x, waypoint_y)
            waypoint_z = (ground[0].z if ground is not None else 0.0) + 1.5
            if not started:
                path.moveTo(waypoint_x, waypoint_y, waypoint_z)
                started = True
            else:
                path.drawTo(waypoint_x, waypoint_y, waypoint_z)
        planned_trajectory_root.attachNewNode(path.create())


def clear_planned_trajectories():
    if planned_trajectory_root is not None:
        planned_trajectory_root.node().removeAllChildren()


def clear_water_target_reticles():
    if water_target_reticle_root is not None:
        water_target_reticle_root.node().removeAllChildren()


def clear_water_hud_target_reticle():
    if water_hud_target_reticle_root is not None:
        water_hud_target_reticle_root.node().removeAllChildren()
        water_hud_target_reticle_root.hide()


def get_water_current_target_for_view(view):
    if view is None or view["role"] != "water" or is_view_damaged(view):
        return None
    mode = get_drone_view_control_mode(view)
    selected_node = get_camera_target_node()
    target_hotspot = None
    if mode == CONTROL_MODE_MANUAL:
        if is_water_spray_active(view["slot"]) or view["root"] is selected_node:
            target_hotspot = choose_nearest_sprayable_hotspot(
                view["root"].getX(),
                view["root"].getY(),
            )
        else:
            return None
    else:
        target_hotspot = get_water_dispatch_target(view["slot"])
        if target_hotspot is None:
            if view.get("follower") is not None:
                target_hotspot = view["follower"].get("target_hotspot")
            else:
                target_hotspot = team_state.water_target_hotspot
    if target_hotspot is None or target_hotspot.root.isEmpty():
        return None
    if not target_hotspot.detected:
        return None
    if target_hotspot.suppression_state not in (
        FIRE_STATE_ACTIVE,
        FIRE_STATE_CONTAINED,
    ):
        return None
    return target_hotspot


def get_selected_water_hud_target():
    if pregame_active or game_over_triggered or overview_camera_enabled:
        return None, None
    view = get_selected_drone_view()
    if view is None or view["role"] != "water" or is_view_damaged(view):
        return None, None
    target_hotspot = get_water_current_target_for_view(view)
    if target_hotspot is None:
        return None, None
    return view, target_hotspot


def project_world_point_to_aspect2d(point):
    if app.camera is None or app.cam is None:
        return None
    camera_point = app.camera.getRelativePoint(app.render, point)
    projected = Point2()
    lens = app.cam.node().getLens()
    if not lens.project(camera_point, projected):
        return None
    aspect_ratio = app.getAspectRatio()
    margin = WATER_HUD_TARGET_RETICLE_CLAMP_MARGIN
    return (
        clamp(projected.getX() * aspect_ratio, -aspect_ratio + margin, aspect_ratio - margin),
        clamp(projected.getY(), -1.0 + margin, 1.0 - margin),
    )


def add_water_hud_target_reticle(screen_x, screen_z, color_rgba):
    pulse = sin(motion_state.sim_time_seconds * 5.4) * WATER_HUD_TARGET_RETICLE_PULSE
    radius = WATER_HUD_TARGET_RETICLE_RADIUS + pulse
    tick = WATER_HUD_TARGET_RETICLE_TICK
    line = LineSegs("water_hud_target_reticle")
    line.setThickness(3.4)
    line.setColor(*color_rgba)

    for start_degrees, end_degrees in ((18, 78), (102, 162), (198, 258), (282, 342)):
        for segment_index in range(WATER_HUD_TARGET_RETICLE_ARC_SEGMENTS + 1):
            angle = radians(
                start_degrees
                + (end_degrees - start_degrees)
                * (segment_index / WATER_HUD_TARGET_RETICLE_ARC_SEGMENTS)
            )
            px = screen_x + cos(angle) * radius
            pz = screen_z + sin(angle) * radius
            if segment_index == 0:
                line.moveTo(px, 0.0, pz)
            else:
                line.drawTo(px, 0.0, pz)

    inner_gap = radius * 0.34
    outer_edge = radius + tick
    line.moveTo(screen_x - outer_edge, 0.0, screen_z)
    line.drawTo(screen_x - inner_gap, 0.0, screen_z)
    line.moveTo(screen_x + inner_gap, 0.0, screen_z)
    line.drawTo(screen_x + outer_edge, 0.0, screen_z)
    line.moveTo(screen_x, 0.0, screen_z - outer_edge)
    line.drawTo(screen_x, 0.0, screen_z - inner_gap)
    line.moveTo(screen_x, 0.0, screen_z + inner_gap)
    line.drawTo(screen_x, 0.0, screen_z + outer_edge)

    center_gap = radius * 0.09
    line.moveTo(screen_x - center_gap, 0.0, screen_z)
    line.drawTo(screen_x + center_gap, 0.0, screen_z)
    line.moveTo(screen_x, 0.0, screen_z - center_gap)
    line.drawTo(screen_x, 0.0, screen_z + center_gap)

    node = water_hud_target_reticle_root.attachNewNode(line.create())
    node.setTransparency(TransparencyAttrib.MAlpha)
    node.setDepthWrite(False)
    node.setDepthTest(False)
    node.setBin("fixed", 175)
    return node


def update_water_hud_target_reticle():
    global water_hud_target_reticle_root
    if water_hud_target_reticle_root is None:
        water_hud_target_reticle_root = app.aspect2d.attachNewNode(
            "water_hud_target_reticle_root"
        )
        water_hud_target_reticle_root.setTransparency(TransparencyAttrib.MAlpha)
        water_hud_target_reticle_root.setDepthWrite(False)
        water_hud_target_reticle_root.setDepthTest(False)
        water_hud_target_reticle_root.hide()
    clear_water_hud_target_reticle()
    if app.camera is None or app.cam is None:
        return
    view, target_hotspot = get_selected_water_hud_target()
    if target_hotspot is None:
        return
    target_point = Point3(
        target_hotspot.root.getX(),
        target_hotspot.root.getY(),
        target_hotspot.ground_z + WATER_DRONE_STREAM_IMPACT_HEIGHT_METERS,
    )
    screen_position = project_world_point_to_aspect2d(target_point)
    if screen_position is None:
        return
    active_spray = (
        (
            is_water_spray_active(view["slot"])
            and water_tank_has_spray(get_water_view_stream_state(view))
        )
        or target_hotspot.water_drone_engaged
    )
    color = (
        (1.0, 0.92, 0.28, 0.98)
        if active_spray
        else (0.28, 0.92, 1.0, 0.96)
    )
    water_hud_target_reticle_root.show()
    add_water_hud_target_reticle(screen_position[0], screen_position[1], color)


def add_water_target_reticle(hotspot, color_rgba):
    center_x = hotspot.root.getX()
    center_y = hotspot.root.getY()
    center_z = hotspot.ground_z + WATER_TARGET_RETICLE_GROUND_OFFSET_METERS
    radius = WATER_TARGET_RETICLE_RADIUS_METERS
    line = LineSegs("water_target_reticle")
    line.setThickness(2.4)
    line.setColor(*color_rgba)
    for segment_index in range(WATER_TARGET_RETICLE_SEGMENTS + 1):
        angle = tau * segment_index / WATER_TARGET_RETICLE_SEGMENTS
        px = center_x + cos(angle) * radius
        py = center_y + sin(angle) * radius
        if segment_index == 0:
            line.moveTo(px, py, center_z)
        else:
            line.drawTo(px, py, center_z)
    cross_radius = radius * 0.62
    line.moveTo(center_x - cross_radius, center_y, center_z)
    line.drawTo(center_x + cross_radius, center_y, center_z)
    line.moveTo(center_x, center_y - cross_radius, center_z)
    line.drawTo(center_x, center_y + cross_radius, center_z)
    node = water_target_reticle_root.attachNewNode(line.create())
    node.setTransparency(TransparencyAttrib.MAlpha)
    node.setDepthWrite(False)
    return node


def update_water_targeting_visuals():
    global water_target_reticle_root
    if water_target_reticle_root is None:
        water_target_reticle_root = app.render.attachNewNode("water_target_reticle_root")
        water_target_reticle_root.setTransparency(TransparencyAttrib.MAlpha)
        water_target_reticle_root.setDepthWrite(False)
    clear_water_target_reticles()
    if pregame_active or game_over_triggered:
        return
    selected_node = get_camera_target_node()
    for view in get_all_drone_views():
        if view["role"] != "water":
            continue
        target_hotspot = get_water_current_target_for_view(view)
        if target_hotspot is None:
            continue
        color = (
            (1.0, 0.93, 0.35, 0.92)
            if view["root"] is selected_node
            else (0.36, 0.72, 1.0, 0.76)
        )
        add_water_target_reticle(target_hotspot, color)


def _draw_dashed_path(line, points, dash, gap, phase_shift=0.0):
    """Draw a dashed polyline through 2D points [(x, z), ...] into a LineSegs."""
    period = dash + gap
    min_step = period * 0.02  # guarantees forward progress (no stalls)
    phase = phase_shift % period
    for index in range(len(points) - 1):
        x0, z0 = points[index]
        x1, z1 = points[index + 1]
        segment_length = ((x1 - x0) ** 2 + (z1 - z0) ** 2) ** 0.5
        if segment_length < 1e-6:
            continue
        position = 0.0
        while position < segment_length - 1e-9:
            if phase < dash:
                span = min(dash - phase, segment_length - position)
                span = max(span, min_step)
                span = min(span, segment_length - position)
                t0 = position / segment_length
                t1 = (position + span) / segment_length
                line.moveTo(x0 + (x1 - x0) * t0, 0.0, z0 + (z1 - z0) * t0)
                line.drawTo(x0 + (x1 - x0) * t1, 0.0, z0 + (z1 - z0) * t1)
            else:
                span = min(period - phase, segment_length - position)
                span = max(span, min_step)
                span = min(span, segment_length - position)
            position += span
            phase = (phase + span) % period


def update_fire_map_trajectories():
    """Draw automated drone routes on the little map as DOTTED lines. Survey
    routes are white, water routes are blue, and the selected drone is yellow."""
    global fire_map_trajectory_root
    if fire_map_panel is None:
        return
    if fire_map_trajectory_root is None:
        fire_map_trajectory_root = fire_map_panel.attachNewNode("fire_map_trajectory_root")
        fire_map_trajectory_root.setDepthWrite(False)
        fire_map_trajectory_root.setDepthTest(False)
        fire_map_trajectory_root.setBin("fixed", 108)
    fire_map_trajectory_root.node().removeAllChildren()
    if pregame_active or game_over_triggered:
        return
    selected_node = get_camera_target_node()
    march = (motion_state.sim_time_seconds * 0.02) % 0.024
    for view in get_all_drone_views():
        if is_view_damaged(view):
            continue
        route = get_view_automation_route_world_points(view)
        if len(route) < 2:
            continue
        is_selected = view["root"] is selected_node
        if is_selected:
            color = (1.0, 0.93, 0.35, 0.98)   # yellow = the drone you're on
            thickness = 2.2
        elif view["role"] == "water":
            color = (0.36, 0.72, 1.0, 0.86)
            thickness = 1.8
        else:
            color = (0.95, 0.97, 1.0, 0.72)   # white for the rest
            thickness = 1.4
        points = [world_to_fire_map_coords(wx, wy) for wx, wy in route]
        line = LineSegs(f"fire_map_traj_{view['role']}_{view['slot']}")
        line.setThickness(thickness)
        line.setColor(*color)
        _draw_dashed_path(line, points, dash=0.013, gap=0.011, phase_shift=march)
        node = fire_map_trajectory_root.attachNewNode(line.create())
        node.setBin("fixed", 108)


def world_to_fire_map_coords(x, y):
    x_fraction = 0.0
    y_fraction = 0.0
    if abs(SPAWN_X_MAX - SPAWN_X_MIN) > 0.001:
        x_fraction = (x - SPAWN_X_MIN) / (SPAWN_X_MAX - SPAWN_X_MIN)
    if abs(SPAWN_Y_MAX - SPAWN_Y_MIN) > 0.001:
        y_fraction = (y - SPAWN_Y_MIN) / (SPAWN_Y_MAX - SPAWN_Y_MIN)
    return (
        FIRE_MAP_VIEW_LEFT + (x_fraction * FIRE_MAP_VIEW_WIDTH),
        FIRE_MAP_VIEW_BOTTOM + (y_fraction * FIRE_MAP_VIEW_HEIGHT),
    )


def make_fire_map_square(node_name, half_size, color_rgba, parent=None):
    card_maker = CardMaker(node_name)
    card_maker.setFrame(-half_size, half_size, -half_size, half_size)
    target_parent = fire_map_panel if parent is None else parent
    node = target_parent.attachNewNode(card_maker.generate())
    node.setColor(*color_rgba)
    node.setTransparency(TransparencyAttrib.MAlpha)
    node.setDepthWrite(False)
    node.setDepthTest(False)
    node.setBin("fixed", 105)
    return node


def build_fire_map_circle(radius_meters, color_rgba, parent=None, thickness=1.3):
    line = LineSegs("fire_map_ring")
    line.setThickness(thickness)
    line.setColor(*color_rgba)
    radius_x = FIRE_MAP_VIEW_WIDTH * (
        radius_meters / max(1.0, (SPAWN_X_MAX - SPAWN_X_MIN))
    )
    radius_y = FIRE_MAP_VIEW_HEIGHT * (
        radius_meters / max(1.0, (SPAWN_Y_MAX - SPAWN_Y_MIN))
    )
    for segment_index in range(FIRE_MAP_RING_SEGMENTS + 1):
        angle = tau * (segment_index / FIRE_MAP_RING_SEGMENTS)
        px = cos(angle) * radius_x
        py = sin(angle) * radius_y
        if segment_index == 0:
            line.moveTo(px, 0.0, py)
        else:
            line.drawTo(px, 0.0, py)
    target_parent = fire_map_panel if parent is None else parent
    node = target_parent.attachNewNode(line.create())
    node.setDepthWrite(False)
    node.setDepthTest(False)
    node.setBin("fixed", 104)
    return node


def build_fire_map_cross_marker(parent, color_rgba, half_size=0.012, thickness=2.8):
    line = LineSegs("fire_map_cross")
    line.setThickness(thickness)
    line.setColor(*color_rgba)
    line.moveTo(-half_size, 0.0, -half_size)
    line.drawTo(half_size, 0.0, half_size)
    line.moveTo(-half_size, 0.0, half_size)
    line.drawTo(half_size, 0.0, -half_size)
    marker = parent.attachNewNode(line.create())
    marker.setTransparency(TransparencyAttrib.MAlpha)
    marker.setDepthWrite(False)
    marker.setDepthTest(False)
    marker.setBin("fixed", 112)
    return marker


def _fire_map_rect(parent, x0, x1, z0, z1, color_rgba, bin_order=118):
    card_maker = CardMaker("fire_map_rect")
    card_maker.setFrame(x0, x1, z0, z1)
    node = parent.attachNewNode(card_maker.generate())
    node.setColor(*color_rgba)
    node.setTransparency(TransparencyAttrib.MAlpha)
    node.setDepthWrite(False)
    node.setDepthTest(False)
    node.setBin("fixed", bin_order)
    return node


def clear_fire_map_icon(icon_root):
    if icon_root is None:
        return
    for child in icon_root.getChildren():
        child.removeNode()


def _fire_map_icon_line(
    parent,
    name,
    segments,
    color_rgba=(0.96, 0.99, 1.0, 1.0),
    thickness=2.0,
    bin_order=127,
):
    line = LineSegs(name)
    line.setThickness(thickness)
    line.setColor(*color_rgba)
    for x0, z0, x1, z1 in segments:
        line.moveTo(x0, 0.0, z0)
        line.drawTo(x1, 0.0, z1)
    node = parent.attachNewNode(line.create())
    node.setTransparency(TransparencyAttrib.MAlpha)
    node.setDepthWrite(False)
    node.setDepthTest(False)
    node.setBin("fixed", bin_order)
    return node


def _fire_map_icon_circle(
    parent,
    name,
    center_x,
    center_z,
    radius,
    color_rgba=(0.96, 0.99, 1.0, 1.0),
    thickness=2.0,
    bin_order=127,
):
    line = LineSegs(name)
    line.setThickness(thickness)
    line.setColor(*color_rgba)
    for segment_index in range(25):
        angle = tau * (segment_index / 24.0)
        px = center_x + cos(angle) * radius
        pz = center_z + sin(angle) * radius
        if segment_index == 0:
            line.moveTo(px, 0.0, pz)
        else:
            line.drawTo(px, 0.0, pz)
    node = parent.attachNewNode(line.create())
    node.setTransparency(TransparencyAttrib.MAlpha)
    node.setDepthWrite(False)
    node.setDepthTest(False)
    node.setBin("fixed", bin_order)
    return node


def draw_fire_map_zoom_icon(icon_root):
    clear_fire_map_icon(icon_root)
    icon_color = (0.95, 0.99, 1.0, 1.0)
    _fire_map_icon_circle(icon_root, "fire_map_zoom_lens", -0.006, 0.005, 0.012, icon_color, 2.3)
    _fire_map_icon_line(
        icon_root,
        "fire_map_zoom_handle",
        ((0.004, -0.005, 0.018, -0.019),),
        icon_color,
        2.5,
    )
    symbol_segments = [(-0.013, 0.005, 0.001, 0.005)]
    if not fire_map_expanded:
        symbol_segments.append((-0.006, -0.002, -0.006, 0.012))
    _fire_map_icon_line(
        icon_root,
        "fire_map_zoom_symbol",
        tuple(symbol_segments),
        icon_color,
        2.0,
    )


def build_fire_map_icon_button_frame(name, bounds, color_rgba):
    x0, x1, z0, z1 = bounds
    button_card = CardMaker(name)
    button_card.setFrame(x0, x1, z0, z1)
    frame = fire_map_panel.attachNewNode(button_card.generate())
    frame.setColor(*color_rgba)
    frame.setTransparency(TransparencyAttrib.MAlpha)
    frame.setDepthWrite(False)
    frame.setDepthTest(False)
    frame.setBin("fixed", 126)
    return frame


def build_fire_map_icon_root(name, bounds):
    x0, x1, z0, z1 = bounds
    root = fire_map_panel.attachNewNode(name)
    root.setPos((x0 + x1) * 0.5, 0.0, (z0 + z1) * 0.5)
    root.setDepthWrite(False)
    root.setDepthTest(False)
    root.setBin("fixed", 127)
    return root


def build_fire_map_truck_marker(parent, offset_x=0.018, offset_z=0.004):
    """A tiny fire-truck icon shown on a detected fire while a truck crew is
    actively suppressing it (#8). Grounded in real practice: fast initial-attack
    crews stop ~94% of new Canadian wildfires (BC Wildfire Service)."""
    truck = parent.attachNewNode("fire_map_truck")
    truck.setDepthWrite(False)
    truck.setDepthTest(False)
    truck.setBin("fixed", 118)
    # Sit the truck just to the side of the flame pin so both stay readable.
    truck.setPos(offset_x, 0.0, offset_z)
    red = (0.86, 0.12, 0.12, 1.0)
    dark = (0.08, 0.09, 0.11, 1.0)
    pale = (0.93, 0.95, 0.98, 1.0)
    _fire_map_rect(truck, -0.016, 0.004, 0.0, 0.013, red)        # cargo body
    _fire_map_rect(truck, 0.004, 0.017, 0.0, 0.008, red)         # cab
    _fire_map_rect(truck, 0.006, 0.015, 0.003, 0.0075, pale)     # windshield
    _fire_map_rect(truck, -0.013, -0.007, -0.004, 0.0, dark)     # rear wheel
    _fire_map_rect(truck, 0.006, 0.012, -0.004, 0.0, dark)       # front wheel
    truck.hide()
    return truck


def build_fire_map_legend():
    legend_y = FIRE_MAP_VIEW_BOTTOM - 0.034
    legend_items = (
        ((1.0, 0.55, 0.3, 1.0), "fire"),
        ((1.0, 0.82, 0.4, 1.0), "need water"),
        ((0.5, 0.85, 1.0, 1.0), "water"),
        ((0.46, 0.96, 0.56, 1.0), "out"),
    )
    box_half = 0.013

    def _add_color_legend_item(x, y, color, label, scale=0.027):
        swatch = _fire_map_rect(
            fire_map_panel,
            x,
            x + box_half * 2.0,
            y - box_half,
            y + box_half,
            color,
            bin_order=122,
        )
        swatch.setTransparency(TransparencyAttrib.MAlpha)
        label_text = OnscreenText(
            text=label,
            parent=fire_map_panel,
            pos=(x + 0.034, y - 0.009),
            scale=scale,
            fg=(0.94, 0.98, 1.0, 1.0),
            align=TextNode.ALeft,
            mayChange=False,
        )
        label_text.setBin("fixed", 123)
        label_text.setDepthWrite(False)
        label_text.setDepthTest(False)

    legend_columns = (
        FIRE_MAP_VIEW_LEFT,
        FIRE_MAP_VIEW_LEFT + 0.15,
        FIRE_MAP_VIEW_LEFT + 0.35,
        FIRE_MAP_VIEW_LEFT + 0.50,
    )
    for x, (color, label) in zip(legend_columns, legend_items):
        _add_color_legend_item(x, legend_y, color, label)

    legend_truck = build_fire_map_truck_marker(
        fire_map_panel,
        offset_x=FIRE_MAP_VIEW_LEFT + 0.655,
        offset_z=legend_y - 0.005,
    )
    legend_truck.setScale(1.1)
    legend_truck.show()
    truck_text = OnscreenText(
        text="firetruck",
        parent=fire_map_panel,
        pos=(FIRE_MAP_VIEW_LEFT + 0.690, legend_y - 0.009),
        scale=0.024,
        fg=(1.0, 0.86, 0.82, 1.0),
        align=TextNode.ALeft,
        mayChange=False,
    )
    truck_text.setBin("fixed", 123)
    truck_text.setDepthWrite(False)
    truck_text.setDepthTest(False)

    purple_x = FIRE_MAP_VIEW_LEFT + 0.38
    purple_swatch = _fire_map_rect(
        fire_map_panel,
        purple_x,
        purple_x + box_half * 2.0,
        legend_y - 0.074,
        legend_y - 0.048,
        FIRE_TRUCK_SUPPRESSING_COLOR,
        bin_order=122,
    )
    purple_swatch.setTransparency(TransparencyAttrib.MAlpha)
    purple_text = OnscreenText(
        text="purple fire: suppressed by firetrucks",
        parent=fire_map_panel,
        pos=(purple_x + 0.034, legend_y - 0.065),
        scale=0.024,
        fg=(0.96, 0.86, 1.0, 1.0),
        align=TextNode.ALeft,
        mayChange=False,
    )
    purple_text.setBin("fixed", 123)
    purple_text.setDepthWrite(False)
    purple_text.setDepthTest(False)

    mode_text = OnscreenText(
        text="S survey     W water     yellow selected     M/A mode",
        parent=fire_map_panel,
        pos=(FIRE_MAP_VIEW_LEFT, legend_y - 0.112),
        scale=0.026,
        fg=(0.78, 0.88, 0.96, 1.0),
        align=TextNode.ALeft,
        mayChange=False,
    )
    mode_text.setBin("fixed", 123)
    mode_text.setDepthWrite(False)
    mode_text.setDepthTest(False)

    tab_text = OnscreenText(
        text="[TAB] open / close map",
        parent=fire_map_panel,
        pos=(FIRE_MAP_VIEW_LEFT, legend_y - 0.154),
        scale=0.028,
        fg=(1.0, 0.84, 0.42, 1.0),
        align=TextNode.ALeft,
        mayChange=False,
    )
    tab_text.setBin("fixed", 123)
    tab_text.setDepthWrite(False)
    tab_text.setDepthTest(False)


def build_fire_map_wind_widget():
    global fire_map_wind_arrow_root, fire_map_wind_label

    fire_map_wind_arrow_root = fire_map_panel.attachNewNode("fire_map_wind_arrow_root")
    fire_map_wind_arrow_root.setPos(
        FIRE_MAP_WIND_WIDGET_X,
        0.0,
        FIRE_MAP_WIND_WIDGET_Z,
    )
    fire_map_wind_arrow_root.setDepthWrite(False)
    fire_map_wind_arrow_root.setDepthTest(False)
    fire_map_wind_arrow_root.setBin("fixed", 121)

    arrow_lines = LineSegs("fire_map_wind_arrow")
    arrow_lines.setThickness(2.2)
    arrow_lines.setColor(0.58, 0.9, 1.0, 1.0)
    tail_z = -FIRE_MAP_WIND_ARROW_LENGTH * 0.45
    tip_z = FIRE_MAP_WIND_ARROW_LENGTH * 0.55
    head_back_z = tip_z - FIRE_MAP_WIND_ARROW_HEAD_BACK_OFFSET

    arrow_lines.moveTo(0.0, 0.0, tail_z)
    arrow_lines.drawTo(0.0, 0.0, tip_z)
    arrow_lines.moveTo(0.0, 0.0, tip_z)
    arrow_lines.drawTo(-FIRE_MAP_WIND_ARROW_HEAD_HALF_WIDTH, 0.0, head_back_z)
    arrow_lines.moveTo(0.0, 0.0, tip_z)
    arrow_lines.drawTo(FIRE_MAP_WIND_ARROW_HEAD_HALF_WIDTH, 0.0, head_back_z)

    arrow_node = fire_map_wind_arrow_root.attachNewNode(arrow_lines.create())
    arrow_node.setTransparency(TransparencyAttrib.MAlpha)
    arrow_node.setDepthWrite(False)
    arrow_node.setDepthTest(False)
    arrow_node.setBin("fixed", 122)

    fire_map_wind_label = OnscreenText(
        text="WIND",
        parent=fire_map_panel,
        pos=(
            FIRE_MAP_WIND_WIDGET_X + FIRE_MAP_WIND_LABEL_OFFSET_X,
            FIRE_MAP_WIND_WIDGET_Z - FIRE_MAP_WIND_LABEL_OFFSET_Z,
        ),
        scale=0.024,
        fg=(0.7, 0.92, 1.0, 0.95),
        align=TextNode.ALeft,
        mayChange=True,
    )
    fire_map_wind_label.setBin("fixed", 123)
    fire_map_wind_label.setDepthWrite(False)
    fire_map_wind_label.setDepthTest(False)


def build_fire_map_flame_pin(parent, half_height=0.016, thickness=4.5):
    """A small, clean drawn flame icon (filled-look teardrop) for the fire map."""
    flame_root = parent.attachNewNode("fire_map_flame_pin")
    flame_root.setTransparency(TransparencyAttrib.MAlpha)
    flame_root.setDepthWrite(False)
    flame_root.setDepthTest(False)
    flame_root.setBin("fixed", 113)

    h = half_height
    outline = [
        (0.0, 1.0 * h),
        (0.42 * h, 0.28 * h),
        (0.60 * h, -0.26 * h),
        (0.34 * h, -0.64 * h),
        (0.0, -0.74 * h),
        (-0.34 * h, -0.64 * h),
        (-0.60 * h, -0.26 * h),
        (-0.42 * h, 0.28 * h),
    ]

    outer = LineSegs("fire_map_flame_outer")
    outer.setThickness(thickness)
    outer.setColor(1.0, 0.42, 0.14, 1.0)
    outer.moveTo(outline[0][0], 0.0, outline[0][1])
    for px, pz in outline[1:]:
        outer.drawTo(px, 0.0, pz)
    outer.drawTo(outline[0][0], 0.0, outline[0][1])
    outer_node = flame_root.attachNewNode(outer.create())
    outer_node.setDepthWrite(False)
    outer_node.setDepthTest(False)
    outer_node.setBin("fixed", 113)

    core_scale = 0.5
    inner = LineSegs("fire_map_flame_core")
    inner.setThickness(max(1.5, thickness * 0.5))
    inner.setColor(1.0, 0.86, 0.32, 1.0)
    inner.moveTo(outline[0][0] * core_scale, 0.0, outline[0][1] * core_scale)
    for px, pz in outline[1:]:
        inner.drawTo(px * core_scale, 0.0, pz * core_scale)
    inner.drawTo(outline[0][0] * core_scale, 0.0, outline[0][1] * core_scale)
    inner_node = flame_root.attachNewNode(inner.create())
    inner_node.setDepthWrite(False)
    inner_node.setDepthTest(False)
    inner_node.setBin("fixed", 114)

    flame_root.setPythonTag("outer_node", outer_node)
    flame_root.setPythonTag("inner_node", inner_node)
    return flame_root


def set_fire_map_marker_color(node, color_rgba):
    if node is None:
        return
    node.setColor(*color_rgba)
    node.setColorScale(1.0, 1.0, 1.0, 1.0)


def set_fire_map_flame_pin_colors(flame_node, outer_color, inner_color=None):
    if flame_node is None:
        return
    set_fire_map_marker_color(flame_node, (1.0, 1.0, 1.0, 1.0))
    set_fire_map_marker_color(
        flame_node.getPythonTag("outer_node"),
        outer_color,
    )
    set_fire_map_marker_color(
        flame_node.getPythonTag("inner_node"),
        inner_color or outer_color,
    )


def build_fire_map_hotspot_node():
    root = fire_map_marker_root.attachNewNode("fire_map_hotspot_marker")
    root.setDepthWrite(False)
    root.setDepthTest(False)
    root.setBin("fixed", 110)

    firetruck_suppression_fill_node = make_fire_map_square(
        "fire_map_firetruck_suppression_fill",
        0.017,
        FIRE_TRUCK_SUPPRESSING_COLOR,
        parent=root,
    )
    firetruck_suppression_fill_node.setBin("fixed", 111)
    firetruck_suppression_fill_node.hide()

    flame_node = build_fire_map_flame_pin(root)
    # Small subtle halo (replaces the old oversized 10 m pulse ring).
    halo_node = build_fire_map_circle(
        FIRE_MAP_FIRE_PULSE_INNER_RADIUS_METERS * 0.5,
        (1.0, 0.5, 0.2, 0.32),
        parent=root,
        thickness=1.2,
    )
    cross_node = build_fire_map_cross_marker(
        root,
        (1.0, 0.34, 0.14, 1.0),
        half_size=0.006,
        thickness=2.0,
    )
    water_target_ring_node = build_fire_map_circle(
        WATER_TARGET_RETICLE_RADIUS_METERS,
        (0.34, 0.88, 1.0, 0.86),
        parent=root,
        thickness=2.2,
    )
    water_target_cross_node = build_fire_map_cross_marker(
        root,
        (0.62, 0.94, 1.0, 0.96),
        half_size=0.017,
        thickness=2.6,
    )
    water_target_ring_node.setBin("fixed", 114)
    water_target_cross_node.setBin("fixed", 115)
    water_target_ring_node.hide()
    water_target_cross_node.hide()
    truck_node = build_fire_map_truck_marker(root)
    # Optional status label (kept empty). State is shown by the flame pin + cross color.
    label_text = OnscreenText(
        text="",
        parent=fire_map_panel,
        pos=(0.0, 0.0),
        scale=0.026,
        fg=(1.0, 0.96, 0.92, 0.98),
        align=TextNode.ACenter,
        mayChange=True,
    )
    label_text.setBin("fixed", 120)
    label_text.setDepthWrite(False)
    label_text.setDepthTest(False)

    root.setPythonTag("flame_node", flame_node)
    root.setPythonTag("firetruck_suppression_fill_node", firetruck_suppression_fill_node)
    root.setPythonTag("halo_node", halo_node)
    root.setPythonTag("cross_node", cross_node)
    root.setPythonTag("water_target_ring_node", water_target_ring_node)
    root.setPythonTag("water_target_cross_node", water_target_cross_node)
    root.setPythonTag("truck_node", truck_node)
    root.setPythonTag("label_text", label_text)
    return root


def current_fire_map_scale():
    return FIRE_MAP_EXPANDED_SCALE if fire_map_expanded else FIRE_MAP_COLLAPSED_SCALE


def apply_fire_map_panel_layout():
    if fire_map_panel is None:
        return
    scale = current_fire_map_scale()
    fire_map_panel.setScale(scale)
    fire_map_panel.setPos(
        -(FIRE_MAP_PANEL_WIDTH * scale) - FIRE_MAP_PANEL_RIGHT_MARGIN,
        0.0,
        -(FIRE_MAP_PANEL_TOP_MARGIN + (FIRE_MAP_PANEL_HEIGHT * scale)),
    )


def get_fire_map_size_button_bounds():
    x1 = FIRE_MAP_PANEL_WIDTH - FIRE_MAP_SIZE_BUTTON_RIGHT_MARGIN
    x0 = x1 - FIRE_MAP_SIZE_BUTTON_WIDTH
    z1 = FIRE_MAP_PANEL_HEIGHT - FIRE_MAP_SIZE_BUTTON_TOP_MARGIN
    z0 = z1 - FIRE_MAP_SIZE_BUTTON_HEIGHT
    return x0, x1, z0, z1


def update_fire_map_size_button():
    if fire_map_size_button_frame is not None:
        button_color = (
            (0.10, 0.34, 0.50, 0.92)
            if fire_map_expanded
            else (0.16, 0.42, 0.66, 0.92)
        )
        fire_map_size_button_frame.setColor(*button_color)
    if fire_map_size_button_icon_root is not None:
        draw_fire_map_zoom_icon(fire_map_size_button_icon_root)


def build_fire_map_size_button():
    global fire_map_size_button_frame, fire_map_size_button_icon_root
    bounds = get_fire_map_size_button_bounds()
    fire_map_size_button_frame = build_fire_map_icon_button_frame(
        "fire_map_size_button",
        bounds,
        (0.16, 0.42, 0.66, 0.92),
    )
    fire_map_size_button_icon_root = build_fire_map_icon_root(
        "fire_map_size_button_icon",
        bounds,
    )
    update_fire_map_size_button()


def toggle_fire_map_size():
    global fire_map_expanded
    fire_map_expanded = not fire_map_expanded
    apply_fire_map_panel_layout()
    update_fire_map_size_button()
    update_fire_map_popup()


def build_fire_map_overlay():
    global fire_map_panel, fire_map_view_root, fire_map_marker_root
    global fire_map_header_text, fire_map_prompt_text, fire_map_status_text
    global survey_map_marker_node, survey_map_ring_node, survey_map_label
    global water_map_marker_node, water_map_ring_node, water_map_label
    global fire_map_wind_arrow_root, fire_map_wind_label
    global fire_map_detected_hotspot_nodes, fire_map_drone_nodes
    global fire_map_size_button_frame, fire_map_size_button_icon_root

    fire_map_detected_hotspot_nodes = {}
    fire_map_drone_nodes = {}
    fire_map_wind_arrow_root = None
    fire_map_wind_label = None
    fire_map_size_button_frame = None
    fire_map_size_button_icon_root = None
    fire_map_panel = app.a2dTopRight.attachNewNode("fire_response_map_root")
    apply_fire_map_panel_layout()

    background = CardMaker("fire_response_map_background")
    background.setFrame(0.0, FIRE_MAP_PANEL_WIDTH, 0.0, FIRE_MAP_PANEL_HEIGHT)
    background_np = fire_map_panel.attachNewNode(background.generate())
    background_np.setColor(0.04, 0.05, 0.07, 0.92)
    background_np.setTransparency(TransparencyAttrib.MAlpha)
    background_np.setDepthWrite(False)
    background_np.setDepthTest(False)
    background_np.setBin("fixed", 90)

    view_bg_card = CardMaker("fire_response_map_view_bg")
    view_bg_card.setFrame(
        FIRE_MAP_VIEW_LEFT,
        FIRE_MAP_VIEW_LEFT + FIRE_MAP_VIEW_WIDTH,
        FIRE_MAP_VIEW_BOTTOM,
        FIRE_MAP_VIEW_BOTTOM + FIRE_MAP_VIEW_HEIGHT,
    )
    view_bg_np = fire_map_panel.attachNewNode(view_bg_card.generate())
    view_bg_np.setColor(0.09, 0.17, 0.13, 0.96)
    view_bg_np.setTransparency(TransparencyAttrib.MAlpha)
    view_bg_np.setDepthWrite(False)
    view_bg_np.setDepthTest(False)
    view_bg_np.setBin("fixed", 91)

    frame_lines = LineSegs("fire_map_frame")
    frame_lines.setThickness(1.6)
    frame_lines.setColor(0.96, 0.94, 0.9, 0.75)
    x0 = FIRE_MAP_VIEW_LEFT
    x1 = FIRE_MAP_VIEW_LEFT + FIRE_MAP_VIEW_WIDTH
    y0 = FIRE_MAP_VIEW_BOTTOM
    y1 = FIRE_MAP_VIEW_BOTTOM + FIRE_MAP_VIEW_HEIGHT
    frame_lines.moveTo(x0, 0.0, y0)
    frame_lines.drawTo(x1, 0.0, y0)
    frame_lines.drawTo(x1, 0.0, y1)
    frame_lines.drawTo(x0, 0.0, y1)
    frame_lines.drawTo(x0, 0.0, y0)
    for division_index in range(1, FIRE_MAP_GRID_DIVISIONS):
        x_div = x0 + (FIRE_MAP_VIEW_WIDTH * (division_index / FIRE_MAP_GRID_DIVISIONS))
        y_div = y0 + (FIRE_MAP_VIEW_HEIGHT * (division_index / FIRE_MAP_GRID_DIVISIONS))
        frame_lines.moveTo(x_div, 0.0, y0)
        frame_lines.drawTo(x_div, 0.0, y1)
        frame_lines.moveTo(x0, 0.0, y_div)
        frame_lines.drawTo(x1, 0.0, y_div)
    frame_np = fire_map_panel.attachNewNode(frame_lines.create())
    frame_np.setDepthWrite(False)
    frame_np.setDepthTest(False)
    frame_np.setBin("fixed", 92)

    for tree_node in trees:
        tree_x, tree_y = world_to_fire_map_coords(tree_node.getX(), tree_node.getY())
        tree_dot = CardMaker(f"fire_map_tree_dot_{tree_x:.3f}_{tree_y:.3f}")
        tree_dot.setFrame(
            -FIRE_MAP_TREE_DOT_HALF_SIZE,
            FIRE_MAP_TREE_DOT_HALF_SIZE,
            -FIRE_MAP_TREE_DOT_HALF_SIZE,
            FIRE_MAP_TREE_DOT_HALF_SIZE,
        )
        tree_np = fire_map_panel.attachNewNode(tree_dot.generate())
        tree_np.setPos(tree_x, 0.0, tree_y)
        tree_np.setColor(0.18, 0.38, 0.22, 0.46)
        tree_np.setTransparency(TransparencyAttrib.MAlpha)
        tree_np.setDepthWrite(False)
        tree_np.setDepthTest(False)
        tree_np.setBin("fixed", 93)

    fire_map_marker_root = fire_map_panel.attachNewNode("fire_map_marker_root")
    fire_map_marker_root.setBin("fixed", 100)
    build_fire_map_size_button()
    build_fire_map_legend()

    fire_map_header_text = OnscreenText(
        text="DETECTED FIRE MAP",
        parent=fire_map_panel,
        pos=(FIRE_MAP_PANEL_WIDTH * 0.5, FIRE_MAP_PANEL_HEIGHT - 0.05),
        scale=0.045,
        fg=(1.0, 0.84, 0.42, 1.0),
        align=TextNode.ACenter,
        mayChange=True,
    )
    fire_map_prompt_text = OnscreenText(
        text="Detected fire markers only: undetected fires stay hidden.",
        parent=fire_map_panel,
        pos=(FIRE_MAP_VIEW_LEFT, 0.094),
        scale=0.03,
        fg=(0.98, 0.96, 0.9, 1.0),
        align=TextNode.ALeft,
        mayChange=True,
        wordwrap=29.0,
    )
    fire_map_status_text = OnscreenText(
        text="No detected fires yet. S survey, W water.",
        parent=fire_map_panel,
        pos=(FIRE_MAP_VIEW_LEFT, 0.044),
        scale=0.024,
        fg=(0.86, 0.92, 0.96, 1.0),
        align=TextNode.ALeft,
        mayChange=True,
        wordwrap=33.0,
    )

    survey_map_marker_node = None
    survey_map_ring_node = None
    survey_map_label = None
    water_map_marker_node = None
    water_map_ring_node = None
    water_map_label = None

    build_fire_map_wind_widget()
    fire_map_panel.show()


def compute_safe_altitude_at(
    x,
    y,
    base_altitude,
    min_altitude_above_ground_meters,
    canopy_clearance_meters=0.0,
    canopy_query_radius_meters=AUTOMATION_CANOPY_QUERY_RADIUS_METERS,
):
    safe_altitude = base_altitude
    ground_sample = sample_ground(x, y)
    if ground_sample is not None:
        safe_altitude = max(
            safe_altitude,
            ground_sample[0].z + min_altitude_above_ground_meters,
        )

    if canopy_clearance_meters > 0.0 and tree_canopy_samples:
        canopy_height = compute_local_canopy_height(
            tree_canopy_samples,
            x,
            y,
            canopy_query_radius_meters,
        )
        if canopy_height is not None:
            safe_altitude = max(
                safe_altitude,
                canopy_height + canopy_clearance_meters,
            )

    return safe_altitude


def build_canopy_probe_points(start_x, start_y, end_x, end_y):
    delta_x = end_x - start_x
    delta_y = end_y - start_y
    return tuple(
        (
            start_x + (delta_x * fraction),
            start_y + (delta_y * fraction),
        )
        for fraction in AUTOMATION_CANOPY_PATH_PROBE_FRACTIONS
    )


def compute_probe_path_canopy_height(probe_points):
    if not tree_canopy_samples:
        return None

    max_canopy_height = None
    for probe_x, probe_y in probe_points:
        canopy_height = compute_local_canopy_height(
            tree_canopy_samples,
            probe_x,
            probe_y,
            AUTOMATION_CANOPY_QUERY_RADIUS_METERS,
        )
        if canopy_height is None:
            continue
        if max_canopy_height is None or canopy_height > max_canopy_height:
            max_canopy_height = canopy_height

    return max_canopy_height


def resolve_third_person_camera_obstruction(
    target_x,
    target_y,
    target_z,
    camera_x,
    camera_y,
    camera_z,
):
    if not tree_canopy_samples:
        return camera_x, camera_y, camera_z

    resolved_x = camera_x
    resolved_y = camera_y
    resolved_z = camera_z

    for _ in range(THIRD_PERSON_CAMERA_MAX_OBSTRUCTION_PASSES):
        required_lift = 0.0
        for fraction in THIRD_PERSON_CAMERA_PATH_PROBE_FRACTIONS:
            probe_x = target_x + ((resolved_x - target_x) * fraction)
            probe_y = target_y + ((resolved_y - target_y) * fraction)
            probe_z = target_z + ((resolved_z - target_z) * fraction)
            canopy_height = compute_local_canopy_height(
                tree_canopy_samples,
                probe_x,
                probe_y,
                THIRD_PERSON_CAMERA_CANOPY_QUERY_RADIUS_METERS,
            )
            if canopy_height is None:
                continue
            lift_needed = (canopy_height + THIRD_PERSON_CAMERA_CANOPY_BUFFER_METERS) - probe_z
            if lift_needed > required_lift:
                required_lift = lift_needed

        if required_lift <= 0.0:
            break

        target_distance = ((resolved_x - target_x) ** 2 + (resolved_y - target_y) ** 2) ** 0.5
        if (
            required_lift > THIRD_PERSON_CAMERA_MAX_VERTICAL_LIFT_METERS
            and target_distance > THIRD_PERSON_CAMERA_MIN_DISTANCE
        ):
            scale = THIRD_PERSON_CAMERA_OBSTRUCTION_DISTANCE_SCALE
            desired_distance = max(
                THIRD_PERSON_CAMERA_MIN_DISTANCE,
                target_distance * scale,
            )
            if target_distance > 0.001:
                distance_scale = desired_distance / target_distance
                resolved_x = target_x + ((resolved_x - target_x) * distance_scale)
                resolved_y = target_y + ((resolved_y - target_y) * distance_scale)
            continue

        resolved_z += min(required_lift, THIRD_PERSON_CAMERA_MAX_VERTICAL_LIFT_METERS)

    local_canopy = compute_local_canopy_height(
        tree_canopy_samples,
        resolved_x,
        resolved_y,
        THIRD_PERSON_CAMERA_CANOPY_QUERY_RADIUS_METERS,
    )
    if local_canopy is not None:
        resolved_z = max(
            resolved_z,
            local_canopy + THIRD_PERSON_CAMERA_CANOPY_BUFFER_METERS,
        )

    return resolved_x, resolved_y, resolved_z


def resolve_first_person_camera_eye(eye_x, eye_y, eye_z, look_dir):
    resolved_x = eye_x
    resolved_y = eye_y
    resolved_z = eye_z

    horizontal_look = Vec3(look_dir.x, look_dir.y, 0.0)
    if horizontal_look.lengthSquared() > 0.0001:
        horizontal_look.normalize()
        resolved_x += horizontal_look.x * FIRST_PERSON_CAMERA_FORWARD_OFFSET_METERS
        resolved_y += horizontal_look.y * FIRST_PERSON_CAMERA_FORWARD_OFFSET_METERS

    if tree_canopy_samples:
        for probe_x, probe_y in ((eye_x, eye_y), (resolved_x, resolved_y)):
            canopy_height = compute_local_canopy_height(
                tree_canopy_samples,
                probe_x,
                probe_y,
                FIRST_PERSON_CAMERA_CANOPY_QUERY_RADIUS_METERS,
            )
            if canopy_height is not None:
                resolved_z = max(
                    resolved_z,
                    canopy_height + FIRST_PERSON_CAMERA_CANOPY_BUFFER_METERS,
                )

    return resolved_x, resolved_y, resolved_z


def get_water_stream_state_value(stream_state, key):
    if isinstance(stream_state, dict):
        return stream_state.get(key)
    return getattr(stream_state, key, None)


def set_water_stream_state_value(stream_state, key, value):
    if isinstance(stream_state, dict):
        stream_state[key] = value
    else:
        setattr(stream_state, key, value)


def get_water_tank_liters(stream_state):
    if stream_state is None:
        return 0.0
    liters = get_water_stream_state_value(stream_state, "water_liters")
    if liters is None:
        liters = WATER_DRONE_TANK_CAPACITY_LITERS
        set_water_stream_state_value(stream_state, "water_liters", liters)
    return max(0.0, float(liters))


def set_water_tank_liters(stream_state, liters):
    if stream_state is None:
        return
    remaining_liters = clamp(
        float(liters),
        0.0,
        WATER_DRONE_TANK_CAPACITY_LITERS,
    )
    if remaining_liters <= WATER_DRONE_EMPTY_EPSILON_LITERS:
        remaining_liters = 0.0
    set_water_stream_state_value(stream_state, "water_liters", remaining_liters)


def water_tank_has_spray(stream_state):
    return get_water_tank_liters(stream_state) > WATER_DRONE_EMPTY_EPSILON_LITERS


def consume_water_tank(stream_state, dt):
    if stream_state is None:
        return False
    starting_liters = get_water_tank_liters(stream_state)
    if starting_liters <= WATER_DRONE_EMPTY_EPSILON_LITERS:
        set_water_tank_liters(stream_state, 0.0)
        return False
    used_liters = min(
        starting_liters,
        WATER_DRONE_SPRAY_FLOW_LITERS_PER_SECOND * max(0.0, dt),
    )
    if used_liters <= 0.0:
        return False
    set_water_tank_liters(stream_state, starting_liters - used_liters)
    return True


def engage_water_suppression_if_possible(stream_state, is_in_suppression_range, dt):
    if not is_in_suppression_range:
        return False
    return consume_water_tank(stream_state, dt)


def format_water_tank_label(stream_state):
    liters = get_water_tank_liters(stream_state)
    if liters <= WATER_DRONE_EMPTY_EPSILON_LITERS:
        return "EMPTY"
    return f"{liters:.0f}L"


def format_water_tank_label_for_view(view):
    return format_water_tank_label(get_water_view_stream_state(view))


def clear_water_suppression_visual(stream_state=None):
    if stream_state is None:
        stream_state = water_drone_state

    stream_outer_node = get_water_stream_state_value(stream_state, "stream_outer_node")
    if stream_outer_node is not None:
        stream_outer_node.removeNode()
        set_water_stream_state_value(stream_state, "stream_outer_node", None)
    stream_inner_node = get_water_stream_state_value(stream_state, "stream_inner_node")
    if stream_inner_node is not None:
        stream_inner_node.removeNode()
        set_water_stream_state_value(stream_state, "stream_inner_node", None)

    stream_root = get_water_stream_state_value(stream_state, "stream_root")
    if stream_root is None:
        return
    for mist_node in get_water_stream_state_value(stream_state, "mist_nodes") or tuple():
        mist_node.hide()
    stream_root.hide()


def update_water_suppression_visual(
    target_hotspot,
    source_root=None,
    stream_state=None,
    engaged=None,
):
    if source_root is None:
        source_root = water_drone
    if stream_state is None:
        stream_state = water_drone_state

    stream_root = get_water_stream_state_value(stream_state, "stream_root")
    if stream_root is None:
        return

    clear_water_suppression_visual(stream_state)
    if engaged is None:
        engaged = target_hotspot is not None and target_hotspot.water_drone_engaged
    if target_hotspot is None or not engaged:
        return

    stream_start = Vec3(
        source_root.getX(),
        source_root.getY(),
        source_root.getZ() - 0.55,
    )
    spray_phase = motion_state.sim_time_seconds * 6.2
    wind_vx, wind_vy, _, wind_speed_mps = get_wind_velocity()
    impact_center = Vec3(
        target_hotspot.root.getX() + (sin(spray_phase * 0.9) * 0.4) + (wind_vx * 0.08),
        target_hotspot.root.getY() + (cos(spray_phase * 1.07) * 0.35) + (wind_vy * 0.08),
        target_hotspot.ground_z + WATER_DRONE_STREAM_IMPACT_HEIGHT_METERS,
    )

    stream_vector = impact_center - stream_start
    if stream_vector.lengthSquared() <= 0.001:
        return

    stream_direction = Vec3(stream_vector)
    stream_direction.normalize()
    lateral_axis = Vec3(-stream_direction.y, stream_direction.x, 0.0)
    if lateral_axis.lengthSquared() <= 0.001:
        lateral_axis = Vec3(1.0, 0.0, 0.0)
    lateral_axis.normalize()
    stream_root.show()

    def _build_stream_line(node_name, thickness, color_rgba):
        stream_line = LineSegs(node_name)
        stream_line.setThickness(thickness)
        stream_line.setColor(*color_rgba)
        for point_index in range(WATER_DRONE_STREAM_POINT_COUNT):
            t = point_index / max(1, WATER_DRONE_STREAM_POINT_COUNT - 1)
            arc_point = stream_start + (stream_vector * t)
            lateral_wobble = (
                sin(spray_phase + (t * 7.4))
                * WATER_DRONE_STREAM_WOBBLE_METERS
                * (1.0 - t)
            )
            arc_point += lateral_axis * lateral_wobble
            arc_point.x += wind_vx * t * 0.18
            arc_point.y += wind_vy * t * 0.18
            arc_point.z -= sin(t * tau * 0.5) * (0.3 + wind_speed_mps * 0.03)
            if point_index == 0:
                stream_line.moveTo(arc_point.x, arc_point.y, arc_point.z)
            else:
                stream_line.drawTo(arc_point.x, arc_point.y, arc_point.z)
        line_node = stream_root.attachNewNode(stream_line.create())
        line_node.setTransparency(TransparencyAttrib.MAlpha)
        line_node.setDepthWrite(False)
        return line_node

    set_water_stream_state_value(
        stream_state,
        "stream_outer_node",
        _build_stream_line(
            "water_stream_outer",
            WATER_DRONE_STREAM_OUTER_THICKNESS,
            (0.56, 0.84, 1.0, 0.24),
        ),
    )
    set_water_stream_state_value(
        stream_state,
        "stream_inner_node",
        _build_stream_line(
            "water_stream_inner",
            WATER_DRONE_STREAM_INNER_THICKNESS,
            (0.86, 0.96, 1.0, 0.82),
        ),
    )

    mist_nodes = get_water_stream_state_value(stream_state, "mist_nodes") or tuple()
    mist_count = max(1, len(mist_nodes))
    for mist_index, mist_node in enumerate(mist_nodes):
        angle = spray_phase + ((tau / mist_count) * mist_index)
        radius = 0.34 + (0.08 * mist_index)
        mist_node.setPos(
            impact_center.x + cos(angle) * radius,
            impact_center.y + sin(angle * 1.13) * radius,
            impact_center.z + 0.16 + (0.08 * mist_index),
        )
        mist_node.setScale(
            WATER_DRONE_STREAM_MIST_SCALE_METERS
            * (0.7 + mist_index * 0.12)
        )
        mist_node.setColor(
            0.8,
            0.94,
            1.0,
            max(0.18, 0.42 - mist_index * 0.06),
        )
        mist_node.show()


def compute_tree_avoidance_velocity(origin_x, origin_y, forward_dx, forward_dy, max_horizontal_speed):
    if "trees" not in globals():
        return 0.0, 0.0

    forward_norm = (forward_dx * forward_dx + forward_dy * forward_dy) ** 0.5
    if forward_norm <= 0.001:
        return 0.0, 0.0

    unit_forward_x = forward_dx / forward_norm
    unit_forward_y = forward_dy / forward_norm
    lookahead_distance = max_horizontal_speed * AUTOMATION_TREE_LOOKAHEAD_SECONDS

    probe_points = (
        (origin_x, origin_y, 1.0),
        (
            origin_x + unit_forward_x * lookahead_distance,
            origin_y + unit_forward_y * lookahead_distance,
            1.25,
        ),
    )

    avoid_x = 0.0
    avoid_y = 0.0
    avoid_radius_sq = AUTOMATION_TREE_AVOID_RADIUS_METERS * AUTOMATION_TREE_AVOID_RADIUS_METERS

    for tree_x, tree_y in iter_nearby_tree_avoidance_positions(
        probe_points,
        AUTOMATION_TREE_AVOID_RADIUS_METERS,
    ):
        for probe_x, probe_y, probe_weight in probe_points:
            diff_x = probe_x - tree_x
            diff_y = probe_y - tree_y
            distance_sq = diff_x * diff_x + diff_y * diff_y

            if distance_sq <= 0.00001 or distance_sq >= avoid_radius_sq:
                continue

            distance = distance_sq ** 0.5
            closeness = 1.0 - (distance / AUTOMATION_TREE_AVOID_RADIUS_METERS)
            repulse_strength = (closeness * closeness) * probe_weight
            avoid_x += (diff_x / distance) * repulse_strength
            avoid_y += (diff_y / distance) * repulse_strength

    avoid_norm = (avoid_x * avoid_x + avoid_y * avoid_y) ** 0.5
    if avoid_norm <= 0.0001:
        return 0.0, 0.0

    avoid_speed = max_horizontal_speed * AUTOMATION_TREE_AVOID_GAIN * min(1.0, avoid_norm)
    return (avoid_x / avoid_norm) * avoid_speed, (avoid_y / avoid_norm) * avoid_speed


def compute_survey_orbit_velocity(hotspot, dt):
    if hotspot is not team_state.survey_focus_hotspot:
        team_state.survey_focus_hotspot = hotspot
        team_state.survey_orbit_phase_radians = atan2(
            drone.getY() - hotspot.root.getY(),
            drone.getX() - hotspot.root.getX(),
        )

    orbit_angular_speed = (
        SURVEY_ORBIT_SPEED_METERS_PER_SECOND
        / max(1.0, SURVEY_ORBIT_RADIUS_METERS)
    )
    team_state.survey_orbit_phase_radians = (
        team_state.survey_orbit_phase_radians
        + orbit_angular_speed * dt
    ) % tau

    orbit_target_x = hotspot.root.getX() + (
        cos(team_state.survey_orbit_phase_radians) * SURVEY_ORBIT_RADIUS_METERS
    )
    orbit_target_y = hotspot.root.getY() + (
        sin(team_state.survey_orbit_phase_radians) * SURVEY_ORBIT_RADIUS_METERS
    )
    dx = orbit_target_x - drone.getX()
    dy = orbit_target_y - drone.getY()
    horizontal_distance = (dx * dx + dy * dy) ** 0.5
    max_horizontal_speed = get_current_drone_speed_units_per_second() * AUTOMATION_SPEED_FACTOR

    if horizontal_distance > 0.001:
        speed_scale = clamp(horizontal_distance / SURVEY_ORBIT_RADIUS_METERS, 0.3, 1.0)
        desired_vx = dx / horizontal_distance * max_horizontal_speed * speed_scale
        desired_vy = dy / horizontal_distance * max_horizontal_speed * speed_scale
    else:
        desired_vx = 0.0
        desired_vy = 0.0

    drone_avoid_vx, drone_avoid_vy = compute_drone_avoidance_velocity(
        drone,
        drone.getX(),
        drone.getY(),
        desired_vx,
        desired_vy,
        max_horizontal_speed,
    )
    desired_vx += drone_avoid_vx
    desired_vy += drone_avoid_vy
    orbit_speed = (desired_vx * desired_vx + desired_vy * desired_vy) ** 0.5
    if orbit_speed > max_horizontal_speed and orbit_speed > 0.001:
        desired_vx = (desired_vx / orbit_speed) * max_horizontal_speed
        desired_vy = (desired_vy / orbit_speed) * max_horizontal_speed

    desired_altitude = max(
        AUTOMATION_CRUISE_ALTITUDE_METERS,
        hotspot.ground_z + SURVEY_ORBIT_ALTITUDE_OFFSET_METERS,
    )
    if AUTOMATION_ENABLE_CANOPY_CLEARANCE:
        orbit_probe_points = list(
            build_canopy_probe_points(
                drone.getX(),
                drone.getY(),
                orbit_target_x,
                orbit_target_y,
            )
        )
        orbit_probe_points.append((hotspot.root.getX(), hotspot.root.getY()))
        orbit_canopy_height = compute_probe_path_canopy_height(orbit_probe_points)
        if orbit_canopy_height is not None:
            desired_altitude = max(
                desired_altitude,
                orbit_canopy_height + AUTOMATION_CANOPY_CLEARANCE_METERS,
            )
    altitude_error = desired_altitude - drone.getZ()
    max_vertical_speed = (
        get_current_drone_speed_units_per_second()
        * AUTOMATION_VERTICAL_SPEED_FACTOR
    )
    desired_vz = clamp(
        altitude_error * AUTOMATION_ALTITUDE_GAIN,
        -max_vertical_speed,
        max_vertical_speed,
    )
    return desired_vx, desired_vy, desired_vz


def update_survey_mapping_progress(dt):
    survey_roots = get_active_survey_drone_roots()
    for hotspot in unresolved_detected_hotspots():
        mapping_drone_count = 0
        for survey_root in survey_roots:
            if survey_root is None or survey_root.isEmpty():
                continue
            horizontal_dx = survey_root.getX() - hotspot.root.getX()
            horizontal_dy = survey_root.getY() - hotspot.root.getY()
            horizontal_distance = (horizontal_dx * horizontal_dx + horizontal_dy * horizontal_dy) ** 0.5
            altitude_above_hotspot = survey_root.getZ() - hotspot.ground_z
            if altitude_above_hotspot < FIRE_HOTSPOT_MIN_SCAN_ALTITUDE_METERS:
                continue
            if abs(horizontal_distance - SURVEY_ORBIT_RADIUS_METERS) > SURVEY_ORBIT_CAPTURE_BAND_METERS:
                continue
            mapping_drone_count += 1
        if mapping_drone_count <= 0:
            continue
        hotspot.mapping_progress_seconds = min(
            SURVEY_MAP_BUILD_TIME_SECONDS,
            hotspot.mapping_progress_seconds + (dt * mapping_drone_count),
        )


def choose_automation_search_waypoint():
    spread_sources = [
        hotspot
        for hotspot in fire_hotspots
        if hotspot.suppression_state == FIRE_STATE_ACTIVE
    ]
    for _ in range(AUTOMATION_SEARCH_WAYPOINT_ATTEMPTS):
        if spread_sources:
            anchor_hotspot = random.choice(spread_sources)
            anchor_x = anchor_hotspot.root.getX()
            anchor_y = anchor_hotspot.root.getY()
            search_angle = random.uniform(0.0, tau)
            search_distance = random.uniform(
                FIRE_SPREAD_MIN_DISTANCE_METERS * AUTOMATION_SEARCH_SPAWN_RING_MIN_FACTOR,
                FIRE_SPREAD_MAX_DISTANCE_METERS * AUTOMATION_SEARCH_SPAWN_RING_MAX_FACTOR,
            )
            candidate_x = anchor_x + cos(search_angle) * search_distance
            candidate_y = anchor_y + sin(search_angle) * search_distance
        else:
            unburned_cells = tuple(fire_burnable_cells.difference(fire_burned_cells))
            if not unburned_cells:
                return None
            cell_x, cell_y = random.choice(unburned_cells)
            candidate_x, candidate_y = fire_cell_to_world(cell_x, cell_y)

        if (
            candidate_x < SPAWN_X_MIN
            or candidate_x > SPAWN_X_MAX
            or candidate_y < SPAWN_Y_MIN
            or candidate_y > SPAWN_Y_MAX
        ):
            continue

        candidate_cell = world_to_fire_cell(candidate_x, candidate_y)
        if candidate_cell in fire_burned_cells:
            continue

        ground_sample = sample_ground(candidate_x, candidate_y)
        if ground_sample is None:
            continue
        return candidate_x, candidate_y, ground_sample[0].z

    unburned_cells = tuple(fire_burnable_cells.difference(fire_burned_cells))
    if not unburned_cells:
        return None
    cell_x, cell_y = random.choice(unburned_cells)
    fallback_x, fallback_y = fire_cell_to_world(cell_x, cell_y)
    fallback_ground = sample_ground(fallback_x, fallback_y)
    if fallback_ground is None:
        return None
    return fallback_x, fallback_y, fallback_ground[0].z


def choose_lead_survey_sector_waypoint():
    global survey_lead_patrol_sector_label, survey_lead_patrol_waypoints
    global survey_lead_patrol_waypoint_index

    sector = get_survey_search_sector(1)
    if sector is None:
        return choose_automation_search_waypoint()
    if (
        survey_lead_patrol_sector_label != sector.label
        or not survey_lead_patrol_waypoints
    ):
        survey_lead_patrol_sector_label = sector.label
        survey_lead_patrol_waypoints = rotate_waypoints_to_nearest(
            build_survey_sector_waypoints(sector),
            drone.getX(),
            drone.getY(),
        )
        survey_lead_patrol_waypoint_index = 0

    if not survey_lead_patrol_waypoints:
        center_x, center_y = sector.center()
        ground = sample_ground(center_x, center_y)
        center_z = ground[0].z if ground is not None else 0.0
        return patrol_waypoint_with_safe_altitude((center_x, center_y, center_z))

    for _ in range(len(survey_lead_patrol_waypoints)):
        waypoint = survey_lead_patrol_waypoints[
            survey_lead_patrol_waypoint_index % len(survey_lead_patrol_waypoints)
        ]
        safe_waypoint = patrol_waypoint_with_safe_altitude(waypoint)
        dx = safe_waypoint[0] - drone.getX()
        dy = safe_waypoint[1] - drone.getY()
        if (
            (dx * dx + dy * dy) ** 0.5 <= AUTOMATION_SEARCH_REACH_RADIUS_METERS
            or is_search_waypoint_exhausted(safe_waypoint)
        ):
            survey_lead_patrol_waypoint_index = (
                survey_lead_patrol_waypoint_index + 1
            ) % len(survey_lead_patrol_waypoints)
            continue
        return safe_waypoint

    survey_lead_patrol_waypoint_index = (
        survey_lead_patrol_waypoint_index + 1
    ) % len(survey_lead_patrol_waypoints)
    return patrol_waypoint_with_safe_altitude(
        survey_lead_patrol_waypoints[survey_lead_patrol_waypoint_index]
    )


def choose_automation_hotspot_target(
    drone_x,
    drone_y,
    excluded_hotspot=None,
    sector=None,
    allow_out_of_sector_fallback=False,
    survey_slot=1,
):
    def _eligible(hotspot, respect_cooldown):
        if hotspot is excluded_hotspot:
            return False
        if hotspot.suppression_state in (FIRE_STATE_OUT, FIRE_STATE_BURNED) or hotspot.detected:
            return False
        if not survey_slot_may_target_hotspot(survey_slot, hotspot):
            return False
        if (
            respect_cooldown
            and (motion_state.sim_time_seconds - hotspot.last_flyby_time_seconds)
            < AUTOMATION_HOTSPOT_REVISIT_DELAY_SECONDS
        ):
            return False
        return True

    def _sector_filter(hotspot):
        return sector is None or sector.contains(hotspot.root.getX(), hotspot.root.getY())

    cooled_candidates = [
        hotspot
        for hotspot in fire_hotspots
        if _eligible(hotspot, True) and _sector_filter(hotspot)
    ]
    if cooled_candidates:
        return min(
            cooled_candidates,
            key=lambda hotspot: (hotspot.root.getX() - drone_x) ** 2 + (hotspot.root.getY() - drone_y) ** 2,
        )

    fallback_candidates = [
        hotspot
        for hotspot in fire_hotspots
        if _eligible(hotspot, False) and _sector_filter(hotspot)
    ]
    if (
        not fallback_candidates
        and sector is not None
        and allow_out_of_sector_fallback
    ):
        fallback_candidates = [
            hotspot
            for hotspot in fire_hotspots
            if _eligible(hotspot, False)
        ]
    if fallback_candidates:
        return min(
            fallback_candidates,
            key=lambda hotspot: (hotspot.root.getX() - drone_x) ** 2 + (hotspot.root.getY() - drone_y) ** 2,
        )

    return None


def compute_drone_avoidance_velocity(
    drone_root,
    drone_x,
    drone_y,
    forward_dx,
    forward_dy,
    max_horizontal_speed,
):
    if max_horizontal_speed <= 0.001:
        return 0.0, 0.0
    if "get_active_drone_motion_entries" not in globals():
        return 0.0, 0.0

    radius = DRONE_INTERACTION_AVOIDANCE_RADIUS_METERS
    lookahead_seconds = DRONE_INTERACTION_AVOIDANCE_LOOKAHEAD_SECONDS
    self_future_x = drone_x + (forward_dx * lookahead_seconds)
    self_future_y = drone_y + (forward_dy * lookahead_seconds)
    avoid_x = 0.0
    avoid_y = 0.0
    forward_norm = (forward_dx * forward_dx + forward_dy * forward_dy) ** 0.5
    self_view = next(
        (view for view in get_all_drone_views() if view["root"] is drone_root),
        None,
    )
    self_slot = self_view["slot"] if self_view is not None else 1
    self_role_offset = 3 if self_view is not None and self_view["role"] == "water" else 0
    pass_direction = -1.0 if ((self_slot + self_role_offset) % 2) else 1.0

    for entry in get_active_drone_motion_entries():
        other_root = entry.get("root")
        if other_root is drone_root or other_root is None or other_root.isEmpty():
            continue

        other_motion = entry.get("motion_state")
        other_vx = other_motion.velocity.x if other_motion is not None else 0.0
        other_vy = other_motion.velocity.y if other_motion is not None else 0.0

        current_diff_x = drone_x - other_root.getX()
        current_diff_y = drone_y - other_root.getY()
        current_distance = (current_diff_x * current_diff_x + current_diff_y * current_diff_y) ** 0.5

        other_future_x = other_root.getX() + (other_vx * lookahead_seconds)
        other_future_y = other_root.getY() + (other_vy * lookahead_seconds)
        future_diff_x = self_future_x - other_future_x
        future_diff_y = self_future_y - other_future_y
        future_distance = (future_diff_x * future_diff_x + future_diff_y * future_diff_y) ** 0.5

        effective_distance = min(current_distance, future_distance * 0.85)
        if effective_distance >= radius:
            continue

        if future_distance < current_distance and future_distance > 0.001:
            unit_x = future_diff_x / future_distance
            unit_y = future_diff_y / future_distance
        elif current_distance > 0.001:
            unit_x = current_diff_x / current_distance
            unit_y = current_diff_y / current_distance
        else:
            slot_bias = get_survey_slot_for_root(drone_root) or 1
            angle = radians(120.0 * (slot_bias - 1))
            unit_x = cos(angle)
            unit_y = sin(angle)

        closeness = 1.0 - (effective_distance / radius)
        stuck_closeness = max(
            0.0,
            1.0 - (effective_distance / DRONE_INTERACTION_STUCK_DISTANCE_METERS),
        )
        repulse_speed = (
            max_horizontal_speed
            * DRONE_INTERACTION_AVOIDANCE_GAIN
            * closeness
            * closeness
            * (1.0 + (1.25 * stuck_closeness))
        )
        avoid_x += unit_x * repulse_speed
        avoid_y += unit_y * repulse_speed

        if forward_norm > 0.001:
            perp_x = -unit_y
            perp_y = unit_x
            if (perp_x * forward_dx + perp_y * forward_dy) < 0.0:
                perp_x = -perp_x
                perp_y = -perp_y
            side_speed = (
                max_horizontal_speed
                * DRONE_INTERACTION_AVOIDANCE_GAIN
                * 0.42
                * closeness
            )
            avoid_x += perp_x * side_speed
            avoid_y += perp_y * side_speed

        if stuck_closeness > 0.0:
            if forward_norm > 0.001:
                pass_x = -forward_dy / forward_norm
                pass_y = forward_dx / forward_norm
            else:
                pass_x = -unit_y
                pass_y = unit_x
            pass_x *= pass_direction
            pass_y *= pass_direction
            pass_speed = (
                max_horizontal_speed
                * DRONE_INTERACTION_STUCK_PASS_AROUND_FACTOR
                * stuck_closeness
            )
            avoid_x += pass_x * pass_speed
            avoid_y += pass_y * pass_speed

    avoid_speed = (avoid_x * avoid_x + avoid_y * avoid_y) ** 0.5
    max_avoid_speed = max_horizontal_speed * DRONE_INTERACTION_AVOIDANCE_MAX_SPEED_FACTOR
    if avoid_speed > max_avoid_speed and avoid_speed > 0.001:
        avoid_x = (avoid_x / avoid_speed) * max_avoid_speed
        avoid_y = (avoid_y / avoid_speed) * max_avoid_speed
    return avoid_x, avoid_y


def compute_automation_velocity(dt):
    global automation_target_hotspot, automation_search_waypoint

    drone_x = drone.getX()
    drone_y = drone.getY()
    response_hotspot = choose_survey_mapping_hotspot_for_slot(1, drone_x, drone_y)
    if should_survey_orbit_hotspot(response_hotspot, dt):
        automation_target_hotspot = None
        automation_search_waypoint = None
        return compute_survey_orbit_velocity(response_hotspot, dt)

    team_state.survey_focus_hotspot = None
    survey_sector = get_survey_search_sector(1)
    if AUTOMATION_KNOWS_UNDETECTED_FIRES:
        candidate_hotspot = choose_automation_hotspot_target(
            drone_x,
            drone_y,
            sector=survey_sector,
        )
    else:
        # No prior fire knowledge: stick to the S-shaped search pattern and
        # let the thermal sensor find fires along the way.
        candidate_hotspot = None

    target_hotspot = candidate_hotspot
    automation_target_hotspot = target_hotspot

    target_world_z = drone.getZ()
    using_hotspot_target = target_hotspot is not None

    if using_hotspot_target:
        target_world_x = target_hotspot.root.getX()
        target_world_y = target_hotspot.root.getY()
        target_world_z = target_hotspot.ground_z
    else:
        global survey_lead_waypoint_timer_seconds
        global survey_lead_patrol_waypoint_index

        if automation_search_waypoint is not None:
            if is_search_waypoint_exhausted(automation_search_waypoint):
                automation_search_waypoint = None

        # Anti-stall: skip a patrol leg only when it takes far longer than
        # its distance justifies (avoidance deadlock near corners).
        survey_lead_waypoint_timer_seconds += dt
        if automation_search_waypoint is not None:
            stall_distance = (
                (automation_search_waypoint[0] - drone_x) ** 2
                + (automation_search_waypoint[1] - drone_y) ** 2
            ) ** 0.5
            if survey_lead_waypoint_timer_seconds > (
                patrol_waypoint_stall_timeout_seconds(stall_distance)
            ):
                if survey_lead_patrol_waypoints:
                    survey_lead_patrol_waypoint_index = (
                        survey_lead_patrol_waypoint_index + 1
                    ) % len(survey_lead_patrol_waypoints)
                automation_search_waypoint = None

        if automation_search_waypoint is None:
            automation_search_waypoint = choose_lead_survey_sector_waypoint()
            survey_lead_waypoint_timer_seconds = 0.0
            if automation_search_waypoint is None:
                return 0.0, 0.0, 0.0

        target_world_x, target_world_y, target_world_z = automation_search_waypoint
        waypoint_dx = target_world_x - drone_x
        waypoint_dy = target_world_y - drone_y
        waypoint_distance = (waypoint_dx * waypoint_dx + waypoint_dy * waypoint_dy) ** 0.5
        if waypoint_distance <= AUTOMATION_SEARCH_REACH_RADIUS_METERS:
            automation_search_waypoint = choose_lead_survey_sector_waypoint()
            survey_lead_waypoint_timer_seconds = 0.0
            if automation_search_waypoint is None:
                return 0.0, 0.0, 0.0
            target_world_x, target_world_y, target_world_z = automation_search_waypoint

    dx = target_world_x - drone_x
    dy = target_world_y - drone_y
    horizontal_distance = (dx * dx + dy * dy) ** 0.5

    if (
        using_hotspot_target
        and (not target_hotspot.detected)
        and horizontal_distance <= AUTOMATION_HOTSPOT_FLYBY_RADIUS_METERS
    ):
        mark_hotspot_detected(
            target_hotspot,
            motion_state.sim_time_seconds,
            detected_by_survey_slot=1,
        )
        target_hotspot.last_flyby_time_seconds = motion_state.sim_time_seconds
        next_hotspot = choose_automation_hotspot_target(
            drone_x,
            drone_y,
            excluded_hotspot=target_hotspot,
            sector=survey_sector,
        )
        if next_hotspot is not None:
            target_hotspot = next_hotspot
            automation_target_hotspot = target_hotspot
            target_world_x = target_hotspot.root.getX()
            target_world_y = target_hotspot.root.getY()
            target_world_z = target_hotspot.ground_z
            dx = target_world_x - drone_x
            dy = target_world_y - drone_y
            horizontal_distance = (dx * dx + dy * dy) ** 0.5
        else:
            using_hotspot_target = False
            automation_target_hotspot = None

    max_horizontal_speed = get_current_drone_speed_units_per_second() * AUTOMATION_SPEED_FACTOR
    arrival_radius = AUTOMATION_TARGET_REACH_RADIUS_METERS
    if using_hotspot_target and (not target_hotspot.detected):
        arrival_radius = AUTOMATION_HOTSPOT_COMMIT_RADIUS_METERS

    if horizontal_distance > arrival_radius and horizontal_distance > 0.001:
        approach_speed_scale = min(1.0, horizontal_distance / FIRE_HOTSPOT_DETECTION_RADIUS_METERS)
        if using_hotspot_target:
            if target_hotspot.detected:
                approach_speed_scale = max(0.45, approach_speed_scale)
            else:
                min_approach_scale = min(
                    1.0,
                    AUTOMATION_HOTSPOT_MIN_APPROACH_SPEED_METERS_PER_SECOND / max_horizontal_speed,
                )
                approach_speed_scale = max(min_approach_scale, approach_speed_scale)
        else:
            approach_speed_scale = max(0.7, approach_speed_scale)
        desired_vx = dx / horizontal_distance * max_horizontal_speed * approach_speed_scale
        desired_vy = dy / horizontal_distance * max_horizontal_speed * approach_speed_scale
    else:
        desired_vx = 0.0
        desired_vy = 0.0

    base_desired_vx = desired_vx
    base_desired_vy = desired_vy

    local_canopy_max_z = None
    if AUTOMATION_ENABLE_CANOPY_CLEARANCE:
        lookahead_x = drone_x + desired_vx * AUTOMATION_CANOPY_LOOKAHEAD_SECONDS
        lookahead_y = drone_y + desired_vy * AUTOMATION_CANOPY_LOOKAHEAD_SECONDS
        canopy_probe_points = build_canopy_probe_points(
            drone_x,
            drone_y,
            lookahead_x,
            lookahead_y,
        )
        local_canopy_max_z = compute_probe_path_canopy_height(canopy_probe_points)

    should_apply_tree_avoidance = True
    if AUTOMATION_ENABLE_CANOPY_CLEARANCE and local_canopy_max_z is not None:
        canopy_safe_altitude = local_canopy_max_z + AUTOMATION_CANOPY_CLEARANCE_METERS
        if drone.getZ() >= canopy_safe_altitude - 0.5:
            # Once we're safely above nearby canopy, avoid needless lateral weaving.
            should_apply_tree_avoidance = False

    if should_apply_tree_avoidance:
        avoid_vx, avoid_vy = compute_tree_avoidance_velocity(
            drone_x,
            drone_y,
            desired_vx,
            desired_vy,
            max_horizontal_speed,
        )
        desired_vx += avoid_vx
        desired_vy += avoid_vy

    drone_avoid_vx, drone_avoid_vy = compute_drone_avoidance_velocity(
        drone,
        drone_x,
        drone_y,
        base_desired_vx,
        base_desired_vy,
        max_horizontal_speed,
    )
    desired_vx += drone_avoid_vx
    desired_vy += drone_avoid_vy

    base_desired_speed = (base_desired_vx * base_desired_vx + base_desired_vy * base_desired_vy) ** 0.5
    if base_desired_speed > 0.001 and AUTOMATION_MIN_FORWARD_PROGRESS_FACTOR > 0.0:
        unit_target_x = base_desired_vx / base_desired_speed
        unit_target_y = base_desired_vy / base_desired_speed
        forward_speed = desired_vx * unit_target_x + desired_vy * unit_target_y
        min_forward_speed = max_horizontal_speed * AUTOMATION_MIN_FORWARD_PROGRESS_FACTOR
        if forward_speed < min_forward_speed:
            forward_deficit = min_forward_speed - forward_speed
            desired_vx += unit_target_x * forward_deficit
            desired_vy += unit_target_y * forward_deficit

    desired_speed = (desired_vx * desired_vx + desired_vy * desired_vy) ** 0.5
    if desired_speed > max_horizontal_speed and desired_speed > 0.001:
        desired_vx = (desired_vx / desired_speed) * max_horizontal_speed
        desired_vy = (desired_vy / desired_speed) * max_horizontal_speed

    desired_altitude = max(
        AUTOMATION_CRUISE_ALTITUDE_METERS,
        target_world_z + AUTOMATION_SCAN_ALTITUDE_METERS,
    )
    drone_ground = sample_ground(drone_x, drone_y)
    if drone_ground is not None:
        desired_altitude = max(
            desired_altitude,
            drone_ground[0].z + AUTOMATION_MIN_ALTITUDE_ABOVE_GROUND_METERS,
        )

    if AUTOMATION_ENABLE_CANOPY_CLEARANCE and local_canopy_max_z is not None:
        desired_altitude = max(
            desired_altitude,
            local_canopy_max_z + AUTOMATION_CANOPY_CLEARANCE_METERS,
        )

    altitude_error = desired_altitude - drone.getZ()
    max_vertical_speed = get_current_drone_speed_units_per_second() * AUTOMATION_VERTICAL_SPEED_FACTOR
    desired_vz = max(
        -max_vertical_speed,
        min(max_vertical_speed, altitude_error * AUTOMATION_ALTITUDE_GAIN),
    )

    return desired_vx, desired_vy, desired_vz


def update_kinematic_drone_motion(
    drone_root,
    drone_visual_root,
    drone_propeller_nodes,
    drone_state,
    target_x,
    target_y,
    target_z,
    cruise_speed,
    vertical_speed,
    dt,
):
    dx = target_x - drone_root.getX()
    dy = target_y - drone_root.getY()
    horizontal_distance = (dx * dx + dy * dy) ** 0.5

    if horizontal_distance > 0.001:
        step_distance = min(horizontal_distance, cruise_speed * dt)
        drone_root.setX(drone_root.getX() + (dx / horizontal_distance) * step_distance)
        drone_root.setY(drone_root.getY() + (dy / horizontal_distance) * step_distance)
    z_error = target_z - drone_root.getZ()
    if abs(z_error) > 0.001:
        z_step = min(abs(z_error), vertical_speed * dt)
        drone_root.setZ(drone_root.getZ() + copysign(z_step, z_error))

    drone_visual_root.setP(0.0)
    drone_visual_root.setR(0.0)
    drone_state.propeller_spin_degrees = (
        drone_state.propeller_spin_degrees
        + DRONE_PROPELLER_SPIN_BASE_DEGREES_PER_SECOND * dt
    ) % 360.0
    for rotor_node, spin_direction in drone_propeller_nodes:
        rotor_node.setR(drone_state.propeller_spin_degrees * spin_direction)


def compute_point_tracking_velocity(
    origin_x,
    origin_y,
    target_x,
    target_y,
    max_horizontal_speed,
    slow_radius_meters,
    min_speed_scale=0.2,
):
    dx = target_x - origin_x
    dy = target_y - origin_y
    horizontal_distance = (dx * dx + dy * dy) ** 0.5
    if horizontal_distance <= 0.001:
        return 0.0, 0.0, horizontal_distance

    speed_scale = clamp(
        horizontal_distance / max(0.001, slow_radius_meters),
        min_speed_scale,
        1.0,
    )
    desired_speed = max_horizontal_speed * speed_scale
    return (
        (dx / horizontal_distance) * desired_speed,
        (dy / horizontal_distance) * desired_speed,
        horizontal_distance,
    )


def update_manual_water_drone(dt, current_speed_units_per_second, input_active=True):
    team_state.water_target_hotspot = None
    water_motion_state.sim_time_seconds += dt
    if input_active and manual_movement_command_active():
        clear_dispatch_alert()

    manual_speed_units_per_second = (
        get_manual_drone_speed_units_per_second()
        if input_active
        else current_speed_units_per_second
    )
    manual_tilt_limit_radians = (
        get_manual_max_tilt_radians()
        if input_active
        else get_current_max_tilt_radians()
    )
    if input_active:
        desired_vx, desired_vy, desired_vz = compute_manual_desired_velocity(
            camera_angle,
            key_map,
            manual_speed_units_per_second,
        )
    else:
        desired_vx, desired_vy, desired_vz = 0.0, 0.0, 0.0
    target_heading_degrees = None
    if input_active:
        horizontal_intent = (desired_vx * desired_vx + desired_vy * desired_vy) ** 0.5
        if horizontal_intent > DRONE_MANUAL_YAW_INPUT_SPEED_METERS_PER_SECOND:
            target_heading_degrees = camera_angle
    manual_vertical_command_active = input_active and abs(desired_vz) > 0.001
    if water_motion_state.target_altitude_meters is None:
        water_motion_state.target_altitude_meters = water_drone.getZ()
    elif (
        water_motion_state.vertical_command_was_active
        and not manual_vertical_command_active
    ):
        water_motion_state.target_altitude_meters = water_drone.getZ()

    water_motion_state.target_altitude_meters += desired_vz * dt
    water_motion_state.vertical_command_was_active = manual_vertical_command_active
    water_motion_state.target_altitude_meters = compute_safe_altitude_at(
        water_drone.getX(),
        water_drone.getY(),
        water_motion_state.target_altitude_meters,
        WATER_DRONE_MIN_ALTITUDE_ABOVE_GROUND_METERS,
        canopy_clearance_meters=WATER_DRONE_CANOPY_CLEARANCE_METERS,
    )

    manual_horizontal_speed = min(
        WATER_DRONE_CRUISE_SPEED_METERS_PER_SECOND,
        manual_speed_units_per_second,
    )
    manual_vertical_speed = min(
        WATER_DRONE_VERTICAL_SPEED_METERS_PER_SECOND,
        manual_speed_units_per_second,
    )
    update_guided_drone_motion(
        water_drone,
        water_drone_visual,
        water_drone_propellers,
        water_motion_state,
        desired_vx,
        desired_vy,
        water_motion_state.target_altitude_meters,
        manual_horizontal_speed,
        manual_vertical_speed,
        dt,
        wind_command_blend=DRONE_WIND_COMMAND_BLEND_MANUAL,
        apply_drone_avoidance=input_active,
        target_heading_degrees=target_heading_degrees,
        max_tilt_radians=manual_tilt_limit_radians,
    )

    closest_hotspot = choose_nearest_sprayable_hotspot(
        water_drone.getX(),
        water_drone.getY(),
    )
    lead_water_engaged = False
    if is_water_spray_active(1) and not water_tank_has_spray(water_drone_state):
        manual_water_spray_slots.discard(1)
        clear_water_suppression_visual(water_drone_state)
    # Manual water 1 only suppresses while its spray is toggled ON ([J]).
    if (
        closest_hotspot is not None
        and is_water_spray_active(1)
        and water_tank_has_spray(water_drone_state)
    ):
        team_state.water_target_hotspot = closest_hotspot
        closest_ground_target_altitude = compute_safe_altitude_at(
            closest_hotspot.root.getX(),
            closest_hotspot.root.getY(),
            closest_hotspot.ground_z + WATER_DRONE_SUPPRESSION_ALTITUDE_METERS,
            WATER_DRONE_MIN_ALTITUDE_ABOVE_GROUND_METERS,
            canopy_clearance_meters=WATER_DRONE_CANOPY_CLEARANCE_METERS,
        )
        dx = water_drone.getX() - closest_hotspot.root.getX()
        dy = water_drone.getY() - closest_hotspot.root.getY()
        horizontal_distance = (dx * dx + dy * dy) ** 0.5
        altitude_error = abs(water_drone.getZ() - closest_ground_target_altitude)
        # Manual spray uses a more forgiving reach so [J] reliably hits a fire
        # you fly near, without having to line up perfectly.
        in_suppression_range = (
            horizontal_distance <= WATER_MANUAL_SPRAY_RADIUS_METERS
            and altitude_error <= WATER_MANUAL_SPRAY_ALTITUDE_TOLERANCE_METERS
        )
        lead_water_engaged = engage_water_suppression_if_possible(
            water_drone_state,
            in_suppression_range,
            dt,
        )
        register_hotspot_water_drone_contact(closest_hotspot, lead_water_engaged)

    update_water_suppression_visual(
        team_state.water_target_hotspot,
        engaged=lead_water_engaged,
    )


def remove_follower_drones():
    global follower_drones
    for follower in follower_drones:
        stream_root = follower.get("stream_root")
        if stream_root is not None and not stream_root.isEmpty():
            stream_root.removeNode()
        rig_root = follower.get("root")
        if rig_root is not None and not rig_root.isEmpty():
            rig_root.removeNode()
    follower_drones = []


def deploy_survey_lead_to_sector_start():
    global automation_target_hotspot, automation_search_waypoint
    global survey_lead_patrol_sector_label, survey_lead_patrol_waypoints
    global survey_lead_patrol_waypoint_index

    if get_team_survey_drone_count() <= 1:
        return
    # Fixed birth spots: the survey lead always deploys to its fixed spawn.
    spawn_x, spawn_y = get_fixed_drone_spawn_xy("survey", 1)
    spawn_z = compute_safe_altitude_at(
        spawn_x,
        spawn_y,
        AUTOMATION_CRUISE_ALTITUDE_METERS,
        AUTOMATION_MIN_ALTITUDE_ABOVE_GROUND_METERS,
        canopy_clearance_meters=(
            AUTOMATION_CANOPY_CLEARANCE_METERS
            if AUTOMATION_ENABLE_CANOPY_CLEARANCE
            else 0.0
        ),
    )
    drone.setPos(spawn_x, spawn_y, spawn_z)
    drone.setH(0.0)
    drone_visual.setP(0.0)
    drone_visual.setR(0.0)
    automation_target_hotspot = None
    automation_search_waypoint = None
    survey_lead_patrol_sector_label = None
    survey_lead_patrol_waypoints = tuple()
    survey_lead_patrol_waypoint_index = 0
    initialize_motion_state_from_drone_pose()


def get_follower_survey_slot(follower):
    return max(1, follower.get("slot", 1) + 1)


def refresh_follower_survey_patrol(follower):
    sector = get_survey_search_sector(get_follower_survey_slot(follower))
    if sector is None:
        return None
    if (
        follower.get("patrol_sector_label") != sector.label
        or not follower.get("patrol_waypoints")
    ):
        follower["patrol_sector_label"] = sector.label
        follower["patrol_waypoints"] = rotate_waypoints_to_nearest(
            build_survey_sector_waypoints(sector),
            follower["root"].getX(),
            follower["root"].getY(),
        )
        follower["patrol_waypoint_index"] = 0
    return sector


def assign_follower_waypoint(follower):
    edge_margin = FOLLOWER_PATROL_EDGE_MARGIN_METERS
    if follower.get("role") == "survey":
        sector = refresh_follower_survey_patrol(follower)
        waypoints = follower.get("patrol_waypoints") or tuple()
        if waypoints:
            waypoint = waypoints[
                follower.get("patrol_waypoint_index", 0) % len(waypoints)
            ]
            safe_waypoint = patrol_waypoint_with_safe_altitude(waypoint)
            follower["wx"], follower["wy"], follower["wz"] = safe_waypoint
            return
        if sector is not None:
            x_min = sector.x_min + edge_margin
            x_max = sector.x_max - edge_margin
            if x_max <= x_min:
                x_min, _ = sector.center()
                x_max = x_min
            follower["wx"] = random.uniform(x_min, x_max)
        else:
            follower["wx"] = random.uniform(SPAWN_X_MIN + edge_margin, SPAWN_X_MAX - edge_margin)
    else:
        follower["wx"] = random.uniform(SPAWN_X_MIN + edge_margin, SPAWN_X_MAX - edge_margin)
    follower["wy"] = random.uniform(SPAWN_Y_MIN + edge_margin, SPAWN_Y_MAX - edge_margin)
    follower["wz"] = compute_safe_altitude_at(
        follower["wx"],
        follower["wy"],
        AUTOMATION_CRUISE_ALTITUDE_METERS + random.uniform(-2.0, 5.0),
        AUTOMATION_MIN_ALTITUDE_ABOVE_GROUND_METERS,
        canopy_clearance_meters=(
            AUTOMATION_CANOPY_CLEARANCE_METERS
            if AUTOMATION_ENABLE_CANOPY_CLEARANCE
            else 0.0
        ),
    )


def spawn_follower_drone(role, lead_node, slot):
    tint = SURVEY_DRONE_MODEL_TINT if role == "survey" else WATER_DRONE_MODEL_TINT
    rig_root, rig_visual, rig_model, rig_props = build_drone_rig(
        f"follower_{role}_{slot}",
        tint,
    )
    if role == "survey":
        survey_slot = slot + 1
        # Fixed birth spots: each survey drone spawns at the same relative
        # point of its own map part (paired side by side with its water drone).
        spawn_x, spawn_y = get_fixed_drone_spawn_xy("survey", survey_slot)
        spawn_z = compute_safe_altitude_at(
            spawn_x,
            spawn_y,
            AUTOMATION_CRUISE_ALTITUDE_METERS,
            AUTOMATION_MIN_ALTITUDE_ABOVE_GROUND_METERS,
            canopy_clearance_meters=(
                AUTOMATION_CANOPY_CLEARANCE_METERS
                if AUTOMATION_ENABLE_CANOPY_CLEARANCE
                else 0.0
            ),
        )
        rig_root.setPos(spawn_x, spawn_y, spawn_z)
    else:
        water_slot = slot + 1
        water_home_x, water_home_y = get_water_drone_home_xy(water_slot)
        water_home_z = compute_safe_altitude_at(
            water_home_x,
            water_home_y,
            WATER_DRONE_HOME_ALTITUDE_METERS,
            WATER_DRONE_MIN_ALTITUDE_ABOVE_GROUND_METERS,
            canopy_clearance_meters=WATER_DRONE_CANOPY_CLEARANCE_METERS,
        )
        rig_root.setPos(
            water_home_x,
            water_home_y,
            water_home_z,
        )
    motion = DroneMotionState()
    apply_drone_physical_parameters(motion, role)
    initialize_drone_motion_state_from_pose(rig_root, motion)
    home_z = compute_safe_altitude_at(
        rig_root.getX(),
        rig_root.getY(),
        WATER_DRONE_HOME_ALTITUDE_METERS if role == "water" else AUTOMATION_CRUISE_ALTITUDE_METERS,
        WATER_DRONE_MIN_ALTITUDE_ABOVE_GROUND_METERS if role == "water" else AUTOMATION_MIN_ALTITUDE_ABOVE_GROUND_METERS,
        canopy_clearance_meters=(
            WATER_DRONE_CANOPY_CLEARANCE_METERS
            if role == "water"
            else (
                AUTOMATION_CANOPY_CLEARANCE_METERS
                if AUTOMATION_ENABLE_CANOPY_CLEARANCE
                else 0.0
            )
        ),
    )
    stream_root = None
    mist_nodes = tuple()
    if role == "water":
        stream_root, mist_nodes = build_water_suppression_visual()

    follower = {
        "root": rig_root,
        "visual": rig_visual,
        "model": rig_model,
        "propellers": rig_props,
        "role": role,
        "slot": slot,
        "motion_state": motion,
        # Every follower starts in MANUAL at its birth point. Selecting that
        # drone and pressing [O] starts its automation; pressing a water
        # dispatch key ([L]/[K]) also switches that specific water drone to
        # automation and releases it.
        "control_mode": CONTROL_MODE_MANUAL,
        "automation_started": False,
        "target_hotspot": None,
        "search_waypoint": None,
        "patrol_sector_label": None,
        "patrol_waypoints": tuple(),
        "patrol_waypoint_index": 0,
        "orbit_hotspot_id": None,
        "orbit_phase_radians": random.uniform(0.0, tau),
        "tracked_hotspot_id": None,
        "hover_offset_angle_radians": random.uniform(0.0, tau),
        "home_x": rig_root.getX(),
        "home_y": rig_root.getY(),
        "home_z": home_z,
        "water_liters": (
            WATER_DRONE_TANK_CAPACITY_LITERS if role == "water" else 0.0
        ),
        "stream_root": stream_root,
        "stream_outer_node": None,
        "stream_inner_node": None,
        "mist_nodes": mist_nodes,
    }
    assign_follower_waypoint(follower)
    follower_drones.append(follower)
    return follower


def respawn_follower_drones():
    """Spawn (total - 2) extra drones as flying followers of the two lead drones.

    The lead survey drone and lead water drone already cover one of each role, so
    the followers make up the remainder of the operator-chosen team size/ratio.
    """
    global camera_target_index, camera_target_node_override
    remove_follower_drones()
    extra_survey = max(0, get_team_survey_drone_count() - 1)
    extra_water = max(0, team_water_drone_count - 1)
    for slot in range(1, extra_survey + 1):
        spawn_follower_drone("survey", drone, slot)
    for slot in range(1, extra_water + 1):
        spawn_follower_drone("water", water_drone, slot)
    # Reset camera to the survey lead so a stale follower view can't dangle.
    camera_target_index = 0
    camera_target_node_override = None


def choose_follower_mapping_hotspot(follower):
    rig = follower["root"]
    return choose_survey_mapping_hotspot_for_slot(
        get_follower_survey_slot(follower),
        rig.getX(),
        rig.getY(),
    )


def choose_follower_undetected_hotspot_target(follower):
    rig = follower["root"]
    follower_survey_slot = get_follower_survey_slot(follower)
    sector = get_survey_search_sector(follower_survey_slot)

    def _eligible(hotspot, respect_cooldown):
        if hotspot.suppression_state in (FIRE_STATE_OUT, FIRE_STATE_BURNED) or hotspot.detected:
            return False
        if not survey_slot_may_target_hotspot(follower_survey_slot, hotspot):
            return False
        if (
            respect_cooldown
            and (motion_state.sim_time_seconds - hotspot.last_flyby_time_seconds)
            < AUTOMATION_HOTSPOT_REVISIT_DELAY_SECONDS
        ):
            return False
        return True

    for respect_cooldown in (True, False):
        candidates = [
            hotspot
            for hotspot in fire_hotspots
            if _eligible(hotspot, respect_cooldown)
            and (
                sector is None
                or sector.contains(hotspot.root.getX(), hotspot.root.getY())
            )
        ]
        if not candidates:
            continue
        candidates.sort(
            key=lambda hotspot: (
                (hotspot.root.getX() - rig.getX()) ** 2
                + (hotspot.root.getY() - rig.getY()) ** 2
            )
        )
        return candidates[0]
    return None


def compute_follower_navigation_velocity(
    follower,
    target_world_x,
    target_world_y,
    target_world_z,
    arrival_radius,
    using_hotspot_target=False,
):
    rig = follower["root"]
    drone_x = rig.getX()
    drone_y = rig.getY()
    dx = target_world_x - drone_x
    dy = target_world_y - drone_y
    horizontal_distance = (dx * dx + dy * dy) ** 0.5
    max_horizontal_speed = get_current_drone_speed_units_per_second() * AUTOMATION_SPEED_FACTOR

    if horizontal_distance > arrival_radius and horizontal_distance > 0.001:
        approach_speed_scale = min(1.0, horizontal_distance / FIRE_HOTSPOT_DETECTION_RADIUS_METERS)
        if using_hotspot_target:
            min_approach_scale = min(
                1.0,
                AUTOMATION_HOTSPOT_MIN_APPROACH_SPEED_METERS_PER_SECOND / max_horizontal_speed,
            )
            approach_speed_scale = max(min_approach_scale, approach_speed_scale)
        else:
            approach_speed_scale = max(0.7, approach_speed_scale)
        desired_vx = dx / horizontal_distance * max_horizontal_speed * approach_speed_scale
        desired_vy = dy / horizontal_distance * max_horizontal_speed * approach_speed_scale
    else:
        desired_vx = 0.0
        desired_vy = 0.0

    base_desired_vx = desired_vx
    base_desired_vy = desired_vy

    local_canopy_max_z = None
    if AUTOMATION_ENABLE_CANOPY_CLEARANCE:
        lookahead_x = drone_x + desired_vx * AUTOMATION_CANOPY_LOOKAHEAD_SECONDS
        lookahead_y = drone_y + desired_vy * AUTOMATION_CANOPY_LOOKAHEAD_SECONDS
        canopy_probe_points = build_canopy_probe_points(
            drone_x,
            drone_y,
            lookahead_x,
            lookahead_y,
        )
        local_canopy_max_z = compute_probe_path_canopy_height(canopy_probe_points)

    should_apply_tree_avoidance = True
    if AUTOMATION_ENABLE_CANOPY_CLEARANCE and local_canopy_max_z is not None:
        canopy_safe_altitude = local_canopy_max_z + AUTOMATION_CANOPY_CLEARANCE_METERS
        if rig.getZ() >= canopy_safe_altitude - 0.5:
            should_apply_tree_avoidance = False

    if should_apply_tree_avoidance:
        avoid_vx, avoid_vy = compute_tree_avoidance_velocity(
            drone_x,
            drone_y,
            desired_vx,
            desired_vy,
            max_horizontal_speed,
        )
        desired_vx += avoid_vx
        desired_vy += avoid_vy

    desired_speed = (desired_vx * desired_vx + desired_vy * desired_vy) ** 0.5
    if desired_speed > max_horizontal_speed and desired_speed > 0.001:
        desired_vx = (desired_vx / desired_speed) * max_horizontal_speed
        desired_vy = (desired_vy / desired_speed) * max_horizontal_speed

    desired_altitude = max(
        AUTOMATION_CRUISE_ALTITUDE_METERS,
        target_world_z + AUTOMATION_SCAN_ALTITUDE_METERS,
    )
    drone_ground = sample_ground(drone_x, drone_y)
    if drone_ground is not None:
        desired_altitude = max(
            desired_altitude,
            drone_ground[0].z + AUTOMATION_MIN_ALTITUDE_ABOVE_GROUND_METERS,
        )
    if AUTOMATION_ENABLE_CANOPY_CLEARANCE and local_canopy_max_z is not None:
        desired_altitude = max(
            desired_altitude,
            local_canopy_max_z + AUTOMATION_CANOPY_CLEARANCE_METERS,
        )

    return desired_vx, desired_vy, desired_altitude


def compute_follower_survey_orbit_velocity(follower, hotspot, dt):
    rig = follower["root"]
    hotspot_id = id(hotspot)
    if follower.get("orbit_hotspot_id") != hotspot_id:
        follower["orbit_hotspot_id"] = hotspot_id
        slot_offset = radians(
            FOLLOWER_SURVEY_ORBIT_SLOT_SPACING_DEGREES * follower.get("slot", 1)
        )
        follower["orbit_phase_radians"] = (
            atan2(
                rig.getY() - hotspot.root.getY(),
                rig.getX() - hotspot.root.getX(),
            )
            + slot_offset
        ) % tau

    orbit_angular_speed = (
        SURVEY_ORBIT_SPEED_METERS_PER_SECOND
        / max(1.0, SURVEY_ORBIT_RADIUS_METERS)
    )
    follower["orbit_phase_radians"] = (
        follower["orbit_phase_radians"] + orbit_angular_speed * dt
    ) % tau

    orbit_target_x = hotspot.root.getX() + (
        cos(follower["orbit_phase_radians"]) * SURVEY_ORBIT_RADIUS_METERS
    )
    orbit_target_y = hotspot.root.getY() + (
        sin(follower["orbit_phase_radians"]) * SURVEY_ORBIT_RADIUS_METERS
    )
    desired_vx, desired_vy, desired_altitude = compute_follower_navigation_velocity(
        follower,
        orbit_target_x,
        orbit_target_y,
        hotspot.ground_z,
        AUTOMATION_TARGET_REACH_RADIUS_METERS,
        using_hotspot_target=True,
    )
    desired_altitude = max(
        desired_altitude,
        hotspot.ground_z + SURVEY_ORBIT_ALTITUDE_OFFSET_METERS,
    )
    return desired_vx, desired_vy, desired_altitude


def update_survey_follower_drone(follower, dt):
    rig = follower["root"]
    if not follower.get("automation_started", True):
        # Standby: hover at the birth point until the operator selects this
        # drone and presses [O] to start its automation.
        follower["motion_state"].sim_time_seconds = motion_state.sim_time_seconds
        update_guided_drone_motion(
            rig,
            follower["visual"],
            follower["propellers"],
            follower["motion_state"],
            0.0,
            0.0,
            follower["home_z"],
            get_current_drone_speed_units_per_second() * AUTOMATION_SPEED_FACTOR,
            get_current_drone_speed_units_per_second() * AUTOMATION_VERTICAL_SPEED_FACTOR,
            dt,
        )
        return
    target_hotspot = choose_follower_mapping_hotspot(follower)
    if target_hotspot is not None:
        follower["target_hotspot"] = target_hotspot
        follower["search_waypoint"] = None
        follower["patrol_waypoint_timer_seconds"] = 0.0
        desired_vx, desired_vy, desired_altitude = compute_follower_survey_orbit_velocity(
            follower,
            target_hotspot,
            dt,
        )
    else:
        follower["orbit_hotspot_id"] = None
        # Without prior fire knowledge the follower sticks to its S-shaped
        # sweep; fires are only found by the sensor along the pattern.
        target_hotspot = (
            choose_follower_undetected_hotspot_target(follower)
            if AUTOMATION_KNOWS_UNDETECTED_FIRES
            else None
        )
        if target_hotspot is not None:
            follower["target_hotspot"] = target_hotspot
            follower["search_waypoint"] = None
            target_world_x = target_hotspot.root.getX()
            target_world_y = target_hotspot.root.getY()
            target_world_z = target_hotspot.ground_z
            dx = target_world_x - rig.getX()
            dy = target_world_y - rig.getY()
            horizontal_distance = (dx * dx + dy * dy) ** 0.5
            if horizontal_distance <= AUTOMATION_HOTSPOT_FLYBY_RADIUS_METERS:
                mark_hotspot_detected(
                    target_hotspot,
                    motion_state.sim_time_seconds,
                    detected_by_survey_slot=get_follower_survey_slot(follower),
                )
                target_hotspot.last_flyby_time_seconds = motion_state.sim_time_seconds
            desired_vx, desired_vy, desired_altitude = compute_follower_navigation_velocity(
                follower,
                target_world_x,
                target_world_y,
                target_world_z,
                AUTOMATION_HOTSPOT_COMMIT_RADIUS_METERS,
                using_hotspot_target=True,
            )
        else:
            follower["target_hotspot"] = None
            refresh_follower_survey_patrol(follower)
            waypoints = follower.get("patrol_waypoints") or tuple()
            if waypoints:
                waypoint_index = follower.get("patrol_waypoint_index", 0) % len(waypoints)
                waypoint = patrol_waypoint_with_safe_altitude(waypoints[waypoint_index])
                dx = waypoint[0] - rig.getX()
                dy = waypoint[1] - rig.getY()
                # Anti-stall: tree avoidance near a waypoint (e.g. the corner
                # birth point) can hold the drone in equilibrium just outside
                # the reach radius. Only skip when the leg takes far longer
                # than its distance justifies, so long sweep legs are safe.
                waypoint_distance = (dx * dx + dy * dy) ** 0.5
                follower["patrol_waypoint_timer_seconds"] = (
                    follower.get("patrol_waypoint_timer_seconds", 0.0) + dt
                )
                if (
                    waypoint_distance <= AUTOMATION_SEARCH_REACH_RADIUS_METERS
                    or is_search_waypoint_exhausted(waypoint)
                    or follower["patrol_waypoint_timer_seconds"]
                    > patrol_waypoint_stall_timeout_seconds(waypoint_distance)
                ):
                    follower["patrol_waypoint_index"] = (waypoint_index + 1) % len(waypoints)
                    follower["patrol_waypoint_timer_seconds"] = 0.0
                    waypoint = patrol_waypoint_with_safe_altitude(
                        waypoints[follower["patrol_waypoint_index"]]
                    )
                follower["search_waypoint"] = waypoint
            else:
                assign_follower_waypoint(follower)
                waypoint = (follower["wx"], follower["wy"], follower["wz"])
                follower["search_waypoint"] = waypoint
            desired_vx, desired_vy, desired_altitude = compute_follower_navigation_velocity(
                follower,
                waypoint[0],
                waypoint[1],
                waypoint[2],
                AUTOMATION_SEARCH_REACH_RADIUS_METERS,
                using_hotspot_target=False,
            )

    update_guided_drone_motion(
        rig,
        follower["visual"],
        follower["propellers"],
        follower["motion_state"],
        desired_vx,
        desired_vy,
        desired_altitude,
        get_current_drone_speed_units_per_second() * AUTOMATION_SPEED_FACTOR,
        get_current_drone_speed_units_per_second() * AUTOMATION_VERTICAL_SPEED_FACTOR,
        dt,
    )


def choose_water_support_hotspot_for_follower(follower):
    # Water followers stay stationary at their spawn until the operator calls
    # each one with its own key: [P] water 1 (lead), [L] water 2, [K] water 3.
    water_slot = follower.get("slot", 1) + 1
    if follower.get("role") == "water" and water_slot not in dispatched_water_slots:
        return None
    rig = follower["root"]
    return choose_water_support_hotspot_for_slot(
        water_slot,
        rig.getX(),
        rig.getY(),
    )


def compute_water_support_hover_angle(target_hotspot, water_slot, current_x, current_y):
    wind_vx, wind_vy, _, wind_speed_mps = get_wind_velocity()
    if wind_speed_mps > 0.25 and (abs(wind_vx) > 0.01 or abs(wind_vy) > 0.01):
        downwind_angle = atan2(wind_vy, wind_vx)
        if water_slot == 1:
            return (downwind_angle + (tau * 0.5)) % tau
        if water_slot == 2:
            return downwind_angle % tau
        helper_side = -1.0 if water_slot % 2 else 1.0
        helper_offset = radians(95.0 + (14.0 * max(0, water_slot - 3)))
        return (downwind_angle + (helper_side * helper_offset)) % tau

    base_angle = atan2(
        current_y - target_hotspot.root.getY(),
        current_x - target_hotspot.root.getX(),
    )
    return (
        base_angle
        + radians(FOLLOWER_WATER_HOVER_SLOT_SPACING_DEGREES * max(0, water_slot - 1))
    ) % tau


def water_suppression_heading_degrees(source_x, source_y, target_hotspot):
    return heading_from_world_vector(
        target_hotspot.root.getX() - source_x,
        target_hotspot.root.getY() - source_y,
    )


def update_water_follower_drone(follower, dt):
    rig = follower["root"]
    water_slot = follower.get("slot", 1) + 1
    if WATER_FOLLOWS_PAIRED_SURVEY:
        target_hotspot = choose_paired_survey_water_hotspot(
            water_slot, rig.getX(), rig.getY()
        )
    else:
        target_hotspot = choose_water_support_hotspot_for_follower(follower)
    follower["target_hotspot"] = target_hotspot
    follower["motion_state"].sim_time_seconds = motion_state.sim_time_seconds
    max_horizontal_speed = WATER_DRONE_CRUISE_SPEED_METERS_PER_SECOND
    max_vertical_speed = WATER_DRONE_VERTICAL_SPEED_METERS_PER_SECOND

    if target_hotspot is None:
        follower["tracked_hotspot_id"] = None
        clear_water_suppression_visual(follower)
        # #6: trail the paired survey drone instead of parking at spawn.
        follow_anchor = (
            get_water_follow_anchor(water_slot) if WATER_FOLLOWS_PAIRED_SURVEY else None
        )
        if follow_anchor is not None:
            loiter_x, loiter_y = follow_anchor
        else:
            loiter_x, loiter_y = follower["home_x"], follower["home_y"]
        follower["home_z"] = compute_safe_altitude_at(
            loiter_x,
            loiter_y,
            max(WATER_DRONE_HOME_ALTITUDE_METERS, follower["home_z"]),
            WATER_DRONE_MIN_ALTITUDE_ABOVE_GROUND_METERS,
            canopy_clearance_meters=WATER_DRONE_CANOPY_CLEARANCE_METERS,
        )
        desired_vx, desired_vy, _ = compute_point_tracking_velocity(
            rig.getX(),
            rig.getY(),
            loiter_x,
            loiter_y,
            max_horizontal_speed * 0.72,
            8.0,
            min_speed_scale=0.0,
        )
        update_guided_drone_motion(
            rig,
            follower["visual"],
            follower["propellers"],
            follower["motion_state"],
            desired_vx,
            desired_vy,
            follower["home_z"],
            max_horizontal_speed * 0.72,
            max_vertical_speed,
            dt,
            visual_tilt_scale=WATER_DRONE_AUTO_VISUAL_TILT_SCALE,
            visual_tilt_time_constant_seconds=WATER_DRONE_AUTO_VISUAL_TILT_TIME_CONSTANT_SECONDS,
        )
        return

    hotspot_id = id(target_hotspot)
    if follower.get("tracked_hotspot_id") != hotspot_id:
        follower["tracked_hotspot_id"] = hotspot_id
        follower["hover_offset_angle_radians"] = compute_water_support_hover_angle(
            target_hotspot,
            follower.get("slot", 1) + 1,
            rig.getX(),
            rig.getY(),
        )

    desired_x = target_hotspot.root.getX() + (
        cos(follower["hover_offset_angle_radians"]) * WATER_DRONE_HOVER_HOLD_RADIUS_METERS
    )
    desired_y = target_hotspot.root.getY() + (
        sin(follower["hover_offset_angle_radians"]) * WATER_DRONE_HOVER_HOLD_RADIUS_METERS
    )
    desired_z = max(
        compute_safe_altitude_at(
            rig.getX(),
            rig.getY(),
            WATER_DRONE_HOME_ALTITUDE_METERS,
            WATER_DRONE_MIN_ALTITUDE_ABOVE_GROUND_METERS,
            canopy_clearance_meters=WATER_DRONE_CANOPY_CLEARANCE_METERS,
        ),
        compute_safe_altitude_at(
            desired_x,
            desired_y,
            target_hotspot.ground_z + WATER_DRONE_SUPPRESSION_ALTITUDE_METERS,
            WATER_DRONE_MIN_ALTITUDE_ABOVE_GROUND_METERS,
            canopy_clearance_meters=WATER_DRONE_CANOPY_CLEARANCE_METERS,
        ),
        compute_safe_altitude_at(
            target_hotspot.root.getX(),
            target_hotspot.root.getY(),
            target_hotspot.ground_z + WATER_DRONE_SUPPRESSION_ALTITUDE_METERS,
            WATER_DRONE_MIN_ALTITUDE_ABOVE_GROUND_METERS,
            canopy_clearance_meters=WATER_DRONE_CANOPY_CLEARANCE_METERS,
        ),
    )
    desired_vx, desired_vy, _ = compute_point_tracking_velocity(
        rig.getX(),
        rig.getY(),
        desired_x,
        desired_y,
        max_horizontal_speed * 0.82,
        WATER_DRONE_HOVER_HOLD_RADIUS_METERS * 2.1,
        min_speed_scale=0.0,
    )
    target_heading_degrees = water_suppression_heading_degrees(
        rig.getX(),
        rig.getY(),
        target_hotspot,
    )
    update_guided_drone_motion(
        rig,
        follower["visual"],
        follower["propellers"],
        follower["motion_state"],
        desired_vx,
        desired_vy,
        desired_z,
        max_horizontal_speed * 0.82,
        max_vertical_speed * 0.92,
        dt,
        target_heading_degrees=target_heading_degrees,
        visual_tilt_scale=WATER_DRONE_AUTO_VISUAL_TILT_SCALE,
        visual_tilt_time_constant_seconds=WATER_DRONE_AUTO_VISUAL_TILT_TIME_CONSTANT_SECONDS,
    )

    dx = rig.getX() - target_hotspot.root.getX()
    dy = rig.getY() - target_hotspot.root.getY()
    horizontal_distance = (dx * dx + dy * dy) ** 0.5
    altitude_error = abs(rig.getZ() - desired_z)
    in_suppression_range = (
        horizontal_distance <= WATER_DRONE_SUPPRESSION_RADIUS_METERS
        and altitude_error <= 3.2
    )
    is_engaged = engage_water_suppression_if_possible(
        follower,
        in_suppression_range,
        dt,
    )
    register_hotspot_water_drone_contact(
        target_hotspot,
        is_engaged,
    )
    update_water_suppression_visual(
        target_hotspot,
        source_root=rig,
        stream_state=follower,
        engaged=is_engaged,
    )


def update_manual_follower_drone(follower, dt, input_active):
    rig = follower["root"]
    drone_motion_state = follower["motion_state"]
    drone_motion_state.sim_time_seconds = motion_state.sim_time_seconds
    current_speed_units_per_second = get_current_drone_speed_units_per_second()
    if (
        follower.get("role") == "water"
        and input_active
        and manual_movement_command_active()
    ):
        clear_dispatch_alert()

    manual_speed_units_per_second = (
        get_manual_drone_speed_units_per_second()
        if input_active
        else current_speed_units_per_second
    )
    manual_tilt_limit_radians = (
        get_manual_max_tilt_radians()
        if input_active
        else get_current_max_tilt_radians()
    )
    if input_active:
        desired_vx, desired_vy, desired_vz = compute_manual_desired_velocity(
            camera_angle,
            key_map,
            manual_speed_units_per_second,
        )
    else:
        desired_vx, desired_vy, desired_vz = 0.0, 0.0, 0.0
    target_heading_degrees = None
    if input_active:
        horizontal_intent = (desired_vx * desired_vx + desired_vy * desired_vy) ** 0.5
        if horizontal_intent > DRONE_MANUAL_YAW_INPUT_SPEED_METERS_PER_SECOND:
            target_heading_degrees = camera_angle

    manual_vertical_command_active = input_active and abs(desired_vz) > 0.001
    if drone_motion_state.target_altitude_meters is None:
        drone_motion_state.target_altitude_meters = rig.getZ()
    elif (
        drone_motion_state.vertical_command_was_active
        and not manual_vertical_command_active
    ):
        drone_motion_state.target_altitude_meters = rig.getZ()

    drone_motion_state.target_altitude_meters += desired_vz * dt
    drone_motion_state.vertical_command_was_active = manual_vertical_command_active

    if follower.get("role") == "water":
        min_altitude = WATER_DRONE_MIN_ALTITUDE_ABOVE_GROUND_METERS
        canopy_clearance = WATER_DRONE_CANOPY_CLEARANCE_METERS
        horizontal_speed_limit = min(
            WATER_DRONE_CRUISE_SPEED_METERS_PER_SECOND,
            manual_speed_units_per_second,
        )
        vertical_speed_limit = min(
            WATER_DRONE_VERTICAL_SPEED_METERS_PER_SECOND,
            manual_speed_units_per_second,
        )
    else:
        min_altitude = AUTOMATION_MIN_ALTITUDE_ABOVE_GROUND_METERS
        canopy_clearance = (
            AUTOMATION_CANOPY_CLEARANCE_METERS
            if AUTOMATION_ENABLE_CANOPY_CLEARANCE
            else 0.0
        )
        horizontal_speed_limit = manual_speed_units_per_second
        vertical_speed_limit = manual_speed_units_per_second

    drone_motion_state.target_altitude_meters = compute_safe_altitude_at(
        rig.getX(),
        rig.getY(),
        drone_motion_state.target_altitude_meters,
        min_altitude,
        canopy_clearance_meters=canopy_clearance,
    )

    update_guided_drone_motion(
        rig,
        follower["visual"],
        follower["propellers"],
        drone_motion_state,
        desired_vx,
        desired_vy,
        drone_motion_state.target_altitude_meters,
        horizontal_speed_limit,
        vertical_speed_limit,
        dt,
        wind_command_blend=DRONE_WIND_COMMAND_BLEND_MANUAL,
        apply_drone_avoidance=input_active,
        target_heading_degrees=target_heading_degrees,
        max_tilt_radians=manual_tilt_limit_radians,
    )

    if follower.get("role") != "water":
        return

    closest_hotspot = choose_nearest_sprayable_hotspot(
        rig.getX(),
        rig.getY(),
    )
    follower["target_hotspot"] = closest_hotspot
    # Water follower only suppresses while its spray is toggled ON ([J]).
    spray_view_slot = follower.get("slot", 0) + 1
    if is_water_spray_active(spray_view_slot) and not water_tank_has_spray(follower):
        manual_water_spray_slots.discard(spray_view_slot)
        clear_water_suppression_visual(follower)
    if closest_hotspot is None or not is_water_spray_active(spray_view_slot):
        clear_water_suppression_visual(follower)
        return

    closest_ground_target_altitude = compute_safe_altitude_at(
        closest_hotspot.root.getX(),
        closest_hotspot.root.getY(),
        closest_hotspot.ground_z + WATER_DRONE_SUPPRESSION_ALTITUDE_METERS,
        WATER_DRONE_MIN_ALTITUDE_ABOVE_GROUND_METERS,
        canopy_clearance_meters=WATER_DRONE_CANOPY_CLEARANCE_METERS,
    )
    dx = rig.getX() - closest_hotspot.root.getX()
    dy = rig.getY() - closest_hotspot.root.getY()
    horizontal_distance = (dx * dx + dy * dy) ** 0.5
    altitude_error = abs(rig.getZ() - closest_ground_target_altitude)
    in_suppression_range = (
        horizontal_distance <= WATER_MANUAL_SPRAY_RADIUS_METERS
        and altitude_error <= WATER_MANUAL_SPRAY_ALTITUDE_TOLERANCE_METERS
    )
    is_engaged = engage_water_suppression_if_possible(
        follower,
        in_suppression_range,
        dt,
    )
    register_hotspot_water_drone_contact(
        closest_hotspot,
        is_engaged,
    )
    update_water_suppression_visual(
        closest_hotspot,
        source_root=rig,
        stream_state=follower,
        engaged=is_engaged,
    )


def update_follower_drones(dt):
    if not follower_drones:
        return
    selected_node = get_camera_target_node()
    for follower in follower_drones:
        rig = follower["root"]
        if rig is None or rig.isEmpty():
            continue
        if is_drone_damaged(follower.get("role"), follower.get("slot", 0) + 1):
            # Offline: hovers dead, ignores input and automation.
            update_manual_follower_drone(follower, dt, False)
        elif follower.get("control_mode", CONTROL_MODE_AUTOMATION) == CONTROL_MODE_MANUAL:
            update_manual_follower_drone(follower, dt, selected_node is rig)
        elif follower.get("role") == "water":
            update_water_follower_drone(follower, dt)
        else:
            update_survey_follower_drone(follower, dt)


def update_water_drone(dt):
    if WATER_FOLLOWS_PAIRED_SURVEY:
        target_hotspot = choose_paired_survey_water_hotspot(
            1, water_drone.getX(), water_drone.getY()
        )
    else:
        target_hotspot = choose_water_support_hotspot(
            water_drone.getX(),
            water_drone.getY(),
        )
    team_state.water_target_hotspot = target_hotspot
    water_motion_state.sim_time_seconds += dt
    max_horizontal_speed = WATER_DRONE_CRUISE_SPEED_METERS_PER_SECOND
    max_vertical_speed = WATER_DRONE_VERTICAL_SPEED_METERS_PER_SECOND

    if target_hotspot is None:
        water_drone_state.tracked_hotspot_id = None
        # #6: with no fire from its paired survey drone, trail that survey drone
        # instead of returning all the way home.
        follow_anchor = (
            get_water_follow_anchor(1) if WATER_FOLLOWS_PAIRED_SURVEY else None
        )
        if follow_anchor is not None:
            loiter_x, loiter_y = follow_anchor
        else:
            loiter_x, loiter_y = water_drone_state.home_x, water_drone_state.home_y
        water_drone_state.home_z = compute_safe_altitude_at(
            loiter_x,
            loiter_y,
            max(WATER_DRONE_HOME_ALTITUDE_METERS, water_drone_state.home_z),
            WATER_DRONE_MIN_ALTITUDE_ABOVE_GROUND_METERS,
            canopy_clearance_meters=WATER_DRONE_CANOPY_CLEARANCE_METERS,
        )
        desired_vx, desired_vy, _ = compute_point_tracking_velocity(
            water_drone.getX(),
            water_drone.getY(),
            loiter_x,
            loiter_y,
            max_horizontal_speed * 0.72,
            8.0,
            min_speed_scale=0.0,
        )
        update_guided_drone_motion(
            water_drone,
            water_drone_visual,
            water_drone_propellers,
            water_motion_state,
            desired_vx,
            desired_vy,
            water_drone_state.home_z,
            max_horizontal_speed * 0.72,
            max_vertical_speed,
            dt,
            visual_tilt_scale=WATER_DRONE_AUTO_VISUAL_TILT_SCALE,
            visual_tilt_time_constant_seconds=WATER_DRONE_AUTO_VISUAL_TILT_TIME_CONSTANT_SECONDS,
        )
        home_dx = water_drone.getX() - water_drone_state.home_x
        home_dy = water_drone.getY() - water_drone_state.home_y
        home_distance = (home_dx * home_dx + home_dy * home_dy) ** 0.5
        if (
            water_drone_state.mission_active
            and home_distance <= 2.0
            and abs(water_drone.getZ() - water_drone_state.home_z) <= 1.2
        ):
            water_drone_state.mission_active = False
        clear_water_suppression_visual()
        return

    water_drone_state.mission_active = True
    hotspot_id = id(target_hotspot)
    if water_drone_state.tracked_hotspot_id != hotspot_id:
        water_drone_state.tracked_hotspot_id = hotspot_id
        water_drone_state.hover_offset_angle_radians = compute_water_support_hover_angle(
            target_hotspot,
            1,
            water_drone.getX(),
            water_drone.getY(),
        )

    desired_x = target_hotspot.root.getX() + (
        cos(water_drone_state.hover_offset_angle_radians)
        * WATER_DRONE_HOVER_HOLD_RADIUS_METERS
    )
    desired_y = target_hotspot.root.getY() + (
        sin(water_drone_state.hover_offset_angle_radians)
        * WATER_DRONE_HOVER_HOLD_RADIUS_METERS
    )
    desired_z = max(
        compute_safe_altitude_at(
            water_drone.getX(),
            water_drone.getY(),
            WATER_DRONE_HOME_ALTITUDE_METERS,
            WATER_DRONE_MIN_ALTITUDE_ABOVE_GROUND_METERS,
            canopy_clearance_meters=WATER_DRONE_CANOPY_CLEARANCE_METERS,
        ),
        compute_safe_altitude_at(
            desired_x,
            desired_y,
            target_hotspot.ground_z + WATER_DRONE_SUPPRESSION_ALTITUDE_METERS,
            WATER_DRONE_MIN_ALTITUDE_ABOVE_GROUND_METERS,
            canopy_clearance_meters=WATER_DRONE_CANOPY_CLEARANCE_METERS,
        ),
        compute_safe_altitude_at(
            target_hotspot.root.getX(),
            target_hotspot.root.getY(),
            target_hotspot.ground_z + WATER_DRONE_SUPPRESSION_ALTITUDE_METERS,
            WATER_DRONE_MIN_ALTITUDE_ABOVE_GROUND_METERS,
            canopy_clearance_meters=WATER_DRONE_CANOPY_CLEARANCE_METERS,
        ),
    )
    desired_vx, desired_vy, _ = compute_point_tracking_velocity(
        water_drone.getX(),
        water_drone.getY(),
        desired_x,
        desired_y,
        max_horizontal_speed * 0.82,
        WATER_DRONE_HOVER_HOLD_RADIUS_METERS * 2.1,
        min_speed_scale=0.0,
    )
    target_heading_degrees = water_suppression_heading_degrees(
        water_drone.getX(),
        water_drone.getY(),
        target_hotspot,
    )
    update_guided_drone_motion(
        water_drone,
        water_drone_visual,
        water_drone_propellers,
        water_motion_state,
        desired_vx,
        desired_vy,
        desired_z,
        max_horizontal_speed * 0.82,
        max_vertical_speed * 0.92,
        dt,
        target_heading_degrees=target_heading_degrees,
        visual_tilt_scale=WATER_DRONE_AUTO_VISUAL_TILT_SCALE,
        visual_tilt_time_constant_seconds=WATER_DRONE_AUTO_VISUAL_TILT_TIME_CONSTANT_SECONDS,
    )

    dx = water_drone.getX() - target_hotspot.root.getX()
    dy = water_drone.getY() - target_hotspot.root.getY()
    horizontal_distance = (dx * dx + dy * dy) ** 0.5
    altitude_error = abs(water_drone.getZ() - desired_z)
    in_suppression_range = (
        horizontal_distance <= WATER_DRONE_SUPPRESSION_RADIUS_METERS
        and altitude_error <= 3.2
    )
    is_engaged = engage_water_suppression_if_possible(
        water_drone_state,
        in_suppression_range,
        dt,
    )
    register_hotspot_water_drone_contact(target_hotspot, is_engaged)

    update_water_suppression_visual(target_hotspot, engaged=is_engaged)


def get_unprotected_drone_roots():
    """Roots of drones that get NO separation assist: manually flown drones
    (collision avoidance off in manual). Already-offline drones are ignored so
    a crash cannot cascade through the rest of the team while falling."""
    unprotected = set()
    for view in get_all_drone_views():
        root = view.get("root")
        if root is None or root.isEmpty():
            continue
        if is_view_damaged(view):
            continue
        if get_drone_view_control_mode(view) == CONTROL_MODE_MANUAL:
            unprotected.add(id(root))
    return unprotected


def check_manual_collisions():
    """Damage both drones of any pair that actually collides. Pairs of two
    automated drones are excluded (their assist keeps them apart), and already
    crashed drones cannot cause new damage while falling."""
    views = [
        view
        for view in get_all_drone_views()
        if view.get("root") is not None
        and not view["root"].isEmpty()
        and not is_view_damaged(view)
    ]
    for first_index in range(len(views)):
        first = views[first_index]
        first_unassisted = get_drone_view_control_mode(first) == CONTROL_MODE_MANUAL
        for second in views[first_index + 1:]:
            second_unassisted = get_drone_view_control_mode(second) == CONTROL_MODE_MANUAL
            if not (first_unassisted or second_unassisted):
                continue
            dx = second["root"].getX() - first["root"].getX()
            dy = second["root"].getY() - first["root"].getY()
            dz = second["root"].getZ() - first["root"].getZ()
            horizontal_distance = (dx * dx + dy * dy) ** 0.5
            if (
                horizontal_distance < DRONE_CRASH_RADIUS_METERS
                and abs(dz) < DRONE_CRASH_VERTICAL_RADIUS_METERS
            ):
                labels = (
                    format_drone_label(first["role"], first["slot"]),
                    format_drone_label(second["role"], second["slot"]),
                )
                first_damaged = mark_drone_damaged(
                    first["role"],
                    first["slot"],
                    damage_reason=DAMAGE_REASON_COLLISION,
                    show_alert=False,
                )
                second_damaged = mark_drone_damaged(
                    second["role"],
                    second["slot"],
                    damage_reason=DAMAGE_REASON_COLLISION,
                    show_alert=False,
                )
                if first_damaged or second_damaged:
                    show_collision_alert(labels)


def get_environment_impact_reason(view):
    root = view.get("root")
    if root is None or root.isEmpty() or is_view_damaged(view):
        return None

    drone_x = root.getX()
    drone_y = root.getY()
    drone_z = root.getZ()
    ground_sample = sample_ground(drone_x, drone_y)
    ground_z = ground_sample[0].z if ground_sample is not None else None

    if (
        ground_z is not None
        and drone_z <= ground_z + DRONE_GROUND_IMPACT_CLEARANCE_METERS
    ):
        return DAMAGE_REASON_GROUND_IMPACT

    if tree_canopy_samples:
        canopy_height = compute_local_canopy_height(
            tree_canopy_samples,
            drone_x,
            drone_y,
            DRONE_BRANCH_IMPACT_QUERY_RADIUS_METERS,
        )
        if canopy_height is not None:
            above_ground = (
                ground_z is None
                or drone_z >= ground_z + DRONE_GROUND_IMPACT_CLEARANCE_METERS
            )
            if (
                above_ground
                and drone_z <= canopy_height + DRONE_BRANCH_IMPACT_VERTICAL_MARGIN_METERS
            ):
                return DAMAGE_REASON_BRANCH_IMPACT

    return None


def check_environment_impacts():
    """Tree/ground impacts damage only the impacted drone.

    Drone-drone collisions are handled separately and damage exactly the two
    drones in the collision pair.
    """
    for view in get_all_drone_views():
        damage_reason = get_environment_impact_reason(view)
        if damage_reason is None:
            continue
        mark_drone_damaged(
            view["role"],
            view["slot"],
            damage_reason=damage_reason,
        )


def get_active_drone_motion_entries():
    entries = [
        {
            "root": drone,
            "motion_state": motion_state,
            "role": "survey",
        },
        {
            "root": water_drone,
            "motion_state": water_motion_state,
            "role": "water",
        },
    ]
    for follower in iter_active_follower_drones():
        entries.append(
            {
                "root": follower["root"],
                "motion_state": follower["motion_state"],
                "role": follower["role"],
            }
        )
    return entries


def damp_drone_entry_horizontal_velocity(entry, scale=0.35):
    entry_motion = entry.get("motion_state")
    if entry_motion is None:
        return
    entry_motion.velocity.x *= scale
    entry_motion.velocity.y *= scale


def lift_water_entry_above_survey_entry(water_entry, survey_entry):
    water_root = water_entry["root"]
    survey_root = survey_entry["root"]
    target_water_z = max(
        water_root.getZ(),
        survey_root.getZ() + DRONE_INTERACTION_MIN_VERTICAL_SEPARATION_METERS,
        survey_root.getZ() + DRONE_INTERACTION_WATER_ALTITUDE_BIAS_METERS,
    )
    target_water_z = compute_safe_altitude_at(
        water_root.getX(),
        water_root.getY(),
        target_water_z,
        WATER_DRONE_MIN_ALTITUDE_ABOVE_GROUND_METERS,
        canopy_clearance_meters=WATER_DRONE_CANOPY_CLEARANCE_METERS,
    )
    if water_root.getZ() < target_water_z:
        water_root.setZ(target_water_z)
        water_motion = water_entry.get("motion_state")
        if water_motion is not None:
            water_motion.velocity.z = max(0.0, water_motion.velocity.z)
            if water_motion.target_altitude_meters is None:
                water_motion.target_altitude_meters = target_water_z
            else:
                water_motion.target_altitude_meters = max(
                    water_motion.target_altitude_meters,
                    target_water_z,
                )


def enforce_inter_drone_separation():
    unprotected_roots = get_unprotected_drone_roots()
    lead_pair_protected = (
        id(drone) not in unprotected_roots
        and id(water_drone) not in unprotected_roots
    )
    separation_dx = water_drone.getX() - drone.getX()
    separation_dy = water_drone.getY() - drone.getY()
    horizontal_distance = (separation_dx * separation_dx + separation_dy * separation_dy) ** 0.5

    if (
        lead_pair_protected
        and horizontal_distance < DRONE_INTERACTION_MIN_HORIZONTAL_SEPARATION_METERS
    ):
        if horizontal_distance <= 0.001:
            unit_x = 1.0
            unit_y = 0.0
        else:
            unit_x = separation_dx / horizontal_distance
            unit_y = separation_dy / horizontal_distance

        push_distance = DRONE_INTERACTION_MIN_HORIZONTAL_SEPARATION_METERS - horizontal_distance
        water_push = push_distance * 0.72
        survey_push = push_distance * 0.28

        water_x = clamp(
            water_drone.getX() + (unit_x * water_push),
            SPAWN_X_MIN + DRONE_INTERACTION_EDGE_MARGIN_METERS,
            SPAWN_X_MAX - DRONE_INTERACTION_EDGE_MARGIN_METERS,
        )
        water_y = clamp(
            water_drone.getY() + (unit_y * water_push),
            SPAWN_Y_MIN + DRONE_INTERACTION_EDGE_MARGIN_METERS,
            SPAWN_Y_MAX - DRONE_INTERACTION_EDGE_MARGIN_METERS,
        )
        drone_x = clamp(
            drone.getX() - (unit_x * survey_push),
            SPAWN_X_MIN + DRONE_INTERACTION_EDGE_MARGIN_METERS,
            SPAWN_X_MAX - DRONE_INTERACTION_EDGE_MARGIN_METERS,
        )
        drone_y = clamp(
            drone.getY() - (unit_y * survey_push),
            SPAWN_Y_MIN + DRONE_INTERACTION_EDGE_MARGIN_METERS,
            SPAWN_Y_MAX - DRONE_INTERACTION_EDGE_MARGIN_METERS,
        )

        water_drone.setX(water_x)
        water_drone.setY(water_y)
        drone.setX(drone_x)
        drone.setY(drone_y)

        water_motion_state.velocity.x = 0.0
        water_motion_state.velocity.y = 0.0
        motion_state.velocity.x *= 0.35
        motion_state.velocity.y *= 0.35

    post_dx = water_drone.getX() - drone.getX()
    post_dy = water_drone.getY() - drone.getY()
    post_distance = (post_dx * post_dx + post_dy * post_dy) ** 0.5
    if lead_pair_protected and post_distance < (
        DRONE_INTERACTION_MIN_HORIZONTAL_SEPARATION_METERS * 1.35
    ):
        target_water_z = max(
            water_drone.getZ(),
            drone.getZ() + DRONE_INTERACTION_MIN_VERTICAL_SEPARATION_METERS,
            drone.getZ() + DRONE_INTERACTION_WATER_ALTITUDE_BIAS_METERS,
        )
        target_water_z = compute_safe_altitude_at(
            water_drone.getX(),
            water_drone.getY(),
            target_water_z,
            WATER_DRONE_MIN_ALTITUDE_ABOVE_GROUND_METERS,
            canopy_clearance_meters=WATER_DRONE_CANOPY_CLEARANCE_METERS,
        )
        if water_drone.getZ() < target_water_z:
            water_drone.setZ(target_water_z)
            water_motion_state.velocity.z = max(0.0, water_motion_state.velocity.z)
            if water_motion_state.target_altitude_meters is None:
                water_motion_state.target_altitude_meters = target_water_z
            else:
                water_motion_state.target_altitude_meters = max(
                    water_motion_state.target_altitude_meters,
                    target_water_z,
                )

    entries = get_active_drone_motion_entries()
    for first_index in range(len(entries)):
        first = entries[first_index]
        first_root = first["root"]
        if first_root is drone:
            skip_root = water_drone
        elif first_root is water_drone:
            skip_root = drone
        else:
            skip_root = None
        if id(first_root) in unprotected_roots:
            continue
        for second in entries[first_index + 1:]:
            second_root = second["root"]
            if skip_root is second_root:
                continue
            if id(second_root) in unprotected_roots:
                continue

            separation_dx = second_root.getX() - first_root.getX()
            separation_dy = second_root.getY() - first_root.getY()
            horizontal_distance = (separation_dx * separation_dx + separation_dy * separation_dy) ** 0.5
            if horizontal_distance >= DRONE_INTERACTION_MIN_HORIZONTAL_SEPARATION_METERS:
                continue

            if horizontal_distance <= 0.001:
                unit_x = 1.0
                unit_y = 0.0
            else:
                unit_x = separation_dx / horizontal_distance
                unit_y = separation_dy / horizontal_distance

            push_distance = DRONE_INTERACTION_MIN_HORIZONTAL_SEPARATION_METERS - horizontal_distance
            first_push = push_distance * 0.5
            second_push = push_distance * 0.5
            if first["role"] == "water" and second["role"] != "water":
                first_push = push_distance * 0.72
                second_push = push_distance * 0.28
            elif first["role"] != "water" and second["role"] == "water":
                first_push = push_distance * 0.28
                second_push = push_distance * 0.72

            first_root.setX(
                clamp(
                    first_root.getX() - (unit_x * first_push),
                    SPAWN_X_MIN + DRONE_INTERACTION_EDGE_MARGIN_METERS,
                    SPAWN_X_MAX - DRONE_INTERACTION_EDGE_MARGIN_METERS,
                )
            )
            first_root.setY(
                clamp(
                    first_root.getY() - (unit_y * first_push),
                    SPAWN_Y_MIN + DRONE_INTERACTION_EDGE_MARGIN_METERS,
                    SPAWN_Y_MAX - DRONE_INTERACTION_EDGE_MARGIN_METERS,
                )
            )
            second_root.setX(
                clamp(
                    second_root.getX() + (unit_x * second_push),
                    SPAWN_X_MIN + DRONE_INTERACTION_EDGE_MARGIN_METERS,
                    SPAWN_X_MAX - DRONE_INTERACTION_EDGE_MARGIN_METERS,
                )
            )
            second_root.setY(
                clamp(
                    second_root.getY() + (unit_y * second_push),
                    SPAWN_Y_MIN + DRONE_INTERACTION_EDGE_MARGIN_METERS,
                    SPAWN_Y_MAX - DRONE_INTERACTION_EDGE_MARGIN_METERS,
                )
            )
            damp_drone_entry_horizontal_velocity(first)
            damp_drone_entry_horizontal_velocity(second)

            if first["role"] == "water" and second["role"] != "water":
                lift_water_entry_above_survey_entry(first, second)
            elif first["role"] != "water" and second["role"] == "water":
                lift_water_entry_above_survey_entry(second, first)


def get_current_water_target_hotspot_ids():
    target_ids = set()
    for view in get_all_drone_views():
        if view["role"] != "water" or is_view_damaged(view):
            continue
        target_hotspot = get_water_current_target_for_view(view)
        if target_hotspot is None:
            continue
        target_ids.add(id(target_hotspot))
    return target_ids


def update_fire_map_hotspot_nodes(visible_hotspots):
    visible_hotspot_ids = {id(hotspot) for hotspot in visible_hotspots}
    water_target_hotspot_ids = get_current_water_target_hotspot_ids()

    stale_hotspot_ids = [
        hotspot_id
        for hotspot_id in list(fire_map_detected_hotspot_nodes.keys())
        if hotspot_id not in visible_hotspot_ids
    ]
    for hotspot_id in stale_hotspot_ids:
        marker_node = fire_map_detected_hotspot_nodes.pop(hotspot_id, None)
        if marker_node is not None:
            label_text = marker_node.getPythonTag("label_text")
            if label_text is not None:
                label_text.destroy()
            marker_node.removeNode()

    for hotspot_index, hotspot in enumerate(visible_hotspots, start=1):
        hotspot_id = id(hotspot)
        marker_node = fire_map_detected_hotspot_nodes.get(hotspot_id)
        if marker_node is None:
            marker_node = build_fire_map_hotspot_node()
            fire_map_detected_hotspot_nodes[hotspot_id] = marker_node

        map_x, map_y = world_to_fire_map_coords(
            hotspot.root.getX(),
            hotspot.root.getY(),
        )
        marker_node.setPos(map_x, 0.0, map_y)
        label_text = marker_node.getPythonTag("label_text")
        label_text.setPos(map_x, map_y - 0.004)

        flame_node = marker_node.getPythonTag("flame_node")
        firetruck_suppression_fill_node = marker_node.getPythonTag("firetruck_suppression_fill_node")
        halo_node = marker_node.getPythonTag("halo_node")
        cross_node = marker_node.getPythonTag("cross_node")
        water_target_ring_node = marker_node.getPythonTag("water_target_ring_node")
        water_target_cross_node = marker_node.getPythonTag("water_target_cross_node")
        truck_node = marker_node.getPythonTag("truck_node")
        pulse_phase = motion_state.sim_time_seconds * 2.5
        pulse_scale = 1.0 + (sin(pulse_phase) * 0.16)
        target_pulse_scale = 1.0 + (sin(pulse_phase + 0.8) * 0.22)
        firetruck_response_active = hotspot_has_fire_truck_response(hotspot)
        show_water_target = (
            hotspot_id in water_target_hotspot_ids
            and hotspot.suppression_state in (FIRE_STATE_ACTIVE, FIRE_STATE_CONTAINED)
            and not firetruck_response_active
        )
        if show_water_target:
            water_target_ring_node.show()
            water_target_cross_node.show()
            water_target_ring_node.setScale(target_pulse_scale)
            water_target_cross_node.setScale(0.9 + (target_pulse_scale - 1.0) * 0.35)
        else:
            water_target_ring_node.hide()
            water_target_cross_node.hide()
        if firetruck_suppression_fill_node is not None:
            if firetruck_response_active and hotspot.suppression_state in (
                FIRE_STATE_ACTIVE,
                FIRE_STATE_CONTAINED,
            ):
                set_fire_map_marker_color(
                    firetruck_suppression_fill_node,
                    FIRE_TRUCK_SUPPRESSING_COLOR,
                )
                firetruck_suppression_fill_node.show()
            else:
                firetruck_suppression_fill_node.hide()

        if hotspot.suppression_state == FIRE_STATE_ACTIVE:
            flame_node.show()
            halo_node.show()
            if firetruck_response_active:
                label_text.setText("")
                set_fire_map_flame_pin_colors(
                    flame_node,
                    FIRE_TRUCK_SUPPRESSING_COLOR,
                    FIRE_TRUCK_SUPPRESSING_CORE_COLOR,
                )
                set_fire_map_marker_color(halo_node, FIRE_TRUCK_SUPPRESSING_GLOW_COLOR)
                set_fire_map_marker_color(cross_node, FIRE_TRUCK_SUPPRESSING_COLOR)
                truck_node.hide()
            elif hotspot.water_drone_engaged:
                label_text.setText("")
                set_fire_map_flame_pin_colors(
                    flame_node,
                    (0.5, 0.85, 1.0, 1.0),
                    (0.72, 0.94, 1.0, 1.0),
                )
                set_fire_map_marker_color(halo_node, (0.4, 0.8, 1.0, 0.5))
                set_fire_map_marker_color(cross_node, (0.46, 0.92, 1.0, 1.0))
                truck_node.hide()
            elif hotspot.water_drone_requested:
                label_text.setText("")
                set_fire_map_flame_pin_colors(
                    flame_node,
                    (1.0, 0.82, 0.4, 1.0),
                    (1.0, 0.92, 0.58, 1.0),
                )
                set_fire_map_marker_color(halo_node, (1.0, 0.7, 0.25, 0.5))
                set_fire_map_marker_color(cross_node, (1.0, 0.86, 0.28, 1.0))
                truck_node.hide()
            else:
                label_text.setText("")
                set_fire_map_flame_pin_colors(
                    flame_node,
                    (1.0, 0.55, 0.3, 1.0),
                    (1.0, 0.86, 0.32, 1.0),
                )
                set_fire_map_marker_color(halo_node, (1.0, 0.4, 0.18, 0.5))
                set_fire_map_marker_color(cross_node, (1.0, 0.36, 0.16, 1.0))
                truck_node.hide()
            flame_node.setScale(pulse_scale)
            halo_node.setScale(pulse_scale)
            cross_node.setScale(0.8)
        elif hotspot.suppression_state == FIRE_STATE_CONTAINED:
            flame_node.show()
            halo_node.hide()
            label_text.setText("")
            if firetruck_response_active:
                set_fire_map_flame_pin_colors(
                    flame_node,
                    FIRE_TRUCK_SUPPRESSING_COLOR,
                    FIRE_TRUCK_SUPPRESSING_CORE_COLOR,
                )
                set_fire_map_marker_color(cross_node, FIRE_TRUCK_SUPPRESSING_COLOR)
                truck_node.hide()
            else:
                set_fire_map_flame_pin_colors(
                    flame_node,
                    (1.0, 0.82, 0.4, 0.85),
                    (1.0, 0.92, 0.58, 0.95),
                )
                set_fire_map_marker_color(cross_node, (1.0, 0.84, 0.42, 0.98))
                truck_node.hide()
            flame_node.setScale(0.85)
            cross_node.setScale(0.8)
        elif hotspot.suppression_state == FIRE_STATE_OUT:
            flame_node.hide()
            halo_node.hide()
            label_text.setText("")
            truck_node.hide()
            set_fire_map_marker_color(cross_node, (0.46, 0.96, 0.56, 0.98))
            cross_node.setScale(0.85)
        else:
            flame_node.hide()
            halo_node.hide()
            label_text.setText("")
            truck_node.hide()
            set_fire_map_marker_color(cross_node, (0.58, 0.18, 0.14, 0.9))
            cross_node.setScale(0.8)


def fire_map_drone_key(view):
    return f"{view['role']}_{view['slot']}"


def build_fire_map_drone_node(view):
    role = view["role"]
    slot = view["slot"]
    if role == "survey":
        marker_color = SURVEY_DRONE_MODEL_TINT
        ring_color = (0.86, 0.96, 1.0, 0.24)
        ring_radius_meters = FIRE_HOTSPOT_DETECTION_RADIUS_METERS
    else:
        marker_color = WATER_DRONE_MODEL_TINT
        ring_color = (0.36, 0.72, 1.0, 0.28)
        ring_radius_meters = WATER_DRONE_SUPPRESSION_RADIUS_METERS

    ring_node = build_fire_map_circle(
        ring_radius_meters,
        ring_color,
        parent=fire_map_marker_root,
        thickness=1.1,
    )
    ring_node.setTransparency(TransparencyAttrib.MAlpha)
    ring_node.setBin("fixed", 106)

    marker_node = make_fire_map_square(
        f"fire_map_{role}_{slot}_marker",
        FIRE_MAP_DRONE_MARKER_HALF_SIZE,
        marker_color,
        parent=fire_map_marker_root,
    )
    marker_node.setBin("fixed", 118)

    label_text = OnscreenText(
        text="",
        parent=fire_map_panel,
        pos=(0.0, 0.0),
        scale=0.022,
        fg=(1.0, 1.0, 1.0, 0.96),
        align=TextNode.ALeft,
        mayChange=True,
    )
    label_text.setBin("fixed", 124)
    label_text.setDepthWrite(False)
    label_text.setDepthTest(False)

    return {
        "marker_node": marker_node,
        "ring_node": ring_node,
        "label_text": label_text,
    }


def remove_fire_map_drone_node(node_info):
    if node_info is None:
        return
    label_text = node_info.get("label_text")
    if label_text is not None:
        label_text.destroy()
    for node_key in ("marker_node", "ring_node"):
        node = node_info.get(node_key)
        if node is not None and not node.isEmpty():
            node.removeNode()


def build_fire_map_truck_node(truck_state):
    marker_node = build_fire_map_truck_marker(
        fire_map_marker_root,
        offset_x=0.0,
        offset_z=0.0,
    )
    marker_node.setScale(FIRE_TRUCK_MAP_MARKER_SCALE * FIRE_TRUCK_MAP_ICON_IDLE_SCALE)
    marker_node.setBin("fixed", 119)
    marker_node.show()
    return {
        "marker_node": marker_node,
    }


def remove_fire_map_truck_node(node_info):
    if node_info is None:
        return
    marker_node = node_info.get("marker_node")
    if marker_node is not None and not marker_node.isEmpty():
        marker_node.removeNode()


def update_fire_map_truck_nodes():
    if fire_map_panel is None or fire_map_marker_root is None:
        return
    active_keys = {truck_state.slot for truck_state in fire_truck_states}
    for stale_key in [
        node_key
        for node_key in list(fire_map_truck_nodes.keys())
        if node_key not in active_keys
    ]:
        remove_fire_map_truck_node(fire_map_truck_nodes.pop(stale_key, None))

    for truck_state in fire_truck_states:
        node_info = fire_map_truck_nodes.get(truck_state.slot)
        if node_info is None:
            node_info = build_fire_map_truck_node(truck_state)
            fire_map_truck_nodes[truck_state.slot] = node_info
        map_x, map_y = world_to_fire_map_coords(
            truck_state.root.getX(),
            truck_state.root.getY(),
        )
        marker_node = node_info["marker_node"]
        if truck_state.target_hotspot is not None:
            icon_scale = FIRE_TRUCK_MAP_ICON_ACTIVE_SCALE
            bob_amount = FIRE_TRUCK_MAP_ICON_ACTIVE_BOB
            bob_speed = 6.0
        else:
            icon_scale = FIRE_TRUCK_MAP_ICON_IDLE_SCALE
            bob_amount = FIRE_TRUCK_MAP_ICON_IDLE_BOB
            bob_speed = 2.4
        icon_phase = motion_state.sim_time_seconds * bob_speed + (truck_state.slot * 1.3)
        marker_node.setScale(FIRE_TRUCK_MAP_MARKER_SCALE * icon_scale)
        marker_node.setPos(
            clamp(
                map_x + (sin(icon_phase) * bob_amount),
                FIRE_MAP_VIEW_LEFT + 0.018,
                FIRE_MAP_VIEW_LEFT + FIRE_MAP_VIEW_WIDTH - 0.018,
            ),
            0.0,
            clamp(
                map_y + (abs(cos(icon_phase)) * bob_amount),
                FIRE_MAP_VIEW_BOTTOM + 0.014,
                FIRE_MAP_VIEW_BOTTOM + FIRE_MAP_VIEW_HEIGHT - 0.014,
            ),
        )


def update_fire_map_drone_nodes():
    if fire_map_panel is None or fire_map_marker_root is None:
        return

    views = [
        view
        for view in get_all_drone_views()
        if view["root"] is not None and not view["root"].isEmpty()
    ]
    active_keys = {fire_map_drone_key(view) for view in views}
    for stale_key in [
        node_key
        for node_key in list(fire_map_drone_nodes.keys())
        if node_key not in active_keys
    ]:
        remove_fire_map_drone_node(fire_map_drone_nodes.pop(stale_key, None))

    selected_node = get_camera_target_node()
    for view in views:
        node_key = fire_map_drone_key(view)
        node_info = fire_map_drone_nodes.get(node_key)
        if node_info is None:
            node_info = build_fire_map_drone_node(view)
            fire_map_drone_nodes[node_key] = node_info

        map_x, map_y = world_to_fire_map_coords(
            view["root"].getX(),
            view["root"].getY(),
        )
        selected = view["root"] is selected_node
        marker_scale = FIRE_MAP_SELECTED_MARKER_SCALE if selected else 1.0
        marker_node = node_info["marker_node"]
        ring_node = node_info["ring_node"]
        label_text = node_info["label_text"]

        marker_node.setPos(map_x, 0.0, map_y)
        marker_node.setScale(marker_scale, 1.0, marker_scale)
        marker_node.setColorScale(
            (1.0, 1.0, 0.62, 1.0) if selected else (1.0, 1.0, 1.0, 1.0)
        )

        ring_node.setPos(map_x, 0.0, map_y)
        ring_node.setScale(1.0, 1.0, 1.0)
        ring_node.setColorScale(
            (1.0, 1.0, 0.74, 1.0) if selected else (1.0, 1.0, 1.0, 1.0)
        )

        prefix = "S" if view["role"] == "survey" else "W"
        if is_view_damaged(view):
            mode_letter = "OFF"
        elif get_drone_view_control_mode(view) == CONTROL_MODE_MANUAL:
            mode_letter = "M"
        else:
            mode_letter = "A"
        label_text.setText(f"{prefix}{view['slot']} {mode_letter}")

        label_offset_y = 0.014 + (0.012 * (view["slot"] - 1))
        if view["role"] == "water":
            label_offset_y *= -1.0
        label_x = clamp(
            map_x + 0.014,
            FIRE_MAP_VIEW_LEFT + 0.004,
            FIRE_MAP_VIEW_LEFT + FIRE_MAP_VIEW_WIDTH - 0.065,
        )
        label_y = clamp(
            map_y + label_offset_y,
            FIRE_MAP_VIEW_BOTTOM + 0.006,
            FIRE_MAP_VIEW_BOTTOM + FIRE_MAP_VIEW_HEIGHT - 0.012,
        )
        label_text.setPos(label_x, label_y)
        if selected:
            label_text.setFg((1.0, 1.0, 0.68, 1.0))
        elif view["role"] == "survey":
            label_text.setFg((0.96, 0.99, 1.0, 0.96))
        else:
            label_text.setFg((0.68, 0.86, 1.0, 0.96))


def update_fire_map_popup():
    detected_hotspots = [
        hotspot
        for hotspot in fire_hotspots
        if hotspot.detected
        and hotspot.suppression_state
        in (
            FIRE_STATE_ACTIVE,
            FIRE_STATE_CONTAINED,
            FIRE_STATE_OUT,
            FIRE_STATE_BURNED,
        )
    ]
    pending_hotspot = choose_pending_water_dispatch_hotspot()
    team_state.pending_water_dispatch_hotspot = pending_hotspot
    if fire_map_panel is None:
        return

    update_fire_map_drone_nodes()
    update_fire_map_truck_nodes()
    update_fire_map_trajectories()

    visible_hotspots = sorted(
        detected_hotspots,
        key=lambda hotspot: (
            {
                FIRE_STATE_ACTIVE: 0,
                FIRE_STATE_CONTAINED: 1,
                FIRE_STATE_OUT: 2,
                FIRE_STATE_BURNED: 3,
            }.get(hotspot.suppression_state, 4),
            -hotspot.mapping_progress_seconds,
        ),
    )
    update_fire_map_hotspot_nodes(visible_hotspots)
    fire_map_header_text.setText("DETECTED FIRE MAP")
    if visible_hotspots:
        fire_map_prompt_text.setText(
            f"Detected fire markers shown: {len(visible_hotspots)}."
        )
    else:
        fire_map_prompt_text.setText(
            "Detected fire markers only: no detected fires yet."
        )

    if fire_map_wind_arrow_root is not None:
        _, _, wind_direction_degrees, wind_speed_mps = get_wind_velocity()
        fire_map_wind_arrow_root.setR(wind_direction_degrees)
        if fire_map_wind_label is not None:
            fire_map_wind_label.setText(
                f"WIND {get_wind_compass_label(wind_direction_degrees)} {wind_speed_mps:.1f} m/s"
            )
   
    fire_map_status_text.setText(
        "Undetected fire stays hidden."
    )
    # The little map hides while the operator is in the full-screen satellite
    # view (#14); it reappears as soon as they return to a single drone view.
    if overview_camera_enabled and overview_is_operator:
        fire_map_panel.hide()
    else:
        fire_map_panel.show()


def update_dispatch_alert_overlay(dt):
    global dispatch_alert_flash_timer_seconds, dispatch_alert_hotspot
    global dispatch_prompt_cycle_active

    if dispatch_alert_root is None:
        return
    # Full auto dispatches water automatically. If any water slot is manual or
    # still holding at spawn, the operator still gets the dispatch prompt.
    if (
        not dispatch_prompt_cycle_active
        and any_water_slot_can_auto_dispatch()
        and not any_water_slot_needs_operator_dispatch()
    ):
        dispatch_alert_root.hide()
        return
    if dispatch_prompt_cycle_active and not any_water_slot_needs_operator_dispatch():
        clear_dispatch_alert()
        return

    pending_hotspot = dispatch_alert_hotspot
    if pending_hotspot is None or pending_hotspot.root.isEmpty():
        pending_hotspot = choose_pending_water_dispatch_hotspot()
        dispatch_alert_hotspot = pending_hotspot
        if pending_hotspot is None:
            dispatch_prompt_cycle_active = False
            dispatch_alert_root.hide()
            return
    if not is_active_detected_hotspot(pending_hotspot):
        pending_hotspot = choose_pending_water_dispatch_hotspot()
        dispatch_alert_hotspot = pending_hotspot
        if pending_hotspot is None:
            dispatch_prompt_cycle_active = False
            dispatch_alert_root.hide()
            return
    if not dispatch_prompt_cycle_active:
        dispatch_alert_root.hide()
        return

    hotspot_x = pending_hotspot.root.getX()
    hotspot_y = pending_hotspot.root.getY()
    dispatch_alert_text.setText("FIRE DETECTED - WATER DISPATCH")
    # One plain line per water drone, updating live as you press P/L/K.
    status_line = build_water_dispatch_status_line()
    dispatch_alert_body_text.setText(
        f"{status_line}\n"
        "Fixed keys switch only that water drone to AUTO + dispatch.\n"
        "Manual water: select it, fly it, then press [J] to spray.\n"
        f"Fire near map point ({hotspot_x:.0f}, {hotspot_y:.0f})"
    )

    dispatch_alert_backdrop.setColor(0.46, 0.08, 0.06, 0.9)
    dispatch_alert_text.setScale(0.047)
    dispatch_alert_body_text.setScale(0.038)
    dispatch_alert_root.show()


def try_detect_hotspot_from_drone(
    drone_root,
    hotspot,
    dt,
    forced_detection_enabled,
    detector_survey_slot=None,
):
    if drone_root is None or drone_root.isEmpty():
        return False
    dx = hotspot.root.getX() - drone_root.getX()
    dy = hotspot.root.getY() - drone_root.getY()
    horizontal_distance = (dx * dx + dy * dy) ** 0.5
    if horizontal_distance > FIRE_HOTSPOT_DETECTION_RADIUS_METERS:
        return False

    altitude_above_hotspot = drone_root.getZ() - hotspot.ground_z
    if altitude_above_hotspot < FIRE_HOTSPOT_MIN_SCAN_ALTITUDE_METERS:
        return False

    forced_detection_radius = (
        AUTOMATION_FORCED_DETECTION_RADIUS_METERS
        * survey_forced_detection_radius_scale()
    )
    if forced_detection_enabled and horizontal_distance <= forced_detection_radius:
        mark_hotspot_detected(
            hotspot,
            motion_state.sim_time_seconds,
            detected_by_survey_slot=detector_survey_slot,
        )
        return True

    range_factor = max(
        0.0,
        1.0 - (horizontal_distance / FIRE_HOTSPOT_DETECTION_RADIUS_METERS),
    )
    altitude_factor = max(
        0.25,
        min(
            1.0,
            altitude_above_hotspot / (FIRE_HOTSPOT_MIN_SCAN_ALTITUDE_METERS * 2.5),
        ),
    )
    detection_probability = (
        FIRE_HOTSPOT_DETECTION_PROBABILITY_PER_SECOND
        * survey_detection_probability_scale()
        * range_factor
        * altitude_factor
        * dt
    )
    detection_probability = max(0.0, min(0.95, detection_probability))

    if random.random() < detection_probability:
        mark_hotspot_detected(
            hotspot,
            motion_state.sim_time_seconds,
            detected_by_survey_slot=detector_survey_slot,
        )
        return True
    return False


def update_hotspot_detection(dt):
    survey_roots = get_active_survey_drone_roots()
    for hotspot in fire_hotspots:
        if hotspot.suppression_state != FIRE_STATE_ACTIVE:
            continue
        if hotspot.detected:
            continue
        for survey_root in survey_roots:
            forced_detection_enabled = (
                get_survey_drone_root_control_mode(survey_root)
                == CONTROL_MODE_AUTOMATION
            )
            if try_detect_hotspot_from_drone(
                survey_root,
                hotspot,
                dt,
                forced_detection_enabled,
                detector_survey_slot=get_survey_slot_for_root(survey_root),
            ):
                break


def update_hotspot_lifecycle(dt):
    for hotspot in fire_hotspots:
        previous_state = hotspot.suppression_state
        hotspot.ground_firefighter_engaged = False
        hotspot.current_suppression_rate_per_second = 0.0

        if (
            hotspot.suppression_state == FIRE_STATE_ACTIVE
            and not hotspot.water_drone_engaged
        ):
            hotspot.burn_age_seconds += dt
            if hotspot.burn_age_seconds >= FIRE_UNDETECTED_BURNOUT_DELAY_SECONDS:
                hotspot.suppression_state = FIRE_STATE_BURNED
                mark_hotspot_detected(
                    hotspot,
                    motion_state.sim_time_seconds,
                    highlight_dispatch=False,
                    trigger_alarm=False,
                )
                hotspot.suppression_work_seconds = 0.0
                hotspot.water_drone_requested = False
                hotspot.water_drone_assigned = False
                hotspot.water_drone_engaged = False
                hotspot.water_drone_assignment_count = 0
                hotspot.water_drone_engagement_count = 0
                hotspot.ground_firefighter_engaged = False
                hotspot.current_suppression_rate_per_second = 0.0

        if hotspot.detected and hotspot.suppression_state in (
            FIRE_STATE_ACTIVE,
            FIRE_STATE_CONTAINED,
            FIRE_STATE_OUT,
        ):
            hotspot.detection_age_seconds += dt

        if hotspot.suppression_state in (FIRE_STATE_ACTIVE, FIRE_STATE_CONTAINED):
            if hotspot_has_engaged_fire_truck(hotspot):
                hotspot.ground_firefighter_engaged = True
                hotspot.current_suppression_rate_per_second += (
                    GROUND_FIREFIGHTER_SUPPRESSION_RATE_PER_SECOND
                )
            if hotspot.water_drone_engaged:
                engaged_water_drone_count = max(
                    1,
                    hotspot.water_drone_engagement_count,
                )
                hotspot.current_suppression_rate_per_second += (
                    WATER_DRONE_ASSIST_SUPPRESSION_RATE_PER_SECOND
                    * engaged_water_drone_count
                )
            if hotspot.current_suppression_rate_per_second > 0.0:
                hotspot.suppression_work_seconds += (
                    hotspot.current_suppression_rate_per_second * dt
                )

        if hotspot.suppression_state == FIRE_STATE_ACTIVE:
            if hotspot.suppression_work_seconds >= FIRE_SUPPRESSION_CONTAIN_DELAY_SECONDS:
                hotspot.suppression_state = FIRE_STATE_CONTAINED
        elif hotspot.suppression_state == FIRE_STATE_CONTAINED:
            total_time_to_out = (
                FIRE_SUPPRESSION_CONTAIN_DELAY_SECONDS
                + FIRE_SUPPRESSION_EXTINGUISH_DELAY_SECONDS
            )
            if hotspot.suppression_work_seconds >= total_time_to_out:
                hotspot.suppression_state = FIRE_STATE_OUT
                hotspot.water_drone_requested = False
                hotspot.water_drone_assigned = False
                hotspot.water_drone_engaged = False
                hotspot.water_drone_assignment_count = 0
                hotspot.water_drone_engagement_count = 0
        elif hotspot.suppression_state == FIRE_STATE_OUT:
            reignite_probability = FIRE_REIGNITE_PROBABILITY_PER_SECOND * dt
            if random.random() < reignite_probability:
                hotspot.suppression_state = FIRE_STATE_ACTIVE
                hotspot.detected = False
                hotspot.detection_age_seconds = 0.0
                hotspot.burn_age_seconds = 0.0
                hotspot.mapping_progress_seconds = 0.0
                hotspot.suppression_work_seconds = 0.0
                hotspot.water_drone_requested = False
                hotspot.water_drone_assigned = False
                hotspot.water_drone_engaged = False
                hotspot.water_drone_assignment_count = 0
                hotspot.water_drone_engagement_count = 0
                hotspot.ground_firefighter_engaged = False
                hotspot.current_suppression_rate_per_second = 0.0

        if (
            hotspot.suppression_state != previous_state
            or (
                hotspot.suppression_state == FIRE_STATE_ACTIVE
                and not hotspot.detected
            )
        ):
            set_hotspot_visual(hotspot)


def get_fire_ignition_schedule():
    """Ignition times for this run. 4-drone runs get one fire at t = 15 s;
    6-drone runs get a second random fire at t = 75 s (1:15)."""
    schedule = [FIRE_IGNITION_DELAY_SECONDS]
    if get_fire_source_count_for_current_team() >= 2:
        schedule.append(SECOND_FIRE_IGNITION_DELAY_SECONDS)
    return schedule[:get_fire_source_count_for_current_team()]


def get_fire_source_count_for_current_team():
    """Maximum number of scheduled ignition sources for the active team size."""
    if should_schedule_second_fire_for_current_team():
        return FIRE_SOURCE_COUNT_SIX_DRONE_CASE
    return FIRE_SOURCE_COUNT_FOUR_DRONE_CASE


def should_schedule_second_fire_for_current_team():
    return (
        team_total_drone_count >= SECOND_FIRE_MIN_TEAM_SIZE
        and get_team_survey_drone_count() >= 3
        and team_water_drone_count >= 3
    )


def _float_is_invalid(value):
    if value is None:
        return False
    return value != value or value == float("inf") or value == float("-inf")


def sanitize_drone_motion_state(label, node, drone_motion_state):
    """Self-heal a corrupted (NaN/inf) motion state.

    A single NaN anywhere in the physics freezes that drone permanently and
    no input can recover it. Detect it, log it, and reset the drone in place.
    """
    if node is None or node.isEmpty():
        return False
    suspect_values = (
        node.getX(), node.getY(), node.getZ(),
        drone_motion_state.velocity.x,
        drone_motion_state.velocity.y,
        drone_motion_state.velocity.z,
        drone_motion_state.pitch_radians,
        drone_motion_state.roll_radians,
        drone_motion_state.yaw_radians,
        drone_motion_state.heading_hold_radians,
        drone_motion_state.pitch_rate_rad_per_second,
        drone_motion_state.roll_rate_rad_per_second,
        drone_motion_state.yaw_rate_rad_per_second,
        drone_motion_state.target_altitude_meters,
        *drone_motion_state.rotor_omegas_rad_per_second,
    )
    if not any(_float_is_invalid(value) for value in suspect_values):
        return False
    print(
        f"[real-sim] WARNING: invalid motion state on {label} at "
        f"t={motion_state.sim_time_seconds:.1f}s, resetting that drone."
    )
    if (
        _float_is_invalid(node.getX())
        or _float_is_invalid(node.getY())
        or _float_is_invalid(node.getZ())
    ):
        recover_x, recover_y = get_fixed_drone_spawn_xy("survey", 1)
        node.setPos(
            recover_x,
            recover_y,
            compute_safe_altitude_at(
                recover_x,
                recover_y,
                AUTOMATION_CRUISE_ALTITUDE_METERS,
                AUTOMATION_MIN_ALTITUDE_ABOVE_GROUND_METERS,
            ),
        )
    initialize_drone_motion_state_from_pose(node, drone_motion_state)
    return True


def sanitize_all_drone_motion_states():
    sanitize_drone_motion_state("Survey drone 1", drone, motion_state)
    sanitize_drone_motion_state("Water drone 1", water_drone, water_motion_state)
    for follower in follower_drones:
        label = (
            f"{'Survey' if follower.get('role') == 'survey' else 'Water'}"
            f" drone {follower.get('slot', 0) + 1}"
        )
        sanitize_drone_motion_state(label, follower.get("root"), follower["motion_state"])


def maybe_ignite_initial_fire():
    """Ignite scheduled random fire sources at their planned times."""
    global fire_hotspots, fire_ignitions_done, incident_ignition_time_seconds

    if fire_ignitions_done >= get_fire_source_count_for_current_team():
        return
    schedule = get_fire_ignition_schedule()
    if fire_ignitions_done >= len(schedule):
        return
    if motion_state.sim_time_seconds < schedule[fire_ignitions_done]:
        return
    fire_ignitions_done += 1
    new_hotspots = spawn_fire_hotspots(FIRE_HOTSPOT_INITIAL_COUNT)
    for hotspot in new_hotspots:
        hotspot.fire_event_id = fire_ignitions_done
    fire_hotspots.extend(new_hotspots)
    if fire_ignitions_done == 1:
        incident_ignition_time_seconds = motion_state.sim_time_seconds


def update_hotspot_spread(dt):
    global fire_spread_timer_seconds

    if not FIRE_SPREAD_ENABLED:
        return
    if burn_ratio() >= 1.0:
        return
    if not fire_hotspots:
        return

    fire_spread_timer_seconds += dt
    if fire_spread_timer_seconds < FIRE_SPREAD_CHECK_INTERVAL_SECONDS:
        return
    fire_spread_timer_seconds = 0.0

    unburned_burnable_cells = fire_burnable_cells.difference(fire_burned_cells)
    if not unburned_burnable_cells:
        return
    burnable_total = max(1, len(fire_burnable_cells))
    ignition_elapsed_seconds = max(
        0.0,
        motion_state.sim_time_seconds - incident_ignition_time_seconds,
    )
    burn_budget_fraction = clamp(
        ignition_elapsed_seconds / max(1.0, FIRE_FULL_MAP_TARGET_SECONDS),
        0.0,
        1.0,
    )
    target_burned_cell_count = min(
        burnable_total,
        int(ceil(burnable_total * burn_budget_fraction)),
    )
    spread_cell_budget = target_burned_cell_count - len(fire_burned_cells)
    if spread_cell_budget <= 0:
        return
    def try_spawn_at(candidate_x, candidate_y, parent_fire_event_id=1):
        candidate_cell = world_to_fire_cell(candidate_x, candidate_y)
        if candidate_cell not in fire_burnable_cells:
            return False
        if candidate_cell in fire_burned_cells:
            return False

        if (
            candidate_x < SPAWN_X_MIN
            or candidate_x > SPAWN_X_MAX
            or candidate_y < SPAWN_Y_MIN
            or candidate_y > SPAWN_Y_MAX
        ):
            return False

        if not FIRE_SPREAD_IGNORE_TREE_CLEARANCE:
            if not _is_position_clear_of_trees(
                candidate_x,
                candidate_y,
                FIRE_SPREAD_TREE_CLEARANCE_METERS,
            ):
                return False

        ground_sample = sample_ground(candidate_x, candidate_y)
        if ground_sample is None:
            return False

        child_hotspot = build_fire_hotspot_at(
            candidate_x,
            candidate_y,
            ground_sample[0].z,
        )
        child_hotspot.fire_event_id = parent_fire_event_id
        fire_hotspots.append(child_hotspot)
        trim_active_fire_hotspots()
        return True

    def try_spawn_in_neighbor_cell(source_x, source_y, parent_fire_event_id=1):
        source_cell_x, source_cell_y = world_to_fire_cell(source_x, source_y)
        max_neighbor_radius = max(
            1,
            int(
                ceil(
                    FIRE_SPREAD_MAX_DISTANCE_METERS
                    / max(1.0, FIRE_BURN_CELL_SIZE_METERS)
                )
            )
            + 1,
        )
        for radius in range(1, max_neighbor_radius + 1):
            candidate_cells = []
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if max(abs(dx), abs(dy)) != radius:
                        continue
                    candidate_cell = (source_cell_x + dx, source_cell_y + dy)
                    if candidate_cell not in fire_burnable_cells:
                        continue
                    if candidate_cell in fire_burned_cells:
                        continue
                    candidate_cells.append(candidate_cell)
            random.shuffle(candidate_cells)
            for candidate_cell in candidate_cells[:FIRE_SPREAD_FALLBACK_RANDOM_ATTEMPTS]:
                candidate_x, candidate_y = fire_cell_to_world(*candidate_cell)
                if try_spawn_at(candidate_x, candidate_y, parent_fire_event_id):
                    return True
        return False

    spread_event_count = FIRE_SPREAD_EVENTS_PER_TICK
    if random.random() < FIRE_SPREAD_EXTRA_EVENT_PROBABILITY:
        spread_event_count += 1
    spread_event_count = min(spread_event_count, spread_cell_budget)

    for _ in range(spread_event_count):
        # Containment-as-a-race (Fried & Fried 1996; Finney & Zimmer 2025):
        # suppression works by progressively stopping spread along the
        # perimeter. A hotspot with crews or a water drone engaged is behind
        # a fireline and no longer spreads; only unworked ACTIVE spots do.
        spread_sources = [
            hotspot
            for hotspot in fire_hotspots
            if hotspot.suppression_state == FIRE_STATE_ACTIVE
            and not hotspot.root.isEmpty()
            and (
                hotspot.current_suppression_rate_per_second <= 0.0
                # Worked spots are mostly held behind the line, but still flare
                # up occasionally so the fire keeps growing (Cheryl June 14:
                # "bring back the original spread" -> more fire to fight).
                or random.random() < FIRE_WORKED_SPOT_SPREAD_PROBABILITY
            )
        ]
        if not spread_sources:
            return

        source_hotspot = random.choice(spread_sources)
        if source_hotspot.root.isEmpty():
            continue
        source_x = source_hotspot.root.getX()
        source_y = source_hotspot.root.getY()

        grew_this_event = False
        for _ in range(FIRE_SPREAD_SPAWN_ATTEMPTS):
            spread_angle = random.uniform(0.0, tau)
            spread_distance = random.uniform(
                FIRE_SPREAD_MIN_DISTANCE_METERS,
                FIRE_SPREAD_MAX_DISTANCE_METERS,
            )

            candidate_x = source_x + cos(spread_angle) * spread_distance
            candidate_y = source_y + sin(spread_angle) * spread_distance
            if try_spawn_at(
                candidate_x,
                candidate_y,
                getattr(source_hotspot, "fire_event_id", 1),
            ):
                grew_this_event = True
                break

        if grew_this_event:
            if burn_ratio() >= 1.0:
                return
            unburned_burnable_cells = fire_burnable_cells.difference(fire_burned_cells)
            if not unburned_burnable_cells:
                return
            continue

        # Contiguous fallback: if random radial attempts all land in already
        # burned cells, step outward through neighboring unburned cells instead
        # of silently stalling the fire front.
        if try_spawn_in_neighbor_cell(
            source_x,
            source_y,
            getattr(source_hotspot, "fire_event_id", 1),
        ):
            if burn_ratio() >= 1.0:
                return
            continue
        continue


def estimate_full_burn_eta_seconds(current_time_seconds):
    global projected_full_burn_eta_seconds

    current_burn_ratio = burn_ratio()
    burn_ratio_history.append((current_time_seconds, current_burn_ratio))

    if current_burn_ratio >= 1.0:
        projected_full_burn_eta_seconds = 0.0
        return

    recent_samples = [
        sample
        for sample in burn_ratio_history
        if (current_time_seconds - sample[0]) <= 30.0
    ]
    if len(recent_samples) < 2:
        projected_full_burn_eta_seconds = None
        return

    first_sample_time, first_sample_ratio = recent_samples[0]
    latest_sample_time, latest_sample_ratio = recent_samples[-1]
    delta_time = latest_sample_time - first_sample_time
    delta_ratio = latest_sample_ratio - first_sample_ratio

    if delta_time <= 0.5 or delta_ratio <= 0.0002:
        projected_full_burn_eta_seconds = None
        return

    growth_per_second = delta_ratio / delta_time
    projected_full_burn_eta_seconds = max(
        0.0,
        (1.0 - current_burn_ratio) / growth_per_second,
    )


def format_sim_duration(seconds):
    if seconds is None:
        return "--"
    if seconds < 60.0:
        return f"{seconds:.0f}s"
    if seconds < 3600.0:
        return f"{seconds / 60.0:.1f}m"
    return f"{seconds / 3600.0:.1f}h"


def format_real_duration_from_sim_seconds(sim_seconds):
    if sim_seconds is None:
        return "--"
    real_minutes = sim_seconds * FIRE_SIM_REAL_MINUTES_PER_SECOND
    if real_minutes < 60.0:
        return f"{real_minutes:.0f}m"
    if real_minutes < 1440.0:
        return f"{real_minutes / 60.0:.1f}h"
    return f"{real_minutes / 1440.0:.1f}d"


def format_round_clock(seconds):
    total_seconds = max(0, int(seconds))
    minutes = total_seconds // 60
    remaining_seconds = total_seconds % 60
    return f"{minutes}:{remaining_seconds:02d}"


def format_selected_drone_status():
    # Shows which drone the number keys selected and that drone's OWN mode,
    # so the operator can see M/O acting on the currently viewed drone.
    view = get_selected_drone_view()
    if view is None:
        return "VIEW: (none)"
    index_label = metrics_short_label(view)
    if is_view_damaged(view):
        damage_reason = format_drone_damage_reason(view["role"], view["slot"])
        return f"VIEW: {index_label}  {view['label']}  [OFFLINE - {damage_reason}]"
    mode = get_drone_view_control_mode(view)
    mode_label = "MANUAL" if mode == CONTROL_MODE_MANUAL else "AUTO"
    tank_label = ""
    if view["role"] == "water":
        water_tank_label = format_water_tank_label_for_view(view)
        if water_tank_label == "EMPTY":
            tank_label = "  [EMPTY]"
        else:
            tank_label = f"  [Water {water_tank_label}]"
    spray_label = ""
    if (
        view["role"] == "water"
        and mode == CONTROL_MODE_MANUAL
        and is_water_spray_active(view["slot"])
    ):
        spray_label = "  [SPRAY ON]"
    return f"VIEW: {index_label}  {view['label']}  [{mode_label}]{tank_label}{spray_label}"


def update_status_overlay():
    burned_percent = burn_ratio() * 100.0
    pending_dispatch_hotspot = choose_pending_water_dispatch_hotspot()
    if pending_dispatch_hotspot is None and dispatch_prompt_cycle_active:
        pending_dispatch_hotspot = get_alert_dispatch_hotspot()
    team_state.pending_water_dispatch_hotspot = pending_dispatch_hotspot
    performance = compute_performance_matrix(motion_state.sim_time_seconds)
    round_time_remaining_seconds = max(
        0.0,
        FIRE_TARGET_GAME_DURATION_SECONDS - motion_state.sim_time_seconds,
    )

    status_text.setText(
        f"Round Time: {format_round_clock(round_time_remaining_seconds)}/{format_round_clock(FIRE_TARGET_GAME_DURATION_SECONDS)} | Burned total: {burned_percent:.0f}%\n"
        f"{format_team_ratio_menu()}\n"
        f"{format_selected_drone_status()}\n"
        f"{format_speed_mode_menu()}\n"
    )
    update_water_tank_status_overlay()
    update_automation_message_overlay()
    update_operator_performance_overlay(performance)


def update(task):
    global camera_angle, camera_pitch, last_mouse_x, last_mouse_y
    global overview_camera_angle, overview_camera_pitch
    global camera_manual_override_active, camera_yaw_offset_degrees
    global game_over_triggered, game_end_reason
    global fire_effect_update_accumulator_seconds, fire_map_update_accumulator_seconds
    global dispatch_alert_update_accumulator_seconds, status_update_accumulator_seconds
    global burn_eta_update_accumulator_seconds
    global pregame_update_accumulator_seconds
    global headless_frame_counter

    dt = ClockObject.getGlobalClock().getDt()
    if REAL_SIM_HEADLESS_FIXED_DT_SECONDS > 0.0:
        dt = REAL_SIM_HEADLESS_FIXED_DT_SECONDS
    if dt <= 0:
        dt = 1.0 / 60.0
    dt = min(dt, 0.05)  # Avoid giant movement jumps on temporary stalls.
    if REAL_SIM_HEADLESS_FRAME_LIMIT:
        headless_frame_counter += 1
        if headless_frame_counter >= REAL_SIM_HEADLESS_FRAME_LIMIT:
            if REAL_SIM_HEADLESS_SUMMARY:
                total_count, detected_count, extinguished_count = count_fire_totals()
                print(
                    "REAL_SIM_HEADLESS_SUMMARY "
                    f"sim_time={motion_state.sim_time_seconds:.1f} "
                    f"fire={total_count} detected={detected_count} "
                    f"out={extinguished_count}"
                )
            app.userExit()
            return task.done

    if pregame_active:
        refresh_tx12_controller_device()
        clear_water_hud_target_reticle()
        # Freeze the sim on the mission-setup screen; show a static overview.
        pregame_update_accumulator_seconds += dt
        if pregame_update_accumulator_seconds >= PREGAME_UPDATE_INTERVAL_SECONDS:
            update_pregame_overlay()
            pregame_update_accumulator_seconds = 0.0
        if app.camera is not None:
            app.camera.setPos(0.0, -95.0, 72.0)
            app.camera.lookAt(0.0, 0.0, 0.0)
        pace_main_loop()
        return task.cont

    if game_over_triggered:
        clear_water_hud_target_reticle()
        # Round finished: freeze the sim while the results page is shown.
        pace_main_loop()
        return task.cont

    update_wind(dt)
    sanitize_all_drone_motion_states()
    record_view_dwell_time(dt)
    accumulate_drone_mode_time(dt)
    update_tx12_controller_input(dt)
    if hover_all_active and raw_keyboard_movement_command_active():
        set_hover_all(False)

    selected_node = get_camera_target_node()
    survey_lead_selected = selected_node is drone
    water_lead_selected = selected_node is water_drone
    survey_lead_manual_input_active = (
        control_mode == CONTROL_MODE_MANUAL
        and survey_lead_selected
    )
    survey_lead_automation_active = control_mode == CONTROL_MODE_AUTOMATION
    if is_drone_damaged("survey", 1):
        # Offline: no input and no automation; the drone just hovers dead.
        survey_lead_manual_input_active = False
        survey_lead_automation_active = False

    motion_state.sim_time_seconds += dt
    current_speed_units_per_second = get_current_drone_speed_units_per_second()
    horizontal_speed_limit = current_speed_units_per_second
    vertical_speed_limit = current_speed_units_per_second
    tilt_limit_radians = get_current_max_tilt_radians()
    if survey_lead_automation_active:
        desired_world_vx, desired_world_vy, desired_world_vz = compute_automation_velocity(dt)
        horizontal_speed_limit *= AUTOMATION_SPEED_FACTOR
        vertical_speed_limit *= AUTOMATION_VERTICAL_SPEED_FACTOR
    elif survey_lead_manual_input_active:
        manual_speed_units_per_second = get_manual_drone_speed_units_per_second()
        horizontal_speed_limit = manual_speed_units_per_second
        vertical_speed_limit = manual_speed_units_per_second
        tilt_limit_radians = get_manual_max_tilt_radians()
        desired_world_vx, desired_world_vy, desired_world_vz = compute_manual_desired_velocity(
            camera_angle,
            key_map,
            manual_speed_units_per_second,
        )
    else:
        desired_world_vx, desired_world_vy, desired_world_vz = 0.0, 0.0, 0.0
    control_lag_seconds = 0.0
    motion_state.input_command_buffer.append(
        (
            motion_state.sim_time_seconds + control_lag_seconds,
            desired_world_vx,
            desired_world_vy,
            desired_world_vz,
        )
    )

    while (
        motion_state.input_command_buffer
        and motion_state.input_command_buffer[0][0] <= motion_state.sim_time_seconds
    ):
        _, motion_state.applied_cmd.x, motion_state.applied_cmd.y, motion_state.applied_cmd.z = (
            motion_state.input_command_buffer.popleft()
        )

    wind_vx, wind_vy, _, _ = get_wind_velocity()
    wind_command_blend = (
        DRONE_WIND_COMMAND_BLEND_AUTOMATION
        if survey_lead_automation_active
        else DRONE_WIND_COMMAND_BLEND_MANUAL
    )
    commanded_vx = motion_state.applied_cmd.x + wind_vx * wind_command_blend
    commanded_vy = motion_state.applied_cmd.y + wind_vy * wind_command_blend

    manual_vertical_command_active = (
        survey_lead_manual_input_active
        and abs(motion_state.applied_cmd.z) > 0.001
    )

    if motion_state.target_altitude_meters is None:
        motion_state.target_altitude_meters = drone.getZ()
    elif motion_state.vertical_command_was_active and not manual_vertical_command_active:
        # When manual climb/descent input is released, hold the current altitude
        # instead of continuing to chase the older accumulated target.
        motion_state.target_altitude_meters = drone.getZ()

    motion_state.target_altitude_meters += motion_state.applied_cmd.z * dt
    motion_state.vertical_command_was_active = manual_vertical_command_active

    drone_ground = sample_ground(drone.getX(), drone.getY())
    if drone_ground is not None:
        min_hold_altitude = drone_ground[0].z + 1.6
        motion_state.target_altitude_meters = max(
            min_hold_altitude,
            motion_state.target_altitude_meters,
        )

    body_right, body_forward = body_axes_from_yaw(motion_state.yaw_radians)
    desired_body_right_speed = commanded_vx * body_right[0] + commanded_vy * body_right[1]
    desired_body_forward_speed = commanded_vx * body_forward[0] + commanded_vy * body_forward[1]
    current_body_right_speed = (
        motion_state.velocity.x * body_right[0]
        + motion_state.velocity.y * body_right[1]
    )
    current_body_forward_speed = (
        motion_state.velocity.x * body_forward[0]
        + motion_state.velocity.y * body_forward[1]
    )

    desired_body_right_accel = clamp(
        (desired_body_right_speed - current_body_right_speed) * DRONE_HORIZONTAL_VELOCITY_GAIN,
        -DRONE_MAX_HORIZONTAL_ACCEL_METERS_PER_SECOND_SQ,
        DRONE_MAX_HORIZONTAL_ACCEL_METERS_PER_SECOND_SQ,
    )
    desired_body_forward_accel = clamp(
        (desired_body_forward_speed - current_body_forward_speed) * DRONE_HORIZONTAL_VELOCITY_GAIN,
        -DRONE_MAX_HORIZONTAL_ACCEL_METERS_PER_SECOND_SQ,
        DRONE_MAX_HORIZONTAL_ACCEL_METERS_PER_SECOND_SQ,
    )

    desired_roll_radians = clamp(
        desired_body_right_accel / DRONE_GRAVITY_METERS_PER_SECOND_SQ,
        -tilt_limit_radians,
        tilt_limit_radians,
    )
    desired_pitch_radians = clamp(
        -desired_body_forward_accel / DRONE_GRAVITY_METERS_PER_SECOND_SQ,
        -tilt_limit_radians,
        tilt_limit_radians,
    )

    manual_horizontal_intent = (
        motion_state.applied_cmd.x * motion_state.applied_cmd.x
        + motion_state.applied_cmd.y * motion_state.applied_cmd.y
    ) ** 0.5
    if (
        survey_lead_manual_input_active
        and manual_horizontal_intent > DRONE_MANUAL_YAW_INPUT_SPEED_METERS_PER_SECOND
    ):
        set_drone_heading_hold_degrees(motion_state, camera_angle)
    target_yaw_radians = get_drone_heading_hold_radians(motion_state)

    altitude_error = drone.getZ() - motion_state.target_altitude_meters
    desired_vertical_rate = clamp(
        -DRONE_HOVER_GUIDANCE_GAIN * bounded_guidance(altitude_error),
        -vertical_speed_limit,
        vertical_speed_limit,
    )
    desired_vertical_accel = clamp(
        (desired_vertical_rate - motion_state.velocity.z) * DRONE_HOVER_VERTICAL_RATE_GAIN,
        -DRONE_MAX_VERTICAL_ACCEL_METERS_PER_SECOND_SQ,
        DRONE_MAX_VERTICAL_ACCEL_METERS_PER_SECOND_SQ,
    )
    lift_projection = max(
        0.3,
        cos(motion_state.pitch_radians) * cos(motion_state.roll_radians),
    )
    collective_thrust_command = clamp(
        (
            motion_state.mass_kg
            * (DRONE_GRAVITY_METERS_PER_SECOND_SQ + desired_vertical_accel)
        ) / lift_projection,
        motion_state.min_collective_thrust_newtons,
        motion_state.max_collective_thrust_newtons,
    )

    pitch_torque_command = clamp(
        motion_state.inertia_xx_kg_m2
        * (
            DRONE_PITCH_ANGLE_P_GAIN * (desired_pitch_radians - motion_state.pitch_radians)
            - DRONE_PITCH_RATE_D_GAIN * motion_state.pitch_rate_rad_per_second
        ),
        -DRONE_MAX_PITCH_TORQUE_NM,
        DRONE_MAX_PITCH_TORQUE_NM,
    )
    roll_torque_command = clamp(
        motion_state.inertia_yy_kg_m2
        * (
            DRONE_ROLL_ANGLE_P_GAIN * (desired_roll_radians - motion_state.roll_radians)
            - DRONE_ROLL_RATE_D_GAIN * motion_state.roll_rate_rad_per_second
        ),
        -DRONE_MAX_ROLL_TORQUE_NM,
        DRONE_MAX_ROLL_TORQUE_NM,
    )
    yaw_error_radians = wrap_angle_radians(target_yaw_radians - motion_state.yaw_radians)
    if (
        abs(yaw_error_radians) <= DRONE_YAW_HOLD_DEADBAND_RADIANS
        and abs(motion_state.yaw_rate_rad_per_second)
        <= DRONE_YAW_RATE_SETTLE_RADIANS_PER_SECOND
    ):
        motion_state.yaw_radians = target_yaw_radians
        motion_state.yaw_rate_rad_per_second = 0.0
        yaw_error_radians = 0.0
    yaw_torque_command = clamp(
        motion_state.inertia_zz_kg_m2
        * (
            DRONE_YAW_ANGLE_P_GAIN * yaw_error_radians
            - DRONE_YAW_RATE_D_GAIN * motion_state.yaw_rate_rad_per_second
        ),
        -DRONE_MAX_YAW_TORQUE_NM,
        DRONE_MAX_YAW_TORQUE_NM,
    )

    desired_rotor_thrusts = mix_controls_to_rotor_thrusts(
        collective_thrust_command,
        pitch_torque_command,
        roll_torque_command,
        yaw_torque_command,
    )
    actual_rotor_thrusts = update_rotor_dynamics(desired_rotor_thrusts, dt)
    (
        collective_thrust_newtons,
        pitch_torque_nm,
        roll_torque_nm,
        yaw_torque_nm,
    ) = rotor_thrusts_to_control_inputs(actual_rotor_thrusts)

    pitch_accel = (
        (
            (motion_state.inertia_yy_kg_m2 - motion_state.inertia_zz_kg_m2)
            / motion_state.inertia_xx_kg_m2
        )
        * motion_state.roll_rate_rad_per_second
        * motion_state.yaw_rate_rad_per_second
    ) + (pitch_torque_nm / motion_state.inertia_xx_kg_m2)
    roll_accel = (
        (
            (motion_state.inertia_zz_kg_m2 - motion_state.inertia_xx_kg_m2)
            / motion_state.inertia_yy_kg_m2
        )
        * motion_state.pitch_rate_rad_per_second
        * motion_state.yaw_rate_rad_per_second
    ) + (roll_torque_nm / motion_state.inertia_yy_kg_m2)
    yaw_accel = yaw_torque_nm / motion_state.inertia_zz_kg_m2

    motion_state.pitch_rate_rad_per_second += pitch_accel * dt
    motion_state.roll_rate_rad_per_second += roll_accel * dt
    motion_state.yaw_rate_rad_per_second += yaw_accel * dt

    motion_state.pitch_radians = clamp(
        motion_state.pitch_radians + motion_state.pitch_rate_rad_per_second * dt,
        -tilt_limit_radians,
        tilt_limit_radians,
    )
    motion_state.roll_radians = clamp(
        motion_state.roll_radians + motion_state.roll_rate_rad_per_second * dt,
        -tilt_limit_radians,
        tilt_limit_radians,
    )
    motion_state.yaw_radians = wrap_angle_radians(
        motion_state.yaw_radians + motion_state.yaw_rate_rad_per_second * dt
    )

    cos_yaw = cos(motion_state.yaw_radians)
    sin_yaw = sin(motion_state.yaw_radians)
    cos_pitch = cos(motion_state.pitch_radians)
    sin_pitch = sin(motion_state.pitch_radians)
    cos_roll = cos(motion_state.roll_radians)
    sin_roll = sin(motion_state.roll_radians)
    thrust_accel = collective_thrust_newtons / motion_state.mass_kg

    world_accel_x = thrust_accel * (
        (cos_yaw * sin_roll) + (sin_yaw * sin_pitch * cos_roll)
    )
    world_accel_y = thrust_accel * (
        (sin_yaw * sin_roll) - (cos_yaw * sin_pitch * cos_roll)
    )
    world_accel_z = thrust_accel * cos_pitch * cos_roll - DRONE_GRAVITY_METERS_PER_SECOND_SQ

    command_horizontal_intent = (
        motion_state.applied_cmd.x * motion_state.applied_cmd.x
        + motion_state.applied_cmd.y * motion_state.applied_cmd.y
    ) ** 0.5
    brake_factor = clamp(
        1.0 - (command_horizontal_intent / max(0.001, horizontal_speed_limit)),
        0.0,
        1.0,
    )
    horizontal_damping_rate = (
        DRONE_HORIZONTAL_BASE_DAMPING_PER_SECOND
        + DRONE_HORIZONTAL_BRAKE_DAMPING_PER_SECOND * brake_factor
    )
    vertical_command_intent = abs(motion_state.applied_cmd.z)
    vertical_brake_factor = clamp(
        1.0 - (vertical_command_intent / max(0.001, vertical_speed_limit)),
        0.0,
        1.0,
    )
    vertical_damping_rate = (
        DRONE_VERTICAL_ACTIVE_DRAG_PER_SECOND
        + DRONE_VERTICAL_PASSIVE_DRAG_PER_SECOND * vertical_brake_factor
    )
    world_accel_x -= motion_state.velocity.x * horizontal_damping_rate
    world_accel_y -= motion_state.velocity.y * horizontal_damping_rate
    world_accel_z -= motion_state.velocity.z * vertical_damping_rate

    motion_state.velocity.x += world_accel_x * dt
    motion_state.velocity.y += world_accel_y * dt
    motion_state.velocity.z += world_accel_z * dt

    horizontal_speed = (
        motion_state.velocity.x * motion_state.velocity.x
        + motion_state.velocity.y * motion_state.velocity.y
    ) ** 0.5
    max_horizontal_speed = horizontal_speed_limit * 1.15
    if horizontal_speed > max_horizontal_speed and horizontal_speed > 0.001:
        horizontal_speed_scale = max_horizontal_speed / horizontal_speed
        motion_state.velocity.x *= horizontal_speed_scale
        motion_state.velocity.y *= horizontal_speed_scale
        horizontal_speed = max_horizontal_speed
    if (
        brake_factor > 0.92
        and horizontal_speed < DRONE_HORIZONTAL_STOP_SETTLE_SPEED_METERS_PER_SECOND
    ):
        motion_state.velocity.x = 0.0
        motion_state.velocity.y = 0.0
        horizontal_speed = 0.0
    motion_state.velocity.z = clamp(
        motion_state.velocity.z,
        -vertical_speed_limit * 1.2,
        vertical_speed_limit * 1.2,
    )
    if (
        vertical_brake_factor > 0.92
        and abs(motion_state.velocity.z) < DRONE_VERTICAL_STOP_SETTLE_SPEED_METERS_PER_SECOND
        and abs(drone.getZ() - motion_state.target_altitude_meters)
        < DRONE_VERTICAL_STOP_SETTLE_ALTITUDE_METERS
    ):
        motion_state.velocity.z = 0.0
        motion_state.target_altitude_meters = drone.getZ()

    drone.setX(drone.getX() + motion_state.velocity.x * dt)
    drone.setY(drone.getY() + motion_state.velocity.y * dt)
    drone.setZ(drone.getZ() + motion_state.velocity.z * dt)

    drone.setH(degrees(motion_state.yaw_radians))
    motion_state.visual_pitch = degrees(motion_state.pitch_radians)
    motion_state.visual_roll = degrees(motion_state.roll_radians)
    drone_visual.setP(motion_state.visual_pitch)
    drone_visual.setR(motion_state.visual_roll)

    average_rotor_omega = sum(motion_state.rotor_omegas_rad_per_second) / max(
        1,
        len(motion_state.rotor_omegas_rad_per_second),
    )
    propeller_spin_speed = min(
        DRONE_PROPELLER_SPIN_MAX_DEGREES_PER_SECOND,
        DRONE_PROPELLER_SPIN_BASE_DEGREES_PER_SECOND + (average_rotor_omega * 2.35),
    )
    motion_state.propeller_spin_degrees = (
        motion_state.propeller_spin_degrees + propeller_spin_speed * dt
    ) % 360.0
    for rotor_node, spin_direction in drone_propellers:
        rotor_node.setR(motion_state.propeller_spin_degrees * spin_direction)

    edge_margin = 4
    x_before_clamp = drone.getX()
    y_before_clamp = drone.getY()

    clamped_x = max(SPAWN_X_MIN + edge_margin, min(SPAWN_X_MAX - edge_margin, x_before_clamp))
    clamped_y = max(SPAWN_Y_MIN + edge_margin, min(SPAWN_Y_MAX - edge_margin, y_before_clamp))

    if clamped_x != x_before_clamp:
        motion_state.velocity.x = 0.0
    if clamped_y != y_before_clamp:
        motion_state.velocity.y = 0.0

    drone.setX(clamped_x)
    drone.setY(clamped_y)

    if ENFORCE_MIN_FLYING_ALTITUDE:
        #kinda adds a safe constrain of only move down when its above terrain
        drone_ground = sample_ground(drone.getX(), drone.getY())
        if drone_ground is not None:
            min_altitude = drone_ground[0].z + 1.6
            if drone.getZ() < min_altitude:
                drone.setZ(min_altitude)
                motion_state.velocity.z = max(0.0, motion_state.velocity.z)

    maybe_ignite_initial_fire()
    update_hotspot_spread(dt)
    update_hotspot_detection(dt)
    auto_request_water_support_for_detected_hotspots()
    update_survey_mapping_progress(dt)
    reset_hotspot_water_drone_contact()
    if is_drone_damaged("water", 1):
        # Offline: hovers dead, ignores input and automation.
        update_manual_water_drone(
            dt,
            current_speed_units_per_second,
            input_active=False,
        )
    elif water_control_mode == CONTROL_MODE_MANUAL:
        update_manual_water_drone(
            dt,
            current_speed_units_per_second,
            input_active=water_lead_selected,
        )
    else:
        update_water_drone(dt)
    update_follower_drones(dt)
    update_water_targeting_visuals()
    update_fire_truck_fleet(dt)
    enforce_inter_drone_separation()
    check_manual_collisions()
    check_environment_impacts()
    update_crashed_drones(dt)
    if hover_all_active:
        apply_hover_all_freeze()
    update_collision_alert_overlay()
    update_hotspot_lifecycle(dt)
    if overview_camera_enabled and overview_is_operator:
        # Keep undetected fire hidden in the operator satellite view; reveal a
        # fire the instant a drone detects it.
        refresh_operator_fire_visibility()
    fire_effect_update_accumulator_seconds += dt
    if fire_effect_update_accumulator_seconds >= FIRE_EFFECT_UPDATE_INTERVAL_SECONDS:
        update_hotspot_effect_animation(fire_effect_update_accumulator_seconds)
        fire_effect_update_accumulator_seconds = 0.0

    burn_eta_update_accumulator_seconds += dt
    if burn_eta_update_accumulator_seconds >= BURN_ETA_UPDATE_INTERVAL_SECONDS:
        estimate_full_burn_eta_seconds(motion_state.sim_time_seconds)
        burn_eta_update_accumulator_seconds = 0.0

    fire_map_update_accumulator_seconds += dt
    if fire_map_update_accumulator_seconds >= FIRE_MAP_UPDATE_INTERVAL_SECONDS:
        update_fire_map_popup()
        update_planned_trajectories()
        fire_map_update_accumulator_seconds = 0.0

    dispatch_alert_update_accumulator_seconds += dt
    if (
        dispatch_alert_update_accumulator_seconds
        >= DISPATCH_ALERT_UPDATE_INTERVAL_SECONDS
    ):
        update_dispatch_alert_overlay(dispatch_alert_update_accumulator_seconds)
        dispatch_alert_update_accumulator_seconds = 0.0

    if motion_state.sim_time_seconds >= FIRE_TARGET_GAME_DURATION_SECONDS:
        if not game_over_triggered:
            game_over_triggered = True
            game_end_reason = "burned_out" if burn_ratio() >= 1.0 else "time_limit"
            motion_state.input_command_buffer.clear()
            motion_state.applied_cmd = AxisState()
            motion_state.velocity = AxisState()
            motion_state.last_body_right_speed = 0.0
            motion_state.filtered_body_right_accel = 0.0
            motion_state.roll_sway_body_speed = 0.0
            motion_state.last_body_right_command_speed = 0.0
            motion_state.roll_anticipation_body_speed = 0.0
            update_status_overlay()
            show_results_page()
        pace_main_loop()
        return task.cont

    if dragging_camera and app.mouseWatcherNode.hasMouse():
        current_mouse_x = app.mouseWatcherNode.getMouseX()
        current_mouse_y = app.mouseWatcherNode.getMouseY()

        raw_mouse_delta_x = current_mouse_x - last_mouse_x
        raw_mouse_delta_y = current_mouse_y - last_mouse_y
        mouse_delta_x = max(-CAMERA_DRAG_DELTA_CLAMP, min(CAMERA_DRAG_DELTA_CLAMP, raw_mouse_delta_x))
        mouse_delta_y = max(-CAMERA_DRAG_DELTA_CLAMP, min(CAMERA_DRAG_DELTA_CLAMP, raw_mouse_delta_y))

        if overview_camera_enabled:
            overview_camera_angle += mouse_delta_x * CAMERA_DRAG_X_DEGREES_PER_UNIT
            overview_camera_pitch += (
                mouse_delta_y * OVERVIEW_CAMERA_DRAG_Y_DEGREES_PER_UNIT
            )
            overview_camera_pitch = clamp(
                overview_camera_pitch,
                OVERVIEW_CAMERA_MIN_PITCH_DEGREES,
                OVERVIEW_CAMERA_MAX_PITCH_DEGREES,
            )
        else:
            selected_view = get_selected_drone_view()
            selected_is_automated = (
                selected_view is not None
                and get_drone_view_control_mode(selected_view) == CONTROL_MODE_AUTOMATION
            )
            if abs(mouse_delta_x) > 0.0001 or abs(mouse_delta_y) > 0.0001:
                camera_manual_override_active = True
            yaw_delta = mouse_delta_x * CAMERA_DRAG_X_DEGREES_PER_UNIT
            if selected_is_automated:
                camera_yaw_offset_degrees = clamp(
                    camera_yaw_offset_degrees + yaw_delta,
                    -AUTOMATION_CAMERA_LOOK_CONE_HALF_ANGLE_DEGREES,
                    AUTOMATION_CAMERA_LOOK_CONE_HALF_ANGLE_DEGREES,
                )
            else:
                camera_angle = wrap_angle_degrees(camera_angle + yaw_delta)
            drag_y_gain = (
                FIRST_PERSON_DRAG_Y_DEGREES_PER_UNIT
                if first_person_view
                else THIRD_PERSON_DRAG_Y_DEGREES_PER_UNIT
            )
            camera_pitch += mouse_delta_y * drag_y_gain
            clamp_camera_pitch_for_current_mode()

        last_mouse_x = current_mouse_x
        last_mouse_y = current_mouse_y

    #all the following make sueres the camera follows the drone around,
    #instead of just fixed screen
    auto_align_camera_to_selected_drone(dt)
    camera_target_node = get_camera_target_node()
    if app.camera is not None:
        if overview_camera_enabled:
            focus_x, focus_y, focus_z, scene_span = get_overview_focus_point()
            # Pure manual zoom: the wheel ([wheel up]/[wheel down]) sets the distance
            # directly (it starts framed on the whole map via set_overhead_view).
            effective_distance = clamp(
                overview_camera_distance,
                OVERVIEW_CAMERA_MIN_DISTANCE,
                OVERVIEW_CAMERA_MAX_DISTANCE,
            )
            horizontal_distance = effective_distance * cos(radians(overview_camera_pitch))
            camera_x = focus_x + sin(radians(overview_camera_angle)) * horizontal_distance
            camera_y = focus_y - cos(radians(overview_camera_angle)) * horizontal_distance
            camera_z = focus_z + sin(radians(overview_camera_pitch)) * effective_distance

            app.camera.setPos(camera_x, camera_y, camera_z)
            app.camera.lookAt(Point3(focus_x, focus_y, focus_z))
        elif first_person_view:
            eye_x = camera_target_node.getX()
            eye_y = camera_target_node.getY()
            eye_z = camera_target_node.getZ() + first_person_eye_height

            yaw = radians(camera_angle)
            pitch = radians(camera_pitch)
            look_dir = Vec3(
                -sin(yaw) * cos(pitch),
                cos(yaw) * cos(pitch),
                sin(pitch),
            )
            eye_x, eye_y, eye_z = resolve_first_person_camera_eye(
                eye_x,
                eye_y,
                eye_z,
                look_dir,
            )

            app.camera.setPos(eye_x, eye_y, eye_z)
            app.camera.lookAt(Point3(eye_x, eye_y, eye_z) + look_dir * 30.0)
        else:
            horizontal_distance = camera_distance * cos(radians(camera_pitch))
            target_x = camera_target_node.getX()
            target_y = camera_target_node.getY()
            target_z = camera_target_node.getZ() + 0.8

            camera_x = target_x + sin(radians(camera_angle)) * horizontal_distance
            camera_y = target_y - cos(radians(camera_angle)) * horizontal_distance
            camera_z = camera_target_node.getZ() + camera_height + sin(radians(camera_pitch)) * camera_distance
            camera_x, camera_y, camera_z = resolve_third_person_camera_obstruction(
                target_x,
                target_y,
                target_z,
                camera_x,
                camera_y,
                camera_z,
            )

            app.camera.setPos(camera_x, camera_y, camera_z)
            app.camera.lookAt(camera_target_node)

    update_water_hud_target_reticle()

    status_update_accumulator_seconds += dt
    if status_update_accumulator_seconds >= STATUS_UPDATE_INTERVAL_SECONDS:
        update_status_overlay()
        status_update_accumulator_seconds = 0.0
    pace_main_loop()
    return task.cont


build_dispatch_alert_overlay()
build_collision_alert_overlay()
build_fire_map_overlay()
build_automation_message_overlay()
build_water_tank_status_overlay()
build_operator_performance_overlay()
build_controls_help_overlay()
build_mode_buttons_overlay()
build_pregame_overlay()
update_fire_map_popup()
update_status_overlay()
app.taskMgr.add(update, "update")
app.accept("mouse1", start_camera_drag)
app.accept("mouse1-up", stop_camera_drag)


def run_water_dispatch_self_test():
    set_team_total_drone_count(6)
    start_game_from_pregame()
    assert not pregame_active
    assert team_water_drone_count == 3
    for slot in range(1, team_water_drone_count + 1):
        view = get_water_view_by_slot(slot)
        assert view is not None
        assert get_drone_view_control_mode(view) == CONTROL_MODE_MANUAL
    for slot in range(2, get_team_survey_drone_count() + 1):
        view = get_drone_view_for_role_slot("survey", slot)
        assert view is not None
        assert get_drone_view_control_mode(view) == CONTROL_MODE_MANUAL
        assert not view["follower"].get("automation_started", False)

    fire_x, fire_y = get_survey_search_sector(3).center()
    ground_sample = sample_ground(fire_x, fire_y)
    fire_z = ground_sample[0].z if ground_sample is not None else 0.0
    hotspot = build_fire_hotspot_at(fire_x, fire_y, fire_z)
    hotspot.fire_event_id = 99
    fire_hotspots.append(hotspot)
    mark_hotspot_detected(
        hotspot,
        motion_state.sim_time_seconds,
        detected_by_survey_slot=3,
    )
    assert dispatch_prompt_cycle_active
    assert get_alert_dispatch_hotspot() is hotspot

    request_water_follower_dispatch(3)
    water_1_view = get_water_view_by_slot(1)
    water_2_view = get_water_view_by_slot(2)
    water_3_view = get_water_view_by_slot(3)
    assert get_drone_view_control_mode(water_1_view) == CONTROL_MODE_MANUAL
    assert get_drone_view_control_mode(water_2_view) == CONTROL_MODE_MANUAL
    assert get_drone_view_control_mode(water_3_view) == CONTROL_MODE_AUTOMATION
    assert water_dispatch_targets.get(3) is hotspot
    assert choose_paired_survey_water_hotspot(3, water_3_view["root"].getX(), water_3_view["root"].getY()) is hotspot
    assert len(get_water_route_world_points(water_3_view)) >= 2
    assert not get_water_route_world_points(water_1_view)
    reset_hotspot_water_drone_contact()
    update_water_follower_drone(water_3_view["follower"], 0.1)
    assert water_3_view["follower"]["target_hotspot"] is hotspot
    assert hotspot.water_drone_assigned
    update_water_targeting_visuals()
    assert water_target_reticle_root is not None
    assert water_target_reticle_root.node().getNumChildren() >= 1
    status_line = build_water_dispatch_status_line()
    assert "Water 1: MANUAL" in status_line
    assert "Water 2: MANUAL" in status_line
    assert "Water 3: AUTO + DISPATCHED" in status_line
    assert "water 40L" in status_line
    assert dispatch_prompt_cycle_active

    fire_2_x, fire_2_y = get_survey_search_sector(2).center()
    fire_2_ground_sample = sample_ground(fire_2_x, fire_2_y)
    fire_2_z = fire_2_ground_sample[0].z if fire_2_ground_sample is not None else 0.0
    hotspot_2 = build_fire_hotspot_at(fire_2_x, fire_2_y, fire_2_z)
    # Same fire_event_id as the S3 fire above: ownership must still follow the
    # survey that directly detected this hotspot, so W2 goes to S2's fire.
    hotspot_2.fire_event_id = 99
    fire_hotspots.append(hotspot_2)
    mark_hotspot_detected(
        hotspot_2,
        motion_state.sim_time_seconds,
        highlight_dispatch=False,
        trigger_alarm=False,
        detected_by_survey_slot=2,
    )
    survey_2_view = get_drone_view_for_role_slot("survey", 2)
    assert survey_2_view is not None
    set_drone_view_control_mode(survey_2_view, CONTROL_MODE_AUTOMATION)
    hotspot_2.mapping_progress_seconds = SURVEY_MAP_BUILD_TIME_SECONDS
    update_survey_follower_drone(survey_2_view["follower"], 0.1)
    assert survey_2_view["follower"]["target_hotspot"] is hotspot_2
    assert len(get_survey_route_world_points(survey_2_view)) >= 8
    set_drone_view_control_mode(water_2_view, CONTROL_MODE_AUTOMATION)
    dispatched_water_slots.add(2)
    water_dispatch_targets.pop(2, None)
    assert choose_paired_survey_water_hotspot(2, water_2_view["root"].getX(), water_2_view["root"].getY()) is hotspot_2
    reset_hotspot_water_drone_contact()
    update_water_follower_drone(water_2_view["follower"], 0.1)
    assert water_2_view["follower"]["target_hotspot"] is hotspot_2
    assert hotspot_2.water_drone_assigned
    set_drone_view_control_mode(water_2_view, CONTROL_MODE_MANUAL)
    dispatched_water_slots.discard(2)
    water_dispatch_targets.pop(2, None)
    water_2_view["follower"]["target_hotspot"] = None

    selected_before = camera_target_index
    set_camera_target_role_slot("water", 1)
    assert get_drone_view_control_mode(water_1_view) == CONTROL_MODE_MANUAL
    assert not is_water_spray_active(1)
    set_water_tank_liters(water_drone_state, 0.0)
    toggle_manual_water_spray()
    assert not is_water_spray_active(1)
    assert "[EMPTY]" in format_selected_drone_status()
    set_water_tank_liters(water_drone_state, WATER_DRONE_TANK_CAPACITY_LITERS)
    toggle_manual_water_spray()
    assert is_water_spray_active(1)
    starting_liters = get_water_tank_liters(water_drone_state)
    assert engage_water_suppression_if_possible(water_drone_state, True, 2.0)
    assert get_water_tank_liters(water_drone_state) < starting_liters
    manual_water_spray_slots.discard(1)
    set_water_tank_liters(water_drone_state, WATER_DRONE_TANK_CAPACITY_LITERS)
    water_1_manual_target = get_water_current_target_for_view(water_1_view)
    assert water_1_manual_target in (hotspot, hotspot_2)
    update_water_targeting_visuals()
    assert water_target_reticle_root is not None
    assert water_target_reticle_root.node().getNumChildren() >= 1
    if app.camera is not None and app.cam is not None:
        hud_target_point = Point3(
            water_1_manual_target.root.getX(),
            water_1_manual_target.root.getY(),
            water_1_manual_target.ground_z + WATER_DRONE_STREAM_IMPACT_HEIGHT_METERS,
        )
        app.camera.setPos(
            water_1_view["root"].getX(),
            water_1_view["root"].getY() - 24.0,
            water_1_view["root"].getZ() + 7.0,
        )
        app.camera.lookAt(hud_target_point)
        update_water_hud_target_reticle()
        assert water_hud_target_reticle_root is not None
        assert water_hud_target_reticle_root.node().getNumChildren() >= 1
    set_camera_target_index(selected_before)

    request_water_drone_dispatch()
    assert get_drone_view_control_mode(water_1_view) == CONTROL_MODE_AUTOMATION
    water_1_dispatch_target = water_dispatch_targets.get(1)
    assert water_1_dispatch_target is None
    assert get_drone_view_control_mode(water_2_view) == CONTROL_MODE_MANUAL
    reset_hotspot_water_drone_contact()
    update_water_drone(0.1)
    assert team_state.water_target_hotspot is None
    assert not hotspot.water_drone_assigned
    assert not hotspot_2.water_drone_assigned

    set_full_automation()
    set_hover_all(True)
    assert hover_all_active
    assert not water_drone_state.auto_dispatch_enabled
    assert not dispatched_water_slots
    assert not water_dispatch_targets
    for view in get_all_drone_views():
        if not is_view_damaged(view):
            assert get_drone_view_control_mode(view) == CONTROL_MODE_MANUAL

    print("REAL_SIM_SELF_TEST water_dispatch OK")
    app.userExit()


def run_fire_truck_self_test():
    start_game_from_pregame()
    reset_fire_truck_fleet()
    staging_margin = max(
        FIRE_TRUCK_STAGING_CORNER_MARGIN_METERS,
        FIRE_TRUCK_TREE_CLEARANCE_METERS * 0.5,
    )
    staging_max_x = (
        SPAWN_X_MIN
        + staging_margin
        + FIRE_TRUCK_STAGING_SPACING_METERS * max(1, min(2, FIRE_TRUCK_COUNT))
    )
    staging_max_y = (
        SPAWN_Y_MIN
        + staging_margin
        + FIRE_TRUCK_STAGING_SPACING_METERS * max(1, (FIRE_TRUCK_COUNT + 1) // 2)
    )
    staging_epsilon = 0.05
    assert all(
        SPAWN_X_MIN - staging_epsilon
        <= truck_state.home_x
        <= staging_max_x + staging_epsilon
        and SPAWN_Y_MIN - staging_epsilon
        <= truck_state.home_y
        <= staging_max_y + staging_epsilon
        for truck_state in fire_truck_states
    )
    hotspots = []
    for index, (fire_x, fire_y) in enumerate(((-28.0, -18.0), (0.0, 22.0), (28.0, -6.0)), start=1):
        ground_sample = sample_ground(fire_x, fire_y)
        fire_z = ground_sample[0].z if ground_sample is not None else 0.0
        hotspot = build_fire_hotspot_at(fire_x, fire_y, fire_z)
        hotspot.fire_event_id = 200 + index
        fire_hotspots.append(hotspot)
        mark_hotspot_detected(
            hotspot,
            motion_state.sim_time_seconds,
            highlight_dispatch=False,
            trigger_alarm=False,
            detected_by_survey_slot=1,
        )
        hotspots.append(hotspot)

    update_fire_truck_fleet(0.1)
    assigned_trucks = [
        truck_state
        for truck_state in fire_truck_states
        if truck_state.target_hotspot is not None
    ]
    assert len(assigned_trucks) == min(FIRE_TRUCK_COUNT, len(hotspots))
    assigned_hotspot_ids = {id(truck_state.target_hotspot) for truck_state in assigned_trucks}
    assert len(assigned_hotspot_ids) == len(assigned_trucks)
    for truck_state in assigned_trucks:
        assert truck_state.target_hotspot.fire_cluster_color == FIRE_TRUCK_SUPPRESSING_COLOR
        assert not truck_state.target_hotspot.scar_cross_a.isHidden()
        assert truck_state.target_hotspot.scar_cross_a.getColor()[2] >= 0.95

    first_truck = assigned_trucks[0]
    target_hotspot = first_truck.target_hotspot
    target_x, target_y = fire_truck_fireline_target_xy(first_truck, target_hotspot)
    first_truck.root.setX(target_x)
    first_truck.root.setY(target_y)
    update_fire_truck_fleet(0.1)
    assert first_truck.engaged
    previous_work = target_hotspot.suppression_work_seconds
    update_hotspot_lifecycle(1.0)
    assert target_hotspot.ground_firefighter_engaged
    assert target_hotspot.suppression_work_seconds > previous_work
    assert hotspot_has_fire_truck_response(target_hotspot)

    update_fire_map_popup()
    assert len(fire_map_truck_nodes) == len(fire_truck_states)
    map_node = fire_map_detected_hotspot_nodes.get(id(target_hotspot))
    assert map_node is not None
    fill_node = map_node.getPythonTag("firetruck_suppression_fill_node")
    assert fill_node is not None and not fill_node.isHidden()
    assert fill_node.getColor()[2] >= 0.95

    print("REAL_SIM_SELF_TEST fire_trucks OK")
    app.userExit()


def run_kill_switch_self_test():
    start_game_from_pregame()
    trigger_kill_switch()
    views = get_all_drone_views()
    assert views
    assert all(is_view_damaged(view) for view in views)
    assert all(
        get_drone_damage_reason(view["role"], view["slot"])
        == DAMAGE_REASON_KILL_SWITCH
        for view in views
    )
    assert collision_alert_text.getText() == "KILL SWITCH ACTIVATED"
    assert "Emergency stop" in collision_alert_body_text.getText()
    assert "collision" not in format_selected_drone_status().lower()
    assert "kill switch" in format_selected_drone_status().lower()
    print("REAL_SIM_SELF_TEST kill_switch OK")
    app.userExit()


def run_collision_damage_self_test():
    start_game_from_pregame()
    views = get_all_drone_views()
    assert len(views) >= 3
    for view in views:
        if not is_view_damaged(view):
            set_drone_view_control_mode(view, CONTROL_MODE_MANUAL)

    first, second, third = views[:3]
    for index, view in enumerate(views):
        view["root"].setPos(30.0 + index * 12.0, 30.0, 22.0)
        motion = get_view_crash_rig_parts(view)[2]
        motion.velocity = Vec3(0.0, 0.0, 0.0)
        motion.target_altitude_meters = view["root"].getZ()

    first["root"].setPos(-35.0, -35.0, 22.0)
    second["root"].setPos(-35.6, -35.0, 22.0)
    check_manual_collisions()
    assert is_view_damaged(first)
    assert is_view_damaged(second)
    assert get_drone_damage_reason(first["role"], first["slot"]) == DAMAGE_REASON_COLLISION
    assert get_drone_damage_reason(second["role"], second["slot"]) == DAMAGE_REASON_COLLISION
    assert not is_view_damaged(third)
    assert collision_alert_text.getText() == "DRONE COLLISION"
    assert "Only the colliding drones" in collision_alert_body_text.getText()

    third["root"].setPos(5.0, 5.0, 22.0)
    first["root"].setPos(5.0, 5.0, 22.0)
    second["root"].setPos(5.0, 5.0, 22.0)
    check_manual_collisions()
    assert not is_view_damaged(third)
    assert not any(
        get_drone_damage_reason(view["role"], view["slot"])
        == DAMAGE_REASON_KILL_SWITCH
        for view in views
    )

    print("REAL_SIM_SELF_TEST collision_damage OK")
    app.userExit()


def run_environment_damage_self_test():
    start_game_from_pregame()
    views = get_all_drone_views()
    assert len(views) >= 3
    first, second, third = views[:3]
    safe_test_z = max(
        32.0,
        max((sample["top_z"] for sample in tree_canopy_samples), default=22.0) + 10.0,
    )
    for index, view in enumerate(views):
        set_drone_view_control_mode(view, CONTROL_MODE_MANUAL)
        view["root"].setPos(28.0 + index * 9.0, 32.0, safe_test_z)
        motion = get_view_crash_rig_parts(view)[2]
        motion.velocity = Vec3(0.0, 0.0, 0.0)
        motion.target_altitude_meters = view["root"].getZ()

    check_environment_impacts()
    assert not any(is_view_damaged(view) for view in views)

    branch_impact_position = None
    for canopy_sample in tree_canopy_samples:
        canopy_x = (canopy_sample["min_x"] + canopy_sample["max_x"]) * 0.5
        canopy_y = (canopy_sample["min_y"] + canopy_sample["max_y"]) * 0.5
        ground_sample = sample_ground(canopy_x, canopy_y)
        ground_z = ground_sample[0].z if ground_sample is not None else 0.0
        impact_z = canopy_sample["top_z"] - 0.1
        if impact_z > ground_z + DRONE_GROUND_IMPACT_CLEARANCE_METERS + 0.5:
            branch_impact_position = (canopy_x, canopy_y, impact_z)
            break
    assert branch_impact_position is not None

    first["root"].setPos(*branch_impact_position)
    check_environment_impacts()
    assert is_view_damaged(first)
    assert get_drone_damage_reason(first["role"], first["slot"]) == DAMAGE_REASON_BRANCH_IMPACT
    assert not is_view_damaged(second)
    assert not is_view_damaged(third)
    assert collision_alert_text.getText() == "BRANCH IMPACT"
    assert "Only this drone" in collision_alert_body_text.getText()
    assert "kill" not in collision_alert_text.getText().lower()

    ground_sample = sample_ground(22.0, 24.0)
    ground_z = ground_sample[0].z if ground_sample is not None else 0.0
    second["root"].setPos(22.0, 24.0, ground_z + 0.1)
    check_environment_impacts()
    assert is_view_damaged(second)
    assert get_drone_damage_reason(second["role"], second["slot"]) == DAMAGE_REASON_GROUND_IMPACT
    assert not is_view_damaged(third)
    assert collision_alert_text.getText() == "GROUND IMPACT"
    assert "Only this drone" in collision_alert_body_text.getText()
    assert not any(
        get_drone_damage_reason(view["role"], view["slot"])
        == DAMAGE_REASON_KILL_SWITCH
        for view in views
    )

    print("REAL_SIM_SELF_TEST environment_damage OK")
    app.userExit()


def run_keyboard_controls_self_test():
    global tx12_controller_device, tx12_controller_name
    global tx12_controller_pressed_buttons
    global tx12_controller_input_enabled
    global tx12_controller_channel_kill_active, tx12_controller_recenter_pending
    global tx12_controller_kill_channel_last_value

    start_game_from_pregame()
    set_camera_target_role_slot("survey", 1)
    set_manual_mode()
    set_hover_all(False)

    tx12_controller_device = None
    tx12_controller_name = ""
    tx12_controller_pressed_buttons = set()
    tx12_controller_input_enabled = True
    for axis_name in tx12_controller_axes:
        tx12_controller_axes[axis_name] = 1.0
    refresh_tx12_controller_device()
    assert not tx12_controller_is_connected()
    assert all(abs(value) <= 0.001 for value in tx12_controller_axes.values())
    assert abs(_tx12_apply_centered_axis(0.7, 0.7)) <= 0.001
    assert _tx12_apply_centered_axis(-0.7, 0.7) < 0.0
    tx12_controller_channel_baselines[5] = -1.0
    tx12_controller_channel_kill_active = False
    tx12_controller_kill_channel_last_value = None
    update_tx12_channel_switches((0.0, 0.0, -1.0, 0.0, -1.0, 0.0, -1.0, 0.0))
    assert not tx12_controller_channel_kill_active
    assert set(tx12_controller_axes) == {"right", "forward", "vertical", "yaw"}
    assert set(tx12_controller_axis_centers) == {"right", "forward", "vertical", "yaw"}

    for action_name in key_map:
        key_map[action_name] = False
    assert not manual_movement_command_active()

    set_key("forward", True)
    set_key("right", True)
    set_key("up", True)
    assert manual_movement_command_active()
    desired_vx, desired_vy, desired_vz = compute_manual_desired_velocity(
        0.0,
        key_map,
        4.0,
    )
    assert desired_vx > 0.0
    assert desired_vy > 0.0
    assert desired_vz > 0.0
    set_hover_all(True)
    assert hover_all_active
    if hover_all_active and raw_keyboard_movement_command_active():
        set_hover_all(False)
    assert not hover_all_active

    set_key("forward", False)
    set_key("right", False)
    set_key("up", False)
    set_key("down", True)
    desired_vx, desired_vy, desired_vz = compute_manual_desired_velocity(
        0.0,
        key_map,
        4.0,
    )
    assert abs(desired_vx) <= 0.001
    assert abs(desired_vy) <= 0.001
    assert desired_vz < 0.0

    class FakeAxisState:
        known = True

        def __init__(self, value):
            self.value = value

    class FakeConnectedController:
        connected = True
        device_class = InputDevice.DeviceClass.flight_stick
        buttons = ()

        def __init__(self, channel_values=()):
            self.axes = [FakeAxisState(value) for value in channel_values]

        def poll(self):
            return None

    set_key("down", False)
    tx12_controller_device = FakeConnectedController(
        (0.25, -0.50, 0.75, -0.25, -1.0),
    )
    tx12_controller_name = "self-test controller"
    for axis_name in tx12_controller_axis_centers:
        tx12_controller_axis_centers[axis_name] = 0.0
    tx12_controller_recenter_pending = False
    tx12_controller_channel_kill_active = False
    update_tx12_controller_input(0.1)
    assert tx12_controller_axes["right"] > 0.0
    assert tx12_controller_axes["forward"] > 0.0
    assert tx12_controller_axes["vertical"] > 0.0
    assert tx12_controller_axes["yaw"] > 0.0
    assert not tx12_controller_channel_kill_active
    tx12_controller_device = FakeConnectedController(
        (0.0, 0.0, 0.0, 0.0, -1.0),
    )
    update_tx12_controller_input(0.1)
    assert abs(tx12_controller_axes["vertical"]) <= 0.001
    set_speed_preset(DRONE_SPEED_PRESET_SLOW)
    assert (
        abs(
            get_manual_drone_speed_units_per_second()
            - TX12_CONTROLLER_MANUAL_SPEED_UNITS_PER_SECOND
        )
        <= 0.001
    )

    tx12_controller_axes["forward"] = 0.6
    tx12_controller_axes["right"] = 0.0
    tx12_controller_axes["vertical"] = 0.0
    tx12_controller_axes["yaw"] = 0.0
    set_key("backward", True)
    desired_vx, desired_vy, desired_vz = compute_manual_desired_velocity(
        0.0,
        key_map,
        4.0,
    )
    assert keyboard_movement_command_active()
    assert desired_vy < 0.0

    tx12_controller_axes["forward"] = 0.0
    set_key("backward", False)
    assert not manual_movement_command_active()
    tx12_controller_axes["vertical"] = 0.75
    desired_vx, desired_vy, desired_vz = compute_manual_desired_velocity(
        0.0,
        key_map,
        4.0,
    )
    assert abs(desired_vx) <= 0.001
    assert abs(desired_vy) <= 0.001
    assert desired_vz > 0.0
    tx12_controller_axes["vertical"] = -0.75
    desired_vx, desired_vy, desired_vz = compute_manual_desired_velocity(
        0.0,
        key_map,
        4.0,
    )
    assert desired_vz < 0.0

    tx12_controller_device = FakeConnectedController(
        (0.0, 0.0, 0.20, 0.0, -1.0),
    )
    tx12_controller_recenter_pending = True
    update_tx12_controller_input(0.1)
    assert abs(tx12_controller_axes["vertical"]) <= 0.001
    update_tx12_controller_input(0.1)
    assert abs(tx12_controller_axes["vertical"]) <= 0.001
    tx12_controller_device = FakeConnectedController(
        (0.0, 0.0, 0.60, 0.0, -1.0),
    )
    update_tx12_controller_input(0.1)
    assert tx12_controller_axes["vertical"] > 0.0
    tx12_controller_device = FakeConnectedController(
        (0.0, 0.0, -0.20, 0.0, -1.0),
    )
    update_tx12_controller_input(0.1)
    assert tx12_controller_axes["vertical"] < 0.0

    tx12_controller_input_enabled = False
    set_key("backward", True)
    assert keyboard_movement_command_active()
    desired_vx, desired_vy, desired_vz = compute_manual_desired_velocity(
        0.0,
        key_map,
        4.0,
    )
    assert desired_vy < 0.0

    set_key("backward", False)
    tx12_controller_input_enabled = True
    tx12_controller_channel_kill_active = False
    tx12_controller_kill_channel_last_value = None
    update_tx12_channel_switches((0.0, 0.0, 0.0, 0.0, 1.0))
    assert not tx12_controller_channel_kill_active
    assert not all(is_view_damaged(view) for view in get_all_drone_views())
    update_tx12_channel_switches((0.0, 0.0, 0.0, 0.0, -1.0))
    assert tx12_controller_channel_kill_active
    assert all(is_view_damaged(view) for view in get_all_drone_views())
    assert collision_alert_text.getText() == "KILL SWITCH ACTIVATED"

    tx12_controller_device = None
    tx12_controller_name = ""
    tx12_controller_input_enabled = True
    tx12_controller_channel_kill_active = False
    set_hover_all(False)
    zero_tx12_controller_axes()
    for action_name in key_map:
        key_map[action_name] = False
    print("REAL_SIM_SELF_TEST keyboard_controls OK")
    app.userExit()


def run_team_setup_self_test():
    assert pregame_active
    assert TEAM_TOTAL_DRONE_OPTIONS == (2, 4, 6)

    set_team_total_drone_count(2)
    assert team_total_drone_count == 2
    assert get_team_survey_drone_count() == 1
    assert team_water_drone_count == 1
    choose_more_survey_drones()
    choose_more_water_drones()
    assert get_team_survey_drone_count() == 1
    assert team_water_drone_count == 1

    set_team_total_drone_count(4)
    assert team_total_drone_count == 4
    assert get_team_survey_drone_count() == 2
    assert team_water_drone_count == 2
    handle_number_key(2)
    assert get_team_survey_drone_count() == 1
    assert team_water_drone_count == 3
    handle_number_key(1)
    assert get_team_survey_drone_count() == 2
    assert team_water_drone_count == 2

    set_team_total_drone_count(6)
    assert team_total_drone_count == 6
    assert get_team_survey_drone_count() == 3
    assert team_water_drone_count == 3
    handle_number_key(1)
    assert get_team_survey_drone_count() == 4
    assert team_water_drone_count == 2

    set_team_total_drone_count(2)
    assert team_total_drone_count == 2
    assert get_team_survey_drone_count() == 1
    assert team_water_drone_count == 1
    start_game_from_pregame()
    views = get_all_drone_views()
    assert len(views) == 2
    assert [view["role"] for view in views] == ["survey", "water"]
    assert get_drone_view_for_role_slot("survey", 2) is None
    assert get_water_view_by_slot(2) is None

    print("REAL_SIM_SELF_TEST team_setup OK")
    app.userExit()


def run_spawn_integrity_self_test():
    start_game_from_pregame()
    views = get_all_drone_views()
    assert len(views) == team_total_drone_count
    assert sum(1 for view in views if view["role"] == "survey") == get_team_survey_drone_count()
    assert sum(1 for view in views if view["role"] == "water") == team_water_drone_count

    for view in views:
        root = view["root"]
        assert SPAWN_X_MIN <= root.getX() <= SPAWN_X_MAX
        assert SPAWN_Y_MIN <= root.getY() <= SPAWN_Y_MAX
        assert not is_view_damaged(view)

    for first_index in range(len(views)):
        first = views[first_index]
        for second in views[first_index + 1:]:
            dx = second["root"].getX() - first["root"].getX()
            dy = second["root"].getY() - first["root"].getY()
            dz = second["root"].getZ() - first["root"].getZ()
            horizontal_distance = (dx * dx + dy * dy) ** 0.5
            assert not (
                horizontal_distance < DRONE_CRASH_RADIUS_METERS
                and abs(dz) < DRONE_CRASH_VERTICAL_RADIUS_METERS
            ), (
                f"spawn collision risk: {first['label']} vs {second['label']} "
                f"h={horizontal_distance:.2f} dz={abs(dz):.2f}"
            )

    check_manual_collisions()
    check_environment_impacts()
    assert not any(is_view_damaged(view) for view in views)

    print(
        "REAL_SIM_SELF_TEST spawn_integrity OK "
        f"{get_team_survey_drone_count()}S/{team_water_drone_count}W"
    )
    app.userExit()


def run_extra_water_pairing_self_test():
    set_team_total_drone_count(6)
    choose_more_water_drones()
    assert get_team_survey_drone_count() == 2
    assert team_water_drone_count == 4
    start_game_from_pregame()

    set_full_automation()
    for slot in (1, 2):
        assert get_drone_view_control_mode(
            get_drone_view_for_role_slot("survey", slot)
        ) == CONTROL_MODE_AUTOMATION
        assert get_drone_view_control_mode(
            get_water_view_by_slot(slot)
        ) == CONTROL_MODE_AUTOMATION
        assert water_slot_has_paired_survey(slot)
        assert water_slot_can_auto_dispatch(slot)

    for slot in (3, 4):
        view = get_water_view_by_slot(slot)
        assert view is not None
        assert not water_slot_has_paired_survey(slot)
        assert get_drone_view_control_mode(view) == CONTROL_MODE_MANUAL
        assert not water_slot_can_auto_dispatch(slot)
        assert not is_water_slot_dispatched(slot)

    fire_x, fire_y = get_survey_search_sector(1).center()
    ground_sample = sample_ground(fire_x, fire_y)
    fire_z = ground_sample[0].z if ground_sample is not None else 0.0
    hotspot = build_fire_hotspot_at(fire_x, fire_y, fire_z)
    hotspot.fire_event_id = 301
    fire_hotspots.append(hotspot)
    mark_hotspot_detected(
        hotspot,
        motion_state.sim_time_seconds,
        highlight_dispatch=False,
        trigger_alarm=False,
        detected_by_survey_slot=1,
    )

    water_1 = get_water_view_by_slot(1)
    water_2 = get_water_view_by_slot(2)
    water_3 = get_water_view_by_slot(3)
    water_4 = get_water_view_by_slot(4)
    assert choose_paired_survey_water_hotspot(
        1,
        water_1["root"].getX(),
        water_1["root"].getY(),
    ) is hotspot
    assert choose_paired_survey_water_hotspot(
        2,
        water_2["root"].getX(),
        water_2["root"].getY(),
    ) is None
    assert choose_paired_survey_water_hotspot(
        3,
        water_3["root"].getX(),
        water_3["root"].getY(),
    ) is None
    assert choose_paired_survey_water_hotspot(
        4,
        water_4["root"].getX(),
        water_4["root"].getY(),
    ) is None

    update_water_drone(0.1)
    update_water_follower_drone(water_2["follower"], 0.1)
    update_manual_follower_drone(water_3["follower"], 0.1, False)
    update_manual_follower_drone(water_4["follower"], 0.1, False)
    assert team_state.water_target_hotspot is hotspot
    assert water_2["follower"]["target_hotspot"] is None
    assert get_water_current_target_for_view(water_3) is None
    assert get_water_current_target_for_view(water_4) is None
    assert get_water_route_world_points(water_3) == tuple()
    assert get_water_route_world_points(water_4) == tuple()

    print("REAL_SIM_SELF_TEST extra_water_pairing OK")
    app.userExit()


def run_water_suppression_heading_self_test():
    set_team_total_drone_count(4)
    start_game_from_pregame()
    set_full_automation()

    fire_x, fire_y = get_survey_search_sector(1).center()
    ground_sample = sample_ground(fire_x, fire_y)
    fire_z = ground_sample[0].z if ground_sample is not None else 0.0
    hotspot = build_fire_hotspot_at(fire_x, fire_y, fire_z)
    hotspot.fire_event_id = 401
    fire_hotspots.append(hotspot)
    mark_hotspot_detected(
        hotspot,
        motion_state.sim_time_seconds,
        highlight_dispatch=False,
        trigger_alarm=False,
        detected_by_survey_slot=1,
    )

    water_x = water_drone.getX()
    water_y = water_drone.getY()
    expected_heading = water_suppression_heading_degrees(water_x, water_y, hotspot)
    update_water_drone(0.05)
    actual_heading = degrees(water_motion_state.heading_hold_radians)
    assert abs(wrap_angle_degrees(actual_heading - expected_heading)) <= 0.1

    fire_2_x, fire_2_y = get_survey_search_sector(2).center()
    fire_2_ground_sample = sample_ground(fire_2_x, fire_2_y)
    fire_2_z = fire_2_ground_sample[0].z if fire_2_ground_sample is not None else 0.0
    hotspot_2 = build_fire_hotspot_at(fire_2_x, fire_2_y, fire_2_z)
    hotspot_2.fire_event_id = 402
    fire_hotspots.append(hotspot_2)
    mark_hotspot_detected(
        hotspot_2,
        motion_state.sim_time_seconds,
        highlight_dispatch=False,
        trigger_alarm=False,
        detected_by_survey_slot=2,
    )
    water_2_view = get_water_view_by_slot(2)
    assert water_2_view is not None and water_2_view["follower"] is not None
    follower = water_2_view["follower"]
    follower_x = follower["root"].getX()
    follower_y = follower["root"].getY()
    expected_follower_heading = water_suppression_heading_degrees(
        follower_x,
        follower_y,
        hotspot_2,
    )
    update_water_follower_drone(follower, 0.05)
    actual_follower_heading = degrees(follower["motion_state"].heading_hold_radians)
    assert abs(wrap_angle_degrees(actual_follower_heading - expected_follower_heading)) <= 0.1

    print("REAL_SIM_SELF_TEST water_suppression_heading OK")
    app.userExit()


def run_metrics_toggle_self_test():
    start_game_from_pregame()
    update_performance_metrics_visibility()
    assert performance_panel_root is not None
    assert performance_toggle_button_root is not None
    assert not performance_panel_root.isHidden()
    assert not performance_toggle_button_root.isHidden()

    toggle_performance_metrics_panel()
    assert performance_metrics_hidden
    assert performance_panel_root.isHidden()
    assert not performance_toggle_button_root.isHidden()
    assert performance_toggle_button_text.getText() == "SHOW METRICS"

    toggle_performance_metrics_panel()
    assert not performance_metrics_hidden
    assert not performance_panel_root.isHidden()
    assert performance_toggle_button_text.getText() == "HIDE"

    print("REAL_SIM_SELF_TEST metrics_toggle OK")
    app.userExit()


self_test_name = os.environ.get("REAL_SIM_SELF_TEST", "").strip().lower()
if self_test_name == "water_dispatch":
    run_water_dispatch_self_test()
    sys.exit(0)
if self_test_name == "fire_trucks":
    run_fire_truck_self_test()
    sys.exit(0)
if self_test_name == "kill_switch":
    run_kill_switch_self_test()
    sys.exit(0)
if self_test_name == "collision_damage":
    run_collision_damage_self_test()
    sys.exit(0)
if self_test_name == "environment_damage":
    run_environment_damage_self_test()
    sys.exit(0)
if self_test_name == "keyboard_controls":
    run_keyboard_controls_self_test()
    sys.exit(0)
if self_test_name == "team_setup":
    run_team_setup_self_test()
    sys.exit(0)
if self_test_name == "spawn_integrity":
    run_spawn_integrity_self_test()
    sys.exit(0)
if self_test_name == "extra_water_pairing":
    run_extra_water_pairing_self_test()
    sys.exit(0)
if self_test_name == "water_suppression_heading":
    run_water_suppression_heading_self_test()
    sys.exit(0)
if self_test_name == "metrics_toggle":
    run_metrics_toggle_self_test()
    sys.exit(0)

if REAL_SIM_AUTOSTART:
    start_game_from_pregame()
    if REAL_SIM_AUTOSTART_FULL_AUTO:
        set_full_automation()

app.run()
