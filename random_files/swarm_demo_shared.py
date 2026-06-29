import json
import os
import subprocess
from math import atan2, copysign, cos, degrees, exp, radians, sin, tau
from pathlib import Path
import random
from collections import deque
from dataclasses import dataclass, field

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
    LineSegs,
    NodePath,
    PandaNode,
    Plane,
    PlaneNode,
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
    compute_desired_velocity,
    count_detected_hotspots,
    update_axis_velocity,
)


def read_nonnegative_int_env(name, default_value):
    try:
        return max(0, int(os.environ.get(name, str(default_value))))
    except (TypeError, ValueError):
        return default_value


def read_positive_int_env(name, default_value):
    return max(1, read_nonnegative_int_env(name, default_value))


# ------------------------------
# World and visual configuration
# ------------------------------
# Finalized project scale convention (May 14, 2026):
# 1 simulation world unit equals 1 meter.
REAL_SIM_MAX_FPS = max(15, read_positive_int_env("REAL_SIM_MAX_FPS", 45))
REAL_SIM_FIRE_VISUAL_HZ = read_positive_int_env("REAL_SIM_FIRE_VISUAL_HZ", 18)
REAL_SIM_MAP_HZ = read_positive_int_env("REAL_SIM_MAP_HZ", 10)
REAL_SIM_HUD_HZ = read_positive_int_env("REAL_SIM_HUD_HZ", 4)

SIM_METERS_PER_UNIT = 1.0
SIM_UNITS_PER_METER = 1.0 / SIM_METERS_PER_UNIT

WORLD_X_LIMIT = 50
WORLD_Y_LIMIT = 50
GRASS_COUNT = read_nonnegative_int_env("REAL_SIM_GRASS_COUNT", 140)

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
TREE_COUNT = min(
    TREE_COUNT,
    read_positive_int_env("REAL_SIM_TREE_COUNT_CAP", 140),
)

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
CONTROL_INPUT_LAG_SECONDS = 0.12
DRONE_ACCEL_UNITS_PER_SECOND_SQ = 18.0
DRONE_DECEL_UNITS_PER_SECOND_SQ = 22.0
DRONE_SPEED_PRESET_SLOW = "slow"
DRONE_SPEED_PRESET_NORMAL = "normal"
DRONE_SPEED_PRESET_FAST = "fast"
DRONE_SPEED_PRESET_MULTIPLIERS = {
    DRONE_SPEED_PRESET_SLOW: 0.35,
    DRONE_SPEED_PRESET_NORMAL: 0.85,
    DRONE_SPEED_PRESET_FAST: 1.75,
}
DRONE_MAX_ROLL_DEGREES = 10.0
DRONE_MAX_PITCH_DEGREES = 7.5
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
DRONE_PROPELLER_BLADE_HALF_LENGTH = 1.35
DRONE_PROPELLER_BLADE_HALF_WIDTH = 0.12
DRONE_PROPELLER_BLADE_ALPHA = 0.56
DRONE_PROPELLER_X_OFFSET_FACTOR = 0.34
DRONE_PROPELLER_Z_OFFSET_FACTOR = 0.33
DRONE_PROPELLER_Y_TOP_MARGIN_FACTOR = 0.28
# Fine alignment knobs in drone-model local units.
# Negative Y lowers rotor plane toward the arms/body.
DRONE_PROPELLER_X_NUDGE_LOCAL = 0.0
DRONE_PROPELLER_Y_NUDGE_LOCAL = -0.45
DRONE_PROPELLER_Z_NUDGE_LOCAL = 0.0
DRONE_PROPELLER_SPIN_BASE_DEGREES_PER_SECOND = 980.0
DRONE_PROPELLER_SPIN_GAIN_DEGREES_PER_SECOND = 58.0
CAMERA_DRAG_X_DEGREES_PER_UNIT = -60.0
THIRD_PERSON_DRAG_Y_DEGREES_PER_UNIT = -60.0
FIRST_PERSON_DRAG_Y_DEGREES_PER_UNIT = 60.0
CAMERA_DRAG_DELTA_CLAMP = 0.05

THIRD_PERSON_CAMERA_DISTANCE = 13.5
THIRD_PERSON_CAMERA_HEIGHT = 4.2
THIRD_PERSON_CAMERA_PITCH_DEGREES = 22.0

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

FIRE_HOTSPOT_INITIAL_COUNT = 2
FIRE_HOTSPOT_ACTIVE_LIMIT = 80
FIRE_HOTSPOT_SPAWN_ATTEMPTS = 350
FIRE_HOTSPOT_TREE_CLEARANCE_METERS = 6.0
FIRE_HOTSPOT_DETECTION_RADIUS_METERS = 15.0
FIRE_HOTSPOT_DETECTION_PROBABILITY_PER_SECOND = 1.8
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
FIRE_REIGNITE_PROBABILITY_PER_SECOND = 0.0035
FIRE_SPREAD_ENABLED = True
FIRE_SPREAD_CHECK_INTERVAL_SECONDS = 1.2
FIRE_SPREAD_EVENTS_PER_TICK = 2
FIRE_SPREAD_MIN_DISTANCE_METERS = 10.0
FIRE_SPREAD_MAX_DISTANCE_METERS = 22.0
FIRE_SPREAD_MIN_HOTSPOT_SEPARATION_METERS = 8.0
FIRE_SPREAD_TREE_CLEARANCE_METERS = 3.5
FIRE_SPREAD_IGNORE_TREE_CLEARANCE = True
FIRE_SPREAD_SPAWN_ATTEMPTS = 48
FIRE_SPREAD_FALLBACK_RANDOM_ATTEMPTS = 36
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
WIND_FIRE_DIRECTIONAL_BIAS_PER_METER_PER_SECOND = 0.11
WIND_FIRE_BIAS_CONE_DEGREES = 55.0
WIND_FIRE_DOWNWIND_DISTANCE_GAIN_METERS_PER_METER_PER_SECOND = 1.1
WIND_FIRE_UPWIND_DISTANCE_PENALTY_METERS_PER_METER_PER_SECOND = 0.85

AUTOMATION_SPEED_FACTOR = 1.0
AUTOMATION_CRUISE_ALTITUDE_METERS = 22.0
AUTOMATION_ALTITUDE_GAIN = 2.0
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
AUTOMATION_CANOPY_QUERY_RADIUS_METERS = 16.0
AUTOMATION_CANOPY_LOOKAHEAD_SECONDS = 1.2
AUTOMATION_CANOPY_CLEARANCE_METERS = 6.0
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

DEFAULT_BACKGROUND_COLOR = (0.53, 0.76, 0.92, 1)
THERMAL_BACKGROUND_COLOR = (0.04, 0.05, 0.08, 1)
THERMAL_HIDE_GRASS = True
THERMAL_HOTSPOT_GLOW_SCALE = 2.6
THERMAL_HOTSPOT_BEACON_SCALE = 5.8
THERMAL_HOTSPOT_BIN_ORDER = 60
DETECTED_HOTSPOT_GLOW_SCALE = 2.35
DETECTED_HOTSPOT_BEACON_SCALE = 1.65
DETECTED_HOTSPOT_OVERLAY_BIN_ORDER = 42
BAKED_FIRE_FRAME_RATE = 24.0
BAKED_FIRE_CARD_HALF_WIDTH_METERS = 4.2
BAKED_FIRE_CARD_HEIGHT_METERS = 10.2
BAKED_FIRE_CARD_LIFT_METERS = 0.24
BAKED_FIRE_GLOW_HALF_WIDTH_METERS = 2.05
BAKED_FIRE_GLOW_HALF_DEPTH_METERS = 1.08
BAKED_FIRE_ACTIVE_SCALE = 1.08
BAKED_FIRE_DETECTED_SCALE = 1.18
BAKED_FIRE_CONTAINED_SCALE = 0.86
BAKED_FIRE_GROWTH_MIN_SCALE = 0.82
BAKED_FIRE_GROWTH_MAX_SCALE = 1.38
BAKED_FIRE_MESH_TARGET_HEIGHT_METERS = 8.6
BAKED_FIRE_MESH_GROUND_OFFSET_METERS = 0.06
BAKED_FIRE_MESH_HORIZONTAL_SCALE = 0.28
BAKED_FIRE_MESH_ACTIVE_ALPHA = 0.14
BAKED_FIRE_MESH_DETECTED_ALPHA = 0.11
BAKED_FIRE_MESH_CONTAINED_ALPHA = 0.08

FIRE_STATE_ACTIVE = "active"
FIRE_STATE_CONTAINED = "contained"
FIRE_STATE_OUT = "out"
FIRE_STATE_BURNED = "burned"

thermal_view_enabled = False

PROJECT_ROOT = Path(__file__).resolve().parent
FOREST_SIM_ROOT = PROJECT_ROOT.parent / "forest_sim_demo"


# Keep startup logs useful without noisy model warnings.
if os.environ.get("REAL_SIM_HEADLESS", "0") == "1":
    loadPrcFileData("", "window-type none")
loadPrcFileData("", "notify-level-assimp error")
loadPrcFileData("", "audio-library-name p3openal_audio")
loadPrcFileData("", "clock-mode limited")
loadPrcFileData("", f"clock-frame-rate {REAL_SIM_MAX_FPS}")
loadPrcFileData("", "sync-video true")
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


def setup_lights():
    ambient_light = AmbientLight("ambient_light")
    ambient_light.setColor((0.9, 0.9, 0.9, 1))
    ambient_np = app.render.attachNewNode(ambient_light)
    app.render.setLight(ambient_np)

    sun_light = DirectionalLight("sun_light")
    sun_light.setColor((0.55, 0.55, 0.5, 1))
    sun_np = app.render.attachNewNode(sun_light)
    sun_np.setHpr(45, -60, 0)
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

    image = PNMImage()
    if not image.read(texture_path):
        texture = app.loader.loadTexture(texture_path)
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

    texture = app.loader.loadTexture(texture_path)
    texture.load(image)
    texture.setWrapU(SamplerState.WM_clamp)
    texture.setWrapV(SamplerState.WM_clamp)
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


# ---------------------
# Ground / terrain mesh
# ---------------------
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
    terrain = GeoMipTerrain("procedural_mountains")
    terrain.setHeightfield(build_mountain_heightfield(TERRAIN_HEIGHTFIELD_SIZE))
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


def sample_ground(x, y):
    ground_probe_np.setPos(x, y, 150)
    ground_probe_queue.clearEntries()
    ground_probe_traverser.traverse(ground)

    if ground_probe_queue.getNumEntries() == 0:
        return None

    ground_probe_queue.sortEntries()
    entry = ground_probe_queue.getEntry(0)
    point = entry.getSurfacePoint(app.render)
    normal = entry.getSurfaceNormal(app.render)

    if normal.lengthSquared() > 0:
        normal.normalize()
    else:
        normal = Vec3(0, 0, 1)

    return point, normal


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


def spawn_fire_truck_marker():
    """Spawn a fire truck model on the ground; fallback to a flat marker if unavailable."""
    chosen_spawn = None
    for _ in range(FIRE_TRUCK_SPAWN_ATTEMPTS):
        candidate_spawn = random_world_position(exclusion_radius=14)
        candidate_x = candidate_spawn[0]
        candidate_y = candidate_spawn[1]
        if _is_position_clear_of_trees(
            candidate_x,
            candidate_y,
            FIRE_TRUCK_TREE_CLEARANCE_METERS,
        ):
            chosen_spawn = candidate_spawn
            break

    if chosen_spawn is None:
        chosen_spawn = random_world_position(exclusion_radius=14)

    x = chosen_spawn[0]
    y = chosen_spawn[1]
    z = chosen_spawn[2]

    truck_root = app.render.attachNewNode("fire_truck")
    truck_root.setPos(x, y, z + 0.08)
    truck_root.setH(random.uniform(0, 360))

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
        if hotspot.detected:
            hotspot.fire_cluster_color = (1.0, 0.72, 0.3, 0.74)
            hotspot.fire_cluster_scale_multiplier = (
                BAKED_FIRE_DETECTED_SCALE if using_baked_fire else 1.08
            )
            hotspot.base_glow_scale_multiplier = 1.02 if using_baked_fire else 1.18
            hotspot.base_glow.setColor(
                1.0,
                0.56,
                0.14,
                0.06 if using_baked_fire_mesh else (0.1 if using_baked_fire else 0.28),
            )
        else:
            hotspot.fire_cluster_color = (1.0, 0.56, 0.14, 0.9)
            hotspot.fire_cluster_scale_multiplier = (
                BAKED_FIRE_ACTIVE_SCALE if using_baked_fire else 1.0
            )
            hotspot.base_glow_scale_multiplier = 0.96 if using_baked_fire else 1.06
            hotspot.base_glow.setColor(
                1.0,
                0.34,
                0.08,
                0.04 if using_baked_fire_mesh else (0.08 if using_baked_fire else 0.24),
            )
    elif state_is_contained:
        hotspot.base_glow.show()
        for flame_node in hotspot.flame_nodes:
            flame_node.show()
        if hotspot.current_mesh_frame_index >= 0:
            hotspot.mesh_frame_nodes[hotspot.current_mesh_frame_index].show()
        hotspot.fire_cluster_color = (0.95, 0.34, 0.12, 0.5)
        hotspot.fire_cluster_scale_multiplier = (
            BAKED_FIRE_CONTAINED_SCALE if using_baked_fire else 0.6
        )
        hotspot.base_glow_scale_multiplier = 0.82 if using_baked_fire else 0.72
        hotspot.base_glow.setColor(
            0.82,
            0.18,
            0.06,
            0.03 if using_baked_fire_mesh else (0.05 if using_baked_fire else 0.14),
        )
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
                if hotspot.detected:
                    hotspot.effect_model.setColorScale(1.0, 0.58, 0.22, 0.84)
                    hotspot.effect_scale_multiplier = FIRE_EFFECT_DETECTED_SCALE
                else:
                    hotspot.effect_model.setColorScale(1.0, 0.42, 0.08, 0.94)
                    hotspot.effect_scale_multiplier = FIRE_EFFECT_ACTIVE_SCALE
            elif state_is_contained:
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
            if hotspot.detected:
                green_channel = max(0.1, 0.34 - burn_progress * 0.16)
                blue_channel = max(0.04, 0.14 - burn_progress * 0.07)
                hotspot.glow.setColor(1.0, green_channel, blue_channel, 0.98)
                hotspot.beacon_a.setColor(1.0, min(1.0, green_channel + 0.2), min(1.0, blue_channel + 0.1), 0.98)
                hotspot.beacon_b.setColor(1.0, min(1.0, green_channel + 0.2), min(1.0, blue_channel + 0.1), 0.98)
            else:
                green_channel = max(0.06, 0.26 - burn_progress * 0.2)
                blue_channel = max(0.01, 0.08 - burn_progress * 0.06)
                hotspot.glow.setColor(1.0, green_channel, blue_channel, 0.98)
                hotspot.beacon_a.setColor(1.0, min(1.0, green_channel + 0.24), min(1.0, blue_channel + 0.1), 0.98)
                hotspot.beacon_b.setColor(1.0, min(1.0, green_channel + 0.24), min(1.0, blue_channel + 0.1), 0.98)
        elif state_is_contained:
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
        show_detected_overlay = hotspot.detected and hotspot_state in (
            FIRE_STATE_ACTIVE,
            FIRE_STATE_CONTAINED,
        )
        if show_detected_overlay:
            hotspot.glow.show()
            hotspot.beacon_a.show()
            hotspot.beacon_b.show()
            hotspot.glow.setScale(DETECTED_HOTSPOT_GLOW_SCALE)
            hotspot.beacon_a.setScale(DETECTED_HOTSPOT_BEACON_SCALE)
            hotspot.beacon_b.setScale(DETECTED_HOTSPOT_BEACON_SCALE * 0.72)
            for part in hotspot_parts:
                part.setBin("fixed", DETECTED_HOTSPOT_OVERLAY_BIN_ORDER)
                part.setDepthTest(False)
                part.setDepthWrite(False)
            if state_is_active:
                hotspot.glow.setColor(1.0, 0.14, 0.06, 0.74)
                hotspot.beacon_a.setColor(1.0, 0.45, 0.12, 0.9)
                hotspot.beacon_b.setColor(1.0, 0.82, 0.22, 0.96)
            else:
                hotspot.glow.setColor(1.0, 0.56, 0.14, 0.62)
                hotspot.beacon_a.setColor(1.0, 0.76, 0.26, 0.82)
                hotspot.beacon_b.setColor(1.0, 0.92, 0.5, 0.88)
        else:
            hotspot.glow.hide()
            hotspot.beacon_a.hide()
            hotspot.beacon_b.hide()
            for part in hotspot_parts:
                part.clearBin()
                part.setDepthTest(True)
                part.setDepthWrite(True)
        if state_is_burned:
            hotspot.scar_core.setColor(0.01, 0.01, 0.01, 0.98)
            hotspot.scar_cross_a.setColor(0.78, 0.06, 0.03, 0.96)
            hotspot.scar_cross_b.setColor(0.78, 0.06, 0.03, 0.96)


def update_hotspot_effect_animation(dt):
    animated_fire_textures = load_baked_fire_textures()
    animated_fire_frame_rate = baked_fire_frame_rate()
    animated_fire_mesh_frame_rate = baked_fire_mesh_frame_rate()
    for hotspot in fire_hotspots:
        fire_growth_scale = hotspot_fire_growth_scale(hotspot)
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
                )

                brightness = max(0.72, 1.0 - (index * 0.08))
                if animated_fire_textures:
                    flame_node.setColor(
                        brightness,
                        min(1.0, brightness * 0.98),
                        min(1.0, brightness * 0.92),
                        hotspot.fire_cluster_color[3]
                        * (0.76 if hotspot.mesh_frame_nodes else 1.0)
                        * max(0.62, 1.0 - (index * 0.1)),
                    )
                else:
                    flame_node.setColor(
                        1.0,
                        min(1.0, hotspot.fire_cluster_color[1] * brightness + 0.08),
                        hotspot.fire_cluster_color[2] * brightness,
                        hotspot.fire_cluster_color[3] * max(0.55, 1.0 - (index * 0.08)),
                    )

            glow_pulse = 1.0 + (sin(hotspot.flame_flicker_phase_radians * 0.72) * 0.08)
            hotspot.base_glow.setScale(
                hotspot.base_glow_scale_multiplier
                * max(0.72, fire_growth_scale * 0.82)
                * glow_pulse
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
                mesh_node.setColorScale(
                    1.0,
                    min(1.0, hotspot.fire_cluster_color[1] + 0.08),
                    min(1.0, hotspot.fire_cluster_color[2] + 0.05),
                    mesh_alpha,
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

        if hotspot.detected and hotspot.suppression_state in (
            FIRE_STATE_ACTIVE,
            FIRE_STATE_CONTAINED,
        ):
            incident_scale = 1.0 + (
                clamp(incident_state.current_fire_area_sqm / 24.0, 0.0, 2.0) * 0.32
            )
            overlay_pulse = 1.0 + (
                sin(hotspot.effect_pulse_phase_radians * 1.22) * 0.18
            )
            beacon_pulse = 1.0 + (
                sin(hotspot.effect_pulse_phase_radians * 1.8 + 0.9) * 0.12
            )
            hotspot.glow.setScale(
                DETECTED_HOTSPOT_GLOW_SCALE
                * incident_scale
                * overlay_pulse
            )
            hotspot.beacon_a.setScale(
                DETECTED_HOTSPOT_BEACON_SCALE
                * incident_scale
                * beacon_pulse
            )
            hotspot.beacon_b.setScale(
                DETECTED_HOTSPOT_BEACON_SCALE
                * 0.72
                * incident_scale
                * (2.0 - beacon_pulse)
            )


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
    animated_fire_textures = load_baked_fire_textures()
    if animated_fire_textures:
        hotspot.flame_texture_frame_offset = random.randint(
            0,
            len(animated_fire_textures) - 1,
        )
    set_hotspot_visual(hotspot)
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
        tree_model = random.choice(tree_prototypes).instanceTo(tree_root)

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

        tint = random.uniform(0.94, 1.1)
        tree_model.setColorScale(tint, tint, tint, 1)
        tree_nodes.append(tree_root)
        #stores them in the list

    return tree_nodes


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


def build_drone_propellers(drone_model):
    # Compute bounds in the model's own local space so rotor offsets are not
    # accidentally double-scaled after parent transforms are applied.
    bounds = drone_model.getTightBounds(drone_model)
    if bounds is None:
        return []

    min_bound, max_bound = bounds
    span_x = max_bound.x - min_bound.x
    span_y = max_bound.y - min_bound.y
    span_z = max_bound.z - min_bound.z
    center_x = (min_bound.x + max_bound.x) * 0.5
    center_z = (min_bound.z + max_bound.z) * 0.5
    rotor_center_x = center_x + DRONE_PROPELLER_X_NUDGE_LOCAL
    rotor_center_z = center_z + DRONE_PROPELLER_Z_NUDGE_LOCAL
    rotor_y = (
        max_bound.y
        - (span_y * DRONE_PROPELLER_Y_TOP_MARGIN_FACTOR)
        + DRONE_PROPELLER_Y_NUDGE_LOCAL
    )
    rotor_offset_x = span_x * DRONE_PROPELLER_X_OFFSET_FACTOR
    rotor_offset_z = span_z * DRONE_PROPELLER_Z_OFFSET_FACTOR
    blade_color = (0.08, 0.09, 0.11, DRONE_PROPELLER_BLADE_ALPHA)
    rotor_specs = (
        ((rotor_center_x - rotor_offset_x, rotor_y, rotor_center_z - rotor_offset_z), 1.0),
        ((rotor_center_x + rotor_offset_x, rotor_y, rotor_center_z - rotor_offset_z), -1.0),
        ((rotor_center_x - rotor_offset_x, rotor_y, rotor_center_z + rotor_offset_z), -1.0),
        ((rotor_center_x + rotor_offset_x, rotor_y, rotor_center_z + rotor_offset_z), 1.0),
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


SWARM_SCENARIO_CASE1 = "case1"
SWARM_SCENARIO_CASE2 = "case2"
SWARM_SCENARIO_NAME = os.environ.get(
    "REAL_SIM_SWARM_SCENARIO",
    SWARM_SCENARIO_CASE1,
).strip().lower()
if SWARM_SCENARIO_NAME not in (SWARM_SCENARIO_CASE1, SWARM_SCENARIO_CASE2):
    SWARM_SCENARIO_NAME = SWARM_SCENARIO_CASE1

SWARM_SCENARIO_TITLE = (
    "Case 1: Full Coverage Patrol"
    if SWARM_SCENARIO_NAME == SWARM_SCENARIO_CASE1
    else "Case 2: Fire Detection and Response"
)
SWARM_DRONE_COUNT = max(
    1,
    read_positive_int_env("REAL_SIM_SWARM_DRONE_COUNT", 3),
)
SWARM_PATROL_PATTERN_LABEL = "BOUSTROPHEDON"
SWARM_SECTOR_MARGIN_METERS = 6.0
SWARM_PATROL_SWEEP_SPACING_METERS = max(
    8.0,
    FIRE_HOTSPOT_DETECTION_RADIUS_METERS * 0.7,
)
SWARM_MAP_TRAIL_POINT_INTERVAL_SECONDS = 0.8
SWARM_TRAIL_MAX_POINTS = 120
SWARM_SCAN_CELL_SIZE_METERS = 4.0
SWARM_CASE1_DURATION_SECONDS = 180.0
SWARM_CASE2_IGNITION_DELAY_SECONDS = float(
    os.environ.get("REAL_SIM_CASE2_IGNITION_DELAY_SECONDS", "60.0")
)
SWARM_CASE2_GROWTH_RATE_SQM_PER_SECOND = 1.0
SWARM_CASE2_TRUCK_ETA_SECONDS = max(
    4.0,
    FIRE_SUPPRESSION_CONTAIN_DELAY_SECONDS * 0.7,
)
SWARM_CASE2_EXTRA_HOTSPOT_AREA_TRIGGER_SQM = 16.0
SWARM_RESPONSE_TRACK_RADIUS_METERS = 8.0
SWARM_RESPONSE_TRACK_ALTITUDE_BOOST_METERS = 10.0
SWARM_RESPONSE_TRACK_RATE_DEGREES_PER_SECOND = 18.0
SWARM_RESPONSE_OVERWATCH_RADIUS_METERS = 18.0
SWARM_RESPONSE_OVERWATCH_ALTITUDE_BOOST_METERS = 16.0
SWARM_RESPONSE_OVERWATCH_RATE_DEGREES_PER_SECOND = 10.0
SWARM_RESPONSE_SUPPORT_DRONES = read_nonnegative_int_env(
    "REAL_SIM_RESPONSE_SUPPORT_DRONES",
    2,
)
SWARM_CANOPY_SAFE_MARGIN_METERS = max(
    9.0,
    AUTOMATION_CANOPY_CLEARANCE_METERS + 3.0,
)
SWARM_ALTITUDE_SMOOTHING_TIME_SECONDS = 1.8
SWARM_CANOPY_FOOTPRINT_EXPANSION_METERS = 4.5
SWARM_FIRE_EFFECT_UPDATE_INTERVAL_SECONDS = 1.0 / REAL_SIM_FIRE_VISUAL_HZ
SWARM_MAP_UPDATE_INTERVAL_SECONDS = 1.0 / REAL_SIM_MAP_HZ
SWARM_STATUS_UPDATE_INTERVAL_SECONDS = 1.0 / REAL_SIM_HUD_HZ
SWARM_INCIDENT_FAILURE_WINDOW_SECONDS = (
    24.0 * 60.0 / FIRE_SIM_REAL_MINUTES_PER_SECOND
)
SWARM_SUPPRESSION_ERROR_PROBABILITY = 0.05
SWARM_SUPPRESSION_DELAY_SECONDS = 6.0
SWARM_REIGNITION_RECOVERY_SECONDS = 8.0
SWARM_BASE_SUPPRESSION_TARGET = 0.98
SWARM_ADAPTIVE_PERFORMANCE_ENABLED = (
    os.environ.get("REAL_SIM_ADAPTIVE_PERFORMANCE", "0").strip().lower()
    in ("1", "true", "yes", "on")
)
SWARM_ADAPTIVE_PROFILE = os.environ.get(
    "REAL_SIM_ADAPTIVE_PROFILE",
    "performance",
).strip().lower()
if SWARM_ADAPTIVE_PROFILE not in ("performance", "strict", "lenient"):
    SWARM_ADAPTIVE_PROFILE = "performance"
SWARM_ADAPTIVE_FAILURE_WINDOW_MIN_HOURS = max(
    6.0,
    float(os.environ.get("REAL_SIM_ADAPTIVE_FAILURE_WINDOW_MIN_HOURS", "18.0")),
)
SWARM_ADAPTIVE_FAILURE_WINDOW_MAX_HOURS = max(
    SWARM_ADAPTIVE_FAILURE_WINDOW_MIN_HOURS,
    float(os.environ.get("REAL_SIM_ADAPTIVE_FAILURE_WINDOW_MAX_HOURS", "30.0")),
)
SWARM_ADAPTIVE_FAILURE_WINDOW_MIN_SECONDS = (
    SWARM_ADAPTIVE_FAILURE_WINDOW_MIN_HOURS
    * 60.0
    / FIRE_SIM_REAL_MINUTES_PER_SECOND
)
SWARM_ADAPTIVE_FAILURE_WINDOW_MAX_SECONDS = (
    SWARM_ADAPTIVE_FAILURE_WINDOW_MAX_HOURS
    * 60.0
    / FIRE_SIM_REAL_MINUTES_PER_SECOND
)
SWARM_ADAPTIVE_SUPPRESSION_TARGET_MIN = max(
    0.5,
    min(
        0.999,
        float(os.environ.get("REAL_SIM_ADAPTIVE_SUPPRESSION_TARGET_MIN", "0.94")),
    ),
)
SWARM_ADAPTIVE_SUPPRESSION_TARGET_MAX = max(
    SWARM_ADAPTIVE_SUPPRESSION_TARGET_MIN,
    min(
        0.999,
        float(os.environ.get("REAL_SIM_ADAPTIVE_SUPPRESSION_TARGET_MAX", "0.99")),
    ),
)
SWARM_ADAPTIVE_SUPPRESSION_ERROR_PROB_MIN = max(
    0.0,
    min(
        0.95,
        float(os.environ.get("REAL_SIM_ADAPTIVE_ERROR_PROB_MIN", "0.01")),
    ),
)
SWARM_ADAPTIVE_SUPPRESSION_ERROR_PROB_MAX = max(
    SWARM_ADAPTIVE_SUPPRESSION_ERROR_PROB_MIN,
    min(
        0.95,
        float(os.environ.get("REAL_SIM_ADAPTIVE_ERROR_PROB_MAX", "0.10")),
    ),
)
SWARM_ADAPTIVE_DETECTION_PROB_SCALE_MIN = max(
    0.2,
    float(os.environ.get("REAL_SIM_ADAPTIVE_DETECTION_SCALE_MIN", "0.75")),
)
SWARM_ADAPTIVE_DETECTION_PROB_SCALE_MAX = max(
    SWARM_ADAPTIVE_DETECTION_PROB_SCALE_MIN,
    float(os.environ.get("REAL_SIM_ADAPTIVE_DETECTION_SCALE_MAX", "1.20")),
)
SWARM_ADAPTIVE_GROWTH_RATE_SCALE_MIN = max(
    0.2,
    float(os.environ.get("REAL_SIM_ADAPTIVE_GROWTH_SCALE_MIN", "0.85")),
)
SWARM_ADAPTIVE_GROWTH_RATE_SCALE_MAX = max(
    SWARM_ADAPTIVE_GROWTH_RATE_SCALE_MIN,
    float(os.environ.get("REAL_SIM_ADAPTIVE_GROWTH_SCALE_MAX", "1.25")),
)
SWARM_ADAPTIVE_DETECTION_TARGET_MINUTES = max(
    30.0,
    float(os.environ.get("REAL_SIM_ADAPTIVE_DETECTION_TARGET_MINUTES", "120.0")),
)
SWARM_ADAPTIVE_DETECTION_TARGET_SECONDS = (
    SWARM_ADAPTIVE_DETECTION_TARGET_MINUTES / FIRE_SIM_REAL_MINUTES_PER_SECOND
)
SWARM_HEADLESS_FRAME_LIMIT = max(
    0,
    read_nonnegative_int_env("REAL_SIM_HEADLESS_FRAMES", 0),
)
SWARM_ALARM_SOUND_PATH = PROJECT_ROOT / "audio" / "swarm_alarm.wav"
SWARM_ALARM_COOLDOWN_SECONDS = 4.0
SWARM_DRONE_COLORS = (
    (0.92, 0.33, 0.26, 1.0),
    (0.2, 0.72, 0.95, 1.0),
    (0.98, 0.78, 0.2, 1.0),
    (0.56, 0.87, 0.44, 1.0),
    (0.8, 0.46, 0.92, 1.0),
    (0.95, 0.62, 0.32, 1.0),
)
SWARM_PLAYBACK_SPEED_MULTIPLIERS = {
    "0.5x": 0.5,
    "1x": 1.0,
    "2x": 2.0,
    "4x": 4.0,
}
SWARM_DEFAULT_PLAYBACK_LABEL = os.environ.get(
    "REAL_SIM_SWARM_DEFAULT_PLAYBACK",
    "1x",
)
if SWARM_DEFAULT_PLAYBACK_LABEL not in SWARM_PLAYBACK_SPEED_MULTIPLIERS:
    SWARM_DEFAULT_PLAYBACK_LABEL = "1x"


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
camera_angle = 0
camera_pitch = THIRD_PERSON_CAMERA_PITCH_DEGREES
camera_distance = THIRD_PERSON_CAMERA_DISTANCE
camera_height = THIRD_PERSON_CAMERA_HEIGHT
first_person_view = False
first_person_eye_height = 1.2
current_speed_preset = DRONE_SPEED_PRESET_NORMAL
dragging_camera = False
last_mouse_x = 0
last_mouse_y = 0

drones = []
trees = []
tree_canopy_volumes = []
grass_clumps = []
fire_hotspots = []
fire_burned_cells = set()
fire_burnable_cells = set()
fire_truck_marker = None
drone_sector_bounds = []
selected_drone_index = 0
current_playback_speed_label = SWARM_DEFAULT_PLAYBACK_LABEL
fire_spread_timer_seconds = 0.0
fire_spread_area_budget_sqm = 0.0
game_over_triggered = False
mission_completed = False
burn_ratio_history = deque(maxlen=360)
projected_full_burn_eta_seconds = None
alert_banner_text = ""
alert_banner_timer_seconds = 0.0
headless_frame_counter = 0
fire_effect_update_accumulator_seconds = 0.0
map_update_accumulator_seconds = 0.0
status_update_accumulator_seconds = 0.0


@dataclass
class AxisState:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class DroneMotionState:
    sim_time_seconds: float = 0.0
    input_command_buffer: deque = field(default_factory=deque)
    applied_cmd: AxisState = field(default_factory=AxisState)
    velocity: AxisState = field(default_factory=AxisState)
    visual_pitch: float = 0.0
    visual_roll: float = 0.0
    propeller_spin_degrees: float = 0.0
    last_body_right_speed: float = 0.0
    filtered_body_right_accel: float = 0.0
    roll_sway_body_speed: float = 0.0
    last_body_right_command_speed: float = 0.0
    roll_anticipation_body_speed: float = 0.0


motion_state = DroneMotionState()


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


def get_current_speed_preset_multiplier():
    return DRONE_SPEED_PRESET_MULTIPLIERS[current_speed_preset]


def get_current_drone_speed_units_per_second():
    return DRONE_SPEED_UNITS_PER_SECOND * get_current_speed_preset_multiplier()


def get_current_speed_preset_label():
    return current_speed_preset.upper()


def format_speed_mode_menu():
    return (
        f"DRONE SPEED: {get_current_speed_preset_label()} {get_current_drone_speed_units_per_second():.1f} m/s"
        f" | [Z] Slow {DRONE_SPEED_PRESET_MULTIPLIERS[DRONE_SPEED_PRESET_SLOW] * DRONE_SPEED_UNITS_PER_SECOND:.1f} m/s"
        f" | [X] Normal {DRONE_SPEED_PRESET_MULTIPLIERS[DRONE_SPEED_PRESET_NORMAL] * DRONE_SPEED_UNITS_PER_SECOND:.1f} m/s"
        f" | [C] Fast {DRONE_SPEED_PRESET_MULTIPLIERS[DRONE_SPEED_PRESET_FAST] * DRONE_SPEED_UNITS_PER_SECOND:.1f} m/s"
    )


def set_speed_preset(new_speed_preset):
    global current_speed_preset
    if new_speed_preset not in DRONE_SPEED_PRESET_MULTIPLIERS:
        return
    current_speed_preset = new_speed_preset
    if "status_text" in globals():
        update_status_overlay()


def set_slow_speed():
    set_speed_preset(DRONE_SPEED_PRESET_SLOW)


def set_normal_speed():
    set_speed_preset(DRONE_SPEED_PRESET_NORMAL)


def set_fast_speed():
    set_speed_preset(DRONE_SPEED_PRESET_FAST)


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


def set_key(key, value):
    #this is a helper fuhction
    key_map[key] = value


def apply_camera_mode():
    selected_drone = get_selected_drone()
    for drone_agent in drones:
        if first_person_view and drone_agent is selected_drone:
            drone_agent.root.hide()
        else:
            drone_agent.root.show()


@dataclass
class SectorBounds:
    drone_id: int
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
class DroneAgent:
    drone_id: int
    root: NodePath
    visual: NodePath
    model: NodePath
    propellers: tuple
    sector: SectorBounds
    motion_state: DroneMotionState = field(default_factory=DroneMotionState)
    control_mode: str = CONTROL_MODE_AUTOMATION
    detection_radius_meters: float = FIRE_HOTSPOT_DETECTION_RADIUS_METERS
    patrol_sector: SectorBounds | None = None
    patrol_waypoints: tuple = field(default_factory=tuple)
    patrol_waypoint_index: int = 0
    patrol_loop_count: int = 0
    autonomy_role: str = "patrol"
    response_angle_offset_degrees: float = 0.0
    response_anchor_sector: SectorBounds | None = None
    backfill_secondary_sector: SectorBounds | None = None
    detected_hotspots_count: int = 0
    status_note: str = "PATROL"
    target_altitude_meters: float | None = None
    trail_world_points: deque = field(
        default_factory=lambda: deque(maxlen=SWARM_TRAIL_MAX_POINTS)
    )
    trail_timer_seconds: float = 0.0
    map_marker_node: object | None = None
    map_ring_node: object | None = None
    map_trail_node: object | None = None
    map_label: object | None = None
    map_trail_dirty: bool = True


@dataclass
class IncidentState:
    ignited: bool = False
    ignition_time_seconds: float | None = None
    first_detection_time_seconds: float | None = None
    truck_arrival_time_seconds: float | None = None
    response_hotspot: FireHotspot | None = None
    responder_ids: set = field(default_factory=set)
    tracker_drone_id: int | None = None
    overwatch_drone_id: int | None = None
    backfill_drone_id: int | None = None
    permanent_markers: list = field(default_factory=list)
    current_fire_area_sqm: float = 0.0
    peak_fire_area_sqm: float = 0.0
    total_burned_area_sqm: float = 0.0
    suppression_progress: float = 0.0
    suppression_status: str = "STANDBY"
    suppression_rate_sqm_per_second: float = 0.0
    full_suppression_eta_seconds: float | None = None
    last_alarm_time_seconds: float = -999.0
    suppression_error_mode: str | None = None
    reignition_extension_seconds: float = 0.0
    reignition_triggered: bool = False
    failure_triggered: bool = False
    success_announced: bool = False
    performance_score: float = 0.0
    required_suppression_threshold: float = SWARM_BASE_SUPPRESSION_TARGET
    effective_failure_window_seconds: float = SWARM_INCIDENT_FAILURE_WINDOW_SECONDS
    effective_suppression_error_probability: float = SWARM_SUPPRESSION_ERROR_PROBABILITY
    effective_detection_probability_scale: float = 1.0
    effective_growth_rate_scale: float = 1.0


MAP_LEFT = 0.54
MAP_BOTTOM = -0.12
MAP_WIDTH = 0.74
MAP_HEIGHT = 0.74
MAP_TREE_DOT_HALF_SIZE = 0.0024
MAP_DRONE_MARKER_HALF_SIZE = 0.009
MAP_SELECTED_MARKER_SCALE = 1.35
MAP_SENSOR_RING_SEGMENTS = 28
MAP_TRAIL_THICKNESS = 2.2
MAP_FIRE_PULSE_INNER_RADIUS_METERS = 8.0
MAP_FIRE_PULSE_OUTER_RADIUS_METERS = 15.0

mission_total_area_sqm = max(
    1.0,
    (SPAWN_X_MAX - SPAWN_X_MIN) * (SPAWN_Y_MAX - SPAWN_Y_MIN),
)
coverage_total_cells = set()
coverage_scanned_cells = set()
coverage_cells_by_drone = {}
incident_state = IncidentState()
alarm_sound = None
map_root = None
map_fire_marker_root = None
status_text = None
alert_text = None
map_header_text = None
map_detected_hotspot_nodes = {}


def clamp(value, minimum_value, maximum_value):
    return max(minimum_value, min(maximum_value, value))


def format_mission_duration(seconds):
    if seconds is None:
        return "--:--"
    total_seconds = max(0, int(round(seconds)))
    hours = total_seconds // 3600
    minutes = (total_seconds % 3600) // 60
    secs = total_seconds % 60
    if hours > 0:
        return f"{hours:02d}:{minutes:02d}:{secs:02d}"
    return f"{minutes:02d}:{secs:02d}"


def format_operational_duration_from_demo_seconds(demo_seconds):
    if demo_seconds is None:
        return "--"
    operational_minutes = demo_seconds * FIRE_SIM_REAL_MINUTES_PER_SECOND
    if operational_minutes < 60.0:
        return f"{operational_minutes:.0f}m"
    if operational_minutes < 1440.0:
        return f"{operational_minutes / 60.0:.1f}h"
    return f"{operational_minutes / 1440.0:.1f}d"


def get_selected_drone():
    if not drones:
        return None
    clamped_index = clamp(selected_drone_index, 0, len(drones) - 1)
    return drones[clamped_index]


def get_drone_by_id(drone_id):
    for drone_agent in drones:
        if drone_agent.drone_id == drone_id:
            return drone_agent
    return None


def get_current_playback_multiplier():
    return SWARM_PLAYBACK_SPEED_MULTIPLIERS[current_playback_speed_label]


def set_playback_speed(label):
    global current_playback_speed_label
    if label in SWARM_PLAYBACK_SPEED_MULTIPLIERS:
        current_playback_speed_label = label


def set_half_playback_speed():
    set_playback_speed("0.5x")


def set_normal_playback_speed():
    set_playback_speed("1x")


def set_double_playback_speed():
    set_playback_speed("2x")


def set_quad_playback_speed():
    set_playback_speed("4x")


def build_drone_sectors(drone_count):
    sector_width = (SPAWN_X_MAX - SPAWN_X_MIN) / max(1, drone_count)
    sectors = []
    for index in range(drone_count):
        x_min = SPAWN_X_MIN + (sector_width * index)
        x_max = SPAWN_X_MAX if index == drone_count - 1 else x_min + sector_width
        sectors.append(
            SectorBounds(
                drone_id=index + 1,
                x_min=x_min,
                x_max=x_max,
                y_min=SPAWN_Y_MIN,
                y_max=SPAWN_Y_MAX,
                label=chr(ord("A") + index),
            )
        )
    return sectors


def compute_safe_cruise_altitude(x, y):
    desired_altitude = AUTOMATION_CRUISE_ALTITUDE_METERS
    ground_sample = sample_ground(x, y)
    if ground_sample is not None:
        desired_altitude = max(
            desired_altitude,
            ground_sample[0].z + AUTOMATION_MIN_ALTITUDE_ABOVE_GROUND_METERS,
        )

    if AUTOMATION_ENABLE_CANOPY_CLEARANCE:
        start_canopy = compute_local_canopy_ceiling(
            x,
            y,
            AUTOMATION_CANOPY_QUERY_RADIUS_METERS,
        )
        if start_canopy is not None:
            desired_altitude = max(
                desired_altitude,
                start_canopy + SWARM_CANOPY_SAFE_MARGIN_METERS,
            )
    return desired_altitude


def choose_sector_spawn(sector, preferred_positions=None):
    sector_center_x, sector_center_y = sector.center()
    sector_margin_x = min(
        SWARM_SECTOR_MARGIN_METERS,
        max(2.0, (sector.x_max - sector.x_min) * 0.22),
    )
    sector_margin_y = min(
        SWARM_SECTOR_MARGIN_METERS,
        max(2.0, (sector.y_max - sector.y_min) * 0.12),
    )
    candidate_positions = []
    if preferred_positions:
        for candidate_x, candidate_y in preferred_positions:
            candidate_positions.append(
                (
                    clamp(candidate_x, sector.x_min + sector_margin_x, sector.x_max - sector_margin_x),
                    clamp(candidate_y, sector.y_min + sector_margin_y, sector.y_max - sector_margin_y),
                )
            )
    candidate_positions.extend(
        [
            (
                sector.x_min + sector_margin_x,
                sector.y_min + sector_margin_y,
            ),
            (
                sector.x_min + sector_margin_x,
                sector.y_max - sector_margin_y,
            ),
            (
                sector.x_max - sector_margin_x,
                sector.y_min + sector_margin_y,
            ),
            (
                sector.x_max - sector_margin_x,
                sector.y_max - sector_margin_y,
            ),
            (sector_center_x, sector_center_y),
        ]
    )
    for candidate_x, candidate_y in candidate_positions:
        if not _is_position_clear_of_trees(candidate_x, candidate_y, 6.0):
            continue
        ground_sample = sample_ground(candidate_x, candidate_y)
        if ground_sample is None:
            continue
        return candidate_x, candidate_y, ground_sample[0].z

    for _ in range(150):
        candidate_x = random.uniform(
            sector.x_min + sector_margin_x,
            sector.x_max - sector_margin_x,
        )
        candidate_y = random.uniform(
            sector.y_min + sector_margin_y,
            sector.y_max - sector_margin_y,
        )
        if not _is_position_clear_of_trees(candidate_x, candidate_y, 6.0):
            continue
        ground_sample = sample_ground(candidate_x, candidate_y)
        if ground_sample is None:
            continue
        return candidate_x, candidate_y, ground_sample[0].z

    fallback_x, fallback_y = sector.center()
    fallback_ground = sample_ground(fallback_x, fallback_y)
    fallback_z = fallback_ground[0].z if fallback_ground is not None else 0.0
    return fallback_x, fallback_y, fallback_z


def build_boustrophedon_waypoints(sector):
    margin_x = min(
        SWARM_SECTOR_MARGIN_METERS,
        max(2.0, (sector.x_max - sector.x_min) * 0.18),
    )
    margin_y = min(
        SWARM_SECTOR_MARGIN_METERS,
        max(3.0, (sector.y_max - sector.y_min) * 0.08),
    )
    start_x = sector.x_min + margin_x
    end_x = sector.x_max - margin_x
    start_y = sector.y_min + margin_y
    end_y = sector.y_max - margin_y

    if end_x <= start_x:
        start_x, end_x = sector.center()[0], sector.center()[0]
    if end_y <= start_y:
        start_y, end_y = sector.center()[1], sector.center()[1]

    sweep_x_positions = []
    sweep_x = start_x
    while sweep_x <= end_x + 0.001:
        sweep_x_positions.append(sweep_x)
        sweep_x += SWARM_PATROL_SWEEP_SPACING_METERS
    if not sweep_x_positions:
        sweep_x_positions.append((start_x + end_x) * 0.5)
    elif abs(sweep_x_positions[-1] - end_x) > SWARM_PATROL_SWEEP_SPACING_METERS * 0.4:
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


def set_drone_patrol_sector(drone_agent, patrol_sector):
    if patrol_sector is None:
        patrol_sector = drone_agent.sector

    if (
        drone_agent.patrol_sector is not None
        and drone_agent.patrol_sector.label == patrol_sector.label
        and drone_agent.backfill_secondary_sector is None
    ):
        return

    drone_agent.patrol_sector = patrol_sector
    drone_agent.backfill_secondary_sector = None
    drone_agent.patrol_waypoints = build_boustrophedon_waypoints(patrol_sector)
    drone_agent.patrol_waypoint_index = 0


def set_drone_backfill_patrol(drone_agent, primary_sector, secondary_sector):
    if primary_sector is None:
        primary_sector = drone_agent.sector
    if secondary_sector is None or secondary_sector.label == primary_sector.label:
        set_drone_patrol_sector(drone_agent, primary_sector)
        return

    primary_waypoints = list(build_boustrophedon_waypoints(primary_sector))
    secondary_waypoints = list(build_boustrophedon_waypoints(secondary_sector))

    drone_x = drone_agent.root.getX()
    drone_y = drone_agent.root.getY()
    primary_waypoints = list(
        rotate_waypoints_to_nearest(primary_waypoints, drone_x, drone_y)
    )
    if primary_waypoints:
        secondary_reference_x = primary_waypoints[-1][0]
        secondary_reference_y = primary_waypoints[-1][1]
    else:
        secondary_reference_x = drone_x
        secondary_reference_y = drone_y
    secondary_waypoints = list(
        rotate_waypoints_to_nearest(
            secondary_waypoints,
            secondary_reference_x,
            secondary_reference_y,
        )
    )

    combined_waypoints = tuple(primary_waypoints + secondary_waypoints)
    if not combined_waypoints:
        center_x, center_y = primary_sector.center()
        combined_waypoints = ((center_x, center_y, 0.0),)

    drone_agent.patrol_sector = primary_sector
    drone_agent.backfill_secondary_sector = secondary_sector
    drone_agent.patrol_waypoints = combined_waypoints
    drone_agent.patrol_waypoint_index = 0


def build_tree_canopy_volumes(tree_nodes):
    canopy_volumes = []
    for tree_node in tree_nodes:
        bounds = tree_node.getTightBounds()
        if bounds is None:
            continue
        min_bound, max_bound = bounds
        canopy_half_width = max(
            abs(max_bound.x - min_bound.x),
            abs(max_bound.y - min_bound.y),
        ) * 0.5
        canopy_volumes.append(
            (
                tree_node.getX(),
                tree_node.getY(),
                max_bound.z,
                canopy_half_width,
            )
        )
    return canopy_volumes


def compute_local_canopy_ceiling(query_x, query_y, query_radius_meters):
    canopy_ceiling = None
    expanded_query_radius_sq = (
        query_radius_meters + SWARM_CANOPY_FOOTPRINT_EXPANSION_METERS
    ) ** 2

    for canopy_x, canopy_y, canopy_top_z, canopy_half_width in tree_canopy_volumes:
        dx = canopy_x - query_x
        dy = canopy_y - query_y
        effective_radius_sq = (query_radius_meters + canopy_half_width) ** 2
        if effective_radius_sq < expanded_query_radius_sq:
            effective_radius_sq = expanded_query_radius_sq
        if (dx * dx + dy * dy) > effective_radius_sq:
            continue

        if canopy_ceiling is None or canopy_top_z > canopy_ceiling:
            canopy_ceiling = canopy_top_z

    return canopy_ceiling


def build_scannable_cells():
    scan_cells = set()
    grid_width = max(1, int((SPAWN_X_MAX - SPAWN_X_MIN) / SWARM_SCAN_CELL_SIZE_METERS) + 1)
    grid_height = max(1, int((SPAWN_Y_MAX - SPAWN_Y_MIN) / SWARM_SCAN_CELL_SIZE_METERS) + 1)
    for cell_x in range(grid_width):
        for cell_y in range(grid_height):
            world_x = SPAWN_X_MIN + (cell_x + 0.5) * SWARM_SCAN_CELL_SIZE_METERS
            world_y = SPAWN_Y_MIN + (cell_y + 0.5) * SWARM_SCAN_CELL_SIZE_METERS
            if sample_ground(world_x, world_y) is None:
                continue
            scan_cells.add((cell_x, cell_y))
    return scan_cells


def world_to_scan_cell(x, y):
    scan_width = max(1, int((SPAWN_X_MAX - SPAWN_X_MIN) / SWARM_SCAN_CELL_SIZE_METERS) + 1)
    scan_height = max(1, int((SPAWN_Y_MAX - SPAWN_Y_MIN) / SWARM_SCAN_CELL_SIZE_METERS) + 1)
    cell_x = int((x - SPAWN_X_MIN) / SWARM_SCAN_CELL_SIZE_METERS)
    cell_y = int((y - SPAWN_Y_MIN) / SWARM_SCAN_CELL_SIZE_METERS)
    return (
        clamp(cell_x, 0, scan_width - 1),
        clamp(cell_y, 0, scan_height - 1),
    )


def scan_cell_center(cell_x, cell_y):
    return (
        SPAWN_X_MIN + (cell_x + 0.5) * SWARM_SCAN_CELL_SIZE_METERS,
        SPAWN_Y_MIN + (cell_y + 0.5) * SWARM_SCAN_CELL_SIZE_METERS,
    )


def build_drone_swarm():
    drone_agents = []
    drone_model_prototype = app.loader.loadModel(
        asset_path("models/Drone_Costum/Material/drone_costum.obj")
    )

    for sector in drone_sector_bounds:
        drone_root = app.render.attachNewNode(f"drone_{sector.drone_id}")
        drone_visual = drone_root.attachNewNode(f"drone_visual_{sector.drone_id}")
        drone_model = drone_model_prototype.copyTo(drone_visual)
        drone_model.setScale(DRONE_MODEL_SCALE)
        drone_model.setP(DRONE_BASE_PITCH_DEGREES)
        drone_model.setH(DRONE_MODEL_HEADING_OFFSET_DEGREES)
        color = SWARM_DRONE_COLORS[(sector.drone_id - 1) % len(SWARM_DRONE_COLORS)]
        drone_model.setColorScale(*color)
        drone_propellers_local = tuple(build_drone_propellers(drone_model))
        patrol_waypoints = build_boustrophedon_waypoints(sector)
        preferred_spawn_positions = []
        if patrol_waypoints:
            preferred_spawn_positions.append((patrol_waypoints[0][0], patrol_waypoints[0][1]))
        if len(patrol_waypoints) > 1:
            preferred_spawn_positions.append((patrol_waypoints[1][0], patrol_waypoints[1][1]))
        spawn_x, spawn_y, spawn_z = choose_sector_spawn(
            sector,
            preferred_positions=preferred_spawn_positions,
        )
        drone_root.setPos(spawn_x, spawn_y, compute_safe_cruise_altitude(spawn_x, spawn_y))
        if len(patrol_waypoints) > 1:
            heading_dx = patrol_waypoints[1][0] - patrol_waypoints[0][0]
            heading_dy = patrol_waypoints[1][1] - patrol_waypoints[0][1]
            if abs(heading_dx) > 0.001 or abs(heading_dy) > 0.001:
                drone_root.setH(
                    degrees(
                        atan2(
                            heading_dx * DRONE_HEADING_X_SIGN,
                            heading_dy * DRONE_HEADING_Y_SIGN,
                        )
                    )
                )
            else:
                drone_root.setH(random.uniform(0.0, 360.0))
        else:
            drone_root.setH(random.uniform(0.0, 360.0))
        drone_agent = DroneAgent(
            drone_id=sector.drone_id,
            root=drone_root,
            visual=drone_visual,
            model=drone_model,
            propellers=drone_propellers_local,
            sector=sector,
            control_mode=CONTROL_MODE_AUTOMATION,
            patrol_sector=sector,
            patrol_waypoints=patrol_waypoints,
            response_angle_offset_degrees=((sector.drone_id - 1) * 120.0) % 360.0,
            response_anchor_sector=sector,
            target_altitude_meters=drone_root.getZ(),
        )
        drone_agent.trail_world_points.append((spawn_x, spawn_y))
        drone_agents.append(drone_agent)

    return drone_agents


def world_to_map_coords(x, y):
    x_fraction = 0.0
    y_fraction = 0.0
    if abs(SPAWN_X_MAX - SPAWN_X_MIN) > 0.001:
        x_fraction = (x - SPAWN_X_MIN) / (SPAWN_X_MAX - SPAWN_X_MIN)
    if abs(SPAWN_Y_MAX - SPAWN_Y_MIN) > 0.001:
        y_fraction = (y - SPAWN_Y_MIN) / (SPAWN_Y_MAX - SPAWN_Y_MIN)
    return (x_fraction * MAP_WIDTH, y_fraction * MAP_HEIGHT)


def make_map_square(node_name, half_size, color_rgba):
    card_maker = CardMaker(node_name)
    card_maker.setFrame(-half_size, half_size, -half_size, half_size)
    node = map_root.attachNewNode(card_maker.generate())
    node.setColor(*color_rgba)
    node.setTransparency(TransparencyAttrib.MAlpha)
    node.setDepthWrite(False)
    return node


def build_map_circle(radius_meters, color_rgba, parent=None, thickness=1.3):
    line = LineSegs("sensor_ring")
    line.setThickness(thickness)
    line.setColor(*color_rgba)
    radius_x = MAP_WIDTH * (radius_meters / max(1.0, (SPAWN_X_MAX - SPAWN_X_MIN)))
    radius_y = MAP_HEIGHT * (radius_meters / max(1.0, (SPAWN_Y_MAX - SPAWN_Y_MIN)))
    for segment_index in range(MAP_SENSOR_RING_SEGMENTS + 1):
        angle = tau * (segment_index / MAP_SENSOR_RING_SEGMENTS)
        px = cos(angle) * radius_x
        py = sin(angle) * radius_y
        if segment_index == 0:
            line.moveTo(px, 0.0, py)
        else:
            line.drawTo(px, 0.0, py)
    target_parent = map_root if parent is None else parent
    return target_parent.attachNewNode(line.create())


def build_cross_marker(parent, color_rgba, half_size=0.012, thickness=2.8):
    line = LineSegs("fire_marker")
    line.setThickness(thickness)
    line.setColor(*color_rgba)
    line.moveTo(-half_size, 0.0, -half_size)
    line.drawTo(half_size, 0.0, half_size)
    line.moveTo(-half_size, 0.0, half_size)
    line.drawTo(half_size, 0.0, -half_size)
    marker = parent.attachNewNode(line.create())
    marker.setTransparency(TransparencyAttrib.MAlpha)
    marker.setDepthTest(False)
    marker.setDepthWrite(False)
    marker.setBin("fixed", 120)
    return marker


def build_fire_marker_node(world_x, world_y, color_rgba):
    marker = build_cross_marker(map_fire_marker_root, color_rgba)
    map_x, map_y = world_to_map_coords(world_x, world_y)
    marker.setPos(map_x, 0.0, map_y)
    return marker


def build_detected_hotspot_map_node():
    root = map_fire_marker_root.attachNewNode("detected_hotspot_marker")
    root.setBin("fixed", 110)
    root.setDepthTest(False)
    root.setDepthWrite(False)

    cross_node = build_cross_marker(
        root,
        (1.0, 0.3, 0.14, 1.0),
        half_size=0.014,
        thickness=3.6,
    )
    inner_ring = build_map_circle(
        MAP_FIRE_PULSE_INNER_RADIUS_METERS,
        (1.0, 0.68, 0.22, 0.82),
        parent=root,
        thickness=2.0,
    )
    outer_ring = build_map_circle(
        MAP_FIRE_PULSE_OUTER_RADIUS_METERS,
        (1.0, 0.28, 0.12, 0.56),
        parent=root,
        thickness=2.2,
    )
    root.setPythonTag("cross_node", cross_node)
    root.setPythonTag("inner_ring", inner_ring)
    root.setPythonTag("outer_ring", outer_ring)
    return root


def rebuild_trail_node(drone_agent):
    if drone_agent.map_trail_node is not None:
        drone_agent.map_trail_node.removeNode()
        drone_agent.map_trail_node = None
    if len(drone_agent.trail_world_points) < 2:
        drone_agent.map_trail_dirty = False
        return

    trail_line = LineSegs(f"drone_trail_{drone_agent.drone_id}")
    trail_line.setThickness(MAP_TRAIL_THICKNESS)
    color = SWARM_DRONE_COLORS[(drone_agent.drone_id - 1) % len(SWARM_DRONE_COLORS)]
    trail_line.setColor(color[0], color[1], color[2], 0.8)
    first_x, first_y = drone_agent.trail_world_points[0]
    map_x, map_y = world_to_map_coords(first_x, first_y)
    trail_line.moveTo(map_x, 0.0, map_y)
    for world_x, world_y in list(drone_agent.trail_world_points)[1:]:
        map_x, map_y = world_to_map_coords(world_x, world_y)
        trail_line.drawTo(map_x, 0.0, map_y)
    drone_agent.map_trail_node = map_root.attachNewNode(trail_line.create())
    drone_agent.map_trail_dirty = False


def build_map_overlay():
    global map_root, map_fire_marker_root, map_header_text, map_detected_hotspot_nodes
    map_detected_hotspot_nodes = {}
    map_root = app.aspect2d.attachNewNode("operator_map_root")
    map_root.setPos(MAP_LEFT, 0.0, MAP_BOTTOM)

    background = CardMaker("operator_map_background")
    background.setFrame(0.0, MAP_WIDTH, 0.0, MAP_HEIGHT)
    background_np = map_root.attachNewNode(background.generate())
    background_np.setColor(0.08, 0.16, 0.12, 0.88)
    background_np.setTransparency(TransparencyAttrib.MAlpha)

    sector_lines = LineSegs("map_sectors")
    sector_lines.setThickness(1.5)
    sector_lines.setColor(0.92, 0.92, 0.92, 0.7)
    for sector in drone_sector_bounds:
        x0, y0 = world_to_map_coords(sector.x_min, sector.y_min)
        x1, y1 = world_to_map_coords(sector.x_max, sector.y_max)
        sector_lines.moveTo(x0, 0.0, y0)
        sector_lines.drawTo(x1, 0.0, y0)
        sector_lines.drawTo(x1, 0.0, y1)
        sector_lines.drawTo(x0, 0.0, y1)
        sector_lines.drawTo(x0, 0.0, y0)
        sector_label = OnscreenText(
            text=f"S{sector.label}",
            parent=map_root,
            pos=(x0 + (x1 - x0) * 0.5, y1 - 0.028),
            scale=0.03,
            fg=(1.0, 1.0, 1.0, 0.85),
            align=TextNode.ACenter,
            mayChange=False,
        )
        sector_label.setBin("fixed", 100)
    map_root.attachNewNode(sector_lines.create())

    for tree_node in trees:
        tree_x, tree_y = world_to_map_coords(tree_node.getX(), tree_node.getY())
        tree_dot = CardMaker(f"tree_dot_{tree_x:.3f}_{tree_y:.3f}")
        tree_dot.setFrame(
            -MAP_TREE_DOT_HALF_SIZE,
            MAP_TREE_DOT_HALF_SIZE,
            -MAP_TREE_DOT_HALF_SIZE,
            MAP_TREE_DOT_HALF_SIZE,
        )
        tree_np = map_root.attachNewNode(tree_dot.generate())
        tree_np.setPos(tree_x, 0.0, tree_y)
        tree_np.setColor(0.2, 0.44, 0.2, 0.85)
        tree_np.setTransparency(TransparencyAttrib.MAlpha)
        tree_np.setDepthWrite(False)

    map_fire_marker_root = map_root.attachNewNode("map_fire_markers")
    map_header_text = OnscreenText(
        text="Operator Map",
        parent=map_root,
        pos=(MAP_WIDTH * 0.5, MAP_HEIGHT + 0.035),
        scale=0.04,
        fg=(1.0, 1.0, 1.0, 0.96),
        align=TextNode.ACenter,
        mayChange=False,
    )

    for drone_agent in drones:
        color = SWARM_DRONE_COLORS[(drone_agent.drone_id - 1) % len(SWARM_DRONE_COLORS)]
        drone_agent.map_marker_node = make_map_square(
            f"drone_map_marker_{drone_agent.drone_id}",
            MAP_DRONE_MARKER_HALF_SIZE,
            color,
        )
        drone_agent.map_ring_node = build_map_circle(
            drone_agent.detection_radius_meters,
            (color[0], color[1], color[2], 0.36),
        )
        drone_agent.map_label = OnscreenText(
            text=f"D{drone_agent.drone_id}",
            parent=map_root,
            pos=(0.0, 0.0),
            scale=0.03,
            fg=(1.0, 1.0, 1.0, 0.95),
            align=TextNode.ALeft,
            mayChange=True,
        )
        rebuild_trail_node(drone_agent)

    for marker_x, marker_y, marker_label in incident_state.permanent_markers:
        marker_node = build_fire_marker_node(
            marker_x,
            marker_y,
            (1.0, 0.22, 0.14, 1.0),
        )
        marker_text = OnscreenText(
            text=marker_label,
            parent=map_fire_marker_root,
            pos=(
                world_to_map_coords(marker_x, marker_y)[0] + 0.018,
                world_to_map_coords(marker_x, marker_y)[1] + 0.012,
            ),
            scale=0.025,
            fg=(1.0, 0.92, 0.9, 0.96),
            align=TextNode.ALeft,
            mayChange=False,
        )
        marker_text.setBin("fixed", 120)
        marker_text.setDepthTest(False)
        marker_text.setDepthWrite(False)
        marker_node.setPythonTag("label_text", marker_text)


def add_permanent_fire_marker(hotspot, detector_drone):
    marker_label = f"F{len(incident_state.permanent_markers) + 1}"
    incident_state.permanent_markers.append(
        (hotspot.root.getX(), hotspot.root.getY(), marker_label)
    )
    marker_node = build_fire_marker_node(
        hotspot.root.getX(),
        hotspot.root.getY(),
        (1.0, 0.22, 0.14, 1.0),
    )
    marker_text = OnscreenText(
        text=f"{marker_label}/D{detector_drone.drone_id}",
        parent=map_fire_marker_root,
        pos=(
            world_to_map_coords(hotspot.root.getX(), hotspot.root.getY())[0] + 0.018,
            world_to_map_coords(hotspot.root.getX(), hotspot.root.getY())[1] + 0.012,
        ),
        scale=0.025,
        fg=(1.0, 0.92, 0.9, 0.96),
        align=TextNode.ALeft,
        mayChange=False,
    )
    marker_node.setBin("fixed", 120)
    marker_node.setDepthTest(False)
    marker_node.setDepthWrite(False)
    marker_text.setBin("fixed", 120)
    marker_text.setDepthTest(False)
    marker_text.setDepthWrite(False)
    marker_node.setPythonTag("label_text", marker_text)


def update_detected_hotspot_map_nodes():
    if map_fire_marker_root is None:
        return

    active_detected_hotspots = [
        hotspot for hotspot in fire_hotspots if hotspot.detected
    ]
    active_hotspot_ids = {id(hotspot) for hotspot in active_detected_hotspots}

    stale_hotspot_ids = [
        hotspot_id
        for hotspot_id in list(map_detected_hotspot_nodes.keys())
        if hotspot_id not in active_hotspot_ids
    ]
    for hotspot_id in stale_hotspot_ids:
        node = map_detected_hotspot_nodes.pop(hotspot_id, None)
        if node is not None:
            node.removeNode()

    for hotspot in active_detected_hotspots:
        hotspot_id = id(hotspot)
        marker_node = map_detected_hotspot_nodes.get(hotspot_id)
        if marker_node is None:
            marker_node = build_detected_hotspot_map_node()
            map_detected_hotspot_nodes[hotspot_id] = marker_node

        map_x, map_y = world_to_map_coords(hotspot.root.getX(), hotspot.root.getY())
        marker_node.setPos(map_x, 0.0, map_y)
        pulse_phase = motion_state.sim_time_seconds * 2.5
        pulse_scale = 1.0 + (sin(pulse_phase) * 0.18)
        current_area_scale = 1.0 + (
            clamp(incident_state.current_fire_area_sqm / 22.0, 0.0, 2.0) * 0.42
        )

        cross_node = marker_node.getPythonTag("cross_node")
        inner_ring = marker_node.getPythonTag("inner_ring")
        outer_ring = marker_node.getPythonTag("outer_ring")

        if hotspot.suppression_state == FIRE_STATE_ACTIVE:
            cross_node.setColorScale(1.0, 1.0, 1.0, 1.0)
            inner_ring.setColorScale(1.0, 1.0, 1.0, 1.0)
            outer_ring.setColorScale(1.0, 1.0, 1.0, 1.0)
            cross_node.setScale(1.28 * pulse_scale)
            inner_ring.setScale(0.92 + (pulse_scale * 0.26))
            outer_ring.setScale(current_area_scale * pulse_scale)
        elif hotspot.suppression_state == FIRE_STATE_CONTAINED:
            cross_node.setColorScale(1.0, 0.86, 0.5, 0.95)
            inner_ring.setColorScale(1.0, 0.82, 0.45, 0.85)
            outer_ring.setColorScale(1.0, 0.72, 0.38, 0.7)
            cross_node.setScale(1.12)
            inner_ring.setScale(0.94)
            outer_ring.setScale(1.08)
        elif hotspot.suppression_state == FIRE_STATE_OUT:
            cross_node.setColorScale(0.52, 0.98, 0.58, 0.95)
            inner_ring.setColorScale(0.5, 0.92, 0.55, 0.8)
            outer_ring.setColorScale(0.42, 0.82, 0.5, 0.6)
            cross_node.setScale(1.0)
            inner_ring.setScale(0.86)
            outer_ring.setScale(0.86)
        else:
            cross_node.setColorScale(0.88, 0.26, 0.22, 0.92)
            inner_ring.setColorScale(0.8, 0.24, 0.2, 0.68)
            outer_ring.setColorScale(0.7, 0.18, 0.15, 0.5)
            cross_node.setScale(0.94)
            inner_ring.setScale(0.78)
            outer_ring.setScale(0.78)


def update_map_overlay():
    selected_drone = get_selected_drone()
    for drone_agent in drones:
        map_x, map_y = world_to_map_coords(
            drone_agent.root.getX(),
            drone_agent.root.getY(),
        )
        if drone_agent.map_marker_node is not None:
            drone_agent.map_marker_node.setPos(map_x, 0.0, map_y)
            marker_scale = (
                MAP_SELECTED_MARKER_SCALE
                if drone_agent is selected_drone
                else 1.0
            )
            drone_agent.map_marker_node.setScale(marker_scale, 1.0, marker_scale)
        if drone_agent.map_ring_node is not None:
            drone_agent.map_ring_node.setPos(map_x, 0.0, map_y)
        if drone_agent.map_label is not None:
            drone_agent.map_label.setPos(map_x + 0.016, map_y + 0.012)
            label_color = (
                (1.0, 1.0, 0.78, 1.0)
                if drone_agent is selected_drone
                else (1.0, 1.0, 1.0, 0.95)
            )
            role_code = "P"
            if drone_agent.control_mode == CONTROL_MODE_MANUAL:
                role_code = "M"
            elif drone_agent.autonomy_role == "track":
                role_code = "T"
            elif drone_agent.autonomy_role == "overwatch":
                role_code = "O"
            elif drone_agent.autonomy_role == "backfill":
                role_code = "B"
            drone_agent.map_label.setFg(label_color)
            drone_agent.map_label.setText(
                f"D{drone_agent.drone_id} {role_code}"
            )
        if drone_agent.map_trail_dirty:
            rebuild_trail_node(drone_agent)
    update_detected_hotspot_map_nodes()


def initialize_world_state():
    global trees, tree_canopy_volumes, grass_clumps, fire_truck_marker, fire_burnable_cells
    global coverage_total_cells, coverage_cells_by_drone, coverage_scanned_cells, drones
    global drone_sector_bounds, alarm_sound

    tree_prototypes = build_tree_prototypes()
    grass_prototypes = build_grass_prototypes()
    trees = spawn_trees(tree_prototypes)
    tree_canopy_volumes = build_tree_canopy_volumes(trees)
    drone_sector_bounds = build_drone_sectors(SWARM_DRONE_COUNT)
    drones = build_drone_swarm()
    grass_clumps = spawn_grass(grass_prototypes)
    fire_truck_marker = spawn_fire_truck_marker()
    fire_truck_marker.hide()
    fire_burnable_cells = build_fire_burnable_cells()
    coverage_total_cells = build_scannable_cells()
    coverage_scanned_cells.clear()
    coverage_cells_by_drone = {
        drone_agent.drone_id: set()
        for drone_agent in drones
    }
    if SWARM_ALARM_SOUND_PATH.exists():
        try:
            alarm_sound = app.loader.loadSfx(str(SWARM_ALARM_SOUND_PATH))
            if alarm_sound is not None:
                alarm_sound.setVolume(0.9)
        except Exception:
            alarm_sound = None


def apply_thermal_view():
    app.setBackgroundColor(
        *(THERMAL_BACKGROUND_COLOR if thermal_view_enabled else DEFAULT_BACKGROUND_COLOR)
    )

    terrain_color = (0.18, 0.24, 0.34, 1.0)
    foliage_color = (0.12, 0.2, 0.18, 1.0)
    if thermal_view_enabled:
        ground.setColorScale(*terrain_color)
        for tree_node in trees:
            tree_node.setColorScale(*foliage_color)
        for grass_node in grass_clumps:
            if THERMAL_HIDE_GRASS:
                grass_node.hide()
            else:
                grass_node.setColorScale(*foliage_color)
        for drone_agent in drones:
            drone_agent.root.setColorScale(0.92, 0.95, 0.98, 1.0)
    else:
        ground.clearColorScale()
        for tree_node in trees:
            tree_node.clearColorScale()
        for grass_node in grass_clumps:
            grass_node.show()
            grass_node.clearColorScale()
        for drone_agent in drones:
            drone_agent.root.clearColorScale()

    for hotspot in fire_hotspots:
        set_hotspot_visual(hotspot)

    apply_camera_mode()


def set_thermal_view(enabled):
    global thermal_view_enabled
    thermal_view_enabled = bool(enabled)
    apply_thermal_view()


def toggle_thermal_view():
    set_thermal_view(not thermal_view_enabled)


def set_first_person_mode():
    global first_person_view
    first_person_view = True
    apply_camera_mode()


def set_third_person_mode():
    global first_person_view
    first_person_view = False
    apply_camera_mode()


def toggle_camera_mode():
    if first_person_view:
        set_third_person_mode()
    else:
        set_first_person_mode()


def reset_drone_manual_state(drone_agent):
    drone_agent.motion_state.input_command_buffer.clear()
    drone_agent.motion_state.applied_cmd = AxisState()
    drone_agent.motion_state.last_body_right_speed = 0.0
    drone_agent.motion_state.filtered_body_right_accel = 0.0
    drone_agent.motion_state.roll_sway_body_speed = 0.0
    drone_agent.motion_state.last_body_right_command_speed = 0.0
    drone_agent.motion_state.roll_anticipation_body_speed = 0.0


def set_selected_drone(index):
    global selected_drone_index
    if 0 <= index < len(drones):
        selected_drone_index = index
        apply_camera_mode()


def set_selected_drone_control_mode(new_mode):
    selected_drone = get_selected_drone()
    if selected_drone is None:
        return
    if new_mode not in (CONTROL_MODE_MANUAL, CONTROL_MODE_AUTOMATION):
        return
    selected_drone.control_mode = new_mode
    selected_drone.status_note = "MANUAL" if new_mode == CONTROL_MODE_MANUAL else "PATROL"
    if new_mode == CONTROL_MODE_AUTOMATION:
        set_drone_patrol_sector(selected_drone, selected_drone.sector)
        selected_drone.autonomy_role = "patrol"
    reset_drone_manual_state(selected_drone)

    if incident_state.response_hotspot is not None:
        reference_detector = get_drone_by_id(
            getattr(incident_state.response_hotspot, "detected_by_drone_id", None)
        )
        if reference_detector is not None:
            assign_response_drones(reference_detector, incident_state.response_hotspot)


def set_selected_drone_manual():
    set_selected_drone_control_mode(CONTROL_MODE_MANUAL)


def set_selected_drone_automation():
    set_selected_drone_control_mode(CONTROL_MODE_AUTOMATION)


def toggle_selected_drone_control_mode():
    selected_drone = get_selected_drone()
    if selected_drone is None:
        return
    if selected_drone.control_mode == CONTROL_MODE_MANUAL:
        set_selected_drone_automation()
    else:
        set_selected_drone_manual()


def bind_key_pairs():
    key_bindings = {
        "a": "left",
        "d": "right",
        "w": "forward",
        "s": "backward",
        "q": "up",
        "e": "down",
    }
    for keyboard_key, action in key_bindings.items():
        app.accept(keyboard_key, set_key, [action, True])
        app.accept(f"{keyboard_key}-up", set_key, [action, False])


def start_camera_drag():
    global dragging_camera, last_mouse_x, last_mouse_y
    dragging_camera = True
    if app.mouseWatcherNode.hasMouse():
        last_mouse_x = app.mouseWatcherNode.getMouseX()
        last_mouse_y = app.mouseWatcherNode.getMouseY()


def stop_camera_drag():
    global dragging_camera
    dragging_camera = False


def choose_incident_fire_spawn():
    target_sector = drone_sector_bounds[0]
    center_x, center_y = target_sector.center()
    candidate_positions = (
        (center_x, center_y),
        (
            target_sector.x_min + (target_sector.x_max - target_sector.x_min) * 0.35,
            center_y,
        ),
        (
            target_sector.x_min + (target_sector.x_max - target_sector.x_min) * 0.65,
            center_y,
        ),
    )
    for candidate_x, candidate_y in candidate_positions:
        if not _is_position_clear_of_trees(
            candidate_x,
            candidate_y,
            FIRE_HOTSPOT_TREE_CLEARANCE_METERS,
        ):
            continue
        ground_sample = sample_ground(candidate_x, candidate_y)
        if ground_sample is None:
            continue
        return candidate_x, candidate_y, ground_sample[0].z
    for _ in range(FIRE_HOTSPOT_SPAWN_ATTEMPTS):
        candidate_x = random.uniform(
            target_sector.x_min + SWARM_SECTOR_MARGIN_METERS,
            target_sector.x_max - SWARM_SECTOR_MARGIN_METERS,
        )
        candidate_y = random.uniform(
            target_sector.y_min + SWARM_SECTOR_MARGIN_METERS,
            target_sector.y_max - SWARM_SECTOR_MARGIN_METERS,
        )
        if not _is_position_clear_of_trees(
            candidate_x,
            candidate_y,
            FIRE_HOTSPOT_TREE_CLEARANCE_METERS,
        ):
            continue
        ground_sample = sample_ground(candidate_x, candidate_y)
        if ground_sample is None:
            continue
        return candidate_x, candidate_y, ground_sample[0].z
    return choose_sector_spawn(target_sector)


def emit_alert(message, duration_seconds=4.0):
    global alert_banner_text, alert_banner_timer_seconds
    alert_banner_text = message
    alert_banner_timer_seconds = max(alert_banner_timer_seconds, duration_seconds)
    if alert_text is not None:
        alert_text.setText(alert_banner_text)


def play_detection_alarm(current_time_seconds):
    if (
        current_time_seconds - incident_state.last_alarm_time_seconds
    ) < SWARM_ALARM_COOLDOWN_SECONDS:
        return
    incident_state.last_alarm_time_seconds = current_time_seconds
    if alarm_sound is not None:
        try:
            alarm_sound.stop()
            alarm_sound.play()
        except Exception:
            pass
    if SWARM_ALARM_SOUND_PATH.exists() and os.uname().sysname == "Darwin":
        try:
            subprocess.Popen(
                ["afplay", str(SWARM_ALARM_SOUND_PATH)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception:
            pass


def get_patrol_target(drone_agent):
    if not drone_agent.patrol_waypoints:
        patrol_sector = drone_agent.patrol_sector or drone_agent.sector
        center_x, center_y = patrol_sector.center()
        return center_x, center_y, 0.0

    target = drone_agent.patrol_waypoints[drone_agent.patrol_waypoint_index]
    dx = target[0] - drone_agent.root.getX()
    dy = target[1] - drone_agent.root.getY()
    if (dx * dx + dy * dy) ** 0.5 <= max(2.0, AUTOMATION_TARGET_REACH_RADIUS_METERS * 1.8):
        drone_agent.patrol_waypoint_index += 1
        if drone_agent.patrol_waypoint_index >= len(drone_agent.patrol_waypoints):
            drone_agent.patrol_waypoint_index = 0
            drone_agent.patrol_loop_count += 1
        target = drone_agent.patrol_waypoints[drone_agent.patrol_waypoint_index]
    return target


def get_incident_focus():
    tracked_hotspots = [
        hotspot
        for hotspot in fire_hotspots
        if hotspot.suppression_state in (FIRE_STATE_ACTIVE, FIRE_STATE_CONTAINED)
        and hotspot.detected
    ]
    if not tracked_hotspots and incident_state.response_hotspot is not None:
        tracked_hotspots = [incident_state.response_hotspot]
    if not tracked_hotspots:
        return None

    avg_x = sum(hotspot.root.getX() for hotspot in tracked_hotspots) / len(tracked_hotspots)
    avg_y = sum(hotspot.root.getY() for hotspot in tracked_hotspots) / len(tracked_hotspots)
    avg_z = sum(hotspot.ground_z for hotspot in tracked_hotspots) / len(tracked_hotspots)
    return avg_x, avg_y, avg_z


def assign_response_drones(detector_drone, hotspot):
    incident_state.response_hotspot = hotspot
    auto_drones = [
        drone_agent
        for drone_agent in drones
        if drone_agent.control_mode == CONTROL_MODE_AUTOMATION
    ]

    tracker_drone = detector_drone if detector_drone in auto_drones else None
    if tracker_drone is None and auto_drones:
        tracker_drone = min(
            auto_drones,
            key=lambda drone_agent: (
                (drone_agent.root.getX() - hotspot.root.getX()) ** 2
                + (drone_agent.root.getY() - hotspot.root.getY()) ** 2
            ),
        )

    remaining_auto_drones = [
        drone_agent
        for drone_agent in auto_drones
        if tracker_drone is None or drone_agent.drone_id != tracker_drone.drone_id
    ]
    remaining_auto_drones.sort(
        key=lambda drone_agent: (
            (drone_agent.root.getX() - hotspot.root.getX()) ** 2
            + (drone_agent.root.getY() - hotspot.root.getY()) ** 2
        )
    )

    support_count = min(SWARM_RESPONSE_SUPPORT_DRONES, len(remaining_auto_drones))
    overwatch_drone = None
    backfill_drone = None

    # Demo policy requested by user:
    # - If D1 detects, D2 responds and D3 backfills D2's sector.
    if detector_drone.drone_id == 1:
        preferred_overwatch = next(
            (drone_agent for drone_agent in remaining_auto_drones if drone_agent.drone_id == 2),
            None,
        )
        preferred_backfill = next(
            (drone_agent for drone_agent in remaining_auto_drones if drone_agent.drone_id == 3),
            None,
        )
        if support_count >= 1:
            overwatch_drone = preferred_overwatch or remaining_auto_drones[0]
        if support_count >= 2:
            remaining_for_backfill = [
                drone_agent
                for drone_agent in remaining_auto_drones
                if overwatch_drone is None or drone_agent.drone_id != overwatch_drone.drone_id
            ]
            backfill_drone = preferred_backfill
            if (
                backfill_drone is None
                or (
                    overwatch_drone is not None
                    and backfill_drone.drone_id == overwatch_drone.drone_id
                )
            ):
                backfill_drone = remaining_for_backfill[0] if remaining_for_backfill else None
    else:
        if support_count >= 1:
            overwatch_drone = remaining_auto_drones[0]
        if support_count >= 2:
            remaining_for_backfill = [
                drone_agent
                for drone_agent in remaining_auto_drones
                if overwatch_drone is None or drone_agent.drone_id != overwatch_drone.drone_id
            ]
            backfill_drone = remaining_for_backfill[0] if remaining_for_backfill else None

    responder_ids = set()
    if tracker_drone is not None:
        responder_ids.add(tracker_drone.drone_id)
    if overwatch_drone is not None:
        responder_ids.add(overwatch_drone.drone_id)

    incident_state.responder_ids = responder_ids
    incident_state.tracker_drone_id = (
        tracker_drone.drone_id if tracker_drone is not None else None
    )
    incident_state.overwatch_drone_id = (
        overwatch_drone.drone_id if overwatch_drone is not None else None
    )
    incident_state.backfill_drone_id = (
        backfill_drone.drone_id if backfill_drone is not None else None
    )

    for drone_agent in drones:
        drone_agent.response_anchor_sector = detector_drone.sector
        if drone_agent.control_mode == CONTROL_MODE_MANUAL:
            drone_agent.status_note = "MANUAL"
            continue

        set_drone_patrol_sector(drone_agent, drone_agent.sector)
        drone_agent.autonomy_role = "patrol"
        drone_agent.status_note = "PATROL"

        if tracker_drone is not None and drone_agent.drone_id == tracker_drone.drone_id:
            drone_agent.autonomy_role = "track"
            drone_agent.status_note = "TRACK"
        elif overwatch_drone is not None and drone_agent.drone_id == overwatch_drone.drone_id:
            drone_agent.autonomy_role = "overwatch"
            drone_agent.status_note = "OVERWATCH"
        elif backfill_drone is not None and drone_agent.drone_id == backfill_drone.drone_id:
            backfill_target_sector = (
                overwatch_drone.sector if overwatch_drone is not None else detector_drone.sector
            )
            set_drone_backfill_patrol(
                drone_agent,
                drone_agent.sector,
                backfill_target_sector,
            )
            drone_agent.autonomy_role = "backfill"
            if backfill_target_sector.label == drone_agent.sector.label:
                drone_agent.status_note = f"BACKFILL {drone_agent.sector.label}"
            else:
                drone_agent.status_note = (
                    f"BACKFILL {drone_agent.sector.label}+{backfill_target_sector.label}"
                )


def clear_response_assignment():
    incident_state.responder_ids.clear()
    incident_state.response_hotspot = None
    incident_state.tracker_drone_id = None
    incident_state.overwatch_drone_id = None
    incident_state.backfill_drone_id = None
    for drone_agent in drones:
        if drone_agent.control_mode == CONTROL_MODE_AUTOMATION:
            set_drone_patrol_sector(drone_agent, drone_agent.sector)
            drone_agent.autonomy_role = "patrol"
            drone_agent.status_note = "PATROL"


def register_hotspot_detection(hotspot, detector_drone):
    current_time_seconds = motion_state.sim_time_seconds
    hotspot.detected = True
    hotspot.detection_age_seconds = 0.0
    hotspot.detected_by_drone_id = detector_drone.drone_id
    detector_drone.detected_hotspots_count += 1
    set_hotspot_visual(hotspot)

    already_marked = any(
        abs(marker_x - hotspot.root.getX()) < 1.0 and abs(marker_y - hotspot.root.getY()) < 1.0
        for marker_x, marker_y, _ in incident_state.permanent_markers
    )
    if not already_marked:
        add_permanent_fire_marker(hotspot, detector_drone)

    if incident_state.first_detection_time_seconds is None:
        controls = compute_performance_controls(current_time_seconds)
        incident_state.performance_score = controls["performance_score"]
        incident_state.required_suppression_threshold = controls["required_suppression_threshold"]
        incident_state.effective_failure_window_seconds = controls["effective_failure_window_seconds"]
        incident_state.effective_suppression_error_probability = controls["suppression_error_probability"]
        incident_state.effective_detection_probability_scale = controls["detection_probability_scale"]
        incident_state.effective_growth_rate_scale = controls["growth_rate_scale"]
        incident_state.first_detection_time_seconds = current_time_seconds
        incident_state.truck_arrival_time_seconds = (
            current_time_seconds + SWARM_CASE2_TRUCK_ETA_SECONDS
        )
        if random.random() < incident_state.effective_suppression_error_probability:
            incident_state.suppression_error_mode = random.choice(("delay", "reignite"))

    assign_response_drones(detector_drone, hotspot)
    emit_alert(
        f"THERMAL ALERT: Drone {detector_drone.drone_id} detected fire in Sector {detector_drone.sector.label}",
        duration_seconds=5.5,
    )
    play_detection_alarm(current_time_seconds)


def spawn_scheduled_fire_if_needed():
    if SWARM_SCENARIO_NAME != SWARM_SCENARIO_CASE2:
        return
    if incident_state.ignited:
        return
    if motion_state.sim_time_seconds < SWARM_CASE2_IGNITION_DELAY_SECONDS:
        return

    fire_x, fire_y, fire_z = choose_incident_fire_spawn()
    fire_hotspots.append(build_fire_hotspot_at(fire_x, fire_y, fire_z))
    incident_state.ignited = True
    incident_state.ignition_time_seconds = motion_state.sim_time_seconds
    incident_state.current_fire_area_sqm = 4.0
    incident_state.peak_fire_area_sqm = 4.0
    incident_state.total_burned_area_sqm = 4.0
    emit_alert("Ignition Event: fire started in Sector A", duration_seconds=4.5)


def compute_tree_avoidance_velocity(origin_x, origin_y, forward_dx, forward_dy, max_horizontal_speed):
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

    for tree_node in trees:
        tree_x = tree_node.getX()
        tree_y = tree_node.getY()
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


def compute_automation_velocity_for_drone(drone_agent, dt):
    response_focus_hotspot = incident_state.response_hotspot
    if (
        response_focus_hotspot is not None
        and response_focus_hotspot.suppression_state in (FIRE_STATE_OUT, FIRE_STATE_BURNED)
    ):
        response_focus_hotspot = None

    incident_focus = None
    if response_focus_hotspot is not None:
        incident_focus = (
            response_focus_hotspot.root.getX(),
            response_focus_hotspot.root.getY(),
            response_focus_hotspot.ground_z,
        )
    else:
        incident_focus = get_incident_focus()

    role = drone_agent.autonomy_role
    if role == "track" and incident_focus is not None:
        orbit_angle = radians(
            (
                motion_state.sim_time_seconds * SWARM_RESPONSE_TRACK_RATE_DEGREES_PER_SECOND
                + drone_agent.response_angle_offset_degrees
            ) % 360.0
        )
        target_world_x = incident_focus[0] + cos(orbit_angle) * SWARM_RESPONSE_TRACK_RADIUS_METERS
        target_world_y = incident_focus[1] + sin(orbit_angle) * SWARM_RESPONSE_TRACK_RADIUS_METERS
        target_world_z = incident_focus[2] + SWARM_RESPONSE_TRACK_ALTITUDE_BOOST_METERS
        drone_agent.status_note = "TRACK"
    elif role == "overwatch" and incident_focus is not None:
        orbit_angle = radians(
            (
                motion_state.sim_time_seconds * SWARM_RESPONSE_OVERWATCH_RATE_DEGREES_PER_SECOND
                + drone_agent.response_angle_offset_degrees
            ) % 360.0
        )
        target_world_x = incident_focus[0] + cos(orbit_angle) * SWARM_RESPONSE_OVERWATCH_RADIUS_METERS
        target_world_y = incident_focus[1] + sin(orbit_angle) * SWARM_RESPONSE_OVERWATCH_RADIUS_METERS
        target_world_z = incident_focus[2] + SWARM_RESPONSE_OVERWATCH_ALTITUDE_BOOST_METERS
        drone_agent.status_note = "OVERWATCH"
    else:
        target_world_x, target_world_y, target_world_z = get_patrol_target(drone_agent)
        if role == "backfill":
            primary_label = (
                drone_agent.patrol_sector.label
                if drone_agent.patrol_sector is not None
                else drone_agent.sector.label
            )
            secondary_label = (
                drone_agent.backfill_secondary_sector.label
                if drone_agent.backfill_secondary_sector is not None
                else None
            )
            if secondary_label is not None and secondary_label != primary_label:
                drone_agent.status_note = f"BACKFILL {primary_label}+{secondary_label}"
            else:
                drone_agent.status_note = f"BACKFILL {primary_label}"
        else:
            drone_agent.status_note = "PATROL"

    drone_x = drone_agent.root.getX()
    drone_y = drone_agent.root.getY()
    dx = target_world_x - drone_x
    dy = target_world_y - drone_y
    horizontal_distance = (dx * dx + dy * dy) ** 0.5
    max_horizontal_speed = get_current_drone_speed_units_per_second() * AUTOMATION_SPEED_FACTOR

    if horizontal_distance > AUTOMATION_TARGET_REACH_RADIUS_METERS and horizontal_distance > 0.001:
        approach_speed_scale = clamp(
            horizontal_distance / FIRE_HOTSPOT_DETECTION_RADIUS_METERS,
            0.55,
            1.0,
        )
        desired_vx = dx / horizontal_distance * max_horizontal_speed * approach_speed_scale
        desired_vy = dy / horizontal_distance * max_horizontal_speed * approach_speed_scale
    else:
        desired_vx = 0.0
        desired_vy = 0.0

    base_desired_vx = desired_vx
    base_desired_vy = desired_vy

    local_canopy_max_z = None
    canopy_probe_points = [(drone_x, drone_y)]
    lookahead_x = drone_x + desired_vx * AUTOMATION_CANOPY_LOOKAHEAD_SECONDS
    lookahead_y = drone_y + desired_vy * AUTOMATION_CANOPY_LOOKAHEAD_SECONDS
    canopy_probe_points.append((lookahead_x, lookahead_y))
    for probe_x, probe_y in canopy_probe_points:
        canopy_height = compute_local_canopy_ceiling(
            probe_x,
            probe_y,
            AUTOMATION_CANOPY_QUERY_RADIUS_METERS,
        )
        if canopy_height is None:
            continue
        if local_canopy_max_z is None or canopy_height > local_canopy_max_z:
            local_canopy_max_z = canopy_height

    should_apply_tree_avoidance = True
    if local_canopy_max_z is not None:
        canopy_safe_altitude = local_canopy_max_z + SWARM_CANOPY_SAFE_MARGIN_METERS
        if drone_agent.root.getZ() >= canopy_safe_altitude - 0.5:
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

    raw_desired_altitude = max(
        AUTOMATION_CRUISE_ALTITUDE_METERS,
        target_world_z + AUTOMATION_SCAN_ALTITUDE_METERS,
    )
    drone_ground = sample_ground(drone_x, drone_y)
    if drone_ground is not None:
        raw_desired_altitude = max(
            raw_desired_altitude,
            drone_ground[0].z + AUTOMATION_MIN_ALTITUDE_ABOVE_GROUND_METERS,
        )
    if local_canopy_max_z is not None:
        raw_desired_altitude = max(
            raw_desired_altitude,
            local_canopy_max_z + SWARM_CANOPY_SAFE_MARGIN_METERS,
        )

    if drone_agent.target_altitude_meters is None:
        drone_agent.target_altitude_meters = raw_desired_altitude
    else:
        drone_agent.target_altitude_meters = low_pass_value(
            drone_agent.target_altitude_meters,
            raw_desired_altitude,
            dt,
            SWARM_ALTITUDE_SMOOTHING_TIME_SECONDS,
        )

    desired_altitude = drone_agent.target_altitude_meters
    if local_canopy_max_z is not None:
        desired_altitude = max(
            desired_altitude,
            local_canopy_max_z + SWARM_CANOPY_SAFE_MARGIN_METERS,
        )

    altitude_error = desired_altitude - drone_agent.root.getZ()
    max_vertical_speed = (
        get_current_drone_speed_units_per_second() * AUTOMATION_VERTICAL_SPEED_FACTOR
    )
    desired_vz = clamp(
        altitude_error * AUTOMATION_ALTITUDE_GAIN,
        -max_vertical_speed,
        max_vertical_speed,
    )

    if abs(base_desired_vx) < 0.05 and abs(base_desired_vy) < 0.05:
        desired_vx = 0.0
        desired_vy = 0.0

    return desired_vx, desired_vy, desired_vz


def apply_drone_motion(drone_agent, desired_world_vx, desired_world_vy, desired_world_vz, dt):
    drone_agent.motion_state.sim_time_seconds += dt
    current_speed_units_per_second = get_current_drone_speed_units_per_second()
    horizontal_speed_limit = current_speed_units_per_second
    vertical_speed_limit = current_speed_units_per_second * AUTOMATION_VERTICAL_SPEED_FACTOR
    selected_drone = get_selected_drone()
    is_selected_manual = (
        drone_agent is selected_drone and drone_agent.control_mode == CONTROL_MODE_MANUAL
    )

    control_lag_seconds = CONTROL_INPUT_LAG_SECONDS if is_selected_manual else 0.0
    drone_agent.motion_state.input_command_buffer.append(
        (
            drone_agent.motion_state.sim_time_seconds + control_lag_seconds,
            desired_world_vx,
            desired_world_vy,
            desired_world_vz,
        )
    )

    while (
        drone_agent.motion_state.input_command_buffer
        and drone_agent.motion_state.input_command_buffer[0][0]
        <= drone_agent.motion_state.sim_time_seconds
    ):
        _, (
            drone_agent.motion_state.applied_cmd.x,
            drone_agent.motion_state.applied_cmd.y,
            drone_agent.motion_state.applied_cmd.z,
        ) = (
            drone_agent.motion_state.input_command_buffer[0][0],
            (
                drone_agent.motion_state.input_command_buffer[0][1],
                drone_agent.motion_state.input_command_buffer[0][2],
                drone_agent.motion_state.input_command_buffer[0][3],
            ),
        )
        drone_agent.motion_state.input_command_buffer.popleft()

    wind_vx, wind_vy, _, _ = get_wind_velocity()
    commanded_vx = drone_agent.motion_state.applied_cmd.x + wind_vx * WIND_DRONE_INFLUENCE_FACTOR
    commanded_vy = drone_agent.motion_state.applied_cmd.y + wind_vy * WIND_DRONE_INFLUENCE_FACTOR

    drone_agent.motion_state.velocity.x = update_axis_velocity(
        drone_agent.motion_state.velocity.x,
        commanded_vx,
        dt,
        DRONE_ACCEL_UNITS_PER_SECOND_SQ,
        DRONE_DECEL_UNITS_PER_SECOND_SQ,
    )
    drone_agent.motion_state.velocity.y = update_axis_velocity(
        drone_agent.motion_state.velocity.y,
        commanded_vy,
        dt,
        DRONE_ACCEL_UNITS_PER_SECOND_SQ,
        DRONE_DECEL_UNITS_PER_SECOND_SQ,
    )
    drone_agent.motion_state.velocity.z = update_axis_velocity(
        drone_agent.motion_state.velocity.z,
        drone_agent.motion_state.applied_cmd.z,
        dt,
        DRONE_ACCEL_UNITS_PER_SECOND_SQ,
        DRONE_DECEL_UNITS_PER_SECOND_SQ,
    )

    drone_agent.motion_state.velocity.x = apply_axis_drag(
        drone_agent.motion_state.velocity.x,
        commanded_vx,
        dt,
        horizontal_speed_limit,
        DRONE_HORIZONTAL_ACTIVE_DRAG_PER_SECOND,
        DRONE_HORIZONTAL_PASSIVE_DRAG_PER_SECOND,
    )
    drone_agent.motion_state.velocity.y = apply_axis_drag(
        drone_agent.motion_state.velocity.y,
        commanded_vy,
        dt,
        horizontal_speed_limit,
        DRONE_HORIZONTAL_ACTIVE_DRAG_PER_SECOND,
        DRONE_HORIZONTAL_PASSIVE_DRAG_PER_SECOND,
    )
    drone_agent.motion_state.velocity.z = apply_axis_drag(
        drone_agent.motion_state.velocity.z,
        drone_agent.motion_state.applied_cmd.z,
        dt,
        vertical_speed_limit,
        DRONE_VERTICAL_ACTIVE_DRAG_PER_SECOND,
        DRONE_VERTICAL_PASSIVE_DRAG_PER_SECOND,
    )

    drone_agent.root.setX(drone_agent.root.getX() + drone_agent.motion_state.velocity.x * dt)
    drone_agent.root.setY(drone_agent.root.getY() + drone_agent.motion_state.velocity.y * dt)
    drone_agent.root.setZ(drone_agent.root.getZ() + drone_agent.motion_state.velocity.z * dt)

    previous_heading_degrees = drone_agent.root.getH()
    new_heading_degrees = previous_heading_degrees
    if is_selected_manual:
        target_heading_degrees = camera_angle
        new_heading_degrees = approach_angle_degrees(
            previous_heading_degrees,
            target_heading_degrees,
            DRONE_HEADING_RESPONSE_DEGREES_PER_SECOND,
            dt,
        )
        drone_agent.root.setH(new_heading_degrees)

    horizontal_speed = (
        drone_agent.motion_state.velocity.x * drone_agent.motion_state.velocity.x
        + drone_agent.motion_state.velocity.y * drone_agent.motion_state.velocity.y
    ) ** 0.5
    shake_speed_range = max(0.001, horizontal_speed_limit - DRONE_ROLL_SHAKE_MIN_SPEED)
    shake_strength = clamp(
        (horizontal_speed - DRONE_ROLL_SHAKE_MIN_SPEED) / shake_speed_range,
        0.0,
        1.0,
    )
    roll_shake = (
        DRONE_ROLL_SHAKE_AMPLITUDE_DEGREES
        * shake_strength
        * sin(tau * DRONE_ROLL_SHAKE_FREQUENCY_HZ * drone_agent.motion_state.sim_time_seconds)
    )
    turn_rate_degrees_per_second = _shortest_angle_delta_degrees(
        previous_heading_degrees,
        new_heading_degrees,
    ) / max(dt, 0.0001)
    yaw_bank = clamp(
        -turn_rate_degrees_per_second * DRONE_YAW_BANK_FROM_TURN_RATE_GAIN,
        -DRONE_YAW_BANK_MAX_DEGREES,
        DRONE_YAW_BANK_MAX_DEGREES,
    )
    body_heading_radians = radians(new_heading_degrees)
    body_right_speed = (
        drone_agent.motion_state.velocity.x * cos(body_heading_radians)
        + drone_agent.motion_state.velocity.y * sin(body_heading_radians)
    )
    body_right_command_speed = (
        drone_agent.motion_state.applied_cmd.x * cos(body_heading_radians)
        + drone_agent.motion_state.applied_cmd.y * sin(body_heading_radians)
    )
    body_forward_speed = (
        -drone_agent.motion_state.velocity.x * sin(body_heading_radians)
        + drone_agent.motion_state.velocity.y * cos(body_heading_radians)
    )
    body_right_accel = (
        body_right_speed - drone_agent.motion_state.last_body_right_speed
    ) / max(dt, 0.0001)
    drone_agent.motion_state.last_body_right_speed = body_right_speed
    drone_agent.motion_state.filtered_body_right_accel = low_pass_value(
        drone_agent.motion_state.filtered_body_right_accel,
        body_right_accel,
        dt,
        DRONE_ROLL_ACCEL_FILTER_TIME_SECONDS,
    )
    target_roll = clamp(
        (
            -drone_agent.motion_state.filtered_body_right_accel * DRONE_ROLL_FROM_ACCEL_GAIN
            + body_right_speed * DRONE_ROLL_REBOUND_FROM_SPEED_GAIN
            + yaw_bank
            + roll_shake
        ),
        -DRONE_MAX_ROLL_DEGREES,
        DRONE_MAX_ROLL_DEGREES,
    )
    target_pitch = clamp(
        body_forward_speed * DRONE_PITCH_FROM_SPEED_GAIN,
        -DRONE_MAX_PITCH_DEGREES,
        DRONE_MAX_PITCH_DEGREES,
    )
    drone_agent.motion_state.visual_roll = approach(
        drone_agent.motion_state.visual_roll,
        target_roll,
        DRONE_ROLL_RESPONSE_DEGREES_PER_SECOND,
        dt,
    )
    drone_agent.motion_state.visual_pitch = approach(
        drone_agent.motion_state.visual_pitch,
        target_pitch,
        DRONE_PITCH_RESPONSE_DEGREES_PER_SECOND,
        dt,
    )

    body_right_command_delta = (
        body_right_command_speed - drone_agent.motion_state.last_body_right_command_speed
    )
    drone_agent.motion_state.last_body_right_command_speed = body_right_command_speed
    if is_selected_manual:
        drone_agent.motion_state.roll_anticipation_body_speed += clamp(
            -body_right_command_delta * DRONE_ROLL_ANTICIPATION_DELTA_GAIN,
            -DRONE_ROLL_ANTICIPATION_MAX_SPEED,
            DRONE_ROLL_ANTICIPATION_MAX_SPEED,
        )
    drone_agent.motion_state.roll_anticipation_body_speed = low_pass_value(
        drone_agent.motion_state.roll_anticipation_body_speed,
        0.0,
        dt,
        DRONE_ROLL_ANTICIPATION_DECAY_TIME_SECONDS,
    )

    target_roll_sway_body_speed = 0.0
    if is_selected_manual:
        target_roll_sway_body_speed = (
            drone_agent.motion_state.visual_roll / max(0.001, DRONE_MAX_ROLL_DEGREES)
        ) * DRONE_ROLL_SWAY_MAX_SPEED
    drone_agent.motion_state.roll_sway_body_speed = low_pass_value(
        drone_agent.motion_state.roll_sway_body_speed,
        target_roll_sway_body_speed,
        dt,
        DRONE_ROLL_SWAY_FILTER_TIME_SECONDS,
    )
    total_roll_sway_body_speed = (
        drone_agent.motion_state.roll_sway_body_speed
        + drone_agent.motion_state.roll_anticipation_body_speed
    )
    sway_world_vx = total_roll_sway_body_speed * cos(body_heading_radians)
    sway_world_vy = total_roll_sway_body_speed * sin(body_heading_radians)
    drone_agent.root.setX(drone_agent.root.getX() + sway_world_vx * dt)
    drone_agent.root.setY(drone_agent.root.getY() + sway_world_vy * dt)

    drone_agent.visual.setP(drone_agent.motion_state.visual_pitch)
    drone_agent.visual.setR(drone_agent.motion_state.visual_roll)
    propeller_spin_speed = (
        DRONE_PROPELLER_SPIN_BASE_DEGREES_PER_SECOND
        + horizontal_speed * DRONE_PROPELLER_SPIN_GAIN_DEGREES_PER_SECOND
        + abs(drone_agent.motion_state.velocity.z) * (DRONE_PROPELLER_SPIN_GAIN_DEGREES_PER_SECOND * 1.5)
    )
    drone_agent.motion_state.propeller_spin_degrees = (
        drone_agent.motion_state.propeller_spin_degrees + propeller_spin_speed * dt
    ) % 360.0
    for rotor_node, spin_direction in drone_agent.propellers:
        rotor_node.setR(drone_agent.motion_state.propeller_spin_degrees * spin_direction)

    edge_margin = 4.0
    x_before_clamp = drone_agent.root.getX()
    y_before_clamp = drone_agent.root.getY()
    clamped_x = clamp(x_before_clamp, SPAWN_X_MIN + edge_margin, SPAWN_X_MAX - edge_margin)
    clamped_y = clamp(y_before_clamp, SPAWN_Y_MIN + edge_margin, SPAWN_Y_MAX - edge_margin)
    if clamped_x != x_before_clamp:
        drone_agent.motion_state.velocity.x = 0.0
    if clamped_y != y_before_clamp:
        drone_agent.motion_state.velocity.y = 0.0
    drone_agent.root.setX(clamped_x)
    drone_agent.root.setY(clamped_y)

    if ENFORCE_MIN_FLYING_ALTITUDE:
        drone_ground = sample_ground(drone_agent.root.getX(), drone_agent.root.getY())
        if drone_ground is not None:
            min_altitude = drone_ground[0].z + 1.6
            if drone_agent.root.getZ() < min_altitude:
                drone_agent.root.setZ(min_altitude)
                drone_agent.motion_state.velocity.z = max(0.0, drone_agent.motion_state.velocity.z)

    drone_agent.trail_timer_seconds += dt
    if drone_agent.trail_timer_seconds >= SWARM_MAP_TRAIL_POINT_INTERVAL_SECONDS:
        drone_agent.trail_timer_seconds = 0.0
        drone_agent.trail_world_points.append(
            (drone_agent.root.getX(), drone_agent.root.getY())
        )
        drone_agent.map_trail_dirty = True


def mark_drone_scan_coverage(drone_agent):
    if drone_agent.root.getZ() < FIRE_HOTSPOT_MIN_SCAN_ALTITUDE_METERS:
        return
    radius_sq = drone_agent.detection_radius_meters * drone_agent.detection_radius_meters
    center_cell_x, center_cell_y = world_to_scan_cell(
        drone_agent.root.getX(),
        drone_agent.root.getY(),
    )
    scan_cell_radius = max(1, int(drone_agent.detection_radius_meters / SWARM_SCAN_CELL_SIZE_METERS) + 1)
    for cell_x in range(center_cell_x - scan_cell_radius, center_cell_x + scan_cell_radius + 1):
        for cell_y in range(center_cell_y - scan_cell_radius, center_cell_y + scan_cell_radius + 1):
            cell = (cell_x, cell_y)
            if cell not in coverage_total_cells:
                continue
            sample_x, sample_y = scan_cell_center(cell_x, cell_y)
            dx = sample_x - drone_agent.root.getX()
            dy = sample_y - drone_agent.root.getY()
            if (dx * dx + dy * dy) > radius_sq:
                continue
            coverage_scanned_cells.add(cell)
            coverage_cells_by_drone[drone_agent.drone_id].add(cell)


def coverage_ratio():
    if not coverage_total_cells:
        return 0.0
    return len(coverage_scanned_cells) / len(coverage_total_cells)


def burn_ratio():
    return clamp(
        incident_state.total_burned_area_sqm / mission_total_area_sqm,
        0.0,
        1.0,
    )


def compute_performance_controls(current_time_seconds):
    coverage_score = coverage_ratio()
    burn_control_score = 1.0 - burn_ratio()
    suppression_score = incident_state.suppression_progress

    detection_score = 0.5
    if incident_state.ignition_time_seconds is not None:
        if incident_state.first_detection_time_seconds is not None:
            detection_delay_seconds = max(
                0.0,
                incident_state.first_detection_time_seconds - incident_state.ignition_time_seconds,
            )
        else:
            detection_delay_seconds = max(
                0.0,
                current_time_seconds - incident_state.ignition_time_seconds,
            )
        detection_score = 1.0 - clamp(
            detection_delay_seconds / max(0.1, SWARM_ADAPTIVE_DETECTION_TARGET_SECONDS),
            0.0,
            1.0,
        )

    performance_score = clamp(
        0.35 * coverage_score
        + 0.25 * detection_score
        + 0.25 * suppression_score
        + 0.15 * burn_control_score,
        0.0,
        1.0,
    )

    required_suppression_threshold = SWARM_BASE_SUPPRESSION_TARGET
    effective_failure_window_seconds = SWARM_INCIDENT_FAILURE_WINDOW_SECONDS
    suppression_error_probability = SWARM_SUPPRESSION_ERROR_PROBABILITY
    detection_probability_scale = 1.0
    growth_rate_scale = 1.0

    if SWARM_ADAPTIVE_PERFORMANCE_ENABLED and SWARM_SCENARIO_NAME == SWARM_SCENARIO_CASE2:
        # Higher measured performance raises difficulty (stricter target/shorter window).
        difficulty_anchor = performance_score
        if SWARM_ADAPTIVE_PROFILE == "strict":
            difficulty_anchor = max(difficulty_anchor, 0.55)
        elif SWARM_ADAPTIVE_PROFILE == "lenient":
            difficulty_anchor = min(difficulty_anchor, 0.45)

        required_suppression_threshold = (
            SWARM_ADAPTIVE_SUPPRESSION_TARGET_MIN
            + (
                SWARM_ADAPTIVE_SUPPRESSION_TARGET_MAX
                - SWARM_ADAPTIVE_SUPPRESSION_TARGET_MIN
            )
            * difficulty_anchor
        )
        effective_failure_window_seconds = (
            SWARM_ADAPTIVE_FAILURE_WINDOW_MAX_SECONDS
            - (
                SWARM_ADAPTIVE_FAILURE_WINDOW_MAX_SECONDS
                - SWARM_ADAPTIVE_FAILURE_WINDOW_MIN_SECONDS
            )
            * difficulty_anchor
        )
        suppression_error_probability = (
            SWARM_ADAPTIVE_SUPPRESSION_ERROR_PROB_MIN
            + (
                SWARM_ADAPTIVE_SUPPRESSION_ERROR_PROB_MAX
                - SWARM_ADAPTIVE_SUPPRESSION_ERROR_PROB_MIN
            )
            * difficulty_anchor
        )
        detection_probability_scale = (
            SWARM_ADAPTIVE_DETECTION_PROB_SCALE_MAX
            - (
                SWARM_ADAPTIVE_DETECTION_PROB_SCALE_MAX
                - SWARM_ADAPTIVE_DETECTION_PROB_SCALE_MIN
            )
            * difficulty_anchor
        )
        growth_rate_scale = (
            SWARM_ADAPTIVE_GROWTH_RATE_SCALE_MIN
            + (
                SWARM_ADAPTIVE_GROWTH_RATE_SCALE_MAX
                - SWARM_ADAPTIVE_GROWTH_RATE_SCALE_MIN
            )
            * difficulty_anchor
        )

    return {
        "performance_score": performance_score,
        "required_suppression_threshold": clamp(required_suppression_threshold, 0.5, 0.999),
        "effective_failure_window_seconds": max(1.0, effective_failure_window_seconds),
        "suppression_error_probability": clamp(suppression_error_probability, 0.0, 0.95),
        "detection_probability_scale": max(0.1, detection_probability_scale),
        "growth_rate_scale": max(0.1, growth_rate_scale),
    }


def update_hotspot_detection(dt):
    for hotspot in fire_hotspots:
        if hotspot.suppression_state != FIRE_STATE_ACTIVE or hotspot.detected:
            continue
        for drone_agent in drones:
            dx = hotspot.root.getX() - drone_agent.root.getX()
            dy = hotspot.root.getY() - drone_agent.root.getY()
            horizontal_distance = (dx * dx + dy * dy) ** 0.5
            if horizontal_distance > drone_agent.detection_radius_meters:
                continue
            altitude_above_hotspot = drone_agent.root.getZ() - hotspot.ground_z
            if altitude_above_hotspot < FIRE_HOTSPOT_MIN_SCAN_ALTITUDE_METERS:
                continue
            range_factor = max(
                0.0,
                1.0 - (horizontal_distance / drone_agent.detection_radius_meters),
            )
            altitude_factor = clamp(
                altitude_above_hotspot / (FIRE_HOTSPOT_MIN_SCAN_ALTITUDE_METERS * 2.2),
                0.25,
                1.0,
            )
            detection_probability = clamp(
                FIRE_HOTSPOT_DETECTION_PROBABILITY_PER_SECOND
                * incident_state.effective_detection_probability_scale
                * range_factor
                * altitude_factor
                * dt,
                0.0,
                0.98,
            )
            if (
                drone_agent.drone_id in incident_state.responder_ids
                and horizontal_distance <= max(2.0, AUTOMATION_FORCED_DETECTION_RADIUS_METERS)
            ):
                detection_probability = 1.0
            if random.random() < detection_probability:
                register_hotspot_detection(hotspot, drone_agent)
                break


def maybe_trigger_reignition():
    if incident_state.suppression_error_mode != "reignite":
        return
    if incident_state.reignition_triggered:
        return
    if incident_state.suppression_progress < 0.85:
        return
    candidate_hotspots = [
        hotspot
        for hotspot in fire_hotspots
        if hotspot.suppression_state in (FIRE_STATE_CONTAINED, FIRE_STATE_OUT)
    ]
    if not candidate_hotspots:
        return
    chosen_hotspot = random.choice(candidate_hotspots)
    chosen_hotspot.suppression_state = FIRE_STATE_ACTIVE
    chosen_hotspot.detected = True
    chosen_hotspot.detection_age_seconds = max(
        0.0,
        FIRE_SUPPRESSION_CONTAIN_DELAY_SECONDS * 0.65,
    )
    incident_state.current_fire_area_sqm = max(
        incident_state.current_fire_area_sqm,
        incident_state.peak_fire_area_sqm * 0.18,
    )
    incident_state.reignition_extension_seconds = SWARM_REIGNITION_RECOVERY_SECONDS
    incident_state.reignition_triggered = True
    assign_response_drones(get_selected_drone() or drones[0], chosen_hotspot)
    set_hotspot_visual(chosen_hotspot)
    emit_alert("Suppression delay: hotspot reignition reported", duration_seconds=4.5)
    play_detection_alarm(motion_state.sim_time_seconds)


def update_hotspot_lifecycle(dt):
    contain_delay_seconds = FIRE_SUPPRESSION_CONTAIN_DELAY_SECONDS
    if incident_state.suppression_error_mode == "delay":
        contain_delay_seconds += SWARM_SUPPRESSION_DELAY_SECONDS
    total_time_to_out = (
        contain_delay_seconds
        + FIRE_SUPPRESSION_EXTINGUISH_DELAY_SECONDS
        + incident_state.reignition_extension_seconds
    )

    for hotspot in fire_hotspots:
        previous_state = hotspot.suppression_state
        if hotspot.suppression_state == FIRE_STATE_ACTIVE and not hotspot.detected:
            hotspot.burn_age_seconds += dt
            if hotspot.burn_age_seconds >= FIRE_UNDETECTED_BURNOUT_DELAY_SECONDS:
                hotspot.suppression_state = FIRE_STATE_BURNED
                hotspot.detected = True
                hotspot.detection_age_seconds = 0.0

        if hotspot.detected and hotspot.suppression_state in (
            FIRE_STATE_ACTIVE,
            FIRE_STATE_CONTAINED,
            FIRE_STATE_OUT,
        ):
            hotspot.detection_age_seconds += dt
            if hotspot.suppression_state == FIRE_STATE_ACTIVE:
                if hotspot.detection_age_seconds >= contain_delay_seconds:
                    hotspot.suppression_state = FIRE_STATE_CONTAINED
            elif hotspot.suppression_state == FIRE_STATE_CONTAINED:
                if hotspot.detection_age_seconds >= total_time_to_out:
                    hotspot.suppression_state = FIRE_STATE_OUT

        if hotspot.suppression_state != previous_state or hotspot.suppression_state == FIRE_STATE_ACTIVE:
            set_hotspot_visual(hotspot)

    maybe_trigger_reignition()

    if not any(
        hotspot.suppression_state in (FIRE_STATE_ACTIVE, FIRE_STATE_CONTAINED)
        for hotspot in fire_hotspots
    ):
        clear_response_assignment()


def update_hotspot_spread(dt):
    global fire_spread_area_budget_sqm

    if SWARM_SCENARIO_NAME != SWARM_SCENARIO_CASE2:
        return
    if not incident_state.ignited:
        return
    active_sources = [
        hotspot
        for hotspot in fire_hotspots
        if hotspot.suppression_state == FIRE_STATE_ACTIVE
    ]
    if not active_sources:
        return

    fire_spread_area_budget_sqm += SWARM_CASE2_GROWTH_RATE_SQM_PER_SECOND * dt
    if fire_spread_area_budget_sqm < SWARM_CASE2_EXTRA_HOTSPOT_AREA_TRIGGER_SQM:
        return

    _, _, wind_direction_degrees, wind_speed_mps = get_wind_velocity()
    wind_heading_radians = radians(wind_direction_degrees)
    wind_direction_bias = min(
        0.9,
        wind_speed_mps * WIND_FIRE_DIRECTIONAL_BIAS_PER_METER_PER_SECOND,
    )
    wind_bias_half_cone = radians(WIND_FIRE_BIAS_CONE_DEGREES)

    def try_spawn(candidate_x, candidate_y):
        if (
            candidate_x < SPAWN_X_MIN
            or candidate_x > SPAWN_X_MAX
            or candidate_y < SPAWN_Y_MIN
            or candidate_y > SPAWN_Y_MAX
        ):
            return False
        if not _is_position_clear_of_trees(
            candidate_x,
            candidate_y,
            FIRE_SPREAD_TREE_CLEARANCE_METERS,
        ):
            return False
        if not _is_position_clear_of_hotspots(
            candidate_x,
            candidate_y,
            FIRE_SPREAD_MIN_HOTSPOT_SEPARATION_METERS,
        ):
            return False
        ground_sample = sample_ground(candidate_x, candidate_y)
        if ground_sample is None:
            return False
        fire_hotspots.append(
            build_fire_hotspot_at(candidate_x, candidate_y, ground_sample[0].z)
        )
        trim_active_fire_hotspots()
        return True

    spawned = False
    for _ in range(FIRE_SPREAD_SPAWN_ATTEMPTS):
        source_hotspot = random.choice(active_sources)
        source_x = source_hotspot.root.getX()
        source_y = source_hotspot.root.getY()
        if wind_speed_mps > 0.05 and random.random() < wind_direction_bias:
            spread_angle = wind_heading_radians + random.uniform(
                -wind_bias_half_cone,
                wind_bias_half_cone,
            )
        else:
            spread_angle = random.uniform(0.0, tau)
        spread_distance = random.uniform(
            FIRE_SPREAD_MIN_DISTANCE_METERS * 0.7,
            FIRE_SPREAD_MAX_DISTANCE_METERS * 1.1,
        )
        candidate_x = source_x + cos(spread_angle) * spread_distance
        candidate_y = source_y + sin(spread_angle) * spread_distance
        if try_spawn(candidate_x, candidate_y):
            spawned = True
            break

    if spawned:
        fire_spread_area_budget_sqm = max(
            0.0,
            fire_spread_area_budget_sqm - SWARM_CASE2_EXTRA_HOTSPOT_AREA_TRIGGER_SQM,
        )


def update_incident_metrics(dt):
    if not incident_state.ignited:
        return

    current_time_seconds = motion_state.sim_time_seconds
    controls = compute_performance_controls(current_time_seconds)
    incident_state.performance_score = controls["performance_score"]
    incident_state.required_suppression_threshold = controls["required_suppression_threshold"]
    incident_state.effective_failure_window_seconds = controls["effective_failure_window_seconds"]
    incident_state.effective_suppression_error_probability = controls["suppression_error_probability"]
    incident_state.effective_detection_probability_scale = controls["detection_probability_scale"]
    incident_state.effective_growth_rate_scale = controls["growth_rate_scale"]
    base_growth_rate_sqm_per_second = (
        SWARM_CASE2_GROWTH_RATE_SQM_PER_SECOND * incident_state.effective_growth_rate_scale
    )
    growth_rate_sqm_per_second = 0.0
    suppression_rate_sqm_per_second = 0.0
    incident_state.full_suppression_eta_seconds = None

    if incident_state.first_detection_time_seconds is None:
        growth_rate_sqm_per_second = base_growth_rate_sqm_per_second
        incident_state.suppression_status = "UNDETECTED"
    else:
        contain_delay_seconds = FIRE_SUPPRESSION_CONTAIN_DELAY_SECONDS
        if incident_state.suppression_error_mode == "delay":
            contain_delay_seconds += SWARM_SUPPRESSION_DELAY_SECONDS
        total_time_to_out = (
            contain_delay_seconds
            + FIRE_SUPPRESSION_EXTINGUISH_DELAY_SECONDS
            + incident_state.reignition_extension_seconds
        )
        contain_phase_time = (
            incident_state.first_detection_time_seconds + contain_delay_seconds
        )
        out_phase_time = incident_state.first_detection_time_seconds + total_time_to_out

        if current_time_seconds < incident_state.truck_arrival_time_seconds:
            growth_rate_sqm_per_second = base_growth_rate_sqm_per_second * 0.8
            incident_state.suppression_status = "TRUCK EN ROUTE"
        elif current_time_seconds < contain_phase_time:
            growth_rate_sqm_per_second = base_growth_rate_sqm_per_second * 0.35
            incident_state.suppression_status = "INITIAL ATTACK"
        elif current_time_seconds < out_phase_time:
            incident_state.suppression_status = (
                "MOP-UP"
                if incident_state.current_fire_area_sqm <= incident_state.peak_fire_area_sqm * 0.45
                else "CONTAINED"
            )
            remaining_seconds = max(0.1, out_phase_time - current_time_seconds)
            suppression_rate_sqm_per_second = incident_state.current_fire_area_sqm / remaining_seconds
            incident_state.full_suppression_eta_seconds = remaining_seconds
        else:
            incident_state.suppression_status = "SUPPRESSED"

    previous_fire_area = incident_state.current_fire_area_sqm
    if growth_rate_sqm_per_second > 0.0:
        incident_state.current_fire_area_sqm += growth_rate_sqm_per_second * dt
        incident_state.total_burned_area_sqm += growth_rate_sqm_per_second * dt
    elif suppression_rate_sqm_per_second > 0.0:
        incident_state.current_fire_area_sqm = max(
            0.0,
            incident_state.current_fire_area_sqm - suppression_rate_sqm_per_second * dt,
        )
        incident_state.total_burned_area_sqm += 0.12 * dt
    incident_state.peak_fire_area_sqm = max(
        incident_state.peak_fire_area_sqm,
        incident_state.current_fire_area_sqm,
    )

    if incident_state.peak_fire_area_sqm > 0.0:
        incident_state.suppression_progress = clamp(
            1.0 - (incident_state.current_fire_area_sqm / incident_state.peak_fire_area_sqm),
            0.0,
            1.0,
        )
    else:
        incident_state.suppression_progress = 0.0

    if previous_fire_area > incident_state.current_fire_area_sqm:
        incident_state.suppression_rate_sqm_per_second = (
            previous_fire_area - incident_state.current_fire_area_sqm
        ) / max(dt, 0.0001)
    else:
        incident_state.suppression_rate_sqm_per_second = 0.0

    if (
        incident_state.ignition_time_seconds is not None
        and (current_time_seconds - incident_state.ignition_time_seconds)
        >= incident_state.effective_failure_window_seconds
        and incident_state.suppression_progress < incident_state.required_suppression_threshold
        and not incident_state.failure_triggered
    ):
        incident_state.failure_triggered = True
        required_percent = incident_state.required_suppression_threshold * 100.0
        emit_alert(
            f"FAILURE: fire was below {required_percent:.0f}% suppression before window close",
            duration_seconds=6.0,
        )

    if (
        incident_state.suppression_progress >= incident_state.required_suppression_threshold
        and not incident_state.success_announced
    ):
        incident_state.success_announced = True
        required_percent = incident_state.required_suppression_threshold * 100.0
        emit_alert(
            f"SUCCESS: incident reached {required_percent:.0f}% suppression",
            duration_seconds=5.0,
        )

    if incident_state.current_fire_area_sqm <= 0.01 and all(
        hotspot.suppression_state in (FIRE_STATE_OUT, FIRE_STATE_BURNED)
        for hotspot in fire_hotspots
    ):
        clear_response_assignment()


def update_status_overlay():
    selected_drone = get_selected_drone()
    detected_count = count_detected_hotspots(fire_hotspots)
    active_count = sum(
        1 for hotspot in fire_hotspots
        if hotspot.suppression_state == FIRE_STATE_ACTIVE
    )
    contained_count = sum(
        1 for hotspot in fire_hotspots
        if hotspot.suppression_state == FIRE_STATE_CONTAINED
    )
    suppressed_out_count = sum(
        1 for hotspot in fire_hotspots
        if hotspot.suppression_state == FIRE_STATE_OUT
    )
    burned_out_count = sum(
        1 for hotspot in fire_hotspots
        if hotspot.suppression_state == FIRE_STATE_BURNED
    )
    auto_drone_labels = ", ".join(
        f"D{drone_agent.drone_id}"
        for drone_agent in drones
        if drone_agent.control_mode == CONTROL_MODE_AUTOMATION
    ) or "-"
    manual_drone_labels = ", ".join(
        f"D{drone_agent.drone_id}"
        for drone_agent in drones
        if drone_agent.control_mode == CONTROL_MODE_MANUAL
    ) or "-"
    wind_direction_degrees = get_wind_direction_degrees()
    wind_speed_mps = get_wind_speed_mps()
    wind_label = get_wind_compass_label(wind_direction_degrees)
    mission_time_label = format_mission_duration(motion_state.sim_time_seconds)
    ops_time_label = format_operational_duration_from_demo_seconds(
        motion_state.sim_time_seconds
    )
    selected_label = f"D{selected_drone.drone_id}" if selected_drone is not None else "--"
    thermal_label = "ON" if thermal_view_enabled else "OFF"
    coverage_label = coverage_ratio() * 100.0
    burned_percent = burn_ratio() * 100.0
    role_summary = ", ".join(
        f"D{drone_agent.drone_id}:{drone_agent.status_note}"
        for drone_agent in drones
    )

    line_one = (
        f"{SWARM_SCENARIO_TITLE} | Elapsed: {mission_time_label} demo / {ops_time_label} ops"
        f" | {format_speed_mode_menu()} | Playback: {current_playback_speed_label} | Selected: {selected_label}"
    )
    line_two = (
        f"Drones: {len(drones)} active | Auto: {auto_drone_labels} | Manual: {manual_drone_labels}"
        f" | Roles: {role_summary} | Thermal: {thermal_label}"
    )
    line_three = (
        f"Coverage: {coverage_label:.0f}% | Burned: {burned_percent:.1f}% | Fires Detected: {detected_count}"
        f" | Hotspots A/C/O/B: {active_count}/{contained_count}/{suppressed_out_count}/{burned_out_count}"
        f" | Wind: {wind_speed_mps:.1f}m/s {wind_label}"
    )
    if SWARM_SCENARIO_NAME == SWARM_SCENARIO_CASE1:
        status_suffix = (
            "Coverage complete"
            if coverage_label >= 99.0
            else "Sector patrol underway"
        )
        line_four = (
            f"Mission: 3-minute autonomous no-fire coverage demo | Status: {status_suffix}"
            f" | Controls: 1-{len(drones)} select, WASD/QE manual, M/O mode, Z/X/C drone speed, 7/8/9/0 playback"
        )
    else:
        truck_eta_label = "--"
        if incident_state.first_detection_time_seconds is None:
            truck_eta_label = "Standby"
        elif motion_state.sim_time_seconds < incident_state.truck_arrival_time_seconds:
            truck_eta_label = format_mission_duration(
                incident_state.truck_arrival_time_seconds - motion_state.sim_time_seconds
            )
        else:
            truck_eta_label = "Arrived"

        failure_window_label = "--"
        if incident_state.ignition_time_seconds is not None:
            failure_window_label = format_operational_duration_from_demo_seconds(
                max(
                    0.0,
                    incident_state.ignition_time_seconds
                    + incident_state.effective_failure_window_seconds
                    - motion_state.sim_time_seconds,
                )
            )

        suppression_rate_label = (
            f"{incident_state.suppression_rate_sqm_per_second:.2f} m2/s area reduced"
            if incident_state.suppression_rate_sqm_per_second > 0.0
            else "--"
        )
        suppression_eta_label = (
            (
                f"{format_mission_duration(incident_state.full_suppression_eta_seconds)} demo"
                f" / {format_operational_duration_from_demo_seconds(incident_state.full_suppression_eta_seconds)} ops"
            )
            if incident_state.full_suppression_eta_seconds is not None
            else "--"
        )
        incident_outcome_label = (
            "FAILED"
            if incident_state.failure_triggered
            else ("SUCCESS" if incident_state.success_announced else "IN PROGRESS")
        )
        suppression_goal_label = f"{incident_state.required_suppression_threshold * 100.0:.1f}%"
        performance_score_label = f"{incident_state.performance_score * 100.0:.0f}%"
        suppression_progress_label = f"{incident_state.suppression_progress * 100.0:.0f}%"
        adaptive_label = "ADAPTIVE" if SWARM_ADAPTIVE_PERFORMANCE_ENABLED else "FIXED"
        success_signal_label = (
            "SUCCESS LIKELY"
            if incident_state.suppression_progress >= incident_state.required_suppression_threshold
            else "FAILURE RISK"
        )
        line_four = (
            f"Truck ETA: {truck_eta_label} | Suppression: {incident_state.suppression_status}"
            f" | Supp Rate(now): {suppression_rate_label} | ETA to 100%: {suppression_eta_label}"
            f" | Window Left: {failure_window_label} | Goal: {suppression_goal_label}"
            f" | SUCCESS NOW: {suppression_progress_label} {success_signal_label}"
            f" | Operator Perf: {performance_score_label} ({adaptive_label})"
            f" | Outcome: {incident_outcome_label}"
            f" | Controls: Z/X/C drone speed, 7/8/9/0 playback"
        )

    status_text.setText("\n".join((line_one, line_two, line_three, line_four)))
    if alert_banner_timer_seconds <= 0.0:
        alert_text.setText("")
    else:
        alert_text.setText(alert_banner_text)


def initialize_ui():
    global status_text, alert_text
    status_text = OnscreenText(
        text="",
        pos=(-1.31, 0.91),
        align=TextNode.ALeft,
        scale=0.04,
        fg=(1.0, 1.0, 1.0, 1.0),
        mayChange=True,
    )
    alert_text = OnscreenText(
        text="",
        pos=(0.0, 0.92),
        align=TextNode.ACenter,
        scale=0.06,
        fg=(1.0, 0.25, 0.18, 1.0),
        mayChange=True,
    )


def update_camera():
    global camera_angle, camera_pitch, last_mouse_x, last_mouse_y
    selected_drone = get_selected_drone()
    if selected_drone is None or app.camera is None:
        return

    if dragging_camera and app.mouseWatcherNode.hasMouse():
        current_mouse_x = app.mouseWatcherNode.getMouseX()
        current_mouse_y = app.mouseWatcherNode.getMouseY()
        raw_mouse_delta_x = current_mouse_x - last_mouse_x
        raw_mouse_delta_y = current_mouse_y - last_mouse_y
        mouse_delta_x = clamp(raw_mouse_delta_x, -CAMERA_DRAG_DELTA_CLAMP, CAMERA_DRAG_DELTA_CLAMP)
        mouse_delta_y = clamp(raw_mouse_delta_y, -CAMERA_DRAG_DELTA_CLAMP, CAMERA_DRAG_DELTA_CLAMP)
        camera_angle += mouse_delta_x * CAMERA_DRAG_X_DEGREES_PER_UNIT
        drag_y_gain = (
            FIRST_PERSON_DRAG_Y_DEGREES_PER_UNIT
            if first_person_view
            else THIRD_PERSON_DRAG_Y_DEGREES_PER_UNIT
        )
        camera_pitch = clamp(camera_pitch + mouse_delta_y * drag_y_gain, -85.0, 85.0)
        last_mouse_x = current_mouse_x
        last_mouse_y = current_mouse_y

    if first_person_view:
        eye_x = selected_drone.root.getX()
        eye_y = selected_drone.root.getY()
        eye_z = selected_drone.root.getZ() + first_person_eye_height
        yaw = radians(camera_angle)
        pitch = radians(camera_pitch)
        look_dir = Vec3(
            -sin(yaw) * cos(pitch),
            cos(yaw) * cos(pitch),
            sin(pitch),
        )
        app.camera.setPos(eye_x, eye_y, eye_z)
        app.camera.lookAt(Point3(eye_x, eye_y, eye_z) + look_dir * 30.0)
    else:
        horizontal_distance = camera_distance * cos(radians(camera_pitch))
        camera_x = selected_drone.root.getX() + sin(radians(camera_angle)) * horizontal_distance
        camera_y = selected_drone.root.getY() - cos(radians(camera_angle)) * horizontal_distance
        camera_z = selected_drone.root.getZ() + camera_height + sin(radians(camera_pitch)) * camera_distance
        app.camera.setPos(camera_x, camera_y, camera_z)
        app.camera.lookAt(selected_drone.root)


def update(task):
    global alert_banner_timer_seconds, headless_frame_counter, mission_completed
    global fire_effect_update_accumulator_seconds
    global map_update_accumulator_seconds, status_update_accumulator_seconds
    raw_dt = ClockObject.getGlobalClock().getDt()
    if raw_dt <= 0.0:
        raw_dt = 1.0 / 60.0
    raw_dt = min(raw_dt, 0.05)
    remaining_dt = raw_dt * get_current_playback_multiplier()

    while remaining_dt > 0.0001:
        dt = min(0.05, remaining_dt)
        remaining_dt -= dt

        update_wind(dt)
        motion_state.sim_time_seconds += dt
        spawn_scheduled_fire_if_needed()

        selected_drone = get_selected_drone()
        for drone_agent in drones:
            if drone_agent.control_mode == CONTROL_MODE_AUTOMATION:
                desired_world_vx, desired_world_vy, desired_world_vz = compute_automation_velocity_for_drone(
                    drone_agent,
                    dt,
                )
            elif drone_agent is selected_drone:
                desired_world_vx, desired_world_vy, desired_world_vz = compute_desired_velocity(
                    camera_angle,
                    key_map,
                    get_current_drone_speed_units_per_second(),
                )
                drone_agent.status_note = "MANUAL"
            else:
                desired_world_vx, desired_world_vy, desired_world_vz = (0.0, 0.0, 0.0)
                drone_agent.status_note = "HOLD"
            apply_drone_motion(
                drone_agent,
                desired_world_vx,
                desired_world_vy,
                desired_world_vz,
                dt,
            )
            mark_drone_scan_coverage(drone_agent)

        update_hotspot_spread(dt)
        update_hotspot_detection(dt)
        update_hotspot_lifecycle(dt)
        update_incident_metrics(dt)
        fire_effect_update_accumulator_seconds += dt
        if fire_effect_update_accumulator_seconds >= SWARM_FIRE_EFFECT_UPDATE_INTERVAL_SECONDS:
            update_hotspot_effect_animation(fire_effect_update_accumulator_seconds)
            fire_effect_update_accumulator_seconds = 0.0

        if (
            SWARM_SCENARIO_NAME == SWARM_SCENARIO_CASE1
            and motion_state.sim_time_seconds >= SWARM_CASE1_DURATION_SECONDS
            and not mission_completed
        ):
            mission_completed = True
            emit_alert("Case 1 demo window complete: coverage patrol finished", duration_seconds=5.0)

        alert_banner_timer_seconds = max(0.0, alert_banner_timer_seconds - dt)

    update_camera()
    map_update_accumulator_seconds += raw_dt
    if map_update_accumulator_seconds >= SWARM_MAP_UPDATE_INTERVAL_SECONDS:
        update_map_overlay()
        map_update_accumulator_seconds = 0.0
    status_update_accumulator_seconds += raw_dt
    if status_update_accumulator_seconds >= SWARM_STATUS_UPDATE_INTERVAL_SECONDS:
        update_status_overlay()
        status_update_accumulator_seconds = 0.0

    headless_frame_counter += 1
    if SWARM_HEADLESS_FRAME_LIMIT and headless_frame_counter >= SWARM_HEADLESS_FRAME_LIMIT:
        if os.environ.get("REAL_SIM_HEADLESS", "0") == "1":
            os._exit(0)
        try:
            app.userExit()
        except Exception:
            pass
        return task.done

    return task.cont


initialize_world_state()
build_map_overlay()
initialize_ui()
bind_key_pairs()
for drone_index in range(min(9, len(drones))):
    app.accept(str(drone_index + 1), set_selected_drone, [drone_index])
app.accept("b", toggle_camera_mode)
app.accept("f", set_first_person_mode)
app.accept("g", set_third_person_mode)
app.accept("m", set_selected_drone_manual)
app.accept("o", set_selected_drone_automation)
app.accept("tab", toggle_selected_drone_control_mode)
app.accept("t", toggle_thermal_view)
app.accept("z", set_slow_speed)
app.accept("x", set_normal_speed)
app.accept("c", set_fast_speed)
app.accept("7", set_half_playback_speed)
app.accept("8", set_normal_playback_speed)
app.accept("9", set_double_playback_speed)
app.accept("0", set_quad_playback_speed)
app.accept("mouse1", start_camera_drag)
app.accept("mouse1-up", stop_camera_drag)
apply_camera_mode()
apply_thermal_view()
update_map_overlay()
update_status_overlay()
app.taskMgr.add(update, "update")
app.run()
