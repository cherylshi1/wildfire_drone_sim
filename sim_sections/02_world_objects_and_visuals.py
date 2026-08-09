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
FIRE_TRUCK_TARGET_LENGTH_METERS = 3.4
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

# One mark scheme shared by the world map and the detected fire map
# (Cheryl, July 28). The X is used for ONE thing only: a fire that is out,
# and it is green in both places. Purple marks a fire the trucks are working
# (the fire itself glows purple, never an X). Burned fires keep the dark red
# X so an out fire and a lost fire never look alike.
FIRE_MARK_OUT_COLOR = (0.4, 0.94, 0.52, 0.96)
FIRE_MARK_BURNED_COLOR = (0.58, 0.18, 0.14, 0.94)
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
    # How many of the engaged water drones are being flown by hand right now,
    # and whether the operator was flying the survey drone that found this fire.
    # Both feed the operator credit in the collaboration score (July 29).
    manual_water_drone_engagement_count: int = 0
    manual_water_suppression_work_seconds: float = 0.0
    detected_under_manual_control: bool = False
    ground_firefighter_engaged: bool = False
    current_suppression_rate_per_second: float = 0.0
    # Who actually put this fire out (Cheryl, July 28): the round report needs
    # "N put out by fire trucks, M by drones, K shared 20/80". Both accumulate
    # the suppression work each source contributed; extinguished_by is set the
    # moment the hotspot reaches FIRE_STATE_OUT and reset on reignition.
    ground_suppression_work_seconds: float = 0.0
    water_suppression_work_seconds: float = 0.0
    extinguished_by: str | None = None
    extinguish_ground_share: float = 0.0
    # Which ignition event this hotspot belongs to (spread children inherit
    # it). Used for fire ownership: the survey drone that first detects any
    # hotspot of an event owns the whole fire; other surveys keep sweeping.
    fire_event_id: int = 1
    reignition_count: int = 0


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
    burnout_delay_seconds = current_undetected_burnout_delay_seconds()
    if burnout_delay_seconds <= 0.0:
        return 1.0
    return max(
        0.0,
        min(1.0, hotspot.burn_age_seconds / burnout_delay_seconds),
    )


def hotspot_fire_growth_scale(hotspot):
    growth_reference_seconds = max(
        1.0,
        min(
            current_undetected_burnout_delay_seconds(),
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
    elif hotspot_state == FIRE_STATE_OUT:
        # Consistent marking scheme (July 6): an extinguished fire leaves a
        # GREEN X in the world, matching the green X on the operator map
        # (previously an out fire just vanished, which read as a map error).
        hotspot.scar_core.hide()
        hotspot.scar_cross_a.show()
        hotspot.scar_cross_b.show()
        hotspot.scar_cross_a.setScale(1.15)
        hotspot.scar_cross_b.setScale(1.15)
        hotspot.scar_cross_a.setColor(*FIRE_MARK_OUT_COLOR)
        hotspot.scar_cross_b.setColor(*FIRE_MARK_OUT_COLOR)
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
            hotspot.scar_cross_a.setColor(*FIRE_MARK_BURNED_COLOR)
            hotspot.scar_cross_b.setColor(*FIRE_MARK_BURNED_COLOR)
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
                marker_part.setBin("fixed", THERMAL_HOTSPOT_BIN_ORDER)
                marker_part.setDepthTest(False)
                marker_part.setDepthWrite(False)
        elif hotspot.detected and hotspot_state in (
            FIRE_STATE_ACTIVE,
            FIRE_STATE_CONTAINED,
        ):
            hotspot.glow.show()
            hotspot.beacon_a.show()
            hotspot.beacon_b.show()
            contained_scale = 0.88 if state_is_contained else 1.0
            hotspot.glow.setScale(
                DETECTED_FIRE_OVERLAY_GLOW_SCALE * contained_scale
            )
            hotspot.beacon_a.setScale(
                DETECTED_FIRE_OVERLAY_BEACON_A_SCALE * contained_scale
            )
            hotspot.beacon_b.setScale(
                DETECTED_FIRE_OVERLAY_BEACON_B_SCALE * contained_scale
            )
            if state_is_contained:
                hotspot.glow.setColor(1.0, 0.68, 0.16, 0.82)
                hotspot.beacon_a.setColor(1.0, 0.84, 0.36, 0.92)
                hotspot.beacon_b.setColor(1.0, 0.95, 0.66, 0.96)
            else:
                hotspot.glow.setColor(0.0, 0.9, 1.0, 0.82)
                hotspot.beacon_a.setColor(0.1, 1.0, 1.0, 0.9)
                hotspot.beacon_b.setColor(0.78, 1.0, 1.0, 0.96)
            for marker_part in (hotspot.glow, hotspot.beacon_a, hotspot.beacon_b):
                marker_part.setBin("fixed", THERMAL_HOTSPOT_BIN_ORDER)
                marker_part.setDepthTest(False)
                marker_part.setDepthWrite(False)
        else:
            hotspot.glow.hide()
            hotspot.beacon_a.hide()
            hotspot.beacon_b.hide()
        if state_is_burned:
            hotspot.scar_core.setColor(0.01, 0.01, 0.01, 0.98)
            hotspot.scar_cross_a.setColor(*FIRE_MARK_BURNED_COLOR)
            hotspot.scar_cross_b.setColor(*FIRE_MARK_BURNED_COLOR)

    if firetruck_response_active:
        # A truck-worked fire is marked by the PURPLE hotspot ring, never by an
        # X (Cheryl, July 28): the X now means "fire out" and nothing else, on
        # both the world map and the detected fire map.
        hotspot.scar_core.show()
        hotspot.scar_cross_a.hide()
        hotspot.scar_cross_b.hide()
        hotspot.scar_core.setScale(1.28)
        hotspot.scar_core.setColor(
            FIRE_TRUCK_SUPPRESSING_COLOR[0],
            FIRE_TRUCK_SUPPRESSING_COLOR[1],
            FIRE_TRUCK_SUPPRESSING_COLOR[2],
            0.34,
        )
        hotspot.scar_core.setBin("fixed", THERMAL_HOTSPOT_BIN_ORDER + 1)
        hotspot.scar_core.setDepthTest(False)
        hotspot.scar_core.setDepthWrite(False)


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
                    current_ground_suppression_rate_per_second()
                    + current_water_suppression_rate_per_second()
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


# Per-prototype (scale, z) placement cache for baked fire mesh frames; see
# the PERF note inside build_ground_fire_visual.
_baked_fire_mesh_placement_cache = {}


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
        for prototype_index, mesh_prototype in enumerate(mesh_prototypes):
            mesh_node = mesh_prototype.copyTo(root)
            mesh_node.hide()
            mesh_node.setTransparency(TransparencyAttrib.MAlpha)
            mesh_node.setTwoSided(False)
            mesh_node.setDepthWrite(True)
            mesh_node.setP(90)
            # PERF (July 9, Cheryl's lag report): the scale/snap math calls
            # getTightBounds twice per mesh frame = ~130 ms of CPU on EVERY
            # fire spawn, a visible stutter each time the fire spread. Every
            # copy of the same prototype gets the same scale and Z, so
            # compute once per prototype and reuse.
            cached_placement = _baked_fire_mesh_placement_cache.get(prototype_index)
            if cached_placement is None:
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
                cached_placement = (Vec3(mesh_node.getScale()), mesh_node.getZ())
                _baked_fire_mesh_placement_cache[prototype_index] = cached_placement
            else:
                mesh_node.setScale(cached_placement[0])
                mesh_node.setZ(cached_placement[1])
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


def prewarm_fire_visual_caches():
    """Build one throwaway fire visual at boot so the per-prototype placement
    cache is filled before play - otherwise the FIRST fire spawn of a round
    still pays the ~300 ms getTightBounds cost as a visible stutter."""
    try:
        prewarm_root = app.render.attachNewNode("fire_visual_prewarm")
        prewarm_root.hide()
        build_ground_fire_visual(prewarm_root)
        prewarm_root.removeNode()
    except Exception as prewarm_error:
        print(f"[real-sim] fire visual prewarm skipped: {prewarm_error}")


prewarm_fire_visual_caches()


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
    if "fire_cell_ignition_counts" in globals():
        fire_cell_ignition_counts[hotspot_cell] = (
            fire_cell_ignition_counts.get(hotspot_cell, 0) + 1
        )
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
# History used by the final map. Ignitions include spread and regrowth;
# burn counts record each fire cycle that reaches OUT or fully burned.
fire_cell_ignition_counts = {}
fire_cell_burn_counts = {}
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


