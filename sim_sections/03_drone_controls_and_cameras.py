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
# Experiment session state (July 9 protocol): the sim boots into participant
# ID entry, then the stage-select screen. The ID names the per-participant
# export folder; the stage fixes team size and the stage-1 water-only lock.
participant_id = ""
participant_entry_active = True
selected_stage = 0
water_only_stage_active = False
# Scripts/tests may build custom teams by calling set_team_*_count directly;
# that sets this flag and launch then respects the custom team instead of
# resetting it to the selected stage's team. Stage buttons clear the flag.
custom_team_override = False
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
    refill_progress_seconds: float = 0.0
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
    global team_water_drone_count, custom_team_override
    if not pregame_active:
        return
    custom_team_override = True
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
    global team_total_drone_count, team_water_drone_count, custom_team_override
    if not pregame_active:
        return
    custom_team_override = True
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


def select_experiment_stage(stage_number):
    """[0]-[4] on the stage-select screen (July 9 protocol). The stage fixes
    the team size/split and whether the survey drone is operator-locked."""
    global selected_stage, water_only_stage_active, custom_team_override
    if not pregame_active or stage_number not in EXPERIMENT_STAGES:
        return
    selected_stage = stage_number
    stage = EXPERIMENT_STAGES[stage_number]
    set_team_total_drone_count(stage["total"])
    set_team_water_drone_count(stage["water"])
    custom_team_override = False  # stage-picked team, not a script override
    update_pregame_overlay()


def cycle_experiment_stage():
    """[G] steps through the stages in order on the stage-select screen."""
    if not pregame_active or participant_entry_active:
        return
    stage_numbers = sorted(EXPERIMENT_STAGES)
    try:
        current_index = stage_numbers.index(selected_stage)
    except ValueError:
        current_index = 0
    select_experiment_stage(
        stage_numbers[(current_index + 1) % len(stage_numbers)]
    )


def get_selected_stage_definition():
    return EXPERIMENT_STAGES.get(selected_stage, EXPERIMENT_STAGES[0])


def sanitize_participant_id(raw_participant_id):
    cleaned = "".join(
        character if (character.isalnum() or character in "-_") else "_"
        for character in str(raw_participant_id).strip()
    ).strip("_")
    return cleaned[:PARTICIPANT_ID_MAX_CHARS] or "anonymous"


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
        hotspot.manual_water_drone_engagement_count = 0


def water_hotspot_is_targetable(hotspot):
    """True only for a genuinely survey-detected, operator-visible live fire."""
    return (
        hotspot is not None
        and not hotspot.root.isEmpty()
        and hotspot.detected
        and hotspot.detection_time_seconds is not None
        and hotspot.suppression_state in (FIRE_STATE_ACTIVE, FIRE_STATE_CONTAINED)
    )


def register_hotspot_water_drone_contact(hotspot, engaged=False, manual=False):
    """manual=True means the engaged water drone is being flown by the operator
    (manual [J] spray), which is what earns operator credit in the
    collaboration score."""
    if not water_hotspot_is_targetable(hotspot):
        return False
    hotspot.water_drone_assigned = True
    hotspot.water_drone_assignment_count += 1
    if engaged:
        hotspot.water_drone_engaged = True
        hotspot.water_drone_engagement_count += 1
        if manual:
            hotspot.manual_water_drone_engagement_count += 1
    return True


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


_drone_views_cache = None


def invalidate_drone_views_cache():
    """Drop the cached view list. The roster only changes at round setup
    (follower spawn / teardown), so callers invalidate there; the frame loop
    also invalidates once per frame as a safety net. See get_all_drone_views."""
    global _drone_views_cache
    _drone_views_cache = None


def get_all_drone_views():
    """Ordered camera/control targets: all survey drones, then all water drones.

    The dispatch and camera code calls this dozens of times per frame, and the
    roster is constant during a round, so the built list is cached and only
    rebuilt after invalidate_drone_views_cache() (roster change / frame start)."""
    global _drone_views_cache
    if _drone_views_cache is not None:
        return _drone_views_cache
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
    _drone_views_cache = views
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


def compute_first_person_look_direction(yaw_degrees, pitch_degrees):
    yaw = radians(yaw_degrees)
    pitch = radians(pitch_degrees)
    return Vec3(
        -sin(yaw) * cos(pitch),
        cos(yaw) * cos(pitch),
        sin(pitch),
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
    sync_thermal_view_for_camera()


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
    sync_thermal_view_for_camera()
    refresh_operator_fire_visibility()
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
    # During setup, [0]-[4] pick the experiment stage (July 9 protocol). While
    # the participant is typing their ID the digits belong to the ID instead.
    # In flight, number keys switch by fixed role slots: 1-3 survey, 4-6 water.
    if pregame_active:
        if participant_entry_active:
            return
        if key_number in EXPERIMENT_STAGES:
            select_experiment_stage(key_number)
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
    sync_thermal_view_for_camera()
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


def survey_drone_within_detection_range(hotspot):
    """True if any active survey (thermal) drone is close enough to physically
    see this still-undetected fire. Mirrors the detection radius so an
    undetected fire is only REVEALED once a survey drone is over it, instead of
    every fire on the map showing up in the survey view (Aug 11, 2026)."""
    root = getattr(hotspot, "root", None)
    if root is None or root.isEmpty():
        return False
    reveal_radius = FIRE_HOTSPOT_DETECTION_RADIUS_METERS
    for survey_root in get_active_survey_drone_roots():
        if survey_root is None or survey_root.isEmpty():
            continue
        dx = root.getX() - survey_root.getX()
        dy = root.getY() - survey_root.getY()
        if (dx * dx + dy * dy) ** 0.5 <= reveal_radius:
            return True
    return False


def refresh_operator_fire_visibility():
    """Apply role-appropriate fire visibility.

    Detected fires are always shown. An UNDETECTED fire is only revealed when a
    survey (thermal) drone is within its detection radius - so no view shows the
    whole map's fires at once. The operator satellite map only ever shows
    detected fires; the dev bird view shows everything for debugging.
    """
    dev_bird_view = overview_camera_enabled and not overview_is_operator
    operator_satellite = overview_camera_enabled and overview_is_operator
    for hotspot in fire_hotspots:
        root = getattr(hotspot, "root", None)
        if root is None or root.isEmpty():
            continue
        if hotspot.detected:
            root.show()
        elif dev_bird_view:
            root.show()
        elif operator_satellite:
            root.hide()
        elif survey_drone_within_detection_range(hotspot):
            root.show()
        else:
            root.hide()


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
        water_hotspot_is_targetable(hotspot)
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
            if tank_label.startswith("EMPTY")
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
    if detected_by_survey_slot is not None and not was_detected:
        # Was the operator flying that survey drone when it found this fire?
        # The collaboration score credits the person for those (July 29).
        detector_view = get_drone_view_for_role_slot("survey", detected_by_survey_slot)
        hotspot.detected_under_manual_control = (
            detector_view is not None
            and get_drone_view_control_mode(detector_view) == CONTROL_MODE_MANUAL
        )
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


def current_camera_supports_thermal_view():
    if overview_camera_enabled:
        return not overview_is_operator
    selected_view = get_selected_drone_view()
    return selected_view is not None and selected_view["role"] == "survey"


def set_thermal_view(enabled):
    global thermal_view_enabled
    thermal_view_enabled = bool(enabled) and current_camera_supports_thermal_view()
    apply_thermal_view()
    refresh_operator_fire_visibility()


def sync_thermal_view_for_camera():
    """Survey cameras and the developer bird view start in thermal mode.

    Water cameras and the operator [Tab] satellite view are always normal.
    """
    set_thermal_view(current_camera_supports_thermal_view())


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


