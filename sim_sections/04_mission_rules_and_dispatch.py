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
    if (
        new_mode == CONTROL_MODE_MANUAL
        and water_only_stage_active
        and not pregame_active
        and view["role"] == "survey"
    ):
        # Stage 1 (July 9 protocol): the survey drone is locked to automation;
        # the operator only flies the water drone this game.
        if "status_text" in globals():
            update_status_overlay()
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
    sync_thermal_view_for_camera()
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
    """Red button: emergency stop. Every drone loses power, then the WHOLE round
    ends immediately and the final report screen is shown (Cheryl, Aug 9 2026:
    the kill switch ends the game, it does not just drop the drones)."""
    global game_over_triggered, game_end_reason
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
    # End the round now: same finalization the time-limit path uses in 07.
    game_over_triggered = True
    game_end_reason = "kill_switch"
    motion_state.input_command_buffer.clear()
    motion_state.applied_cmd = AxisState()
    motion_state.velocity = AxisState()
    motion_state.last_body_right_speed = 0.0
    motion_state.filtered_body_right_accel = 0.0
    motion_state.roll_sway_body_speed = 0.0
    motion_state.last_body_right_command_speed = 0.0
    motion_state.roll_anticipation_body_speed = 0.0
    if "status_text" in globals():
        update_status_overlay()
    show_results_page()


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


# The mission score P1-P4 (renamed from M1-M4) now lives in its own file:
#   04b_trust_matrix.py  (Cheryl, July 28)
# Nothing else moved; the scores are still computed from the same globals.



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
    if not is_active_detected_hotspot(hotspot):
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
        # Aug 9, 2026 (drone-centric fix): include CONTAINED fires, not just
        # ACTIVE ones, so a water drone stays with a fire until it is fully out
        # instead of abandoning it at containment. Without this, water drones
        # only ever contain fires and rely on the trucks to finish them, so a
        # drone-first team (few/no trucks) never actually extinguishes anything.
        return unresolved_detected_hotspots()
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
    # Accept ACTIVE or CONTAINED (not just ACTIVE) so the paired water drone
    # keeps working its fire until it is out (Aug 9, 2026 drone-centric fix).
    if (
        hotspot is None
        or hotspot.root.isEmpty()
        or not hotspot.detected
        or hotspot.suppression_state not in (FIRE_STATE_ACTIVE, FIRE_STATE_CONTAINED)
    ):
        return False
    paired_survey_slot = get_paired_survey_slot(water_slot)
    return survey_slot_owns_hotspot_event(paired_survey_slot, hotspot)


def choose_paired_water_candidate(water_slot, reference_x, reference_y):
    candidates = [
        hotspot
        for hotspot in unresolved_detected_hotspots()
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
    stream_state = get_water_view_stream_state(view)
    if water_tank_needs_refill(stream_state):
        return (current_xy, get_water_refill_station_xy())
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
    active_truck_count = max(0, min(len(fire_truck_states), active_fire_truck_count))
    for truck_index, truck_state in enumerate(fire_truck_states):
        previous_target = truck_state.target_hotspot
        # Trucks beyond the stage's active count are off duty: drop any target
        # and send them home (calibration knob, Aug 9 2026).
        truck_is_active = truck_index < active_truck_count
        if not truck_is_active or not is_unresolved_truck_hotspot(truck_state.target_hotspot):
            truck_state.target_hotspot = None
            truck_state.engaged = False

        if truck_is_active and truck_state.target_hotspot is None:
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
    candidates = [
        hotspot
        for hotspot in unresolved_detected_hotspots()
        if water_hotspot_is_targetable(hotspot)
    ]
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
    global fire_cell_ignition_counts, fire_cell_burn_counts
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
        marker_node.removeNode()
        fire_map_detected_hotspot_nodes.pop(hotspot_id, None)

    fire_burned_cells = set()
    fire_burnable_cells = build_fire_burnable_cells()
    fire_cell_ignition_counts = {}
    fire_cell_burn_counts = {}
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
    water_drone_state.refill_progress_seconds = 0.0
    water_drone_state.auto_dispatch_enabled = False
    reset_second_water_dispatch()
    manual_water_spray_slots.clear()
    clear_water_suppression_visual()

    reset_view_switch_tracking()
    reset_live_metrics_tracking()
    reset_trust_metrics_tracking()
    reset_round_suppression_work()
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


def entry_guard(handler):
    """While the participant is typing their ID, every letter belongs to the
    ID - global hotkeys (speed presets, thermal, modes, ...) must not fire."""
    def guarded_handler(*args):
        if participant_entry_active:
            return
        return handler(*args)
    return guarded_handler


def begin_participant_entry():
    """[I] on the stage-select screen reopens the participant ID prompt."""
    global participant_entry_active
    if not pregame_active:
        return
    participant_entry_active = True
    update_pregame_overlay()


def finish_participant_entry():
    global participant_entry_active, participant_id
    participant_id = sanitize_participant_id(participant_id)
    participant_entry_active = False
    # Letters typed into the ID also hit the WASD key map; clear it so the
    # drone does not fly off on launch.
    for action_name in key_map:
        key_map[action_name] = False
    update_pregame_overlay()


def handle_participant_keystroke(key_character):
    """Panda3D keystroke event: printable characters typed into the ID."""
    global participant_id
    if not (pregame_active and participant_entry_active):
        return
    if key_character in ("\x08", "\x7f"):
        participant_id = participant_id[:-1]
    elif (
        len(participant_id) < PARTICIPANT_ID_MAX_CHARS
        and key_character.isprintable()
        and (key_character.isalnum() or key_character in "-_ ")
    ):
        participant_id += key_character
    else:
        return
    update_pregame_overlay()


def handle_participant_backspace():
    global participant_id
    if not (pregame_active and participant_entry_active):
        return
    participant_id = participant_id[:-1]
    update_pregame_overlay()


def apply_selected_stage_for_launch():
    """Enforce the selected stage's team + locks right before launch.

    Scripts/tests that build a custom team with set_team_*_count keep it
    (custom_team_override); participants always launch the stage's team."""
    global water_only_stage_active, custom_team_override
    stage = get_selected_stage_definition()
    if not custom_team_override:
        set_team_total_drone_count(stage["total"])
        set_team_water_drone_count(stage["water"])
        custom_team_override = False
        water_only_stage_active = bool(stage["water_only"])
    else:
        water_only_stage_active = False


# CALIBRATION knobs per stage (Aug 9, 2026, Cheryl): the fire-truck (ground
# crew) count and the water-drone tank capacity are tuned per stage so a
# hands-off automation run reaches the SAME success rate in every scenario.
# A stage with no entry falls back to the sim defaults (full truck fleet at
# FIRE_TRUCK_COUNT, tank at DEFAULT_WATER_DRONE_TANK_CAPACITY_LITERS). Fill these
# in once the per-stage automation success rate has been measured.
STAGE_FIRE_TRUCK_COUNTS = {}           # stage number -> trucks that respond
STAGE_WATER_TANK_CAPACITY_LITERS = {}  # stage number -> water tank capacity (L)
# Aug 12, 2026 calibration: per-stage fire slowdown (full-map target seconds;
# bigger = calmer wind = smaller fire) tuned so a hands-off full-auto round lands
# on the SAME ~50% fires-out anchor in every stage. The anchor is taken from
# UAV WILDFIRE-FIREFIGHTING SIMULATION studies (not human crews): the DSPFC
# coordinated drone-swarm suppression sim reports Fire Mitigation Effectiveness
# ~50% and ~73% of fires mitigated in a hard scenario (Bristol/Springer 2024,
# "Extinguishing Wildfires in Large Scale Scenarios Using Swarms of UAVs"), and
# ~82% mitigated / FME 61% in an easy scenario. Detection ~82% (20 UAVs, DSPF) /
# ~86% (two-drone RL localisation sim) matches this sim's 73-91% detection. So
# ~50% fires out is a mid, coordinated-swarm outcome, leaving clear headroom for
# a human operator to improve on the automation. Bigger teams get a bigger fire
# so every stage is equally hard. Measured full-auto on the fixed map: stage1
# 50.9%, stage2 50.0%, stage3 49.3%, stage4 52.2%.
STAGE_FIRE_FULL_MAP_TARGET_SECONDS = {1: 380.0, 2: 420.0, 3: 300.0, 4: 290.0}


def apply_stage_resource_limits(stage_number):
    """Set the per-stage calibration knobs (active truck count, water tank
    capacity, fire slowdown) for the round about to start. Falls back to the sim
    defaults when a stage has no override."""
    global active_fire_truck_count, WATER_DRONE_TANK_CAPACITY_LITERS
    global active_full_map_target_seconds_override, round_duration_seconds
    # Per-stage round length: the warm-up (stage 0) runs longer than the scored
    # stages. See STAGE_ROUND_MINUTES_OVERRIDE.
    round_duration_seconds = get_stage_round_duration_seconds(stage_number)
    active_fire_truck_count = max(
        0,
        min(
            FIRE_TRUCK_COUNT,
            int(STAGE_FIRE_TRUCK_COUNTS.get(stage_number, FIRE_TRUCK_COUNT)),
        ),
    )
    WATER_DRONE_TANK_CAPACITY_LITERS = float(
        STAGE_WATER_TANK_CAPACITY_LITERS.get(
            stage_number, DEFAULT_WATER_DRONE_TANK_CAPACITY_LITERS
        )
    )
    active_full_map_target_seconds_override = STAGE_FIRE_FULL_MAP_TARGET_SECONDS.get(
        stage_number
    )


def apply_stage_start_control_modes():
    """Stage 1: survey drones start (and stay) in automation; the operator's
    view starts on the water drone they are supposed to fly."""
    if not water_only_stage_active:
        return
    for view in get_all_drone_views():
        if view["role"] == "survey" and not is_view_damaged(view):
            set_drone_view_control_mode(view, CONTROL_MODE_AUTOMATION)
    set_camera_target_role_slot("water", 1)


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
        app.accept(keyboard_key, entry_guard(set_key), [action, True])
        app.accept(f"{keyboard_key}-up", entry_guard(set_key), [action, False])


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
    # Size the backdrop from the ACTUAL rendered text bounds (July 9: the old
    # line-count estimate clipped the bottom of the longer stage-select
    # screen). Bounds are taken relative to pregame_root, the same space the
    # backdrop frame lives in.
    padding = 0.05
    text_bounds = pregame_text.getTightBounds(pregame_root)
    title_bounds = pregame_title.getTightBounds(pregame_root)
    if text_bounds is None:
        lines = message_text.splitlines()
        max_line_chars = max((len(line) for line in lines), default=28)
        frame_width = clamp(0.50 + (max_line_chars * 0.0125), 1.06, 1.5)
        frame_x0, frame_x1 = -frame_width * 0.5, frame_width * 0.5
        frame_z1 = 0.50
        frame_z0 = frame_z1 - clamp(0.20 + (len(lines) * 0.038), 0.48, 1.3)
        rebuild_pregame_backdrop(frame_x0, frame_x1, frame_z0, frame_z1)
        return
    low_point, high_point = text_bounds
    frame_x0 = low_point.getX() - padding
    frame_x1 = high_point.getX() + padding
    frame_z0 = low_point.getZ() - padding
    frame_z1 = high_point.getZ() + padding
    if title_bounds is not None:
        title_low, title_high = title_bounds
        frame_x0 = min(frame_x0, title_low.getX() - padding)
        frame_x1 = max(frame_x1, title_high.getX() + padding)
        # Leave room for the accent bar (0.05) above the title.
        frame_z1 = max(frame_z1, title_high.getZ() + padding + 0.05)
    rebuild_pregame_backdrop(frame_x0, frame_x1, frame_z0, frame_z1)


def toggle_round_duration():
    """Switch profiles from the mission-setup window without affecting a run."""
    global round_duration_seconds
    if not pregame_active:
        return
    round_duration_seconds = (
        ROUND_DURATION_LONG_SECONDS
        if round_duration_seconds < ROUND_DURATION_LONG_SECONDS
        else ROUND_DURATION_SHORT_SECONDS
    )
    update_pregame_overlay()


def update_pregame_overlay():
    if pregame_text is None:
        return
    if participant_entry_active:
        # Session start (July 9 protocol): the experimenter/participant types
        # a name or ID first; every exported file for this session is named
        # with it and saved in the participant's own folder.
        message_text = (
            "PARTICIPANT\n\n"
            "Type the participant name / ID\n"
            "(letters, numbers, - and _):\n\n"
            f"   {participant_id}_\n\n"
            "Results are saved to their own folder:\n"
            f"   reports/{sanitize_participant_id(participant_id)}/\n\n"
            "[BACKSPACE] erase   [ENTER] confirm"
        )
        pregame_text.setText(message_text)
        layout_pregame_overlay(message_text)
        return
    survey_count = get_team_survey_drone_count()
    setup_header = (
        "Round complete. Choose next stage, then launch.\n\n"
        if round_restart_pending_from_setup
        else ""
    )
    stage_lines = ""
    for stage_number in sorted(EXPERIMENT_STAGES):
        stage = EXPERIMENT_STAGES[stage_number]
        marker = ">" if stage_number == selected_stage else "  "
        stage_lines += (
            f"{marker}[{stage_number}] Stage {stage_number}: {stage['label']}\n"
        )
    message_text = (
        f"{setup_header}"
        f"Participant: {sanitize_participant_id(participant_id)}   [I] change\n\n"
        "CHOOSE STAGE (press its number, [G] next)\n"
        f"{stage_lines}"
        f"Team: {team_total_drone_count} drones"
        f" ({survey_count}S/{team_water_drone_count}W)\n"
        f"[B] mission profile: {format_round_clock(round_duration_seconds)}"
        f" ({'3 fire episodes + regrowth' if round_is_long_profile() else 'original profile'})\n\n"
        "HOW TO PLAY\n"
        "Keyboard:\n"
        "W/A/S/D move   Q/E altitude\n"
        f"{format_radio_pregame_controls()}"
        "Views: 1-6 / map click   Mode: M/O\n"
        "Water: P/L/K call   J spray   TAB map\n"
        f"Refill: cyan NE ring; hover W drone {WATER_REFILL_HOLD_SECONDS:.0f}s.\n"
        "Empty AUTO water drones return there themselves.\n\n"
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
    set_thermal_view(False)
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


def apply_autostart_full_automation():
    """Fast profile (Cheryl, July 28): EVERY round starts in full automation.

    main_fast.py is the only launcher that sets REAL_SIM_AUTOSTART_FULL_AUTO,
    so this is a no-op in a real study round. It runs at the end of every
    launch, not just the first one, so picking a different stage on the setup
    screen still drops straight into a hands-off run.
    """
    if not REAL_SIM_AUTOSTART_FULL_AUTO:
        return
    set_full_automation()


def start_game_from_pregame():
    global pregame_active, round_restart_pending_from_setup
    global tx12_controller_channel_kill_active, tx12_controller_kill_channel_last_value
    global tx12_controller_recenter_pending
    if not pregame_active:
        return
    if participant_entry_active:
        # Scripted/legacy launches skip the ID screen: accept what was typed.
        finish_participant_entry()
    apply_selected_stage_for_launch()
    # Pin the fire scenario to this stage so every participant faces the same map
    # (and full-auto is reproducible). No-op when REAL_SIM_FIXED_MAP is off.
    reseed_scenario_for_stage(selected_stage)
    # Apply this stage's calibration knobs (truck count, water tank capacity).
    apply_stage_resource_limits(selected_stage)
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
        apply_stage_start_control_modes()
        apply_autostart_full_automation()
        return
    deploy_survey_lead_to_sector_start()
    if "respawn_follower_drones" in globals():
        respawn_follower_drones()
    apply_stage_start_control_modes()
    apply_autostart_full_automation()
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
    f"Refill: cyan NE ring, hover {WATER_REFILL_HOLD_SECONDS:.0f}s\n"
    "Empty AUTO water returns itself\n"
    "TAB operator map   \\ dev bird\n"
    "T thermal (survey/dev only)   V camera\n"
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
        if participant_entry_active:
            # First [ENTER] confirms the typed participant ID and moves on
            # to the stage-select screen; launch needs a second [ENTER].
            finish_participant_entry()
            return
        start_game_from_pregame()
    else:
        # [ENTER] mid-round now returns to the drone-selection page (so you can
        # change the team) rather than silently restarting the same round.
        show_pregame_setup_after_round()


bind_key_pairs()
app.accept("v", entry_guard(toggle_camera_mode))
for _perspective_key in range(0, 7):
    # [0]-[4] pick the experiment stage in pregame; 1-6 switch views in game.
    app.accept(str(_perspective_key), handle_number_key, [_perspective_key])
app.accept("tab", entry_guard(toggle_operator_satellite_view))   # operator satellite (only detected fire)
app.accept("space", entry_guard(toggle_operator_satellite_view)) # alias for the operator view
app.accept("\\", entry_guard(toggle_dev_bird_view))              # DEV-ONLY full-info bird view
app.accept("f5", entry_guard(set_manual_mode))
app.accept("f6", entry_guard(set_automation_mode))
app.accept("m", entry_guard(set_manual_mode))
app.accept("o", entry_guard(set_automation_mode))
app.accept("shift-m", entry_guard(toggle_control_mode))
app.accept("u", entry_guard(toggle_tx12_controller_input))
app.accept("n", entry_guard(request_tx12_controller_recenter))
app.accept("t", entry_guard(toggle_thermal_view))
app.accept("p", entry_guard(request_water_drone_dispatch))
app.accept("l", entry_guard(request_water_follower_dispatch), [2])
app.accept("k", entry_guard(request_water_follower_dispatch), [3])
app.accept("j", entry_guard(toggle_manual_water_spray))
app.accept("h", entry_guard(toggle_controls_help))
app.accept("z", entry_guard(set_slow_speed))
app.accept("x", entry_guard(set_normal_speed))
app.accept("c", entry_guard(set_fast_speed))
app.accept("r", entry_guard(cycle_team_algorithm))
app.accept("g", entry_guard(cycle_experiment_stage))
app.accept("b", entry_guard(toggle_round_duration))
app.accept("i", entry_guard(begin_participant_entry))
app.accept("backspace", handle_participant_backspace)
app.accept("enter", confirm_pregame_or_restart)
app.accept("return", confirm_pregame_or_restart)
app.accept("num_enter", confirm_pregame_or_restart)
# Printable characters typed on the participant ID screen. The keystroke
# thrower only exists when a real window is up; headless runs call
# handle_participant_keystroke directly.
try:
    app.buttonThrowers[0].node().setKeystrokeEvent("keystroke")
    app.accept("keystroke", handle_participant_keystroke)
except Exception as keystroke_error:
    print(f"[real-sim] keystroke capture unavailable: {keystroke_error}")
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
            fill_percent_override = None
            tank_text = f"W{slot}  OFFLINE"
            fill_color = (0.9, 0.16, 0.12, 0.88)
        else:
            stream_state = get_water_view_stream_state(view)
            remaining_liters = get_water_tank_liters(stream_state)
            refill_progress = get_water_refill_progress_seconds(stream_state)
            fill_percent_override = None
            tank_percent = (
                remaining_liters / max(0.001, WATER_DRONE_TANK_CAPACITY_LITERS)
            )
            if refill_progress > 0.0:
                refill_remaining = max(
                    0,
                    int(ceil(WATER_REFILL_HOLD_SECONDS - refill_progress)),
                )
                tank_text = f"W{slot}  REFILL {refill_remaining}s"
                fill_color = (0.18, 0.9, 0.72, 0.92)
                fill_percent_override = refill_progress / WATER_REFILL_HOLD_SECONDS
            elif remaining_liters <= WATER_DRONE_EMPTY_EPSILON_LITERS:
                tank_text = f"W{slot}  EMPTY -> NE REFILL"
                fill_color = (0.9, 0.16, 0.12, 0.88)
            elif tank_percent <= 0.25:
                tank_text = f"W{slot}  {remaining_liters:.0f}L"
                fill_color = (1.0, 0.62, 0.16, 0.9)
            else:
                tank_text = f"W{slot}  {remaining_liters:.0f}L"
                fill_color = (0.24, 0.72, 1.0, 0.86)

        fill_percent = clamp(
            fill_percent_override
            if fill_percent_override is not None
            else remaining_liters / max(0.001, WATER_DRONE_TANK_CAPACITY_LITERS),
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

