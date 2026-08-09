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
    assert engage_water_suppression_if_possible(
        water_drone_state,
        hotspot,
        True,
        2.0,
    )
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
        # July 28: a truck-worked fire is marked by the PURPLE hotspot ring,
        # not by an X. The X is reserved for "fire out" (green).
        assert truck_state.target_hotspot.fire_cluster_color == FIRE_TRUCK_SUPPRESSING_COLOR
        assert not truck_state.target_hotspot.scar_core.isHidden()
        assert truck_state.target_hotspot.scar_cross_a.isHidden()
        assert truck_state.target_hotspot.scar_core.getColor()[2] >= 0.95

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
    # The kill switch now ends the whole round and opens the final report.
    assert game_over_triggered
    assert game_end_reason == "kill_switch"
    assert results_page_active
    print("REAL_SIM_SELF_TEST kill_switch OK")
    app.userExit()


def run_collision_damage_self_test():
    # Needs >= 3 drones; the stage-select default is now the 2-drone stage,
    # so build a custom 4-drone team (set_team_* marks it as an override).
    set_team_total_drone_count(4)
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
    # Needs >= 3 drones; the stage-select default is now the 2-drone stage.
    set_team_total_drone_count(4)
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

    # July 9 protocol: in pregame the number keys pick the experiment stage
    # (after the participant ID is confirmed), no longer the S:W ratio.
    finish_participant_entry()
    handle_number_key(2)
    assert selected_stage == 2
    assert team_total_drone_count == 2 and team_water_drone_count == 1
    handle_number_key(3)
    assert selected_stage == 3
    assert get_team_survey_drone_count() == 2 and team_water_drone_count == 2
    handle_number_key(4)
    assert selected_stage == 4
    assert get_team_survey_drone_count() == 3 and team_water_drone_count == 3
    cycle_experiment_stage()
    assert selected_stage == 0
    assert team_total_drone_count == 2

    handle_number_key(2)
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


class SelfTestTask:
    """Stand-in for the Panda3D task object. update_simulation_frame only ever
    compares its return value against task.cont, so this is enough to step the
    simulation from a test without the real task manager (and without update(),
    whose pace_main_loop waits on real time)."""

    cont = 1
    done = 2


def run_collaboration_score_self_test():
    """The 100-point collaboration score (Cheryl, July 29; base reworked Aug 9).

    The base is the round's P1-P4 mission performance out of 100 (the fixed
    40/67 water tier was dropped), so a hands-off round scores exactly its base
    and only operator actions move the number off it.
    """
    select_experiment_stage(3)
    start_game_from_pregame()

    # A short stretch of real automation so the mode timelines and the fire
    # state are genuine, then the tier rule is exercised directly on the work
    # totals. Driving the drones into each configuration would depend on where
    # the fire happens to start, and this rule has to hold every time.
    set_full_automation()
    for _ in range(40):
        if game_over_triggered:
            break
        update_simulation_frame(SelfTestTask(), 0.5)

    # --- base = P1-P4 mission performance, no operator action yet -------------
    # Aug 9, 2026: the fixed 40/67 water tier is gone. The base IS the round's
    # P1-P4 performance out of 100, so a hands-off round scores exactly its base.
    reset_round_suppression_work()
    performance = compute_performance_matrix(motion_state.sim_time_seconds)
    expected_base = clamp(performance["performance_score"] * 100.0, 0.0, 100.0)
    grade = compute_collaboration_score()
    assert abs(grade["base"] - expected_base) < 1e-6, (grade["base"], expected_base)
    assert grade["credit_total"] == 0.0, grade["credits"]
    assert grade["penalty_total"] == 0.0, grade["penalties"]
    assert abs(grade["score"] - grade["base"]) < 1e-6, grade["score"]
    assert compute_final_grade()["score"] == grade["score"]
    assert collaboration_run_succeeded()
    baseline_base = grade["base"]

    # --- operator credit: one fire's worth of manual [J] spray ---------------
    full_work = (
        FIRE_SUPPRESSION_CONTAIN_DELAY_SECONDS
        + FIRE_SUPPRESSION_EXTINGUISH_DELAY_SECONDS
    )
    accumulate_round_suppression_work(0.0, full_work, full_work)
    grade = compute_collaboration_score()
    assert (
        abs(grade["credits"]["suppression sprayed manually"]
            - COLLABORATION_MANUAL_SUPPRESSION_POINTS_PER_FIRE) < 1e-6
    ), grade["credits"]
    assert abs(
        grade["score"]
        - clamp(baseline_base + COLLABORATION_MANUAL_SUPPRESSION_POINTS_PER_FIRE, 0.0, 100.0)
    ) < 1e-6, grade["score"]
    # Operator credit/cost is tracked through operator_delta from here on: the
    # base is now the P1-P4 performance, which itself shifts when fires are added
    # or drones are lost, so comparing raw scores would not isolate the operator.
    delta_after_spray = grade["operator_delta"]

    # --- operator credit: a fire found while flying by hand ------------------
    set_control_mode(CONTROL_MODE_MANUAL)
    fire_x = drone.getX() + 4.0
    fire_y = drone.getY() + 4.0
    ground_sample = sample_ground(fire_x, fire_y)
    fire_z = ground_sample[0].z if ground_sample is not None else 0.0
    manual_hotspot = build_fire_hotspot_at(fire_x, fire_y, fire_z)
    manual_hotspot.fire_event_id = 901
    fire_hotspots.append(manual_hotspot)
    mark_hotspot_detected(
        manual_hotspot,
        motion_state.sim_time_seconds,
        highlight_dispatch=False,
        trigger_alarm=False,
        detected_by_survey_slot=1,
    )
    assert manual_hotspot.detected_under_manual_control
    grade = compute_collaboration_score()
    assert (
        abs(grade["credits"]["fires found flying manually"]
            - COLLABORATION_MANUAL_DETECTION_POINTS) < 1e-6
    ), grade["credits"]
    # Finding a fire by hand adds exactly the detection credit to operator_delta.
    assert abs(
        grade["operator_delta"]
        - (delta_after_spray + COLLABORATION_MANUAL_DETECTION_POINTS)
    ) < 1e-6, grade["operator_delta"]
    delta_before_loss = grade["operator_delta"]

    # --- operator cost: a drone lost -----------------------------------------
    mark_drone_damaged("survey", 2, show_alert=False)
    grade = compute_collaboration_score()
    assert (
        abs(grade["penalties"]["drones lost"]
            - COLLABORATION_DRONE_LOSS_PENALTY_POINTS) < 1e-6
    ), grade["penalties"]
    # Losing a drone subtracts exactly the loss penalty from operator_delta.
    assert abs(
        grade["operator_delta"]
        - (delta_before_loss - COLLABORATION_DRONE_LOSS_PENALTY_POINTS)
    ) < 1e-6, grade["operator_delta"]

    # --- the composite no longer moves with team size ------------------------
    performance = compute_performance_matrix(motion_state.sim_time_seconds)
    assert performance["survey_effectiveness"] == 1.0
    assert performance["water_effectiveness"] == 1.0
    assert performance["extra_drone_bonus"] == 0.0
    assert performance["performance_score"] == performance["raw_performance_score"]

    print("REAL_SIM_SELF_TEST collaboration_score OK")
    app.userExit()


def run_trust_matrix_self_test():
    """Trust matrix M1-M4 (July 29): the behavioural metrics, not the mission
    score P1-P4. Flies a few seconds of automation, then a manual stretch, and
    checks each metric appears. Writes nothing: the run log and the per-run
    tables are exercised against a synthetic history instead."""
    select_experiment_stage(3)
    start_game_from_pregame()
    set_full_automation()
    for _ in range(60):
        update_simulation_frame(SelfTestTask(), 0.25)
    set_control_mode(CONTROL_MODE_MANUAL)
    key_map["forward"] = True
    for _ in range(40):
        update_simulation_frame(SelfTestTask(), 0.25)
    key_map["forward"] = False

    rows = compute_trust_metrics()
    assert set(rows) == {"S1", "S2", "W1", "W2"}, sorted(rows)
    for label, row in rows.items():
        assert row["flown_m"] > 1.0, (label, row["flown_m"])
    for label in ("S1", "S2"):
        # Survey drones sweep their own planned polyline, so sigma' and the
        # Hausdorff distance to that plan must both be defined.
        assert rows[label]["planned_m"] > TRUST_MIN_PLANNED_PROGRESS_METERS
        assert rows[label]["m1_sigma"] is not None
        assert rows[label]["m4_hausdorff_m"] is not None
    assert rows["S1"]["flown_manual_m"] > 0.0
    assert rows["S1"]["m2_samples"] > 0
    assert rows["S1"]["m2_clearance_mean_m"] is not None
    assert rows["S1"]["m2_clearance_min_m"] is not None
    assert rows["S1"]["m3_manual_pct"] > 0.0
    assert rows["S2"]["m2_samples"] == 0, "M2 is manual flight only"

    # Both round tables, and the per-run tables with the dS/dW/dTotal columns.
    round_columns, round_rows = build_trust_round_table(rows)
    assert round_columns[0] == "Drone" and len(round_rows) == 4
    assert compose_trust_matrix_screen_lines(rows)[1].startswith("S1")

    synthetic_history = [
        {
            "run": 1,
            "stage": 3,
            "timestamp": "",
            "round_seconds": 60.0,
            "drones": {
                "S1": {"m1_sigma": 1.0, "m3_manual_pct": 40.0},
                "S2": {"m1_sigma": 1.4, "m3_manual_pct": 0.0},
                "W1": {"m1_sigma": 1.2, "m3_manual_pct": 10.0},
            },
        },
        {
            "run": 2,
            "stage": 3,
            "timestamp": "",
            "round_seconds": 60.0,
            "drones": {
                "S1": {"m1_sigma": 1.2, "m3_manual_pct": 20.0},
                "S2": {"m1_sigma": 1.4, "m3_manual_pct": 0.0},
                "W1": {"m1_sigma": 1.2, "m3_manual_pct": 10.0},
            },
        },
    ]
    columns, table_rows = build_trust_run_table("m1_sigma", synthetic_history)
    assert columns == ["Run", "S1", "S2", "W1", "dS", "dW", "dTotal"], columns
    assert len(table_rows) == 2
    assert table_rows[0][-3:] == ["-", "-", "-"], table_rows[0]
    # survey mean 1.20 -> 1.30, water mean unchanged, team mean 1.20 -> 1.27
    assert table_rows[1][-3] == "+0.10", table_rows[1]
    assert table_rows[1][-2] == "+0.00", table_rows[1]

    columns, table_rows = build_trust_run_table("m3_manual_pct", synthetic_history)
    assert table_rows[1][-3] == "-10.00", table_rows[1]

    # Hausdorff sanity: a path offset by a constant is that far from the plan.
    plan = [(0.0, 0.0), (10.0, 0.0), (20.0, 0.0)]
    offset_path = [(x, 3.0) for x, _ in plan]
    assert abs(trust_hausdorff_distance(offset_path, plan) - 3.0) < 1e-6

    print("REAL_SIM_SELF_TEST trust_matrix OK")
    app.userExit()


def run_round_profiles_self_test():
    global round_duration_seconds
    original_duration_seconds = round_duration_seconds
    try:
        round_duration_seconds = ROUND_DURATION_SHORT_SECONDS
        short_schedule = get_fire_ignition_schedule()
        assert short_schedule[0] == FIRE_IGNITION_DELAY_SECONDS
        assert len(short_schedule) in (1, 2)
        assert current_reignite_probability_per_second() == 0.0
        assert current_full_map_target_seconds() == FIRE_FULL_MAP_TARGET_SECONDS

        round_duration_seconds = ROUND_DURATION_LONG_SECONDS
        assert get_fire_ignition_schedule() == list(
            LONG_ROUND_FIRE_IGNITION_SCHEDULE_SECONDS
        )
        assert get_fire_source_count_for_current_team() == 3
        assert current_reignite_probability_per_second() > 0.0
        assert (
            current_ground_suppression_rate_per_second()
            < GROUND_FIREFIGHTER_SUPPRESSION_RATE_PER_SECOND
        )
        assert (
            current_water_suppression_rate_per_second()
            < WATER_DRONE_ASSIST_SUPPRESSION_RATE_PER_SECOND
        )
        assert current_full_map_target_seconds() == LONG_ROUND_FULL_MAP_TARGET_SECONDS
        grade = compute_final_grade()
        assert 0.0 <= grade["score"] <= 100.0
    finally:
        round_duration_seconds = original_duration_seconds

    print("REAL_SIM_SELF_TEST round_profiles OK")
    app.userExit()


def run_sensor_visibility_self_test():
    start_game_from_pregame()
    assert thermal_view_enabled
    fire_x, fire_y = get_survey_search_sector(1).center()
    ground_sample = sample_ground(fire_x, fire_y)
    fire_z = ground_sample[0].z if ground_sample is not None else 0.0
    hotspot = build_fire_hotspot_at(fire_x, fire_y, fire_z)
    fire_hotspots.append(hotspot)

    set_camera_target_role_slot("survey", 1)
    refresh_operator_fire_visibility()
    assert thermal_view_enabled
    assert not hotspot.root.isHidden()

    set_camera_target_role_slot("water", 1)
    refresh_operator_fire_visibility()
    assert hotspot.root.isHidden()
    set_thermal_view(True)
    assert not thermal_view_enabled

    set_overhead_view(True, False)
    assert thermal_view_enabled
    assert not hotspot.root.isHidden()

    set_overhead_view(True, True)
    assert not thermal_view_enabled
    assert hotspot.root.isHidden()

    mark_hotspot_detected(
        hotspot,
        motion_state.sim_time_seconds,
        highlight_dispatch=False,
        trigger_alarm=False,
        detected_by_survey_slot=1,
    )
    refresh_operator_fire_visibility()
    assert not hotspot.root.isHidden()

    hotspot.suppression_state = FIRE_STATE_CONTAINED
    hotspot.suppression_work_seconds = FIRE_SUPPRESSION_CONTAIN_DELAY_SECONDS
    set_hotspot_visual(hotspot)
    assert not hotspot.glow.isHidden()
    assert not hotspot.beacon_a.isHidden()
    assert not hotspot.beacon_b.isHidden()
    hotspot.suppression_work_seconds = 999.0
    update_hotspot_lifecycle(0.0)
    hotspot_cell = world_to_fire_cell(hotspot.root.getX(), hotspot.root.getY())
    assert hotspot.suppression_state == FIRE_STATE_OUT
    assert fire_cell_burn_counts[hotspot_cell] == 1
    set_camera_target_role_slot("survey", 1)
    update_fire_map_popup()
    map_marker = fire_map_detected_hotspot_nodes[id(hotspot)]
    assert not map_marker.hasPythonTag("label_text")
    assert map_marker.getPythonTag("flame_node").isHidden()
    assert not map_marker.getPythonTag("cross_node").isHidden()

    southern_x, southern_z = world_to_fire_map_coords(SPAWN_X_MIN, SPAWN_Y_MIN)
    assert southern_x > FIRE_MAP_VIEW_LEFT
    assert southern_z > FIRE_MAP_VIEW_BOTTOM

    show_results_page()
    assert results_page_active
    assert compute_final_grade()["score"] <= 100.0
    exported_report_path = export_round_report_pdf()
    assert os.path.isfile(exported_report_path)
    with open(exported_report_path, "rb") as exported_report_file:
        exported_report_bytes = exported_report_file.read()
    assert b"/Type /Pages" in exported_report_bytes
    # Page count grew when the formula sheet was added (July 28).
    assert b"/Count " in exported_report_bytes
    assert b"How Every Number Is Calculated" in exported_report_bytes
    # July 29: the paragraph dumps became dense tables, so the per-fire section
    # is now titled EVERY FIRE and the trust matrix rides on page 1.
    assert b"EVERY FIRE" in exported_report_bytes
    assert b"TRUST MATRIX M1-M4" in exported_report_bytes
    assert b"BURN GRID CELLS" in exported_report_bytes
    os.unlink(exported_report_path)

    print("REAL_SIM_SELF_TEST sensor_visibility OK")
    app.userExit()


def run_water_refill_self_test():
    start_game_from_pregame()
    assert 120 < FIRE_MAP_DRONE_RING_BIN
    assert FIRE_MAP_DRONE_RING_BIN < FIRE_MAP_DRONE_MARKER_BIN
    assert FIRE_MAP_DRONE_MARKER_BIN < FIRE_MAP_DRONE_LABEL_BIN
    station_x, station_y = get_water_refill_station_xy()
    station_z = compute_safe_altitude_at(
        station_x,
        station_y,
        WATER_DRONE_HOME_ALTITUDE_METERS,
        WATER_DRONE_MIN_ALTITUDE_ABOVE_GROUND_METERS,
        canopy_clearance_meters=WATER_DRONE_CANOPY_CLEARANCE_METERS,
    )
    water_drone.setPos(station_x, station_y, station_z)
    initialize_drone_motion_state_from_pose(water_drone, water_motion_state)
    set_water_tank_liters(water_drone_state, 0.0)
    set_water_refill_progress_seconds(water_drone_state, 0.0)

    water_view = get_water_view_by_slot(1)
    set_drone_view_control_mode(water_view, CONTROL_MODE_AUTOMATION)
    route = get_water_route_world_points(water_view)
    assert route and route[-1] == (station_x, station_y)

    update_water_drone(0.5)
    assert get_water_refill_progress_seconds(water_drone_state) > 0.0
    update_water_refill_progress(
        water_drone_state,
        water_drone,
        WATER_REFILL_HOLD_SECONDS,
    )
    assert get_water_tank_liters(water_drone_state) == WATER_DRONE_TANK_CAPACITY_LITERS
    assert get_water_refill_progress_seconds(water_drone_state) == 0.0
    assert not fire_map_marker_root.find("**/water_refill_station").isEmpty()
    station_map_x, station_map_y = world_to_fire_map_coords(station_x, station_y)
    resize_x0, resize_x1, resize_z0, resize_z1 = get_fire_map_size_button_bounds()
    assert not (
        resize_x0 <= station_map_x <= resize_x1
        and resize_z0 <= station_map_y <= resize_z1
    )
    update_fire_map_popup()
    caption_blob = "\n".join(
        text_path.node().getText()
        for text_path in fire_map_panel.findAllMatches("**/+TextNode")
    )
    # Caption list matches the July 6 legend rework + July 9 fire caption
    # ("MARKS:"/"DRONES:" headings were removed on July 6; this list had
    # gone stale and failed ever since).
    for required_caption in (
        "WATER REFILL",
        "Hover W inside ring",
        "ROUTES:",
        "RESIZE",
        "DETECTED FIRE MAP",
        "orange flame = fire not found yet   turns blue when found",
    ):
        assert required_caption in caption_blob, (required_caption, caption_blob)
    assert "REFILL: cyan NE ring" in fire_map_status_text.getText()

    set_water_tank_liters(water_drone_state, 0.0)
    water_drone.setX(station_x - WATER_REFILL_RADIUS_METERS - 1.0)
    update_water_refill_progress(water_drone_state, water_drone, 2.0)
    assert get_water_refill_progress_seconds(water_drone_state) == 0.0

    print("REAL_SIM_SELF_TEST water_refill OK")
    app.userExit()


def run_camera_start_self_test():
    start_game_from_pregame()
    assert first_person_view
    assert not overview_camera_enabled
    assert camera_pitch == MISSION_START_CAMERA_PITCH_DEGREES
    launch_look_direction = compute_first_person_look_direction(
        camera_angle,
        camera_pitch,
    )
    assert launch_look_direction.z < -0.6
    assert launch_look_direction.z < 0.0

    print("REAL_SIM_SELF_TEST camera_start OK")
    app.userExit()


def run_water_target_visibility_self_test():
    start_game_from_pregame()
    water_view = get_water_view_by_slot(1)
    assert water_view is not None

    near_x = water_drone.getX() + 1.0
    near_y = water_drone.getY()
    near_ground = sample_ground(near_x, near_y)
    near_z = near_ground[0].z if near_ground is not None else 0.0
    hidden_hotspot = build_fire_hotspot_at(near_x, near_y, near_z)
    hidden_hotspot.fire_event_id = 901
    fire_hotspots.append(hidden_hotspot)

    visible_x = water_drone.getX() + 6.0
    visible_y = water_drone.getY()
    visible_ground = sample_ground(visible_x, visible_y)
    visible_z = visible_ground[0].z if visible_ground is not None else 0.0
    visible_hotspot = build_fire_hotspot_at(visible_x, visible_y, visible_z)
    visible_hotspot.fire_event_id = 902
    fire_hotspots.append(visible_hotspot)
    mark_hotspot_detected(
        visible_hotspot,
        motion_state.sim_time_seconds,
        highlight_dispatch=False,
        trigger_alarm=False,
        detected_by_survey_slot=1,
    )

    assert not water_hotspot_is_targetable(hidden_hotspot)
    assert water_hotspot_is_targetable(visible_hotspot)
    assert choose_nearest_sprayable_hotspot(
        water_drone.getX(),
        water_drone.getY(),
    ) is visible_hotspot

    set_water_tank_liters(water_drone_state, WATER_DRONE_TANK_CAPACITY_LITERS)
    starting_liters = get_water_tank_liters(water_drone_state)
    assert not engage_water_suppression_if_possible(
        water_drone_state,
        hidden_hotspot,
        True,
        1.0,
    )
    assert get_water_tank_liters(water_drone_state) == starting_liters
    assert not register_hotspot_water_drone_contact(hidden_hotspot, engaged=True)
    assert not hidden_hotspot.water_drone_assigned
    assert not hidden_hotspot.water_drone_engaged

    set_drone_view_control_mode(water_view, CONTROL_MODE_AUTOMATION)
    team_state.water_target_hotspot = hidden_hotspot
    assert get_water_current_target_for_view(water_view) is None

    print("REAL_SIM_SELF_TEST water_target_visibility OK")
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
if self_test_name == "collaboration_score":
    run_collaboration_score_self_test()
    sys.exit(0)
if self_test_name == "trust_matrix":
    run_trust_matrix_self_test()
    sys.exit(0)
if self_test_name == "round_profiles":
    run_round_profiles_self_test()
    sys.exit(0)
if self_test_name == "sensor_visibility":
    run_sensor_visibility_self_test()
    sys.exit(0)
if self_test_name == "water_refill":
    run_water_refill_self_test()
    sys.exit(0)
if self_test_name == "camera_start":
    run_camera_start_self_test()
    sys.exit(0)
if self_test_name == "water_target_visibility":
    run_water_target_visibility_self_test()
    sys.exit(0)

# Stage preselected from the environment (July 28) so the fast profile can
# open straight on the stage you asked for. The participant ID screen still
# comes first, exactly like a study run, so the exports land in that person's
# folder: main_fast.py deliberately does NOT skip it.
if REAL_SIM_AUTOSTART_PARTICIPANT:
    participant_id = sanitize_participant_id(REAL_SIM_AUTOSTART_PARTICIPANT)
if REAL_SIM_AUTOSTART_STAGE in EXPERIMENT_STAGES:
    select_experiment_stage(REAL_SIM_AUTOSTART_STAGE)

if REAL_SIM_AUTOSTART:
    # Scripted runs only (headless tuning, calibration). Full automation is
    # applied inside start_game_from_pregame now, so every round of a fast run
    # starts hands-off, not just this first one.
    start_game_from_pregame()

