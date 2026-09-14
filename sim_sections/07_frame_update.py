def update_simulation_frame(task, dt):
    """One simulated step. `update` below calls this once per rendered frame,
    or several times per frame when a fast-forward time scale is active."""
    global camera_angle, camera_pitch, last_mouse_x, last_mouse_y
    global overview_camera_angle, overview_camera_pitch
    global camera_manual_override_active, camera_yaw_offset_degrees
    global game_over_triggered, game_end_reason
    global fire_effect_update_accumulator_seconds, fire_map_update_accumulator_seconds
    global dispatch_alert_update_accumulator_seconds, status_update_accumulator_seconds
    global burn_eta_update_accumulator_seconds
    global pregame_update_accumulator_seconds

    # The drone roster is fixed during a round; refresh the cached view list
    # once per frame so the dozens of per-frame lookups all reuse one build.
    invalidate_drone_views_cache()

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
        return task.cont

    if game_over_triggered:
        clear_water_hud_target_reticle()
        # Round finished: freeze the sim while the results page is shown.
        return task.cont

    update_wind(dt)
    sanitize_all_drone_motion_states()
    record_view_dwell_time(dt)
    accumulate_drone_mode_time(dt)
    accumulate_trust_metrics(dt)
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
    # Reveal undetected fire only while a survey drone is within detection range
    # (thermal sees nearby heat); keep it hidden everywhere else. Runs every
    # frame so fires pop in/out as the drones move, not just on view change.
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

    if motion_state.sim_time_seconds >= round_duration_seconds:
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

            look_dir = compute_first_person_look_direction(
                camera_angle,
                camera_pitch,
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
    return task.cont


def update(task):
    """Rendered frame: advance the simulation, then pace the loop.

    Fast-forward profile (REAL_SIM_TIME_SCALE, July 28): the same mission runs
    with the clock multiplied, so a 10-minute round plays out in seconds. The
    scaled time is split into substeps no longer than
    SIMULATION_MAX_SUBSTEP_SECONDS so the flight physics stay stable instead of
    taking one giant leap. Scale 1.0 (the default for the 3- and 10-minute
    profiles) is exactly the old single-step behaviour.
    """
    global headless_frame_counter

    dt = ClockObject.getGlobalClock().getDt()
    if REAL_SIM_HEADLESS_FIXED_DT_SECONDS > 0.0:
        dt = REAL_SIM_HEADLESS_FIXED_DT_SECONDS
    if dt <= 0:
        dt = 1.0 / 60.0
    dt = min(dt, SIMULATION_MAX_FRAME_DT_SECONDS)
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

    scaled_dt = dt * SIMULATION_TIME_SCALE
    substep_count = max(1, int(ceil(scaled_dt / SIMULATION_MAX_SUBSTEP_SECONDS)))
    substep_dt = scaled_dt / substep_count
    result = task.cont
    for _ in range(substep_count):
        result = update_simulation_frame(task, substep_dt)
        if result is not task.cont or pregame_active or game_over_triggered:
            break
    pace_main_loop()
    return result


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


