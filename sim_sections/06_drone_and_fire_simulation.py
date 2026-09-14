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


def get_water_refill_progress_seconds(stream_state):
    progress = get_water_stream_state_value(stream_state, "refill_progress_seconds")
    return max(0.0, float(progress or 0.0))


def set_water_refill_progress_seconds(stream_state, seconds):
    if stream_state is None:
        return
    set_water_stream_state_value(
        stream_state,
        "refill_progress_seconds",
        clamp(float(seconds), 0.0, WATER_REFILL_HOLD_SECONDS),
    )


def water_tank_needs_refill(stream_state):
    return (
        not water_tank_has_spray(stream_state)
        or get_water_refill_progress_seconds(stream_state) > 0.0
    )


def water_drone_inside_refill_station(drone_root):
    if drone_root is None or drone_root.isEmpty():
        return False
    station_x, station_y = get_water_refill_station_xy()
    dx = drone_root.getX() - station_x
    dy = drone_root.getY() - station_y
    return (dx * dx + dy * dy) <= WATER_REFILL_RADIUS_METERS ** 2


def update_water_refill_progress(stream_state, drone_root, dt):
    """Refill a non-full tank after it remains inside the station for 8 s."""
    if get_water_tank_liters(stream_state) >= WATER_DRONE_TANK_CAPACITY_LITERS:
        set_water_refill_progress_seconds(stream_state, 0.0)
        return False
    if not water_drone_inside_refill_station(drone_root):
        set_water_refill_progress_seconds(stream_state, 0.0)
        return False

    progress = get_water_refill_progress_seconds(stream_state) + max(0.0, dt)
    if progress >= WATER_REFILL_HOLD_SECONDS:
        set_water_tank_liters(stream_state, WATER_DRONE_TANK_CAPACITY_LITERS)
        set_water_refill_progress_seconds(stream_state, 0.0)
    else:
        set_water_refill_progress_seconds(stream_state, progress)
    return True


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


def engage_water_suppression_if_possible(
    stream_state,
    hotspot,
    is_in_suppression_range,
    dt,
):
    if not water_hotspot_is_targetable(hotspot) or not is_in_suppression_range:
        return False
    return consume_water_tank(stream_state, dt)


def format_water_tank_label(stream_state):
    liters = get_water_tank_liters(stream_state)
    refill_progress = get_water_refill_progress_seconds(stream_state)
    if refill_progress > 0.0:
        refill_remaining = max(0, int(ceil(WATER_REFILL_HOLD_SECONDS - refill_progress)))
        return f"REFILL {refill_remaining}s"
    if liters <= WATER_DRONE_EMPTY_EPSILON_LITERS:
        return "EMPTY -> NE REFILL"
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


def update_automated_water_refill(
    drone_root,
    visual,
    propellers,
    drone_motion_state,
    stream_state,
    dt,
):
    """Send an empty automated water drone to the map-only refill station."""
    if not water_tank_needs_refill(stream_state):
        return False

    station_x, station_y = get_water_refill_station_xy()
    station_z = compute_safe_altitude_at(
        station_x,
        station_y,
        WATER_DRONE_HOME_ALTITUDE_METERS,
        WATER_DRONE_MIN_ALTITUDE_ABOVE_GROUND_METERS,
        canopy_clearance_meters=WATER_DRONE_CANOPY_CLEARANCE_METERS,
    )
    desired_vx, desired_vy, _ = compute_point_tracking_velocity(
        drone_root.getX(),
        drone_root.getY(),
        station_x,
        station_y,
        WATER_DRONE_CRUISE_SPEED_METERS_PER_SECOND * 0.82,
        WATER_REFILL_RADIUS_METERS * 1.5,
        min_speed_scale=0.0,
    )
    update_guided_drone_motion(
        drone_root,
        visual,
        propellers,
        drone_motion_state,
        desired_vx,
        desired_vy,
        station_z,
        WATER_DRONE_CRUISE_SPEED_METERS_PER_SECOND * 0.82,
        WATER_DRONE_VERTICAL_SPEED_METERS_PER_SECOND,
        dt,
        visual_tilt_scale=WATER_DRONE_AUTO_VISUAL_TILT_SCALE,
        visual_tilt_time_constant_seconds=WATER_DRONE_AUTO_VISUAL_TILT_TIME_CONSTANT_SECONDS,
    )
    update_water_refill_progress(stream_state, drone_root, dt)
    clear_water_suppression_visual(stream_state)
    return True


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

    if update_water_refill_progress(water_drone_state, water_drone, dt):
        manual_water_spray_slots.discard(1)
        clear_water_suppression_visual(water_drone_state)
        return

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
            closest_hotspot,
            in_suppression_range,
            dt,
        )
        # Manual spray by the operator: flagged so the collaboration score can
        # credit suppression the operator did personally (July 29).
        register_hotspot_water_drone_contact(
            closest_hotspot,
            lead_water_engaged,
            manual=True,
        )

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
    invalidate_drone_views_cache()


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
        "refill_progress_seconds": 0.0,
        "stream_root": stream_root,
        "stream_outer_node": None,
        "stream_inner_node": None,
        "mist_nodes": mist_nodes,
    }
    assign_follower_waypoint(follower)
    follower_drones.append(follower)
    invalidate_drone_views_cache()
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


def find_hotspot_by_id(hotspot_id):
    if hotspot_id is None:
        return None
    for hotspot in fire_hotspots:
        if id(hotspot) == hotspot_id:
            return hotspot
    return None


def committed_water_target(tracked_hotspot_id, water_slot):
    """Keep a water drone on the fire it is already working until that fire is
    OUT (or burned/gone), instead of re-picking the best target every frame.

    Aug 9, 2026: re-picking each frame made the drone flip-flop between nearby
    fires - the twitchy rotating motion Cheryl saw - and meant fires only ever
    got contained, never carried all the way to out. Committing to one fire
    fixes both the motion and the effectiveness."""
    committed = find_hotspot_by_id(tracked_hotspot_id)
    if committed is None or not water_hotspot_is_targetable(committed):
        return None
    if WATER_FOLLOWS_PAIRED_SURVEY and not water_slot_matches_hotspot_owner(
        water_slot, committed
    ):
        return None
    return committed


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


def get_water_lock_heading(state):
    if isinstance(state, dict):
        return state.get("locked_suppression_heading_degrees")
    return getattr(state, "locked_suppression_heading_degrees", None)


def set_water_lock_heading(state, value):
    if isinstance(state, dict):
        state["locked_suppression_heading_degrees"] = value
    else:
        setattr(state, "locked_suppression_heading_degrees", value)


def resolve_water_suppression_heading(state, target_hotspot, source_x, source_y):
    """Face the fire while approaching, then LOCK that heading once within
    suppression range so tiny hover drift stops swinging the nose back and
    forth - the spinning-over-the-fire motion Cheryl saw (Aug 11, 2026). The
    lock clears automatically when the drone leaves range or switches fires."""
    dx = source_x - target_hotspot.root.getX()
    dy = source_y - target_hotspot.root.getY()
    distance = (dx * dx + dy * dy) ** 0.5
    if distance <= WATER_DRONE_SUPPRESSION_RADIUS_METERS:
        locked = get_water_lock_heading(state)
        if locked is None:
            locked = water_suppression_heading_degrees(
                source_x, source_y, target_hotspot
            )
            set_water_lock_heading(state, locked)
        return locked
    set_water_lock_heading(state, None)
    return water_suppression_heading_degrees(source_x, source_y, target_hotspot)


def update_water_follower_drone(follower, dt):
    rig = follower["root"]
    water_slot = follower.get("slot", 1) + 1
    follower["motion_state"].sim_time_seconds = motion_state.sim_time_seconds
    if update_automated_water_refill(
        rig,
        follower["visual"],
        follower["propellers"],
        follower["motion_state"],
        follower,
        dt,
    ):
        follower["target_hotspot"] = None
        follower["tracked_hotspot_id"] = None
        return
    target_hotspot = committed_water_target(
        follower.get("tracked_hotspot_id"), water_slot
    )
    if target_hotspot is None:
        if WATER_FOLLOWS_PAIRED_SURVEY:
            target_hotspot = choose_paired_survey_water_hotspot(
                water_slot, rig.getX(), rig.getY()
            )
        else:
            target_hotspot = choose_water_support_hotspot_for_follower(follower)
    if not water_hotspot_is_targetable(target_hotspot):
        target_hotspot = None
    follower["target_hotspot"] = target_hotspot
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
        set_water_lock_heading(follower, None)
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
    target_heading_degrees = resolve_water_suppression_heading(
        follower,
        target_hotspot,
        rig.getX(),
        rig.getY(),
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
        target_hotspot,
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

    if update_water_refill_progress(follower, rig, dt):
        manual_water_spray_slots.discard(follower.get("slot", 0) + 1)
        clear_water_suppression_visual(follower)
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
        closest_hotspot,
        in_suppression_range,
        dt,
    )
    register_hotspot_water_drone_contact(
        closest_hotspot,
        is_engaged,
        manual=True,
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
    water_motion_state.sim_time_seconds += dt
    if update_automated_water_refill(
        water_drone,
        water_drone_visual,
        water_drone_propellers,
        water_motion_state,
        water_drone_state,
        dt,
    ):
        team_state.water_target_hotspot = None
        water_drone_state.tracked_hotspot_id = None
        return
    target_hotspot = committed_water_target(water_drone_state.tracked_hotspot_id, 1)
    if target_hotspot is None:
        if WATER_FOLLOWS_PAIRED_SURVEY:
            target_hotspot = choose_paired_survey_water_hotspot(
                1, water_drone.getX(), water_drone.getY()
            )
        else:
            target_hotspot = choose_water_support_hotspot(
                water_drone.getX(),
                water_drone.getY(),
            )
    if not water_hotspot_is_targetable(target_hotspot):
        target_hotspot = None
    team_state.water_target_hotspot = target_hotspot
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
        set_water_lock_heading(water_drone_state, None)
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
    target_heading_degrees = resolve_water_suppression_heading(
        water_drone_state,
        target_hotspot,
        water_drone.getX(),
        water_drone.getY(),
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
        target_hotspot,
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

    July 9 (Cheryl): an AUTOMATION drone must NEVER be lost to a branch or
    ground impact - the autopilot is assumed terrain-aware, and losing an
    auto drone to scenery would corrupt the trust measurement. Environment
    damage therefore only applies to manually flown drones, matching the
    drone-drone rule where a manual party is required.
    """
    for view in get_all_drone_views():
        if get_drone_view_control_mode(view) != CONTROL_MODE_MANUAL:
            continue
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
            marker_node.removeNode()

    for hotspot in visible_hotspots:
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
            # Ring only. The X is reserved for "fire out" (Cheryl, July 28).
            water_target_ring_node.show()
            water_target_cross_node.hide()
            water_target_ring_node.setScale(target_pulse_scale)
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

        # The X marks one thing only: a fire that is out (green), or a fire
        # lost to burn (dark red). While a fire is active or contained the map
        # shows the flame pin and nothing else (Cheryl, July 28).
        if hotspot.suppression_state in (FIRE_STATE_ACTIVE, FIRE_STATE_CONTAINED):
            cross_node.hide()
        else:
            cross_node.show()

        if hotspot.suppression_state == FIRE_STATE_ACTIVE:
            flame_node.show()
            halo_node.show()
            if firetruck_response_active:
                set_fire_map_flame_pin_colors(
                    flame_node,
                    FIRE_TRUCK_SUPPRESSING_COLOR,
                    FIRE_TRUCK_SUPPRESSING_CORE_COLOR,
                )
                set_fire_map_marker_color(halo_node, FIRE_TRUCK_SUPPRESSING_GLOW_COLOR)
                truck_node.hide()
            elif hotspot.water_drone_engaged:
                set_fire_map_flame_pin_colors(
                    flame_node,
                    (0.5, 0.85, 1.0, 1.0),
                    (0.72, 0.94, 1.0, 1.0),
                )
                set_fire_map_marker_color(halo_node, (0.4, 0.8, 1.0, 0.5))
                truck_node.hide()
            else:
                # Consistent marking scheme (Cheryl, July 6): a DETECTED fire
                # is BLUE everywhere (matches the cyan detected fire in the
                # world). Red = undetected fire (never on this map), green X =
                # out, dark red X = burned.
                set_fire_map_flame_pin_colors(
                    flame_node,
                    (0.25, 0.85, 1.0, 1.0),
                    (0.68, 0.95, 1.0, 1.0),
                )
                set_fire_map_marker_color(halo_node, (0.2, 0.8, 1.0, 0.5))
                truck_node.hide()
            flame_node.setScale(pulse_scale)
            halo_node.setScale(pulse_scale)
        elif hotspot.suppression_state == FIRE_STATE_CONTAINED:
            flame_node.show()
            halo_node.hide()
            if firetruck_response_active:
                set_fire_map_flame_pin_colors(
                    flame_node,
                    FIRE_TRUCK_SUPPRESSING_COLOR,
                    FIRE_TRUCK_SUPPRESSING_CORE_COLOR,
                )
                truck_node.hide()
            else:
                # Contained but still detected -> dimmer blue, same family.
                set_fire_map_flame_pin_colors(
                    flame_node,
                    (0.4, 0.72, 0.95, 0.85),
                    (0.62, 0.86, 1.0, 0.95),
                )
                truck_node.hide()
            flame_node.setScale(0.85)
        elif hotspot.suppression_state == FIRE_STATE_OUT:
            flame_node.hide()
            halo_node.hide()
            truck_node.hide()
            set_fire_map_marker_color(cross_node, FIRE_MARK_OUT_COLOR)
            cross_node.setScale(0.95)
        else:
            flame_node.hide()
            halo_node.hide()
            truck_node.hide()
            set_fire_map_marker_color(cross_node, FIRE_MARK_BURNED_COLOR)
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
    ring_node.setBin("fixed", FIRE_MAP_DRONE_RING_BIN)

    marker_node = make_fire_map_square(
        f"fire_map_{role}_{slot}_marker",
        FIRE_MAP_DRONE_MARKER_HALF_SIZE,
        marker_color,
        parent=fire_map_marker_root,
    )
    marker_node.setBin("fixed", FIRE_MAP_DRONE_MARKER_BIN)

    label_text = OnscreenText(
        text="",
        parent=fire_map_panel,
        pos=(0.0, 0.0),
        scale=0.030,
        fg=(1.0, 1.0, 1.0, 0.96),
        bg=(0.015, 0.03, 0.05, 0.82),
        shadow=(0.0, 0.0, 0.0, 0.9),
        align=TextNode.ALeft,
        mayChange=True,
    )
    label_text.textNode.setCardAsMargin(0.12, 0.12, 0.08, 0.08)
    label_text.setBin("fixed", FIRE_MAP_DRONE_LABEL_BIN)
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
        marker_node.setBin(
            "fixed",
            FIRE_MAP_DRONE_MARKER_BIN + (2 if selected else 0),
        )
        marker_node.setColorScale(
            (1.0, 1.0, 0.62, 1.0) if selected else (1.0, 1.0, 1.0, 1.0)
        )

        ring_node.setPos(map_x, 0.0, map_y)
        ring_node.setScale(1.0, 1.0, 1.0)
        ring_node.setBin(
            "fixed",
            FIRE_MAP_DRONE_RING_BIN + (2 if selected else 0),
        )
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
        label_text.setBin(
            "fixed",
            FIRE_MAP_DRONE_LABEL_BIN + (2 if selected else 0),
        )

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
        f"REFILL: cyan NE ring. Hover W {WATER_REFILL_HOLD_SECONDS:.0f}s; empty AUTO returns."
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
    detection_rate_per_second = (
        FIRE_HOTSPOT_DETECTION_PROBABILITY_PER_SECOND
        * survey_detection_probability_scale()
        * range_factor
        * altitude_factor
    )
    # Poisson form instead of rate * dt (July 28). Identical to the old value
    # at normal frame times, but it keeps the detection rate PER SECOND correct
    # when the fast-forward profile takes long substeps; the plain product
    # over-detects once rate * dt stops being small.
    detection_probability = 1.0 - exp(-max(0.0, detection_rate_per_second) * dt)
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
            if hotspot.burn_age_seconds >= current_undetected_burnout_delay_seconds():
                hotspot.suppression_state = FIRE_STATE_BURNED
                hotspot_cell = world_to_fire_cell(
                    hotspot.root.getX(),
                    hotspot.root.getY(),
                )
                fire_cell_burn_counts[hotspot_cell] = (
                    fire_cell_burn_counts.get(hotspot_cell, 0) + 1
                )
                mark_hotspot_detected(
                    hotspot,
                    motion_state.sim_time_seconds,
                    highlight_dispatch=False,
                    trigger_alarm=False,
                )
                hotspot.suppression_work_seconds = 0.0
                hotspot.ground_suppression_work_seconds = 0.0
                hotspot.water_suppression_work_seconds = 0.0
                hotspot.manual_water_suppression_work_seconds = 0.0
                hotspot.extinguished_by = None
                hotspot.extinguish_ground_share = 0.0
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
            ground_rate = 0.0
            water_rate = 0.0
            if hotspot_has_engaged_fire_truck(hotspot):
                hotspot.ground_firefighter_engaged = True
                ground_rate = current_ground_suppression_rate_per_second()
                hotspot.current_suppression_rate_per_second += ground_rate
            if hotspot.water_drone_engaged:
                engaged_water_drone_count = max(
                    1,
                    hotspot.water_drone_engagement_count,
                )
                water_rate = (
                    current_water_suppression_rate_per_second()
                    * engaged_water_drone_count
                )
                hotspot.current_suppression_rate_per_second += water_rate
            if hotspot.current_suppression_rate_per_second > 0.0:
                hotspot.suppression_work_seconds += (
                    hotspot.current_suppression_rate_per_second * dt
                )
                # Split the same work between the two sources so the report can
                # say who put each fire out (Cheryl, July 28).
                hotspot.ground_suppression_work_seconds += ground_rate * dt
                hotspot.water_suppression_work_seconds += water_rate * dt
                # July 29: also book the share of the water work that came from
                # a MANUALLY flown drone, and keep round totals that survive the
                # per-hotspot resets on reignition. The collaboration score
                # reads both.
                manual_water_work = 0.0
                if water_rate > 0.0 and hotspot.manual_water_drone_engagement_count:
                    manual_share = clamp(
                        hotspot.manual_water_drone_engagement_count
                        / max(1, hotspot.water_drone_engagement_count),
                        0.0,
                        1.0,
                    )
                    manual_water_work = water_rate * dt * manual_share
                    hotspot.manual_water_suppression_work_seconds += manual_water_work
                accumulate_round_suppression_work(
                    ground_rate * dt,
                    water_rate * dt,
                    manual_water_work,
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
                # Freeze the credit split at the moment the fire goes out.
                hotspot.extinguished_by, hotspot.extinguish_ground_share = (
                    classify_extinguish_source(
                        hotspot.ground_suppression_work_seconds,
                        hotspot.water_suppression_work_seconds,
                    )
                )
                hotspot_cell = world_to_fire_cell(
                    hotspot.root.getX(),
                    hotspot.root.getY(),
                )
                fire_cell_burn_counts[hotspot_cell] = (
                    fire_cell_burn_counts.get(hotspot_cell, 0) + 1
                )
                hotspot.water_drone_requested = False
                hotspot.water_drone_assigned = False
                hotspot.water_drone_engaged = False
                hotspot.water_drone_assignment_count = 0
                hotspot.water_drone_engagement_count = 0
        elif hotspot.suppression_state == FIRE_STATE_OUT:
            reignite_probability = 1.0 - exp(
                -max(0.0, current_reignite_probability_per_second()) * dt
            )
            if (
                hotspot.reignition_count < LONG_ROUND_MAX_REIGNITIONS_PER_HOTSPOT
                and random.random() < reignite_probability
            ):
                hotspot.suppression_state = FIRE_STATE_ACTIVE
                hotspot.reignition_count += 1
                hotspot_cell = world_to_fire_cell(
                    hotspot.root.getX(),
                    hotspot.root.getY(),
                )
                fire_cell_ignition_counts[hotspot_cell] = (
                    fire_cell_ignition_counts.get(hotspot_cell, 0) + 1
                )
                hotspot.detected = False
                hotspot.detection_age_seconds = 0.0
                hotspot.burn_age_seconds = 0.0
                hotspot.mapping_progress_seconds = 0.0
                hotspot.suppression_work_seconds = 0.0
                hotspot.ground_suppression_work_seconds = 0.0
                hotspot.water_suppression_work_seconds = 0.0
                hotspot.manual_water_suppression_work_seconds = 0.0
                hotspot.extinguished_by = None
                hotspot.extinguish_ground_share = 0.0
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
    """Ignition times for the selected 3- or 10-minute mission profile."""
    if round_is_long_profile():
        return list(LONG_ROUND_FIRE_IGNITION_SCHEDULE_SECONDS)
    schedule = [FIRE_IGNITION_DELAY_SECONDS]
    if get_fire_source_count_for_current_team() >= 2:
        schedule.append(SECOND_FIRE_IGNITION_DELAY_SECONDS)
    return schedule[:get_fire_source_count_for_current_team()]


def get_fire_source_count_for_current_team():
    """Maximum number of scheduled ignition sources for the active team size."""
    if round_is_long_profile():
        return LONG_ROUND_FIRE_SOURCE_COUNT
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
    # Fairness across team sizes (July 9 protocol): bigger teams face a
    # faster fire so a hands-off full-auto round scores the same for 2/4/6
    # drones. The multiplier divides the check interval (>1 = faster fire).
    team_spread_rate_multiplier = FIRE_TEAM_SIZE_SPREAD_RATE_MULTIPLIERS.get(
        team_total_drone_count, 1.0
    )
    effective_spread_interval_seconds = FIRE_SPREAD_CHECK_INTERVAL_SECONDS / max(
        0.1, team_spread_rate_multiplier
    )
    if fire_spread_timer_seconds < effective_spread_interval_seconds:
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
        ignition_elapsed_seconds / max(1.0, current_full_map_target_seconds()),
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
            scenario_random.shuffle(candidate_cells)
            for candidate_cell in candidate_cells[:FIRE_SPREAD_FALLBACK_RANDOM_ATTEMPTS]:
                candidate_x, candidate_y = fire_cell_to_world(*candidate_cell)
                if try_spawn_at(candidate_x, candidate_y, parent_fire_event_id):
                    return True
        return False

    spread_event_count = FIRE_SPREAD_EVENTS_PER_TICK
    if scenario_random.random() < FIRE_SPREAD_EXTRA_EVENT_PROBABILITY:
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
                or scenario_random.random() < FIRE_WORKED_SPOT_SPREAD_PROBABILITY
            )
        ]
        if not spread_sources:
            return

        source_hotspot = scenario_random.choice(spread_sources)
        if source_hotspot.root.isEmpty():
            continue
        source_x = source_hotspot.root.getX()
        source_y = source_hotspot.root.getY()

        grew_this_event = False
        for _ in range(FIRE_SPREAD_SPAWN_ATTEMPTS):
            spread_angle = scenario_random.uniform(0.0, tau)
            spread_distance = scenario_random.uniform(
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
        if water_tank_label.startswith("EMPTY"):
            tank_label = "  [EMPTY] [GO TO CYAN NE REFILL]"
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
        round_duration_seconds - motion_state.sim_time_seconds,
    )

    status_text.setText(
        f"Round Time: {format_round_clock(round_time_remaining_seconds)}/{format_round_clock(round_duration_seconds)} | Burned total: {burned_percent:.0f}%\n"
        f"{format_team_ratio_menu()}\n"
        f"{format_selected_drone_status()}\n"
        f"{format_speed_mode_menu()}\n"
    )
    update_water_tank_status_overlay()
    update_automation_message_overlay()
    update_operator_performance_overlay(performance)

