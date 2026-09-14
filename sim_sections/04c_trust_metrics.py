# ---------------------------------------------------------------
# TRUST MATRIX M1-M4  (Darya's behavioural metrics, added July 29, 2026)
#
# The wildfire outcome score that used to be called M1-M4 is now the MISSION
# SCORE P1-P4 (see 04b_trust_matrix.py). M1-M4 in this file are the real trust
# metrics carried over from the single-drone obstacle-avoidance study:
#
#   M1  sigma' = length of the path actually flown / length of the path the
#       automation planned for the same progress.  Efficiency factor, dynamic.
#   M2  obstacle clearance while the operator is flying manually: distance to
#       the nearest tree or other drone, reported as mean and worst (min).
#       Dispositional - it should stabilise per person, not per round.
#   M3  manual vs automatic mode usage, drawn as the mode-usage-over-runs
#       figure from the SMC journal paper (cyan auto / orange manual bands).
#   M4  Hausdorff distance between the flown path and the automation's planned
#       route: how geometrically different the operator's idea of the route is.
#
# Everything is recorded PER DRONE PER ROUND, and every round is appended to a
# per-participant run log so the report can also show PER RUN tables with the
# run-to-run change (dS survey, dW water, dTotal team).  Cheryl, July 29:
# "everything per drone for each round, but also per run, two big tables for
# each metric".
#
# Reference for M1/M2 columns: the handwritten table in the July 29 notes
# (rows = Run 1..N, columns = W1 W2 S1 S2 dS dW dTotal).
# ---------------------------------------------------------------

# --- Sampling knobs ---------------------------------------------------------
TRUST_PATH_SAMPLE_MIN_METERS = 1.5          # one flown-path sample every 1.5 m
TRUST_PATH_SAMPLE_MAX_POINTS = 1200         # then decimate, so memory is bounded
TRUST_HAUSDORFF_MAX_PATH_POINTS = 420       # decimate again before the O(n*m) pass
TRUST_HAUSDORFF_REFERENCE_STEP_METERS = 2.0
TRUST_CLEARANCE_SAMPLE_INTERVAL_SECONDS = 0.25
TRUST_DISTANCE_TRACE_INTERVAL_SECONDS = 1.0
TRUST_CLEARANCE_TRACE_MAX_POINTS = 90
TRUST_OBSTACLE_SEARCH_RADIUS_METERS = 30.0
TRUST_MAX_STEP_METERS = 25.0                # ignore respawn/teleport jumps
TRUST_MIN_PLANNED_PROGRESS_METERS = 8.0     # below this sigma' is meaningless
# Monotone projection onto the planned S-sweep: only look a few segments ahead,
# otherwise a drone drifting sideways snaps onto a later sweep leg (they are
# only SURVEY_SECTOR_SWEEP_SPACING_METERS apart) and fakes planned progress.
TRUST_SWEEP_LOOKAHEAD_SEGMENTS = 8
TRUST_SWEEP_MAX_OFFSET_METERS = 26.0
# Water drones have no fixed route, so their planned reference is the straight
# line between the points where they actually stopped (dispatch -> fire -> ...).
TRUST_LEG_STOP_SPEED_METERS_PER_SECOND = 1.2
TRUST_LEG_STOP_SECONDS = 1.5
TRUST_LEG_MIN_METERS = 4.0

TRUST_RUN_CSV_FIELDS = (
    "run",
    "timestamp",
    "participant",
    "stage",
    "round_seconds",
    "drone",
    "role",
    "slot",
    "flown_m",
    "planned_m",
    "m1_sigma",
    "m2_clearance_mean_m",
    "m2_clearance_min_m",
    "m3_manual_pct",
    "m3_auto_pct",
    "m4_hausdorff_m",
    "events",
    "crashed",
)

# metric key, report label, printf format, higher-is-better flag
TRUST_METRIC_DEFINITIONS = (
    ("m1_sigma", "M1 path length ratio sigma'", "%.2f", False),
    ("m2_clearance_mean_m", "M2 obstacle clearance, mean (m)", "%.1f", True),
    ("m3_manual_pct", "M3 manual control (% of round)", "%.0f", False),
    ("m4_hausdorff_m", "M4 Hausdorff distance to plan (m)", "%.1f", False),
)

trust_drone_records = {}
trust_round_recorded = False
trust_round_finalized = False
trust_last_recorded_run = None
# compute_trust_metrics runs the O(n*m) Hausdorff pass, and the results page,
# the CSV export and the PDF all want the same numbers, so it is computed once
# per round and cached until the next restart.
trust_round_metrics_cache = None


# --- Small geometry helpers -------------------------------------------------
def _trust_distance_2d(ax, ay, bx, by):
    dx = ax - bx
    dy = ay - by
    return (dx * dx + dy * dy) ** 0.5


def _trust_polyline_cumulative_lengths(points):
    cumulative = [0.0]
    for index in range(1, len(points)):
        cumulative.append(
            cumulative[-1]
            + _trust_distance_2d(
                points[index][0],
                points[index][1],
                points[index - 1][0],
                points[index - 1][1],
            )
        )
    return cumulative


def _trust_project_on_segment(px, py, ax, ay, bx, by):
    """Return (distance_to_segment, t along segment in 0..1)."""
    segment_x = bx - ax
    segment_y = by - ay
    segment_length_sq = segment_x * segment_x + segment_y * segment_y
    if segment_length_sq <= 1e-9:
        return _trust_distance_2d(px, py, ax, ay), 0.0
    t = ((px - ax) * segment_x + (py - ay) * segment_y) / segment_length_sq
    t = clamp(t, 0.0, 1.0)
    nearest_x = ax + segment_x * t
    nearest_y = ay + segment_y * t
    return _trust_distance_2d(px, py, nearest_x, nearest_y), t


def _trust_densify_polyline(points, step_meters):
    """Sample a polyline every step_meters so Hausdorff sees the whole line,
    not just its corner points."""
    if len(points) < 2:
        return list(points)
    dense = [points[0]]
    for index in range(1, len(points)):
        ax, ay = points[index - 1]
        bx, by = points[index]
        span = _trust_distance_2d(ax, ay, bx, by)
        if span <= step_meters:
            dense.append((bx, by))
            continue
        step_count = int(span / step_meters)
        for step_index in range(1, step_count + 1):
            t = (step_index * step_meters) / span
            if t >= 1.0:
                break
            dense.append((ax + (bx - ax) * t, ay + (by - ay) * t))
        dense.append((bx, by))
    return dense


def _trust_decimate(points, max_points):
    if len(points) <= max_points or max_points < 2:
        return list(points)
    stride = len(points) / float(max_points)
    decimated = [points[int(index * stride)] for index in range(max_points)]
    if decimated[-1] != points[-1]:
        decimated[-1] = points[-1]
    return decimated


def trust_hausdorff_distance(path_points, reference_points):
    """Symmetric Hausdorff distance in metres between two 2D point sets."""
    if not path_points or not reference_points:
        return None
    path = _trust_decimate(path_points, TRUST_HAUSDORFF_MAX_PATH_POINTS)
    reference = _trust_decimate(reference_points, TRUST_HAUSDORFF_MAX_PATH_POINTS)

    def directed(from_points, to_points):
        worst_sq = 0.0
        for from_x, from_y in from_points:
            best_sq = None
            for to_x, to_y in to_points:
                dx = from_x - to_x
                dy = from_y - to_y
                distance_sq = dx * dx + dy * dy
                if best_sq is None or distance_sq < best_sq:
                    best_sq = distance_sq
            if best_sq is not None and best_sq > worst_sq:
                worst_sq = best_sq
        return worst_sq ** 0.5

    return max(directed(path, reference), directed(reference, path))


# --- Per-drone record -------------------------------------------------------
def reset_trust_metrics_tracking():
    """Called from restart_simulation: the trust matrix is per ROUND."""
    global trust_round_recorded, trust_round_finalized, trust_round_metrics_cache
    trust_drone_records.clear()
    trust_round_recorded = False
    trust_round_finalized = False
    trust_round_metrics_cache = None


def _trust_new_record(view, x, y):
    return {
        "label": metrics_short_label(view),
        "role": view["role"],
        "slot": view["slot"],
        "flown_total_m": 0.0,
        "flown_manual_m": 0.0,
        "flown_auto_m": 0.0,
        "last_position": None,
        "path_points": [(x, y)],
        # M4 reference, sampled in lockstep with path_points: the automation's
        # plan point for each flown sample (the flown point itself while in auto,
        # the route foot while manual). Full auto -> plan == path -> Hausdorff 0.
        "planned_path_points": [(x, y)],
        "last_plan_point": (x, y),
        "path_accum_m": 0.0,
        "path_sample_step_m": TRUST_PATH_SAMPLE_MIN_METERS,
        # planned reference: survey = its own S-sweep, water = stop-to-stop legs
        "reference_points": [] if view["role"] == "survey" else [(x, y)],
        "reference_cumulative": [],
        "reference_segment_index": 0,
        # M1 numerator/denominator. route_flown_m is the 2D distance flown while
        # a planned route was active; planned_progress_m is the arc length the
        # drone made good ALONG that live route over the same interval. In full
        # automation the drone rides its own route, so the two track together and
        # sigma' comes out ~1 (the efficiency-factor sanity value).
        "route_flown_m": 0.0,
        "planned_progress_m": 0.0,
        # Cached live-route state for M1 (see _trust_accumulate_route_progress).
        "route_cache_key": None,
        "route_cache_points": None,
        "route_cache_cumulative": None,
        "route_prev_s": 0.0,
        "leg_origin": (x, y),
        "leg_stop_timer_seconds": 0.0,
        "leg_at_stop": True,
        "clearance_sum_m": 0.0,
        "clearance_samples": 0,
        "clearance_min_m": None,
        "clearance_timer_seconds": 0.0,
        "clearance_trace": [],
        "distance_trace": [(0.0, 0.0)],
        "distance_trace_timer_seconds": 0.0,
        "event_distances_m": [],
        "water_working_previous": False,
    }


def get_trust_record(view):
    label = metrics_short_label(view)
    record = trust_drone_records.get(label)
    if record is None:
        root = view["root"]
        start_x = root.getX() if root is not None and not root.isEmpty() else 0.0
        start_y = root.getY() if root is not None and not root.isEmpty() else 0.0
        record = _trust_new_record(view, start_x, start_y)
        trust_drone_records[label] = record
    return record


def _trust_ensure_survey_reference(record, view):
    """The planned route for a survey drone is its own serpentine sector sweep.
    It is the same polyline whether the operator flies manually or hands over,
    so M1 and M4 compare like with like."""
    if record["reference_points"]:
        return
    try:
        waypoints = build_survey_sector_waypoints(get_survey_search_sector(view["slot"]))
    except Exception:
        waypoints = tuple()
    points = [(waypoint[0], waypoint[1]) for waypoint in waypoints]
    if len(points) < 2:
        return
    record["reference_points"] = points
    record["reference_cumulative"] = _trust_polyline_cumulative_lengths(points)


def _trust_update_sweep_progress(record, x, y, step_meters):
    points = record["reference_points"]
    cumulative = record["reference_cumulative"]
    if len(points) < 2 or len(cumulative) != len(points):
        return
    start_index = int(clamp(record["reference_segment_index"], 0, len(points) - 2))
    end_index = min(len(points) - 1, start_index + TRUST_SWEEP_LOOKAHEAD_SEGMENTS)
    best = None
    for index in range(start_index, end_index):
        distance, t = _trust_project_on_segment(
            x,
            y,
            points[index][0],
            points[index][1],
            points[index + 1][0],
            points[index + 1][1],
        )
        if best is None or distance < best[0]:
            best = (distance, index, t)
    if best is None or best[0] > TRUST_SWEEP_MAX_OFFSET_METERS:
        return
    _, index, t = best
    segment_length = cumulative[index + 1] - cumulative[index]
    progress = cumulative[index] + segment_length * t
    # Planned progress can never grow faster than the drone actually flew. The
    # sweep legs are only SURVEY_SECTOR_SWEEP_SPACING_METERS apart, so without
    # this clamp a drone sitting at the bottom of one column can project onto
    # the neighbouring column and be credited with a whole leg it never flew
    # (seen as sigma' 0.64 in testing). With the clamp sigma' >= 1, which is the
    # classic efficiency-factor reading: 1 = flew the ideal length.
    progress_ceiling = record["planned_progress_m"] + max(0.0, step_meters)
    if progress > record["planned_progress_m"]:
        record["planned_progress_m"] = min(progress, progress_ceiling)
        if progress <= progress_ceiling:
            record["reference_segment_index"] = index


def _trust_update_leg_progress(record, x, y, step_meters, dt):
    """Record the water drone's stop-to-stop legs. Each completed leg appends the
    stop point to reference_points, which is the polyline M4's Hausdorff pass
    compares against. M1's denominator is handled separately by
    _trust_accumulate_route_progress (live route projection), so this no longer
    writes planned_progress_m - it only tracks stops and the leg origin."""
    speed = step_meters / dt if dt > 1e-6 else 0.0
    if speed < TRUST_LEG_STOP_SPEED_METERS_PER_SECOND:
        record["leg_stop_timer_seconds"] += dt
    else:
        record["leg_stop_timer_seconds"] = 0.0
        record["leg_at_stop"] = False
    if (
        record["leg_stop_timer_seconds"] >= TRUST_LEG_STOP_SECONDS
        and not record["leg_at_stop"]
    ):
        origin_x, origin_y = record["leg_origin"]
        straight_line = _trust_distance_2d(x, y, origin_x, origin_y)
        if straight_line >= TRUST_LEG_MIN_METERS:
            record["reference_points"].append((x, y))
        record["leg_origin"] = (x, y)
        record["leg_at_stop"] = True


def _trust_arclength_of_projection(points, cumulative, px, py):
    """(arc length, offset, foot) of the point on `points` closest to (px, py).
    Arc length is measured from the start of the polyline; offset is how far
    (px, py) sits off the route; foot is that closest point's (x, y). Projecting
    both the previous and current positions onto the SAME polyline and
    subtracting the arc lengths cancels the polyline's origin, so a route that
    shifts a little between frames does not corrupt the per-step progress. The
    offset lets the caller ignore cross-country transit the plan does not
    represent; the foot is M4's per-sample reference point."""
    if len(points) < 2 or len(cumulative) != len(points):
        return 0.0, None, None
    best = None
    for index in range(len(points) - 1):
        ax, ay = points[index]
        bx, by = points[index + 1]
        distance, t = _trust_project_on_segment(px, py, ax, ay, bx, by)
        segment_length = cumulative[index + 1] - cumulative[index]
        arclength = cumulative[index] + segment_length * t
        if best is None or distance < best[0]:
            foot = (ax + (bx - ax) * t, ay + (by - ay) * t)
            best = (distance, arclength, foot)
    if best is None:
        return 0.0, None, None
    return best[1], best[0], best[2]


def _trust_route_target(record, view, x, y):
    """Identify the route the automation is planning for this drone right now, as
    (key, points). The key is a stable identity for that route; the points are
    built once, when the key first appears, and then cached - so a moving fire
    orbit or the drone's own position never destabilises the arc-length baseline.

    Survey: the serpentine sweep while searching (key 'sweep'), or a fixed orbit
    ring around the fire it is working plus the transit leg it flew in on
    (key ('orbit', fire id)). Water: the straight line from its last stop to its
    current dispatch target (key ('water', fire id)), or None when unassigned."""
    if view["role"] == "survey":
        slot = view["slot"]
        survey_xy = get_survey_drone_position_by_slot(slot)
        reference_x = survey_xy[0] if survey_xy else x
        reference_y = survey_xy[1] if survey_xy else y
        try:
            focus = get_survey_focus_hotspot(slot, reference_x, reference_y)
        except Exception:
            focus = None
        if focus is None or focus.root is None or focus.root.isEmpty():
            return ("sweep",), (record["reference_points"] or None)
        focus_id = getattr(focus, "fire_event_id", None)
        if focus_id is None:
            focus_id = id(focus)
        focus_x, focus_y = focus.root.getX(), focus.root.getY()
        # Transit leg (drone's position when it took this fire) + the orbit ring.
        ring = [(x, y)]
        for step_index in range(17):
            angle = tau * step_index / 16.0
            ring.append(
                (
                    focus_x + cos(angle) * SURVEY_FOCUS_ORBIT_RADIUS_METERS,
                    focus_y + sin(angle) * SURVEY_FOCUS_ORBIT_RADIUS_METERS,
                )
            )
        return ("orbit", focus_id), ring
    slot = view["slot"]
    # The auto water plan is "fly to the fire chosen for this slot". Use the same
    # chooser the automation uses (get_water_dispatch_target only reports a target
    # once one has been latched into the dispatch dict, which full-auto skips).
    target = None
    try:
        target = get_water_dispatch_target(slot)
    except Exception:
        target = None
    if target is None or target.root is None or target.root.isEmpty():
        try:
            target = choose_water_dispatch_target_for_slot(slot)
        except Exception:
            target = None
    if target is None or target.root is None or target.root.isEmpty():
        return None, None
    target_id = getattr(target, "fire_event_id", None)
    if target_id is None:
        target_id = id(target)
    return ("water", target_id), [record["leg_origin"], (target.root.getX(), target.root.getY())]


def _trust_refresh_route_anchor(record, view, x, y):
    """Keep the cached route and its arc-length anchor current at the drone's
    position. Used every automation step so that if the operator later takes
    manual control, the deviation measurement starts from where the drone is,
    not from a stale baseline."""
    key, route = _trust_route_target(record, view, x, y)
    if key != record.get("route_cache_key"):
        record["route_cache_key"] = key
        if route is None or len(route) < 2:
            record["route_cache_points"] = None
            record["route_cache_cumulative"] = None
            record["route_prev_s"] = 0.0
            return
        cumulative = _trust_polyline_cumulative_lengths(route)
        record["route_cache_points"] = route
        record["route_cache_cumulative"] = cumulative
        record["route_prev_s"], _, _ = _trust_arclength_of_projection(route, cumulative, x, y)
        return
    points = record["route_cache_points"]
    cumulative = record["route_cache_cumulative"]
    if points is not None:
        record["route_prev_s"], _, _ = _trust_arclength_of_projection(points, cumulative, x, y)


def _trust_accumulate_route_progress(record, view, last_xy, x, y, step_planar, is_manual):
    """Add one step to M1's numerator and denominator.

    sigma' = actual path / automated path. An AUTOMATION step IS the automated
    path by definition, so it contributes exactly 1 (numerator gain == denominator
    gain). This matches the original study, where auto mode flew the optimal
    route and M1 read 1, and it means a fully hands-off round comes out at 1.0.

    A MANUAL step is measured against the automation's planned route: the
    denominator gain is how far the projection foot advanced along that route,
    clamped to [0, distance actually flown]. Ride the plan and the foot advances
    a full step (ratio 1); deviate and it advances less (planned < flown,
    sigma' > 1). The route is cached per target and re-anchored on target change
    so a moving fire orbit never corrupts the baseline."""
    if not is_manual:
        # Automation IS the plan: the flown point is its own reference (M4 = 0)
        # and the segment counts as a full unit of efficiency (M1 = 1).
        record["route_flown_m"] += step_planar
        record["planned_progress_m"] += step_planar
        record["last_plan_point"] = (x, y)
        _trust_refresh_route_anchor(record, view, x, y)
        return
    key, route = _trust_route_target(record, view, x, y)
    if key != record.get("route_cache_key"):
        # Target changed (or first sight): rebuild + re-anchor, count nothing.
        record["route_cache_key"] = key
        if route is None or len(route) < 2:
            record["route_cache_points"] = None
            record["route_cache_cumulative"] = None
            record["route_prev_s"] = 0.0
            record["last_plan_point"] = (x, y)
            return
        cumulative = _trust_polyline_cumulative_lengths(route)
        record["route_cache_points"] = route
        record["route_cache_cumulative"] = cumulative
        s0, _, foot0 = _trust_arclength_of_projection(route, cumulative, x, y)
        record["route_prev_s"] = s0
        record["last_plan_point"] = foot0 if foot0 is not None else (x, y)
        return
    points = record["route_cache_points"]
    cumulative = record["route_cache_cumulative"]
    if points is None:
        record["last_plan_point"] = (x, y)
        return
    s_now, offset, foot = _trust_arclength_of_projection(points, cumulative, x, y)
    # The M4 reference for a manual step is the point on the automation's route
    # the operator is deviating from.
    record["last_plan_point"] = foot if foot is not None else (x, y)
    if step_planar <= 0.0:
        return
    # While the plan is the sweep, a drone flying cross-country to a fire it has
    # not been formally assigned yet is off-plan by more than a lane. That transit
    # is the automation's own choice, not inefficiency, so pause M1 (count neither
    # numerator nor denominator) instead of billing it as a longer-than-planned
    # path. Orbit/water routes hug their target, so this only trips on the sweep.
    if offset is not None and offset > TRUST_SWEEP_MAX_OFFSET_METERS:
        record["route_prev_s"] = s_now
        return
    delta = s_now - record["route_prev_s"]
    record["route_prev_s"] = s_now
    if delta < 0.0:
        # Projection wrapped past the ring seam (one lap done); count that lap's
        # last sliver as a step of good progress rather than zero.
        delta = min(step_planar, cumulative[-1] + delta) if cumulative[-1] > 0 else 0.0
        if delta < 0.0:
            delta = 0.0
    elif delta > step_planar:
        delta = step_planar
    record["route_flown_m"] += step_planar
    record["planned_progress_m"] += delta


def trust_nearest_obstacle_distance(view, x, y, z):
    """M2 obstacle: nearest tree trunk (horizontal) or other drone (3D).
    Cheryl chose trees + other drones on July 29 so the number ties into the
    3 m drone-drone crash rule as well as tree risk."""
    nearest = None
    try:
        for tree_x, tree_y in iter_nearby_tree_avoidance_positions(
            ((x, y, z),),
            TRUST_OBSTACLE_SEARCH_RADIUS_METERS,
        ):
            distance = _trust_distance_2d(x, y, tree_x, tree_y)
            if nearest is None or distance < nearest:
                nearest = distance
    except Exception:
        pass
    for other_view in get_all_drone_views():
        other_root = other_view["root"]
        if other_root is None or other_root.isEmpty():
            continue
        if other_root is view["root"]:
            continue
        dx = x - other_root.getX()
        dy = y - other_root.getY()
        dz = z - other_root.getZ()
        distance = (dx * dx + dy * dy + dz * dz) ** 0.5
        if nearest is None or distance < nearest:
            nearest = distance
    if nearest is None:
        return None
    return min(nearest, TRUST_OBSTACLE_SEARCH_RADIUS_METERS)


def _trust_water_is_working(view):
    """Rising edge of this = an 'event' tick on a water drone's row."""
    slot = view["slot"]
    try:
        if is_water_spray_active(slot):
            return True
    except Exception:
        pass
    try:
        return get_water_dispatch_target(slot) is not None
    except Exception:
        return False


def accumulate_trust_metrics(dt):
    """One sampling step of the trust matrix. Called from the frame update right
    after accumulate_drone_mode_time, so both use the same dt and the same
    simulated clock."""
    if dt <= 0.0:
        return
    now = motion_state.sim_time_seconds
    for view in get_all_drone_views():
        root = view["root"]
        if root is None or root.isEmpty():
            continue
        record = get_trust_record(view)
        x, y, z = root.getX(), root.getY(), root.getZ()
        damaged = is_view_damaged(view)
        is_manual = get_drone_view_control_mode(view) == CONTROL_MODE_MANUAL

        step_meters = 0.0
        step_planar = 0.0
        last_xy = None
        last_position = record["last_position"]
        if last_position is not None and not damaged:
            last_xy = (last_position[0], last_position[1])
            dx = x - last_position[0]
            dy = y - last_position[1]
            dz = z - last_position[2]
            step = (dx * dx + dy * dy + dz * dz) ** 0.5
            if step <= TRUST_MAX_STEP_METERS:
                step_meters = step
                # M1 compares against a 2D planned route, so its numerator is the
                # planar distance - a drone climbing in place must not read as a
                # longer-than-planned path.
                step_planar = (dx * dx + dy * dy) ** 0.5
                record["flown_total_m"] += step
                if is_manual:
                    record["flown_manual_m"] += step
                else:
                    record["flown_auto_m"] += step
                record["path_accum_m"] += step
        record["last_position"] = (x, y, z)

        if record["path_accum_m"] >= record["path_sample_step_m"]:
            record["path_accum_m"] = 0.0
            record["path_points"].append((x, y))
            # M4 reference point for this sample (see last_plan_point). Kept in
            # lockstep with path_points so the two polylines stay the same length.
            record["planned_path_points"].append(record["last_plan_point"])
            if len(record["path_points"]) > TRUST_PATH_SAMPLE_MAX_POINTS:
                # Halve the resolution of BOTH instead of growing without limit.
                record["path_points"] = record["path_points"][::2]
                record["planned_path_points"] = record["planned_path_points"][::2]
                record["path_sample_step_m"] *= 2.0

        if not damaged:
            if view["role"] == "survey":
                # Fills reference_points (the static sweep) for M4's Hausdorff.
                _trust_ensure_survey_reference(record, view)
            else:
                # Tracks water stops for M4's reference polyline + leg origin.
                _trust_update_leg_progress(record, x, y, step_meters, dt)
            # M1: automation steps count as 1 (auto flies the plan); manual steps
            # are measured against the live planned route. Replaces the frozen-
            # sweep projection that made full-auto sigma' run to ~2.4.
            _trust_accumulate_route_progress(
                record, view, last_xy, x, y, step_planar, is_manual
            )

        record["clearance_timer_seconds"] += dt
        if (
            is_manual
            and not damaged
            and record["clearance_timer_seconds"] >= TRUST_CLEARANCE_SAMPLE_INTERVAL_SECONDS
        ):
            record["clearance_timer_seconds"] = 0.0
            clearance = trust_nearest_obstacle_distance(view, x, y, z)
            if clearance is not None:
                record["clearance_sum_m"] += clearance
                record["clearance_samples"] += 1
                if (
                    record["clearance_min_m"] is None
                    or clearance < record["clearance_min_m"]
                ):
                    record["clearance_min_m"] = clearance
                record["clearance_trace"].append(
                    (record["flown_total_m"], clearance)
                )
                if len(record["clearance_trace"]) > TRUST_CLEARANCE_TRACE_MAX_POINTS * 2:
                    record["clearance_trace"] = record["clearance_trace"][::2]

        record["distance_trace_timer_seconds"] += dt
        if record["distance_trace_timer_seconds"] >= TRUST_DISTANCE_TRACE_INTERVAL_SECONDS:
            record["distance_trace_timer_seconds"] = 0.0
            record["distance_trace"].append((now, record["flown_total_m"]))

        if view["role"] == "water":
            working = _trust_water_is_working(view)
            if working and not record["water_working_previous"]:
                record["event_distances_m"].append(record["flown_total_m"])
            record["water_working_previous"] = working


# --- End of round: finish the numbers --------------------------------------
def _trust_distance_at_time(record, time_seconds):
    trace = record["distance_trace"]
    if not trace:
        return 0.0
    if time_seconds <= trace[0][0]:
        return trace[0][1]
    previous_time, previous_distance = trace[0]
    for sample_time, sample_distance in trace:
        if sample_time >= time_seconds:
            span = sample_time - previous_time
            if span <= 1e-6:
                return sample_distance
            t = (time_seconds - previous_time) / span
            return previous_distance + (sample_distance - previous_distance) * t
        previous_time, previous_distance = sample_time, sample_distance
    return trace[-1][1]


def _trust_finalize_records():
    """Close the open water leg and add the fire-detection event ticks. Safe to
    call more than once."""
    global trust_round_finalized
    if trust_round_finalized:
        return
    for record in trust_drone_records.values():
        record["distance_trace"].append(
            (motion_state.sim_time_seconds, record["flown_total_m"])
        )
        if record["role"] == "water":
            last_position = record["last_position"]
            if last_position is not None:
                origin_x, origin_y = record["leg_origin"]
                straight_line = _trust_distance_2d(
                    last_position[0],
                    last_position[1],
                    origin_x,
                    origin_y,
                )
                if straight_line >= TRUST_LEG_MIN_METERS:
                    record["reference_points"].append(
                        (last_position[0], last_position[1])
                    )
    # Survey event ticks = the fires that drone found, placed at the distance it
    # had flown when it found them.
    for hotspot in fire_hotspots:
        slot = getattr(hotspot, "detected_by_survey_slot", None)
        detected_at = getattr(hotspot, "detection_time_seconds", None)
        if slot is None or detected_at is None:
            continue
        record = trust_drone_records.get("S%d" % int(slot))
        if record is None:
            continue
        record["event_distances_m"].append(
            _trust_distance_at_time(record, detected_at)
        )
    for record in trust_drone_records.values():
        record["event_distances_m"].sort()
    trust_round_finalized = True


def _trust_reference_polyline(record):
    if record["role"] == "survey":
        return record["reference_points"]
    return record["reference_points"]


def trust_drone_sort_key(label):
    role_rank = 0 if label.startswith("S") else 1
    try:
        slot = int(label[1:])
    except (ValueError, IndexError):
        slot = 0
    return (role_rank, slot)


def trust_crash_times_for_label(label):
    """Damage events, matched back to the S1/W2 style short label."""
    times = []
    for event_time, event_label, _reason in drone_damage_events:
        parts = str(event_label).split()
        if len(parts) < 3:
            continue
        short = ("S" if parts[0].lower().startswith("survey") else "W") + parts[-1]
        if short == label:
            times.append(event_time)
    return times


def compute_trust_metrics():
    """The trust matrix for the round just played: one row per drone."""
    global trust_round_metrics_cache
    if trust_round_metrics_cache is not None:
        return trust_round_metrics_cache
    _trust_finalize_records()
    rows = {}
    for label in sorted(trust_drone_records.keys(), key=trust_drone_sort_key):
        record = trust_drone_records[label]
        flown = record["flown_total_m"]
        planned = record["planned_progress_m"]
        # sigma' numerator is the distance flown WHILE a route was active (planar),
        # matched to the planned arc length made good along that same route.
        # M1 sigma' (Cheryl, Aug 18 2026): planned / flown, where
        #   planned = the automation-equivalent body distance (what the drone
        #             travels under automation, or the automation route it made
        #             good while the operator flew), and
        #   flown   = the actual body distance travelled on task.
        # A pure-automation drone has planned == flown, so sigma' = 1.00. A
        # manual detour flies further than the automation would, so flown > planned
        # and sigma' drops below 1.00 (1.0 = flew like the automation, lower = less
        # efficient than the automation).
        route_flown = record["route_flown_m"]
        sigma = (
            (planned / route_flown)
            if route_flown >= TRUST_MIN_PLANNED_PROGRESS_METERS
            else None
        )
        clearance_mean = (
            record["clearance_sum_m"] / record["clearance_samples"]
            if record["clearance_samples"]
            else None
        )
        manual_seconds, auto_seconds = drone_mode_time_seconds.get(label, [0.0, 0.0])
        mode_total = manual_seconds + auto_seconds
        # M4: geometric disagreement between the flown path and the plan the
        # automation was executing at each sample (planned_path_points). Auto
        # samples reference the flown point itself, so a hands-off round -> ~0;
        # manual deviation from the route shows up as the Hausdorff distance.
        reference = record["planned_path_points"]
        hausdorff = None
        if len(reference) >= 2 and len(record["path_points"]) >= 2:
            hausdorff = trust_hausdorff_distance(
                record["path_points"],
                _trust_densify_polyline(
                    reference,
                    TRUST_HAUSDORFF_REFERENCE_STEP_METERS,
                ),
            )
        crash_times = trust_crash_times_for_label(label)
        rows[label] = {
            "drone": label,
            "role": record["role"],
            "slot": record["slot"],
            "flown_m": flown,
            "flown_manual_m": record["flown_manual_m"],
            "flown_auto_m": record["flown_auto_m"],
            # route_flown_m is the actual on-task body distance (the M1 "flown");
            # planned_m is the automation-equivalent distance (the M1 "planned").
            "route_flown_m": route_flown,
            "planned_m": planned,
            "m1_sigma": sigma,
            "m2_clearance_mean_m": clearance_mean,
            "m2_clearance_min_m": record["clearance_min_m"],
            "m2_samples": record["clearance_samples"],
            "m3_manual_pct": (manual_seconds / mode_total * 100.0) if mode_total else 0.0,
            "m3_auto_pct": (auto_seconds / mode_total * 100.0) if mode_total else 0.0,
            "m4_hausdorff_m": hausdorff,
            "events": len(record["event_distances_m"]),
            "crashed": 1 if crash_times else 0,
        }
    return rows


def build_trust_round_figure_payload(rows):
    """Per-drone bands for the mode-usage-over-runs figure, in PATH LENGTH
    space (the paper's horizontal axis is length, not time)."""
    round_seconds = max(1.0, motion_state.sim_time_seconds)
    payload = {}
    for label, row in rows.items():
        record = trust_drone_records.get(label)
        if record is None:
            continue
        timeline = drone_mode_timeline.get(label, [])
        segments = []
        for index, (segment_start, state) in enumerate(timeline):
            segment_end = (
                timeline[index + 1][0]
                if index + 1 < len(timeline)
                else round_seconds
            )
            start_m = _trust_distance_at_time(record, segment_start)
            end_m = _trust_distance_at_time(record, segment_end)
            state_name = (
                "auto" if state is True else ("manual" if state is False else "offline")
            )
            segments.append([round(start_m, 2), round(end_m, 2), state_name])
        payload[label] = {
            "flown_m": round(row["flown_m"], 2),
            "planned_m": round(row["planned_m"], 2),
            "segments": segments,
            "events": [round(value, 2) for value in record["event_distances_m"]],
            "crashes": [
                round(_trust_distance_at_time(record, crash_time), 2)
                for crash_time in trust_crash_times_for_label(label)
            ],
            "clearance": [
                [round(distance, 2), round(clearance, 2)]
                for distance, clearance in _trust_decimate(
                    record["clearance_trace"],
                    TRUST_CLEARANCE_TRACE_MAX_POINTS,
                )
            ],
        }
    return payload


# --- Run history (per participant, across rounds) ---------------------------
def _trust_reports_dir():
    try:
        base_dir = os.path.dirname(os.path.abspath(__file__))
    except Exception:
        base_dir = os.getcwd()
    return os.path.join(base_dir, "reports", get_participant_folder_name())


def get_trust_history_csv_path():
    participant = get_participant_folder_name() or "unknown"
    return os.path.join(_trust_reports_dir(), "%s_trust_runs.csv" % participant)


def get_trust_history_figure_path():
    participant = get_participant_folder_name() or "unknown"
    return os.path.join(_trust_reports_dir(), "%s_trust_runs.json" % participant)


def _trust_parse_float(text_value):
    try:
        if text_value in ("", None):
            return None
        return float(text_value)
    except (TypeError, ValueError):
        return None


def load_trust_run_history():
    """Every round this participant has played, oldest first, from the run log.
    Returns [{run, stage, timestamp, round_seconds, drones: {label: row}}]."""
    import csv

    path = get_trust_history_csv_path()
    if not os.path.exists(path):
        return []
    runs = {}
    try:
        with open(path, "r", newline="") as history_file:
            for row in csv.DictReader(history_file):
                try:
                    run_number = int(row.get("run") or 0)
                except (TypeError, ValueError):
                    continue
                run = runs.setdefault(
                    run_number,
                    {
                        "run": run_number,
                        "stage": row.get("stage", ""),
                        "timestamp": row.get("timestamp", ""),
                        "round_seconds": _trust_parse_float(row.get("round_seconds")),
                        "drones": {},
                    },
                )
                label = row.get("drone") or ""
                if not label:
                    continue
                run["drones"][label] = {
                    "drone": label,
                    "role": row.get("role", ""),
                    "flown_m": _trust_parse_float(row.get("flown_m")),
                    "planned_m": _trust_parse_float(row.get("planned_m")),
                    "m1_sigma": _trust_parse_float(row.get("m1_sigma")),
                    "m2_clearance_mean_m": _trust_parse_float(
                        row.get("m2_clearance_mean_m")
                    ),
                    "m2_clearance_min_m": _trust_parse_float(
                        row.get("m2_clearance_min_m")
                    ),
                    "m3_manual_pct": _trust_parse_float(row.get("m3_manual_pct")),
                    "m3_auto_pct": _trust_parse_float(row.get("m3_auto_pct")),
                    "m4_hausdorff_m": _trust_parse_float(row.get("m4_hausdorff_m")),
                    "events": _trust_parse_float(row.get("events")),
                    "crashed": _trust_parse_float(row.get("crashed")),
                }
    except Exception as history_error:
        print("[real-sim] could not read the trust run log: %s" % history_error)
        return []
    return [runs[key] for key in sorted(runs.keys())]


def load_trust_run_figures():
    import json

    path = get_trust_history_figure_path()
    if not os.path.exists(path):
        return []
    try:
        with open(path, "r") as figure_file:
            payload = json.load(figure_file)
    except Exception as figure_error:
        print("[real-sim] could not read the trust figure log: %s" % figure_error)
        return []
    if isinstance(payload, list):
        return payload
    return []


def record_trust_round(force=False):
    """Append this round to the participant's trust run log. Returns the run
    number, or None when there is nothing to record."""
    global trust_round_recorded, trust_last_recorded_run
    import csv
    import datetime
    import json

    if trust_round_recorded and not force:
        return trust_last_recorded_run
    rows = compute_trust_metrics()
    if not rows:
        return None

    history = load_trust_run_history()
    run_number = (max(run["run"] for run in history) + 1) if history else 1
    generated_at = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    stage = globals().get("selected_stage", 0)
    round_seconds = round(motion_state.sim_time_seconds, 2)

    def csv_value(value, digits=2):
        if value is None:
            return ""
        return round(value, digits)

    try:
        os.makedirs(_trust_reports_dir(), exist_ok=True)
        csv_path = get_trust_history_csv_path()
        write_header = not os.path.exists(csv_path)
        with open(csv_path, "a", newline="") as history_file:
            writer = csv.writer(history_file)
            if write_header:
                writer.writerow(TRUST_RUN_CSV_FIELDS)
            for label in sorted(rows.keys(), key=trust_drone_sort_key):
                row = rows[label]
                writer.writerow((
                    run_number,
                    generated_at,
                    get_participant_folder_name(),
                    stage,
                    round_seconds,
                    label,
                    row["role"],
                    row["slot"],
                    csv_value(row["flown_m"], 1),
                    csv_value(row["planned_m"], 1),
                    csv_value(row["m1_sigma"], 3),
                    csv_value(row["m2_clearance_mean_m"], 2),
                    csv_value(row["m2_clearance_min_m"], 2),
                    csv_value(row["m3_manual_pct"], 1),
                    csv_value(row["m3_auto_pct"], 1),
                    csv_value(row["m4_hausdorff_m"], 2),
                    row["events"],
                    row["crashed"],
                ))
    except Exception as write_error:
        print("[real-sim] could not append to the trust run log: %s" % write_error)
        return None

    try:
        figures = load_trust_run_figures()
        figures.append({
            "run": run_number,
            "stage": stage,
            "timestamp": generated_at,
            "round_seconds": round_seconds,
            "drones": build_trust_round_figure_payload(rows),
        })
        with open(get_trust_history_figure_path(), "w") as figure_file:
            json.dump(figures, figure_file)
    except Exception as figure_error:
        print("[real-sim] could not append to the trust figure log: %s" % figure_error)

    trust_round_recorded = True
    trust_last_recorded_run = run_number
    return run_number


# --- Tables -----------------------------------------------------------------
def trust_history_drone_labels(history):
    labels = set()
    for run in history:
        labels.update(run["drones"].keys())
    return sorted(labels, key=trust_drone_sort_key)


def _trust_role_mean(run, labels, metric_key, role_prefix):
    values = [
        run["drones"][label][metric_key]
        for label in labels
        if label in run["drones"]
        and label.startswith(role_prefix)
        and run["drones"][label].get(metric_key) is not None
    ]
    if not values:
        return None
    return sum(values) / len(values)


def build_trust_run_table(metric_key, history=None):
    """Per-RUN table for one metric: rows = runs, columns = each drone plus the
    run-to-run change for survey (dS), water (dW) and the whole team (dTotal).
    Mirrors the handwritten table in Cheryl's July 29 notes."""
    if history is None:
        history = load_trust_run_history()
    labels = trust_history_drone_labels(history)
    columns = ["Run"] + labels + ["dS", "dW", "dTotal"]
    rows = []
    previous = {"S": None, "W": None, "T": None}
    for run in history:
        cells = ["%d" % run["run"]]
        for label in labels:
            value = run["drones"].get(label, {}).get(metric_key)
            cells.append("-" if value is None else "%.2f" % value)
        survey_mean = _trust_role_mean(run, labels, metric_key, "S")
        water_mean = _trust_role_mean(run, labels, metric_key, "W")
        all_values = [
            run["drones"][label][metric_key]
            for label in labels
            if label in run["drones"]
            and run["drones"][label].get(metric_key) is not None
        ]
        team_mean = (sum(all_values) / len(all_values)) if all_values else None
        for key, current in (("S", survey_mean), ("W", water_mean), ("T", team_mean)):
            if current is None or previous[key] is None:
                cells.append("-")
            else:
                cells.append("%+.2f" % (current - previous[key]))
            if current is not None:
                previous[key] = current
        rows.append(cells)
    return columns, rows


def build_trust_round_table(rows=None):
    """Per-DRONE table for the round just played."""
    if rows is None:
        rows = compute_trust_metrics()
    columns = [
        "Drone",
        "Planned m",
        "Flown m",
        "M1 sigma'",
        "M2 mean m",
        "M2 min m",
        "M3 man %",
        "M3 auto %",
        "M4 Haus m",
        "Events",
    ]
    table_rows = []
    for label in sorted(rows.keys(), key=trust_drone_sort_key):
        row = rows[label]

        def cell(value, fmt="%.1f"):
            return "-" if value is None else fmt % value

        table_rows.append([
            label,
            cell(row["planned_m"]),
            cell(row.get("route_flown_m")),
            cell(row["m1_sigma"], "%.2f"),
            cell(row["m2_clearance_mean_m"], "%.1f"),
            cell(row["m2_clearance_min_m"], "%.1f"),
            cell(row["m3_manual_pct"], "%.0f"),
            cell(row["m3_auto_pct"], "%.0f"),
            cell(row["m4_hausdorff_m"], "%.1f"),
            "%d" % row["events"],
        ])
    return columns, table_rows


def compose_trust_matrix_lines(rows=None):
    """Compact text form for the in-app results page."""
    if rows is None:
        rows = compute_trust_metrics()
    lines = ["TRUST MATRIX M1-M4 (per drone, this round)"]
    if not rows:
        lines.append("  no drone track recorded")
        return lines
    for label in sorted(rows.keys(), key=trust_drone_sort_key):
        row = rows[label]

        def value(key, fmt="%.1f"):
            return "-" if row[key] is None else fmt % row[key]

        lines.append(
            "  %-3s M1 %s   M2 %s / %s m   M3 man %s%%   M4 %s m"
            % (
                label,
                value("m1_sigma", "%.2f"),
                value("m2_clearance_mean_m", "%.1f"),
                value("m2_clearance_min_m", "%.1f"),
                value("m3_manual_pct", "%.0f"),
                value("m4_hausdorff_m", "%.1f"),
            )
        )
    return lines


def compose_trust_matrix_screen_lines(rows=None):
    """Narrow column version for the results page, so it cannot run into the
    fire-history map on the right."""
    if rows is None:
        rows = compute_trust_metrics()
    lines = ["      M1      M2 mean/min   M3 man   M4"]
    if not rows:
        return ["no drone track recorded"]
    for label in sorted(rows.keys(), key=trust_drone_sort_key):
        row = rows[label]

        def value(key, fmt="%.1f"):
            return "-" if row[key] is None else fmt % row[key]

        lines.append(
            "%-3s  %-6s  %5s /%5s   %3s%%   %5s"
            % (
                label,
                value("m1_sigma", "%.2f"),
                value("m2_clearance_mean_m", "%.1f"),
                value("m2_clearance_min_m", "%.1f"),
                value("m3_manual_pct", "%.0f"),
                value("m4_hausdorff_m", "%.1f"),
            )
        )
    return lines


def compose_trust_metric_formula_lines():
    """Appended to the exported formula sheet so the trust matrix is documented
    with the same 'every number, in full' rule as the mission score."""
    lines = []

    def block(title, rows):
        lines.append(title)
        lines.extend(("  " + row) if row else "" for row in rows)
        lines.append("")

    block("12. TRUST MATRIX M1-M4 (behavioural, per drone per run)", (
        "P1-P4 above score the WILDFIRE OUTCOME. M1-M4 here measure the OPERATOR",
        "and carry over from the single-drone obstacle-avoidance study. They are",
        "recorded per drone per round and appended to",
        "reports/<participant>/<participant>_trust_runs.csv, one row per drone",
        "per run, so run-to-run change can be read straight out of the file.",
    ))

    block("13. M1 PATH LENGTH RATIO (sigma', dynamic)", (
        "  sigma' = length of the path actually flown / length of the path the",
        "           automation planned for the same progress",
        "",
        "  Survey drone: the planned route is its own serpentine sector sweep",
        "  (waypoint columns %.0f m apart). Each step the drone's position is"
        % SURVEY_SECTOR_SWEEP_SPACING_METERS,
        "  projected onto that polyline and the planned length is the furthest",
        "  arc length reached (monotone, lookahead %d segments, ignored beyond"
        % TRUST_SWEEP_LOOKAHEAD_SEGMENTS,
        "  %.0f m off route so a drifting drone cannot skip to a later leg)."
        % TRUST_SWEEP_MAX_OFFSET_METERS,
        "  Water drone: no fixed route, so the reference is the straight line",
        "  between the points where it stopped (below %.1f m/s for %.1f s):"
        % (TRUST_LEG_STOP_SPEED_METERS_PER_SECOND, TRUST_LEG_STOP_SECONDS),
        "  planned = sum of straight-line legs, flown = distance actually flown.",
        "",
        "  Planned progress can never grow faster than the distance actually",
        "  flown in the same step, so sigma' >= 1 by construction.",
        "",
        "  sigma' = 1 means the operator flew the ideal length. Above 1 is a",
        "  detour; it is left blank when planned progress < %.0f m."
        % TRUST_MIN_PLANNED_PROGRESS_METERS,
    ))

    block("14. M2 OBSTACLE CLEARANCE (dispositional)", (
        "  Sampled every %.2f s while that drone is under MANUAL control only:"
        % TRUST_CLEARANCE_SAMPLE_INTERVAL_SECONDS,
        "",
        "    clearance = min(horizontal distance to nearest tree trunk,",
        "                    3D distance to nearest other drone)",
        "    M2_mean = mean(clearance samples)     M2_min = worst sample",
        "",
        "  Search radius %.0f m; if nothing is within it the sample is capped"
        % TRUST_OBSTACLE_SEARCH_RADIUS_METERS,
        "  there. Drone-drone distance matters because manual drones get no",
        "  separation assist and both go offline inside %.0f m."
        % DRONE_CRASH_RADIUS_METERS,
        "  This one should be stable for a person across runs, not per round.",
    ))

    block("15. M3 MODE USAGE (dynamic)", (
        "  manual_pct = manual seconds / (manual + auto seconds) x 100",
        "  auto_pct   = 100 - manual_pct",
        "",
        "  Time is accrued per drone per frame; damaged drones accrue nothing.",
        "  The report draws the same figure as the SMC journal paper: one band",
        "  per run with the horizontal axis in PATH LENGTH, cyan for automatic,",
        "  orange for manual, green ticks for events (fires found, water sent),",
        "  a red X where the drone was lost, the magenta dashed line at the",
        "  planned length and the black trace showing distance to obstacle.",
        "  Falling manual_pct over runs is the growth of trust in automation.",
    ))

    block("16. M4 HAUSDORFF DISTANCE (dispositional)", (
        "  Symmetric Hausdorff distance, in metres, between the flown path and",
        "  the same planned route M1 uses:",
        "",
        "    h(A,B) = max over a in A of min over b in B of ||a - b||",
        "    M4     = max(h(flown, planned), h(planned, flown))",
        "",
        "  The flown path is sampled every %.1f m (decimated to %d points) and"
        % (TRUST_PATH_SAMPLE_MIN_METERS, TRUST_HAUSDORFF_MAX_PATH_POINTS),
        "  the planned route is densified every %.1f m before the comparison."
        % TRUST_HAUSDORFF_REFERENCE_STEP_METERS,
        "  Small M4 = execution refinement (same route, tidier flying).",
        "  Large M4 = fundamental disagreement with the automation's plan.",
    ))

    while lines and lines[-1] == "":
        lines.pop()
    return lines
