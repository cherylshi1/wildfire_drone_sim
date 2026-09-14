from math import cos, radians, sin


_CANOPY_SPATIAL_CELL_METERS = 8.0
_canopy_spatial_indexes = {}


def approach(current_value, target_value, rate_per_second, dt):
    delta = target_value - current_value
    max_step = rate_per_second * dt
    if abs(delta) <= max_step:
        return target_value
    return current_value + max_step if delta > 0 else current_value - max_step


def compute_desired_velocity(camera_heading_degrees, key_map, speed):
    forward_input = int(key_map["forward"]) - int(key_map["backward"])
    right_input = int(key_map["right"]) - int(key_map["left"])
    vertical_input = int(key_map["up"]) - int(key_map["down"])

    if forward_input or right_input:
        inverse_input_length = (forward_input * forward_input + right_input * right_input) ** -0.5
        forward_input *= inverse_input_length
        right_input *= inverse_input_length

    angle = radians(camera_heading_degrees)
    return (
        (right_input * cos(angle) - forward_input * sin(angle)) * speed,
        (right_input * sin(angle) + forward_input * cos(angle)) * speed,
        vertical_input * speed,
    )


def update_axis_velocity(current_velocity, commanded_velocity, dt, accel_rate, decel_rate):
    rate = accel_rate if abs(commanded_velocity) > abs(current_velocity) else decel_rate
    return approach(current_velocity, commanded_velocity, rate, dt)


def count_detected_hotspots(hotspots):
    return sum(1 for hotspot in hotspots if hotspot.detected)


def choose_nearest_undetected_hotspot(hotspots, drone_x, drone_y):
    undetected_hotspots = [
        hotspot
        for hotspot in hotspots
        if (getattr(hotspot, "suppression_state", "active") != "out") and (not hotspot.detected)
    ]
    if not undetected_hotspots:
        return None

    return min(
        undetected_hotspots,
        key=lambda hotspot: (hotspot.root.getX() - drone_x) ** 2 + (hotspot.root.getY() - drone_y) ** 2,
    )


def build_tree_canopy_samples(tree_nodes, bounds_space=None):
    canopy_samples = []
    for tree_node in tree_nodes:
        leaves = tree_node.find("**/g2")
        if not leaves.isEmpty():
            leaves_bounds = (
                leaves.getTightBounds(bounds_space)
                if bounds_space is not None
                else leaves.getTightBounds()
            )
            tree_bounds = (
                tree_node.getTightBounds(bounds_space)
                if bounds_space is not None
                else tree_node.getTightBounds()
            )
            bounds = tree_bounds if tree_bounds is not None else leaves_bounds
            top_z = leaves_bounds[1].z if leaves_bounds is not None else None
        else:
            bounds = (
                tree_node.getTightBounds(bounds_space)
                if bounds_space is not None
                else tree_node.getTightBounds()
            )
            top_z = None
        if bounds is None:
            continue
        minimum_point, maximum_point = bounds
        if top_z is None:
            top_z = maximum_point.z
        canopy_samples.append(
            {
                "min_x": minimum_point.x,
                "max_x": maximum_point.x,
                "min_y": minimum_point.y,
                "max_y": maximum_point.y,
                "top_z": top_z,
            }
        )
    return canopy_samples


def _canopy_cell_coord(value):
    return int(value // _CANOPY_SPATIAL_CELL_METERS)


def _get_canopy_spatial_index(canopy_samples):
    cache_key = id(canopy_samples)
    cached_samples, cached_cells = _canopy_spatial_indexes.get(
        cache_key,
        (None, None),
    )
    if cached_samples is canopy_samples and cached_cells is not None:
        return cached_cells

    cells = {}
    for sample_index, canopy_sample in enumerate(canopy_samples):
        min_cell_x = _canopy_cell_coord(canopy_sample["min_x"])
        max_cell_x = _canopy_cell_coord(canopy_sample["max_x"])
        min_cell_y = _canopy_cell_coord(canopy_sample["min_y"])
        max_cell_y = _canopy_cell_coord(canopy_sample["max_y"])
        for cell_x in range(min_cell_x, max_cell_x + 1):
            for cell_y in range(min_cell_y, max_cell_y + 1):
                cells.setdefault((cell_x, cell_y), []).append(sample_index)

    if len(_canopy_spatial_indexes) > 4:
        _canopy_spatial_indexes.clear()
    _canopy_spatial_indexes[cache_key] = (canopy_samples, cells)
    return cells


def _iter_canopy_query_candidates(canopy_samples, query_x, query_y, query_radius_meters):
    if len(canopy_samples) < 32:
        yield from canopy_samples
        return

    cells = _get_canopy_spatial_index(canopy_samples)
    min_cell_x = _canopy_cell_coord(query_x - query_radius_meters)
    max_cell_x = _canopy_cell_coord(query_x + query_radius_meters)
    min_cell_y = _canopy_cell_coord(query_y - query_radius_meters)
    max_cell_y = _canopy_cell_coord(query_y + query_radius_meters)
    seen_indices = set()
    for cell_x in range(min_cell_x, max_cell_x + 1):
        for cell_y in range(min_cell_y, max_cell_y + 1):
            for sample_index in cells.get((cell_x, cell_y), ()):
                if sample_index in seen_indices:
                    continue
                seen_indices.add(sample_index)
                yield canopy_samples[sample_index]


def compute_local_canopy_height(canopy_samples, query_x, query_y, query_radius_meters):
    max_canopy_z = None
    query_radius_sq = query_radius_meters * query_radius_meters

    for canopy_sample in _iter_canopy_query_candidates(
        canopy_samples,
        query_x,
        query_y,
        query_radius_meters,
    ):
        if query_x < canopy_sample["min_x"]:
            dx = canopy_sample["min_x"] - query_x
        elif query_x > canopy_sample["max_x"]:
            dx = query_x - canopy_sample["max_x"]
        else:
            dx = 0.0

        if query_y < canopy_sample["min_y"]:
            dy = canopy_sample["min_y"] - query_y
        elif query_y > canopy_sample["max_y"]:
            dy = query_y - canopy_sample["max_y"]
        else:
            dy = 0.0

        if (dx * dx + dy * dy) > query_radius_sq:
            continue

        sample_z = canopy_sample["top_z"]
        if max_canopy_z is None or sample_z > max_canopy_z:
            max_canopy_z = sample_z

    return max_canopy_z
