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

    if try_click_results_summary():
        return
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
        export_round_data_files()
        # July 29: once the participant has played every stage, the all-stages
        # summary is refreshed automatically as well.
        summary_path = export_participant_summary_if_complete()
        if results_export_status_text is not None:
            # One click saves the PDF plus the analysis CSVs, all in the
            # participant's own folder (July 9 protocol).
            message = (
                "Saved PDF + CSVs: reports/"
                + get_participant_folder_name()
                + "/"
                + os.path.basename(saved_path).replace("_report.pdf", "_*")
            )
            if summary_path:
                message += "   + all-stages summary"
            results_export_status_text.setText(message)
    except Exception as export_error:
        print(f"[real-sim] export failed: {export_error}")
        if results_export_status_text is not None:
            results_export_status_text.setText("Export failed - see console")
    return True


def try_click_results_summary():
    """Build the all-stages participant summary when its button is clicked.

    Cheryl, July 29: after all five stages there has to be one summarised report
    for the whole session, not just the per-round ones. It can be built at any
    time, with however many rounds exist so far.
    """
    if not results_page_active or results_summary_button is None:
        return False
    if not app.mouseWatcherNode.hasMouse():
        return False
    mouse_x = app.mouseWatcherNode.getMouseX()
    mouse_y = app.mouseWatcherNode.getMouseY()
    point = results_root.getRelativePoint(app.render2d, Point3(mouse_x, 0.0, mouse_y))
    x0, x1, z0, z1 = results_summary_button
    if not (x0 <= point.getX() <= x1 and z0 <= point.getZ() <= z1):
        return False
    try:
        export_round_data_files()
        saved_path = export_participant_summary_pdf()
        if results_export_status_text is not None:
            if saved_path:
                rounds = load_participant_round_summaries()
                results_export_status_text.setText(
                    "All-stages summary (%d rounds, stages %s): %s"
                    % (
                        len(rounds),
                        ",".join(
                            sorted(
                                {
                                    str(entry.get("stage", "")).strip()
                                    for entry in rounds
                                    if entry.get("stage") != ""
                                }
                            )
                        )
                        or "-",
                        os.path.basename(saved_path),
                    )
                )
            else:
                results_export_status_text.setText(
                    "Nothing to summarise yet - play a round first"
                )
    except Exception as summary_error:
        print(f"[real-sim] all-stages summary failed: {summary_error}")
        if results_export_status_text is not None:
            results_export_status_text.setText("Summary failed - see console")
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
# Metric 2: fire detection rate (red = total fire, blue = found).
# Metric 3: fire extinguish rate (red = total fire, green = put out).
# Colors match the map marks (July 28): found is blue, out is green.
# All graphs update live, freeze at game over, and feed the results page.
# ---------------------------------------------------------------
METRICS_AUTO_COLOR = (0.20, 0.95, 0.42, 0.95)    # automatic control
METRICS_MANUAL_COLOR = (1.0, 0.62, 0.18, 0.95)   # manual control
METRICS_TOTAL_FIRE_COLOR = (0.95, 0.22, 0.16, 0.95)
METRICS_DETECTED_COLOR = (0.25, 0.85, 1.0, 0.95)     # blue = fire found
METRICS_EXTINGUISHED_COLOR = (0.40, 0.94, 0.52, 0.95)  # green = fire put out
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
    round_seconds = max(1.0, round_duration_seconds)
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
    # July 29: a round counts as a success when the operator at least matched
    # the automation baseline they were handed, not against a fixed 72%.
    if collaboration_run_succeeded():
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

    if PARTICIPANT_SCORE_HIDDEN:
        # Hide the running score from the participant; keep the team readout so
        # the panel still says who is flying. The score is still recorded.
        performance_score_text.setText(
            f"Survey {performance['team_survey_drones']}"
            f" / Water {performance['team_water_drones']}"
        )
    else:
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


def compute_final_grade():
    """The 100-point score. Since July 29, 2026 this is the COLLABORATION score
    from 04d_collaboration_score.py: a fixed base for the responders that did
    the work (40 survey + ground crews, 67 with water drones) plus the
    operator's own credit and cost, so a hands-off round always scores the same
    number. The old outcome deductions are still computed and reported, but as
    context, not as the score."""
    grade = compute_collaboration_score()
    return dict(grade, outcome_context=compute_outcome_context())


def compute_outcome_context():
    """The old deduction-style outcome view, kept for the report because it says
    what actually happened to the forest. Informational only."""
    total_hotspots = max(1, len(fire_hotspots))
    undetected_count = sum(
        1 for hotspot in fire_hotspots if hotspot.detection_time_seconds is None
    )
    burned_out_count = sum(
        1
        for hotspot in fire_hotspots
        if hotspot.suppression_state == FIRE_STATE_BURNED
    )
    repeat_ignition_count = sum(
        max(0, ignition_count - 1)
        for ignition_count in fire_cell_ignition_counts.values()
    )
    return {
        "area burned": 35.0 * burn_ratio(),
        "fully burned spots": 20.0 * clamp(burned_out_count / total_hotspots, 0.0, 1.0),
        "repeat ignitions": 15.0 * clamp(repeat_ignition_count / total_hotspots, 0.0, 1.0),
        "fires not found": 20.0 * clamp(undetected_count / total_hotspots, 0.0, 1.0),
        "drone losses": 10.0 * clamp(
            len(drone_damage_events) / max(1, team_total_drone_count),
            0.0,
            1.0,
        ),
    }


def compose_final_grade_lines():
    grade = compute_final_grade()
    lines = [f"FINAL SCORE: {grade['score']:.0f}/100"]
    lines.append(f"BASE {grade['base']:.0f}  ({grade['base_label']})")
    applied = [
        f"{label} {points:+.1f}"
        for label, points in grade["credits"].items()
        if points >= 0.05
    ] + [
        f"{label} -{points:.1f}"
        for label, points in grade["penalties"].items()
        if points >= 0.05
    ]
    if not applied:
        lines.append("OPERATOR: nothing added or lost (hands-off baseline)")
    else:
        lines.append("OPERATOR: " + ", ".join(applied[:2]))
        if len(applied) > 2:
            lines.append("  " + ", ".join(applied[2:]))
        lines.append(f"  operator total {grade['operator_delta']:+.1f}")
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


def _draw_results_final_map(map_left, map_bottom, width, height):
    """Post-round map with per-cell ignition and completed-burn counts."""
    background = CardMaker("results_final_map_background")
    background.setFrame(0.0, width, 0.0, height)
    background_np = results_root.attachNewNode(background.generate())
    background_np.setPos(map_left, 0.0, map_bottom)
    background_np.setColor(0.07, 0.18, 0.12, 0.96)
    background_np.setTransparency(TransparencyAttrib.MAlpha)
    background_np.setBin("fixed", 211)
    results_dynamic_nodes.append(background_np)

    grid = LineSegs("results_final_map_grid")
    grid.setThickness(1.0)
    grid.setColor(0.76, 0.86, 0.78, 0.45)
    for division in range(FIRE_BURN_GRID_WIDTH + 1):
        x = map_left + width * (division / max(1, FIRE_BURN_GRID_WIDTH))
        grid.moveTo(x, 0.0, map_bottom)
        grid.drawTo(x, 0.0, map_bottom + height)
    for division in range(FIRE_BURN_GRID_HEIGHT + 1):
        z = map_bottom + height * (division / max(1, FIRE_BURN_GRID_HEIGHT))
        grid.moveTo(map_left, 0.0, z)
        grid.drawTo(map_left + width, 0.0, z)
    grid_np = results_root.attachNewNode(grid.create())
    grid_np.setBin("fixed", 212)
    results_dynamic_nodes.append(grid_np)

    occupied_cells = sorted(
        set(fire_cell_ignition_counts).union(fire_cell_burn_counts)
    )
    cell_width = width / max(1, FIRE_BURN_GRID_WIDTH)
    cell_height = height / max(1, FIRE_BURN_GRID_HEIGHT)
    icon_half = min(cell_width, cell_height) * 0.22
    for cell_x, cell_y in occupied_cells:
        center_x = map_left + cell_width * (cell_x + 0.5)
        center_z = map_bottom + cell_height * (cell_y + 0.5)
        ignition_count = fire_cell_ignition_counts.get((cell_x, cell_y), 0)
        burn_count = fire_cell_burn_counts.get((cell_x, cell_y), 0)
        if ignition_count:
            flame_card = CardMaker(f"results_fire_count_{cell_x}_{cell_y}")
            flame_card.setFrame(-icon_half, icon_half, -icon_half, icon_half)
            flame_np = results_root.attachNewNode(flame_card.generate())
            flame_np.setPos(center_x - icon_half * 0.55, 0.0, center_z + icon_half * 0.35)
            flame_np.setR(45.0)
            flame_np.setColor(1.0, 0.45, 0.08, 0.96)
            flame_np.setTransparency(TransparencyAttrib.MAlpha)
            flame_np.setBin("fixed", 213)
            results_dynamic_nodes.append(flame_np)
            _results_text(
                str(ignition_count),
                (center_x - icon_half * 0.83, center_z + icon_half * 0.08),
                scale=max(0.010, icon_half * 0.82),
                color=(1.0, 1.0, 0.9, 1.0),
            )
        if burn_count:
            cross = LineSegs(f"results_burnout_count_{cell_x}_{cell_y}")
            cross.setThickness(2.4)
            cross.setColor(0.95, 0.12, 0.08, 1.0)
            cross_x = center_x + icon_half * 0.55
            cross_z = center_z - icon_half * 0.28
            cross.moveTo(cross_x - icon_half, 0.0, cross_z - icon_half)
            cross.drawTo(cross_x + icon_half, 0.0, cross_z + icon_half)
            cross.moveTo(cross_x - icon_half, 0.0, cross_z + icon_half)
            cross.drawTo(cross_x + icon_half, 0.0, cross_z - icon_half)
            cross_np = results_root.attachNewNode(cross.create())
            cross_np.setBin("fixed", 213)
            results_dynamic_nodes.append(cross_np)
            _results_text(
                str(burn_count),
                (cross_x - icon_half * 0.28, cross_z - icon_half * 0.35),
                scale=max(0.010, icon_half * 0.72),
                color=(1.0, 0.96, 0.94, 1.0),
            )


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
results_summary_button = None       # all-stages summary hit-box (July 29)
results_export_status_text = None   # updated when the user clicks export


def assemble_pdf_bytes(pages):
    """Turn a list of page content streams into a valid single-font PDF.

    Shared by the per-round report and the all-stages participant summary, and
    dependency-free on purpose: a field laptop only needs the sim's Python.
    """
    page_count = max(1, len(pages))
    font_object_id = 3 + page_count * 2
    page_object_ids = [3 + page_index * 2 for page_index in range(page_count)]
    objects = [
        b"<< /Type /Catalog /Pages 2 0 R >>",
        (
            "<< /Type /Pages /Kids [%s] /Count %d >>"
            % (" ".join("%d 0 R" % object_id for object_id in page_object_ids), page_count)
        ).encode(),
    ]
    for page_index in range(page_count):
        stream = pages[page_index] if page_index < len(pages) else b""
        content_object_id = page_object_ids[page_index] + 1
        objects.append(
            (
                "<< /Type /Page /Parent 2 0 R /MediaBox [0 0 612 792] "
                "/Resources << /Font << /F1 %d 0 R >> >> /Contents %d 0 R >>"
                % (font_object_id, content_object_id)
            ).encode()
        )
        objects.append(
            b"<< /Length " + str(len(stream)).encode() + b" >>\nstream\n" + stream + b"\nendstream"
        )
    objects.append(b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>")
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
    return pdf


def export_round_report_pdf():
    """Write a complete, multipage end-of-round PDF next to the sim.

    The report intentionally stays dependency-free: field laptops only need the
    simulation's Python runtime.  Keep every result section here instead of
    silently clipping page-one content when the round produces many rows.
    """
    import datetime
    import textwrap

    try:
        base_dir = os.path.dirname(os.path.abspath(__file__))
    except Exception:
        base_dir = os.getcwd()
    # July 9 protocol: every participant gets their own folder and every file
    # is named with their ID + the stage that was played.
    out_dir = os.path.join(base_dir, "reports", get_participant_folder_name())

    pages = []
    content = []
    page_number = 0

    def esc(text_value):
        return (
            str(text_value)
            .replace("\r", " ")
            .replace("\n", " ")
            .replace("\\", "\\\\")
            .replace("(", "\\(")
            .replace(")", "\\)")
        )

    def text(x, y, string, size=10, color=(0.1, 0.1, 0.12)):
        content.append(
            "BT /F1 %.1f Tf %.3f %.3f %.3f rg %.1f %.1f Td (%s) Tj ET"
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

    def finish_page():
        if not content:
            return
        pages.append("\n".join(content).encode("latin-1", "replace"))
        content.clear()

    def start_page(title, subtitle=""):
        nonlocal page_number
        finish_page()
        page_number += 1
        text(55, 755, title, 15, head_c)
        if subtitle:
            text(55, 738, subtitle, 9, (0.38, 0.4, 0.46))
        text(
            55,
            20,
            "FSC Fire-Drone Trust Sim | Page %d" % page_number,
            8,
            (0.45, 0.47, 0.52),
        )

    def section_lines(title, lines, cursor_y, wrap_width=118, size=7.5,
                      row_height=9.2):
        """Write every line, opening continuation pages instead of truncating.

        Cheryl, July 29: keep all the information but make the report as short
        as possible, so this is deliberately dense (7.5 pt, 9.2 pt leading,
        118 characters per line).
        """
        if cursor_y < 56:
            start_page("FSC Fire-Drone Trust Sim - Detailed Results (continued)")
            cursor_y = 726
        text(55, cursor_y, title, 9.5, head_c)
        cursor_y -= 12
        for source_line in lines:
            wrapped = textwrap.wrap(
                str(source_line),
                width=wrap_width,
                subsequent_indent="    ",
                replace_whitespace=False,
                drop_whitespace=True,
            ) or [""]
            for line in wrapped:
                if cursor_y < 34:
                    start_page("FSC Fire-Drone Trust Sim - Detailed Results (continued)")
                    cursor_y = 726
                text(60, cursor_y, line, size, (0.15, 0.15, 0.2))
                cursor_y -= row_height
        return cursor_y - 6

    def draw_table(columns, rows, cursor_y, title=None, size=7.5,
                   row_height=9.6, left=55, right=560,
                   page_title="FSC Fire-Drone Trust Sim - Data Tables (continued)"):
        """Plain text table: header, one hairline, no fills. Cheryl's table
        style is no background colour and as clean as possible."""
        column_count = max(1, len(columns))
        step = (right - left) / float(column_count)
        column_x = [left + step * index for index in range(column_count)]

        def draw_header(y):
            for x, label in zip(column_x, columns):
                text(x, y, label, size, head_c)
            y -= 5
            polyline(((left - 2, y), (right, y)), (0.62, 0.64, 0.68), 0.6)
            return y - row_height

        if title:
            if cursor_y < 100:
                start_page(page_title)
                cursor_y = 718
            text(left, cursor_y, title, 11, head_c)
            cursor_y -= 17
        cursor_y = draw_header(cursor_y)
        for row in rows:
            if cursor_y < 48:
                start_page(page_title)
                cursor_y = draw_header(718)
            for x, cell in zip(column_x, row):
                text(x, cursor_y, str(cell), size)
            cursor_y -= row_height
        if not rows:
            text(left, cursor_y, "No runs recorded yet.", size, (0.45, 0.47, 0.52))
            cursor_y -= row_height
        return cursor_y - 10

    def damage_reason_label(damage_reason):
        return {
            DAMAGE_REASON_KILL_SWITCH: "kill switch",
            DAMAGE_REASON_BRANCH_IMPACT: "branch impact",
            DAMAGE_REASON_GROUND_IMPACT: "ground impact",
        }.get(damage_reason, "collision")

    auto_c, manual_c, offline_c = (0.20, 0.78, 0.40), (1.0, 0.6, 0.18), (0.5, 0.5, 0.55)
    red_c, yellow_c, blue_c = (0.9, 0.22, 0.16), (0.95, 0.8, 0.2), (0.3, 0.62, 1.0)
    # Fire timeline colors follow the map marks: blue = found, green = put out.
    green_c = (0.16, 0.68, 0.32)
    head_c = (0.12, 0.18, 0.32)

    reason = {
        "burned_out": "Map fully burned",
        "kill_switch": "Kill switch activated (operator ended the round)",
    }.get(game_end_reason, "Round time limit reached")
    generated_at = datetime.datetime.now()
    performance = compute_performance_matrix(motion_state.sim_time_seconds)
    grade = compute_final_grade()
    total_hotspots = len(fire_hotspots)
    state_counts = {
        state: sum(1 for hotspot in fire_hotspots if hotspot.suppression_state == state)
        for state in (FIRE_STATE_ACTIVE, FIRE_STATE_CONTAINED, FIRE_STATE_OUT, FIRE_STATE_BURNED)
    }
    detected_count = sum(1 for hotspot in fire_hotspots if hotspot.detection_time_seconds is not None)
    undetected_count = total_hotspots - detected_count
    repeat_ignition_count = sum(
        max(0, ignition_count - 1)
        for ignition_count in fire_cell_ignition_counts.values()
    )
    extinguish_credit = get_extinguish_credit_summary()

    start_page("FSC Fire-Drone Trust Sim - End of Round Report")
    text(
        55, 736,
        "Outcome: %s   |   Length: %s   |   Burned: %.0f%% of map"
        % (reason, format_round_clock(motion_state.sim_time_seconds), burn_ratio() * 100.0),
        10,
    )
    text(
        55, 720,
        "Generated " + generated_at.strftime("%Y-%m-%d %H:%M:%S")
        + "   |   Participant: " + get_participant_folder_name()
        + "   |   " + get_stage_report_label(),
        9, (0.4, 0.4, 0.45),
    )

    # ONE score (Cheryl, Aug 19, 2026): the score IS the P1-P4 mission outcome,
    # adjusted for the stage's difficulty (calibration), plus manual actions.
    # P1-P4 are shown below as the PARTS of this one number, never a 2nd score.
    score_rows = [
        (
            "+ " + label,
            points,
            auto_c,
            "each fire you found flying by hand: +%.0f"
            % COLLABORATION_MANUAL_DETECTION_POINTS
            if "found" in label
            else "each fire's worth of manual [J] spray: +%.0f"
            % COLLABORATION_MANUAL_SUPPRESSION_POINTS_PER_FIRE,
        )
        for label, points in grade["credits"].items()
    ] + [
        (
            "- " + label,
            points,
            red_c,
            "each drone lost: -%.0f" % COLLABORATION_DRONE_LOSS_PENALTY_POINTS
            if "drones lost" in label
            else "each fire lost while you held its drone: -%.0f"
            % COLLABORATION_MISSED_FIRE_PENALTY_POINTS,
        )
        for label, points in grade["penalties"].items()
    ]
    action_rows = [row for row in score_rows if abs(row[1]) >= 0.05]
    run_succeeded = grade["base"] >= grade["calibration_anchor"] - 0.05

    text(55, 694, "FINAL SCORE: %.0f / 100" % grade["score"], 15, head_c)
    text(
        368, 695,
        "ABOVE AUTOMATION" if run_succeeded else "BELOW AUTOMATION",
        10, auto_c if run_succeeded else red_c,
    )
    text(
        55, 678,
        "how well the fires were handled (P1-P4 below), adjusted so full automation",
        8, (0.42, 0.44, 0.5),
    )
    text(
        55, 668,
        "always scores %.0f on every stage. One number. Above %.0f = you beat automation."
        % (grade["calibration_anchor"], grade["calibration_anchor"]),
        8, (0.42, 0.44, 0.5),
    )

    text(55, 650, "THE PARTS: P1-P4 MISSION OUTCOME", 10, head_c)
    composite_y = 634
    # Every metric carries its plain-language meaning (Cheryl, July 29: the
    # report has to explain itself, not just print bars).
    metric_rows = (
        (
            "P1 detection speed (30%)",
            performance["p1_score"],
            "how quickly each fire was found after it started",
        ),
        (
            "P2 mapping coverage (20%)",
            performance["p2_score"],
            "how much of each fire a survey drone stayed over long enough to map",
        ),
        (
            "P3 suppression progress (20%)",
            performance["p3_score"],
            "how far the fires you did find got towards being out",
        ),
        (
            "P4 fire control/out (30%)",
            performance["p4_score"],
            "how much of the whole incident, found or not, ended up controlled",
        ),
    )
    for label, value, explanation in metric_rows:
        rect(370, composite_y - 2, 105, 8, (0.9, 0.91, 0.93))
        rect(370, composite_y - 2, 105 * clamp(value, 0.0, 1.0), 8, blue_c)
        text(60, composite_y, label, 8)
        text(485, composite_y, "%.0f%%" % (value * 100.0), 8)
        composite_y -= 9
        text(70, composite_y, explanation, 6.5, (0.45, 0.47, 0.52))
        composite_y -= 11

    # Putting the parts into the SINGLE score: the raw P1-P4 average, then the
    # stage-difficulty adjustment (calibration), giving one number. The manual
    # actions are NOT added (outcome-only, Aug 20); they are listed below as
    # context only, since they already show up in P1-P4.
    composite_y -= 8
    text(55, composite_y, "PUTTING IT TOGETHER  ->  ONE SCORE", 10, head_c)
    composite_y -= 15
    text(60, composite_y, "raw P1-P4 average this round", 8)
    text(485, composite_y, "%.0f%%" % (performance["performance_score"] * 100.0), 8)
    composite_y -= 12
    text(
        60, composite_y,
        "adjusted for this stage's difficulty (automation here scores %.0f%%)"
        % grade["stage_baseline_score"],
        8,
    )
    composite_y -= 11
    rect(55, composite_y + 4, 430, 0.6, (0.6, 0.62, 0.66))
    text(60, composite_y - 2, "FINAL SCORE", 9, head_c)
    text(485, composite_y - 2, "%.0f" % grade["score"], 9, head_c)
    composite_y -= 14

    # What the operator actually did, recorded but NOT scored (outcome-only).
    text(55, composite_y, "WHAT YOU DID (recorded, not part of the score)", 9, head_c)
    composite_y -= 12
    did_rows = (
        "fires found while flying by hand: %d" % grade["manual_detection_count"],
        "manual [J] spray done: about %.1f fire(s) worth" % grade["manual_suppression_fires"],
        "drones lost: %d | fires never found or burned: %d"
        % (grade["drones_lost"], grade["missed_fire_count"]),
    )
    for row in did_rows:
        text(60, composite_y, row, 7.5, (0.45, 0.47, 0.52))
        composite_y -= 10

    summary_y = composite_y - 16
    text(55, summary_y, "ROUND TOTALS", 10, head_c)
    summary_rows = (
        "Team: %d drones (%d survey / %d water)"
        % (team_total_drone_count, performance["team_survey_drones"], performance["team_water_drones"]),
        "Fires: %d total | %d detected | %d never detected"
        % (total_hotspots, detected_count, undetected_count),
        "Final states: %d active | %d contained | %d out | %d fully burned"
        % (
            state_counts[FIRE_STATE_ACTIVE],
            state_counts[FIRE_STATE_CONTAINED],
            state_counts[FIRE_STATE_OUT],
            state_counts[FIRE_STATE_BURNED],
        ),
        "Put out by: %d fire trucks | %d drones | %d both (avg %.0f%% truck / %.0f%% drone)"
        % (
            extinguish_credit["trucks_only"],
            extinguish_credit["drones_only"],
            extinguish_credit["shared"],
            extinguish_credit["shared_ground_pct"],
            extinguish_credit["shared_water_pct"],
        ),
        "Burned area: %.0f m2 (%.1f%% of map) | Repeat ignitions: %d"
        % (additional_burned_area_square_meters(), burn_ratio() * 100.0, repeat_ignition_count),
        "Drone losses: %d | View switches: %d" % (len(drone_damage_events), view_switch_count),
    )
    for row in summary_rows:
        summary_y -= 11
        text(60, summary_y, row, 8)

    # Trust matrix M1-M4 for this round, on the same page as the score. The
    # round is appended to the participant run log first, so it also shows up as
    # the last row of the run-by-run tables further on.
    record_trust_round()
    trust_rows = compute_trust_metrics()
    trust_history = load_trust_run_history()
    trust_figures = load_trust_run_figures()

    trust_y = summary_y - 22
    text(55, trust_y, "TRUST MATRIX M1-M4 (per drone, behaviour of the operator)", 10, head_c)
    trust_y -= 11
    for meaning in (
        "M1 sigma' = planned (automation) path / flown (actual) path. 1.0 = flew like the automation, below 1.0 = detoured.",
        "M2 obstacle clearance while flying by hand, mean and worst, in metres. Risk tolerance.",
        "M3 share of the round that drone spent in manual. Falling over runs = growing trust in automation.",
        "M4 Hausdorff distance to the planned route, in metres. Small = same route tidied, large = own plan.",
    ):
        text(60, trust_y, meaning, 7, (0.45, 0.47, 0.52))
        trust_y -= 9
    trust_round_columns, trust_round_rows = build_trust_round_table(trust_rows)
    trust_y = draw_table(trust_round_columns, trust_round_rows, trust_y - 4)
    text(
        60,
        trust_y,
        "'-' means not enough data: M2 needs manual flight, sigma' needs on-task flight.",
        7,
        (0.45, 0.47, 0.52),
    )

    # Page 2: all visual summaries that appear on the in-app results screen.
    start_page("FSC Fire-Drone Trust Sim - Timelines and Final Map")
    cursor_y = 710
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
    text(55, cursor_y, "FIRE TIMELINE (red total / blue found / green put out)", 11, head_c)
    graph_x, graph_w, graph_h = 95, 360, 105
    graph_y = cursor_y - 12 - graph_h
    rect_stroke(graph_x, graph_y, graph_w, graph_h, (0.6, 0.62, 0.66))
    samples = list(metrics_time_series)
    if len(samples) >= 2:
        t0 = samples[0][0]
        t1 = max(samples[-1][0], t0 + 0.001)
        peak = max(1, max(sample[1] for sample in samples))
        for series_index, color in ((1, red_c), (2, blue_c), (3, green_c)):
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

    # Time per selected drone and per camera mode - include every recorded row.
    bars_y = graph_y - 38
    text(55, bars_y, "TIME PER VIEW", 11, head_c)
    bars_y -= 17
    total_dwell = sum(view_time_seconds.values())
    for label, dwell_seconds in sorted(view_time_seconds.items(), key=lambda item: -item[1]):
        fraction = dwell_seconds / total_dwell if total_dwell > 0.0 else 0.0
        rect(185, bars_y - 2, 250, 9, (0.9, 0.91, 0.93))
        rect(185, bars_y - 2, 250 * fraction, 9, blue_c)
        text(60, bars_y, label, 8)
        text(445, bars_y, "%.0f%% (%.0fs)" % (fraction * 100.0, dwell_seconds), 8)
        bars_y -= 14

    mode_total = sum(camera_mode_time_seconds.values())
    text(55, bars_y - 4, "TIME PER CAMERA MODE", 10, head_c)
    bars_y -= 20
    for label, dwell_seconds in sorted(camera_mode_time_seconds.items(), key=lambda item: -item[1]):
        fraction = dwell_seconds / mode_total if mode_total > 0.0 else 0.0
        rect(185, bars_y - 2, 250, 9, (0.9, 0.91, 0.93))
        rect(185, bars_y - 2, 250 * fraction, 9, yellow_c)
        text(60, bars_y, label, 8)
        text(445, bars_y, "%.0f%% (%.0fs)" % (fraction * 100.0, dwell_seconds), 8)
        bars_y -= 14

    # Final fire-history map, including every occupied burn-grid cell.
    map_x, map_y, map_w, map_h = 170, 55, 270, max(80, min(150, bars_y - 78))
    text(55, map_y + map_h + 14, "FINAL FIRE HISTORY MAP", 11, head_c)
    rect(map_x, map_y, map_w, map_h, (0.07, 0.18, 0.12))
    cell_w = map_w / max(1, FIRE_BURN_GRID_WIDTH)
    cell_h = map_h / max(1, FIRE_BURN_GRID_HEIGHT)
    for division in range(FIRE_BURN_GRID_WIDTH + 1):
        x = map_x + cell_w * division
        polyline(((x, map_y), (x, map_y + map_h)), (0.55, 0.65, 0.58), 0.35)
    for division in range(FIRE_BURN_GRID_HEIGHT + 1):
        y = map_y + cell_h * division
        polyline(((map_x, y), (map_x + map_w, y)), (0.55, 0.65, 0.58), 0.35)
    occupied_cells = sorted(set(fire_cell_ignition_counts).union(fire_cell_burn_counts))
    for cell_x, cell_y in occupied_cells:
        x = map_x + cell_w * (cell_x + 0.5)
        y = map_y + cell_h * (cell_y + 0.5)
        ignition_count = fire_cell_ignition_counts.get((cell_x, cell_y), 0)
        burn_count = fire_cell_burn_counts.get((cell_x, cell_y), 0)
        if ignition_count:
            rect(x - 3.5, y - 3.5, 7, 7, (1.0, 0.45, 0.08))
            text(x + 4, y - 2, str(ignition_count), 7, (0.95, 0.75, 0.35))
        if burn_count:
            polyline(((x - 4, y - 4), (x + 4, y + 4)), red_c, 1.2)
            polyline(((x - 4, y + 4), (x + 4, y - 4)), red_c, 1.2)
            text(x + 4, y - 9, str(burn_count), 7, red_c)
    text(450, map_y + map_h - 5, "orange = ignitions", 8, (0.7, 0.45, 0.1))
    text(450, map_y + map_h - 18, "red X = burned/out", 8, red_c)

    # Trust matrix M1-M4 (Cheryl, July 29): the per-drone table for this round
    # sits on page 1 with the score, the run-by-run tables and the figure follow.
    start_page(
        "FSC Fire-Drone Trust Sim - Trust Matrix M1-M4 Run by Run",
        "dS / dW / dTotal = change from the previous run in the survey mean, "
        "the water mean and the team mean",
    )
    cursor_y = 726
    for metric_key, metric_label, _metric_format, _higher_is_better in (
        TRUST_METRIC_DEFINITIONS
    ):
        run_columns, run_rows = build_trust_run_table(metric_key, trust_history)
        cursor_y = draw_table(
            run_columns,
            run_rows,
            cursor_y,
            title="PER RUN: " + metric_label,
        )

    # M3 figure: every drone on one page, rows = runs, horizontal axis = path
    # length, same layout as Figure 3 in the SMC journal submission. It used to
    # be one page per drone; Cheryl asked for the whole report to be as short as
    # possible, so the drones are stacked and only spill when the page is full.
    cyan_c, magenta_c = (0.30, 0.78, 0.88), (0.85, 0.20, 0.75)
    figure_labels = sorted(
        {
            label
            for figure_entry in trust_figures
            for label in figure_entry.get("drones", {})
        },
        key=trust_drone_sort_key,
    )
    figure_title = "FSC Fire-Drone Trust Sim - M3 Mode Usage Over Runs"
    figure_subtitle = (
        "cyan automatic | orange manual | grey offline | green tick event | "
        "red X drone lost | magenta dash planned length | black distance to obstacle"
    )
    if figure_labels:
        start_page(figure_title, figure_subtitle)
        axis_x, axis_w = 92, 420
        band_h, band_gap = 9, 4
        band_y = 716
        for label in figure_labels:
            entries = [
                figure_entry
                for figure_entry in trust_figures
                if label in figure_entry.get("drones", {})
            ]
            if not entries:
                continue
            max_length = 1.0
            for figure_entry in entries:
                drone_entry = figure_entry["drones"][label]
                max_length = max(
                    max_length,
                    drone_entry.get("flown_m") or 0.0,
                    drone_entry.get("planned_m") or 0.0,
                )
            block_height = 16 + len(entries) * (band_h + band_gap) + 22
            if band_y - block_height < 40:
                start_page(figure_title + " (continued)", figure_subtitle)
                band_y = 716
            text(55, band_y, "%s   (0 to %.0f m flown)" % (label, max_length), 9, head_c)
            band_y -= 14
            for figure_entry in entries:
                drone_entry = figure_entry["drones"][label]
                scale = axis_w / max_length
                rect_stroke(axis_x, band_y, axis_w, band_h, (0.82, 0.84, 0.88), 0.4)
                for segment in drone_entry.get("segments", ()):
                    start_m, end_m, state_name = segment
                    x0 = axis_x + max(0.0, start_m) * scale
                    x1 = axis_x + max(0.0, end_m) * scale
                    if x1 - x0 <= 0.3:
                        continue
                    segment_color = (
                        cyan_c if state_name == "auto"
                        else (manual_c if state_name == "manual" else offline_c)
                    )
                    rect(x0, band_y, min(x1, axis_x + axis_w) - x0, band_h, segment_color)
                polyline(
                    [
                        (
                            axis_x + max(0.0, distance) * scale,
                            band_y
                            + band_h
                            * clamp(
                                clearance / TRUST_OBSTACLE_SEARCH_RADIUS_METERS,
                                0.0,
                                1.0,
                            ),
                        )
                        for distance, clearance in drone_entry.get("clearance", ())
                    ],
                    (0.08, 0.08, 0.1),
                    0.7,
                )
                for event_m in drone_entry.get("events", ()):
                    event_x = axis_x + max(0.0, event_m) * scale
                    polyline(
                        ((event_x, band_y - 1), (event_x, band_y + band_h + 1)),
                        green_c,
                        1.0,
                    )
                for crash_m in drone_entry.get("crashes", ()):
                    crash_x = axis_x + max(0.0, crash_m) * scale
                    mid_y = band_y + band_h * 0.5
                    polyline(
                        ((crash_x - 3.0, mid_y - 3.0), (crash_x + 3.0, mid_y + 3.0)),
                        red_c,
                        1.4,
                    )
                    polyline(
                        ((crash_x - 3.0, mid_y + 3.0), (crash_x + 3.0, mid_y - 3.0)),
                        red_c,
                        1.4,
                    )
                planned_m = drone_entry.get("planned_m") or 0.0
                if planned_m > 0.0:
                    planned_x = axis_x + planned_m * scale
                    dash_y = band_y
                    while dash_y < band_y + band_h:
                        polyline(
                            (
                                (planned_x, dash_y),
                                (planned_x, min(dash_y + 2.0, band_y + band_h)),
                            ),
                            magenta_c,
                            1.2,
                        )
                        dash_y += 4.0
                text(58, band_y + 1, "Run %s" % figure_entry.get("run", "?"), 7)
                text(
                    axis_x + axis_w + 6,
                    band_y + 1,
                    "%.0f m" % (drone_entry.get("flown_m") or 0.0),
                    6.5,
                )
                band_y -= band_h + band_gap
            polyline(
                ((axis_x, band_y + 4), (axis_x + axis_w, band_y + 4)),
                (0.6, 0.62, 0.66),
                0.5,
            )
            text(axis_x, band_y - 4, "0", 6.5, (0.4, 0.4, 0.45))
            text(
                axis_x + axis_w * 0.5 - 34,
                band_y - 4,
                "path length flown (m), black trace = 0 to %.0f m clearance"
                % TRUST_OBSTACLE_SEARCH_RADIUS_METERS,
                6.5,
                (0.4, 0.4, 0.45),
            )
            text(axis_x + axis_w - 26, band_y - 4, "%.0f m" % max_length, 6.5, (0.4, 0.4, 0.45))
            band_y -= 18

    # Formula sheet (Cheryl, July 28): every matrix variable, in full, built
    # from the live constants so it can never go stale.
    start_page(
        "FSC Fire-Drone Trust Sim - How Every Number Is Calculated",
        "Each variable, its current value and the exact formula used this round",
    )
    cursor_y = 726
    for formula_line in compose_metric_formula_lines():
        if cursor_y < 34:
            start_page(
                "FSC Fire-Drone Trust Sim - How Every Number Is Calculated (continued)"
            )
            cursor_y = 726
        stripped = formula_line.strip()
        is_heading = bool(stripped) and stripped[0].isdigit() and "." in stripped[:4]
        text(
            55 if is_heading else 60,
            cursor_y,
            formula_line,
            8.5 if is_heading else 7.5,
            head_c if is_heading else (0.15, 0.15, 0.2),
        )
        cursor_y -= 11 if is_heading else 9.2

    # Everything else, dense: the score arithmetic, the round's context and the
    # complete data tables.
    start_page(
        "FSC Fire-Drone Trust Sim - Detailed Results",
        "Same information as before, packed. Every column is also in the CSVs "
        "next to this file.",
    )
    cursor_y = 718
    cursor_y = section_lines("SCORE BREAKDOWN (one score, built up)", [
        "Mission outcome, the basis: %.0f points (%s)" % (grade["base"], grade["base_label"]),
        "Ground crew work: %.1f work-seconds | water drone work: %.1f "
        "(%.1f of it sprayed manually)"
        % (
            grade["ground_work_seconds"],
            grade["water_work_seconds"],
            grade["manual_water_work_seconds"],
        ),
    ] + [
        "Credit  %s: +%.2f points" % (label, points)
        for label, points in grade["credits"].items()
    ] + [
        "Cost    %s: -%.2f points" % (label, points)
        for label, points in grade["penalties"].items()
    ] + [
        "Operator total: %+.2f points" % grade["operator_delta"],
        "FINAL SCORE: %.1f / 100" % grade["score"],
    ], cursor_y)
    cursor_y = section_lines("OUTCOME CONTEXT (reported, not scored)", [
        "%s: %.2f of %.0f points of damage" % (label, points, weight)
        for (label, points), weight in zip(
            grade["outcome_context"].items(),
            (35.0, 20.0, 15.0, 20.0, 10.0),
        )
    ], cursor_y)
    cursor_y = section_lines(
        "WHO PUT THE FIRES OUT",
        compose_extinguish_credit_lines()[1:],
        cursor_y,
    )
    cursor_y = section_lines("CONTROL USAGE", compose_mode_usage_lines()[1:], cursor_y)
    cursor_y = section_lines("FIRE CLUSTERS", compose_cluster_table_lines()[1:], cursor_y)
    cursor_y = section_lines("VIEW SWITCHING SUMMARY", compose_view_switch_lines()[1:], cursor_y)

    source_lines = [
        "%s: %d" % (source, count)
        for source, count in sorted(view_switch_source_counts.items())
    ] or ["No categorized view-switch sources recorded."]
    cursor_y = section_lines("VIEW SWITCH SOURCES", source_lines, cursor_y)
    # Every fire, every burn cell and every log line still ship in the report,
    # but as dense tables instead of paragraphs (Cheryl, July 29: keep all the
    # information, make it as short as possible).
    fire_rows = []
    for fire_index, hotspot in enumerate(fire_hotspots, start=1):
        detection_delay = (
            max(0.0, hotspot.detection_time_seconds - hotspot.ignition_time_seconds)
            if hotspot.detection_time_seconds is not None
            else None
        )
        fire_rows.append([
            "%d" % fire_index,
            "%d" % hotspot.fire_event_id,
            hotspot.suppression_state,
            "%.0f,%.0f" % (hotspot.root.getX(), hotspot.root.getY()),
            format_round_clock(hotspot.ignition_time_seconds),
            format_round_clock(hotspot.detection_time_seconds)
            if hotspot.detection_time_seconds is not None else "never",
            "%.0f" % detection_delay if detection_delay is not None else "-",
            "S%d" % hotspot.detected_by_survey_slot
            if hotspot.detected_by_survey_slot is not None else "-",
            "%.0f%%" % (hotspot_mapping_fraction(hotspot) * 100.0),
            "%.1f" % hotspot.suppression_work_seconds,
            "%.0f" % hotspot.burn_age_seconds,
            "%d" % hotspot.reignition_count,
            "%s/%d/%d" % (
                "y" if hotspot.water_drone_requested else "n",
                hotspot.water_drone_assignment_count,
                hotspot.water_drone_engagement_count,
            ),
            hotspot.extinguished_by or "-",
            "%.0f%%" % (hotspot.extinguish_ground_share * 100.0)
            if hotspot.extinguished_by else "-",
        ])
    cursor_y = draw_table(
        [
            "Fire", "Event", "State", "x,y", "Ignited", "Found", "Delay s",
            "By", "Mapped", "Work s", "Burn s", "Reig", "Water r/a/e",
            "Out by", "Truck",
        ],
        fire_rows or [["-"] * 15],
        cursor_y,
        title="EVERY FIRE (%d)" % len(fire_hotspots),
        size=6.5,
        row_height=8.4,
    )

    cell_rows = []
    cell_group = []
    for cell_x, cell_y in occupied_cells:
        cell_group.append(
            "(%d,%d) %d/%d"
            % (
                cell_x,
                cell_y,
                fire_cell_ignition_counts.get((cell_x, cell_y), 0),
                fire_cell_burn_counts.get((cell_x, cell_y), 0),
            )
        )
        if len(cell_group) == 6:
            cell_rows.append(cell_group)
            cell_group = []
    if cell_group:
        cell_rows.append(cell_group + [""] * (6 - len(cell_group)))
    cursor_y = draw_table(
        ["cell (x,y) ignitions/burned"] + [""] * 5,
        cell_rows or [["none"] + [""] * 5],
        cursor_y,
        title="BURN GRID CELLS (%d)" % len(occupied_cells),
        size=6.5,
        row_height=8.4,
    )

    log_rows = []
    log_group = []
    for event_time, label in view_switch_log:
        log_group.append("%s %s" % (format_round_clock(event_time), label))
        if len(log_group) == 4:
            log_rows.append(log_group)
            log_group = []
    if log_group:
        log_rows.append(log_group + [""] * (4 - len(log_group)))
    cursor_y = draw_table(
        ["view switch log (time -> view)"] + [""] * 3,
        log_rows or [["none"] + [""] * 3],
        cursor_y,
        title="VIEW SWITCH LOG (%d)" % len(view_switch_log),
        size=6.5,
        row_height=8.4,
    )

    damage_rows = [
        [
            format_round_clock(event_time),
            event_label,
            damage_reason_label(damage_reason),
        ]
        for event_time, event_label, damage_reason in drone_damage_events
    ]
    cursor_y = draw_table(
        ["Time", "Drone", "Cause"],
        damage_rows or [["-", "no drone losses", "-"]],
        cursor_y,
        title="DRONE LOSSES (%d)" % len(drone_damage_events),
        size=6.5,
        row_height=8.4,
    )

    finish_page()

    pdf = assemble_pdf_bytes(pages)

    os.makedirs(out_dir, exist_ok=True)
    file_name = get_export_file_stem(generated_at) + "_report.pdf"
    file_path = os.path.join(out_dir, file_name)
    with open(file_path, "wb") as report_file:
        report_file.write(pdf)
    return file_path


def get_participant_folder_name():
    return sanitize_participant_id(globals().get("participant_id", ""))


def get_stage_report_label():
    stage_number = globals().get("selected_stage", 0)
    stage = EXPERIMENT_STAGES.get(stage_number, {})
    return "Stage %s (%s)" % (stage_number, stage.get("label", "unknown"))


def get_export_file_stem(generated_at):
    return "%s_stage%s_%s" % (
        get_participant_folder_name(),
        globals().get("selected_stage", 0),
        generated_at.strftime("%Y%m%d_%H%M%S"),
    )


def export_round_data_files():
    """Machine-readable exports next to the PDF (July 9 decision): July's
    correlation analysis loads these directly instead of retyping the PDF.

    Four CSVs per round, all named <participant>_stage<N>_<timestamp>_*:
      _summary.csv    one row: scores, totals, per-drone auto %
      _fires.csv      one row per fire (ignition/detection/suppression)
      _events.csv     mode changes, view switches, drone losses over time
      _timeseries.csv fire counts over time (total/found/out)
    """
    import csv
    import datetime

    try:
        base_dir = os.path.dirname(os.path.abspath(__file__))
    except Exception:
        base_dir = os.getcwd()
    out_dir = os.path.join(base_dir, "reports", get_participant_folder_name())
    os.makedirs(out_dir, exist_ok=True)
    generated_at = datetime.datetime.now()
    file_stem = get_export_file_stem(generated_at)

    performance = compute_performance_matrix(motion_state.sim_time_seconds)
    grade = compute_final_grade()
    extinguish_credit = get_extinguish_credit_summary()
    total_hotspots = len(fire_hotspots)
    detected_count = sum(
        1 for hotspot in fire_hotspots if hotspot.detection_time_seconds is not None
    )
    state_counts = {
        state: sum(1 for hotspot in fire_hotspots if hotspot.suppression_state == state)
        for state in (FIRE_STATE_ACTIVE, FIRE_STATE_CONTAINED, FIRE_STATE_OUT, FIRE_STATE_BURNED)
    }

    summary_row = {
        "participant": get_participant_folder_name(),
        "stage": globals().get("selected_stage", 0),
        "stage_label": get_stage_report_label(),
        "generated": generated_at.strftime("%Y-%m-%d %H:%M:%S"),
        "round_seconds": round(motion_state.sim_time_seconds, 2),
        "outcome": game_end_reason or "in_progress",
        "burned_pct": round(burn_ratio() * 100.0, 2),
        "score": round(grade["score"], 2),
        "score_base": round(grade["base"], 1),
        "score_base_label": grade["base_label"],
        "water_drones_involved": int(bool(grade["water_involved"])),
        "operator_delta": round(grade["operator_delta"], 2),
        "operator_credit": round(grade["credit_total"], 2),
        "operator_cost": round(grade["penalty_total"], 2),
        "manual_detections": grade["manual_detection_count"],
        "manual_suppression_fire_equivalents": round(grade["manual_suppression_fires"], 3),
        "ground_work_seconds": round(grade["ground_work_seconds"], 2),
        "water_work_seconds": round(grade["water_work_seconds"], 2),
        "manual_water_work_seconds": round(grade["manual_water_work_seconds"], 2),
        "grade_score": round(grade["score"], 2),
        "composite_score_pct": round(performance["performance_score"] * 100.0, 2),
        "raw_composite_pct": round(performance["raw_performance_score"] * 100.0, 2),
        "p1_detection_pct": round(performance["p1_score"] * 100.0, 2),
        "p2_coverage_pct": round(performance["p2_score"] * 100.0, 2),
        "p3_suppression_pct": round(performance["p3_score"] * 100.0, 2),
        "p4_fire_out_pct": round(performance["p4_score"] * 100.0, 2),
        "team_total_drones": team_total_drone_count,
        "team_survey_drones": performance["team_survey_drones"],
        "team_water_drones": performance["team_water_drones"],
        "fires_total": total_hotspots,
        "fires_detected": detected_count,
        "fires_never_detected": total_hotspots - detected_count,
        "fires_active_end": state_counts[FIRE_STATE_ACTIVE],
        "fires_contained_end": state_counts[FIRE_STATE_CONTAINED],
        "fires_out_end": state_counts[FIRE_STATE_OUT],
        "fires_burned_end": state_counts[FIRE_STATE_BURNED],
        "drone_losses": len(drone_damage_events),
        "view_switches": view_switch_count,
        "fires_out_by_trucks_only": extinguish_credit["trucks_only"],
        "fires_out_by_drones_only": extinguish_credit["drones_only"],
        "fires_out_by_both": extinguish_credit["shared"],
        "shared_avg_truck_pct": round(extinguish_credit["shared_ground_pct"], 1),
        "shared_avg_drone_pct": round(extinguish_credit["shared_water_pct"], 1),
    }
    for credit_label, credit_points in grade["credits"].items():
        summary_row["credit_" + credit_label.replace(" ", "_")] = round(credit_points, 2)
    for penalty_label, penalty_points in grade["penalties"].items():
        summary_row["cost_" + penalty_label.replace(" ", "_")] = round(penalty_points, 2)
    for context_label, context_points in grade["outcome_context"].items():
        summary_row["outcome_" + context_label.replace(" ", "_")] = round(
            context_points, 2
        )
    for drone_label in sorted(drone_mode_time_seconds.keys()):
        manual_seconds, auto_seconds = drone_mode_time_seconds[drone_label]
        mode_total_seconds = manual_seconds + auto_seconds
        summary_row["auto_pct_" + drone_label.replace(" ", "_")] = round(
            (auto_seconds / mode_total_seconds * 100.0) if mode_total_seconds else 0.0,
            2,
        )
    summary_path = os.path.join(out_dir, file_stem + "_summary.csv")
    with open(summary_path, "w", newline="") as summary_file:
        writer = csv.DictWriter(summary_file, fieldnames=list(summary_row.keys()))
        writer.writeheader()
        writer.writerow(summary_row)

    fires_path = os.path.join(out_dir, file_stem + "_fires.csv")
    with open(fires_path, "w", newline="") as fires_file:
        writer = csv.writer(fires_file)
        writer.writerow((
            "fire_index", "event_id", "final_state", "x", "y",
            "ignition_s", "detected_s", "detection_delay_s", "detector_survey_slot",
            "mapped_pct", "suppression_work_s", "burn_age_s", "reignitions",
            "water_requested", "water_assignments", "water_engagements",
            "extinguished_by", "truck_work_s", "drone_work_s", "truck_share_pct",
        ))
        for fire_index, hotspot in enumerate(fire_hotspots, start=1):
            detection_delay = (
                max(0.0, hotspot.detection_time_seconds - hotspot.ignition_time_seconds)
                if hotspot.detection_time_seconds is not None
                else ""
            )
            writer.writerow((
                fire_index,
                hotspot.fire_event_id,
                hotspot.suppression_state,
                round(hotspot.root.getX(), 2),
                round(hotspot.root.getY(), 2),
                round(hotspot.ignition_time_seconds, 2),
                round(hotspot.detection_time_seconds, 2)
                if hotspot.detection_time_seconds is not None else "",
                round(detection_delay, 2) if detection_delay != "" else "",
                hotspot.detected_by_survey_slot
                if hotspot.detected_by_survey_slot is not None else "",
                round(hotspot_mapping_fraction(hotspot) * 100.0, 1),
                round(hotspot.suppression_work_seconds, 2),
                round(hotspot.burn_age_seconds, 2),
                hotspot.reignition_count,
                int(bool(hotspot.water_drone_requested)),
                hotspot.water_drone_assignment_count,
                hotspot.water_drone_engagement_count,
                hotspot.extinguished_by or "",
                round(hotspot.ground_suppression_work_seconds, 2),
                round(hotspot.water_suppression_work_seconds, 2),
                round(hotspot.extinguish_ground_share * 100.0, 1)
                if hotspot.extinguished_by else "",
            ))

    events_path = os.path.join(out_dir, file_stem + "_events.csv")
    with open(events_path, "w", newline="") as events_file:
        writer = csv.writer(events_file)
        writer.writerow(("time_s", "event_type", "subject", "detail"))
        event_rows = []
        for drone_label, timeline in drone_mode_timeline.items():
            for segment_start, mode_state in timeline:
                mode_name = (
                    "auto" if mode_state is True
                    else ("manual" if mode_state is False else "offline")
                )
                event_rows.append((segment_start, "mode_change", drone_label, mode_name))
        for event_time, view_label in view_switch_log:
            event_rows.append((event_time, "view_switch", view_label, ""))
        for event_time, event_label, damage_reason in drone_damage_events:
            event_rows.append((event_time, "drone_loss", event_label, damage_reason))
        for event_time, event_type, subject, detail in sorted(event_rows):
            writer.writerow((round(event_time, 2), event_type, subject, detail))

    # Trust matrix, one row per drone for this round (July 29). The cumulative
    # per-participant version is <participant>_trust_runs.csv, appended by
    # record_trust_round below.
    record_trust_round()
    trust_rows = compute_trust_metrics()
    trust_path = os.path.join(out_dir, file_stem + "_trust.csv")
    with open(trust_path, "w", newline="") as trust_file:
        writer = csv.writer(trust_file)
        writer.writerow((
            "drone", "role", "slot", "flown_m", "flown_manual_m", "flown_auto_m",
            "planned_m", "m1_sigma", "m2_clearance_mean_m", "m2_clearance_min_m",
            "m2_samples", "m3_manual_pct", "m3_auto_pct", "m4_hausdorff_m",
            "events", "crashed",
        ))

        def trust_cell(value, digits=2):
            return "" if value is None else round(value, digits)

        for drone_label in sorted(trust_rows.keys(), key=trust_drone_sort_key):
            trust_row = trust_rows[drone_label]
            writer.writerow((
                drone_label,
                trust_row["role"],
                trust_row["slot"],
                trust_cell(trust_row["flown_m"], 1),
                trust_cell(trust_row["flown_manual_m"], 1),
                trust_cell(trust_row["flown_auto_m"], 1),
                trust_cell(trust_row["planned_m"], 1),
                trust_cell(trust_row["m1_sigma"], 3),
                trust_cell(trust_row["m2_clearance_mean_m"], 2),
                trust_cell(trust_row["m2_clearance_min_m"], 2),
                trust_row["m2_samples"],
                trust_cell(trust_row["m3_manual_pct"], 1),
                trust_cell(trust_row["m3_auto_pct"], 1),
                trust_cell(trust_row["m4_hausdorff_m"], 2),
                trust_row["events"],
                trust_row["crashed"],
            ))

    timeseries_path = os.path.join(out_dir, file_stem + "_timeseries.csv")
    with open(timeseries_path, "w", newline="") as timeseries_file:
        writer = csv.writer(timeseries_file)
        writer.writerow(("time_s", "fires_total", "fires_found", "fires_out"))
        for sample in metrics_time_series:
            writer.writerow((round(sample[0], 2),) + tuple(sample[1:4]))

    return out_dir


def show_results_page():
    global results_page_active, results_root, results_pdf_path
    global results_export_button, results_export_status_text
    global results_summary_button

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

    reason_label = {
        "burned_out": "MAP FULLY BURNED",
        "kill_switch": "KILL SWITCH ACTIVATED",
    }.get(game_end_reason, "ROUND TIME LIMIT REACHED")
    grade = compute_final_grade()
    _results_text(
        f"END OF ROUND REPORT  -  {reason_label}",
        (-1.18, 0.72),
        scale=0.052,
        color=(0.98, 0.86, 0.42, 1.0),
    )
    if PARTICIPANT_SCORE_HIDDEN:
        # The score is a research measure; the participant does not see it on
        # screen. It is still computed and written to the exported PDF and CSV.
        _results_text(
            "Round complete - your results have been recorded.",
            (-1.18, 0.64),
            scale=0.036,
            color=(0.72, 0.9, 1.0, 1.0),
        )
    else:
        _results_text(
            f"FINAL SCORE: {grade['score']:.0f}/100"
            f"   (base {grade['base']:.0f} {grade['operator_delta']:+.1f} operator)",
            (-1.18, 0.64),
            scale=0.040,
            color=(0.58, 1.0, 0.68, 1.0),
        )
        _results_text(
            f"Collaboration: {grade['base_label']}",
            (-1.18, 0.585),
            scale=0.026,
            color=(0.72, 0.9, 1.0, 1.0),
        )
    _results_text(
        f"Round length: {format_round_clock(motion_state.sim_time_seconds)}"
        f" | Burned: {burn_ratio() * 100.0:.0f}% of map",
        (-1.18, 0.54),
        scale=0.030,
    )

    # The score/base/operator breakdown (first lines of compose_final_grade_lines)
    # is part of the grade, so it is only shown on screen when the score is not
    # hidden. The behavioural sections (mode usage, fire clusters, view switches)
    # are always shown.
    if PARTICIPANT_SCORE_HIDDEN:
        left_lines = []
    else:
        left_lines = compose_final_grade_lines()[1:] + [""]
    left_lines.extend(compose_mode_usage_lines())
    left_lines.append("")
    left_lines.extend(compose_extinguish_credit_lines())
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
    _results_text("\n".join(left_lines), (-1.18, 0.47), scale=0.030)

    _results_text(
        "CONTROL MODE TIMELINE (green auto / orange manual / grey offline)",
        (0.16, 0.48),
        scale=0.027,
    )
    _draw_results_usage_bars(0.16, 0.445, 0.62)

    _results_text(
        "FIRE TIMELINE (red = total, blue = found, green = put out)",
        (0.16, 0.16),
        scale=0.027,
    )
    _draw_results_time_series_graph(0.16, -0.20, 0.98, 0.33)
    _results_text("TIME PER VIEW", (0.16, -0.27), scale=0.027)
    _draw_results_view_dwell_bars(0.16, -0.31, 0.25)

    # Trust matrix M1-M4 (July 29): the compact per-drone view. The full
    # per-round and per-run tables plus the mode-usage figure are in the
    # exported PDF and in <participant>_trust_runs.csv.
    trust_run_number = record_trust_round()
    _results_text(
        "TRUST MATRIX M1-M4 (per drone, this round)",
        (0.16, -0.58),
        scale=0.025,
    )
    _results_text(
        "\n".join(compose_trust_matrix_screen_lines()),
        (0.16, -0.62),
        scale=0.019,
        color=(0.82, 0.94, 1.0, 1.0),
    )
    _results_text(
        "run %s in this participant's trust run log"
        % (trust_run_number if trust_run_number is not None else "-"),
        (0.16, -0.88),
        scale=0.021,
        color=(0.6, 0.8, 0.95, 1.0),
    )

    _results_text("FINAL FIRE HISTORY MAP", (0.58, -0.27), scale=0.027)
    _results_text(
        "orange diamond = times on fire   red X = times burned/out",
        (0.58, -0.31),
        scale=0.019,
        color=(0.82, 0.9, 0.86, 1.0),
    )
    _draw_results_final_map(0.58, -0.70, 0.54, 0.35)

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

    summary_x0, summary_x1, summary_z0, summary_z1 = 0.02, 0.54, -0.86, -0.78
    results_summary_button = (summary_x0, summary_x1, summary_z0, summary_z1)
    summary_card = CardMaker("results_summary_button")
    summary_card.setFrame(summary_x0, summary_x1, summary_z0, summary_z1)
    summary_card_np = results_root.attachNewNode(summary_card.generate())
    summary_card_np.setColor(0.14, 0.34, 0.30, 0.96)
    summary_card_np.setTransparency(TransparencyAttrib.MAlpha)
    summary_card_np.setBin("fixed", 212)
    results_dynamic_nodes.append(summary_card_np)
    summary_label = OnscreenText(
        text="EXPORT ALL-STAGES SUMMARY",
        parent=results_root,
        pos=((summary_x0 + summary_x1) * 0.5, summary_z0 + 0.026),
        scale=0.03,
        fg=(1.0, 1.0, 1.0, 1.0),
        shadow=(0.0, 0.0, 0.0, 0.7),
        align=TextNode.ACenter,
        mayChange=False,
    )
    summary_label.setBin("fixed", 213)
    results_dynamic_nodes.append(summary_label)

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
    if not water_hotspot_is_targetable(target_hotspot):
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
    # Keep all marker geometry inside the framed map. In particular, an OUT
    # cross at the southern edge must not spill onto the legend/caption.
    # Kept as small as possible: a bigger margin visibly shifted edge marks
    # away from their true world positions (alignment bug, July 6).
    marker_margin_fraction = 0.008
    x_fraction = clamp(x_fraction, marker_margin_fraction, 1.0 - marker_margin_fraction)
    y_fraction = clamp(y_fraction, marker_margin_fraction, 1.0 - marker_margin_fraction)
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


def build_water_refill_station_map_marker():
    station_x, station_y = get_water_refill_station_xy()
    map_x, map_y = world_to_fire_map_coords(station_x, station_y)
    station_root = fire_map_marker_root.attachNewNode("water_refill_station")
    station_root.setPos(map_x, 0.0, map_y)
    station_root.setBin("fixed", 117)

    ring = build_fire_map_circle(
        WATER_REFILL_RADIUS_METERS,
        (0.2, 0.9, 1.0, 0.96),
        parent=station_root,
        thickness=2.5,
    )
    ring.setBin("fixed", 117)
    station_marker = make_fire_map_square(
        "water_refill_station_marker",
        0.012,
        (0.12, 0.62, 1.0, 1.0),
        parent=station_root,
    )
    station_marker.setR(45.0)
    station_marker.setBin("fixed", 118)

    label = OnscreenText(
        text=(
            "WATER REFILL\n"
            f"Hover W inside ring {WATER_REFILL_HOLD_SECONDS:.0f}s"
        ),
        parent=fire_map_panel,
        pos=(map_x - 0.020, map_y - 0.026),
        scale=0.021,
        fg=(0.56, 0.94, 1.0, 1.0),
        shadow=(0.0, 0.0, 0.0, 0.8),
        align=TextNode.ARight,
        mayChange=True,
    )
    label.setBin("fixed", 124)
    label.setDepthWrite(False)
    label.setDepthTest(False)


def build_fire_map_legend():
    legend_y = FIRE_MAP_VIEW_BOTTOM - 0.034
    # One scheme everywhere (July 6, tightened July 28): orange = undetected
    # fire (world only, never on this map), BLUE = detected fire, PURPLE = fire
    # the trucks are working, green X = out, dark red X = burned. The X is used
    # for nothing else on either map.
    legend_items = (
        ((0.25, 0.85, 1.0, 1.0), "detected fire", False),
        (FIRE_MARK_OUT_COLOR, "out", True),
        (FIRE_MARK_BURNED_COLOR, "burned", True),
    )
    box_half = 0.013

    # Text scales here look oversized in panel units on purpose: the whole
    # panel is drawn at FIRE_MAP_COLLAPSED_SCALE (0.76), so captions must be
    # authored bigger to stay readable on a MacBook screen.
    def _add_color_legend_item(x, y, color, label, use_cross=False, scale=0.036):
        if use_cross:
            swatch = build_fire_map_cross_marker(
                fire_map_panel,
                color,
                half_size=box_half,
                thickness=2.8,
            )
            swatch.setPos(x + box_half, 0.0, y)
            swatch.setBin("fixed", 122)
        else:
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
            mayChange=True,
        )
        label_text.setBin("fixed", 123)
        label_text.setDepthWrite(False)
        label_text.setDepthTest(False)

    legend_columns = (
        FIRE_MAP_VIEW_LEFT,
        FIRE_MAP_VIEW_LEFT + 0.32,
        FIRE_MAP_VIEW_LEFT + 0.46,
    )
    for x, (color, label, use_cross) in zip(legend_columns, legend_items):
        _add_color_legend_item(x, legend_y, color, label, use_cross=use_cross)

    # The moving RED icons on the map are fire trucks - keep them captioned.
    legend_truck = build_fire_map_truck_marker(
        fire_map_panel,
        offset_x=FIRE_MAP_VIEW_LEFT + 0.63,
        offset_z=legend_y - 0.005,
    )
    legend_truck.setScale(1.25)
    legend_truck.show()
    truck_text = OnscreenText(
        text="red truck",
        parent=fire_map_panel,
        pos=(FIRE_MAP_VIEW_LEFT + 0.665, legend_y - 0.009),
        scale=0.030,
        fg=(1.0, 0.86, 0.82, 1.0),
        align=TextNode.ALeft,
        mayChange=True,
    )
    truck_text.setBin("fixed", 123)
    truck_text.setDepthWrite(False)
    truck_text.setDepthTest(False)

    purple_x = FIRE_MAP_VIEW_LEFT
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
        scale=0.028,
        fg=(0.96, 0.86, 1.0, 1.0),
        align=TextNode.ALeft,
        mayChange=True,
    )
    purple_text.setBin("fixed", 123)
    purple_text.setDepthWrite(False)
    purple_text.setDepthTest(False)

    # Caption for the fire as it looks in the 3D world (Cheryl, July 9): the
    # operator sees ONE orange - the undetected fire. On detection the flame
    # flips to the artificial blue/cyan rescue cue (same blue as the map pin).
    fires_text = OnscreenText(
        text="orange flame = fire not found yet   turns blue when found",
        parent=fire_map_panel,
        pos=(FIRE_MAP_VIEW_LEFT, legend_y - 0.112),
        scale=0.028,
        fg=(1.0, 0.72, 0.32, 1.0),
        align=TextNode.ALeft,
        mayChange=True,
    )
    fires_text.setBin("fixed", 123)
    fires_text.setDepthWrite(False)
    fires_text.setDepthTest(False)

    marks_text = OnscreenText(
        text="cyan ring = water target   green dots trees",
        parent=fire_map_panel,
        pos=(FIRE_MAP_VIEW_LEFT, legend_y - 0.154),
        scale=0.028,
        fg=(0.94, 0.94, 0.88, 1.0),
        align=TextNode.ALeft,
        mayChange=True,
    )
    marks_text.setBin("fixed", 123)
    marks_text.setDepthWrite(False)
    marks_text.setDepthTest(False)

    routes_text = OnscreenText(
        text="ROUTES: white survey   blue water   yellow selected",
        parent=fire_map_panel,
        pos=(FIRE_MAP_VIEW_LEFT, legend_y - 0.196),
        scale=0.028,
        fg=(0.78, 0.88, 0.96, 1.0),
        align=TextNode.ALeft,
        mayChange=True,
    )
    routes_text.setBin("fixed", 123)
    routes_text.setDepthWrite(False)
    routes_text.setDepthTest(False)


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
        scale=0.028,
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
    outer.setColor(0.25, 0.85, 1.0, 1.0)
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
    inner.setColor(0.68, 0.95, 1.0, 1.0)
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
    # Build defaults are BLUE (= detected fire) so a marker never shows a
    # stale orange frame before its first state update.
    halo_node = build_fire_map_circle(
        FIRE_MAP_FIRE_PULSE_INNER_RADIUS_METERS * 0.5,
        (0.2, 0.8, 1.0, 0.32),
        parent=root,
        thickness=1.2,
    )
    # X = fire out. Built green and hidden until the fire actually goes out
    # (Cheryl, July 28) so the map never shows an X on a burning fire.
    cross_node = build_fire_map_cross_marker(
        root,
        FIRE_MARK_OUT_COLOR,
        half_size=0.007,
        thickness=2.2,
    )
    cross_node.hide()
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

    root.setPythonTag("flame_node", flame_node)
    root.setPythonTag("firetruck_suppression_fill_node", firetruck_suppression_fill_node)
    root.setPythonTag("halo_node", halo_node)
    root.setPythonTag("cross_node", cross_node)
    root.setPythonTag("water_target_ring_node", water_target_ring_node)
    root.setPythonTag("water_target_cross_node", water_target_cross_node)
    root.setPythonTag("truck_node", truck_node)
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
    resize_label = OnscreenText(
        text="RESIZE",
        parent=fire_map_panel,
        pos=(bounds[0] - 0.008, bounds[2] + 0.012),
        scale=0.020,
        fg=(0.78, 0.9, 1.0, 1.0),
        align=TextNode.ARight,
        mayChange=True,
    )
    resize_label.setBin("fixed", 127)
    resize_label.setDepthWrite(False)
    resize_label.setDepthTest(False)
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
    build_water_refill_station_map_marker()
    build_fire_map_size_button()
    build_fire_map_legend()

    fire_map_header_text = OnscreenText(
        text="DETECTED FIRE MAP",
        parent=fire_map_panel,
        pos=(FIRE_MAP_PANEL_WIDTH * 0.5, FIRE_MAP_PANEL_HEIGHT - 0.05),
        scale=0.052,
        fg=(1.0, 0.84, 0.42, 1.0),
        align=TextNode.ACenter,
        mayChange=True,
    )
    fire_map_prompt_text = OnscreenText(
        text="Detected fire markers only: undetected fires stay hidden.",
        parent=fire_map_panel,
        pos=(FIRE_MAP_VIEW_LEFT, 0.094),
        scale=0.034,
        fg=(0.98, 0.96, 0.9, 1.0),
        align=TextNode.ALeft,
        mayChange=True,
        wordwrap=23.0,
    )
    fire_map_status_text = OnscreenText(
        text="No detected fires yet. S survey, W water.",
        parent=fire_map_panel,
        pos=(FIRE_MAP_VIEW_LEFT, 0.050),
        scale=0.024,
        fg=(0.86, 0.92, 0.96, 1.0),
        align=TextNode.ALeft,
        mayChange=True,
        wordwrap=32.0,
    )

    survey_map_marker_node = None
    survey_map_ring_node = None
    survey_map_label = None
    water_map_marker_node = None
    water_map_ring_node = None
    water_map_label = None

    build_fire_map_wind_widget()
    fire_map_panel.show()
