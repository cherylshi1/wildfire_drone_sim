# ---------------------------------------------------------------
# ALL-STAGES PARTICIPANT SUMMARY REPORT  (Cheryl, July 29, 2026)
#
# "right now it is per round; once all five stages are done there needs to be a
# summarised huge one for all stages data too."
#
# So this builds ONE report per participant covering every round they played:
#
#   page 1  every round in order: stage, score, base, operator delta, outcome
#           P1-P4, fires, burned area, drone losses, view switches
#           per-stage averages underneath
#   page 2  trust matrix M1-M4 run by run (the same four tables as the round
#           report, but across the whole session) + what each metric means
#   page 3  graphs: score per run, manual control per run, sigma' per run,
#           burned area per run
#   page 4  the mode-usage-over-runs figure for every drone
#
# Everything comes from files already written next to it, so the summary can be
# rebuilt at any time from a participant folder:
#   <pid>_stage<N>_<ts>_summary.csv   one per round
#   <pid>_trust_runs.csv              one row per drone per run
#   <pid>_trust_runs.json             the figure payload per run
#
# It is exported by the SUMMARY button on the results page, and automatically
# whenever the participant has at least one round on every stage.
# ---------------------------------------------------------------

PARTICIPANT_SUMMARY_FILE_SUFFIX = "_ALL_STAGES_summary.pdf"


def _summary_float(row, *keys):
    for key in keys:
        value = row.get(key)
        if value in (None, ""):
            continue
        try:
            return float(value)
        except (TypeError, ValueError):
            continue
    return None


def load_participant_round_summaries():
    """Every round this participant has played, oldest first, read back from the
    per-round summary CSVs."""
    import csv

    out_dir = _trust_reports_dir()
    if not os.path.isdir(out_dir):
        return []
    rounds = []
    for file_name in sorted(os.listdir(out_dir)):
        if not file_name.endswith("_summary.csv"):
            continue
        path = os.path.join(out_dir, file_name)
        try:
            with open(path, "r", newline="") as summary_file:
                rows = list(csv.DictReader(summary_file))
        except Exception:
            continue
        if not rows:
            continue
        row = rows[0]
        rounds.append({
            "file": file_name,
            "stage": row.get("stage", ""),
            "stage_label": row.get("stage_label", ""),
            "generated": row.get("generated", ""),
            "round_seconds": _summary_float(row, "round_seconds"),
            "outcome": row.get("outcome", ""),
            "score": _summary_float(row, "score", "grade_score"),
            "base": _summary_float(row, "score_base"),
            "operator_delta": _summary_float(row, "operator_delta"),
            "water_involved": _summary_float(row, "water_drones_involved"),
            "burned_pct": _summary_float(row, "burned_pct"),
            "composite_pct": _summary_float(row, "composite_score_pct"),
            "p1": _summary_float(row, "p1_detection_pct", "m1_detection_pct"),
            "p2": _summary_float(row, "p2_coverage_pct", "m2_coverage_pct"),
            "p3": _summary_float(row, "p3_suppression_pct", "m3_suppression_pct"),
            "p4": _summary_float(row, "p4_fire_out_pct", "m4_fire_out_pct"),
            "team_total": _summary_float(row, "team_total_drones"),
            "fires_total": _summary_float(row, "fires_total"),
            "fires_detected": _summary_float(row, "fires_detected"),
            "fires_out": _summary_float(row, "fires_out_end"),
            "fires_burned": _summary_float(row, "fires_burned_end"),
            "drone_losses": _summary_float(row, "drone_losses"),
            "view_switches": _summary_float(row, "view_switches"),
        })
    rounds.sort(key=lambda entry: entry.get("generated") or "")
    return rounds


def participant_has_all_stages(rounds=None):
    if rounds is None:
        rounds = load_participant_round_summaries()
    played = {str(entry.get("stage", "")).strip() for entry in rounds}
    return all(str(stage) in played for stage in EXPERIMENT_STAGES)


def get_participant_summary_path():
    participant = get_participant_folder_name() or "unknown"
    return os.path.join(
        _trust_reports_dir(),
        participant + PARTICIPANT_SUMMARY_FILE_SUFFIX,
    )


def export_participant_summary_pdf():
    """Build the all-stages report. Returns the path, or None with nothing to
    summarise."""
    import datetime

    rounds = load_participant_round_summaries()
    trust_history = load_trust_run_history()
    trust_figures = load_trust_run_figures()
    if not rounds and not trust_history:
        return None

    pages = []
    content = []
    page_number = 0
    head_c = (0.12, 0.18, 0.32)
    grey_c = (0.45, 0.47, 0.52)
    blue_c = (0.3, 0.62, 1.0)
    green_c = (0.16, 0.68, 0.32)
    red_c = (0.9, 0.22, 0.16)
    orange_c = (1.0, 0.6, 0.18)
    cyan_c = (0.30, 0.78, 0.88)
    magenta_c = (0.85, 0.20, 0.75)
    offline_c = (0.5, 0.5, 0.55)

    def esc(text_value):
        return (
            str(text_value)
            .replace("\r", " ")
            .replace("\n", " ")
            .replace("\\", "\\\\")
            .replace("(", "\\(")
            .replace(")", "\\)")
        )

    def text(x, y, string, size=8, color=(0.1, 0.1, 0.12)):
        content.append(
            "BT /F1 %.1f Tf %.3f %.3f %.3f rg %.1f %.1f Td (%s) Tj ET"
            % (size, color[0], color[1], color[2], x, y, esc(string))
        )

    def rect(x, y, w, h, color):
        content.append(
            "%.3f %.3f %.3f rg %.1f %.1f %.1f %.1f re f"
            % (color[0], color[1], color[2], x, y, w, h)
        )

    def rect_stroke(x, y, w, h, color, width=0.5):
        content.append(
            "%.3f %.3f %.3f RG %.2f w %.1f %.1f %.1f %.1f re S"
            % (color[0], color[1], color[2], width, x, y, w, h)
        )

    def polyline(points, color, width=1.2):
        points = list(points)
        if len(points) < 2:
            return
        segment = "%.3f %.3f %.3f RG %.2f w %.1f %.1f m " % (
            color[0], color[1], color[2], width, points[0][0], points[0][1],
        )
        for px, py in points[1:]:
            segment += "%.1f %.1f l " % (px, py)
        content.append(segment + "S")

    def finish_page():
        if content:
            pages.append("\n".join(content).encode("latin-1", "replace"))
            content.clear()

    def start_page(title, subtitle=""):
        nonlocal page_number
        finish_page()
        page_number += 1
        text(55, 755, title, 14, head_c)
        if subtitle:
            text(55, 740, subtitle, 8, grey_c)
        text(55, 20, "FSC all-stages summary | Page %d" % page_number, 7, grey_c)

    def draw_table(columns, rows, cursor_y, title=None, size=7,
                   row_height=9.0, left=55, right=560, page_title="",
                   widths=None):
        """widths = relative column widths, so a long date column cannot run
        into the next one (equal widths collided in the first version)."""
        column_count = max(1, len(columns))
        if widths and len(widths) == column_count:
            total_units = float(sum(widths)) or 1.0
            span = right - left
            column_x = []
            offset = 0.0
            for width in widths:
                column_x.append(left + span * (offset / total_units))
                offset += width
        else:
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
                start_page(page_title or "FSC All-Stages Summary (continued)")
                cursor_y = 726
            text(left, cursor_y, title, 9.5, head_c)
            cursor_y -= 13
        cursor_y = draw_header(cursor_y)
        for row in rows:
            if cursor_y < 40:
                start_page(page_title or "FSC All-Stages Summary (continued)")
                cursor_y = draw_header(726)
            for x, cell in zip(column_x, row):
                text(x, cursor_y, str(cell), size)
            cursor_y -= row_height
        return cursor_y - 8

    def draw_line_graph(x0, y0, width, height, series, title, y_label,
                        x_label="run"):
        """series = [(label, colour, [values in run order])]. Missing values are
        None and simply break the line."""
        text(x0, y0 + height + 12, title, 9, head_c)
        rect_stroke(x0, y0, width, height, (0.6, 0.62, 0.66))
        values = [
            value
            for _label, _color, points in series
            for value in points
            if value is not None
        ]
        if not values:
            text(x0 + 8, y0 + height * 0.5, "no data", 7, grey_c)
            return
        peak = max(values)
        floor = min(0.0, min(values))
        span = max(1e-6, peak - floor)
        point_count = max(
            2,
            max(len(points) for _label, _color, points in series),
        )
        for index, (label, color, points) in enumerate(series):
            plotted = []
            for point_index, value in enumerate(points):
                if value is None:
                    polyline(plotted, color, 1.2)
                    plotted = []
                    continue
                px = x0 + width * (point_index / float(max(1, point_count - 1)))
                py = y0 + height * ((value - floor) / span)
                plotted.append((px, py))
                rect(px - 1.0, py - 1.0, 2.0, 2.0, color)
            polyline(plotted, color, 1.2)
            legend_step = min(78.0, max(34.0, width / max(1, len(series))))
            text(x0 + 6 + index * legend_step, y0 + height - 9, label, 6.5, color)
        text(x0 - 26, y0 + height - 4, "%.0f" % peak, 6.5, grey_c)
        text(x0 - 26, y0, "%.0f" % floor, 6.5, grey_c)
        text(x0 - 26, y0 + height * 0.5, y_label, 6.5, grey_c)
        text(x0 + width * 0.5 - 8, y0 - 10, x_label, 6.5, grey_c)

    generated_at = datetime.datetime.now()
    participant = get_participant_folder_name() or "unknown"
    stages_played = sorted(
        {str(entry.get("stage", "")).strip() for entry in rounds if entry.get("stage") != ""}
    )

    # --- page 1: every round, then the per-stage averages --------------------
    start_page(
        "FSC Fire-Drone Trust Sim - All Stages Summary - %s" % participant,
        "%d rounds | stages played: %s | generated %s"
        % (
            len(rounds),
            ", ".join(stages_played) or "none",
            generated_at.strftime("%Y-%m-%d %H:%M:%S"),
        ),
    )
    cursor_y = 724

    def cell(value, fmt="%.0f"):
        return "-" if value is None else fmt % value

    round_rows = []
    for index, entry in enumerate(rounds, start=1):
        round_rows.append([
            "%d" % index,
            str(entry.get("stage", "-")),
            (entry.get("generated") or "-")[5:16].replace(" ", " "),
            cell(entry.get("score"), "%.0f"),
            cell(entry.get("base"), "%.0f"),
            cell(entry.get("operator_delta"), "%+.1f"),
            cell(entry.get("composite_pct"), "%.0f"),
            cell(entry.get("p1")),
            cell(entry.get("p2")),
            cell(entry.get("p3")),
            cell(entry.get("p4")),
            cell(entry.get("burned_pct"), "%.0f"),
            "%s/%s/%s"
            % (
                cell(entry.get("fires_total")),
                cell(entry.get("fires_detected")),
                cell(entry.get("fires_out")),
            ),
            cell(entry.get("drone_losses")),
            cell(entry.get("view_switches")),
        ])
    cursor_y = draw_table(
        [
            "Run", "Stage", "When", "Score", "Base", "Operator", "P1-P4 %",
            "P1", "P2", "P3", "P4", "Burned%", "Fires t/f/out", "Lost", "Views",
        ],
        round_rows or [["-"] * 15],
        cursor_y,
        title="EVERY ROUND",
        widths=(20, 22, 52, 28, 24, 34, 34, 22, 22, 22, 22, 34, 46, 22, 24),
    )
    text(
        60,
        cursor_y,
        "Score = collaboration base (P1-P4 mission performance out of 100) plus "
        "what the operator added or cost.",
        6.5,
        grey_c,
    )
    cursor_y -= 8
    text(
        60,
        cursor_y,
        "Fires t/f/out = total / found / put out. P1 detection speed, P2 mapping "
        "coverage, P3 suppression progress, P4 fire control.",
        6.5,
        grey_c,
    )
    cursor_y -= 18

    stage_rows = []
    for stage in stages_played:
        stage_rounds = [
            entry for entry in rounds if str(entry.get("stage", "")).strip() == stage
        ]

        def mean(key):
            numbers = [
                entry[key] for entry in stage_rounds if entry.get(key) is not None
            ]
            return (sum(numbers) / len(numbers)) if numbers else None

        stage_rows.append([
            stage,
            (
                EXPERIMENT_STAGES.get(int(stage), {}).get("label", "-")[:34]
                if stage.isdigit() else "-"
            ),
            "%d" % len(stage_rounds),
            cell(mean("score"), "%.1f"),
            cell(mean("base"), "%.0f"),
            cell(mean("operator_delta"), "%+.1f"),
            cell(mean("composite_pct"), "%.0f"),
            cell(mean("burned_pct"), "%.0f"),
            cell(mean("drone_losses"), "%.1f"),
            cell(mean("view_switches"), "%.1f"),
        ])
    cursor_y = draw_table(
        [
            "Stage", "What it was", "Rounds", "Score", "Base", "Operator",
            "P1-P4 %", "Burned%", "Lost", "Views",
        ],
        stage_rows or [["-"] * 10],
        cursor_y,
        title="PER STAGE AVERAGE",
        widths=(22, 120, 30, 30, 26, 34, 34, 32, 24, 26),
    )

    # --- page 2: trust matrix run by run ------------------------------------
    start_page(
        "FSC All Stages - Trust Matrix M1-M4 Run by Run - %s" % participant,
        "dS / dW / dTotal = change from the previous run in the survey mean, the "
        "water mean and the team mean",
    )
    cursor_y = 726
    for meaning in (
        "M1 sigma' = flown path length / planned path length. 1.0 = flew the ideal route, higher = detours.",
        "M2 obstacle clearance while flying by hand, mean in metres. Dispositional: it should settle per person.",
        "M3 share of the round that drone spent in manual. Falling across runs = growing trust in automation.",
        "M4 Hausdorff distance to the planned route, in metres. Small = same route tidied, large = own plan.",
    ):
        text(60, cursor_y, meaning, 7, grey_c)
        cursor_y -= 9
    cursor_y -= 6
    for metric_key, metric_label, _fmt, _higher in TRUST_METRIC_DEFINITIONS:
        columns, table_rows = build_trust_run_table(metric_key, trust_history)
        cursor_y = draw_table(
            columns,
            table_rows,
            cursor_y,
            title=metric_label,
        )

    # --- page 3: graphs -----------------------------------------------------
    start_page(
        "FSC All Stages - Trends - %s" % participant,
        "One point per round, in the order they were played",
    )
    draw_line_graph(
        95, 560, 420, 130,
        [
            ("score", green_c, [entry.get("score") for entry in rounds]),
            ("base", blue_c, [entry.get("base") for entry in rounds]),
            (
                "outcome P1-P4 %",
                orange_c,
                [entry.get("composite_pct") for entry in rounds],
            ),
        ],
        "SCORE AND OUTCOME PER ROUND",
        "points",
    )
    trust_labels = trust_history_drone_labels(trust_history)
    palette = (blue_c, orange_c, green_c, magenta_c, red_c, cyan_c)
    draw_line_graph(
        95, 380, 420, 130,
        [
            (
                label,
                palette[index % len(palette)],
                [
                    run["drones"].get(label, {}).get("m3_manual_pct")
                    for run in trust_history
                ],
            )
            for index, label in enumerate(trust_labels)
        ],
        "M3 MANUAL CONTROL PER RUN, PER DRONE (falling = growing trust)",
        "% manual",
    )
    draw_line_graph(
        95, 200, 420, 130,
        [
            (
                label,
                palette[index % len(palette)],
                [
                    run["drones"].get(label, {}).get("m1_sigma")
                    for run in trust_history
                ],
            )
            for index, label in enumerate(trust_labels)
        ],
        "M1 PATH LENGTH RATIO SIGMA' PER RUN, PER DRONE (1.0 = ideal route)",
        "sigma'",
    )
    draw_line_graph(
        95, 60, 420, 95,
        [("burned %", red_c, [entry.get("burned_pct") for entry in rounds])],
        "BURNED AREA PER ROUND",
        "% of map",
    )

    # --- page 4: the mode usage figure across runs --------------------------
    figure_labels = sorted(
        {
            label
            for figure_entry in trust_figures
            for label in figure_entry.get("drones", {})
        },
        key=trust_drone_sort_key,
    )
    if figure_labels:
        figure_title = "FSC All Stages - M3 Mode Usage Over Runs - %s" % participant
        figure_subtitle = (
            "cyan automatic | orange manual | grey offline | green tick event | "
            "red X drone lost | magenta dash planned length | black distance to obstacle"
        )
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
            block_height = 14 + len(entries) * (band_h + band_gap) + 20
            if band_y - block_height < 40:
                start_page(figure_title + " (continued)", figure_subtitle)
                band_y = 716
            text(55, band_y, "%s   (0 to %.0f m flown)" % (label, max_length), 9, head_c)
            band_y -= 13
            for figure_entry in entries:
                drone_entry = figure_entry["drones"][label]
                scale = axis_w / max_length
                rect_stroke(axis_x, band_y, axis_w, band_h, (0.82, 0.84, 0.88), 0.4)
                for start_m, end_m, state_name in drone_entry.get("segments", ()):
                    x0 = axis_x + max(0.0, start_m) * scale
                    x1 = axis_x + max(0.0, end_m) * scale
                    if x1 - x0 <= 0.3:
                        continue
                    segment_color = (
                        cyan_c if state_name == "auto"
                        else (orange_c if state_name == "manual" else offline_c)
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
                text(
                    58,
                    band_y + 1,
                    "Run %s (st %s)"
                    % (figure_entry.get("run", "?"), figure_entry.get("stage", "?")),
                    6.5,
                )
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
            text(
                axis_x + axis_w * 0.5 - 34,
                band_y - 4,
                "path length flown (m), black trace = 0 to %.0f m clearance"
                % TRUST_OBSTACLE_SEARCH_RADIUS_METERS,
                6.5,
                grey_c,
            )
            band_y -= 16

    finish_page()
    pdf = assemble_pdf_bytes(pages)
    os.makedirs(_trust_reports_dir(), exist_ok=True)
    file_path = get_participant_summary_path()
    with open(file_path, "wb") as summary_file:
        summary_file.write(pdf)
    return file_path


def export_participant_summary_if_complete():
    """Auto-export once every stage has been played at least once."""
    try:
        if not participant_has_all_stages():
            return None
        return export_participant_summary_pdf()
    except Exception as summary_error:
        print("[real-sim] could not build the all-stages summary: %s" % summary_error)
        return None
