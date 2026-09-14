# ---------------------------------------------------------------
# COLLABORATION SCORE OUT OF 100  (Cheryl, July 29, 2026)
#
# The problem this replaces: the old 100-point grade was pure outcome
# (area burned, fires missed, ...) and the composite was multiplied by a
# team-size effectiveness factor, so the same hands-off round scored a
# different number every time and different team sizes were not comparable.
#
# The rule now:
#
#   survey drones + ground crews doing the work, no water drone suppression
#       -> 40 points
#   water drones contributing suppression as well
#       -> 67 points
#
# That is the automation baseline: a round where the operator never touches a
# drone scores exactly the base for the collaboration that happened, every
# time. Anything above or below comes from the operator:
#
#   + fires the operator found while flying a survey drone by hand
#   + suppression the operator did with manual [J] spray
#   - drones lost (manual flying has no separation assist; the kill switch is
#     an operator action too)
#   - fires that burned or were never found while the responsible survey drone
#     was in the operator's hands instead of sweeping
#
# So the score answers the collaboration question directly: what did the three
# responders achieve on their own, and what did the human add or cost?
#
# Round totals live here because the per-hotspot work counters are reset when a
# fire reignites, and the collaboration tier has to look at the whole round.
# ---------------------------------------------------------------

round_ground_suppression_work_seconds = 0.0
round_water_suppression_work_seconds = 0.0
round_manual_water_suppression_work_seconds = 0.0


def reset_round_suppression_work():
    global round_ground_suppression_work_seconds
    global round_water_suppression_work_seconds
    global round_manual_water_suppression_work_seconds
    round_ground_suppression_work_seconds = 0.0
    round_water_suppression_work_seconds = 0.0
    round_manual_water_suppression_work_seconds = 0.0


def accumulate_round_suppression_work(ground_work, water_work, manual_water_work):
    """Called once per fire per step from update_hotspot_lifecycle."""
    global round_ground_suppression_work_seconds
    global round_water_suppression_work_seconds
    global round_manual_water_suppression_work_seconds
    round_ground_suppression_work_seconds += max(0.0, ground_work)
    round_water_suppression_work_seconds += max(0.0, water_work)
    round_manual_water_suppression_work_seconds += max(0.0, manual_water_work)


def water_drones_were_involved():
    """Did the water drones actually put work on a fire this round? No longer
    drives the base score (the 40/67 tier was dropped Aug 9, 2026); kept only as
    an informational flag for the data export."""
    return (
        round_water_suppression_work_seconds
        >= COLLABORATION_WATER_INVOLVEMENT_MIN_WORK_SECONDS
    )


# --- Calibration to a common full-auto anchor (Aug 18, 2026, Cheryl) ---------
# Full automation scores a genuinely different raw P1-P4 on each stage, because
# the stages are genuinely different difficulties (2/4/6 drones, water-only,
# etc.). To make a participant's result comparable ACROSS stages, each stage's
# measured hands-off baseline is mapped onto ONE common anchor, and the raw
# score is shifted by the same amount:
#
#     calibrated = anchor + (raw - baseline[stage])
#
# So a hands-off round scores the anchor on EVERY stage, and a participant's
# distance above/below the anchor is exactly what they added over the
# automation, on one common scale (O'Neill et al. 2022: the human's value is
# the team result minus automation-alone; Kahneman & Tversky 1979: judge a
# result against its reference point). The operator credit/cost still layer on
# top of the calibrated base.
#
# baseline[stage] = mean full-auto P1-P4 x 100, measured deterministically on
# the fixed map. These built-in numbers are only fallbacks; the real ones must
# be measured ON THE MACHINE THAT RUNS THE STUDY, because the 3D trees and
# drones change how the automation flies. Measure and store them with one
# command:  python3 scripts/headless_run.py calibrate
# That writes calibration_baselines.json next to main.py, which is loaded below
# and overrides these fallbacks, so full-auto lands on the anchor on your
# machine and stays correct whenever you retune the fire, resources or team.
STAGE_FULL_AUTO_BASELINE_SCORES = {
    0: 46.6,   # DEMO practice (2 drones)
    1: 65.7,   # 2 drones, water-only
    2: 33.0,   # 2 drones, full control
    3: 59.5,   # 4 drones
    4: 56.7,   # 6 drones
}
# Load machine-measured baselines if present (written by the calibrate command).
CALIBRATION_BASELINE_FILE = PROJECT_ROOT / "calibration_baselines.json"
CALIBRATION_BASELINE_SOURCE = "built-in fallback"
try:
    if CALIBRATION_BASELINE_FILE.exists():
        _loaded_baselines = json.loads(CALIBRATION_BASELINE_FILE.read_text(encoding="utf-8"))
        for _stage_key, _score in (_loaded_baselines.get("baselines") or {}).items():
            STAGE_FULL_AUTO_BASELINE_SCORES[int(_stage_key)] = float(_score)
        CALIBRATION_BASELINE_SOURCE = "measured on this machine (%s)" % (
            _loaded_baselines.get("measured_at", "date unknown")
        )
        # Baselines are specific to the round length (a 3-minute and a 10-minute
        # round have different fire dynamics). Warn loudly if they were measured
        # at a different profile than the one now running.
        _baseline_minutes = _loaded_baselines.get("round_minutes")
        _current_minutes = round_duration_seconds / 60.0
        if _baseline_minutes is not None and abs(_baseline_minutes - _current_minutes) > 0.5:
            print(
                "[real-sim] WARNING: calibration_baselines.json was measured at a "
                "%.0f-minute round but this round is %.0f minutes. The calibration "
                "will be off. Re-run: python3 scripts/headless_run.py calibrate "
                "under the same profile." % (_baseline_minutes, _current_minutes)
            )
except Exception as _baseline_error:
    print("[real-sim] could not read calibration_baselines.json: %s" % _baseline_error)
# The common anchor every stage's hands-off round maps to. Cheryl (Aug 18,
# 2026) chose the current mean of the four scored stages (1-4) so no stage has
# to move further than necessary. Override with REAL_SIM_CALIBRATION_ANCHOR.
CALIBRATION_ANCHOR_SCORE = read_nonnegative_float_env(
    "REAL_SIM_CALIBRATION_ANCHOR",
    sum(STAGE_FULL_AUTO_BASELINE_SCORES[stage] for stage in (1, 2, 3, 4)) / 4.0,
)
# Master switch; set REAL_SIM_CALIBRATION=0 to report the raw P1-P4 base.
CALIBRATION_ENABLED = read_bool_env("REAL_SIM_CALIBRATION", True)


def stage_full_auto_baseline_score():
    """This stage's measured hands-off P1-P4 score (out of 100). Unknown stages
    fall back to the anchor, so the calibration is a no-op for them."""
    return STAGE_FULL_AUTO_BASELINE_SCORES.get(
        selected_stage, CALIBRATION_ANCHOR_SCORE
    )


def raw_mission_score():
    """The uncalibrated P1-P4 mission performance, out of 100."""
    performance = compute_performance_matrix(motion_state.sim_time_seconds)
    return clamp(performance["performance_score"] * 100.0, 0.0, 100.0)


def calibrate_mission_score(raw_score):
    """Shift a raw P1-P4 score so this stage's hands-off baseline lands on the
    common anchor. Full auto -> anchor on every stage; a participant keeps the
    exact distance above/below that the automation left on the table."""
    if not CALIBRATION_ENABLED:
        return clamp(raw_score, 0.0, 100.0)
    shifted = CALIBRATION_ANCHOR_SCORE + (raw_score - stage_full_auto_baseline_score())
    return clamp(shifted, 0.0, 100.0)


def collaboration_base_points():
    """The base the operator is judged against. Aug 18, 2026 (Cheryl): the base
    is now the CALIBRATED mission score, so a hands-off round scores the common
    anchor on every stage instead of the stage's raw P1-P4. That makes results
    comparable across stages of different difficulty; the raw score, the stage
    baseline and the anchor are all carried through to the report. The operator
    credit and cost are still layered on top of this base."""
    return calibrate_mission_score(raw_mission_score())


def collaboration_base_label():
    if CALIBRATION_ENABLED:
        return "calibrated mission score (full-auto anchored to %.0f)" % (
            CALIBRATION_ANCHOR_SCORE
        )
    return "P1-P4 mission performance"


def _collab_full_suppression_work_seconds():
    return max(
        1.0,
        FIRE_SUPPRESSION_CONTAIN_DELAY_SECONDS
        + FIRE_SUPPRESSION_EXTINGUISH_DELAY_SECONDS,
    )


def _collab_survey_manual_fraction(slot, start_time, end_time, samples=8):
    """How much of a window the operator held that survey drone in MANUAL."""
    timeline = drone_mode_timeline.get("S%d" % int(slot)) or []
    if not timeline or end_time <= start_time:
        return 0.0
    manual_samples = 0
    for index in range(samples):
        sample_time = start_time + (end_time - start_time) * ((index + 0.5) / samples)
        if mode_state_at_time(timeline, sample_time) is False:
            manual_samples += 1
    return manual_samples / float(samples)


def compute_collaboration_score():
    """The 100-point score, broken into the base and what the operator changed.

    Returns a dict with 'score', 'base', 'base_label', 'credits', 'penalties'
    (both dicts of labelled points) and the totals, so the report can show the
    full arithmetic instead of a bare number.
    """
    raw_score = raw_mission_score()
    stage_baseline = stage_full_auto_baseline_score()
    base_points = calibrate_mission_score(raw_score)
    full_work = _collab_full_suppression_work_seconds()

    # --- operator credit ---
    manual_detection_count = sum(
        1
        for hotspot in fire_hotspots
        if hotspot.detection_time_seconds is not None
        and getattr(hotspot, "detected_under_manual_control", False)
    )
    manual_detection_points = (
        manual_detection_count * COLLABORATION_MANUAL_DETECTION_POINTS
    )
    manual_suppression_fires = (
        round_manual_water_suppression_work_seconds / full_work
    )
    manual_suppression_points = (
        manual_suppression_fires * COLLABORATION_MANUAL_SUPPRESSION_POINTS_PER_FIRE
    )
    credits = {
        "fires found flying manually": manual_detection_points,
        "suppression sprayed manually": manual_suppression_points,
    }

    # --- operator cost ---
    drone_loss_points = (
        len(drone_damage_events) * COLLABORATION_DRONE_LOSS_PENALTY_POINTS
    )
    missed_fire_points = 0.0
    now_seconds = motion_state.sim_time_seconds
    for hotspot in fire_hotspots:
        never_found = hotspot.detection_time_seconds is None
        burned = hotspot.suppression_state == FIRE_STATE_BURNED
        if not (never_found or burned):
            continue
        slot = get_hotspot_survey_sector_slot(hotspot)
        manual_fraction = _collab_survey_manual_fraction(
            slot,
            hotspot.ignition_time_seconds,
            now_seconds,
        )
        # Only the share of the fire's life the operator was holding the drone
        # counts, so an automation failure is not blamed on the person.
        missed_fire_points += (
            COLLABORATION_MISSED_FIRE_PENALTY_POINTS * manual_fraction
        )
    penalties = {
        "drones lost": drone_loss_points,
        "fires missed while flying manually": missed_fire_points,
    }

    credit_total = sum(credits.values())
    penalty_total = sum(penalties.values())
    operator_delta = credit_total - penalty_total
    # Aug 20, 2026 (Cheryl): OUTCOME-ONLY score. The manual-action point bonuses
    # (+4/+6/-8/-5) are NOT added to the score. They were unbounded (holding the
    # [J] spray could farm +56 and cap the score at 100) and they double-counted
    # the mission outcome, since manual flying and spraying already raise P1-P4.
    # The score is now purely the calibrated mission outcome; the operator's
    # actions still help through P1-P4 and are reported as descriptive counts.
    missed_fire_count = sum(
        1
        for hotspot in fire_hotspots
        if hotspot.detection_time_seconds is None
        or hotspot.suppression_state == FIRE_STATE_BURNED
    )
    score = clamp(base_points, 0.0, 100.0)
    return {
        "score": score,
        "base": base_points,
        "base_label": collaboration_base_label(),
        "raw_mission_score": raw_score,
        "stage_baseline_score": stage_baseline,
        "calibration_anchor": CALIBRATION_ANCHOR_SCORE,
        "calibration_enabled": CALIBRATION_ENABLED,
        "water_involved": water_drones_were_involved(),
        # Kept for the data export and the "what you did" report section, but no
        # longer added to the score (outcome-only, Aug 20).
        "credits": credits,
        "penalties": penalties,
        "credit_total": credit_total,
        "penalty_total": penalty_total,
        "operator_delta": operator_delta,
        "manual_detection_count": manual_detection_count,
        "manual_suppression_fires": manual_suppression_fires,
        "drones_lost": len(drone_damage_events),
        "missed_fire_count": missed_fire_count,
        "ground_work_seconds": round_ground_suppression_work_seconds,
        "water_work_seconds": round_water_suppression_work_seconds,
        "manual_water_work_seconds": round_manual_water_suppression_work_seconds,
    }


def collaboration_run_succeeded():
    """Did the operator match or beat the automation on this stage? Outcome-only
    (Aug 20): the score IS the calibrated mission outcome, so beating automation
    means the score reached the anchor (where full automation lands)."""
    grade = compute_collaboration_score()
    return grade["score"] >= grade["calibration_anchor"] + COLLABORATION_SUCCESS_MARGIN_POINTS


def compose_collaboration_score_lines():
    grade = compute_collaboration_score()
    lines = ["SCORE: %.0f/100  (calibrated mission outcome)" % grade["score"]]
    if grade.get("calibration_enabled"):
        lines.append(
            "  raw P1-P4 %.0f, this stage's automation baseline %.0f"
            % (grade["raw_mission_score"], grade["stage_baseline_score"])
        )
        lines.append(
            "  -> adjusted so full automation scores %.0f on every stage"
            % grade["calibration_anchor"]
        )
    # Manual actions are recorded but do NOT change the score (outcome-only).
    lines.append(
        "  you did (not scored): %d fire(s) found by hand, %.1f fire(s) of manual spray, %d drone(s) lost"
        % (
            grade["manual_detection_count"],
            grade["manual_suppression_fires"],
            grade["drones_lost"],
        )
    )
    return lines


def compose_collaboration_formula_lines():
    lines = []

    def block(title, rows):
        lines.append(title)
        lines.extend(("  " + row) if row else "" for row in rows)
        lines.append("")

    block("17. THE SCORE OUT OF 100 (outcome-only)", (
        "The score is ONE number: the round's mission outcome, adjusted for the",
        "stage's difficulty. Nothing else is added.",
        "",
        "  raw   = P1-P4 mission performance x 100",
        "  score = clamp( anchor + (raw - full_auto_baseline[stage]) , 0 , 100 )",
        "",
        "CALIBRATION (Aug 18, 2026): each stage is a different difficulty, so a",
        "hands-off full-auto round scores a different raw P1-P4 on each. The score",
        "shifts that raw score so every stage's hands-off baseline lands on ONE",
        "common anchor = %.0f. A hands-off round therefore scores the anchor on"
        % CALIBRATION_ANCHOR_SCORE,
        "every stage, and a participant's distance above/below the anchor is what",
        "they added over the automation, on a scale that is comparable across",
        "stages (O'Neill et al. 2022: value = team result - automation alone).",
        "",
        "  full-auto baselines used (mean P1-P4 x 100, measured on the fixed map):",
        "    stage 1 = %.0f   stage 2 = %.0f   stage 3 = %.0f   stage 4 = %.0f"
        % (
            STAGE_FULL_AUTO_BASELINE_SCORES[1],
            STAGE_FULL_AUTO_BASELINE_SCORES[2],
            STAGE_FULL_AUTO_BASELINE_SCORES[3],
            STAGE_FULL_AUTO_BASELINE_SCORES[4],
        ),
        "  Re-measure these (scripts/headless_run.py calibrate) if the fire,",
        "  resources or team are retuned.",
        "",
        "WHY NO SEPARATE OPERATOR POINTS (Aug 20, 2026): the manual actions the",
        "operator takes (finding a fire by hand, spraying with [J], and drone",
        "losses) ALREADY show up in the mission outcome, because they change how",
        "many fires are found and put out (P1-P4). Adding a separate per-action",
        "bonus double-counted that work and was unbounded (holding the spray key",
        "could farm points and cap the score at 100). So those actions are now",
        "RECORDED for the record and the CSV, but they do not change the score.",
        "The operator raises the score only by producing a better outcome than",
        "the automation would have. The team-size multiplier and extra-drone",
        "bonus were removed for the same comparability reason; fairness across",
        "team sizes is handled by making the fire harder for bigger teams.",
    ))

    while lines and lines[-1] == "":
        lines.pop()
    return lines
