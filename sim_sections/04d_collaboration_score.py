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


def collaboration_base_points():
    """Aug 9, 2026 (Cheryl): the base is no longer a fixed 40/67 water tier. It
    IS the round's P1-P4 mission performance, out of 100, so the collaboration
    score and the P1-P4 score overlap: a hands-off round scores exactly its
    mission performance. The operator's own credit and cost are layered on top.

    The P1-P4 performance already responds to the round's difficulty factors
    (fire spread rate, team size, drone speed, ground-crew and water-drone
    suppression rates) because harder fires simply leave fewer points on P1-P4.
    Turning those factors into an explicit, comparable calibration is the next
    step (Cheryl's calibration task)."""
    performance = compute_performance_matrix(motion_state.sim_time_seconds)
    return clamp(performance["performance_score"] * 100.0, 0.0, 100.0)


def collaboration_base_label():
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
    base_points = collaboration_base_points()
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
    score = clamp(base_points + operator_delta, 0.0, 100.0)
    return {
        "score": score,
        "base": base_points,
        "base_label": collaboration_base_label(),
        "water_involved": water_drones_were_involved(),
        "credits": credits,
        "penalties": penalties,
        "credit_total": credit_total,
        "penalty_total": penalty_total,
        "operator_delta": operator_delta,
        "manual_detection_count": manual_detection_count,
        "manual_suppression_fires": manual_suppression_fires,
        "ground_work_seconds": round_ground_suppression_work_seconds,
        "water_work_seconds": round_water_suppression_work_seconds,
        "manual_water_work_seconds": round_manual_water_suppression_work_seconds,
    }


def collaboration_run_succeeded():
    """Did the operator at least match the automation baseline they were given?"""
    grade = compute_collaboration_score()
    return grade["score"] >= grade["base"] + COLLABORATION_SUCCESS_MARGIN_POINTS


def compose_collaboration_score_lines():
    grade = compute_collaboration_score()
    lines = [
        "SCORE: %.0f/100" % grade["score"],
        "  base %.0f  (%s)" % (grade["base"], grade["base_label"]),
    ]
    for label, points in grade["credits"].items():
        if points >= 0.05:
            lines.append("  +%.1f  %s" % (points, label))
    for label, points in grade["penalties"].items():
        if points >= 0.05:
            lines.append("  -%.1f  %s" % (points, label))
    if grade["credit_total"] < 0.05 and grade["penalty_total"] < 0.05:
        lines.append("  operator changed nothing: hands-off baseline")
    else:
        lines.append("  operator total %+.1f" % grade["operator_delta"])
    return lines


def compose_collaboration_formula_lines():
    lines = []

    def block(title, rows):
        lines.append(title)
        lines.extend(("  " + row) if row else "" for row in rows)
        lines.append("")

    block("17. COLLABORATION SCORE OUT OF 100", (
        "The score says how the responders + fire did, then what the operator",
        "added on top of that:",
        "",
        "  base  = P1-P4 mission performance x 100  (Aug 9, 2026: the fixed",
        "          40/67 water tier was dropped; the base now overlaps the",
        "          P1-P4 score, so a hands-off round scores exactly its P1-P4)",
        "  score = clamp(base + operator credit - operator cost, 0, 100)",
        "",
        "Because the base is P1-P4, it already reflects the round's difficulty",
        "factors: fire spread rate, number and speed of the drones, ground-crew",
        "and water-drone suppression rates all leave their mark on P1-P4. Making",
        "those factors an explicit, comparable calibration is the next step.",
        "",
        "OPERATOR CREDIT",
        "  + %.1f per fire first found while that survey drone was in MANUAL"
        % COLLABORATION_MANUAL_DETECTION_POINTS,
        "  + %.1f per fire's worth of suppression sprayed manually with [J],"
        % COLLABORATION_MANUAL_SUPPRESSION_POINTS_PER_FIRE,
        "    measured as manual_water_work / %.1f work-seconds per fire"
        % _collab_full_suppression_work_seconds(),
        "",
        "OPERATOR COST",
        "  - %.1f per drone lost (collision, branch, ground, kill switch)"
        % COLLABORATION_DRONE_LOSS_PENALTY_POINTS,
        "  - %.1f per fire that burned or was never found, scaled by how much"
        % COLLABORATION_MISSED_FIRE_PENALTY_POINTS,
        "    of that fire's life the operator held the responsible survey drone",
        "    in MANUAL (so an automation miss is not charged to the person)",
        "",
        "A hands-off full-automation round has no credit and no cost, so it",
        "scores exactly the base, the same number every time. That is the",
        "baseline each participant is measured against, and it is why the",
        "team-size effectiveness multiplier and the extra-drone bonus were",
        "removed from the mission score: fairness across team sizes is handled",
        "by making the fire harder for bigger teams, not by scaling points.",
    ))

    while lines and lines[-1] == "":
        lines.pop()
    return lines
