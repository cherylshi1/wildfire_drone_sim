# ---------------------------------------------------------------
# MISSION SCORE P1-P4 (the wildfire OUTCOME matrix)
#
# Split out of 04_mission_rules_and_dispatch.py on July 28, 2026 so the
# scoring lives in one readable place. These functions read the same shared
# globals as every other section (fire_hotspots, team_* counts, PERFORMANCE_*
# constants) and are only ever called at run time, so load order is unchanged.
#
# P1 detection speed | P2 mapping coverage | P3 suppression progress
# P4 fire put out. Renamed from M1-M4 on July 29, 2026: M1-M4 now mean
# Darya's behavioural TRUST metrics, which live in 04c_trust_metrics.py.
# Each part is scaled by team_role_effectiveness so a bigger team is expected
# to do better, then weighted into the composite score.
#
# Also holds the credit split for WHO put a fire out (fire trucks vs water
# drones), which the end-of-round report breaks down.
# ---------------------------------------------------------------


def mapping_coverage_score():
    if not fire_hotspots:
        return 0.0
    mapped_equivalent = sum(
        hotspot_mapping_fraction(hotspot)
        for hotspot in fire_hotspots
    )
    return clamp(mapped_equivalent / len(fire_hotspots), 0.0, 1.0)


def suppression_progress_score():
    detected_hotspots = [
        hotspot
        for hotspot in fire_hotspots
        if hotspot.detected
    ]
    if not detected_hotspots:
        return 0.0

    contain_delay = max(1.0, FIRE_SUPPRESSION_CONTAIN_DELAY_SECONDS)
    extinguish_delay = max(1.0, FIRE_SUPPRESSION_EXTINGUISH_DELAY_SECONDS)
    hotspot_progress_values = []
    for hotspot in detected_hotspots:
        if hotspot.suppression_state == FIRE_STATE_OUT:
            hotspot_progress_values.append(1.0)
            continue
        if hotspot.suppression_state == FIRE_STATE_BURNED:
            hotspot_progress_values.append(0.0)
            continue
        if hotspot.suppression_state == FIRE_STATE_CONTAINED:
            contained_progress = clamp(
                (hotspot.suppression_work_seconds - contain_delay) / extinguish_delay,
                0.0,
                1.0,
            )
            hotspot_progress_values.append(0.5 + (contained_progress * 0.5))
            continue
        active_progress = clamp(
            hotspot.suppression_work_seconds / contain_delay,
            0.0,
            1.0,
        )
        if hotspot.ground_firefighter_engaged:
            active_progress = max(
                active_progress,
                PERFORMANCE_FIREFIGHTER_SUPPRESSION_BASE_SCORE,
            )
        hotspot_progress_values.append(active_progress * 0.5)

    if not hotspot_progress_values:
        return 0.0
    return clamp(
        sum(hotspot_progress_values) / len(hotspot_progress_values),
        0.0,
        1.0,
    )


def fire_out_control_score():
    if not fire_hotspots:
        return 0.0

    contain_delay = max(1.0, FIRE_SUPPRESSION_CONTAIN_DELAY_SECONDS)
    extinguish_delay = max(1.0, FIRE_SUPPRESSION_EXTINGUISH_DELAY_SECONDS)
    full_suppression_work = contain_delay + extinguish_delay
    hotspot_control_values = []

    for hotspot in fire_hotspots:
        if hotspot.suppression_state == FIRE_STATE_OUT:
            hotspot_control_values.append(1.0)
            continue
        if hotspot.suppression_state == FIRE_STATE_CONTAINED:
            contained_elapsed = max(
                0.0,
                hotspot.suppression_work_seconds - contain_delay,
            )
            extinguish_progress = clamp(
                contained_elapsed / extinguish_delay,
                0.0,
                1.0,
            )
            hotspot_control_values.append(
                PERFORMANCE_CONTAINED_FIRE_CONTROL_CREDIT
                + ((1.0 - PERFORMANCE_CONTAINED_FIRE_CONTROL_CREDIT) * extinguish_progress)
            )
            continue
        if hotspot.suppression_state == FIRE_STATE_ACTIVE and hotspot.detected:
            hotspot_control_values.append(
                clamp(hotspot.suppression_work_seconds / full_suppression_work, 0.0, 0.35)
            )
            continue
        hotspot_control_values.append(0.0)

    return clamp(
        sum(hotspot_control_values) / max(1, len(hotspot_control_values)),
        0.0,
        1.0,
    )


def compute_performance_matrix(current_time_seconds):
    # Four-part wildfire score:
    # - finding the fire: detection speed + coverage
    # - controlling the fire: suppression progress + putting the fire out
    coverage_score = mapping_coverage_score()
    suppression_score = suppression_progress_score()
    fire_out_score = fire_out_control_score()

    # Detection speed (P1): average how promptly EACH fire was found, across all
    # fires that have ignited. Recomputed every frame so the score keeps updating
    # as new fires ignite and as the drone detects them.
    detection_target_seconds = max(0.1, PERFORMANCE_DETECTION_TARGET_SECONDS)
    per_fire_detection_scores = []
    for hotspot in fire_hotspots:
        if hotspot.detection_time_seconds is not None:
            # Genuinely detected while active: delay = detect time - ignition time.
            fire_delay_seconds = max(
                0.0,
                hotspot.detection_time_seconds - hotspot.ignition_time_seconds,
            )
        else:
            # Not yet detected (still searching, or it burned out undetected):
            # ongoing penalty that grows the longer it stays unfound.
            fire_delay_seconds = max(
                0.0,
                current_time_seconds - hotspot.ignition_time_seconds,
            )
        per_fire_detection_scores.append(
            1.0 - clamp(fire_delay_seconds / detection_target_seconds, 0.0, 1.0)
        )

    if per_fire_detection_scores:
        detection_score = sum(per_fire_detection_scores) / len(per_fire_detection_scores)
        detection_delay_seconds = None
    else:
        detection_score = 0.5
        detection_delay_seconds = None

    survey_drone_count = get_team_survey_drone_count()
    water_drone_count = team_water_drone_count
    # July 29 (Cheryl): the team-size effectiveness multiplier is GONE from the
    # score. It moved the achievable ceiling around with team size, so the same
    # round scored differently for 2, 4 and 6 drones and no round was
    # comparable to another. Fairness across team sizes is handled where it
    # belongs, in the fire itself (FIRE_TEAM_SIZE_SPREAD_RATE_MULTIPLIERS).
    # team_role_effectiveness stays in the code because the detection model and
    # the older reports refer to it, but it no longer scales any points.
    survey_effectiveness = 1.0
    water_effectiveness = 1.0

    p1_score = clamp(detection_score, 0.0, 1.0)
    p2_score = clamp(coverage_score, 0.0, 1.0)
    p3_score = clamp(suppression_score, 0.0, 1.0)
    p4_score = clamp(fire_out_score, 0.0, 1.0)

    performance_weight_total = (
        PERFORMANCE_WEIGHT_P1
        + PERFORMANCE_WEIGHT_P2
        + PERFORMANCE_WEIGHT_P3
        + PERFORMANCE_WEIGHT_P4
    )
    if performance_weight_total <= 0.0:
        performance_weight_total = 1.0
    raw_performance_score = clamp(
        (
            (PERFORMANCE_WEIGHT_P1 * p1_score)
            + (PERFORMANCE_WEIGHT_P2 * p2_score)
            + (PERFORMANCE_WEIGHT_P3 * p3_score)
            + (PERFORMANCE_WEIGHT_P4 * p4_score)
        ) / performance_weight_total,
        0.0,
        1.0,
    )
    # The extra-drone bonus is gone for the same reason (July 29): a bigger team
    # must not start from a higher number.
    extra_drone_bonus = 0.0
    performance_score = clamp(raw_performance_score, 0.0, 1.0)
    return {
        "performance_score": performance_score,
        "raw_performance_score": raw_performance_score,
        "p1_score": p1_score,
        "p2_score": p2_score,
        "p3_score": p3_score,
        "p4_score": p4_score,
        "coverage_score": coverage_score,
        "detection_score": detection_score,
        "detection_delay_seconds": detection_delay_seconds,
        "suppression_score": suppression_score,
        "fire_out_score": fire_out_score,
        "burn_control_score": fire_out_score,
        "team_total_drones": team_total_drone_count,
        "team_survey_drones": survey_drone_count,
        "team_water_drones": water_drone_count,
        "survey_effectiveness": survey_effectiveness,
        "water_effectiveness": water_effectiveness,
        "extra_drone_bonus": extra_drone_bonus,
    }


def reward_eligible():
    # July 29: eligibility follows the collaboration score, so a participant is
    # judged against the automation baseline they were given rather than a fixed
    # outcome percentage that team size used to shift.
    return collaboration_run_succeeded()

# --- Who put the fire out: trucks, drones, or both ---------------------------
# Cheryl (July 28): the report should read like "11 put out by fire trucks,
# 20 by drones, 5 shared 20% truck / 80% drone". Work seconds are accumulated
# per source in update_hotspot_lifecycle; a source counts as involved only if
# it did more than EXTINGUISH_CREDIT_MIN_SHARE of the total work, so a truck
# that arrived for the last half second is not called a joint effort.
EXTINGUISH_CREDIT_MIN_SHARE = 0.05
EXTINGUISH_BY_GROUND = "trucks"
EXTINGUISH_BY_WATER = "drones"
EXTINGUISH_BY_BOTH = "both"


def classify_extinguish_source(ground_work_seconds, water_work_seconds):
    """Return (label, ground_share) for a fire that has just gone out."""
    ground_work = max(0.0, ground_work_seconds)
    water_work = max(0.0, water_work_seconds)
    total_work = ground_work + water_work
    if total_work <= 0.0:
        return EXTINGUISH_BY_GROUND, 1.0
    ground_share = ground_work / total_work
    if ground_share >= 1.0 - EXTINGUISH_CREDIT_MIN_SHARE:
        return EXTINGUISH_BY_GROUND, 1.0
    if ground_share <= EXTINGUISH_CREDIT_MIN_SHARE:
        return EXTINGUISH_BY_WATER, 0.0
    return EXTINGUISH_BY_BOTH, ground_share


def get_extinguish_credit_summary():
    """Round totals for the report: counts per source plus the shared split."""
    out_hotspots = [
        hotspot
        for hotspot in fire_hotspots
        if hotspot.suppression_state == FIRE_STATE_OUT
    ]
    summary = {
        "out_total": len(out_hotspots),
        "trucks_only": 0,
        "drones_only": 0,
        "shared": 0,
        "shared_ground_pct": 0.0,
        "shared_water_pct": 0.0,
        "shared_splits": [],
    }
    shared_ground_shares = []
    for hotspot in out_hotspots:
        label = hotspot.extinguished_by
        if label is None:
            label, hotspot.extinguish_ground_share = classify_extinguish_source(
                hotspot.ground_suppression_work_seconds,
                hotspot.water_suppression_work_seconds,
            )
            hotspot.extinguished_by = label
        if label == EXTINGUISH_BY_GROUND:
            summary["trucks_only"] += 1
        elif label == EXTINGUISH_BY_WATER:
            summary["drones_only"] += 1
        else:
            summary["shared"] += 1
            shared_ground_shares.append(hotspot.extinguish_ground_share)
            summary["shared_splits"].append(
                (
                    round(hotspot.extinguish_ground_share * 100.0),
                    round((1.0 - hotspot.extinguish_ground_share) * 100.0),
                )
            )
    if shared_ground_shares:
        average_ground_share = sum(shared_ground_shares) / len(shared_ground_shares)
        summary["shared_ground_pct"] = average_ground_share * 100.0
        summary["shared_water_pct"] = (1.0 - average_ground_share) * 100.0
    return summary


def compose_extinguish_credit_lines():
    summary = get_extinguish_credit_summary()
    lines = ["FIRES PUT OUT: %d total" % summary["out_total"]]
    if summary["out_total"] == 0:
        lines.append("  (no fire was put out this round)")
        return lines
    lines.append("  by fire trucks only: %d" % summary["trucks_only"])
    lines.append("  by drones only: %d" % summary["drones_only"])
    if summary["shared"]:
        lines.append(
            "  by both: %d (average %.0f%% truck / %.0f%% drone)"
            % (
                summary["shared"],
                summary["shared_ground_pct"],
                summary["shared_water_pct"],
            )
        )
        splits = ", ".join(
            "%d%%/%d%%" % (ground_pct, water_pct)
            for ground_pct, water_pct in summary["shared_splits"][:8]
        )
        lines.append("    truck/drone per shared fire: " + splits)
    else:
        lines.append("  by both: 0")
    return lines


# --- Plain-language formula sheet for the exported report --------------------
# Cheryl (July 28): the exported file has to explain every matrix variable and
# how it is calculated, with the full formula, not just the final number. This
# builds that text from the LIVE constants, so the sheet can never drift out of
# date the way a hand-written document would. dt below is the length of one
# simulation step in simulated seconds.
def compose_metric_formula_lines():
    minutes_per_second = FIRE_SIM_REAL_MINUTES_PER_SECOND
    ground_rate = current_ground_suppression_rate_per_second()
    water_rate = current_water_suppression_rate_per_second()
    contain_delay = FIRE_SUPPRESSION_CONTAIN_DELAY_SECONDS
    extinguish_delay = FIRE_SUPPRESSION_EXTINGUISH_DELAY_SECONDS
    full_work = contain_delay + extinguish_delay
    detection_target = PERFORMANCE_DETECTION_TARGET_SECONDS
    survey_count = get_team_survey_drone_count()
    water_count = team_water_drone_count

    lines = []

    def block(title, rows):
        lines.append(title)
        lines.extend(("  " + row) if row else "" for row in rows)
        lines.append("")

    block("0. TWO FAMILIES OF METRIC", (
        "P1-P4 (sections 6-9) score the WILDFIRE OUTCOME: how fast fires were",
        "found, mapped, worked and put out. They grade the round.",
        "M1-M4 (sections 13-16) are the behavioural TRUST metrics from Darya's",
        "single-drone study, measured per drone per run: path length ratio,",
        "obstacle clearance, mode usage and Hausdorff distance to the plan.",
        "Before July 29, 2026 the outcome metrics were also called M1-M4.",
        "The 100-POINT SCORE (section 17) is neither: it is the collaboration",
        "score, a fixed base for the responders that worked plus what the",
        "operator added or cost.",
    ))

    block("1. TIME BASE", (
        "1 simulated second = %.0f real minutes, so the whole timeline is compressed."
        % minutes_per_second,
        "Round length = %.0f simulated seconds (%.0f real hours of incident time)."
        % (round_duration_seconds, round_duration_seconds * minutes_per_second / 60.0),
        "dt = length of one simulation step, in simulated seconds.",
        "Normal play advances one step per frame with dt <= %.2f s."
        % SIMULATION_MAX_FRAME_DT_SECONDS,
        "Fast-forward profile: sim time per frame = frame_dt x %.0f, split into"
        % SIMULATION_TIME_SCALE,
        "steps of at most %.2f s. Every formula below is unchanged by the scale."
        % SIMULATION_MAX_SUBSTEP_SECONDS,
    ))

    block("2. FIRE DETECTION (feeds P1 and the detection rate)", (
        "A survey drone can only see a fire within %.0f m horizontally and above"
        % FIRE_HOTSPOT_DETECTION_RADIUS_METERS,
        "%.1f m altitude. Each step it rolls once against a Poisson probability:"
        % FIRE_HOTSPOT_MIN_SCAN_ALTITUDE_METERS,
        "",
        "  r = R0 x team_scale x range_factor x altitude_factor      [per second]",
        "  P(detect during this step) = 1 - exp(-r x dt)             capped at 0.95",
        "",
        "  R0 = %.2f per second (sensor base rate)"
        % FIRE_HOTSPOT_DETECTION_PROBABILITY_PER_SECOND,
        "  team_scale = 1.0 with 3 or more survey drones, else %.2f"
        % SMALL_TEAM_DETECTION_PROBABILITY_SCALE,
        "  range_factor = 1 - horizontal_distance / %.0f      (0 at the ring edge)"
        % FIRE_HOTSPOT_DETECTION_RADIUS_METERS,
        "  altitude_factor = clamp(altitude / %.2f, 0.25, 1.0)"
        % (FIRE_HOTSPOT_MIN_SCAN_ALTITUDE_METERS * 2.5),
        "",
        "The exponential form makes the rate per SECOND correct for any dt, so a",
        "fast-forwarded run detects at the same rate as a normal one.",
    ))

    block("3. SUPPRESSION RATE (the core of P3 and P4)", (
        "Every fire carries an accumulated work value, in work-seconds:",
        "",
        "  S(t) = G x [a fire truck is engaged] + W x n_water_drones   [work per second]",
        "  work = work + S(t) x dt                                     [each step]",
        "",
        "  G = %.3f per second   ground crew rate (one engaged truck)" % ground_rate,
        "  W = %.3f per second   water drone assist rate, per engaged drone"
        % water_rate,
        "  n_water_drones = how many water drones are spraying that fire",
        "",
        "State thresholds:",
        "  work >= %.1f            -> CONTAINED  (= %.0f real minutes of work)"
        % (contain_delay, contain_delay * minutes_per_second),
        "  work >= %.1f            -> OUT        (= %.0f real hours of work)"
        % (full_work, full_work * minutes_per_second / 60.0),
        "",
        "  A truck starts working %.0f s after the fire is detected (travel and setup)."
        % GROUND_FIREFIGHTER_RESPONSE_DELAY_SECONDS,
        "  An ACTIVE fire nobody is working burns out after %.0f s and is lost."
        % current_undetected_burnout_delay_seconds(),
        "  Reignition after OUT: P = 1 - exp(-%.4f x dt) per step."
        % current_reignite_probability_per_second(),
        "  A fire being worked stops spreading (containment as a race), except for",
        "  a %.0f%% chance per spread tick that a worked spot still throws a spot fire."
        % (FIRE_WORKED_SPOT_SPREAD_PROBABILITY * 100.0),
    ))

    block("4. WHO PUT THE FIRE OUT", (
        "The same work is booked to whoever produced it:",
        "",
        "  truck_work = truck_work + G x dt        drone_work = drone_work + W x n x dt",
        "  truck_share = truck_work / (truck_work + drone_work)",
        "",
        "  truck_share >= %.2f  -> fire trucks" % (1.0 - EXTINGUISH_CREDIT_MIN_SHARE),
        "  truck_share <= %.2f  -> drones" % EXTINGUISH_CREDIT_MIN_SHARE,
        "  anything between      -> both, reported as truck%% / drone%%",
        "",
        "The split is frozen the moment the fire reaches OUT and is cleared if it",
        "reignites, so the counts always add up to the fires that are out.",
    ))

    block("5. TEAM SIZE (no longer scales any points)", (
        "Until July 29, 2026 every metric was multiplied by a team-size factor",
        "E(n), and a bigger team also got a bonus. Both are REMOVED: they moved",
        "the achievable ceiling with team size, so the same round scored",
        "differently for 2, 4 and 6 drones and the numbers were not comparable.",
        "",
        "  this round: %d survey drones, %d water drones, no score scaling"
        % (survey_count, water_count),
        "",
        "Fairness across team sizes is handled in the fire instead: a bigger",
        "team gets a faster fire (FIRE_TEAM_SIZE_SPREAD_RATE_MULTIPLIERS).",
    ))

    block("6. P1 DETECTION SPEED (weight %.0f%%)" % (PERFORMANCE_WEIGHT_P1 * 100.0), (
        "  delay_i     = detection_time_i - ignition_time_i   (found fires)",
        "              = now - ignition_time_i                (fires still not found)",
        "  score_i     = 1 - clamp(delay_i / %.1f, 0, 1)" % detection_target,
        "  P1          = mean(score_i over EVERY fire)",
        "",
        "  %.1f s is the target: %.0f real minutes, the initial-attack window."
        % (detection_target, detection_target * minutes_per_second),
        "  A fire that is never found keeps pulling P1 down as the round runs.",
    ))

    block("7. P2 MAPPING COVERAGE (weight %.0f%%)" % (PERFORMANCE_WEIGHT_P2 * 100.0), (
        "  mapped_i = min(1, time_survey_spent_over_fire_i / %.1f)"
        % SURVEY_MAP_BUILD_TIME_SECONDS,
        "  P2       = mean(mapped_i over every fire)",
        "",
        "  Finding a fire is not the same as mapping it: the survey drone has to",
        "  stay over the spot for %.1f s to build a full picture of it."
        % SURVEY_MAP_BUILD_TIME_SECONDS,
    ))

    block("8. P3 SUPPRESSION PROGRESS (weight %.0f%%)" % (PERFORMANCE_WEIGHT_P3 * 100.0), (
        "  Over DETECTED fires only:",
        "    OUT        -> 1.0",
        "    BURNED     -> 0.0",
        "    CONTAINED  -> 0.5 + 0.5 x clamp((work - %.1f) / %.1f, 0, 1)"
        % (contain_delay, extinguish_delay),
        "    ACTIVE     -> 0.5 x clamp(work / %.1f, 0, 1), but at least %.2f"
        % (contain_delay, PERFORMANCE_FIREFIGHTER_SUPPRESSION_BASE_SCORE),
        "                  once a truck is engaged (help is on the fire)",
        "  P3 = mean(those values)",
    ))

    block("9. P4 FIRE PUT OUT (weight %.0f%%)" % (PERFORMANCE_WEIGHT_P4 * 100.0), (
        "  Over ALL fires, found or not:",
        "    OUT        -> 1.0",
        "    CONTAINED  -> %.2f + %.2f x clamp((work - %.1f) / %.1f, 0, 1)"
        % (
            PERFORMANCE_CONTAINED_FIRE_CONTROL_CREDIT,
            1.0 - PERFORMANCE_CONTAINED_FIRE_CONTROL_CREDIT,
            contain_delay,
            extinguish_delay,
        ),
        "    ACTIVE and detected -> clamp(work / %.1f, 0, 0.35)" % full_work,
        "    anything else       -> 0.0",
        "  P4 = mean(those values)",
        "",
        "  P3 asks how well the fires you found are being handled. P4 asks how much",
        "  of the whole incident is actually under control.",
    ))

    block("10. COMPOSITE SCORE", (
        "  raw = (%.2f x P1 + %.2f x P2 + %.2f x P3 + %.2f x P4) / %.2f"
        % (
            PERFORMANCE_WEIGHT_P1,
            PERFORMANCE_WEIGHT_P2,
            PERFORMANCE_WEIGHT_P3,
            PERFORMANCE_WEIGHT_P4,
            PERFORMANCE_WEIGHT_P1
            + PERFORMANCE_WEIGHT_P2
            + PERFORMANCE_WEIGHT_P3
            + PERFORMANCE_WEIGHT_P4,
        ),
        "  composite = clamp(raw, 0, 1)      no team-size bonus since July 29",
        "  the composite measures the OUTCOME of the round. Whether the run",
        "  counts as a success is judged on the collaboration score below."
    ))

    block("11. THE 100-POINT SCORE", (
        "The 100-point score is no longer a list of outcome deductions. It is",
        "the COLLABORATION SCORE, defined in section 17 below: the base is the",
        "P1-P4 mission performance above (out of 100) plus what the operator",
        "added or cost. Aug 9, 2026 the fixed 40/67 water tier was dropped so",
        "the base and the P1-P4 score overlap.",
        "",
        "  burn ratio (reported, not deducted) = burned cells / burnable cells",
        "  on the %d x %d burn grid, each cell %.1f m across."
        % (FIRE_BURN_GRID_WIDTH, FIRE_BURN_GRID_HEIGHT, FIRE_BURN_CELL_SIZE_METERS),
    ))

    # The behavioural trust matrix documents itself in 04c_trust_metrics.py so
    # the two families of metric stay clearly separated in the sheet.
    lines.append("")
    lines.extend(compose_trust_metric_formula_lines())
    lines.append("")
    lines.extend(compose_collaboration_formula_lines())

    while lines and lines[-1] == "":
        lines.pop()
    return lines
