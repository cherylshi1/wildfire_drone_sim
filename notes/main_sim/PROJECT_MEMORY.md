# Project Memory

Last updated: 2026-06-16

Use this file as the persistent memory for the real-sim project. Update it whenever project behavior, assumptions, or calibration targets change.

## Current Simulation Intent

- Main working entry point: `main.py` in `human trust w auto sys/real sim`.
- Mixed manual/auto teams are expected. A drone can be `MANUAL`, `AUTO holding`, or `AUTO + DISPATCHED`.
- Water dispatch keys are fixed by slot: `[P]` = W1, `[L]` = W2, `[K]` = W3. Pressing a dispatch key should put that water drone in automation and dispatch it to the currently detected fire.
- Detected fire must be visually obvious: undetected fire is deep red; detected fire is intentionally cyan/blue-white with a large cyan marker/halo, even if it looks less natural, because the experiment needs instant state recognition.
- In automatic mode, the camera should remain centered on the selected drone's current flight direction. Operator camera drag is allowed only inside a bounded look cone around that direction, not as a free detached orbit.
- Current automated-camera look cone: `AUTOMATION_CAMERA_LOOK_CONE_HALF_ANGLE_DEGREES = 55.0`.
- Automated camera heading must be stable in both first-person and third-person: prefer the drone's commanded automation vector, ignore tiny velocity jitter, use the drone body heading while hovering, and snap to the selected drone's heading on view/mode switches so the camera does not spin through a large arc.
- Camera vertical range should allow looking almost straight below the drone: first-person min pitch is `-88.0`, third-person min pitch is `-72.0`.
- Dispatch popup should appear only once per round. It may update while visible, but later detections should not re-open it.
- The fire map needs a color legend directly under the map view.
- If any survey drone detects a hotspot in its own sector, it should switch into the after-fire orbit/mapping pattern for that hotspot even if another survey drone first detected the larger fire event.

## Calibration Target

- Six-drone full-auto case means 3 survey drones and 3 water drones.
- Calibration goal: a clean 3S/3W full-auto run should detect about 80% of fire hotspots and extinguish/control about 65% by the end of the 3-minute round.
- Current named targets in `main.py`:
  - `SIX_DRONE_FULL_AUTO_TARGET_DETECTION_RATE = 0.80`
  - `SIX_DRONE_FULL_AUTO_TARGET_EXTINGUISH_RATE = 0.65`
- Fire spread and suppression changes should be checked against that target before being considered final.
- Fire spread pacing target: a no-suppression run should visually fill the map around `2:45`, not around `1:00`. Current spread uses `FIRE_FULL_MAP_TARGET_SECONDS = 160.0` as an internal burn-budget cap; measured full burn was about `2:49`, with about `37%` burned at `1:00`.
- The smaller 2-survey case should not easily exceed 70% detection. Current small-team dampeners in `main.py`:
  - `SMALL_TEAM_DETECTION_PROBABILITY_SCALE = 0.58`
  - `SMALL_TEAM_FORCED_DETECTION_RADIUS_SCALE = 0.55`

## Research Context

- Recent Canada-specific wildfire/UAV research does not support small teams getting near-perfect perception:
  - 2026 SysCon / arXiv 2601.11794: Saskatchewan prescribed-burn UAV sensing uses only about 2.2 hours of flight data and highlights low-cost sensor drift, cross-sensitivity, response lag, and data scarcity.
  - 2025 CanadaFireSat / arXiv 2506.08690: high-resolution wildfire forecasting across Canada reports a 60.3% peak F1 score on the unseen 2023 wildfire season.
  - 2024 arXiv 2408.10843: real-time UAV wildfire/smoke segmentation reports 63.3% mIoU and emphasizes onboard-compute and smoke-localization constraints.
- Recent FoV-limited drone and angle-aware coverage papers treat camera orientation as a controlled/constrained variable during autonomy, not an arbitrary free orbit. Keep automated camera yaw bounded around the selected drone's flight direction.
- Recent Canadian wildfire context is severe and changing, so fire spread/suppression pacing should remain challenging rather than overly easy for small teams.
