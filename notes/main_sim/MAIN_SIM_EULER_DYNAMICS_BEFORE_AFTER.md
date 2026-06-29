# Main Sim Before/After Using Exact Paper-to-Code Comparison

## Scope
- This note is for `main.py` only.
- `case1.py` and `case2.py` were intentionally left unchanged.

## What Changed In How This Note Is Written
- This note now follows the exact equation-by-equation comparison style.
- Instead of only saying the sim is "paper-inspired", it now separates:
- direct matches
- partial or simplified matches
- equations that are not implemented exactly as written

## Short Symbol Map
- `φ` = roll = `motion_state.roll_radians`
- `θ` = pitch = `motion_state.pitch_radians`
- `ψ` = yaw = `motion_state.yaw_radians`
- `u₁` = collective thrust
- `u₂` = pitch torque
- `u₃` = roll torque
- `u₄` = yaw torque

## Before
- The older main sim behaved more like a game movement model.
- Velocity was pushed toward target velocity using rate-limited smoothing.
- Heading turned toward a target heading.
- Roll and pitch were mostly visual banking, not the real source of acceleration.

## After
- The current main sim uses real stored Euler-angle states.
- Roll, pitch, and yaw are updated through torque and angular acceleration.
- Horizontal acceleration now comes from thrust being redirected by roll and pitch.
- Rotor-level thrust mixing and motor lag are included.

## Exact Paper-to-Code Summary

### Direct Matches
- Eq. (2): world translational thrust projection
  - code: `main.py:3548-3554`
- Eq. (8): attitude dynamics
  - code: `main.py:3510-3520`
- Eq. (12): rotor thrust mixing
  - code: `main.py:2304-2337`
- Eq. (21): yaw acceleration
  - code: `main.py:3520`
- Eq. (33), bounded-guidance part
  - code: `main.py:2292-2293`, `main.py:3443-3448`

### Partial or Simplified Matches
- Eq. (14): motor model
  - paper uses a fuller DC motor transfer function
  - sim uses first-order rotor lag
- Eq. (16) and Eq. (19): linearized hover translation channels
  - sim uses the same small-angle physics idea
  - sim does not explicitly build the `A x + B u` matrices
- Eq. (32) and Eq. (34): altitude channel
  - sim uses the same bounded hover idea
  - sim simplifies the altitude PD structure
- Eq. (40): position-attitude kinematics
  - same physical idea is present

### Not Implemented Exactly As Written
- Eq. (9): full Euler/body-rate mapping
- Eq. (22): full linearized state-space model
- Eq. (35): closed-loop altitude matrix
- Eq. (36) to Eq. (38): paper yaw guidance and yaw closed-loop system

## Biggest Scientific Differences
- The sim uses the small-angle assumption from Eq. (10) instead of the full Eq. (9) transformation.
- The motor model is simplified compared to Eq. (14).
- The yaw controller in the sim is not the same controller used in Eq. (36) to Eq. (38).
- The sim also keeps game-specific damping and stop-settle logic that are not from the paper.

## Most Important Practical Result
- Before: the drone movement mostly decided the visual tilt.
- After: the attitude equations decide the tilt, and the tilt physically creates the movement.
- That is why opposite tilt while braking now makes sense mathematically instead of just looking like a visual trick.

## Detailed Comparison
- The detailed line-by-line paper vs code comparison is now in:
- `/Users/rhyls/Desktop/Summer26 FSC/human trust w auto sys/real sim/notes/main_sim/MAIN_SIM_EQUATIONS_AND_CODE.md`
- `/Users/rhyls/Desktop/Summer26 FSC/human trust w auto sys/real sim/notes/main_sim/MAIN_SIM_EQUATIONS_AND_CODE.txt`
