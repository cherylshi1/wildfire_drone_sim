# Main Sim Paper Equations vs Current Code

## Scope
- This note compares the equations in `/Users/rhyls/Desktop/fsc.pdf` against the current `main.py`.
- Scope is `main.py` only.
- `case1.py` and `case2.py` are not part of this document.

## Important Note About The PDF
- The PDF text extraction damaged some Greek letters and some matrix formatting.
- I rewrote the equations below in clean readable notation using the actual symbols.
- For Eq. (35) and Eq. (38), the raw extraction of the closed-loop matrices is partially broken, so those two are described carefully instead of pretending the PDF extraction was perfect.

## Symbol Map
- `φ` = roll = `motion_state.roll_radians`
- `θ` = pitch = `motion_state.pitch_radians`
- `ψ` = yaw = `motion_state.yaw_radians`
- `φ̇` = `motion_state.roll_rate_rad_per_second`
- `θ̇` = `motion_state.pitch_rate_rad_per_second`
- `ψ̇` = `motion_state.yaw_rate_rad_per_second`
- `φ̈` = roll angular acceleration
- `θ̈` = pitch angular acceleration
- `ψ̈` = yaw angular acceleration
- `u₁` = collective thrust
- `u₂` = pitch torque
- `u₃` = roll torque
- `u₄` = yaw torque
- `u₀₁` = vertical-channel control input from the paper
- `u₀₂` = pitch-channel linearized control input from the paper
- `u₀₃` = roll-channel linearized control input from the paper
- `u₀₄` = yaw-channel control input from the paper

## Variables Used In This Note
- `x, y, z` = world position
- `ẋ, ẏ, ż` = world velocity
- `ẍ, ÿ, z̈` = world acceleration
- `φ, θ, ψ` = Euler angles
- `φ̇, θ̇, ψ̇` = Euler-angle rates
- `φ̈, θ̈, ψ̈` = Euler-angle accelerations
- `p, q, r` = body rates used in the paper
- `T₁, T₂, T₃, T₄` = rotor thrusts
- `ωᵢ` = rotor angular speed
- `m` = mass
- `g` = gravity
- `Iₓ, Iᵧ, I_z` = principal inertias
- `L` = arm length
- `k_T` = thrust coefficient
- `k_r` = yaw-moment ratio in the sim
- `γ_z, γ_ψ` = paper guidance gains
- `P_z, D_z` = paper altitude gains
- `P_ψ, D_ψ` = paper yaw gains

## Actual Current Constants Used By The Sim
- `CONTROL_INPUT_LAG_SECONDS = 0.0`
- `g = 9.81 m/s²`
- `m = 1.6 kg`
- `Iₓ = 0.045 kg·m²`
- `Iᵧ = 0.052 kg·m²`
- `I_z = 0.085 kg·m²`
- `L = 0.28 m`
- `k_T = 2.2 × 10⁻⁵`
- `k_r = 0.018`
- `τ_motor = 0.075 s`
- `T_hover,per rotor = 3.924 N`
- `ω_hover,per rotor ≈ 422.331 rad/s`
- `a_xy,max = 8.4 m/s²`
- `a_z,max = 5.2 m/s²`
- `k_h = 4.2`
- `k_v = 3.4`
- `k_xy = 3.7`
- `d_xy,base = 0.22 s⁻¹`
- `d_xy,brake = 1.9 s⁻¹`
- `v_xy,settle = 0.24 m/s`
- `blend_manual = 0.0`
- `blend_auto = 0.14`
- `k_p,θ = 9.5`
- `k_d,θ = 4.8`
- `k_p,φ = 9.5`
- `k_d,φ = 4.8`
- `k_p,ψ = 6.0`
- `k_d,ψ = 3.0`
- `τ_θ,max = 0.42 N·m`
- `τ_φ,max = 0.42 N·m`
- `τ_ψ,max = 0.18 N·m`
- `u₁,max = 37.6704 N`
- `u₁,min = 5.4936 N`
- `φ_max = 10°`
- `θ_max = 7.5°`
- `heading sign x = -1.0`
- `heading sign y = 1.0`

## Why These Values Were Chosen
- Not all of these numbers came from the paper.
- Some are physical modeling choices.
- Some are control tuning choices.
- Some are just gameplay and stability limits.

### Physical-model choices
- `g = 9.81 m/s²`
  - This is standard gravity.
- `m = 1.6 kg`
  - Chosen to make the drone feel like a medium quadrotor, not a tiny toy drone and not a heavy industrial one.
- `Iₓ = 0.045`, `Iᵧ = 0.052`, `I_z = 0.085`
  - Chosen so yaw is harder to spin than roll or pitch, which is normal for a quadrotor.
  - `I_z` is larger because spinning around the vertical axis usually feels more resistant.
- `L = 0.28 m`
  - Chosen as a medium arm length so torque from rotor thrust is believable without making the drone huge.
- `k_T = 2.2 × 10⁻⁵`
  - Chosen so the hover rotor speed comes out in a believable range instead of needing absurdly high or absurdly low rotor RPM.
- `k_r = 0.018`
  - Chosen so yaw exists, but stays weaker than the main roll/pitch channels.

### Derived values
- `T_hover,per rotor = 3.924 N`
  - Not hand-picked directly.
  - This comes from `mg / 4`, so each rotor carries one quarter of the weight at hover.
- `ω_hover,per rotor ≈ 422.331 rad/s`
  - Also not hand-picked directly.
  - This comes from the hover thrust and the thrust coefficient `k_T`.
- `u₁,max = 37.6704 N`
  - Chosen as `2.4 × mg`, which gives enough extra thrust above hover for climbing and recovery.
- `u₁,min = 5.4936 N`
  - Chosen as `0.35 × mg`, so the drone can reduce thrust a lot without fully killing the control authority.

### Actuator feel
- `τ_motor = 0.075 s`
  - Chosen to make the drone feel heavier and less twitchy.
  - Bigger `τ_motor` means more laggy prop response.
  - Smaller `τ_motor` would feel snappier but more arcade-like.

### Motion and controller limits
- `a_xy,max = 8.4 m/s²`
  - Chosen so horizontal response is strong enough to stop drift, but not so strong that the drone snaps unnaturally.
- `a_z,max = 5.2 m/s²`
  - Chosen lower than horizontal so vertical motion stays controlled and does not bounce too hard.
- `φ_max = 10°`
  - Chosen to keep roll visually believable and to prevent huge side-leaning.
- `θ_max = 7.5°`
  - Chosen even smaller than roll so forward/back motion stays controlled and less aggressive.
- `τ_θ,max = 0.42 N·m`
- `τ_φ,max = 0.42 N·m`
- `τ_ψ,max = 0.18 N·m`
  - Chosen so yaw remains weaker than pitch and roll, and pitch/roll remain responsive without becoming unstable.

### Guidance and feedback gains
- `k_h = 4.2`
  - This is the altitude guidance aggressiveness.
  - Chosen so altitude errors are corrected quickly without feeling too jumpy.
- `k_v = 3.4`
  - This is the vertical rate correction gain.
  - Chosen to support hover stability after the bounded-guidance step.
- `k_xy = 3.7`
  - Chosen so horizontal speed errors are corrected strongly enough to reduce drift.
- `k_p,θ = 9.5`, `k_d,θ = 4.8`
- `k_p,φ = 9.5`, `k_d,φ = 4.8`
  - Chosen as balanced PD gains so pitch and roll track target angles quickly without obvious oscillation.
- `k_p,ψ = 6.0`, `k_d,ψ = 3.0`
  - Chosen lower than pitch/roll because yaw should feel steadier and less twitchy.

### Gameplay damping and stop behavior
- `d_xy,base = 0.22 s⁻¹`
  - Small always-on horizontal damping so the drone does not feel frictionless.
- `d_xy,brake = 1.9 s⁻¹`
  - Extra damping when you release the stick, to reduce long sliding stops.
- `v_xy,settle = 0.24 m/s`
  - If speed gets below this while braking, the sim snaps the residual drift to zero.
  - This is a gameplay choice, not a paper value.
- `blend_manual = 0.0`
  - Chosen so wind does not secretly bias manual movement commands.
- `blend_auto = 0.14`
  - Chosen so automation can feel a little more influenced by wind without losing control.

### Sign conventions
- `heading sign x = -1.0`
- `heading sign y = 1.0`
  - These are not physics tunings.
  - They are coordinate-convention fixes so heading math lines up with the world axes and model orientation.

## What I Mean By Direct, Very Close, And Partial
- `Direct match` means the code is using the same mathematical form as the paper, with only naming, ordering, or bookkeeping differences.
- `Very close` means the code is using the same core equation, but there are a few implementation differences such as clamping, using post-lag thrust instead of ideal thrust, or integrating stored Euler rates directly.
- `Partial or simplified` means the code is using the same physics idea, but not the same equation structure as the paper.

## Exact Paper Equations vs Current Code

### Eq. (2): Translational Dynamics
Paper, rewritten in clean notation:

```text
ẍ = (u₁ / m) [cos(ψ) sin(φ) + sin(ψ) sin(θ) cos(φ)]
ÿ = (u₁ / m) [sin(ψ) sin(φ) - cos(ψ) sin(θ) cos(φ)]
z̈ = (u₁ / m) cos(θ) cos(φ) - g
```

Code:
- `main.py:3548-3554`

Comparison:
- This is a direct match.
- `world_accel_x`, `world_accel_y`, and `world_accel_z` use this same thrust-projection structure.
- Differences:
- the code uses the actual post-lag collective thrust `collective_thrust_newtons / m`, not the ideal pre-lag `u₁ / m`
- extra gameplay damping is applied immediately after this acceleration is computed

### Eq. (8): Attitude Dynamics
Paper, rewritten in clean notation:

```text
θ̈ = ((Iᵧ - I_z) / Iₓ) φ̇ ψ̇ + u₂ / Iₓ
φ̈ = ((I_z - Iₓ) / Iᵧ) θ̇ ψ̇ + u₃ / Iᵧ
ψ̈ = u₄ / I_z
```

Code:
- `main.py:3510-3520`

Comparison:
- This is very close.
- `pitch_accel`, `roll_accel`, and `yaw_accel` are the implemented angular accelerations.
- What is different:
- the code uses actual torques reconstructed after rotor lag, not the ideal control torques before lag
- the code stores Euler rates directly instead of separately storing `p, q, r`
- the code clamps `φ` and `θ` after integration to `±18°` and `±12°`

### Eq. (9): Euler-Rate to Body-Rate Relationship
Paper, rewritten in clean notation:

```text
[φ̇, θ̇, ψ̇]^T = S(φ, θ) [p, q, r]^T
```

Code:
- No exact direct implementation.

Comparison:
- The sim does not explicitly store separate body rates `p, q, r`.
- Instead, it stores Euler-angle rates directly in `DroneMotionState`.
- So this exact matrix relationship is not coded as written.

### Eq. (10): Small-Angle Approximation
Paper, rewritten in clean notation:

```text
φ̇ ≈ p
θ̇ ≈ q
ψ̇ ≈ r
```

Code:
- `main.py:3522-3538`

Comparison:
- This is the approximation the current sim is effectively using.
- What is different:
- the paper presents this as an approximation step
- the code effectively treats this approximation as the runtime model for the Euler-angle rates

### Eq. (11): Full Nonlinear State Model
Paper, rewritten in clean notation:

```text
ẋ = f(x, u)
```

Code:
- `main.py:3443-3601`

Comparison:
- The sim does not implement Eq. (11) as one big state-space block.
- Instead, it implements the same model in pieces:
- altitude and thrust command
- torque command
- rotor lag
- attitude acceleration
- world-frame translational acceleration
- velocity and position integration

### Eq. (12): Rotor Thrust Mixing
Paper, rewritten in clean notation:

```text
[u₁, u₂, u₃, u₄]^T = M [T₁, T₂, T₃, T₄]^T
Tᵢ = k_T ωᵢ²
```

Code:
- `main.py:2304-2326`
- `main.py:2329-2337`

Comparison:
- This is a structural match.
- What is different:
- the paper writes the mixing as a matrix equation
- the code expands the same logic manually into front/back/left/right expressions
- the exact rotor order and sign convention depend on the sim's axis labeling
- the sim uses `k_r = 0.018` for the yaw-moment ratio

### Eq. (14): DC Motor Transfer Function
Paper:

```text
Ω(s) / Vₐ(s) = kₜ / ((L s + R)(J s + B) + kₑ kₜ)
```

Code:
- `main.py:2340-2361`

Comparison:
- This is a simplified match, not an exact one.
- What is different:
- the paper motor model depends on `L, R, J, B, kₑ, kₜ`
- the code collapses all of that into a first-order lag with `τ_motor = 0.03 s`

### Eq. (16): Linearized Pitch-Channel Model
Paper, rewritten in clean notation:

```text
ÿ ≈ -g θ
z̈ ≈ u₁ / m - g
θ̈ ≈ u₂ / Iₓ
```

Code:
- `main.py:3420-3424`
- `main.py:3548-3554`

Comparison:
- This is a partial but close conceptual match.
- What is different:
- the paper is a linearized state-space hover model
- the code uses the same small-angle relationship inside a nonlinear controller loop
- the code clamps both desired and actual pitch angles
- in the controller, the same idea appears as `desired_pitch_radians = -desired_body_forward_accel / g`

### Eq. (19): Linearized Roll-Channel Model
Paper, rewritten in clean notation:

```text
ẍ ≈ g φ
z̈ ≈ u₁ / m - g
φ̈ ≈ u₃ / Iᵧ
```

Code:
- `main.py:3415-3419`
- `main.py:3548-3554`

Comparison:
- This is a partial but close conceptual match.
- What is different:
- the paper is a linearized state-space hover model
- the code uses the same small-angle relationship inside a nonlinear controller loop
- the code clamps both desired and actual roll angles
- in the controller, the same idea appears as `desired_roll_radians = desired_body_right_accel / g`

### Eq. (21): Yaw Dynamics
Paper:

```text
ψ̈ = u₄ / I_z
```

Code:
- `main.py:3520`

Comparison:
- This is basically exact.
- What is different:
- the sim uses the actual post-lag yaw torque recovered from rotor thrusts
- the code wraps `ψ` after integration so heading stays bounded

### Eq. (22): Full Linearized Quadrotor Model
Paper:

```text
ẋ = A x + B u
```

Code:
- No direct `A`, `B` matrix implementation.

Comparison:
- The sim uses the same channel logic, but not in one combined matrix form.

### Eq. (32): Hovering-Mode Local Dynamics
Paper:

```text
z̈ = u₀₁
```

Code:
- `main.py:3449-3453`
- `main.py:3554`

Comparison:
- This is a partial match.
- What is different:
- the paper writes the vertical channel as a local double integrator
- the code turns that vertical command into collective thrust
- the actual final `z̈` still depends on thrust projection through `cos(θ) cos(φ)`

### Eq. (33): Bounded Guidance Law
Paper, rewritten in clean notation:

```text
ż_d = -γ_z [z̃ / (1 + |z̃|)]
h(x) = x / (1 + |x|)
```

Code:
- `main.py:2292-2293`
- `main.py:3443-3448`

Comparison:
- This is one of the strongest direct matches in the sim.
- What is different:
- the paper writes it as a guidance law using `γ_z`
- the code uses `DRONE_HOVER_GUIDANCE_GAIN = 4.2`
- the code then clamps the resulting `ż_d`

### Eq. (34): Altitude PD Channel
Paper, rewritten in clean notation:

```text
u₀₁ = P_z (ż_d - ż) + D_z (z̈_d - z̈)
```

Code:
- `main.py:3449-3453`

Comparison:
- This is simplified compared to the paper.
- The code keeps the velocity-error part:

```text
a_z,des = k_v (ż_d - ż)
```

- What is different:
- the paper includes an extra acceleration-error term
- the code does not explicitly implement that extra term
- the code clamps the resulting desired vertical acceleration to `±5.2 m/s²`

### Eq. (35): Closed-Loop Altitude System
Paper:
- closed-loop matrix form derived from Eq. (32) to Eq. (34)

Code:
- No direct matrix form is coded.

Comparison:
- The raw PDF extraction damaged the exact matrix layout.
- The sim implements controller behavior that creates a related closed loop, but it does not explicitly code Eq. (35) as a matrix system.

### Eq. (36): Yaw Guidance Law
Paper, rewritten in clean notation:

```text
ψ̇_d = -γ_ψ (ψ_T - ψ_A)
```

Code:
- `main.py:3426-3441`

Comparison:
- This is not an exact match.
- What is different:
- the paper commands a desired yaw rate `ψ̇_d`
- in manual mode, the code mostly commands a desired yaw angle from `camera_angle`
- in automation mode, yaw follows command direction or velocity direction instead

### Eq. (37): Yaw PD Channel
Paper, rewritten in clean notation:

```text
u₀₄ = P_ψ (ψ̇_d - ψ̇_A) + D_ψ d/dt (ψ̇_d - ψ̇_A)
```

Code:
- `main.py:3485-3494`

Comparison:
- This is not an exact match.
- The code uses PD on yaw angle error and yaw rate:

```text
u₄ = I_z [k_p,ψ (ψ_des - ψ) - k_d,ψ ψ̇]
```

- What is different:
- the paper is PD on yaw-rate tracking
- the code is PD on yaw-angle error plus yaw-rate damping

### Eq. (38): Closed-Loop Yaw System
Paper:
- closed-loop yaw matrix derived from Eq. (36) and Eq. (37)

Code:
- No direct matrix form is coded.

Comparison:
- The PDF extraction damaged this matrix too.
- The sim also uses a different yaw controller structure, so Eq. (38) is not directly implemented.

### Eq. (40): Position-Attitude Kinematic Relationship
Paper, rewritten in clean notation:

```text
[ẍ, ÿ]^T = g R(ψ) [θ, φ]^T
```

Code:
- `main.py:3415-3424`
- `main.py:3548-3553`

Comparison:
- This is conceptually very close.
- What is different:
- the paper writes the relation as a compact kinematic equation
- the code uses the same idea through the desired-angle controller plus the full thrust projection
- in the sim, horizontal acceleration is also affected by rotor lag and extra damping

## Direct, Partial, And Not Exact

### Strongest Direct Matches
- Eq. (2)
- Eq. (12)
- Eq. (21)
- the bounded-guidance structure in Eq. (33)

### Very Close But Not Exact
- Eq. (8)

### Partial Or Simplified Matches
- Eq. (14)
- Eq. (16)
- Eq. (19)
- Eq. (32)
- Eq. (34)
- Eq. (40)

### Not Implemented Exactly As Written
- Eq. (9)
- Eq. (22)
- Eq. (35)
- Eq. (36)
- Eq. (37)
- Eq. (38)

## Biggest Scientific Differences
- The sim uses the small-angle assumption from Eq. (10) instead of the full Eq. (9) transformation.
- The rotor lag is simplified to first-order response instead of the full DC motor transfer function in Eq. (14).
- The yaw controller is not the same controller as Eq. (36) to Eq. (38).
- The altitude channel is close to the paper, but still simplified compared to the full Eq. (34) form.
- The sim also keeps extra braking damping and a stop-settle rule that are not from the paper.

## Bottom Line
- The main sim is clearly paper-inspired.
- The translational thrust projection, attitude dynamics, rotor mixing, yaw acceleration, and bounded guidance are the strongest paper-to-code links.
- But it is not a line-by-line reproduction of the paper.
- The biggest simplifications are:
- direct Euler-rate integration
- simplified motor lag
- a different yaw controller
- extra gameplay damping and stop-settle logic
