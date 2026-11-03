# Path – design and details (for mentors and developers)

**Status: implemented; unit-tested in simulation (JUnit, not committed); not yet run on a robot. Numbers marked (script) were computed with Python re-implementations; in v1 the Java `profileTime` is a simulation of the per-loop speed law at build plus the hand-over tail (see the v1 scope). Line numbers cited for `ExampleSwerveWithPose.java` are from the file before `getFieldSpeeds()` and `follow(Path)` were added.**

The student guide is [Path.md](Path.md). This page has everything behind it: how a list of points
becomes a path, how the heading and the speed are chosen along it, how the follow command drives
the robot, the full API, every default, and the test cases.

The code is in `Infrastructures/src/main/java/frc/robot/Path` (package `frc.robot.Path`). It uses the [Kinematics](Kinematics.md) module on the drivetrain side and takes
the pose from a `Supplier<Pose2d>`, so any pose source works, for example [RobotPose](RobotPose.md).

## v1 scope (simplified, 2026-11-03)

Branch `Paths`, commit ba11ae9 and later. The Path module was cut to what a first robot test needs: **fewer classes and fewer lines**, so students can read all of it and
there is less to tune. The package `frc.robot.Path` is **8 files**:

| File | Content |
|---|---|
| `Point.java`, `Path.java` | the waypoint and the path (geometry built in the constructor, `warnings()`, `flipped(...)`) |
| `PathGeometry.java` | geometry, segment table, projection, flip |
| `SpeedProfile.java` | heading profile, speed profile, online speed law |
| `FollowPathCommand.java` | the follower, `Result`, `progress()`, a `Sendable` |
| `PathConstraints.java`, `PathsConfig.java`, `PathFollowerConfig.java` | the three config records (`FlipMode` is nested in `PathFollowerConfig`) |

**Removed in v1:** face-target mode; events (use `cmd.progress()` / `distanceTraveled()` with a `Trigger`); `PathLogger`, `PathWarnings`, `PathTelemetry` and the alerts (the command is a
`Sendable`, `Path.warnings()` lists the build warnings, failures go to `DriverStation.reportWarning`); variable-speed corners (`withVelocity` on a corner caps the whole arc); the motor-curve
speed-up and the wheel-speed cap (`moduleRadius`, `PathsConfig.fromModules`); the flipped-copy memo, `forCurrentAlliance` and `holdEndCommand()`; AprilTag field loading (the field size is
`PathFollowerConfig.FIELD_LENGTH` x `FIELD_WIDTH`); the C1 and C2 warnings. `FollowResult` is now `FollowPathCommand.Result`, `FlipMode` is now `PathFollowerConfig.FlipMode`, and
`kTheta`, `tauLag` and `tSettle` are constants.

**Jerk limit removed in v1** (user decision, 2026-11-03). The runtime speed command is the plain trapezoid law of sections 7.6.5 and 8.3.7:
`v_cmd = min(vBack(s_v), sqrt(v_h^2 + 2·D·(s_goal - d_h - s_v)), sqrt(vMax^2 - c^2), v_prev + aAccEff·dt)`, with the lag lead
`s_v = min(s* + v_prev·tau_lag, s_goal - d_h)`, `v_h = k_p·d_h`, D the smallest line aD up to the hand-over point, and aAccEff = 0 on a
constant-speed arc except below the planned arc speed vF (a speed-up floor at aAcc after a correction-budget cut, so the robot cannot stall on an arc). Each braking term is used in the discrete form `sqrt(vE^2 + (D·dt)^2) - D·dt`, so one loop never brakes harder than D. Braking is limited by aD through vBack (position-based); there are no stopDist cases, no landing rule, no D_brake,
no room-to-brake cap vJerk (the plain backward pass already leaves the braking room) and no offline simulation. `d_h` is a fixed
0.05 m (`SpeedProfile.HAND_OVER`); the at-rest hand-over band is `d_h + 0.01 m`. `profileTime` is closed form: the span times of the
passes (section 7.7) + dwells + the hand-over tail `max(0, ln(d_h/pos_tol)/k_p)` per goal. `PathConstraints` has no `jerkMax` and
`PathsConfig` has no `maxJerk` (it keeps `maxAngularVelocity` and `maxAngularAccel`). Section 7.13 is kept as v2 reference.

**Speed profile simplified to a per-loop limit list** (user decision, 2026-11-03; replaces the span passes, the closed form and an
interim 2 cm sampled grid). `SpeedProfile` builds a short list of limits `Limit(sStart, sEnd, vCap)`: every arc with `sqrt(aLat·r)`
(a `withVelocity` on its corner caps the whole arc), every heading piece between anchors with `theta' != 0` with `omegaMax/abs(theta')`,
and a point limit for a `withVelocity` point that is not on an arc; vMax applies everywhere, and each stop and the end are goals.
Each loop (like the 2025 `getVelocity`) the law is `v_cmd = min(ceiling(s_v), cap, v_prev + aAcc·dt)`, where `ceiling` is the
smallest of vMax, every limit overlapping `[s*, s_v]` (so the lead point past a zone end does not lift its cap),
`sqrt(vCap^2 + 2·d·(sStart - s_v))` of every limit ahead of s_v and before s_goal (d = aDec, or `min(aDec, alphaMax/abs(theta'))`
toward a heading piece), and the hand-over envelope `sqrt(v_h^2 + 2·aDec·(s_goal - d_h - s_v))`; the braking terms use the discrete form, so one
loop never brakes harder than aDec. When the command rises, the ceiling is also read at `s_v + tau_lag·dv` (the lead moves that
much more next loop), which keeps the per-loop braking within aDec at the change from speeding up to braking. Speed-up is aAcc,
reduced on every limit overlapping `[s*, s_v]`: on an arc to `aAcc·sqrt(max(0, 1 - (v^2/(r·aLat))^2))` (grip ellipse <= 1) and on a
heading piece to `alphaMax/abs(theta')`. There is no arc floor, heading-kink cap or per-span brake limit, and the CAPTURE law
(8.3.7e) uses aAcc. `profileTime` simulates the same law at build with ideal tracking
(`dt = 0.02 s`, from the measured start speed, `s += (v_prev + v)/2·dt` to each hand-over point, from rest after each stop), plus
the start dwell, and per goal the dwell and the tail `max(0, ln(d_h/pos_tol)/k_p)`; a run longer than
`10·(L/min(vMax, smallest vCap) + sum of dwells) + 10 s` throws. Sections 7.3, 7.4, 7.6, 7.7 and 7.12 are replaced in v1 by this law and kept as reference. Reference numbers (EX, Java):
shared example profileTime **3.3703 s** (closed form before: 3.4426 s), simulated run done 3.40 s, final error 0.020 m, no
overshoot; `P1.stop()` profileTime **4.0606 s** (before: 4.2245 s), done 4.08 s; `P1.stop(1.0)` done 5.10 s, final error 0.018 m,
no overshoot of the stop or the end; `P1.withVelocity(1.0)` profileTime 5.3503 s. The simulated time is shorter than the closed
form because the closed form also counted the last 5 cm before each goal at trapezoid speed on top of the tail, and arcs were
driven at constant speed; it is closer to the simulated runs.

Earlier reference numbers in v1 (closed form, before the simplification above; EX limits, Java): shared example profileTime 3.2723 + 0.1703 = **3.4426 s**, timeout 6.16 s, simulated run (lag
plant, section 8.15) done at 3.36 s, final error 0.018 m, no overshoot; `P1.stop()` profileTime **4.2245 s**, timeout 7.34 s, done 4.00 s;
`P1.stop(1.0)` done 5.02 s, final error 0.016 m, no overshoot of the stop or the end; `P1.withVelocity(1.0)` profileTime 5.4279 s.

**How to read this page:** sections 9 (API), 10 (defaults) and 11 (v1 and v2) match the code. Sections that describe a removed feature are **kept as design reference for v2**; each
starts with the line "Removed in v1 (simplification); kept as v2 reference." (or a "Changed in v1" note). Numbers in sections 7, 8 and 13 come from the earlier design (with the
motor curve and ARC_VAR); where they differ from the values above, the values above and the code win.

## 1. Contents

- [v1 scope (simplified, 2026-11-03)](#v1-scope-simplified-2026-11-03)
- [2. Summary](#2-summary)
- [3. Notation and conventions](#3-notation-and-conventions)
- [4. Lessons from existing code](#4-lessons-from-existing-code)
- [5. Geometry](#5-geometry)
- [6. Heading profile](#6-heading-profile)
- [7. Speed profile](#7-speed-profile)
- [8. Follower](#8-follower)
- [9. Public API](#9-public-api)
- [10. Defaults](#10-defaults)
- [11. v1 and v2 scope](#11-v1-and-v2-scope)
- [12. Open items for the user](#12-open-items-for-the-user)
- [13. Test cases](#13-test-cases)
- [Appendix A. JSON sketch (future editor, not v1)](#appendix-a-json-sketch-future-editor-not-v1)
- [Appendix B. Test oracle: grid passes](#appendix-b-test-oracle-grid-passes)

## 2. Summary

### 2.1 What the user writes

```java
Path path = new Path(
    new Point(1, 2),
    new Point(3, 4).withAngle(Rotation2d.fromDegrees(90)),
    new Point(5, 2));

Command follow = new FollowPathCommand(swerve, path);   // swerve implements PathDrivetrain
// optional: .withConfig(PathFollowerConfig.defaults().withFlip(FlipMode.MIRROR))
```

Points are in meters, in the blue-alliance field frame. Optional per point: an angle (`withAngle`), a
speed limit at that point (`withVelocity`), a corner radius (`withRadius`), a stop with an optional
wait (`stop()`, `stop(seconds)`). Per path: limits (`PathConstraints`), a face-target point, events
at a fraction of the path or at a point. The command takes the pose, the measured field speeds, and the
drivetrain's field-relative drive method.

### 2.2 What the library does

1. **Geometry, in the `Path` constructor (blue frame).** Each middle point gets a circle that passes
   through the point. Lines tangent to the circles join them. The result is line, arc, line, ...,
   line, and every point where the path turns lies exactly on it (a point on a straight line is only a pass
   marker, section 5.8.3). The path is a function of arc length s (meters along the path). Sharp corners (more than 150 deg) and corners where no arc fits become stops.
2. **Alliance flip, in `initialize()`.** On red, the built path is mirrored or rotated once.
3. **Heading and speed profiles, in `initialize()`.** The heading is piecewise linear in s between
   points that have an angle. The speed is a closed-form profile per span (a piece of path with
   constant limits): speed up, cruise, slow down. Arcs are driven at constant speed,
   so the speed changes only on lines; `withVelocity` on a corner caps the whole arc (v1; the variable-speed arc ARC_VAR of section 7.8 is v2). Junction speeds come from one backward and one forward pass. Stops
   split the profile.
4. **Follower, every 20 ms.** The robot must start within 0.5 m of P0. Project the pose onto the path to get progress s*. Command the speed
   with the online trapezoid law `min(vBack(s), previous + aAcc·dt)`, whose braking envelope vBack comes from the profile
   (braking by position with the per-span deceleration), the direction from the path
   tangent a little ahead, a cross-track correction toward the path, and omega from the heading
   profile plus feedback. Near a stop or the end a 2D position law takes over. Output is
   field-relative `ChassisSpeeds`, fed to the drivetrain's limiter and `DemaciaKinematics`.
5. **Finish.** Within 0.03 m and 2 deg for 0.1 s: success. Timeout `1.5·profileTime + 1 s`: zero
   speed once, result `FAILED_TIMEOUT`.

### 2.3 Key decisions

| Decision | Choice | Reason |
|---|---|---|
| Parameter | Everything is a function of arc length s, never of time | A robot that falls behind is never "late"; braking stays tied to position |
| Corner shape | Circle through the waypoint, center on the inner bisector; lines tangent to the circles (section 5.2) | Every waypoint where the path turns lies exactly on the path; a point on a straight line is a pass marker (section 5.8.3) |
| Corner radius | Automatic: `min(R_MAX, d/2 or d/4, bulge cap)`, shrink loop if a check fails, stop below R_MIN; `withRadius(r)` override (section 5.7) | Fast corners without wild bulges; always a valid path |
| Very sharp corners | Turn above TAU_STOP = 150 deg becomes a stop unless `withRadius` is given (section 5.6.4) | A stop is about as fast at 150 deg and keeps the robot on the drawn lines |
| Continuity | G1 (line + arc) in v1; clothoids are v2 if a logged criterion says so (section 5.12) | Closed-form geometry; the speed profile handles the curvature step |
| User stop | `stop()`, `stop(dwell)`, `withVelocity(0)`; removes the corner's arc (section 5.8.7) | At rest an arc buys nothing; swerve changes direction at rest |
| Heading | Piecewise linear in s between anchors, from the start heading to the first anchor, hold after the last; shortest way (section 6; face-target mode is v2) | Constant heading rate per span, so the speed caps stay exact |
| Speed profile | Closed-form trapezoid per span, constant speed on every arc (a `withVelocity` point caps the whole arc), junction passes (section 7) | Equals the time-optimal grid profile when every arc is driven at its curvature cap; stays inside the limiter |
| Runtime speed | Plain online trapezoid law `min(vBack(s_v), v_prev + aAccEff·dt)` with the lag lead and the hand-over envelope (sections 7.6.5, 8.3.7; the jerk-aware law of 7.13 was removed in v1) | No stall at the start or after a stop; braking by position; no overshoot in simulation |
| Projection | Windowed, monotonic progress, window capped at the next stop (section 8.2.3) | No jumps to another branch of a self-approaching path |
| Finish | 0.03 m, 2 deg, 0.1 s settle; timeout `1.5·profileTime + 1 s`; zero speed once on failure (section 8.7) | Brief decision; a command that never ends blocks autonomous sequences |
| Alliance | `FlipMode` NONE / MIRROR / ROTATE in `PathFollowerConfig`, default ROTATE, applied once to the built path (sections 5.13, 8.10) | The 2027 field is unknown |
| Start | The robot must start within 0.5 m of P0, else `FAILED_START_TOO_FAR`; no mid-path start (section 8.2.5) | Simple, and a wrong flip or a wrong path shows at once |
| Settings | Three public config records: `PathConstraints`, `PathsConfig`, `PathFollowerConfig`; everything else is an internal constant (sections 9.6, 10) | Few knobs for students; one place for each value |
| Limits | `PathConstraints(vMax, aAcc, aDec, aLat, omegaMax, alphaMax)`, defaults from `KinematicsConfig` at 90% (section 7.2) | The profile then never fights the acceleration limiter |
| Kinematics module | Not changed. Missing settings are proposed (section 12) | No edits to working code |
| Pose input | `Supplier<Pose2d>` plus `Supplier<ChassisSpeeds>`; no RobotPose dependency | Works with any pose source |
| Editor | Java API only in v1; a JSON sketch for a future editor (Appendix A) | Room for a later path editor |

## 3. Notation and conventions

All symbols in this document. ASCII names are used for Greek letters, one spelling each.

| Symbol | Meaning | Unit | Frame, sign, notes |
|---|---|---|---|
| field frame | WPILib field frame, blue-alliance origin | | x toward the red wall, y left, angles CCW positive. All user input is in this frame on both alliances |
| robot frame | WPILib robot frame | | x forward, y left, CCW positive |
| P_i | waypoint i, i = 0..n-1 | m | n >= 2 after merging points closer than EPS_DUP; the geometry uses the points left after straight middle points are set aside as PASS markers (5.8.3) |
| d_j | length of leg j, `abs(P_{j+1} - P_j)`, j = 0..n-2 | m | the straight line the user drew |
| u_in,i, u_out,i | unit directions `unit(P_i - P_{i-1})`, `unit(P_{i+1} - P_i)` | | |
| tau_turn,i | turn angle at corner i | rad | in [0, pi]; 0 = straight on, pi = U-turn |
| gamma_i | half the interior angle, `(pi - tau_turn,i)/2` | rad | `cos(gamma_i) = sin(tau_turn,i/2)` |
| sigma_i | turn sign, `sign(cross(u_in,i, u_out,i))` | | +1 left (CCW), -1 right, 0 straight |
| b_i | unit inner bisector at P_i, `unit(-u_in + u_out)` | | points into the corner |
| r_i, C_i | corner circle radius and center | m | r = 0 means a point element (endpoint, STOP) |
| h_j, lambda_j | signed tangent offset and length of the line on leg j | m | section 5.4 |
| uD, uD_perp | unit vector from element j to j+1 and its left normal | | section 5.4 |
| Q_in, Q_out | arc entry and exit points | m | |
| phi0, phi_P, dphi | arc start angle (angle of C -> start point), angle of P_i seen from C, signed sweep | rad | dphi + = CCW; arc length = `r·abs(dphi)` |
| w_in, w_out | arc angle from Q_in to P_i and from P_i to Q_out | rad | both >= 0 |
| rot90, cross, mod2pi, wrap | `rot90(x, y) = (-y, x)`; `cross(a, b) = a.x·b.y - a.y·b.x`; mod2pi into [0, 2·pi); wrap into (-pi, pi] (what `MathUtil.angleModulus` returns: both +pi and -pi map to +pi, checked by running it) | | wrap = `MathUtil.angleModulus` |
| s | arc length from the path start | m | s in [0, L] |
| L | total path length | m | L > 0 (a zero-length path is rejected) |
| k, m | segment index, number of segments | | m = 2n-3 for n >= 3 (zero-length STOP entries kept) |
| s_start,k, l_k | start and length of segment k | m | |
| s(P_i) | s of waypoint i | m | section 5.9.4 |
| q | local arc length inside a segment or span, `q = s - s_start` | m | |
| x(s), T(s), N(s) | point, unit tangent (direction of travel), unit left normal = T rotated +90 deg | m, -, - | |
| kappa(s) | signed curvature | 1/m | + left turn, - right turn; line 0, arc `sigma/r` |
| theta_T(s) | angle of T(s) (direction of travel) | rad | not the robot heading |
| p | robot position from the pose supplier | m | field frame, blue origin |
| s*, s_hi, s_prev | projected progress, highest accepted s*, last s* | m | all start at 0 (section 8.2.5) |
| sMin, sMax | projection window | m | section 8.2.3 |
| e_n | signed cross-track error `(p - x(s*))·N(s*)` | m | + = robot LEFT of the path |
| e_t | along-track residual `(p - x(s*))·T(s*)` | m | 0 unless the projection is clamped |
| s_goal, P_goal | next stop point s_c (or L) and its position | m | |
| d_go | signed distance to go, `(s_goal - s*) - e_t` | m | negative after an overshoot |
| s_c, P_c | s and position of a stop point | m | |
| theta(s) | heading profile (robot heading wanted at s), stored unwrapped | rad | field frame, CCW +, 0 = +x |
| theta' | `dtheta/ds`, constant per span | rad/m | CCW + |
| theta0 | measured robot heading when the command starts | rad | |
| (s_a, theta_a) | heading anchor: a waypoint with `withAngle`, at s(P_a) | m, rad | section 6 |
| omega, omega_ff, omega_fb | angular velocity: command, feed-forward, feedback | rad/s | CCW + (WPILib). Only the drivetrain flips it for `toSwerveModuleStates` |
| alpha | angular acceleration `domega/dt` | rad/s^2 | CCW + |
| e_theta | heading error `wrap(theta_ref - theta)` | rad | |
| v | speed along the path | m/s | >= 0, along T |
| v_t | tangential speed command of the follower | m/s | |
| c | cross-track correction speed, `clamp(-k_n·e_n, -c_max, c_max)` | m/s | + = toward the left |
| V | field velocity command `v_t·T(s_L) + c·N(s*)` | m/s | field frame. The brief's form is `V = v(s*)·T(s*) - k_n·e_n·N(s*)`: left error pushes right |
| V_meas | measured field velocity (speed supplier) | m/s | field frame |
| v0, v0_par, v0_perp | start speed `max(0, V_meas·T(0))`, its components along and across | m/s | |
| vBack(s) | runtime speed envelope: caps plus the backward pass only | m/s | 0 at stops and at L |
| vProf(s) | vBack after the forward pass from v0 | m/s | time, timeout, logging |
| K, k, j | number of spans, span index k = 0..K-1, junction index j = 0..K | | junction j = start of span j, junction K = end |
| vCap[k] | span cap | m/s | section 7.4 |
| J[j] | junction cap | m/s | |
| vB[j], vF[j] | junction speeds after the backward pass, after both passes | m/s | |
| aA[k], aD[k] | speed-up limit (at rest) and slow-down limit on span k | m/s^2 | |
| v_in, v_out | entry and exit speed of a span | m/s | |
| a, a_t, a_n | tangential acceleration (+ speeding up), tangential, lateral `v^2·kappa` | m/s^2 | a_n + toward N |
| aAccEff(v, s) | effective speed-up limit at speed v and progress s | m/s^2 | section 7.6.5 |
| rho_lat | grip share of an ARC_VAR span (ARC_RHO = 0.8): cap `sqrt(rho_lat·aLat/abs(kappa))`, tangential limits `sqrt(1 - rho_lat^2)` = 0.6 of aAcc and aDec | | section 7.8 |
| a_0, v_free, v_knee | motor-curve stall acceleration (`0.9·maxForwardAccel`), free speed, knee speed | m/s^2, m/s, m/s | section 7.9 |
| R_mod | largest distance from the robot center to a module | m | robot frame |
| vWheelMax | `KinematicsConfig.maxAllowedModuleVelocity` | m/s | |
| e_allow | heading lag allowed at a heading kink (HEADING_LAG_ALLOWED) | rad | |
| v_cmd | state of the speed law: commanded tangential speed | m/s | sections 7.6.5, 8.3.7 |
| jerkMax, stopDist, D_j | jerk-aware law only (removed in v1; section 7.13 is the v2 reference) | | |
| d_h, v_h | hand-over distance before a goal and the speed there, `v_h = k_p·d_h` | m, m/s | section 8.3.7 |
| t, dt, dt_nom | time, measured loop period, nominal loop period | s | dt_nom = 0.02 s |
| t_k, t0[k] | time of span k, profile time at the start of span k | s | |
| profileTime | offline simulation of the speed law from 0 to L (with hand-over tails) plus the dwells at stops | s | section 7.13.6 |
| t_timeout | `1.5·profileTime + 1.0` | s | |
| k_n, k_p, k_theta | cross-track gain, goal-capture gain, heading gain | 1/s | |
| tau_lag | lookahead time of the follower | s | a setting |
| tau_d | real drivetrain lag (latency + loop + velocity loop) | s | a robot property, to measure |
| s_L, s_v | lookahead points for direction and for speed | m | section 8.4 |
| d_lin | `aDec/k_p^2`, edge of the linear zone of the goal law | m | |
| vsqrt(d; a, k) | sqrt goal law | m/s | section 8.3.7 |
| FL, FW | field length and width | m | `PathFollowerConfig` fields; placeholder pair 16.54 x 8.07 |
| target, pT | face-target point, `pT = target - p` | m | |

Constants (EPS_DUP, R_MIN, TAU_STOP, ...) are named in capitals and defined once, in section 10.

**Labels.** **Fact (file:line)**: read in the code. Paths: Infrastructures code under
`Infrastructures/src/main/java/frc/robot/` (Kinematics files cited by file name); old code under
`/home/user/demacia5635/` (path-ido = `demaciacode` branch path-ido `src/main/java/frc/demacia/path/`,
2025 = `season2025/src/main/java/frc/robot/Path/`, 2026 = `season2026/src/main/java/frc/demacia/path/`,
2027 = `utils2027--alpha7/src/main/java/first/demacia/path/`). **Inference**: derived here.
**Recommendation**: a design choice; values are starting points to tune. **Uncertain**: not settled.
**(script)**: computed by a Python re-implementation of the math in this document, run with
`python3 -I` (not part of the repository; nothing compares it with Java). **(hand)**: computed by hand.

**Shared example.** Unless stated otherwise, worked numbers use:
- path **P0 = (1, 2), P1 = (3, 4), P2 = (5, 2)**, automatic radius (geometry in section 5.11);
- **example constraints EX**: vMax = 4.0 m/s, aAcc = aDec = 3.0 m/s^2, aLat = 4.0 m/s^2,
  omegaMax = 2·pi rad/s, alphaMax = 4·pi rad/s^2;
- **default constraints DEF** (`PathConstraints.fromConfig(new KinematicsConfig())`): 4.05 m/s,
  7.2, 7.2, 7.2 m/s^2, 2·pi rad/s, 4·pi rad/s^2;
- start at rest, measured start heading theta0 = 0, P1 `withAngle(90 deg)`, P2 without angle.

Key numbers of the shared example (all (script)): r = 1.4142 m, C = (3, 2.5858), L = 5.9769 m,
s(P1) = 2.9885 m. Trapezoid profile: 3.2723 s; with `P1.stop()`: L = 5.6569 m and
3.8839 s. v1 profileTime (trapezoid + hand-over tail 0.1703 s per goal): 3.4426 s, timeout 6.16 s; with `P1.stop()`: 4.2245 s,
timeout 7.34 s.

## 4. Lessons from existing code

What the old path code does, checked by reading it and, where marked, by re-implementing its math
in Python (re-impl). Nothing old was built or run.

### 4.1 path-ido (newest geometry, by ipb933)

Files: `Path.java` (97 lines), `points/Point.java`, `points/CirclePoint.java`, `segmants/Segmant.java`,
`segmants/LineSegmant.java`, `segmants/CircleSegmant.java`, `commands/followPathCommand.java` (154).

How it works (**Fact**):
- Radius `r_i = min(RADIUS = 10, d_left/2 or /4, d_right/2 or /4)`, divisor 2 only on a leg that touches
  P_0 or P_{n-1} (Path.java:12, :37-43). No other cap.
- Center on the inner bisector at distance r from the waypoint, so the waypoint is on the circle
  (Path.java:46-59; half interior angle from a law-of-cosines acos, :47-50; side from
  `sigh(prev, P, next)`, :51).
- `Point.sigh` = `signum(cross(P_{i-1} - P_i, P_{i+1} - P_i))` (Point.java:56-61) = **-sigma_i**.
- Endpoint tangent with `asin(r/|P - C|)` (Segmant.java:37-48); circle-to-circle tangent with
  `sign2 = -sigh·sigh` and `acos((r2 + sign2·r1)/d)` (Segmant.java:50-63). Last leg built backward and
  reversed (Path.java:76-77). Order line, arc, ..., line, `2n-3` entries (Path.java:70); n = 2 gives one
  line (Path.java:65-67).
- Follower: constant speed `MAX_VELOCITY = 1.5` (followPathCommand.java:25, :64); segment switch within
  `ALOWED_ARROR = 0.1` m of the segment end point (:24, :67); heading target = angle of the next point
  with an angle, a step (:108, :125-141); current heading from `chassis.getGyroAngle()` (:108); finish =
  segment index past the end (:151-153); `end()` calls `chassis.stop()` (:146).

Suspected issues (from the brief), checked:

| ID | Claim | Verdict | Evidence |
|---|---|---|---|
| a | acos/asin arguments can leave [-1, 1] | **False** for the current construction, unguarded in principle | The caps keep Segmant.java:38 and :52 in range: shared leg `r <= d/4` gives `d(C_i, C_{i+1}) >= r_i + r_{i+1}`; end leg `r <= d/2` gives `abs(P - C) >= r`. Re-impl, 20,000 random polylines: 0 domain errors (max asin argument 0.99999988). The law-of-cosines acos calls (Path.java:47, CircleSegmant.java:28) give NaN only for a zero denominator (repeated waypoint at Path.java:50, robot at a center) |
| b | collinear points make the sign 0 | **True** | Point.java:57; used at Path.java:51, Segmant.java:39, :51, :53. Re-impl (0,0),(4,0),(8,0): first line (0,0)->(0,0), length 0, center on the path line |
| c | line remaining distance is Euclidean, not projected | **True** | LineSegmant.java:24-26, Segmant.java:33-35 |
| d | arc remaining distance uses acos, wrong above 180 deg | **True, common** | CircleSegmant.java:27-31 (range [0, pi]); the stored length uses it too (:13). Re-impl: 37% of random arcs sweep more than 180 deg (18,598 of 49,889) |
| e | constant velocity | **True** | followPathCommand.java:64 |
| f | segment switch within 0.1 m of the end point | **True** | followPathCommand.java:24, :67. A robot passing more than 0.1 m away never switches |
| g | fixedDistance computed but unused | **True in effect** | written :116-122, read only at :106, whose result is never read |
| h | heading from `chassis.getGyroAngle()` | **True** | followPathCommand.java:108, :138-139 |
| i | arc correction sign `signum(r - distanceFromCircle)` with an already signed distance | **True** | CircleSegmant.java:21-23, followPathCommand.java:100-101: the correction reverses for a robot more than 2r from the center |

Also (**Fact**): `updateNextAnglePoint` writes into the user's `Point` (followPathCommand.java:138-139).

### 4.2 season2025 (2025 follower)

| Claim | Verdict | Evidence |
|---|---|---|
| Speed from the straight-line distance to the last point | **True** | DemaciaTrajectory.java:247-249; formula `sqrt(2·a·d)` at :229. Modes switch at thresholds such as `currentVelocity·0.6` (:178, :190) |
| Corner arcs commented out | **True** | :127-129, :133-135. Legs jump across the corners |
| Heading finish check has no `Math.abs` | **True** | :270. Every negative heading error passes |
| `FIELD_LENGTH = FIELD_HEIGHT = -1` | **True** | TrajectoryConstants.java:15-16; used only on the `isRed` path (:58-59, :102-107), which the call sites do not take (FollowTrajectory.java:109, :112). The red conversion keeps the rotation and starts at i = 1 (:95, :102-106) |

Other facts: progress is the Euclidean distance from the leg start (Leg.java:46; the projection is commented
out at :45); heading deadband 0.75 deg with two gains (DemaciaTrajectory.java:254-259); final approach with
two independent ProfiledPIDs (goToFinish.java:20-21), position-only finish (:55); `fixFirstPoint` replaces
the first point with the pose (DemaciaTrajectory.java:86-90).

**Inference** (the straight-line distance): it is never longer than the distance left along the path, so it
is never too fast for the final stop, but it is too slow when the path passes near its end before reaching
it. Example: 0.3 m from the end with 4 m still to go, aDec 6: it commands `sqrt(2·6·0.3) = 1.90` m/s instead
of `min(3, sqrt(2·6·4)) = 3` m/s. It also has no slowdown for corners.

### 4.3 season2026 (Kislev 2026: ArcNew, LegNew, ArcTrajectory)

| Issue | Evidence (**Fact**) |
|---|---|
| Interior corner arcs missing from the segment list | `Trgectory/DemaciaTrajectory.java:107-115` |
| `ArcTrajectory` loop bounds stop before the last corner (n = 3 gives no segments) | ArcTrajectory.java:33, :44 (read, not run) |
| Speed ramps from the measured speed, clamped toward the constant `maxVel` | DemaciaTrajectory.java:155 |
| `/ 2` applied after the sqrt | DemaciaTrajectory.java:149-153 |
| Omega negated inside the trajectory | DemaciaTrajectory.java:193 |
| Heading deadband equals the finish tolerance (2 deg) | DemaciaTrajectory.java:191, TrajectoryConstants.java:12 |
| Heading errors not wrapped | ArcTrajectory.java:58, ArcNew.java:50, LegNew.java:29, FollowTrajectory.java:45-46 |
| Time trigger `abs(t - time) <= 0.03` can be skipped by one overrun | FollowTrajectory.java:52 |
| Raw average of two atan2 angles as the bisector | CalculateUtils.java:16 |
| Arc length from the point's target heading, not the end angle | ArcNew.java:32 |
| `maxVelocity2` read from the wrong point | RoundedPoint.java:36 |
| Unclamped acos/asin in tangents; null tangent dereferenced | CalculateUtils.java:25-27, :33, :39, :44, :55; ArcTrajectory.java:47 |
| `FIELD_LENGTH = FIELD_HEIGHT = -1` | TrajectoryConstants.java:8-9 |

### 4.4 utils2027

`Leg.java:10-34` (**Fact**): same-side tangents use a fixed pi/2 offset and opposite-side tangents
`acos(2·r/d)` on each side. **Inference**: correct only for equal radii; the argument exceeds 1 when
d < 2r, with no check.

### 4.5 Infrastructures Kinematics (facts this design relies on)

- There is no `KinematicsConstants`. The limits are instance fields of `KinematicsConfig`
  (KinematicsConfig.java:20, fields :25-59); `docs/library-plan.md:49` still uses the old name.
  Defaults: `cycleDt` 0.02 (:25), `latency` 0.035 (:30), `maxAllowedModuleVelocity` 4.5 (:32),
  `minVelocity` 0.01 (:34), `maxSteerRate` 12 (:41), `maxForwardAccel` 10 (:46), `freeSpeed` 5.3 (:51),
  `maxSkidAccel` 8 (:53), `maxTiltAccelFront`/`Side` 12 (:55, :57), `maxDt` 0.02 (:59). Its `withX`
  setters change `this` and return it (:61-114).
- `SwerveAccelerationLimiter.calculate(wanted, lastCommanded, heading)` (SwerveAccelerationLimiter.java:64):
  field-relative in and out, omega passed through (:80); dt capped at `maxDt` (:75); skid disk
  `maxSkidAccel·dt` (:106); forward limit `maxForwardAccel·max(0, 1 - speed/freeSpeed)·dt` (:100, :107),
  braking and turning free from it (:94); tilt box (:103, :109-110); an allowed change is returned
  unchanged (:116-126).
- `DemaciaKinematics`: `toSwerveModuleStates` takes omega clockwise-positive (DemaciaKinematics.java:164-168,
  :183; negated in `plan`, :267-268); flips a module more than 90 deg off and cosine scales (:194-198);
  discretizes over `cycleDt + 2·latency` (:84, :280); `desaturate` (:260) scales translation and rotation
  together (:331-345); stands still below `minVelocity` (:271-272); drive acceleration feed-forward from the
  change in commanded speed (:214-216, :295-296); module positions private, no getter (:58, :66).
- `ExampleSwerveWithPose` (example only): `getPose()` (:264), `driveFieldRelative` (:304-313: desaturate,
  limiter, `drive`), `drive` negates omega (:287-297), `setStartPose` (:259), measured speeds computed in
  `periodic()` (:335-336, before `getFieldSpeeds()` was added); `getGyroRate()` and `heading()` are private.
- RobotPose: `getEstimatedPose` documented as blue-alliance origin (RobotPose.java:267, method :270);
  field size from `AprilTagFieldLayout.loadField(kDefaultField)` (:94-95); `initSendable` (:178);
  `getFuturePose` (:291). `PoseLogger` has `putData` (PoseLogger.java:41) and `alert` (:51).

### 4.6 How this design avoids each old problem

| Old problem | This design |
|---|---|
| Constant speed, no braking (path-ido) | Speed profile from s, section 7; runtime speed section 8.3.7 |
| Segment switch near the end point (path-ido) | No segment state; windowed projection every loop, section 8.2.3 |
| Heading step, gyro instead of pose (path-ido) | Interpolated theta(s), heading from the pose supplier, sections 6 and 8.5 |
| Finish without position or heading check (path-ido, 2026) | Finish test, section 8.7 |
| Arc correction sign reverses (path-ido) | One signed e_n for lines and arcs, section 8.3.1 |
| Arc length wrong above 180 deg (path-ido) | Sweep by mod2pi, section 5.5 |
| Collinear points and duplicates (path-ido) | PASS markers, merging, section 5.8 |
| Straight-line distance for the speed (2025) | d_go along the path, section 8.3.7 |
| Euclidean progress (2025) | Orthogonal projection, section 5.10 |
| Heading check without abs, deadbands (2025, 2026) | `abs(wrap(...))`, no deadband, sections 8.5.2, 8.7.2 |
| `FIELD_LENGTH = -1`, red conversion keeps the rotation (2025, 2026) | Flip mode with a real field size, section 5.13 |
| Two independent PIDs on x and y for the finish (2025, 2026) | One 2D capture law, section 8.3.7 |
| Ramp from the measured speed (2026) | Ramp from the previous command, section 8.3.7 |
| Unwrapped heading errors (2026) | `wrap` on every angle difference, section 8.5.4 |
| Time trigger can be skipped (2026) | s-crossing rule, section 8.8 |
| Omega negated inside the trajectory (2026) | Follower outputs omega CCW +; only the drivetrain flips it, section 8.11 |

## 5. Geometry

This chapter defines how a list of waypoints becomes a path parameterized by arc length s: the corner
construction, the tangent lines, the arc sweep, radius selection, every degenerate case, evaluation at
a given s, projection of the robot position onto the path, G1 versus G2, and the alliance flip.
The geometry is built once, in the `Path` constructor, in the blue frame.

### 5.1 Scope and constants

Symbols are in section 3. The geometry constants (EPS_DUP, EPS_ANG, EPS_LEN, EPS_WRAP, EPS_CENTER,
R_MIN, R_MAX, D_BULGE, EPS_DEV, TAU_STOP, SHRINK, MAX_ITER, PASS_TOL) are defined in section 10. All of them
are internal `static final` constants of the geometry builder (**Decision**, user, simplification pass: there is
no geometry config record). The user's per-point knobs are `withRadius` and `stop`.

### 5.2 The idea, from scratch

**What the user gives.** A list of points P_0 .. P_{n-1}. The user means "drive through these points
in order". A polyline (straight legs from point to point) has sharp corners, and a robot cannot change
direction instantly at speed. So each corner must be rounded.

**The classic way, and why it is not used.** The usual rounding (a "fillet") puts a circle *inside* the
corner, touching both legs. The robot drives along leg 1, rolls onto the circle, rolls off onto leg 2. The
circle never touches the corner point itself, so the robot misses P_i by `r·(1/cos(tau_turn/2) - 1)` (0.414 m
for a 90 deg corner with r = 1 (hand)). Every waypoint must lie **exactly** on the path (brief decision 1),
so the fillet is kept only as the future `.cutCorner()` option.

**The waypoint-on-circle idea.** Picture a coin of radius r:

1. Put the coin inside the corner so that its edge touches the corner point P_i.
2. Slide the coin (keeping its edge on P_i) until its center sits on the line that splits the corner
   angle exactly in half. That line is the inner bisector b_i. Now the center is `C_i = P_i + r·b_i`.
   The distance from C_i to P_i is r, so P_i is on the coin's edge. That is the whole reason for
   "distance r": it is the definition of P_i being on the circle.
3. From the previous point, draw the straight line that just grazes the coin (a tangent line) on the side
   that makes the robot turn the right way. Drive along it.
4. Roll around the coin's edge in the turn direction. Because P_i is on the edge, the robot passes over P_i.
5. Leave the coin along the grazing line toward the next point (or the next coin).

Why the bisector: by symmetry, P_i is then the point of the coin furthest out of the corner, and the robot
crosses P_i heading exactly halfway between the two leg directions. For a 90 deg corner from heading
45 deg to heading -45 deg, the robot crosses P_i at heading 0 deg (worked example below).

Why the robot does not simply drive along the leg: the leg passes through P_i, and P_i is on the circle,
so the leg *cuts* the circle (a secant). It is not tangent to it. A robot driving along the leg would
hit the circle at an angle, a corner in the direction of travel. The tangent line from P_{i-1} is the leg rotated slightly
outward, by an angle called the deviation (14.06 deg in the worked example (script)).

Diagram A: where the circle goes (worked example, right turn at P1; not to scale).

```
                           P1 (3,4)        <- the waypoint, top of the circle
                         .' | '.
        leg P0-P1      .'   |   '.      leg P1-P2
      (user's line)  .'     | r   '.    (user's line)
                   .'       |       '.
                 .'         C (3, 2.586)  '.     C = P1 + r*b,  r = 1.4142
               .'      (center, on the      '.   b = (0,-1): splits the corner
             .'         inner bisector)       '.     in half, points into it
          P0 (1,2)                           P2 (5,2)
```

Diagram B: the driven path (solid) against the user's legs (dotted).

```
                        _..--P1--.._
           Q_in   _.-'     .'|'.     '-._   Q_out         P0, P1, P2: all ON the path
   (1.787,3.313) *       .'  |  '.        * (4.213,3.313)
                /      .'    |r   '.       \              /  \ and the top arc: driven path
               /     .'      |      '.      \             .' '.: the user's legs (not driven)
              /    .'        C        '.     \
             /   .'                     '.    \           lower half of the circle: not driven
            /  .'                         '.   \
           / .'                             '.  \
          P0'                                 'P2
```

The robot starts at P0 driving at 59.06 deg instead of the leg's 45 deg. It curves right, passes over P1
driving at 0 deg, keeps curving, leaves at -59.06 deg, and arrives at P2. On both legs the path stays
outside the corner. It bulges out by at most 0.4142 m (section 5.6).

Diagram C: lines between two circles.

```
  both corners turn LEFT (same sign):            corner i turns LEFT, i+1 turns RIGHT
  outer tangent, robot on the right of           (opposite signs): inner (cross) tangent
  both centers                                                          .-.
     .-.                    .-.                                        ( C2 )  robot goes CW
    ( C1 )                 ( C2 )                     .-.             _/'-'
     '-'\__________________/'-'                      ( C1 )        _/
        ---- direction of travel ---->                '-'\      _/
                                                          \  _/   <- crosses the line C1-C2
                                                           \/
```

### 5.3 Corner construction

For each interior waypoint i = 1..n-2 (after duplicate merging, section 5.8.2):

```
u_in  = unit(P_i - P_{i-1})          u_out = unit(P_{i+1} - P_i)
tau_turn,i = atan2(abs(cross(u_in, u_out)), dot(u_in, u_out))      in [0, pi]
sigma_i    = sign(cross(u_in, u_out))                              in {-1, 0, +1}
b_i        = unit(-u_in + u_out)                                   inner bisector
C_i        = P_i + r_i * b_i
```

Derivation and justification:
- `-u_in` points from P_i back toward P_{i-1}, and `u_out` points toward P_{i+1}. Both are unit vectors, so
  their sum lies on the bisector of the interior angle between them. Its length is
  `2·cos(gamma_i) = 2·sin(tau_turn,i/2)`.
- The sum is zero only when `u_out = u_in` (straight on, tau_turn = 0). Straight waypoints never reach
  this formula; they are handled separately (section 5.8.3). For a U-turn (tau_turn = pi) the sum is
  `-2·u_in`, which is well defined. So `unit()` never divides by less than `2·sin(EPS_ANG/2)`, about 1e-4.
- `atan2` for tau_turn never calls acos or asin on an argument that rounding could push past 1.
- sigma_i is + for a left (CCW) turn. The center lies on the turn side.

Worked example (P0 = (1,2), P1 = (3,4), P2 = (5,2)) (hand, script):
- `u_in = (2,2)/2.8284 = (0.7071, 0.7071)`, `u_out = (2,-2)/2.8284 = (0.7071, -0.7071)`.
- `cross = 0.7071·(-0.7071) - 0.7071·0.7071 = -1`, `dot = 0`, so `tau_turn = atan2(1, 0) = 90 deg`,
  `sigma = -1` (right turn).
- `-u_in + u_out = (0, -1.4142)`, length `1.4142 = 2·sin(45 deg)`. `b = (0, -1)`.
- With r = 1.4142 (section 5.7): `C = (3, 4) + 1.4142·(0, -1) = (3, 2.5858)`.

Edge cases: tau_turn < EPS_ANG gives a straight waypoint (section 5.8.3). tau_turn > pi - EPS_ANG is a
U-turn: sigma is 0 from the cross product and must be chosen (section 5.8.4). A duplicate neighbor is
impossible after merging.

Comparison with path-ido: **Fact** (Path.java:46-59). It computes the half interior angle with a
law-of-cosines acos (Path.java:47-50), takes `sign = sigh(prev, P, next)` (Path.java:51), and puts the
center at angle `atan2(prev - P) + sign·gamma` (Path.java:52-55). `sigh` (Point.java:56-61) is
`signum(cross(P_{i-1} - P_i, P_{i+1} - P_i)) = -sigma_i`, so anyone porting path-ido must flip it.
**Inference:** the resulting center is the same as ours for every proper corner (checked on the 4-point
example of section 4.1: same tangent points to 4 decimals (script)). It differs in two places: for
collinear points sign = 0 puts the center on the incoming leg (section 4.1, issue b), and for a repeated
waypoint Path.java:50 divides by zero.

### 5.4 Tangent lines (one formula for all cases)

Treat every element as a circle. A corner with an arc is `(C_i, r_i, sigma_i)`. P_0, P_{n-1} and STOP entries
are "point circles": `(P_i, r = 0, sigma = 0)`. The line on leg j joins element j to
element j+1. One formula covers the endpoint tangent, the outer tangent and the inner tangent:

```
D  = C_{j+1} - C_j          d_c = abs(D)          uD = D / d_c          uD_perp = rot90(uD)
h  = sigma_{j+1}*r_{j+1} - sigma_j*r_j
exists  iff  d_c - abs(h) > EPS_LEN
lambda  = sqrt((d_c - h) * (d_c + h))                     (line length l)
N_line  = ( h*uD + lambda*uD_perp ) / d_c                 (left normal of the line)
T_line  = ( lambda*uD - h*uD_perp ) / d_c                 (direction of travel)
Q_start = C_j     - sigma_j    *r_j    *N_line            (leaves element j)
Q_end   = C_{j+1} - sigma_{j+1}*r_{j+1}*N_line            (arrives on element j+1)
```

Derivation:
1. At a point Q of a circle driven with turn sign sigma, the center is on the turn side:
   `C = Q + sigma·r·N` (left turn: center on the left). So `Q = C - sigma·r·N`.
2. Both tangent points belong to the same line, so they share N_line. Then
   `Q_end - Q_start = D - h·N_line`. This must be along T_line, so it is perpendicular to N_line:
   `D·N_line = h`.
3. Write `N_line = (h/d_c)·uD + c'·uD_perp` with `abs(N_line) = 1`. That gives `c' = +-lambda/d_c`.
4. `T_line = rot(-90)(N_line) = c'·uD - (h/d_c)·uD_perp`. The robot must travel forward, from element j toward
   j+1: `(Q_end - Q_start)·T_line = D·T_line = c'·d_c > 0`, so `c' = +lambda/d_c`. The line length is `lambda`.

**Which of the four tangents this picks, and why.** Two circles have up to four common tangents. Each
is fixed by two binary choices: the side of the line each center lies on, and the direction of travel.
The side is fixed by sigma, because the robot must turn the way the corner turns: the center is left of
travel for a left turn. The direction is fixed by `c' > 0` (forward). Exactly one tangent remains.
Outer versus inner is not a separate branch. If `sigma_j = sigma_{j+1}`, then `abs(h) = abs(r_{j+1} - r_j)`
(outer tangent). If they differ, then `abs(h) = r_j + r_{j+1}` (inner tangent). For an endpoint, `abs(h) = r`
(a point-to-circle tangent). No acos or asin is used. The only square root has a non-negative argument
once the existence test passed.

Existence conditions, which follow from `d_c > abs(h)`:
- Endpoint to circle: the point is outside the circle (`abs(P - C) > r`).
- Same signs (outer): `d_c > abs(r_j - r_{j+1})`. Neither circle is inside the other. Overlapping is allowed.
- Opposite signs (inner): `d_c > r_j + r_{j+1}`. The circles must be disjoint.

**Closed form for the first and last leg** (hand derivation, script check). Put P_0 at distance d from
P_1, with C_1 = P_1 + r·b_1. Then `abs(P_0 - C_1)^2 = d^2 + r^2 - 2·d·r·cos(gamma_1)` and
`cos(gamma_1) = sin(tau_turn,1/2)`. So:

```
tangent exists   iff  d > 2*r*sin(tau_turn/2)       i.e.  r < d / (2*sin(tau_turn/2))
tangent length       lambda = sqrt( d * (d - 2*r*sin(tau_turn/2)) )
```

The same holds for the last leg with corner n-2. A script compared the closed-form test with the direct
`abs(P_0 - C) > r` test on 20,000 random triangles and radii: 0 mismatches (script).

Worked example 1, first line of the shared example (point P0, then circle 1 with sigma = -1) (hand, script):
- `D = C - P0 = (2, 0.5858)`, `d_c = 2.0840`, `uD = (0.9597, 0.2811)`, `uD_perp = (-0.2811, 0.9597)`.
- `h = sigma_1·r_1 - 0 = -1.4142`. `lambda = sqrt(4.3431 - 2.0) = 1.5307`. The closed form agrees:
  `sqrt(2.8284·(2.8284 - 2·1.4142·0.7071)) = sqrt(2.3431) = 1.5307`.
- `N_line = (-1.4142·uD + 1.5307·uD_perp)/2.0840 = (-0.8577, 0.5142)`.
  `T_line = (1.5307·uD + 1.4142·uD_perp)/2.0840 = (0.5142, 0.8577)`, which is 59.06 deg.
- `Q_start = P0 = (1, 2)`. `Q_end = C - (-1)·1.4142·N_line = (3 - 1.2130, 2.5858 + 0.7272) = (1.7870, 3.3130)`.
- Deviation from the leg: 59.06 - 45 = 14.06 deg (script).

Worked example 2, inner tangent. Path (0,0),(4,0),(4,4),(8,4), r = 1 at both corners, a left turn then a
right turn (script):
- `C1 = (3.2929, 0.7071)` (sigma +1), `C2 = (4.7071, 3.2929)` (sigma -1). `D = (1.4142, 2.5858)`, `d_c = 2.9473`.
- `h = (-1)(1) - (+1)(1) = -2`. `lambda = sqrt(4.9473·0.9473) = 2.1648`.
- `N_line = (-0.9700, -0.2429)`, `T_line = (-0.2429, 0.9700)`. `Q_start = (4.2629, 0.9500)`, `Q_end = (3.7371, 3.0500)`.
- These match the re-implementation of path-ido (same tangent points).

Worked example 3, outer tangent with unequal radii. Path (0,0),(4,0),(4,4),(0,4), two left turns, with
`.withRadius(1.0)` and `.withRadius(0.5)` (script):
- `C1 = (3.2929, 0.7071)`, `C2 = (3.6464, 3.6464)`. `D = (0.3536, 2.9393)`, `d_c = 2.9605`.
- `h = (+1)(0.5) - (+1)(1.0) = -0.5`. `lambda = 2.9180`. `T_line = (-0.0500, 0.9988)`.
- `Q_start = (4.2916, 0.7571)`, `Q_end = (4.1458, 3.6714)`.
- With equal radii (both 1.0) the same path gives `h = 0` and a line exactly parallel to the leg: x = 4.2929,
  length 2.5858.

Edge cases: `d_c - abs(h) <= EPS_LEN` means the tangent does not exist, so shrink (section 5.7). `lambda < EPS_LEN`
means a zero-length line. Keep it in the table with `l = 0`; the lookup skips zero-length segments
(section 5.9.3). Both elements are points (both r = 0): the line is the leg itself, `T_line = uD`.

Comparison with path-ido:
- **Fact** (Segmant.java:37-48). The endpoint tangent is `atan2(C - P) + sigh·asin(r/abs(P - C))` with length
  `sqrt(abs(P - C)^2 - r^2)`; the last leg is built from the end point and reversed (Path.java:76-77).
  **Inference:** equal to our formula with `h = sigma·r` for the first leg and `h = -sigma·r` for the last.
  The asin argument is unchecked (Segmant.java:38); it stays in range under path-ido's radius caps
  (section 4.1, issue a), not with arbitrary radii.
- **Fact** (Segmant.java:50-63). For circle to circle, `sign2 = -sigh·sigh = -sigma_i·sigma_{i+1}` selects outer (-1)
  or inner (+1); the direction is `acos((r_{i+1} + sign2·r_i)/d_c)` (Segmant.java:52). **Inference:** the same
  tangent as ours, which reaches it without acos and without a branch.

### 5.5 Arc sweep, including sweeps over 180 deg

The arc of corner i runs from `Q_in` (end of line i-1) **through P_i** to `Q_out` (start of line i), turning sigma_i:

```
phi_in  = atan2(Q_in  - C_i)        (= phi0 of the arc)
phi_P   = atan2(P_i   - C_i)        (= atan2(-b_i))
phi_out = atan2(Q_out - C_i)
w_in  = mod2pi( sigma_i * (phi_P   - phi_in) );   if w_in  > 2*pi - EPS_WRAP: w_in  = 0
w_out = mod2pi( sigma_i * (phi_out - phi_P ) );   if w_out > 2*pi - EPS_WRAP: w_out = 0
dphi_i = sigma_i * (w_in + w_out)                  length l = r_i * abs(dphi_i)
s(P_i) = s_start,arc + r_i * w_in                  (where the waypoint sits on the path)
```

Why split at P_i: the arc must contain P_i, so split it there. Each half is the angle travelled in the
direction sigma_i from one point to the next. mod2pi gives exactly that, in [0, 2·pi), with no
ambiguity. The sum can be anything in (0, 4·pi), so sweeps above 180 deg come out right with no special
case. The wrap guard catches the one ambiguous value, where a tangent point coincides with P_i and rounding
gives `2·pi - 1e-15` instead of 0. A sweep close to a full loop is a valid number but a bad path; the
checks in section 5.7 reject it.

Worked example A, the shared example (hand, script):
- `phi_in = atan2(3.3130 - 2.5858, 1.7870 - 3) = atan2(0.7272, -1.2130) = 149.06 deg`. `phi_P = 90 deg`. `phi_out = 30.94 deg`.
- `w_in = mod2pi(-(90 - 149.06)) = 59.06 deg`. `w_out = mod2pi(-(30.94 - 90)) = 59.06 deg`.
- `dphi = -118.12 deg = -2.0616 rad`. Length `= 1.4142·2.0616 = 2.9155`.
- `s(P1) = 1.5307 + 1.4142·1.03078 = 2.9885`.

Worked example B, a sweep over 180 deg. Path (0,0),(4,0),(2,0) with `.withRadius(0.5)`, a U-turn (script):
- `C = (3.5, 0)`, sigma = +1 (no neighbor turn to copy, so the default applies, section 5.8.4).
- `phi_in = -98.21 deg`. `phi_P = 0 deg`. `Q_out = (3.3333, 0.4714)`.
- `dphi = +207.68 deg`, length `0.5·3.6247 = 1.8124`.
- The law-of-cosines angle that path-ido uses (CircleSegmant.java:27-31) gives 152.32 deg for the same chord,
  which is `360 - 207.68`. Its arc length would be 1.3292 instead of 1.8124 (script).

Comparison with path-ido: **Fact** (CircleSegmant.java:13, :26-31). The arc length is `r·acos(...)`, in
[0, pi], so every sweep above 180 deg is wrong (37% of random arcs under path-ido's radius rule, section 4.1).
The construction itself always contained the waypoint; only the length and the progress were wrong.

### 5.6 Shape of the path: bulge, overshoot, and big sweeps

#### 5.6.1 Shape near one corner

**Inference** (geometry; checked by sampling the path every 1 mm (script)):
- The path passes through P_i with direction `theta_T(u_in) + sigma_i·tau_turn,i/2`, exactly halfway between the leg
  directions. The curvature there is the arc's constant `sigma_i/r_i`.
- On both legs the path stays on the **outside** of the corner (the side away from C_i). It does not cut
  inside the corner the way a fillet does. Once the checks in section 5.7 pass, the sweep is at most
  `tau_turn,i + 2·EPS_DEV`.
- Self-crossing at one corner. When the sweep exceeds 180 deg, the line arriving at the arc and the line
  leaving it can cross. The infinite tangent lines at Q_in and Q_out meet behind Q_in and ahead of Q_out,
  each at distance `r·tan((360 deg - abs(dphi))/2)` (hand), so the two lines cross only if both are longer
  than that. Random check, 5,000 paths of 3 to 7 points on a 16 x 8 m field (script): with the default
  TAU_STOP = 150 deg, 0 crossings at a single corner (20 arcs sweep more than 180 deg); with TAU_STOP
  disabled, 152 crossings. Single-corner endpoint tangents never crossed (for example the U-turn of
  section 5.5 needs 2.03 m on both lines and has 3.46 m and 1.41 m). The follower's window (section 8.2.3)
  is designed for this case, so a crossing is allowed.
- Max outward deviation from the polyline:

  ```
  e_bulge = r_i * (1 - cos(tau_turn,i/2))
  ```

  Derivation: the distance from C_i to the incoming leg line is `r·sin(gamma) = r·cos(tau_turn/2)`. The point
  of the circle furthest outside that line is at distance `r - r·cos(tau_turn/2)` outside it. That point is
  where the path direction equals the leg direction. It lies on the driven arc whenever the incoming line
  deviates outward, which always holds for an endpoint tangent. The outgoing side is symmetric.
- Overshoot past P_i along the incoming direction, for tau_turn >= 90 deg: `r·(1 - sin(tau_turn/2))`. This is
  the circle point `C + r·u_in`, which is on the arc when `tau_turn + (outgoing deviation) >= 90 deg`. At
  tau_turn = 180 deg it is 0, because P_i is then the tip of the circle.

Sampled check, max distance to the polyline against the formula (legs 4 m, radius forced by override) (script):

| tau_turn | r = 0.25 | r = 0.5 | r = 1.0 | classic fillet, distance by which it misses the corner, r = 1.0 |
|---|---|---|---|---|
| 30 deg | 0.0085 / 0.0085 | 0.0170 / 0.0170 | 0.0341 / 0.0341 | 0.0353 |
| 60 deg | 0.0335 / 0.0335 | 0.0670 / 0.0670 | 0.1340 / 0.1340 | 0.1547 |
| 90 deg | 0.0732 / 0.0732 | 0.1464 / 0.1464 | 0.2929 / 0.2929 | 0.4142 |
| 120 deg | 0.1250 / 0.1250 | 0.2500 / 0.2500 | 0.5000 / 0.5000 | 1.0000 |
| 150 deg | 0.1853 / 0.1853 | 0.3706 / 0.3706 | 0.7412 / 0.7412 | 2.8637 |

(sampled / formula, in meters. The fillet column is `r·(1/cos(tau_turn/2) - 1)`.) In the shared example the
bulge is `1.4142·(1 - cos 45 deg) = 0.4142 m`, at the path point (2, 3.5858) (hand, script).

#### 5.6.2 Sweep against turn angle

The sweep is `tau_turn + (incoming deviation) + (outgoing deviation)`. The deviations grow with `r/d` and with
tau_turn. Single corner, legs 4 m, path-ido's rule `min(10, d/2)` against the rule of section 5.7 (script):

| tau_turn | path-ido r | path-ido sweep | path-ido bulge | path-ido line deviation | 5.7 r | 5.7 sweep | 5.7 bulge | 5.7 deviation |
|---|---|---|---|---|---|---|---|---|
| 45 | 2.000 | 50.5 | 0.152 | 2.7 | 2.000 | 50.5 | 0.152 | 2.7 |
| 90 | 2.000 | 118.1 | 0.586 | 14.1 | 1.707 | 111.5 | 0.500 | 10.8 |
| 120 | 2.000 | 180.0 | 1.000 | 30.0 | 1.000 | 138.6 | 0.500 | 9.3 |
| 135 | 2.000 | 218.1 | 1.235 | 41.5 | 0.810 | 152.8 | 0.500 | 8.9 |
| 150 | 2.000 | 261.4 | 1.482 | 55.7 | 0.675 | 167.2 | 0.500 | 8.6 |
| 170 | 2.000 | 326.0 | 1.826 | 78.0 | 0.548 | 186.7 | 0.500 | 8.3 |
| 179 | 2.000 | 356.6 | 1.983 | 88.8 | 0.504 | 195.4 | 0.500 | 8.2 |
| 180 | 1.600 (shrunk, see 5.8.4) | 263.6 | 1.600 | 41.8 | 0.500 | 196.4 | 0.500 | 8.2 |

(Angles in degrees, lengths in meters. The section 5.7 columns are before TAU_STOP is applied; with the
default TAU_STOP = 150 deg the 170, 179 and 180 rows become stops.) The path-ido sweeps at 90/135/170 deg
(118.1/218.1/326.0) match the re-implementation of path-ido (section 4.1).

#### 5.6.3 Is a 326 deg loop acceptable? No.

**Recommendation:** reject it. At a 170 deg turn with path-ido's radius:
- The path is 11.87 m against an 8.0 m polyline (+48%) (script).
- The robot bulges 1.83 m out past the corner, into space the user never drew.
- It leaves P0 driving 78 deg away from the leg, nearly sideways.

Field elements and other robots make that unsafe. Section 5.7 prevents it in three ways:
1. The bulge cap `r <= D_BULGE/(1 - cos(tau_turn/2))` limits the bulge to 0.5 m and the sweep to 186.7 deg at 170 deg.
2. The deviation and sweep checks (EPS_DEV) reject any line more than 30 deg off its leg, and any arc sweeping
   more than `tau_turn + 60 deg`.
3. TAU_STOP turns very sharp automatic corners into stops (next section).

#### 5.6.4 Hairpin or stop: TAU_STOP

For a holonomic swerve robot, a sharp corner can be driven two ways. One is an arc (hairpin) at reduced
speed. The other is stopping exactly at P_i and accelerating in the new direction. The robot's heading is
independent of its travel direction (section 6), so "turn" at a stop only means that the travel direction
changes while v = 0.

Time comparison from a crude forward/backward speed profile. It uses `v <= sqrt(a_lat/abs(kappa))`,
v = 0 at the stop and at both ends, legs of equal length, v_max = 4 m/s and a_acc = a_dec = 3 m/s^2.
Each cell is arc time / stop time, in seconds (script):

| legs, a_lat | 90 deg | 110 | 120 | 130 | 140 | 150 |
|---|---|---|---|---|---|---|
| 1.5 m, 2 m/s^2 | 2.83/2.83 | 3.04/2.83 | 3.17/2.83 | 3.24/2.83 | 3.36/2.83 | 3.44/2.83 |
| 1.5 m, 4 m/s^2 | 2.38/2.83 | 2.50/2.83 | 2.57/2.83 | 2.67/2.83 | 2.74/2.83 | 2.82/2.83 |
| 3 m, 2 m/s^2 | 4.00/4.00 | 4.26/4.00 | 4.37/4.00 | 4.47/4.00 | 4.56/4.00 | 4.63/4.00 |
| 3 m, 4 m/s^2 | 3.37/4.00 | 3.59/4.00 | 3.69/4.00 | 3.79/4.00 | 3.88/4.00 | 3.96/4.00 |
| 6 m, 2 m/s^2 | 5.68/5.67 | 5.94/5.67 | 6.04/5.67 | 6.13/5.67 | 6.21/5.67 | 6.29/5.67 |
| 6 m, 4 m/s^2 | 4.98/5.67 | 5.24/5.67 | 5.34/5.67 | 5.44/5.67 | 5.52/5.67 | 5.60/5.67 |

**Inference:** the crossover depends mostly on a_lat relative to a_acc and a_dec, not on the angle alone. At
150 deg the arc is at best break-even (a_lat = 4) and loses clearly when a_lat = 2. A stop also keeps the
robot on the user's polyline, with no bulge. **Recommendation** (accepted by the user): TAU_STOP = 150 deg, an
internal constant since the simplification pass. An explicit `.withRadius(r)` ignores TAU_STOP, which is how the
user asks for a hairpin. **Uncertain:** v2 could choose per corner by comparing profile times.

### 5.7 Radius selection

#### 5.7.1 Feasibility conditions

With waypoint-on-circle there is no "tangent point inside the leg" in the fillet sense, because the tangent
points are off the legs. The equivalent conditions, which together guarantee a valid, forward-running path:

| ID | Condition | Meaning |
|---|---|---|
| F1 | every line j: `d_c,j - abs(h_j) > EPS_LEN` | the tangent exists: endpoint outside the circle (`d > 2r·sin(tau_turn/2)`), circles disjoint for opposite signs, neither circle inside the other for equal signs |
| F2 | every line j: `abs(angle(T_line,j, u_leg,j)) <= EPS_DEV` | the line runs forward along its leg, so consecutive arcs follow each other along the leg and do not overlap or double back |
| F3 | every arc i: `abs(abs(dphi_i) - tau_turn,i) <= 2·EPS_DEV` | the arc turns roughly by the corner angle: no near-full loops and no wrong-way arcs |

Sufficient, closed-form guarantees for F1 (**Inference**, from section 5.4):
- End leg: `r <= d/2` gives `d >= 2r > 2r·sin(tau_turn/2)` for any tau_turn < 180 deg. So the endpoint tangent exists.
- Shared leg: `r_i <= d/4` and `r_{i+1} <= d/4`. Each circle lies in the disk of radius `2r` around its own
  waypoint, and these disks do not overlap, so the circles are disjoint and both inner and outer tangents exist.

This is why path-ido's divisors 2 (end leg) and 4 (shared leg) work for existence (Path.java:39). They do not
control F2/F3, the quality. That is what the bulge cap and the checks add.

#### 5.7.2 Automatic rule

```
k(j) = 2 if leg j touches P_0, P_{n-1} or a user STOP point (section 5.8.7), else 4
       (straight middle points are set aside before this rule (5.8.3), so a leg here joins two kept points)
r_auto,i = min( R_MAX,
                d_{i-1} / k(i-1),
                d_i     / k(i),
                D_BULGE / (1 - cos(tau_turn,i/2)) )        (term skipped when tau_turn,i < EPS_ANG)
if tau_turn,i > TAU_STOP: r_auto,i = 0                      (stop corner)
```

- `d/k`: guarantees F1 (section 5.7.1).
- `D_BULGE/(1 - cos(tau_turn/2))`: caps the outward bulge at D_BULGE (section 5.6.1). With long legs it is the
  binding term above tau_turn = 82.8 deg, where it equals R_MAX: `0.5/(1 - cos(tau_turn/2)) = 2` at
  `cos(tau_turn/2) = 0.75` (hand).
- R_MAX: big radii are fast but sweep far from the polyline. On a 16.5 x 8 m field a 10 m radius (path-ido,
  Path.java:12) is effectively no cap.

Worked numbers (hand, script):
- Shared example: `min(2, 2.8284/2, 2.8284/2, 0.5/0.29289 = 1.7071) = 1.4142`. path-ido gives the same
  value: `min(10, 1.4142, 1.4142)`.
- 90 deg corner with 15 m and 7 m legs: `min(2, 7.5, 3.5, 1.7071) = 1.7071`, set by the bulge term.
- 30 deg corner, legs 4 m: `min(2, 2, 2, 0.5/0.03407 = 14.7) = 2.0`.

#### 5.7.3 Override, shrink loop, and stop fallback

```
initial r_i:
    user stop (section 5.8.7)  : r_i = 0, never changed
    .withRadius(x), x >= R_MIN : r_i = x       (ignores R_MAX, D_BULGE, TAU_STOP, d/k; NOT the checks)
    .withRadius(x), x <  R_MIN : r_i = 0       (explicit stop corner, e.g. .withRadius(0))
    otherwise                  : r_i = r_auto,i;   if r_i < R_MIN: r_i = 0
loop (at most MAX_ITER times):
    build circles, lines (5.4) and arcs (5.5)
    bad = corners touching a line that fails F1 or F2, plus corners whose arc fails F3   (only corners with r > 0)
    if bad is empty: done
    targets = automatic corners in bad;  if none: targets = bad (override corners) and record a warning
    for i in targets: r_i = SHRINK * r_i;  if r_i < R_MIN: r_i = 0  (stop corner)
```

`.withRadius(x)` is ignored, with warning W4 (section 9.12), on P_0, on P_{n-1} and on a straight waypoint
(tau_turn < EPS_ANG); a straight waypoint then follows section 5.8.3. `.withRadius(r > 0)` together with a user
stop is an error (V30, section 9.12).

- **Termination** (**Inference**). Each pass shrinks at least one radius by a factor of 0.8, or sets it to 0.
  A radius reaches R_MIN after at most `ceil(log(r0/R_MIN)/log(1.25))` passes, which is 17 for r0 = 2 m.
  With every radius at 0 the path is the polyline with stops, which always passes F1 to F3: the lines are the
  legs and there are no arcs. So `17·(n-2) + 1` passes are enough for automatic radii. For overrides, use
  `ceil(log(r_max_override/R_MIN)/log 1.25)` per corner. **Recommendation:** MAX_ITER = 1000 as a hard guard.
- **Shrinking fixes the failures** (**Inference**): the deviations go to 0 as `r/d` goes to 0, so lines approach
  their legs and sweeps approach tau_turn.
- **Warnings** (**Recommendation**): W1 (corner became a stop) and W2 (radius shrunk), section 9.12, reported once
  at build time through `PathLogger.alert` and kept in `Path.warnings()`. Never throw at run time.
- **Stop fallback**: a corner with `r_i = 0` becomes a zero-length STOP segment, and the speed profile gives v = 0
  at its s.

Random check: 5,000 random paths with 3 to 7 points uniform on a 16 x 8 m field, TAU_STOP disabled (script):
- Every path built and passed F1 to F3. At most 18 passes were needed. The deviation checks shrank 866 of
  15,013 corners. 346 corners became stops.
- Each waypoint was on its arc to within 4.4e-15 m. The position and tangent gap at segment joins was at most
  3.5e-14 (G1 holds).
- 2,338 of 14,667 arcs swept more than 180 deg, with a maximum of 231.5 deg.
- With TAU_STOP = 150 deg: 20 arcs swept more than 180 deg (max 191.2 deg), and 5,759 corners became stops.
  Random polylines have far more sharp corners than real FRC paths, so the stop count is not representative.
- The same random set under path-ido's rule: 5,846 of 14,432 arcs swept more than 180 deg, and 3,176 lines
  deviated more than 30 deg from their leg.

### 5.8 Degenerate cases

| Case | Detection | Required behavior |
|---|---|---|
| All points within EPS_DUP of each other | one point left after merging | constructor throws (V6, section 9.12). A path with L = 0 does not exist |
| n = 2 | two points | one line P0 to P1, m = 1. **Fact:** path-ido does the same (Path.java:65-67) |
| Duplicate / near-duplicate | `abs(P_{k+1} - P_k) < EPS_DUP` | drop the later point; merge its attributes into the kept point, later value wins, warning W3 |
| Collinear, straight on | `tau_turn,i < EPS_ANG` | set aside before building: a PASS marker at the closest path point, warning W5 if the path misses it by more than PASS_TOL = 0.01 m (5.8.3) |
| U-turn | `tau_turn,i > pi - EPS_ANG` | stop by default (TAU_STOP). With `.withRadius`: teardrop arc, sigma from a neighbor (5.8.4) |
| Very sharp corner | tau_turn near pi | bulge cap, then TAU_STOP gives a stop |
| Very long legs | `d >> R_MAX` | radius independent of d; lines lie almost on the legs; nothing special |
| Point inside neighbor's circle | F1 fails | shrink (5.7.3) |
| Overlapping circles | `d_c < r_j + r_{j+1}` | same signs: allowed if F2/F3 pass. Opposite signs: F1 fails, so shrink |
| Very short leg | `d/4 < R_MIN` | both corners on that leg become stops |
| Robot exactly at an arc center | `abs(p - C) < EPS_CENTER` | projection keeps s_prev, clamped to the arc (5.10.2) |
| User stop `.stop()` / `withVelocity(0)` / `.stop(dwell)` | flag on a middle waypoint | no arc: STOP entry, neighbors treat P_i as an endpoint (5.8.7) |
| Very short path, L < L_short | L after building | built normally; the follower starts in CAPTURE_END (section 8.12). No division by L anywhere produces NaN, because L >= EPS_DUP |

#### 5.8.1 n = 2

One line segment, `T = unit(P1 - P0)`, `kappa = 0`, `L = d_0`. Example: (0,0) to (3,4) gives `L = 5.0000`,
`T = (0.6, 0.8)` (script).

#### 5.8.2 Duplicates

Merge points closer than EPS_DUP = 1 mm before anything else. Otherwise `unit()` divides by about 0, and
path-ido divides by zero at Path.java:50. Example: (0,0),(2,0),(2.0005,0),(2,2) becomes (0,0),(2,0),(2,2),
with L = 4.2263 and r = 1.0 (script). The merged waypoint keeps one s value. Events and attributes that
referred to the dropped point map to the kept point; the user's indices stay valid (section 5.9.5).

#### 5.8.3 Collinear (straight) waypoints: PASS markers

**Decision** (user, simplification pass). A middle waypoint with `tau_turn,i < EPS_ANG` that is not a user stop is a
**PASS marker**:
1. It is set aside before the corner construction. The geometry is built from the remaining points, so the path is
   exactly the path without that point (the two legs through it form one straight leg).
2. Its s is the s of the closest point of the built path, searched between the s of its kept neighbors. That s is a
   span boundary (section 7.3.1) and carries the point's angle, `withVelocity` and events. A PASS marker adds no
   segment to the table.
3. The miss distance `d_i = abs(x(s(P_i)) - P_i)` is checked. If `d_i > PASS_TOL = 0.01 m`, the build emits warning
   W5, which names the point and d_i and suggests `.stop()` on it or removing it. The path is **not** changed.

This **relaxes "every waypoint lies exactly on the path" for straight middle points only.** Next to an arc corner
the line is rotated outward (section 5.2), so a point on the leg can be missed. Every point where the path turns,
every stop and both endpoints are still exactly on the path. A user stop on a straight point (`.stop()`) works as
before: it is a STOP entry and both lines end exactly at P_i (section 5.8.7). No extra arc is added through such a point.

Results (script):
- (0,0),(2,0),(4,0): one line of 4.0 m; PASS marker at s = 2.0, miss 0, no warning.
- (0,0),(3,0),(3,3),(3,6),(6,6): (3,3) is set aside; r = 1.5 at both corners; the inner tangent between the two
  opposite corners passes exactly through (3,3) by symmetry: s(P2) = 6.3395, miss 0, L = 12.6790, no warning.
- (0,0),(1,0),(2,0),(2,2): the path is that of (0,0),(2,0),(2,2) (r = 1.0, L = 4.2263). The first line heads
  -14.06 deg and misses (1,0) by 0.2429 m (marker at s = 0.9700): warning W5. With `(1,0).stop()` the point is a STOP
  and the path passes through it (GEO-28).
- (0,0),(3,0),(3,3),(3,6),(0,6), two left turns: the middle line is x = 3.4393, so (3,3) is missed by 0.4393 m:
  warning W5.

Prefer events at a fraction or at a turning point over extra points on straight legs (warning W5 says so).

#### 5.8.4 U-turn (tau_turn = 180 deg)

- The construction is well defined: `b = -u_in` (back along the incoming leg) and `C = P_i - r·u_in`. The circle's
  far tip is P_i. The robot drives a teardrop: out on one side of the leg, around the tip through P_i with
  direction `theta_T(u_in) + sigma·90 deg`, and back on the other side.
- sigma is 0 from the cross product, so it must be chosen. **Recommendation:** take sigma from the nearest proper
  corner, the next one first and then the previous one. A U-turn followed by a right turn then also turns right,
  which avoids an S-wiggle. If there is no such corner, use +1 (CCW). This default is not mirror-symmetric
  (section 5.13).
- Because tau_turn > TAU_STOP, an automatic U-turn is a **stop** by default. The robot stops at P_i and drives
  back, which suits a swerve. A teardrop happens only with `.withRadius(r)`.
- Teardrop numbers, (0,0),(4,0),(0,0) with r = 0.5: C = (3.5, 0), sweep +196.43 deg, L = 8.6423, line deviation
  8.21 deg. When P_{i+1} = P_{i-1}, the two lines meet at the shared endpoint (script).
- With `.withRadius`, the endpoint condition `d > 2r·sin(90 deg) = 2r` must hold (F1). path-ido's end-leg rule
  `r = d/2` sits exactly on this limit, so its U-turn tangent does not exist (division by a near-zero length).
  path-ido's rule plus our shrink loop ends at r = 1.6 for 4 m legs (section 5.6.2, row 180) (script).

#### 5.8.5 Very sharp, very long, inside, overlapping

- Very sharp (tau_turn 150 to 180 deg): see sections 5.6.2 to 5.6.4. The bulge term sets r near D_BULGE, and
  TAU_STOP gives a stop.
- Very long legs: (0,0),(15,0),(15,7) gives r = 1.7071 from the bulge term, line deviations of 2.08 and 5.00 deg,
  and L = 22.2981 (script). Doubles have about 1e-15 relative precision, so on a 17 m field rounding is about
  1e-14 m. No numerical concern.
- Point inside a neighbor's circle. With automatic radii this cannot happen for endpoints (section 5.7.1). With
  overrides it can: (0,0),(2,0),(2,2) with `.withRadius(5)` has P0 inside, since `5 > 2/(2·sin 45 deg) = 1.414`.
  The shrink loop reaches 1.3107 after 7 passes, where F1 to F3 pass (line deviation 26.8 deg) (script).
- Overlapping circles, same signs. (0,0),(4,0),(4,4),(0,4) with both radii 2.0 has `d_c = 1.1716 < 4`. The outer
  tangent exists, and the path is valid (the middle line has x = 4.5858, length 1.1716, sweeps 104.06 deg)
  (script). Kept.
- Overlapping circles, opposite signs. (0,0),(4,0),(4,1),(8,1) with both radii 1.0 has no inner tangent. Shrink
  to 0.3277 after 6 passes (`1.0·0.8^5`). The automatic rule gives r = 0.25 (d/4) and needs no shrinking (script).

#### 5.8.6 Numerical tolerances

All epsilons are in section 10. The rationale for each:
- EPS_DUP = 1 mm is far below pose accuracy (about 1 to 3 cm) and far above rounding.
- EPS_ANG = 1e-4 rad: a middle point that turns by less than this is straight, a PASS marker (section 5.8.3).
  Treating it as straight moves the leg by at most `d·EPS_ANG/2`, 0.4 mm on an 8 m leg.
- EPS_LEN = 1e-6 m guards the square root in section 5.4 and zero-length segments.
- EPS_WRAP = 1e-9 rad is far above rounding (about 1e-15) and far below any real sweep (at least EPS_ANG).

#### 5.8.7 User stops

`.stop()`, `withVelocity(0)` and `.stop(dwellSeconds)` on a middle waypoint P_i **remove that corner's arc**
(brief decisions 15 and 17). Build rule:
- P_i becomes a STOP entry: zero length, `r_i = 0`, sigma ignored. This is the same entry as an automatic stop
  corner or `.withRadius(0)`. It carries a `userStop` flag and the dwell time for the follower. The geometry
  ignores the dwell. `s(P_i) = s_start` of the entry.
- The check happens before the radius rules. A user stop is never given a radius, never shrunk and never a PASS
  marker, even when P_i is collinear (tau_turn,i < EPS_ANG). The robot is at rest there, so any direction change
  between the two lines is allowed.
- `.withRadius(r)` with r > 0 on the same point is an **error**: `IllegalArgumentException` from the `Path`
  constructor (V30). `.withRadius(0)` with `.stop()` is allowed (same meaning).
- Both lines touching P_i are point-to-element tangents (section 5.4 with `r = 0, sigma = 0` at P_i). If the
  neighbor is also a point, the line is the leg itself.
- Neighboring radii: a leg that touches a user stop counts as an end leg, so it uses `k = 2` instead of 4 in
  section 5.7.2. That is safe, because only one circle uses that leg, and `r <= d/2` guarantees the endpoint
  tangent exists (section 5.7.1). Neighbor corners keep their own tau_turn, computed from the waypoints.
- Checks: F1 for a line touching P_i is the endpoint test `d > 2·r·sin(tau_turn/2)` for the neighbor corner. F2
  still applies to that line. F3 does not apply to P_i (no arc). The shrink loop never targets a user stop.
- On P_0, `.stop()` changes nothing, because the path starts at rest (warning W4). On P_{n-1} it is the default.
  Automatic stops from TAU_STOP or shrinking keep `k = 4` on their legs. Re-growing neighbors after a shrink-made
  stop is not done in v1.

Worked example. P0 = (1,2), P1 = (3,4).stop(), P2 = (5,2) (hand, script):
- Corner 1 gets no circle. Line 0 is the leg: A = (1, 2), T = (0.7071, 0.7071), l = 2.8284.
- STOP at (3, 4), with s_start = 2.8284 and l = 0.
- Line 2: A = (3, 4), T = (0.7071, -0.7071), l = 2.8284.
- L = 5.6569, s(P1) = 2.8284, kappa = 0 everywhere. The direction of travel turns 90 deg at rest.
- Compare the automatic version (section 5.11): L = 5.9769, with P1 at s = 2.9885.

Effect on a neighbor. In (0,0),(4,0),(4,4).stop(),(0,4), corner 1 now has two end legs, so
`r_1 = min(2, 4/2, 4/2, 1.7071) = 1.7071`. Without the stop it is 1.0, because k = 4 on the shared leg.
The segments are: line 2.5186; arc C = (2.7929, 1.2071), dphi = +111.51 deg, length 3.3224; line from
(4.4700, 1.5257) to (4,4), length 2.5186; STOP at s = 8.3596; line 4.0. L = 12.3596 (script).

### 5.9 Segment table and arc-length parameterization

#### 5.9.1 Table layout

For n >= 3 the table has `m = 2n - 3` entries in order line, corner, line, corner, ..., line. The line on leg j is
segment `2j`, and corner i is segment `2i - 1`. A corner entry is an ARC or a STOP (zero length, the speed profile
gives v = 0). Keeping zero-length STOP entries keeps this index map fixed. Here n counts the points left after
merging duplicates and setting straight middle points aside (5.8.3).

```
segment k: type, s_start,k, l_k, plus
  LINE : A (start point), T (unit direction)                    end = A + l*T
  ARC  : C, r, phi0, dphi (signed), sigma = sign(dphi), s_wp (s of its waypoint)
  STOP : point P, T_in, T_out, corner index, userStop flag, dwell
s_start,0 = 0;  s_start,k+1 = s_start,k + l_k;  L = s_start,m-1 + l_m-1
```

#### 5.9.2 Evaluation at s

With `q = s - s_start,k`:

| | LINE | ARC |
|---|---|---|
| x(s) | `A + q·T` | `C + r·(cos phi, sin phi)`, with `phi = phi0 + sigma·q/r` |
| T(s) | `T` | `sigma·(-sin phi, cos phi)` |
| N(s) | `rot90(T)` | `rot90(T(s)) = -sigma·(cos phi, sin phi)` (points to the center for a left turn, away from it for a right turn) |
| kappa(s) | 0 | `sigma/r` |
| theta_T(s) | `atan2(T.y, T.x)` | `phi + sigma·pi/2` |

Derivation for the arc: `x = C + r·(cos phi, sin phi)` and `dphi/ds = sigma/r` give `dx/ds = sigma·(-sin phi, cos phi)`,
which has unit length. `dT/ds = (sigma/r)·rot90(T) = kappa·N`, the Frenet relation with this sign convention.
`kappa·N = -(x - C)/r^2` always points at the center. theta_T is the direction of travel, not the robot heading;
the heading is a separate profile (section 6). STOP: `x = P`; T, N and kappa come from the next
segment (right limits, section 5.9.3).

Worked example on the shared example (script, s = 3.0 checked by hand):
- s = 1.0 on the line: `x = (1,2) + 1.0·(0.5142, 0.8577) = (1.5142, 2.8577)`, `kappa = 0`, theta_T = 59.06 deg.
- s = 3.0 on the arc: `q = 1.4693`. `phi = 149.06 deg - 1.4693/1.4142 rad = 149.06 - 59.53 = 89.53 deg`.
  `x = (3 + 1.4142·0.0082, 2.5858 + 1.4142·1.0000) = (3.0115, 4.0000)`. `T = (1.0000, -0.0082)`,
  `N = (0.0082, 1.0000)`, `kappa = -0.7071`, theta_T = -0.47 deg.
- s = 2.9885 = s(P1): `x = (3, 4)`, `T = (1, 0)`. The waypoint is on the path, direction 0 deg.

#### 5.9.3 Finding the segment for s

```
segmentIndexAt(s):
    s = clamp(s, 0, L)
    k = (largest index with s_start,k <= s)        // binary search on the non-decreasing s_start array
    while k > 0 and l_k == 0: k = k - 1            // never return a zero-length entry
    return k
```

At a boundary the later segment is returned, because it also starts at s. Zero-length entries are skipped by
stepping back to the previous positive-length segment, which is evaluated at its end. So x(s) is continuous, and
T(s) is right-continuous except across a STOP. O(log m) time. Example: s_start = [0, 1.5307, 4.4462]. s = 3.0 gives
k = 1 (arc). s = 4.4462 gives k = 2. s = 7 is clamped to L = 5.9769, which gives k = 2.

Left limits. At a stop the follower needs T and N of the leg it arrives on (section 8.6). The table therefore
also offers `evaluateLeft(s)`, which picks the largest positive-length segment with `s_start,k < s`. Away from
STOP entries both evaluations agree because the path is G1.

#### 5.9.4 s of every waypoint

These s values are needed by the heading profile, `withVelocity`, stops and events.
- `s(P_0) = 0` and `s(P_{n-1}) = L`.
- ARC corner: `s_wp = s_start + r·w_in` (section 5.5).
- STOP: `s_start` of the zero-length entry.
- PASS marker: the s of the closest path point (section 5.8.3).
- Merged duplicates: the s of the kept point.

Example: s(P1) = 2.9885 (script). It is the midpoint of the arc here only because the corner is symmetric;
in general it is not.

#### 5.9.5 Events: fraction and point index to s

Events are converted to s when they are added to the `Path` (section 9.7). There are two kinds (**Decision**, user,
simplification pass):
- fraction f in [0, 1]: `s_event = f·L`;
- point index i (the user's index, before merging): `s_event = s(P_i)` from section 5.9.4; a merged point maps to
  the kept point.

Example (shared example): fraction 0.5 gives 2.98845 m; point 1 gives 2.9885 m; point 2 gives 5.9769 m.
Flips keep arc length, so event s values are the same on both alliances (section 5.13).

### 5.10 Projection of the robot position onto the path

The geometry projects a point onto one segment and searches a window `[sMin, sMax]` that the follower chooses
(section 8.2.3). The follower's window design is final; the geometry has no window of its own.

#### 5.10.1 Line segment

```
q  = clamp( (p - A)·T, 0, l )
s* = s_start + q;  x* = A + q*T
e_n = (p - x*)·N;   e_t = (p - x*)·T        (e_t != 0 only when clamped)
```

Example: p = (2, 3) on line 0. `(p - A) = (1, 1)`, `q = 0.5142 + 0.8577 = 1.3719`, `s* = 1.3719`.
`e_n = (1,1)·(-0.8577, 0.5142) = -0.3435`: the robot is 0.34 m to the **right** of the path (hand, script).

#### 5.10.2 Arc segment

```
vC = p - C
if abs(vC) < EPS_CENTER:                    // every arc point is equally near
    q = clamp(s_prev - s_start, 0, l)       // keep progress
else:
    beta = mod2pi( sigma * (atan2(vC) - phi0) )     // angle travelled from the arc start, in [0, 2pi)
    if beta <= abs(dphi):                q = r*beta
    elif beta - abs(dphi) < 2pi - beta:  q = l          // past the end: closer to the end
    else:                                q = 0          // before the start
s* = s_start + q;   x* = x(s*)
inside the angular range:  e_n = sigma*(r - abs(vC))      (closed form of (p - x*)·N)
otherwise:                 e_n = (p - x*)·N(s*),  e_t = (p - x*)·T(s*)
```

The `mod2pi(sigma·(...))` line measures the angle from the arc start in the direction of travel, so arcs over
180 deg work. Derivation of `e_n = sigma·(r - abs(vC))`: `x* = C + r·vC/abs(vC)`, so `p - x* = (abs(vC) - r)·vC/abs(vC)`.
Also `N = -sigma·vC/abs(vC)`, which gives `e_n = -sigma·(abs(vC) - r)`.

Examples on the shared arc (sigma = -1, phi0 = 149.06 deg, abs(dphi) = 118.12 deg) (hand, script):
- p = (3, 4.5): `vC = (0, 1.9142)`, atan2 = 90 deg. `beta = mod2pi(-(90 - 149.06)) = 59.06 deg`, within the sweep.
  `q = 1.4578`, `s* = 2.9885`. `e_n = -1·(1.4142 - 1.9142) = +0.5000`: robot 0.5 m left of the path, outside the
  circle, above P1.
- p = (2.5, 1.0): atan2(vC) = -107.5 deg, `beta = 256.6 deg`. That is past the end by 138.4 deg and before the start
  by 103.4 deg, so clamp to the start (q = 0). The windowed search then picks line 0 if it is nearer.
- p = C = (3, 2.5858) with s_prev = 2.5: `s* = 2.5`, `e_n = sigma·r = -1.4142`, no NaN.

#### 5.10.3 STOP entries

Distance to the point P, with `s* = s_start`. These never beat the neighboring lines, which end at the same point,
so they matter only for tie-breaking.

#### 5.10.4 Windowed search

```
project(p, sMin, sMax, s_prev):
    best = none
    for each segment k overlapping [sMin, sMax]   (first and last by two binary searches; usually 1 to 3 segments)
        s_k  = clamp(projectOnSegment(k, p, s_prev), sMin, sMax)
        dist = abs(p - x(s_k))
        keep the smallest dist; on a tie (within 1e-12 m) keep the s_k closest to s_prev
    s* = best;  e_n = (p - x(s*))·N(s*)
    return (s*, e_n)
```

- Why a window: the path can come back near itself (a teardrop, a hairpin, a self-crossing). A global nearest
  point can then jump to a far part of the path. Example: (0,0),(4,0),(2,0) with `.withRadius(0.5)`, robot at
  (2, 0.05) with s_prev = s_hi = 2.0. The global search returns s* = 6.6740, near the **end** of the path, 0.047 m
  away; the follower would think it had finished. The follower's window [1.97, 2.15] (eps_back 0.03, ahead 0.15)
  returns s* = 1.9723 on the first leg, with e_n = +0.3352 (script).
- e_t is computed by the follower (section 8.2.3); it is non-zero when the result is clamped at a window edge.
- e_n sign: `+` means the robot is left of the path. In the velocity command `V = v_t·T + c·N` with
  `c = -k_n·e_n`, a positive e_n pushes the robot right, back toward the path.

### 5.11 Full worked example: P0 = (1,2), P1 = (3,4), P2 = (5,2), automatic radius

All values are (script); the items marked (hand) in sections 5.3 to 5.5 were checked by hand.

| Quantity | Value |
|---|---|
| tau_turn,1, sigma_1 | 90 deg, -1 (right turn) |
| b_1 | (0, -1) |
| d_0, d_1 | 2.8284, 2.8284 |
| r_1 | `min(2.0, 1.4142, 1.4142, 1.7071) = 1.4142` (path-ido: same) |
| C_1 | (3.0000, 2.5858) |
| endpoint test | `2r·sin(45 deg) = 2.0000 < 2.8284`: tangent exists. Largest r with a tangent: 2.0 |
| Q_in (end of line 0) | (1.7870, 3.3129) |
| Q_out (start of line 2) | (4.2130, 3.3129) |
| line directions / deviation | +59.06 deg / +14.06 deg; -59.06 deg / -14.06 deg (F2: 14.06 <= 30, passes) |
| arc phi0, phi_P, phi_out | 149.06 deg, 90.00 deg, 30.94 deg |
| w_in, w_out, dphi | 59.06 deg, 59.06 deg, -118.12 deg (F3: 118.12 - 90 = 28.12 <= 60, passes) |
| bulge, overshoot past P1 | 0.4142 m (at (2, 3.5858)), 0.4142 m |

Segment table:

| k | type | s_start | l | geometry | kappa |
|---|---|---|---|---|---|
| 0 | LINE | 0.0000 | 1.5307 | A = (1, 2), T = (0.5142, 0.8577) | 0 |
| 1 | ARC | 1.5307 | 2.9155 | C = (3, 2.5858), r = 1.4142, phi0 = 149.06 deg, dphi = -118.12 deg, s_wp = 2.9885 | -0.7071 |
| 2 | LINE | 4.4462 | 1.5307 | A = (4.2130, 3.3129), T = (0.5142, -0.8577) | 0 |

L = 5.9769 m, against a polyline length of 5.6569 m (+5.7%). Waypoint s values: 0, 2.9885, 5.9769.

### 5.12 G1 versus G2

#### 5.12.1 The curvature jump, in numbers

On the shared example, kappa jumps from 0 to -0.7071 1/m at s = 1.5307, and back to 0 at s = 4.4462. T is
continuous (G1), but curvature is not (no G2). The lateral acceleration `v^2·abs(kappa)` therefore jumps:
- At v = 2 m/s: from 0 to 2.83 m/s^2 between two 20 ms loops, a discrete jerk of about 141 m/s^3.
- At v = 3 m/s: 6.36 m/s^2. With aLat = 4 m/s^2, the speed profile caps the arc at `sqrt(4·1.4142) = 2.3784 m/s`.
  That cap must already hold at s = 1.5307, which the backward pass ensures (section 7.5).

Toy model ((script); **Inference** about the real robot). A point robot follows `V = v·T - 3·e_n·N`, with a first-order
velocity loop of time constant 0.05 s and an acceleration slew limit of 20 m/s^3. The path is a 3 m line, then an
optional clothoid of length Lc, then the arc. The table shows peak abs(e_n) in the first 1.5 m after the arc entry,
and e_n late on the arc:

| v, r (a_lat) | Lc = 0 (G1) | Lc = 0.5 m | Lc = 1.0 m |
|---|---|---|---|
| 2.0, 1.414 (2.83 m/s^2) | 0.0445 / -0.0454 | 0.0433 / -0.0454 | 0.0440 / -0.0454 |
| 3.0, 3.0 (3.00 m/s^2) | 0.0479 / -0.0490 | 0.0418 / -0.0489 | 0.0434 / -0.0489 |
| 3.0, 1.414 (6.36 m/s^2) | 0.354 / -0.774 | 0.214 / -0.640 | 0.090 / -0.098 |

**Inference:** while a_lat stays within what the robot can follow, the error is mostly a **steady lag** on the arc,
about `a_lat·0.05/k_n`, for example `2.83·0.05/3 = 0.047 m`. Clothoids do not remove it; the follower's lookahead
does (section 8.3.4). Clothoids matter when the robot is pushed near its limits (the last row), which the aLat cap
is meant to prevent. This supports G1 in v1. The **vector** jerk of the command at
line/arc joints (110 to 320 m/s^3 in the plant simulation, section 7.13.8) is not limited, and clothoids are the v2 fix for it. The toy model is not a swerve model (no module steering, no limiter).
**Uncertain** how closely it matches the robot.

#### 5.12.2 Options

(a) **Line + arc (G1), v1.** All formulas are closed form. Arc length is exact, projection is closed form, every
waypoint is exactly on the path.

(b) **Clothoid transitions (Euler spiral).** Curvature changes linearly with s:
`kappa(s) = kappa0 + (kappa1 - kappa0)·s/Lc` and `theta_T(s) = theta_T0 + kappa0·s + (kappa1 - kappa0)·s^2/(2·Lc)`.
For a transition from a line (kappa0 = 0) into radius r, the clothoid parameter is `A_cl^2 = r·Lc`. In local coordinates:

```
x(s) = A_cl*sqrt(pi) * C_F( s / (A_cl*sqrt(pi)) ),     y(s) = sigma * A_cl*sqrt(pi) * S_F( s / (A_cl*sqrt(pi)) )
C_F(z) = integral_0^z cos(pi*u^2/2) du,                S_F(z) = integral_0^z sin(pi*u^2/2) du      (Fresnel integrals)
```

There is no closed form; evaluation needs a series or a precomputed table.
- The transition pulls the circle inward by the shift `p_cl ~= Lc^2/(24·r)`: 0.0074 m for Lc = 0.5, r = 1.414, and
  0.0295 m for Lc = 1.0 (hand). The circle's tangent point also moves back by about Lc/2.
- Waypoint on the path: keep the circle through P_i. Each line then becomes tangent to the circle offset by about
  `r + p_cl`. The clothoid's start and end points must be solved numerically (a few Newton steps per transition).
- P_i must stay on the **circular** part: `w_in >= Lc/(2r)` and `w_out >= Lc/(2r)`, which is 10.1 deg for Lc = 0.5,
  r = 1.414 (hand). Otherwise Lc must shrink.
- Projection onto a clothoid: Newton on `(p - x(s))·T(s) = 0`, 2 to 3 iterations from the window guess.

(c) **Splines.** A cubic Hermite spline passes through all waypoints and is C1; a quintic Hermite spline is C2 if the
second derivatives are matched.
- Curvature is not uniform and not directly bounded, so a minimum radius cannot be guaranteed.
- Arc length needs quadrature plus an s-to-parameter table; s is then approximate (about 1e-4 m).
- Projection is Newton on a polynomial with several local minima.
- WPILib has `CubicHermiteSpline`, `QuinticHermiteSpline` and `SplineParameterizer` in `edu.wpi.first.math.spline`
  (**Uncertain**: known WPILib API, not checked here).

| | (a) line + arc, G1 | (b) line + clothoid + arc, G2 | (c) cubic / quintic spline |
|---|---|---|---|
| Waypoint exactly on path | yes, by construction | yes, if P_i stays on the circular part | yes |
| Curvature continuity | jumps at every line/arc joint | continuous | cubic: jumps at knots; quintic: continuous |
| Curvature control | constant on arcs, min radius guaranteed | linear ramps, bounded | not bounded |
| Arc length | exact, closed form | Fresnel for clothoids | quadrature and reparameterization |
| Projection | closed form | Newton on clothoids | Newton, several local minima |
| Implementation cost | lowest | medium | medium to high |
| Corner bulge | `r·(1 - cos(tau_turn/2))`, controlled | slightly more (shift p_cl) | can overshoot unpredictably |
| Speed profile | curvature step at junctions (backward pass) | smooth | smooth, curvature must be sampled |

**Decision** (brief decision 3): G1 in v1.

#### 5.12.3 Criterion for adopting clothoids in v2

Log at 50 Hz: s*, e_n, segment index, kappa(s*) and v. For every arc entry and arc exit k:

```
E_peak,k   = max abs(e_n) over s in [s_entry,k, s_entry,k + 0.5 m]
E_steady,k = median abs(e_n) over the middle third of that arc (steady lag)
dE_k       = E_peak,k - E_steady,k                                  (the transient caused by the curvature step)
```

Adopt clothoid transitions in v2 if, over at least 20 arc entries and exits driven at `v >= 0.5·v_cap(arc)`, either
`median(dE_k) > 0.03 m` (the finish position tolerance) or any `dE_k > 0.10 m` (accepted by the user). If E_steady is
large but dE is small, the fix is in the follower (tau_lag), not in the geometry.

### 5.13 Alliance flip of the geometry

Field length FL and width FW are fields of `PathFollowerConfig` (section 9.6), never hardcoded. **Fact** (RobotPose.java:94-95):
the pose code reads them from `AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)`. **Recommendation:**
`PathFollowerConfig.defaults()` uses the same source. Examples in this document use the placeholder
pair **FL = 16.54 m, FW = 8.07 m**; verify against the official field layout of the season. The mode is one enum,
`FlipMode` NONE / MIRROR / ROTATE, default ROTATE (sections 9.6, 9.9).

| | MIRROR (about x = FL/2) | ROTATE (180 deg about the field center) |
|---|---|---|
| point | `(x, y) -> (FL - x, y)` | `(x, y) -> (FL - x, FW - y)` |
| robot heading / angle | `theta -> pi - theta` | `theta -> theta + pi` |
| direction vector T | `(Tx, Ty) -> (-Tx, Ty)` | `T -> -T` |
| arc center C | mirrored point | rotated point |
| arc phi0 | `pi - phi0` | `phi0 + pi` |
| arc dphi, sigma, kappa | **sign flips** (left turns become right turns) | unchanged |
| omega, theta' | sign flips | unchanged |
| left normal N | **recomputed** as `rot90(T')` from the flipped geometry, never transformed as a vector (a mirrored N would flip e_n and push the robot away from the path) | recomputed the same way (equals -N) |
| s table, lengths, L, waypoint s, events | unchanged | unchanged |
| face-target point | transformed as a point | transformed as a point |

Why: both maps are isometries, so all lengths and s values are kept. A mirror reverses orientation, so CCW becomes
CW and every sign of turn flips. A rotation keeps orientation.

**Recommendation:** build the geometry once in the blue frame, then transform the built segment table. Do not flip
the waypoints and rebuild. The construction is equivariant under both maps, except for tie-breaks that are not
mirror-symmetric: the U-turn default sigma = +1, and the order of shrink passes. Flipping the built table guarantees
an exact mirror image with identical s values. It is O(m), memoized in the `Path` (`path.flipped(config)`,
section 9.4), and runs in the follow command's `initialize()`, once the alliance is known.

Worked example (placeholder FL = 16.54, FW = 8.07) (script):
- MIRROR: P0 becomes (15.54, 2). `C = (13.54, 2.5858)`, `phi0 = 30.94 deg`, `dphi = +118.12 deg`, `kappa = +0.7071`.
  Line 0 direction 120.94 deg.
- ROTATE: P0 becomes (15.54, 6.07). `C = (13.54, 5.4842)`, `phi0 = -30.94 deg`, `dphi = -118.12 deg`, `kappa = -0.7071`.
  Line 0 direction -120.94 deg. A heading of 0 becomes pi.
- Both: segment lengths 1.5307 / 2.9155 / 1.5307, and L = 5.9769.

### 5.14 Agreement and disagreement with path-ido

| Item | path-ido (**Fact**, file:line) | This design | Verdict |
|---|---|---|---|
| Turn sign | `sigh(prev, P, next)` = signum(cross(P_{i-1}-P_i, P_{i+1}-P_i)) (Point.java:56-61) | sigma_i = sign(cross(u_in, u_out)) | **opposite sign**: sigh = -sigma |
| Center | P + r·(cos, sin)(atan2(prev-P) + sigh·gamma), gamma from a law-of-cosines acos (Path.java:46-59) | `C = P + r·unit(-u_in + u_out)` | same point for proper corners; path-ido gives NaN for duplicates (Path.java:50) and a wrong center for collinear points (Path.java:51) |
| Radius | `min(10, d/2 or d/4, d/2 or d/4)` (Path.java:12, :39) | same d/k terms, plus R_MAX = 2, bulge cap, TAU_STOP, checks, shrink | path-ido has no cap on bulge or sweep (326 deg at 170 deg) |
| Endpoint tangent | `atan2(C-P) + sigh·asin(r/abs(P-C))`, length `sqrt(abs(P-C)^2 - r^2)` (Segmant.java:37-48); last leg reversed (Path.java:76-77) | unified h formula, `h = +-sigma·r` | same line; path-ido's asin is unchecked (Segmant.java:38) |
| Circle-to-circle tangent | `sign2 = -sigh·sigh`, `acos((r2 + sign2·r1)/d)` (Segmant.java:50-63) | unified h formula | same line |
| Arc length / sweep | `r·acos(law of cosines)`, range [0, pi] (CircleSegmant.java:13, :26-31) | `r·(w_in + w_out)` via mod2pi | **disagree**: path-ido is wrong for sweeps over 180 deg (U-turn example: 152.3 vs 207.7 deg) |
| Distance to arc | `abs(p - C) - r`, positive outside (CircleSegmant.java:21-23) | `e_n = sigma·(r - abs(vC))`, positive = left of the path | ours is signed by side |
| Distance to line | unsigned distance to the infinite line (LineSegmant.java:14-21) | signed e_n, projection clamped to the segment | ours is signed and segment-bounded |
| Remaining distance on a line | Euclidean distance to the end (LineSegmant.java:24-26) | s-based, L - s* | ours is projected |
| Collinear points | sign 0: center on the leg, zero-length first line (Path.java:51) | PASS marker, set aside before building (5.8.3) | **disagree** |
| Duplicates | no check; divides by zero (Path.java:50) | merged at 1 mm | **disagree** |
| n = 2 | single line (Path.java:65-67) | single line | same |
| Segment order | line, arc, ..., line, `2n-3` entries (Path.java:70) | same, plus zero-length STOP entries in corner slots | same index map |

### 5.15 Pseudocode

```
build(points) -> SegmentTable                                        // in the Path constructor, blue frame
    P = mergeDuplicates(points, EPS_DUP)                             // warning W3
    if |P| == 1: throw V6
    validate stop + withRadius(r > 0) -> throw V30
    passPoints = middle points with tau_turn < EPS_ANG that are not user stops; remove them from P    (5.8.3)
    if |P| == 2: table = [LINE(P0, P1)]
    else:
        for i in 1..n-2:
            compute tau_turn,i, sigma_i, b_i                         (5.3)
            if not USER_STOP and tau_turn,i > pi - EPS_ANG: sigma_i = neighborSign(i) or +1;  b_i = -u_in
            r_i = initialRadius(i)                                   (5.7.2, 5.7.3; USER_STOP: 0)
        repeat up to MAX_ITER:
            lines = [tangent(elem j, elem j+1) for j in 0..n-2]       (5.4; r = 0 elements are points)
            arcs  = [sweep(i) for corners with r_i > 0]               (5.5)
            bad = failing(F1, F2, F3)
            if bad empty: break
            shrink(bad)                                               (automatic corners first; never a user stop)
        table = (lines, arcs, STOP entries) with s_start, l, s_wp, userStop, dwell
    for P_i in passPoints: s(P_i) = closest s on the table; warning W5 if the miss > PASS_TOL     (5.8.3)
    return table

evaluate(table, s) -> (x, T, N, kappa, theta_T)                      (5.9.2, 5.9.3; evaluateLeft for left limits)
project(table, p, sMin, sMax, s_prev) -> (s*, e_n)                   (5.10)
flipped(table, mode, FL, FW) -> table                                (5.13, memoized in Path)
```

Tests: section 13.1.

## 6. Heading profile

The robot's heading is independent of its direction of travel (swerve). This chapter defines the
heading the robot should have at every s, theta(s), and what it means for the speed profile and the
follower. The heading profile is built in `initialize()`, after the alliance flip, because it starts from
the measured heading.

### 6.1 The rule

(**Decision**, brief decision 4 and integration resolution R1.)
- **Anchors** are the waypoints that have `withAngle`. Each anchor fixes theta at its s.
- **Before the first anchor:** theta is linear in s from the measured start heading theta0 at s = 0 (section 6.8)
  to the first anchor.
- **Between anchors:** theta is linear in s, turning the short way (at most 180 deg per interval).
- **After the last anchor:** theta holds the last anchor's angle.
- **No anchor at all:** theta holds theta0 for the whole path.
- theta(s) is **continuous** everywhere. There are no steps. At a stop point the ROTATE state only corrects a
  residual heading error of more than 3 deg (section 6.7).
- **Face-target mode** (section 6.9) replaces the anchors. If both are given, the anchors are ignored, with
  warning W4.

Why linear in s and not a smooth curve: the speed caps that come from the heading (rotation cap, alpha limits,
wheel-speed cap) are constant on a span only if theta' is constant there (section 7.3.2). With linear pieces,
theta' changes only at anchors, which are span boundaries. A smooth interpolation would make theta' vary inside
spans and would need sub-spans, as face-target mode does (**Inference**).

### 6.2 Anchors and their s

- The anchor of waypoint P_a sits at `s_a = s(P_a)` (section 5.9.4): inside its arc for a corner, at the STOP entry
  or the PASS marker, at 0 for P_0 and at L for P_{n-1}.
- P_0 with an angle is an anchor at s = 0. Then theta(0) is that angle, not theta0. If the robot's measured heading
  differs, that is a tracking error, which the heading feedback removes while the robot drives (section 8.5.2;
  simulated: a 30 deg error is within 2 deg after 0.50 s at k_theta = 4). theta(s) itself has no step.
- A merged duplicate keeps one s; if both points had an angle, the later one wins (warning W3).
- Two anchors always have different s, because merged points are at least EPS_DUP apart along the path.

### 6.3 Formulas and wrap

```
shortTurn(d) = angleModulus(d);   if abs(d) > pi - 1e-9: d = +pi           (exact half turn: CCW)
pts = [(0, theta_start)]
      theta_start = angle of P_0 if it has withAngle, else theta0 (measured, radians)
for each anchor (s_a, theta_a) with s_a > 0, in increasing s:
    th_prev = last unwrapped value in pts
    pts.append( (s_a, th_prev + shortTurn(theta_a - th_prev)) )             (stored unwrapped)
theta(s)  = linear interpolation in pts; held after the last point
theta'(s) = (th_b - th_a)/(s_b - s_a) on [s_a, s_b];  0 after the last point
```

- **Unwrapped storage.** Each anchor's stored value is the previous value plus the short turn, so interpolation
  never crosses a wrap. Angles are wrapped only when compared with the measured heading:
  `e_theta = wrap(theta(s*) - theta_measured)` (section 8.5.4).
- **Tie at exactly 180 deg.** `MathUtil.angleModulus` at exactly +-pi is not checked here (**Uncertain**), so the tie is
  implemented explicitly: a change within 1e-9 rad of 180 deg turns CCW (+pi).
- Example: theta_a = 170 deg at s = 1, theta_b = -170 deg at s = 3. `shortTurn(-340 deg) = +20 deg`, so the robot
  turns 20 deg CCW through 180 deg, not 340 deg CW. theta' = 0.3491/2 = 0.1745 rad/m, and the stored theta_b is
  190 deg (hand).

### 6.4 theta' per span

The speed profile splits the path into spans (section 7.3.1). Every anchor is a waypoint, and every waypoint s is a
span boundary, so each span lies inside one linear piece of theta and has a constant `theta'[k]`. s = 0 is also the
first boundary of the profile. The table field `dtheta[k]` of the profile (section 7.1) is this
value.

### 6.5 Omega feed-forward

```
omega_ff = theta'(s*) * v_t          (v_t = this loop's commanded tangential speed, section 8.3.1)
```

- Derivation: `omega = dtheta/dt = (dtheta/ds)·(ds/dt) = theta'·v`.
- The commanded v_t is used, not the measured speed: the measured speed carries module noise and the
  cosine-scaling dip at direction changes, and translation and rotation then go through the same drivetrain lag
  in step. **Uncertain** whether a filtered `ds*/dt` does better (test FOL-H4).
- No lookahead on the heading. With a constant rotation rate, the delayed feed-forward is the same constant, so a
  ramp has no steady error.
- At an anchor, theta' changes by `dtheta'` and omega_ff jumps by `abs(dtheta')·v`. The follower slews omega at
  alphaMax (`abs(omega_k - omega_{k-1}) <= alphaMax·dt`, section 8.5.2). During the slew time `abs(dtheta')·v/alphaMax`
  the heading falls behind by `e = (dtheta'·v)^2/(2·alphaMax)`. The speed profile keeps `e <= e_allow` with the
  heading-kink cap (section 6.6).
- The feedback `omega_fb` (section 8.5.2) is added; the sum is clamped to `+-omegaMax` and rate limited.

### 6.6 Interaction with spans and the omega and alpha caps

| Effect | Formula | Where it acts |
|---|---|---|
| Rotation cap | `vCap[k] <= omegaMax/abs(theta'[k])` (from `omega = theta'·v <= omegaMax`) | span cap, section 7.4.4 |
| Alpha inside a span | theta'' = 0, so `alpha = theta'·a`: `aA[k], aD[k] <= alphaMax/abs(theta'[k])` | span limits, section 7.4 |
| Alpha at an anchor (heading kink) | `J[j] <= sqrt(2·alphaMax·e_allow)/abs(dtheta'_j)`, skipped where J = 0 (a stop) | junction cap, section 7.4.6 |
| Wheel speed | `vCap[k] <= 0.9·vWheelMax/(1 + abs(theta'[k])·R_mod)` | span cap, section 7.11 |
| Speed law (Amax) | `aAccEff` includes `alphaMax/abs(theta'(s))` | sections 7.6.5, 7.13.4 |

Derivation of the kink cap: require the lag of section 6.5 to stay below e_allow:
`(dtheta'·v)^2/(2·alphaMax) <= e_allow`, so `v <= sqrt(2·alphaMax·e_allow)/abs(dtheta')`. `e_allow` =
HEADING_LAG_ALLOWED = 5 deg (section 10); the 2 deg finish tolerance applies only at the end, where v = 0.

The profile margin (0.9 of the drivetrain limits, section 7.2.1) and the omegaMax clamp leave room for the
feedback term.

### 6.7 Stops

- theta is continuous through a stop point s_c, so the robot arrives with theta(s_c) and leaves with the same
  reference. A heading change around a stop happens along the legs before and after it, as the anchors say.
- The ROTATE state (section 8.6) is entered only if `abs(wrap(theta(s_c) - theta)) > stop_ang_tol` (3 deg, resolution R1) on
  arrival. It holds P_c and corrects the residual. The profile has no turn-in-place time; a residual of a few
  degrees takes well under 0.1 s (from 4 deg to 3 deg: `ln(4/3)/k_theta = 0.072 s` in the linear zone (hand)),
  inside the timeout margin.
- The heading-kink cap is skipped at a stop, because v = 0 there and omega_ff is already 0.
- A deliberate turn in place at a stop (a heading step) is not part of v1 (section 11).

### 6.8 Start heading

The heading profile always starts at s = 0 with the robot's measured heading theta0, or with P_0's angle if P_0 has
`withAngle` (**Decision**, user, simplification pass: no mid-path start; the robot starts within 0.5 m of P0,
section 8.2.5). There is no rule for anchors behind the robot. Example (hand): shared example, measured heading 10 deg:
theta goes from 10 deg at s = 0 to 90 deg at 2.9885, `theta' = 1.3963/2.9885 = 0.4672 rad/m`.

### 6.9 Face-target mode

> Removed in v1 (simplification); kept as v2 reference.

The follower's version is final (resolution R1): the reference is computed from the **robot position** and freezes
near the target. The robot's front faces the target (**Decision**, user, simplification pass).

```
pT        = target - p
theta_ref = atan2(pT.y, pT.x)
omega_ff  = (pT.y*V.x - pT.x*V.y) / abs(pT)^2        (V = this loop's field velocity command)
if abs(pT) < r_face_min (0.5 m): theta_ref = last value, omega_ff = 0       (freeze)
finish heading theta_end = atan2(target - P_end)     (the frozen value if P_end is inside r_face_min)
```

Derivation (section 8.5.3): `theta = atan2(pT)` and `d(pT)/dt = -dp/dt`, so
`dtheta/dt = (pT.y·dp_x - pT.x·dp_y)/abs(pT)^2`; the command V stands for dp/dt.

Speed caps in face-target mode. The profile predicts the robot on the path, so it uses
`theta_FT(s) = atan2(target - x(s))`, with the same freeze:

```
theta'_FT(s) = (pT.y*T.x - pT.x*T.y) / abs(pT)^2      with pT = target - x(s)      (0 inside r_face_min)
```

theta'_FT varies continuously, so the profile uses sub-spans of FT_SPAN = 0.10 m (section 7.3.1) with conservative
caps:
- theta'_max and theta''_max = the largest abs values on the sub-span (at its ends and at the closest approach to the
  target, if inside it);
- `vCap <= omegaMax/theta'_max`, `vCap <= sqrt(0.5·alphaMax/theta''_max)`, `aA, aD <= 0.5·alphaMax/theta'_max`
  (alpha = theta''·v^2 + theta'·a, each term given half of alphaMax);
- no kink caps between sub-spans (the theta'' term covers them), except at the freeze boundary, where theta' jumps
  to 0: the kink cap of section 6.6 applies there.

Example (hand, script): a straight path passing 1.0 m from the target. theta'_max = 1/1.0 = 1.0 rad/m (rotation cap
2·pi/1.0 = 6.28 m/s, not binding); theta''_max = (3·sqrt(3)/8)/1.0^2 = 0.6495 rad/m^2, so
`vCap <= sqrt(0.5·4·pi/0.6495) = 3.110 m/s` near the target, and `aA, aD <= 6.28 m/s^2`. A path passing exactly at
r_face_min = 0.5 m reaches theta' = 2 rad/m at the freeze boundary, where the kink cap is
`sqrt(2·4·pi·0.0873)/2 = 0.740 m/s`. There is no warning for this; the heading simply freezes there.

### 6.10 Alliance flip of headings

The anchors' angles are flipped with the path (section 5.13): MIRROR `theta -> pi - theta`, ROTATE
`theta -> theta + pi`. theta0 is measured in the field frame and is not flipped. The profile is built after the
flip, so theta' of the flipped path follows. Because the 180 deg tie always turns CCW, an anchor change of exactly
180 deg is **not** mirrored under MIRROR (it turns CCW on both alliances). **Recommendation:** avoid exact 180 deg
changes between anchors, or add an anchor in between.

### 6.11 Worked example

Shared example: theta0 = 0 (measured, P0 has no angle), P1 `withAngle(90 deg)` at s(P1) = 2.9885, P2 no angle,
constraints EX (script):
- Points: (0, 0) and (2.9885, pi/2); held after. `theta' = (pi/2)/2.9885 = 0.5256 rad/m` on [0, 2.9885], 0 after.
- Spans (section 7.3.3): spans 0 and 1 have theta' = 0.5256; spans 2 and 3 have 0.

| s [m] | where | theta [deg] | theta' [rad/m] | vProf [m/s] | omega_ff = theta'·v [rad/s] |
|---|---|---|---|---|---|
| 0 | start | 0.00 | 0.5256 | 0 | 0 |
| 1.0 | line 0, speed-up | 30.12 | 0.5256 | 2.4495 | 1.2875 |
| 1.2368 | line 0, peak | 37.25 | 0.5256 | 2.7241 | 1.4318 (max) |
| 1.5307 | arc entry | 46.10 | 0.5256 | 2.3784 | 1.2501 |
| 2.9885 | P1 | 90.00 | 0.5256 before, 0 after | 2.3784 | 1.2501 before, 0 after |
| 4.0 | arc | 90.00 | 0 | 2.3784 | 0 |
| 5.9769 | end | 90.00 | 0 | 0 | 0 |

Caps from the heading:
- Rotation cap on spans 0 and 1: `2·pi/0.5256 = 11.95 m/s`, not binding.
- Alpha inside span 0: `alpha = 0.5256·3 = 1.577 rad/s^2` while speeding up or braking; the limit
  `alphaMax/theta' = 4·pi/0.5256 = 23.9 m/s^2` does not bind. On the arc span 1 (constant speed) alpha = 0.
- Heading-kink cap at P1 (dtheta' = 0.5256): `sqrt(2·4·pi·0.0873)/0.5256 = 2.818 m/s`, above the arc cap 2.3784, so not
  binding. At 2.3784 m/s the omega step is 1.2501 rad/s, slewed in `1.2501/(4·pi) = 0.0995 s` (5 loops), lag
  `1.2501^2/(2·4·pi) = 0.0622 rad = 3.56 deg <= 5 deg`.
- Wheel cap with R_mod = 0.3889 m: `4.05/(1 + 0.5256·0.3889) = 3.363 m/s`, above the 2.7241 peak.
- Result: the trapezoid profile time (no jerk limit) is 3.2723 s, the same as with no heading (script); no heading cap binds, so the
  v1 profileTime (3.4426 s) is unchanged as well.

Variants (script):
- **P1 at 180 deg.** The change is exactly 180 deg, so it turns CCW (+pi). theta' = pi/2.9885 = 1.0512 rad/m; the
  kink cap at P1 is `1.481/1.0512 = 1.4088 m/s` and binds, so the whole arc is driven at 1.4088 m/s (section 7.8).
  Trapezoid profile time **4.2812 s** with the span method (v1 profileTime 4.2812 + 0.1703 = 4.4515 s); the grid oracle, which
  may change speed on the arc, gives 3.4372 s (Appendix B).
- **P1.stop().** theta' = (pi/2)/2.8284 = 0.5554 rad/m on [0, 2.8284], 0 after. The robot reaches 90 deg exactly at
  the stop; ROTATE is skipped unless the residual is more than 3 deg. Profile unchanged (trapezoid 3.8839 s; v1 profileTime 4.2245 s).
- **P0.withAngle(0)** gives the same profile as the measured theta0 = 0.

### 6.12 Edge cases

- Anchor at the start and at the end only: one linear piece over the whole path.
- Anchors close together with a big change: theta' is large and the rotation cap slows the robot. Example: 180 deg
  over 0.1 m gives theta' = 31.4 rad/m and `vCap = 2·pi/31.4 = 0.2 m/s`. That is correct physics.
- theta' at s = 0 with v0 > 0: omega_ff starts at `theta'·v0` while the robot rotates at its measured omega; the omega
  rate limit (initialized from the measured omega, section 8.14) smooths it.
- Face target and anchors both set: anchors ignored, W4.
- Short path (L < L_short): the follower goes straight to CAPTURE_END with theta_end = theta(L).

### 6.13 Pseudocode

```
buildHeading(path(flipped), theta0, faceTarget?) -> HeadingProfile
    if faceTarget: return FaceTargetHeading(target, R_FACE_MIN)                   // 6.9; sub-span caps in 7.3.1
    anchors = [(s(P_a), angle(P_a)) for waypoints with withAngle], sorted by s
    pts = points of section 6.3 (unwrapped, shortTurn per interval)
    return PiecewiseLinear(pts)          // theta(s), theta'(s) on [0, L], held after the last point

runtime (each loop):
    theta_ref = theta(s*);  omega_ff = theta'(s*) * v_t                          // 8.5.1
```

Tests: section 13.2.

## 7. Speed profile

This chapter defines the speed at every point of the path, as a function of s. The method was chosen by
comparison (section 7.15; brief decision 16) and is final (resolution R10): **a closed-form profile per span**,
with motor-curve speed-up and junction passes. The grid method is kept only as a test oracle (section 7.16).

### 7.1 Overview

A **span** is a piece of the path where every speed limit is constant. Each span gets a closed-form profile:
speed up, cruise, slow down. Speed-up follows the drivetrain's motor curve up to the traction limit; slow-down is
constant. The speeds at the junctions between spans come from one backward pass and one forward pass **over the
spans**. A stop is a junction speed of 0 and splits the profile into sub-profiles.

**Runtime command** (v1): the plain online trapezoid `min(vBack(s_v), v_prev + aAccEff·dt)` of sections 7.6.5 and 8.3.7.
Braking is limited by aD through vBack; speed-up is a time ramp. (The jerk-aware law of section 7.13 was removed in v1.)

The profile is built once, in the follow command's `initialize()`, after the flip and the heading profile, because
it needs the measured start speed and heading. It is a short table, one row per span:

| Field (span k) | Meaning | Unit |
|---|---|---|
| `sStart[k]`, `l[k]` | start and length of the span | m |
| `kappa[k]` | curvature (constant on the span) | 1/m |
| `dtheta[k]` | heading slope theta' (constant on the span, section 6.4) | rad/m |
| `kind[k]` | LINE, ARC or ARC_VAR; an ARC span has aA = aD = 0 (constant speed); an ARC_VAR span (the arc of a `withVelocity` point) has the grip-shared limits of section 7.8 | |
| `vCap[k]` | span cap: highest allowed speed on the span | m/s |
| `aA[k]`, `aD[k]` | speed-up limit at rest, slow-down limit | m/s^2 |
| `vB[j]` | junction speed after the backward pass (j = 0..K) | m/s |
| `vF[j]` | junction speed after the backward and forward passes | m/s |
| `t0[k]` | profile time at the start of span k | s |

Two speed functions are read from it:
- **vBack(s)**: built from the backward junction speeds vB only. It is the **braking envelope**; vB and vCap are the speed law's
  limits. The v1 law is `min(vBack, ramp)` (section 7.6.5). vBack(start) > 0, so the robot cannot stall.
- **vProf(s)**: built from the forward-pass junction speeds vF and the measured start speed: the jerk-free lower bound of the
  time, a(s) and logging. The v1 profileTime (timeout) is its closed-form time plus the hand-over tail (section 7.12).

The old code had no profile (path-ido: constant 1.5 m/s) or used the straight-line distance (2025); see section 4.

### 7.2 PathConstraints

```
record PathConstraints(double vMax, double aAcc, double aDec, double aLat, double omegaMax, double alphaMax)
```

| Field | Meaning | Unit | Must be |
|---|---|---|---|
| `vMax` | top speed of the robot center along the path | m/s | finite, > 0 |
| `aAcc` | largest tangential speed-up (along T), traction limit at rest | m/s^2 | finite, > 0 |
| `aDec` | largest tangential slow-down (magnitude) | m/s^2 | finite, > 0 |
| `aLat` | largest lateral acceleration `v^2·abs(kappa)` | m/s^2 | finite, > 0 |
| `omegaMax` | largest robot angular velocity abs(omega) | rad/s | finite, > 0 |
| `alphaMax` | largest robot angular acceleration abs(alpha) | rad/s^2 | finite, > 0 |

A field that is not finite or not > 0 throws `IllegalArgumentException` in the constructor (V16). Silently clamping
would hide a typo.

#### 7.2.1 Defaults from KinematicsConfig

**Fact:** there is no class `KinematicsConstants`; the robot limits are instance fields of `KinematicsConfig`
(KinematicsConfig.java:20, fields :25-59). Defaults are therefore built from a `KinematicsConfig` instance by
`PathConstraints.fromConfig(config, pathsConfig)`, not from static constants (`PathsConfig`, section 9.6).

`PROFILE_MARGIN = 0.9` (**Recommendation**): the profile stays 10% inside the limiter's limits, so the limiter does not
change commands that follow the profile (section 7.14).

| Field | Default formula | Value with `new KinematicsConfig()` | Source fields (**Fact**) |
|---|---|---|---|
| `vMax` | 0.9·min(maxAllowedModuleVelocity, freeSpeed) | 0.9·min(4.5, 5.3) = **4.05 m/s** | :32, :51 |
| `aAcc` | 0.9·min(maxForwardAccel, maxSkidAccel, maxTiltAccelFront, maxTiltAccelSide) | 0.9·min(10, 8, 12, 12) = **7.2 m/s^2** | :46, :53, :55, :57 |
| `aDec` | 0.9·min(maxSkidAccel, maxTiltAccelFront, maxTiltAccelSide) | **7.2 m/s^2** | :53, :55, :57; braking is free from the forward limit (SwerveAccelerationLimiter.java:94) |
| `aLat` | same as aDec | **7.2 m/s^2** | same |
| `omegaMax` | `PathsConfig.maxAngularVelocity` | 2·pi rad/s | placeholder in `PathsConfig.defaults()` |
| `alphaMax` | `PathsConfig.maxAngularAccel` | 4·pi rad/s^2 | placeholder in `PathsConfig.defaults()` |

Notes:
- Both tilt limits are used for every direction, because the path tangent can point along the robot's front or
  side (**Inference**).
- `aAcc` is the traction limit at rest. The speed-dependent motor limit is handled in the speed-up phase
  (section 7.9). It needs `maxForwardAccel` and `freeSpeed`, which are not record fields.
- `maxSteerRate` (:41) is the module steer rate, a different quantity from omegaMax. The limiter passes omega
  through unchanged (**Fact** SwerveAccelerationLimiter.java:80), so nothing limits alpha today; the follower does
  (section 8.5.2).
- A per-path override is a `with...` copy: `PathConstraints.fromConfig(cfg, pathsCfg).withVMax(2.0)`.
- The robot settings that `KinematicsConfig` does not have (angular limits, module distance) live in the Path module's own
  `PathsConfig` (section 9.6). **Decision** (user): the Kinematics module is not changed.

#### 7.2.2 Inputs that are not record fields

> Partly removed in v1: `moduleRadius`, the motor curve and the C2 limit warnings are gone; kept as v2 reference.


The record keeps its fields (brief decisions 6 and 18). Two optional inputs go to the command (section 9.10):
- the `KinematicsConfig`: `maxForwardAccel` and `freeSpeed` (motor curve), `maxAllowedModuleVelocity`
  (wheel cap), and the limiter limits for warning C2;
- the `PathsConfig`, whose `moduleRadius` is R_mod: the largest distance from the robot center to a module.
  `DemaciaKinematics` keeps the module positions private with no getter (DemaciaKinematics.java:58, :66), so the user
  computes R_mod from the same module locations.

If the config is missing, the motor-curve cap is skipped. If R_mod is 0 or missing, the wheel-speed cap is skipped.
`DemaciaKinematics.desaturate` and the limiter still protect the hardware.

#### 7.2.3 Edge cases

- A per-path override above the config-derived value (e.g. `aLat = 10` with `maxSkidAccel = 8`): allowed, warning C2.
  The limiter clips it, and the robot falls behind the profile time.
- `vMax > maxAllowedModuleVelocity`: the robot can never reach it. Warning C2; the wheel cap clamps the effective cap
  to `0.9·maxAllowedModuleVelocity`.

### 7.3 Spans

**Replaced in v1 by the per-loop limit law (see the v1 scope); kept as reference.**

#### 7.3.1 Span boundaries

A span is an interval of s with no boundary inside it. Collect the boundaries:
1. {0, L}; the profile covers [0, L] (there is no mid-path start, section 8.2.5);
2. the start of every segment, s_start,k (section 5.9.1). STOP entries have length 0, so they add only their s;
3. the s of every waypoint P_i (section 5.9.4), PASS markers included. A middle waypoint with an arc lies inside its
   arc (at the midpoint only for a symmetric corner), so this splits the arc;
4. every stop point: automatic STOP corners (section 5.7.3) and user stops (section 5.8.7);
5. (face-target mode only) sub-span boundaries every FT_SPAN = 0.10 m (section 6.9);
6. (v2) the ends of speed zones.

Sort them and merge two boundaries closer than S_EPS = 1e-6 m (keep the first). Each interval between consecutive
boundaries is one span. Its curvature is that of the segment it lies in, so kappa is constant on every span (lines
0, arcs `+-1/r`). Heading anchors are waypoints, so item 3 makes theta' constant per span (section 6.4).

#### 7.3.2 Why spans are exact for line+arc paths

**Inference.** On a span, every pointwise cap is constant:

| Cap | Depends on | Constant on a span because |
|---|---|---|
| vMax | nothing | it is a constant |
| curvature | kappa | kappa is constant per segment, and spans never cross a segment boundary |
| rotation, wheel | theta' | theta' is constant per span (section 6.4) |
| withVelocity, heading kink, stop | a point | they sit on junctions (waypoints) |

The speed-up and slow-down limits are constant too: aDec is constant, and aAcc is constant below the motor-curve knee.
So the best profile under these limits is, on each span, exactly min(cap, speed-up curve from the entry, slow-down
curve to the exit): a trapezoid in the v^2 against s picture. The grid method computes the same thing sample by
sample; on the shared example both give 3.2723 s (script).

Exceptions, where speed-dependent limits make a pure trapezoid inexact:
- arcs: the traction ellipse makes the speed-change limit on an arc depend on v; v1 avoids this by driving every arc
  at constant speed, except the arc of a `withVelocity` point, which gets a fixed grip share with constant limits (ARC_VAR,
  section 7.8);
- speed-up above the motor-curve knee, handled in closed form (section 7.9);
- face-target heading: theta' changes continuously, handled by sub-spans with conservative caps (section 6.9);
- v2 clothoids: kappa changes linearly, handled by sub-spans.

#### 7.3.3 Example span table

Shared example, geometry of section 5.11, heading of section 6.11 (script):

| k | Kind | start | l | kappa | theta' |
|---|---|---|---|---|---|
| 0 | line 0 | 0.0000 | 1.5307 | 0 | 0.5256 |
| 1 | arc, entry to P1 | 1.5307 | 1.4578 | -0.7071 | 0.5256 |
| 2 | arc, P1 to exit | 2.9885 | 1.4577 | -0.7071 | 0 |
| 3 | line 2 | 4.4462 | 1.5307 | 0 | 0 |

L = 5.9769 m. The arc length 2.9155 is split at s(P1) = 2.9885. Edge cases: a span shorter than S_EPS is merged away;
a path with n = 2 and no anchor in the middle is a single span.

### 7.4 Caps per span and at junctions

**Replaced in v1 by the per-loop limit law (see the v1 scope); kept as reference.**

Spans k = 0..K-1, junctions j = 0..K. Junction j is the start of span j; junction K is L.

```
span cap      vCap[k] = min( vMax,
                             sqrt(aLat/abs(kappa[k]))                     (on an arc, section 7.4.3;
                                                                           sqrt(rho_lat*aLat/abs(kappa[k])) on ARC_VAR, 7.8),
                             omegaMax/abs(theta'[k])                      (if theta'[k] != 0),
                             0.9*vWheelMax/(1 + abs(theta'[k])*R_mod)     (if R_mod known) )
junction cap  J[j]    = min( vCap[j-1], vCap[j],                          (the spans on both sides)
                             withVelocity of a waypoint at junction j,
                             heading-kink cap at junction j               (if theta' changes there),
                             0 at a stop point, 0 at L (v1) )
              J[first junction of a sub-profile after a stop] uses the normal caps, NOT 0     (section 7.10.2)
```

On a constant-speed arc span aA = aD = 0, so the junction passes give all junctions of that arc the same speed. The arc of a
`withVelocity` point is ARC_VAR instead: it brakes into the point and speeds up after it inside a grip share (section 7.8).

#### 7.4.1 vMax

`vCap[k] <= vMax`. Example: vMax = 4 gives vCap = 4 on both lines of the shared example.

#### 7.4.2 withVelocity (point cap at a junction)

**Decision** (user, simplification 2026-11-03): in v1 `withVelocity(v)` on a **corner** caps the **whole arc** through the point at `v`. Arcs are driven at one constant
speed, so the robot brakes on the line before the arc and speeds up on the line after it. On a straight part (a PASS marker, section 5.8.3) the cap acts at
the waypoint's single s value: the backward pass brakes before the point and the forward pass speeds up after it, so the robot never cruises at v. Ranges are
v2 speed zones (decision 5). The variable-speed arc (ARC_VAR, brake to the point inside the arc) of the earlier design is in section 7.8 as v2 reference.

Rules:
- `withVelocity(0)` means `stop()` (decision 15). `withVelocity(v)` with v < 0 throws (V11).
- `P_0.withVelocity(v)`: ignored, warning W4. The start speed is the measured speed.
- `P_{n-1}.withVelocity(v)`: ignored in v1, warning W4 (the end speed is 0). In v2 it becomes the end speed (section 7.4.7).
- A value above the arc's curvature cap (or vMax) has no effect; there is no warning in v1 (C2 is removed).

Example: `P1.withVelocity(1.0)` on the shared example holds both arc spans at 1.0 m/s (the curvature cap is 2.3784 m/s with aLat = 4). v1 profileTime:
**5.43 s** (3.44 s without the cap, 4.22 s with `P1.stop()`). The earlier ARC_VAR rule gave 3.7751 s (trapezoid, script) and 4.06 s (old jerk law, v2 reference); those numbers in the text
below and in 7.8 are the v2 reference.

#### 7.4.3 Curvature cap

```
v <= sqrt(aLat / abs(kappa))        (no cap if kappa = 0)
```

Derivation: lateral acceleration on a curve is `a_n = v^2·abs(kappa)`; `a_n <= aLat` gives the formula.
Examples: r = 1.4142 m, aLat = 4 gives `sqrt(4·1.4142) = 2.3784 m/s`; aLat = 7.2 (DEF) gives 3.191 m/s; r = 0.3 m,
aLat = 4 gives 1.0954 m/s.

#### 7.4.4 Rotation cap

`omega = theta'·v`, so `abs(omega) <= omegaMax` gives `v <= omegaMax/abs(theta'[k])`. Example: theta' = 0.5256 rad/m gives
`2·pi/0.5256 = 11.95 m/s`, not binding. A heading change of 180 deg over 0.1 m (theta' = 31.4 rad/m) gives 0.2 m/s: the
robot must slow down to turn in time.

#### 7.4.5 Wheel-speed cap

> Removed in v1 (simplification); kept as v2 reference.

`vCap <= 0.9·vWheelMax/(1 + abs(theta')·R_mod)`; derivation in section 7.11.

#### 7.4.6 Heading-kink cap (alpha)

At a junction where theta' changes by dtheta': `J <= sqrt(2·alphaMax·e_allow)/abs(dtheta')` (derivation in section 6.6).
Skipped at stop points (v = 0). Example: shared example, P1: 2.818 m/s (not binding); P1 at 180 deg: 1.4088 m/s (binding:
the whole arc is driven at it, section 7.8).

#### 7.4.7 End speed

- v1: J[K] = 0. The robot stops at L.
- v2 (non-zero end speed, for chaining): `J[K] = v_end` from `P_{n-1}.withVelocity(v_end)`. Chaining is safe only if
  the next path's start can take that speed: build the next profile at the same time and require
  `v_end <= vBack_next(0)`; otherwise lower v_end and warn. A chained end replaces the finish test with `s* >= L`.

### 7.5 Curvature jumps at line/arc joints

In v1 the path is G1: the tangent is continuous but kappa jumps from 0 to `+-1/r` at every line/arc joint (decision 3).
- The joint is a junction with cap `J = min(vCap_line, vCap_arc)`, so the speed at the joint is already `<= sqrt(aLat·r)`.
- The backward pass makes sure this speed is reached **on the line, before the joint**, where kappa = 0 and all the grip
  is available for braking.
- Lateral acceleration just before the joint: 0. Just after: `v^2/r <= aLat`. So **the jump in lateral acceleration is at
  most aLat**. It is a step, so jerk is unbounded.
- The limiter sees a velocity direction change of `v^2·kappa·dt` per loop: `4.0·0.02 = 0.08 m/s`, inside its skid disk of
  `maxSkidAccel·dt = 8·0.02 = 0.16 m/s` (**Fact** SwerveAccelerationLimiter.java:106).
- Shared example (script): the arc starts at s = 1.5307 with speed 2.3784 m/s; a_n jumps from 0 to
  `2.3784^2/1.4142 = 4.000 m/s^2 = aLat` (same numbers as section 5.12.1).

Why the cap must cover the whole arc: a cap only at the joint would let the profile speed up inside the arc, with
`v^2/r > aLat` for part of it. With spans, vCap is constant over every arc span, so this cannot happen.

Edge case: two arcs that touch (zero-length common tangent) with opposite signs: kappa jumps from `+1/r1` to `-1/r2`; the
junction cap is the smaller cap, and the step in a_n can be up to 2·aLat in direction. Flag it in testing; clothoids (v2)
remove it.

### 7.6 Junction passes (backward and forward)

**Replaced in v1 by the per-loop limit law (see the v1 scope); kept as reference.**

#### 7.6.1 Derivation

With s as the variable, tangential acceleration is `a = dv/dt = (dv/ds)·(ds/dt) = v·dv/ds = 0.5·d(v^2)/ds`. For constant a
over a length l:

```
v_end^2 = v_start^2 + 2*a*l          (exact for constant a)
```

#### 7.6.2 The passes

```
J[j] = junction caps (section 7.4);  vB = copy(J)
backward:  for j = K-1 down to 0:  vB[j] = min(vB[j], sqrt(vB[j+1]^2 + 2*aD[j]*l[j]))
forward:   vF = copy(vB);  vF[0] = min(vF[0], v0)
           for j = 1 .. K:         vF[j] = min(vF[j], F_{j-1}(vF[j-1], l[j-1]))
```

`F_k(v_in, d)` is the speed after distance d from speed v_in on span k: `sqrt(v_in^2 + 2·aA[k]·d)` with constant aA, or
the motor-curve closed form (section 7.9).

One backward pass followed by one forward pass is enough (**Inference**): the forward pass only lowers values, and a
lower vF[j] makes `vF[j] <= sqrt(vF[j+1]^2 + 2·aD·l)` easier to meet, so it cannot break a braking constraint.
Cost: O(K), with K typically 4 to 20 (a 20 m grid needs about 1000 samples).

Worked example (shared example, EX; aA = aD = 0 on both arc spans, section 7.8) (hand, script):

| j | J (cap) | vB (backward) | vF (forward, v0 = 0) |
|---|---|---|---|
| 4 (L) | 0 | 0 | 0 |
| 3 | min(2.3784, 4) = 2.3784 | min(2.3784, sqrt(0 + 2·3·1.5307) = 3.0306) = 2.3784 | 2.3784 |
| 2 (P1) | 2.3784 | min(2.3784, sqrt(2.3784^2 + 0)) = 2.3784 | 2.3784 |
| 1 | 2.3784 | 2.3784 | min(2.3784, sqrt(0 + 9.1842)) = 2.3784 |
| 0 | 4 | min(4, sqrt(5.6569 + 9.1842)) = **3.8524** | 0 |

vBack(0) = 3.8524 m/s > 0.

#### 7.6.3 Why naive independent trapezoids fail

"One trapezoid per segment" without passes across spans gives each span its own entry and exit: entry = its cap,
exit = the next cap. That fails as soon as a span is too short to brake in. Example: a 0.3 m span entered at 3 m/s
must end at 0 with aDec = 3.
- Braking needs `3^2/(2·3) = 1.5 m`. Inside 0.3 m it would need `3^2/(2·0.3) = 15 m/s^2`, five times aDec. The robot
  overshoots by about 1.2 m, or the limiter clips the command and the robot falls behind.
- The cross-span backward pass gives vB at the span's entry = `sqrt(0 + 2·3·0.3) = 1.342 m/s`, imposed on the
  **previous** span, which brakes from 3 to 1.342 m/s over `(9 - 1.8)/6 = 1.2 m`. If that span is shorter than 1.2 m,
  the pass carries the limit back further.
- The same happens on the speed-up side without the forward pass.

#### 7.6.4 Measured start speed

```
V_meas  = measured field velocity (vx, vy) at initialize()
v0_par  = V_meas·T(0)           (component along the path)
v0_perp = V_meas·N(0)           (component across the path, + = moving left)
v0      = max(0, v0_par)
```

V_meas comes from the speed supplier: in the example drivetrain,
`fromRobotRelativeSpeeds(kinematics.toChassisSpeeds(getModuleStates(), gyroRate), heading)`, the same value the limiter
re-syncs from (**Fact** ExampleSwerveWithPose.java:335-336). Example: V_meas = (0.5, 1.0) m/s, T(0) = (1, 0) gives
v0_par = 0.5, v0_perp = 1.0, v0 = 0.5.

#### 7.6.5 Start stall, the runtime ramp and aAccEff

In v1 the command is exactly the time ramp below (the jerk-aware law of section 7.13 was removed in v1), with the lag lead and the
hand-over envelope of section 8.3.7. v1 has no motor curve: aAccEff = aA of the span (0 on a constant arc).

Starting the forward pass from the measured speed is right for the time estimate. It does **not** stop the robot from
stalling when it starts at rest: with v0 = 0, vProf(0) = 0, a follower that commands vProf(s*) commands 0, and s* stays 0
(`ds/dt = sqrt(2·a·s)` has the solution s = 0).

Fix (**Recommendation**, agreed with the follower): command from vBack, the profile **without** the forward pass, and do
the speed-up in time, at run time:

```
v_cmd[k] = min( vBack(s*_k), v_cmd[k-1] + aAccEff(v_cmd[k-1], s*_k) * dt )
v_cmd[-1] = v0                                       (dt measured by the follower, nominal 0.02 s)
aAccEff(v, s) = 0 on a constant-speed arc (no speed-up there, section 7.8); on a line or an ARC_VAR span:
                min( aAcc   (sqrt(1 - rho_lat^2)*aAcc on ARC_VAR),
                     alphaMax/abs(theta'(s))                                  (if theta' != 0),
                     0.9*maxForwardAccel*max(0, 1 - v/freeSpeed) )            (motor curve, if the config is given)
```

- `dv/dt = a` in time is the same constraint as `v·dv/ds = a` in s. If the robot tracks, it moves exactly along vProf.
- Braking stays tied to position (vBack(s*)), so stops and corners are safe even if the robot falls behind.
- The ramp uses the motor curve exactly, not the span's constant aA. On a constant-speed arc it holds the speed the robot had
  at the arc entry.
- The follower adds the lag lead and the hand-over point on top (section 8.3.7).

Example (v1): from rest with aAcc = 3, the commands are 0.06, 0.12, 0.18 m/s in successive loops, until they meet
vBack.

#### 7.6.6 Start faster than the envelope: no check

**Decision** (user, simplification pass): there is no start-speed check and no warning for it. If v0 > vBack(0),
the law brakes at once (v_cmd = vBack), and the limiter bounds the acceleration
(section 7.13.4). A start that cannot meet a cap ahead shows as a cross-track error on that arc, which the follower
corrects.

#### 7.6.7 Robot starts moving sideways

- v0_perp is not part of the profile. The follower command `V = v_t·T + c·N` has no N component when e_n = 0, so the
  limiter removes v0_perp at its skid limit.
- Time to remove it: `abs(v0_perp)/maxSkidAccel`; drift across the path meanwhile: `v0_perp^2/(2·maxSkidAccel)`. Example:
  v0_perp = 1.0, maxSkidAccel = 8: 0.125 s and 0.0625 m.
- The traction used is shared automatically by the limiter's skid disk, so the robot just falls a little behind.
- v0_par < 0 (moving backward along the path): v0 = 0. The limiter brakes the backward motion first.

### 7.7 Closed form inside a span

**Replaced in v1 by the per-loop limit law (see the v1 scope); kept as reference.**

For span k with entry speed `v_in = vF[k]`, exit speed `v_out = vF[k+1]`, cap `vCap = vCap[k]`, length l and
`q = s - sStart[k]` in [0, l]:

```
vProf(q) = min( vCap,  F_k(v_in, q),  sqrt(v_out^2 + 2*aD*(l - q)) )
vBack(q) = min( vCap,  sqrt(vB[k+1]^2 + 2*aD*(l - q)) )
```

The phase at q is the smallest term: speed-up (F), cruise (vCap), slow-down (sqrt term). The braking term of vProf uses
`vF[k+1]`, not `vB[k+1]` (**Inference**): if `vF[k+1] < vB[k+1]`, the forward pass limited it, so F_k is below it on the
whole span and the min is unchanged.

Phase boundaries and time with constant aA (the motor-curve version is in section 7.9):

```
q1 = (vCap^2 - v_in^2)/(2*aA)                (speed-up reaches vCap)
q2 = l - (vCap^2 - v_out^2)/(2*aD)           (slow-down starts)
if q1 <= q2:  cruise exists.  t = (vCap - v_in)/aA + (q2 - q1)/vCap + (vCap - v_out)/aD
else:         peak at q* = (v_out^2 - v_in^2 + 2*aD*l)/(2*(aA + aD)),  v_p = sqrt(v_in^2 + 2*aA*q*)
              t = (v_p - v_in)/aA + (v_p - v_out)/aD
aA = aD = 0 (a constant-speed arc, section 7.8; then v_in = v_out):  v = v_in on the whole span,  t = l/v_in
```

a(q) is +a(v) in speed-up, 0 in cruise, and -aD in slow-down.

Worked example, span 0 (line 0: v_in = 0, v_out = 2.3784, vCap = 4, l = 1.5307, aA = aD = 3) (hand, script):
- `q1 = 16/6 = 2.667 > q2 = 1.5307 - (16 - 5.6569)/6 = -0.193`: no cruise.
- `q* = (5.6569 - 0 + 9.1842)/12 = 1.2368 m`, `v_p = sqrt(6·1.2368) = 2.7241 m/s`.
- `t = 2.7241/3 + (2.7241 - 2.3784)/3 = 0.9080 + 0.1152 = 1.0233 s`.
- At s = 1.0: `vProf = min(4, sqrt(6.0) = 2.4495, sqrt(5.6569 + 6·0.5307) = 2.9734) = 2.4495 m/s`, speed-up, a = +3.
  `vBack(1.0) = min(4, 2.9734) = 2.9734 m/s`.

Span 3 (v_in = 2.3784, v_out = 0): `q* = (0 - 5.6569 + 9.1842)/12 = 0.2939`, `v_p = 2.7241`, `t = 0.1152 + 0.9080 = 1.0233 s`.

Edge cases: an arc span with v_in = v_out = 0 cannot happen: the path starts and ends on a line, every stop has a line on each
side (section 8.1), and a line ends with a speed > 0; if it ever happens, throw at build time (it is a bug). vCap = 0 only at a
zero-length span, which is merged away. Clamp the
radicands at 0; if q1 or q2 lands outside [0, l], use the "else" branch.

### 7.8 Arcs at constant speed, and ARC_VAR at a withVelocity point

> Removed in v1 (simplification); kept as v2 reference. In v1 every arc is driven at one constant speed and `withVelocity(v)` on a corner caps the whole arc at `v` (shared example: 5.51 s with `withVelocity(1.0)`, against 3.53 s without). The ARC_VAR text below is the design of the removed variable-speed arc.

Tangential and lateral acceleration share the tire grip. The record has a limit per direction, so the limit is an ellipse:

```
(a_t / aT)^2 + (a_n / aLat)^2 <= 1,   aT = aAcc (speeding up) or aDec (braking),   a_n = v^2*abs(kappa)
```

If aAcc = aDec = aLat = a_max, this is the friction circle. **Inference**: every point of the ellipse has
`a_t^2 + a_n^2 <= max(aT, aLat)^2`, so if aAcc, aDec and aLat are each <= maxSkidAccel, the profile stays inside the
limiter's skid disk.

**Rule** (**Decision**, user, simplification pass; it replaces the earlier two arc modes, one at the arc's cap and one
with a traction share for speed changes on the arc): **every arc is driven at constant speed; the speed changes only on
lines**, with one exception, the arc of a `withVelocity` point (ARC_VAR, below). On every constant-speed arc span `aA = aD = 0` and
the span cap is `min(vMax, sqrt(aLat/abs(kappa)), rotation cap, wheel cap)`. The junction passes then give all junctions of one
such arc the same speed, the **arc speed**:

```
v_arc = min( sqrt(aLat/abs(kappa)),                curvature cap (7.4.3)
             withVelocity of the arc's waypoint,     point cap (7.4.2), when it does not make the arc ARC_VAR
             heading-kink cap at the waypoint,       (7.4.6)
             rotation and wheel caps of both arc spans,
             vB at the arc exit,                     backward pass (plain in v1; the jerk-aware vJerk below is v2 reference)
             vF at the arc entry )                   forward pass: the speed reached on a short line before it
```

On the arc `a_t = 0` and `a_n = v_arc^2·abs(kappa) <= aLat`, so the arc is inside the ellipse by construction, with no
mode loop and no traction share to tune. The v1 runtime law holds the arc speed on the arc; it speeds up there only below the planned
arc speed vF (recovery after a correction-budget cut, section 8.3.7), never above it.
A speed-up on the line after an arc starts at the arc exit.

**ARC_VAR: the arc of a `withVelocity` point** (**Decision**, user: `withVelocity(v)` caps the speed at the point itself, so only
that point's arc may change speed). The arc through a waypoint P_i with `withVelocity(v)` is already split at s(P_i) (section 7.3.1).
If v is below the ARC_VAR cap of every span of that arc, all its spans become **ARC_VAR**, with a fixed share of the grip (the share
rule of the earlier design, kept for this arc only; ARC_RHO = rho_lat = 0.8, **Recommendation**):

| Span kind | Cap | aA, aD on the span | Ellipse check |
|---|---|---|---|
| ARC (constant speed) | `sqrt(aLat/abs(kappa))` | 0, 0 | a_t = 0, a_n <= aLat: on the ellipse |
| ARC_VAR | `sqrt(rho_lat·aLat/abs(kappa))` | `sqrt(1 - rho_lat^2)·aAcc`, `sqrt(1 - rho_lat^2)·aDec` | `(a_n/aLat)^2 <= rho_lat^2`, `(a_t/aT)^2 <= 1 - rho_lat^2`: sum <= 1 |

Both kinds also take the rotation and wheel caps; on ARC_VAR, alphaMax/abs(theta') lowers aA and aD as on a line, and the speed-up
uses the constant `aA = min(sqrt(1 - rho_lat^2)·aAcc, 0.9·maxForwardAccel·(1 - vCap/freeSpeed))` (motor limit at the span cap; arc caps
are low speeds, so this costs little). With rho_lat = 0.8 the cap drops to sqrt(0.8) = 89.4% of the curvature cap and the tangential
limits to 60%. Example: r = 1.4142, aLat = 4, aAcc = aDec = 3 give cap 2.1273 m/s and aA = aD = 1.8 m/s^2; at the cap
a_n = 3.2 = 0.8·aLat, and with a_t = 1.8 = 0.6·aT the use is 0.64 + 0.36 = 1.

Selection, once per sub-profile before the passes (mechanical, no loop):

```
for every waypoint P_i with withVelocity(v), v > 0, that lies inside an arc:
    capVar = min over the spans of that arc of the ARC_VAR span cap (with the rotation and wheel caps)
    if v < capVar:  every span of that arc is ARC_VAR; J at P_i = min(v, the other junction caps)
    else:           the arc stays constant speed; its arc speed is at most v
```

The second branch keeps the cap at the point and is the faster choice: there v >= capVar (at least 89% of the curvature cap), so an
ARC_VAR arc could not even reach v. Example (script): tight arc + P1.withVelocity(1.0) (ARC_VAR cap 0.9798, curvature cap 1.0954): the
arc is driven at 1.0 m/s, trapezoid 4.4075 s, v1 time 4.70 s.

On an ARC_VAR arc the junction passes work as on lines: the backward pass brakes on the span into P_i, the forward pass speeds up on
the span after it. The junctions at the two ends of the arc take the ARC_VAR cap (the smaller span cap on both sides), so the robot
reaches it on the line before the arc. The runtime law reaches the ARC_VAR entry with a = 0 like every arc entry, may then brake and
speed up inside the span's limits (traction clamp `a >= -aD`, "up" capped by aA), and passes P_i at no more than v (section 7.13.4).
A heading-kink cap at the waypoint of an ARC_VAR arc then also acts at the point only. Face-target sub-spans of the arc are all
ARC_VAR; the junctions between them only need v <= vt.

**Why a = 0 at the ARC_VAR entry** (**Recommendation**, chosen in this change; script). The entry speed is at most the ARC_VAR cap, so
with a = 0 there the use at the entry is at most rho_lat^2 = 0.64, and inside the arc the traction clamp keeps it <= 1. A speed-only
entry (the robot may keep braking from the line, where aD is larger, into the arc) reached 1.198 on ARC_VAR spans (above 1 in 5 of 428
random chains below the C2 threshold); with a = 0 the largest was 0.9505 (random chains below). The entry also gets the room-to-brake
cap vJerk (end of this section), so from a = 0 at the entry the law can meet P_i and every later limit inside the ARC_VAR share; in
the same chains vJerk at the entry changed the largest use by less than 0.005 and the summed time by +0.13%, so it is a guarantee more
than a fix (**Inference**).

**Cost.** The span method is exact (equal to the grid oracle with the exact ellipse, Appendix B) whenever every arc is
driven at its curvature cap, the common case, because the backward pass brakes on the line before the arc. A `withVelocity` on an
arc costs the ARC_VAR share; a heading-kink cap on an arc still slows the whole arc (script):

| Path | Grid + ellipse (Appendix B) | Span method | Penalty | Every arc constant (before ARC_VAR) |
|---|---|---|---|---|
| Shared example, EX | 3.2723 s | 3.2723 s | 0.00% | 3.2723 s |
| Shared example + P1.withVelocity(1.0) | 3.5770 s | 3.7751 s (ARC_VAR) | +5.54% | 5.2576 s (+47.0%) |
| Tight arc: (0,0), (4,0).withRadius(0.3), (4,4) | 4.3210 s | 4.3210 s | 0.00% | 4.3210 s |
| Tight arc + P1.withVelocity(0.5) | 4.4469 s | 4.5577 s (ARC_VAR) | +2.49% | 5.1528 s (+15.9%) |
| (0,0), (4,0), (4,1): short exit line, r = 0.5 | 3.0920 s | 3.0919 s | 0.00% | 3.0919 s |
| Shared example, DEF + motor curve (no heading anchors) | 2.3101 s | 2.3101 s | 0.00% | 2.3101 s |
| Tight arc, DEF + motor curve | 3.1056 s | 3.1055 s | 0.00% | 3.1055 s |
| Shared example, P1 at 180 deg (heading-kink cap 1.4088), EX | 3.4372 s | 4.2812 s (whole arc at 1.4088) | +24.6% | 4.2812 s |

The ARC_VAR rows equal the earlier two-mode rule with the same share (3.7751 and 4.5577 s), which used it on these arcs too.

With the shared example's P1 `withAngle(90 deg)` anchor and DEF + motor curve, the heading-kink cap at P1 (2.8176 m/s) is
below the arc cap (3.191 m/s), so the whole arc is driven at 2.8176 m/s: junctions vF = [0, 2.8176, 2.8176, 2.8176, 0], span
time **2.4549 s** (script; 2.439 s with the earlier rule). With EX the anchor changes nothing (its kink cap 2.818 is above the
EX arc cap 2.3784).

**Recommendation** for users: `withVelocity` on a corner point limits the speed at that point; the robot brakes on the arc into the
point and speeds up after it, with 89% of the arc's curvature cap on that arc. A heading turn that is too fast for a corner (the
heading-kink cap) slows the whole arc; spread the turn over more path (an earlier anchor) or use a larger radius.

**Runtime check** (ideal tracking, 50 Hz, the law of section 7.13; script). Largest traction-ellipse use
`(a_t/aT)^2 + (a_n/aLat)^2` at an arc entry, relative to the profile limits:

| Path, jerkMax 50 | Use at arc entry (ARC_VAR: largest on the arc) | v1 time (ideal) |
|---|---|---|
| Shared example, EX / DEF + motor | 0.9703 / 0.9955 | 3.54 s / 2.78 s |
| Tight arc, EX / DEF + motor | 0.9472 / 0.8892 | 4.62 s / 3.58 s |
| Two arcs joined by a 0.77 m line, the second faster: (0,0), (4,0), (5,1.2).withRadius(1.0), (9,1.2), EX | 0.9639 | 5.22 s |
| Two arcs joined by a 0.12 m line: (0,0), (4,0), (4.6,0.8).withRadius(1.0), (9,0.8), EX | 1.0000 | 6.30 s |
| Shared example + P1.withVelocity(1.0), EX (ARC_VAR) | 0.9608 (speeding up at 1.8 m/s^2 at 2.094 m/s, after P1) | 4.06 s |
| Tight arc + P1.withVelocity(0.5), EX (ARC_VAR) | 0.8885 | 5.00 s |

Over jerkMax 10 to 150 on these paths the use never exceeded 1: it is exactly 1.0000 where the robot enters an arc at its
curvature cap with a = 0 (the 0.12 m case), and at most 0.9982 elsewhere (shared example, jerkMax 10). So it stays <= 0.81 of
the drivetrain limits with PROFILE_MARGIN = 0.9. Random span chains with the room-to-brake rule (script): 600 chains of lines with
random aD, constant-speed arcs and point caps, and 1000 chains from the reviewer's generator, which adds short lines (0.02 to 0.3 m)
after arcs into slower arcs, line caps, stops and the end. Whenever jerkMax is below the threshold of warning C2
(`max(aAcc, aDec)/(5·dt_nom)`), the use stayed <= 0.99995 in all 1096 such chains (without the rule, 23 of the reviewer's 673 chains
below the threshold exceeded 1, up to 1.339). Above the threshold the law is effectively jerk-free, can reach an arc entry still
braking, and reached 1.97 (1.59 of the drivetrain limits) in 16 of 504 such chains; the jerk-free reference column of section 7.13.7
reaches 2.0 the same way. On the 0.12 m line the jerk-limited law gains little speed, so the second arc is driven slower than the closed-form
profile assumes (span time 5.69 s against 6.30 s); profileTime comes from the law, so the timeout is right.

Random chains with ARC_VAR arcs (script, 1200 chains, two seeds): lines with random aD (heading-limited) and caps, `withVelocity` on pass
markers, constant-speed arcs, ARC_VAR arcs (a `withVelocity` point at 15 to 85% of the arc, v from 0.15 to 1.2 of the ARC_VAR cap), short
lines after arcs, the end; aAcc, aDec, aLat from 2 to 8 m/s^2. Below the C2 threshold (841 chains, 531 with an ARC_VAR arc): use <= 0.99944
on all arcs in these chains (an independent run with another seed reached 1.0027 once, by the one-loop constant-arc entry effect below; treat the bound as <= 1 except that effect, <= about 1.08) and <= 0.9505 on ARC_VAR spans; the speed at a `withVelocity` point was never above v, and at any junction at most 0.05 mm/s
above vB; no stall; the goal was never passed. The summed time was 8533 s, against 10631 s with every arc at constant speed (-19.7%). Above the
threshold the use on ARC_VAR spans reached 1.49 and the speed at a point 5.2 mm/s above v (warning C2). These numbers take the law's state at
the end of every loop that ends on an arc, as the earlier chain checks did. A stricter check that also interpolates the state where a loop
crosses a junction next to an arc reached 1.074 in 3 of the 841 chains, always at the entry of a constant-speed arc, and the same with every
arc at constant speed (2 chains, 1.074): the robot gets below vt one loop before the entry while still braking, safe() then skips the entry,
and the last loop of the ramp-out crosses it. On ARC_VAR spans the stricter check gives the same 0.9505. **Uncertain**: this one-loop
quantization of the a = 0 rule is older than ARC_VAR; the implementation tests should look at it (PROF-J17).

**Room to brake after an arc** (**Decision**, review of the simplification pass). The jerk-free backward pass leaves the line after
an arc exactly the jerk-free braking distance to the next limit. The jerk law needs more (it ramps the deceleration in and out), so on
a line about that short it ran out of room and braked at aDec while still on the arc, beyond the ellipse. So for every line span j that
starts at the exit of a constant-speed arc, and for the first span j of every ARC_VAR arc (where the law also holds a = 0), the backward
pass also applies

```
vJerk(j) = the largest v such that, for every later constraint q of the sub-profile,
           stopDist(v, 0, vt_q, jerkMax, D_q) + STOP_DIST_MARGIN <= s_q - s_j
           (q: every later junction with vt_q = vB[q], and the hand-over point s_goal - d_h with vt = v_h;
            D_q = min aD over the line and ARC_VAR spans between, as in section 7.13.4)
```

found by bisection on [0, vB[j]] (stopDist grows with v, so the test is monotone; 60 steps). It caps vB at the arc exit, and the pass
(aD = 0 on the arc) carries it to the whole arc; at an ARC_VAR entry it caps vB there. From the arc exit at a = 0 the law can then
meet every limit after the arc with jerk, so it never has to brake on the arc. Lines that do not start at an arc exit keep the
jerk-free pass: the law brakes on the line before them. Cases from the review (ideal tracking, jerkMax 50 unless stated; script):

| Path | Arc speed before -> after | Ellipse use on the arc before -> after | v1 time before -> after |
|---|---|---|---|
| DEF: (0,0), (6,0).withRadius(2), (6,3.2).withRadius(0.5), (3,3.2) | 3.7947 -> 3.4425 m/s | 1.033 -> 0.964 | 4.64 -> 4.84 s |
| DEF: line 3 m, arc r = 2 (2 m), 0.75 m line, arc r = 0.5 (0.8 m), line 2 m | 3.795 -> 3.035 m/s | 1.224 -> 0.869 | 3.70 -> 3.72 s |
| Same with a 1.0 m line | 3.795 -> 3.510 m/s | 1.033 -> 0.989 | 3.72 -> 3.72 s |
| EX, same chain with a 1.0 m line | 2.828 -> 2.689 m/s | 1.378 -> 0.907 | 4.78 -> 4.84 s |
| EX, jerkMax 30, 1.0 m line | 2.828 -> 2.601 m/s | 1.259 -> 0.971 | 4.84 -> 4.92 s |

The rule costs time only where the line after an arc is short against the braking it must hold (up to +0.26 s in the cases tried, a
0.5 m line at DEF). It changes no path of the rest of this document: their lines after arcs are long enough, so every vB, span time and
v1 time quoted elsewhere is the same, and at the ARC_VAR entries of the examples in this section it does not bind (script).

Edge case: an arc right after the start or a stop is entered at the speed reached on the line before it; a very short line
there (possible only with `withRadius`) gives a slow arc.

### 7.9 Motor curve on the speed-up phase

> Removed in v1 (simplification); kept as v2 reference.

**Fact** (SwerveAccelerationLimiter.java:100, :107): the limiter lets speed grow by at most
`maxForwardAccel·max(0, 1 - speed/freeSpeed)·dt` per loop. A brushless motor gives this shape: torque falls linearly with
speed because back-EMF eats the supply voltage. A constant aAcc is outside this envelope at high speed.

Model for the speed-up phase on a line span:

```
a(v) = min( aAcc, a_0*(1 - v/v_free) ),   a_0 = 0.9*maxForwardAccel,   v_free = freeSpeed
knee:  v_knee = v_free*(1 - aAcc/a_0)      (a = aAcc below v_knee, the motor curve above; v_knee = 0 if aAcc >= a_0)
```

DEF: a_0 = 9, v_free = 5.3, aAcc = 7.2 give `v_knee = 5.3·(1 - 0.8) = 1.06 m/s`. With aAcc = 3 (EX): `v_knee = 3.53 m/s`.

**Closed form in s.** Above the knee, `v·dv/ds = a_0·(1 - v/v_free)`:

```
ds(v1 -> v2) = (v_free/a_0) * [ v_free*ln((v_free - v1)/(v_free - v2)) - (v2 - v1) ]
dt(v1 -> v2) = (v_free/a_0) * ln((v_free - v1)/(v_free - v2))
```

Derivation: separate the variables, `v·dv/(1 - v/v_free) = a_0·ds`. With `v = v_free·(1 - z)`, `dv = -v_free·dz`, the left side
is `-v_free^2·(1/z - 1)·dz`, which integrates to `v_free^2·[ln(z1/z2) - (z1 - z2)]`. Time: `dv/dt = a_0·(1 - v/v_free)`
integrates to the log. Below the knee the usual `(v2^2 - v1^2)/(2·aAcc)` and `(v2 - v1)/aAcc`; add the two parts across v_knee.

Inverse v(s): a closed form exists with the Lambert W function (`v = v_free - (v_free - v1)·z`,
`z = -W0(-b_W·e^(-b_W - k_W))/b_W` with `b_W = (v_free - v1)/v_free`, `k_W = a_0·ds/v_free^2`). Java has no W, so
(**Recommendation**) **invert ds(v) numerically**: it is strictly increasing. Bisection on `[v1, v_free·(1 - 1e-12)]`,
50 iterations, or Newton with `d(ds)/dv = v/a(v)`. One inversion per span at build time, and one per loop only while
in a speed-up phase.

Worked numbers (script):
- Pure motor curve a_0 = 9, v_free = 5.3, 0 -> 2 m/s: `ds = 0.5889·[5.3·ln(5.3/3.3) - 2] = 0.5889·(2.5110 - 2) = 0.3010 m`,
  `dt = 0.5889·0.4738 = 0.2790 s`. Constant 9 m/s^2 would give 0.2222 m and 0.2222 s. The Lambert-W inverse at 0.3009 m
  gives 1.9998 m/s.
- Traction plus motor, `min(7.2, 9·(1 - v/5.3))`, 0 -> 2 m/s: `1.06^2/14.4 + 0.5889·[5.3·ln(4.24/3.3) - 0.94] = 0.0780 + 0.2288 = 0.3068 m`;
  `t = 0.1472 + 0.1476 = 0.2948 s`.
- 0 -> 4 m/s on the pure motor curve: 2.0307 m, 0.8276 s.

Effect on whole paths (script):
- Shared example, EX: the knee 3.53 m/s is above the 2.7241 m/s peak, so no change: 3.2723 s.
- Shared example, DEF, no heading anchors: constant 7.2 (outside the limiter above 1.06 m/s) 2.2573 s; exact motor ramp **2.3101 s** (+2.3%);
  the motor limit folded into a constant per span, `aA = min(7.2, 9·(1 - vCap/5.3)) = 2.12` on lines, 2.8188 s (+22%);
  tight arc path +26% for the folded constant. With the P1 heading anchor (DEF + motor): 2.4549 s (section 7.8).
- So a pure constant-acceleration trapezoid is either outside the limiter (fast, wrong) or 22 to 33% slow (folded). The
  closed-form motor ramp fixes both at no time cost: span method = grid method to 4 decimals.

Braking: the motor helps braking (back-EMF adds to the reverse voltage), so braking is traction-limited and aDec stays
constant. The limiter agrees: braking is free from the forward limit (**Fact** SwerveAccelerationLimiter.java:94).

### 7.10 Stop corners and user stops

#### 7.10.1 Sources

| Source | Geometry | Profile |
|---|---|---|
| Automatic STOP corner: tau_turn > TAU_STOP, r below R_MIN after the shrink loop, `.withRadius(x < R_MIN)` (sections 5.6.4, 5.7.3) | zero-length STOP entry, lines on both sides | junction speed 0 |
| User stop: `.stop()`, `withVelocity(0)`, `.stop(dwellSeconds)` on a middle waypoint | the corner's arc is **removed** (section 5.8.7) | junction speed 0, plus the dwell |
| Last point | end of the path | J[K] = 0 (default); `.stop(d)` there adds a dwell after the finish test (section 8.6) |
| `.stop()` on P_0 | ignored (the path starts at rest, W4) | none; `stop(d)` on P_0 delays the start by d |

#### 7.10.2 Profile math at a stop

A stop at s_c is a span boundary with J = 0. **The profile is split there into sub-profiles** [0, s_c1],
[s_c1, s_c2], ..., [s_cN, L]. Each is built on its own, with the same span rules:
- Sub-profile j ends with J = 0 at s_c.
- Sub-profile j+1 **starts** at s_c with its first junction cap set by the normal rules (vMax, the caps of its first
  span), **not 0**, so its vBack(s_c) > 0 (resolution R10). Its forward pass starts from v0 = 0, because the robot is at
  rest after the stop.

Braking into a stop (last span before s_c, exit 0): `vBack(q) = min(vCap, sqrt(2·aD·(l - q)))`; the braking distance from v
is `v^2/(2·aD)`. Restart: `vProf(q) = min(vCap, F(0, q), braking term)`; at run time the speed law restarts from the state
`(max(0, V_meas·T_out), 0) ~= (0, 0)` (sections 7.13.2, 8.6).

#### 7.10.3 No stall at the stop

- **Approach**: vBack_j goes to 0 as s* goes to s_c. A purely position-indexed command would creep and never arrive. The
  speed law brakes to the hand-over point (s_c - d_h, v_h = k_p·d_h), not to 0 at s_c; the 2D CAPTURE_STOP law then commands
  `k_p·(P_c - p)`, which is not 0 until the robot is there (section 8.3.7).
- **Arrival** (section 8.6): `abs(d_go) <= pos_tol` (0.03 m) along-track and `abs(V_meas) <= STOP_SPEED` (0.1 m/s).
- **Leave**: the follower switches to sub-profile j+1, where vBack(s_c) is the restart envelope (4.0 m/s in the example
  below), and the speed law restarts from (~0, 0).

#### 7.10.4 Dwell

Order (decision 15, section 8.6): arrive, then correct a residual heading error if it is more than 3 deg (ROTATE, section
6.7), then dwell, then leave. The profile counts only the user's `dwellSeconds` (`.stop(0.5)`: 0.5 s).

There is no turn-in-place term (theta is continuous through the stop, section 6.7), and the dwell is the only stop term
(**Decision**, user, simplification pass). The modules turn to the new travel direction in at most `90 deg/maxSteerRate = (pi/2)/12 =
0.13 s` (the kinematics reverses a module instead of turning it more than 90 deg, **Fact** DemaciaKinematics.java:194-197;
maxSteerRate, KinematicsConfig.java:41); the timeout margin covers it.

#### 7.10.5 Time with stops

```
profileTime = sum over sub-profiles (sum over spans t_k) + sum over stops (dwell) + dwell at the last point
t_timeout   = 1.5 * profileTime + 1.0                           (decision 10)
```

In v1 the motion part comes from the offline simulation of the speed law, including the hand-over tail
`max(0, ln(d_h/pos_tol)/k_p)` per goal (0.170 s for d_h = 0.05, k_p = 3; 0 when d_h < pos_tol, i.e. jerkMax < 13.5 m/s^3); see section 7.13.6. The closed-form span sum is the no-jerk
lower bound. profileTime always covers the whole path from s = 0 (there is no mid-path start, section 8.2.5).

#### 7.10.6 Example: P1.stop() on the shared example

Geometry (section 5.8.7): LINE 2.8284 m, STOP at s = 2.8284, LINE 2.8284 m; L = 5.6569 m. Constraints EX.

| Sub-profile | Span | v_in | v_out | vCap | Peak | Time |
|---|---|---|---|---|---|---|
| 1, [0, 2.8284] | line | 0 | 0 | 4 | `q* = 6·2.8284/12 = 1.4142`, `v_p = sqrt(6·1.4142) = 2.9130` | `2·2.9130/3 = 1.9420 s` |
| 2, [2.8284, 5.6569] | line | 0 | 0 | 4 | 2.9130 at s = 4.2426 | 1.9420 s |

- vBack in sub-profile 1: `vB[0] = min(4, sqrt(6·2.8284) = 4.1195) = 4.0`, and `vBack(s) = min(4, sqrt(6·(2.8284 - s)))`.
  Braking into P1 starts at the peak, s = 1.4142.
- In sub-profile 2, vBack(2.8284) = **4.0** > 0: the restart cannot stall.
- Heading: theta' = 0.5554 rad/m before the stop, 0 after; the robot reaches 90 deg at the stop (section 6.11).
- Motion time 3.8839 s (matches `2·sqrt(2.8284/3) = 1.9420` per leg).

| Variant | Trapezoid motion (no jerk limit) | Dwell | Trapezoid total | old profileTime (jerk law, v2 reference) | old timeout |
|---|---|---|---|---|---|
| P1.stop() | 3.8839 | 0 | 3.8839 s | **4.32 s** | **7.48 s** |
| P1.stop(0.5) | 3.8839 | 0.5 | 4.3839 s | 4.82 s | 8.23 s |
| P1.stop(1.5) | 3.8839 | 1.5 | 5.3839 s | 5.82 s | 9.73 s |
| P1.stop(0.5), P1.withAngle(0), P2.withAngle(90 deg): the heading turns on the second leg (theta' = 0.5554), no cap binds (hand) | 3.8839 | 0.5 | 4.3839 s | 4.82 s | 8.23 s |
| no stop (arc, shared example) | 3.2723 | 0 | 3.2723 s | 3.54 s | 6.31 s |

The stop costs 0.61 s here with the trapezoid (v1 profileTime 4.2245 against 3.4426 s; 0.78 s with the old jerk law). Keeping the arc with a stop on it (the rule before decision 17) would take 4.1048 s of motion
(script, computed with the earlier arc rule that allowed braking on the arc), 0.22 s slower than the two straight lines. That supports decision 17.

### 7.11 Swerve wheel-speed limit with rotation

> Removed in v1 (simplification); kept as v2 reference.

A module at `r_m` from the robot center moves at `V + omega x r_m`. Its speed is at most `abs(V) + abs(omega)·abs(r_m)`
(triangle inequality, equal when the two vectors line up). With R_mod = the largest `abs(r_m)` and
vWheelMax = `maxAllowedModuleVelocity`:

```
abs(v) + abs(omega)*R_mod <= vWheelMax            (worst case over modules and directions)
```

- For a given omega: `v <= vWheelMax - abs(omega)·R_mod`.
- In the profile, `omega = theta'·v`, so `v·(1 + abs(theta')·R_mod) <= vWheelMax`:
  `vCap[k] <= 0.9·vWheelMax/(1 + abs(theta'[k])·R_mod)`. It is linear in v, so it is one more span cap. The 0.9 margin
  leaves room for the cross-track correction and the heading feedback.

Numeric example: modules at (+-0.275, +-0.275) m (the layout in `docs/Kinematics-design.md`), so
`R_mod = 0.275·sqrt(2) = 0.3889 m`; vWheelMax = 4.5 m/s (**Fact** KinematicsConfig.java:32).

| Situation | Result |
|---|---|
| omega = 0 | v <= 4.5 |
| omega = 2 rad/s | v <= 4.5 - 2·0.3889 = 3.722 m/s |
| omega = 2·pi rad/s | v <= 4.5 - 6.283·0.3889 = 2.057 m/s |
| theta' = 1 rad/m, no margin | v <= 4.5/1.3889 = 3.240 m/s |
| theta' = 1 rad/m, with margin | v <= 4.05/1.3889 = 2.916 m/s |
| theta' = 0.5256 rad/m (shared example), with margin | v <= 4.05/1.2044 = 3.363 m/s |

The worst case is simpler and conservative, so v1 uses it (**Recommendation**). Without this cap,
`DemaciaKinematics.desaturate` scales translation and rotation by one factor (**Fact** DemaciaKinematics.java:260-262,
:331-345): the robot stays on the path but falls behind the profile time, and the heading lags. Acceptable when R_mod is
unknown. Edge case: `omegaMax·R_mod >= vWheelMax` gives v <= 0 for a turn in place; check `omegaMax·R_mod < vWheelMax`
when the profile is built: `2·pi·0.3889 = 2.44 < 4.5`, OK.

### 7.12 Time from the profile

**Replaced in v1 by the per-loop limit law (see the v1 scope); kept as reference.**

Per span, the closed-form phase times (section 7.7); on a motor-curve speed-up, the log formula plus the constant part
below the knee. Then:

```
t0[0] = 0,  t0[k+1] = t0[k] + t_k
spanTime    = sum of sub-profile span times (section 7.10.5)
profileTime = spanTime + hand-over tail max(0, ln(d_h/pos_tol)/k_p) per goal + dwells   (v1, closed form)
t_timeout   = 1.5 * profileTime + 1.0 s
```

Time to a point inside a span (for logging) adds the phase times before q. Use vF (vProf), not vB, for time.

Examples (closed form):
- Shared example: `1.0233 + 0.6129 + 0.6129 + 1.0233 = 3.2723 s` (script). v1 profileTime = 3.2723 + 0.1703 (tail) = 3.4426 s
  (Java), timeout 6.16 s.
- Arc span at constant speed: `1.4577/2.3784 = 0.6129 s`.

### 7.13 Runtime speed law: jerk-aware online trapezoid

**Removed in v1; kept as v2 reference.** v1 uses the plain trapezoid law (sections 7.6.5, 8.3.7 and the v1 scope section); none of
stopDist, the landing rule, D_brake, vJerk (7.8), STOP_DIST_MARGIN or the offline profileTime of 7.13.6 is in the code.

**Decision** (brief decision 18): the v1 tangential speed command is a **jerk-aware online trapezoid**. Each loop it asks: "can I
still speed up for one more loop and then slow down in time for every limit ahead, with jerk limit jerkMax and deceleration D? If
yes, speed up. If not, hold or reduce the acceleration. If even that is not safe, brake." The jerk limit is **inside** that test,
not a filter after it. The per-span profile (sections 7.3 to 7.10) stays the source of the limits: junction speeds vB, span caps
vCap and per-span decelerations aD. All numbers in this section are (script) unless marked; the scripts are Python models of
this law, of its stopping distance, and of the follower's plant model.

#### 7.13.1 Why a jerk filter after the profile overshoots

The naive layer (section 7.15.3) looked up vBack at `s* + v·A/(2·jerkMax)` and tracked it with a jerk-limited filter. Its braking
decision depended only on v; it ignored the current acceleration a, and with it the time needed to turn +a into -D. Replay on the
shared example with jerkMax = 30: at t = 2.30 s, s = 4.533 m, v = 2.519 m/s and a = +2.62 m/s^2 (still speeding up), the stopping
distance from (v, a) was 1.514 m with 1.444 m left, so it was already too late; the old law cut its acceleration only 0.04 s later
and came to rest at s = 6.262 m against L = 5.977 m (6.382 m with jerkMax = 10). **Rejected as a filter.** The law below includes
a in the stopping distance and decides before each step.

#### 7.13.2 State

```
v_cmd, a_cmd      the command state, integrated from the law's own commands every loop
init:             v_cmd = v0 = max(0, V_meas·T(0)),  a_cmd = 0
after a stop:     v_cmd = max(0, V_meas·T_out) (~= 0),  a_cmd = 0
```

The measured velocity is used only at initialize and at each restart after a stop. It is never fed back
every loop (the 2026 bug). The position input is the follower's lag-led progress `s_v = min(s* + v_cmd·tau_lag, s_goal - d_h)`.
**The lead is required at run time** (the offline profileTime run uses tau_lag = 0, section 7.13.6): without it the plant simulation passes the goal by 2.7 cm (shared example), 4.5 cm
(P1.stop()) and 6.2 cm (tight arc); with tau_lag = 0.05 s none of the runs passes the goal.

#### 7.13.3 Stopping distance from (v, a) to a target speed v_t

`stopDist(v, a, v_t, jerkMax, D)`: the shortest distance (and time) from speed v >= v_t and acceleration a to v_t with a = 0 at
the end, with jerk limit jerkMax and deceleration limit D > 0.

Phase 0, if a > 0 (still speeding up): ramp a down to 0 first:

```
dt0 = a/jerkMax,   dv0 = a^2/(2*jerkMax),   dd0 = v*a/jerkMax + a^3/(3*jerkMax^2);   then v <- v + dv0,  a <- 0
```

Then, with `b = -a >= 0` (current deceleration):

| Case | Condition | Distance d | Time |
|---|---|---|---|
| Over-braking: ramp out now, v reaches v_t during the ramp-out | b > 0 and `v - v_t <= b^2/(2·jerkMax)` | `t1 = (b - sqrt(b^2 - 2·jerkMax·(v - v_t)))/jerkMax`, `d = v·t1 - b·t1^2/2 + jerkMax·t1^3/6` | t1 |
| Full D (reaches -D, holds it, ramps out) | `dv' >= D^2/jerkMax` | `d' = (v'^2 - v_t^2)/(2D) + (v' + v_t)·D/(2·jerkMax)` | `dv'/D + D/jerkMax` |
| Triangular (peak deceleration `sqrt(dv'·jerkMax) < D`) | `dv' < D^2/jerkMax` | `d' = (v' + v_t)·sqrt(dv'/jerkMax)` | `2·sqrt(dv'/jerkMax)` |

For the last two rows `v' = v + b^2/(2·jerkMax)` is the "virtual start" (the speed at a = 0 before the ramp 0 -> -b that has already
happened), `dv' = v' - v_t`, and `d = d' - (v'·b/jerkMax - b^3/(6·jerkMax^2))`, `t = t' - b/jerkMax` (subtract the virtual ramp already
driven). With a = 0, d = d'.

Derivation: in the full-D case the deceleration is ramp in (D/jerkMax), hold, ramp out (D/jerkMax); the velocity curve is
point-symmetric about its midpoint, so the mean speed is exactly `(v + v_t)/2` over the time `dv/D + D/jerkMax`. In the triangular
case each ramp takes `sqrt(dv/jerkMax)` and changes the speed by dv/2; the mean speed is again `(v + v_t)/2`. Phase 0 integrates
`a(t) = a - jerkMax·t` over a/jerkMax.

Verification against numerical integration of the same jerk profile (step 1e-5 to 2e-5 s):

| Case (jerkMax = 30, D = 3) | Formula | Integrated |
|---|---|---|
| v = 3, a = 2, v_t = 0 (phase 0 + full D) | 1.92370 m, 1.1889 s | 1.92370 m, 1.1889 s |
| v = 3, a = 0, v_t = 1 (full D) | 1.53333 m | 1.53333 m |
| v = 1, a = 0, v_t = 0.8 (triangular, peak 2.449 m/s^2) | 0.14697 m | 0.14698 m |
| v = 2.5, a = -1.5, v_t = 0.5 (already braking) | 1.05711 m | 1.05710 m |
| v = 4, a = -3, v_t = 0 (braking at D) | 2.66792 m | 2.66790 m |
| v = 2.7241, a = 3, v_t = 2.3784 (peak of the shared example, into the arc) | 0.97898 m | 0.97899 m |

3000 random cases (jerkMax in {10, 30, 50, 150}, D in [1, 8]): largest difference 2.9e-4 m, which is integration error (at step
5e-7 s the worst case converges to within 6e-7 m). The over-braking row matches to within 1e-4 m in 2000 random cases.

Worked numbers:

| From | jerkMax, D | Distance | Constant-deceleration trapezoid |
|---|---|---|---|
| 2.7241 -> 2.3784 m/s, a = 0 | 30, 3 | 0.5491 m | 0.2940 m |
| same | 10, 3 | 0.9487 m | 0.2940 m |
| same | 50, 3 | 0.4471 m | 0.2940 m |
| 3 -> 0, a = 0 | 30, 3 | `9/6 + 3·3/60 = 1.6500 m` (full D) | 1.5000 m |
| 3 -> 0, a = +3 | 30, 3 | dd0 = 0.31, v' = 3.15; `1.8113 + 0.31 = 2.1212 m` | 1.5000 m |
| 2 -> 0.5, a = -2 | 30, 3 | v' = 2.0667; `0.7985 - 0.1363 = 0.6622 m` | 0.6250 m |
| 1 -> 0.9, a = -2.5 (over-braking) | 30, 3 | t1 = 0.0667; 0.0626 m | 0.0333 m |
| 4 -> 0, a = 0 | 30, 7.2 | 1.5911 m | 1.1111 m |

Edge cases: v <= v_t and a <= 0: distance 0. v <= v_t with a > 0: phase 0 may still raise v above v_t; compute it. jerkMax or D
<= 0: invalid (V16). Clamp radicands at 0.

#### 7.13.4 The decision, once per loop

Inputs: the state (v, a) = (v_cmd, a_cmd), the position s = s_v, the current span k with cap `cap = vCap[k]` (also limited
by the speed budget of section 8.3.9), and the constraint list of the current sub-profile:

```
for every junction j ahead in the sub-profile:   (s_j, vt_j = vB[j], D_j)
the goal (stop or end):                           (s_goal - d_h, vt = v_h = k_p*d_h, D_goal)     (hand-over point, 8.3.7)
D_j    = min(aD[k] of every line or ARC_VAR span overlapping [s, s_j])
                                       (aD[k] = min(aDec, alphaMax/abs(theta'[k])) on a line, min(sqrt(1 - rho_lat^2)*aDec,
                                        alphaMax/abs(theta'[k])) on ARC_VAR; constant-speed arc spans are skipped; aDec if none)
D_goal = the same rule over [s, s_goal - d_h]
```

Two kinds of constraint (**Decision**, simplification pass, with the constant-speed arcs of section 7.8):
- an **arc entry** (a junction from a line span into an arc span, constant-speed or ARC_VAR) and the **hand-over point** must be
  reached with v <= vt **and a = 0, unless the robot is already below vt** (safe() skips a constraint once v1 <= vt and a1 <= 0),
  because a constant-speed arc then holds its speed, an ARC_VAR arc starts its grip share from a = 0 (section 7.8), and the CAPTURE
  law starts from a = 0;
- every other junction (between two line spans or two ARC_VAR spans: a PASS marker, the `withVelocity` point inside an ARC_VAR arc,
  a heading anchor on a line, a face-target sub-span)
  only needs v <= vt. Its test is skipped while `v1 + max(0, a1)^2/(2·jerkMax) <= vt`, because ramping a to 0 from there
  can never pass vt. Without this skip the law forces a = 0 at every such junction: on a 6 m line cut into 0.1 m
  face-target sub-spans it took 4.80 s instead of 3.08 s (EX; DEF + motor 4.70 instead of 2.46 s; the uncut line 3.06 and
  2.42 s) (script).

Braking deceleration, chosen in the loop that reaches **brake** (resolution of a review finding: the minimum over all constraints
would let a low-aD span far ahead weaken the braking for a nearer junction):

```
failed  = the constraints whose stopDist test failed for the candidates tried this loop (the binding ones)
D_brake = max(D_j over failed)                       if failed is not empty
        = D_j of the nearest constraint              otherwise (brake reached through the cap or landing tests only)
D_brake = min(D_brake, aD_cur)                       aD_cur = aD[k] of the current line or ARC_VAR span; aDec on a
                                                     constant-speed arc
                                                     (emergency only, below); safety clamp
```

Every D_j covers the current span when it is a line or ARC_VAR, so `D_j <= aD_cur` already holds there; the clamp only guards rounding
and future changes. On a constant-speed arc the law does not need to brake by construction: the jerk-aware pass of section 7.8 leaves
room to meet every limit after the arc with jerk jerkMax from a = 0 at the arc exit. If it ever must (a robot pushed above the arc
speed, a cut by the speed budget), it brakes at aDec. That is an emergency: the ellipse is exceeded for a moment and the limiter's
skid disk bounds the vector.

stopDist is conservative when the current deceleration b already exceeds D_j (it still plans with b, then at most D_j).

**Traction clamp on every candidate.** All candidates ("up", "toward 0", "keep" and "brake") are clamped to `a1 >= -aD_cur`, so the
deceleration of one line span does not carry onto a following line or ARC_VAR span whose aD is lower (heading-limited, or the ARC_VAR
share). When the clamp cuts a deceleration that is larger than aD_cur, that one loop can step the acceleration by more than
jerkMax·dt. Recommendation: accept that step. Traction takes priority over the jerk limit, because exceeding traction skids the wheels
while a single jerk step does not.

All junctions of the sub-profile are checked every loop; there are only a handful. Candidates, tried in order:
1. **up**, on a line or ARC_VAR span (never on a constant-speed arc, section 7.8): `a1 = min(a + jerkMax·dt, Amax(v, s))`, with
   `Amax = aAccEff` (section 7.6.5: the ARC_VAR share, alphaMax/abs(theta'), 0.9·motor curve);
2. **toward 0**: `a1 = a - sign(a)·min(abs(a), jerkMax·dt)`;
3. **keep**: `a1 = a`.

The first candidate that passes `safe` is taken; otherwise **brake**:
`a1 = min(a + jerkMax·dt, max(a - jerkMax·dt, -D_brake, -sqrt(2·jerkMax·v)))`.

```
v1 = v + (a + a1)/2*dt,   s1 = s + (v + v1)/2*dt            (state after this loop)
safe(v1, a1, s1) =
    not (v1 > cap and a1 >= 0)
    and not (a1 > 0 and v1 + a1^2/(2*jerkMax) > cap)        (can still level off at the cap: cruise mode)
    and for every constraint j:
          (v1 <= vt_j and a1 <= 0)
          or (j is a line-to-line junction and v1 + max(0, a1)^2/(2*jerkMax) <= vt_j)
          or stopDist(v1, a1, vt_j, jerkMax, D_j) + STOP_DIST_MARGIN <= s_j - s1
    and a1 >= -sqrt(2*jerkMax*v1)                            (candidates 2 and 3; landing rule)
```

- **One-step lookahead** handles the 50 Hz quantization: the test is made on the state after the next step, so the law never commits
  to a step it could not undo. dt is the follower's measured dt (section 8.2.2).
- **Cruise mode**: at the cap, "up" fails the level-off test and "toward 0" keeps a = 0. The same mechanism lands on every v_t with
  a = 0. In the ideal-tracking runs (EX and DEF, jerkMax 30 to 150) the speed at an arc entry was 2 to 47 mm/s below vB, and the
  arc was then driven at that speed (no "up" on a constant-speed arc); against the earlier arc rule this changed the times of section 7.13.7 by
  0 to +0.02 s at jerkMax 50 (-0.16 to +0.16 s over jerkMax 10 to 150). In the random span chains of section 7.8 the speed
  interpolated exactly at a junction never exceeded vB while jerkMax is below the C2 threshold; above it, up to 6.7 mm/s above vB
  (jerkMax 150), back at or below vB at the next 20 ms sample. No run stalled (script). The 0.9 profile margin covers this. With
  ARC_VAR arcs (section 7.8) the speed at a `withVelocity` point never exceeded v below the C2 threshold, and was up to 5.2 mm/s above
  it above the threshold.
- **Per-constraint braking check** (script): a 3 m line to vB = 1.0 m/s, then a 1 m line whose aD is lowered to 1.8 m/s^2 by the
  heading (alphaMax/abs(theta')) to vB = 0.5 m/s, then a 2 m line; jerkMax 50, aAcc = aDec = 3. With D_brake as above the robot
  reaches both junctions 0.020 m/s below vB. With the minimum over all constraints (D_brake = 1.8) it would reach the first
  junction 0.939 m/s **above** vB.
- **Landing rule** `abs(a) <= sqrt(2·jerkMax·v)`: ramping out from deceleration b loses `b^2/(2·jerkMax)` of speed, so a larger
  deceleration would reach v = 0 with a != 0 and force a jump. Without it, jerkMax = 30 at DEF showed spikes of 270 to 330 m/s^3.
- **Speed floor**: if v1 <= 0 anyway, set v1 = 0 and a1 = max(a1, 0); with the landing rule it did not occur on the paths of this
  document in ideal tracking (once in the plant model, section 7.13.8). In the random chains of section 7.8 it acted once in 19 of the 841 runs below the C2 threshold, all with jerkMax 10, and
  as often with every arc at constant speed (17 runs): the "up" candidate, which the landing rule does not test, lets the deceleration
  toward the hand-over point outlast the speed, so the robot stops a few cm before that point and starts again. No run stalled (script).
- STOP_DIST_MARGIN = 0.005 m (**Recommendation**) covers rounding and projection noise; a `v·dt` term is not needed because of the
  one-step lookahead (it only cost 0.06 to 0.2 s).
- A sudden drop of Amax (the motor curve) may cut a by more than jerkMax·dt: traction wins. In the simulations it occurred only
  with jerkMax = 10 at DEF + motor (one step of up to 79 m/s^3 per run).

Worked loop (shared example, jerkMax = 30, D = 3, dt = 0.02; a made-up state already too late): v = 2.60, a = +3.0 at s = 1.00 m;
binding constraint the arc entry, vt = 2.3784 at s = 1.5307 m.
- "up": a1 = min(3.6, Amax = 3) = 3.0, v1 = 2.66, s1 = 1.0526. stopDist(2.66, 3.0, 2.3784): phase 0 0.1 s, +0.15 m/s, 0.276 m; full
  D from 2.81 to 2.3784: 0.3732 + 0.2594 = 0.6326 m; total 0.9086 m, plus margin 0.9136 m > 0.4781 m left: not safe.
- "toward 0": a1 = 2.4, v1 = 2.654, s1 = 1.0525, stopDist 0.7915 m + margin > 0.4782: not safe. "keep" equals "up" here.
- So brake. The failed constraint is the arc entry, with only line 0 between, so D_j = 3 and D_brake = 3 (= aD_cur):
  `a1 = min(3.6, max(2.4, -3, -12.5)) = 2.4`. The arc entry is then a few mm/s over vB; the 0.9 profile margin
  (section 7.2.1) covers that, not the limiter, which does not know vB.

In a consistent run from rest (ideal tracking): with jerkMax = 30, a starts to drop at s = 0.938 m (v = 2.37) and the speed peaks at
2.52 m/s at s = 1.184 m; with jerkMax = 50, a starts to drop at s = 1.034 m and the speed peaks at 2.58 m/s (the trapezoid peak is
2.72 m/s); the arc is entered at 2.376 and 2.360 m/s, below its cap 2.3784, with a = 0, and driven at that speed.

Start from rest on line 0 (EX, jerkMax = 50, Amax = 3) (hand, script): a = 1, 2, 3, 3, 3, ... m/s^2 and
`v_cmd = 0.01, 0.04, 0.09, 0.15, 0.21, 0.27 m/s` in the first six loops.

#### 7.13.5 Combining with the follower

| Follower term (section 8.3) | v1 treatment | Breaks the jerk limit? |
|---|---|---|
| vBack(s_v) braking envelope | replaced: the law's constraints use the same vB, vCap, aD | no |
| speed-up ramp `v_prev + aAccEff·dt` | replaced by "up" (Amax = aAccEff), on lines only | no |
| goal law vsqrt(d_go) in TRACK | **removed from TRACK**; the law's last constraint is the hand-over point (s_goal - d_h, v_h = k_p·d_h). vsqrt would cut the jerk-limited speed instantly near the goal | n/a |
| CAPTURE 2D law `V = k_p·(P - p)` | entered at d_go <= d_h (not d_lin). At entry `abs(V) = k_p·d_h = v_h`, so the speed is continuous; the deceleration steps from 0 to `k_p^2·d_h` | one step of `k_p^2·d_h/dt`, which is <= jerkMax with `d_h = min(D_H_MAX, jerkMax·dt/k_p^2)`: jerkMax = 50 gives d_h = 0.05 m, step 22.5 m/s^3; jerkMax = 10 gives d_h = 0.0222 m, step 10 m/s^3 |
| speed budget `sqrt(vMax^2 - c^2)` | enters the cap, so normally jerk-limited. If c jumps (vision), v_t is cut **instantly** to the budget and the state is reset: v_cmd = cut value, a_cmd = min(a_cmd, 0) | yes, rarely; it protects the modules, and desaturate would cut anyway |
| lag lead tau_lag | kept, **required** (7.13.2) | no |
| stop arrival, ROTATE, DWELL | unchanged; restart state (max(0, V_meas·T_out), 0) | no |

Only the speed-budget cut and the speed floor may cut instantly. The lateral term c and the direction change at a G1 arc joint are
not jerk-limited (section 7.13.8).

#### 7.13.6 Offline profileTime

**Recommendation:** profileTime simulates the same law offline with ideal tracking (`s += v·dt`, dt = 0.02 s) and **tau_lag = 0**
(`s_v = s`) from v0 to each goal. The lead compensates the real drivetrain lag, which the offline model does not have; with the lead the
ideal run would brake early for nothing (shared example: 3.68 s with the lead, 3.54 s without (script)). The law
adds the hand-over tail `max(0, ln(d_h/pos_tol)/k_p)` per goal (`ln(0.05/0.03)/3 = 0.170 s` for d_h = 0.05, k_p = 3; 0 if d_h < pos_tol), and adds the
dwells. Cost: a few hundred steps times a handful of constraints, about 10^4 stopDist calls, well under a millisecond in Java
(**Inference**, not measured). It is exactly the runtime law, including the hand-over rule of section 8.3.7e, so the time is what a
perfectly tracking robot would do. A guard ends the offline run after `10·spanTime + 10 s` of simulated time (`spanTime` = the
closed-form lower bound); then profileTime is the guard time and warning C2's channel reports a bug (`profile simulation did not reach
the goal`). It was never reached in the scripts once the hand-over rule was in place. The closed-form span times of section 7.12 are
the no-jerk lower bound.

#### 7.13.7 Default jerk and time cost

New `PathConstraints` field `jerkMax`, validated > 0. **Default 50 m/s^3** (**Recommendation**, placeholder: tune on the robot).
`KinematicsConfig` has no jerk setting, so the value lives in `PathsConfig.maxJerk` (default 50, section 9.6). Justification: at 50 Hz a step of
acceleration A already looks like a jerk of A/dt (150 m/s^3 for 3 m/s^2, 360 for 7.2). For the limit to shape anything, a full
acceleration change should take at least about 5 loops: `jerkMax <= A/(5·dt) = 7.2/0.1 = 72 m/s^3` at DEF. Below that, time grows
quickly. jerkMax = 50 spreads 0 -> 7.2 m/s^2 over 0.144 s (7 loops) and 0 -> 3 m/s^2 over 0.06 s (3 loops). Warning C2 when
`jerkMax > max(aAcc, aDec)/(5·dt_nom)`: the limit is then too high to have an effect at 50 Hz, and the law can reach an arc entry
still braking (section 7.8).

Time to within 0.03 m of the end, including the hand-over tail (ideal tracking, 50 Hz, margin 5 mm; no stop items):

| Path / constraints | No jerk limit | jerkMax = 10 | 30 | **50** | 150 |
|---|---|---|---|---|---|
| Shared example, EX | 3.38 s | 3.76 (+11.2%) | 3.54 (+4.7%) | **3.54 (+4.7%)** | 3.50 (+3.6%) |
| P1.stop(), EX | 4.08 | 4.80 (+17.6%) | 4.40 (+7.8%) | **4.32 (+5.9%)** | 4.28 (+4.9%) |
| Tight arc r = 0.3, EX | 4.46 | 5.12 (+14.8%) | 4.70 (+5.4%) | **4.62 (+3.6%)** | 4.56 (+2.2%) |
| Shared example, DEF + motor (no heading anchors) | 2.42 | 3.30 (+36.4%) | 2.86 (+18.2%) | **2.78 (+14.9%)** | 2.60 (+7.4%) |
| P1.stop(), DEF + motor (no heading anchors) | 2.96 | 4.96 (+67.6%) | 3.84 (+29.7%) | **3.28 (+10.8%)** | 3.08 (+4.1%) |
| Tight arc, DEF + motor | 3.26 | 4.70 (+44.2%) | 3.80 (+16.6%) | **3.58 (+9.8%)** | 3.46 (+6.1%) |

The times have the 0.02 s resolution of the simulation. Even jerkMax = 150 costs a few percent: that is the 50 Hz quantization of the
decision, not the jerk itself. The "no jerk limit" column differs from the closed-form 3.2723 s because it ends at 0.03 m through the
hand-over tail instead of braking to 0 at L. It is a reference only: without a jerk limit the law reaches an arc entry still braking
(section 7.8).

v1 profileTime and timeout (jerkMax = 50, EX unless stated):

| Path | Law (ideal) | + dwells | profileTime | Timeout `1.5·profileTime + 1` |
|---|---|---|---|---|
| Shared example | 3.54 s | - | 3.54 s | **6.31 s** |
| P1.stop() | 4.32 s | - | 4.32 s | **7.48 s** |
| P1.stop(0.5) | 4.32 s | + 0.5 | 4.82 s | 8.23 s |
| P1.stop(1.5) | 4.32 s | + 1.5 | 5.82 s | 9.73 s |
| Tight arc r = 0.3 | 4.62 s | - | 4.62 s | 7.93 s |
| Shared example, DEF + motor (no heading anchors) | 2.78 s | - | 2.78 s | 5.17 s |
| Shared example + P1.withVelocity(1.0) (ARC_VAR arc, at most 1.0 m/s at P1) | 4.06 s | - | 4.06 s | 7.09 s |
| Tight arc + P1.withVelocity(0.5) (ARC_VAR arc) | 5.00 s | - | 5.00 s | 8.50 s |

**Java values.** The table is from the Python simulation. The Java `profileTime` uses the continuous hand-over tail and is about 0.01 s per goal below the Python numbers: shared example 3.53 s (timeout 6.30 s), `P1.stop()` 4.30 s (timeout 7.45 s), `P1.stop()` variants and the other rows lower by the same small amount, `P1.withVelocity(1.0)` 4.05 s (timeout 7.08 s). The 3.54 / 4.32 / 4.06 s values elsewhere in this document carry the same difference. The closed-form peak of PROF-26 is 3.7094 m/s.

#### 7.13.8 Plant simulation

Model: the follower's plant (section 8.15): commands every 20 ms, dead time 0.035 s, first-order velocity lag 0.04 s, skid limiter
8 m/s^2 on the command vector, integration step 1 ms; k_n = 3 (clamp 1 m/s), tau_lag = 0.05, k_p = 3; windowed projection. Finish:
within 0.03 m for 0.1 s; stop arrival 0.03 m and 0.1 m/s. "Overshoot" is the largest `(p - P_goal)·T_in` over the approach; negative
means the robot never passed the goal.

| Path, constraints | jerkMax | Stop: t, error, overshoot | End: t, error, overshoot | Max tangential jerk [m/s^3] | Max vector jerk [m/s^3] |
|---|---|---|---|---|---|
| Shared example, EX | 30 | | 3.48 s, 1.8 cm, -1.8 cm | 30.0 | 161 |
| Shared example, EX | **50** | | **3.42 s, 1.8 cm, -1.8 cm** | 50.0 | 133 |
| P1.stop(), EX | 30 | 2.08 s, 1.8 cm, -1.8 cm | 4.18 s, 1.8 cm, -1.8 cm | 30.0 | 111 |
| P1.stop(), EX | **50** | **2.04 s, 1.4 cm, -1.4 cm** | **4.10 s, 1.4 cm, -1.4 cm** | 50.0 | 135 |
| Tight arc, EX | 30 | | 4.56 s, 1.6 cm, -1.6 cm | 30.0 | 152 |
| Tight arc, EX | **50** | | **4.52 s, 1.5 cm, -1.5 cm** | 50.0 | 127 |
| Shared example, DEF + motor (no heading anchors) | 50 | | 2.56 s, 1.3 cm, -1.3 cm | 50.0 | 317 |
| P1.stop(), DEF + motor (no heading anchors) | 50 | 1.50 s, 0.9 cm, -0.9 cm | 3.00 s, 1.1 cm, -1.1 cm | 50.0 | 179 |
| Tight arc, DEF + motor | 50 | | 3.40 s, 0.7 cm, -0.7 cm | 50.0 | 317 |

- All runs use the law of section 7.13.4 with constant-speed arcs. **No overshoot** of any stop or end in any run; arrival error
  0.7 to 1.8 cm, inside the 3 cm tolerance. The tangential jerk never
  exceeds jerkMax. The plant finishes sooner than the ideal run because the ideal run's tail is a fixed exponential while the plant
  enters the 3 cm zone with some speed left (**Inference**).
- Two fixes came from the simulation and are part of the law: the landing rule, and `d_h <= jerkMax·dt/k_p^2` (without it the
  hand-over step was 22 m/s^3 with jerkMax = 10).
- The **vector** jerk of the command (110 to 320 m/s^3) is **not** limited by this law. Its sources (**Inference**): the G1 curvature
  step at arc joints (a_n jumps by up to aLat, i.e. aLat/dt = 200 to 360 m/s^3), the cross-track correction, the CAPTURE hand-over. The
  limiter limits acceleration, not jerk. Clothoids (v2, section 5.12.3) are the fix for arc joints.
- ARC_VAR arcs, same plant (script): shared example + P1.withVelocity(1.0), EX, jerkMax 30 / 50: end 4.04 / 4.02 s, 1.9 / 2.0 cm;
  tight arc + P1.withVelocity(0.5): 4.94 / 4.86 s, 1.3 / 1.2 cm; shared example + P1.withVelocity(1.0), DEF + motor: 3.26 / 3.04 s;
  (0,0), (3,0).withVelocity(1.0), (3,3).stop(), (0,3), EX: stop 4.08 / 4.00 s, end 6.22 / 6.18 s. No overshoot. Ellipse use of the
  command on arcs (at the law's lag-led s_v) <= 0.944, of the measured acceleration <= 0.86. The measured speed when s* passes the point
  was up to 1.7% above the cap (1.017 m/s for 1.0, 0.505 for 0.5): the lead tau_lag = 0.05 s does not cancel the whole modelled lag
  (**Inference**). The tangential jerk stayed at jerkMax except in one run (DEF + motor, jerkMax 30): one step of 170 m/s^3
  from the speed floor of section 7.13.4 at the hand-over point (v_cmd = 0.057 m/s, a_cmd = -3.4 m/s^2, s_v held at s_goal - d_h); the
  same path without `withVelocity` does not show it (**Uncertain**: a goal-approach case of the speed floor, not of ARC_VAR).
- These are model results, not robot results (**Uncertain** how well the toy plant matches the robot).

#### 7.13.9 Pseudocode (law only)

```
step(v, a, s, cap):                              # once per loop, s = s_v (lag-led, clamped at s_goal - d_h)
  cons = [(s_j, vB[j], D_j, isArcEntry(j)) for junctions ahead] + [(s_goal - d_h, k_p*d_h, D_goal, true)]
  onArc = (span at s is a constant-speed arc);  aD_cur = aDec if onArc else aD[span at s]       # line or ARC_VAR: its own aD
  failed = {}
  cands = ([] if onArc else [min(a + jerkMax*dt, Amax(v, s))]) + [a - sign(a)*min(abs(a), jerkMax*dt), a]   # no "up" on a constant arc
  for a1 in cands:
      a1 = max(a1, -aD_cur)                                                     # traction clamp (may exceed jerkMax for one loop)
      v1 = v + (a + a1)/2*dt;  if v1 < 0: continue
      if a1 is not "up" and a1 < -sqrt(2*jerkMax*v1): continue                    # landing rule
      s1 = s + (v + v1)/2*dt
      if safe(v1, a1, s1, cons, cap, failed): return (v1, a1)
  D_brake = max(Dj for c in failed) if failed else cons[0].Dj                    # binding constraints, else the nearest
  D_brake = min(D_brake, aD_cur)                                                # safety clamp
  a1 = min(a + jerkMax*dt, max(a - jerkMax*dt, -D_brake, -sqrt(2*jerkMax*v)))    # brake
  v1 = v + (a + a1)/2*dt;  if v1 <= 0: (v1, a1) = (0, max(a1, 0))
  return (v1, a1)

safe(v1, a1, s1, cons, cap, failed):
  ok = true
  if v1 > cap and a1 >= 0: ok = false
  if a1 > 0 and v1 + a1^2/(2*jerkMax) > cap: ok = false
  for c = (sj, vt, Dj, needA0) in cons:                    # needA0: arc entry or hand-over point
      if v1 <= vt and a1 <= 0: continue
      if not needA0 and v1 + max(0, a1)^2/(2*jerkMax) <= vt: continue             # line-to-line junction: speed only
      if stopDist(v1, a1, vt, jerkMax, Dj) + STOP_DIST_MARGIN > sj - s1: ok = false; failed.add(c)
  return ok
```

#### 7.13.10 Lookup and feed-forward acceleration

Lookup of the profile, used for the law's caps, for logging, and in the jerkMax-free limit:
1. Find span k with `sStart[k] <= s < sStart[k] + l[k]` (binary search on sStart, or a cursor from the last span). Clamp s to the current
   sub-profile.
2. `q = s - sStart[k]`.
3. `vBack(q) = min(vCap[k], sqrt(vB[k+1]^2 + 2·aD[k]·(l[k] - q)))`: one sqrt, no table interpolation.
4. `vProf(q)` (logging) `= min(vCap[k], F_k(vF[k], q), sqrt(vF[k+1]^2 + 2·aD[k]·(l[k] - q)))`.

Example: s = 1.0 on the shared example: k = 0, q = 1.0, `vBack = min(4, sqrt(5.6569 + 6·0.5307)) = 2.9734 m/s`.

Feed-forward acceleration of the profile: `a(s) = v·dv/ds` is `+a(v)` in speed-up, 0 in cruise, `-aD` in slow-down (span 0 at s = 1.0:
+3.0 m/s^2). The law's own a_cmd is the acceleration actually commanded. Neither is needed for the drive motors: the follower outputs
velocities, and `DemaciaKinematics` already makes the drive acceleration feed-forward from the change in commanded module velocity
(**Fact** DemaciaKinematics.java:214-216, :295-296). They are logged (`aCmd`) and used in tests.

### 7.14 Interaction with SwerveAccelerationLimiter

What the limiter does (**Fact**, SwerveAccelerationLimiter.java): dt capped at `maxDt` (:75); it limits the change of the
**field-relative translation** per loop: skid disk `maxSkidAccel·dt` (:106), forward limit
`maxForwardAccel·(1 - speed/freeSpeed)·dt` (:100, :107, braking and turning free, :94), tilt box (:103, :109); omega passed
through unchanged (:80); an allowed change is returned unchanged (:116-126), so a command inside the envelope passes with
**no lag**; it ramps from the speeds really sent, supplied by the caller; it limits acceleration only, not jerk.

| Profile quantity | Limiter limit | Condition (margin 0.9) |
|---|---|---|
| a_n on arcs (a_t = 0 there, section 7.8; on ARC_VAR spans inside the grip share), a_t on lines | skid disk maxSkidAccel | aAcc, aDec, aLat <= 0.9·maxSkidAccel |
| speed-up a_t | forward: `maxForwardAccel·(1 - v/freeSpeed)` | the speed-up phase follows 0.9 of that curve |
| any acceleration, any direction in the robot frame | tilt box | <= 0.9·min(tiltFront, tiltSide); a box contains its inscribed disk |
| omega, alpha | none (passed through) | the follower limits alpha itself (section 8.5.2) |

With DEF all hold: 7.2 <= 7.2 (skid 8), 7.2 <= 10.8 (tilt 12), and the speed-up follows the motor curve.

**Inference: the profile has the limiter's own shape** (skid-limited changes, motor-curve speed-up) at 90%, so the two
never disagree. **Recommendation: keep the limiter.** In normal driving it does nothing; it still catches jumps in the
cross-track term when a vision update moves the pose, the start transient (a sideways or fast start), and s* jumping
forward. Lag when it engages: a velocity step dv takes `dv/maxSkidAccel`; a 0.5 m/s step takes 0.0625 s (about 3 loops)
with about `dv^2/(2·a) = 0.016 m` of drift. At the start, misaligned modules hold the sent speed back (cosine scaling,
DemaciaKinematics.java:198), so the start is delayed by the module steer time, at most `90 deg/12 rad/s = 0.13 s`; braking
by position keeps this safe.

Required by the drivetrain (`docs/Kinematics-design.md`, "Adding the acceleration limit to your chassis"): re-sync the
limiter's last speeds to the measured speeds when the path command starts, desaturate before the limiter, call it once per
loop. `ExampleSwerveWithPose` already re-syncs in `periodic()` when the last loop did not use `driveFieldRelative`
(:333-339).

### 7.15 Speed-profile method comparison

All methods in the s-domain. Example times are for the shared example with EX unless stated.

#### 7.15.1 Trapezoid (constant acceleration), grid passes

`v_{i+1}^2 = v_i^2 + 2·a·ds_i` on samples every 0.02 m, backward with aDec and forward with aAcc, with the exact ellipse and motor
curve at each sample. **Inference**: for a point-mass model with these bounds, forward-backward passes are time-optimal, so this is
the reference for the other methods. Example: **3.2723 s**; DEF + motor curve 2.3101 s (no heading anchors). It is the test oracle
only, not production code; the equations and its use in tests are in Appendix B.

#### 7.15.2 Trapezoid per span (chosen)

The method of sections 7.3 to 7.10: in span k `v(q) = min(vCap_k, F_k(v_in, q), sqrt(v_out^2 + 2·aD·(l - q)))`, junction passes,
stops as J = 0 with sub-profiles.
- Caps and stops: exact when every arc is driven at its curvature cap (the common case); conservative when a point cap on an arc
  binds: +2.5% and +5.5% for `withVelocity` (the ARC_VAR share), +24.6% for a heading-kink cap, which slows the whole arc (section 7.8);
  each leg of a user stop is
  one span from 0 to 0; face-target needs 0.1 m sub-spans.
- Cost: O(spans); lookup is one sqrt (plus a bisection in a motor-curve speed-up).
- Matches the drivetrain: yes, with the motor curve in closed form. A pure constant-acceleration version is either outside
  the limiter or 22 to 33% slow at DEF.
- Example: **3.2723 s** (equal to the grid oracle); DEF + motor 2.3101 s (equal; no heading anchors); P1.stop() 3.8839 s motion.
- Easy to explain and log ("span 0: up to 2.72 m/s, down to 2.38"); phase and time are exact numbers.

#### 7.15.3 Jerk-limited S-curve

Jerk `j = da/dt = (da/ds)·v`, so a jerk limit is `abs(da/ds) <= jerkMax/v`: it couples a and v along s, so it is not a pointwise
cap. Closed forms for a velocity change from v1 to v2 with acceleration limit A, symmetric jerk profile:

```
dv = abs(v2 - v1)
if dv >= A^2/jerkMax:  t = dv/A + A/jerkMax,       d = (v1 + v2)/2 * (dv/A + A/jerkMax)
else:               t = 2*sqrt(dv/jerkMax),     d = (v1 + v2)*sqrt(dv/jerkMax)
```

Example: 0 -> 2 m/s with A = 3, jerkMax = 10: `A^2/jerkMax = 0.9 <= 2`, t = 0.667 + 0.3 = 0.967 s, d = 0.967 m; the trapezoid takes
0.667 s and 0.667 m.
- Per span (workable): a 7-phase S-curve per span with a = 0 at junctions; the passes solve "largest v_in with
  d(v_in -> v_out) <= l" by bisection. Drawback: a = 0 is forced at every junction.
- Grid third pass on a(s): re-integrating raises v above the envelope; clipping re-adds steps; no clean stopping rule. Not
  recommended.
- Runtime jerk-limited tracking of a trapezoid table (a filter after the profile): tried, fails. In a 50 Hz simulation (script) the robot passed the end
  and stopped at s = 6.27 m (jerkMax = 30) and 6.38 m (jerkMax = 10), against L = 5.98: braking needs the state (v, a). **Rejected as a filter**; root cause in section 7.13.1.
- Online decision with (v, a) in the stopping distance (brief decision 18, chosen for v1): section 7.13. In the plant simulation it
  never passed a stop or the end, and the tangential jerk stayed <= jerkMax.
- At 50 Hz a step from 0 to 3 m/s^2 already spreads over one loop (150 m/s^3), so a jerk limit matters only below about
  50 m/s^3.

| jerkMax [m/s^3] | Total (per-span S-curve, script) | vs trapezoid |
|---|---|---|
| 10 | 3.6025 s | +10.1% |
| 30 | 3.3881 s | +3.5% |
| 150 (= a/dt at 50 Hz) | 3.2950 s | +0.7% |

Straight 2.8284 m rest to rest: trapezoid 1.9420 s, S-curve jerkMax = 10: 2.2650 s (+17%), jerkMax = 30: 2.0445 s (+5%). The limiter has
only acceleration limits (**Fact** SwerveAccelerationLimiter.java:105-110), so nothing downstream needs jerk limiting.
**Uncertain** whether it lowers path error on this robot; the drive acceleration feed-forward already removes most of the
velocity-loop lag at steps (docs/Kinematics-design.md simulation: 14.8 cm with the feed-forward vs 23.8 cm without).

#### 7.15.4 Exponential (motor-curve) acceleration

> Removed in v1 (simplification); kept as v2 reference.

`v·dv/ds = a_0·(1 - v/v_free)` (section 7.9). a continuous during speed-up. Best match to the limiter's forward limit and DC-motor
physics, but braking is not motor-limited and there is no traction cap: pure motor curve (a_0 = 9) gives 2.2961 s on the shared
example with DEF (no heading anchors) but a_t reaches 8.49 m/s^2 at low speed, above aAcc = 7.2. Invalid alone; combined `min(aAcc, a_0·(1 - v/v_free))`:
2.3101 s.

#### 7.15.5 Others

- sqrt ramp `v = sqrt(2·a·s)`: the trapezoid in the s-domain; the 2025 bug was the wrong s, not the shape.
- Cosine (sinusoidal-acceleration) ramp: a velocity change takes pi/2 as long for the same peak; rest to rest over d:
  `T1 = sqrt(pi·d/(2A))`; d = 2.8284, A = 3: T1 = 1.2170, total 2.434 s against 1.942 s (+25%); no closed v(s).
- Optimization (TOPP, nonlinear programs): needs a solver; overkill for line/arc paths. Not for v1 or v2.

#### 7.15.6 Summary

| Method | a continuous? | Caps / stops | Build cost | Matches limiter | Shared example | DEF + motor (no heading anchors) |
|---|---|---|---|---|---|---|
| 1 Grid trapezoid + ellipse + motor | no | exact / sample v = 0 | ~1000 samples | yes | 3.2723 s | 2.3101 s |
| **2 Span closed form + motor ramp + constant-speed arcs (ARC_VAR at a `withVelocity` point)** | no | exact when every arc is at its curvature cap, +2.5..5.5% with `withVelocity` on an arc, +24.6% with a heading-kink cap / junction 0 | O(spans) | yes | **3.2723 s** | **2.3101 s** |
| 2' Span, pure constant-acceleration trapezoid | no | same | O(spans) | no: outside, or slow if folded | 3.2723 s | 2.2573 s (outside) or 2.8188 s (folded) |
| 3 Per-span S-curve | yes | jerk-aware junction distances / junction 0 | O(spans) + bisection | no extra benefit | 3.3881 s (j=30), 3.6025 s (j=10) | not computed |
| 4 Motor curve alone | in speed-up | needs a braking model and traction cap | O(spans) | best shape, invalid alone | n/a | 2.2961 s (a_t 8.49 > 7.2) |
| 5 Cosine ramp | yes | numerical v(s) | grid | no | +25% on a rest-to-rest leg | not computed |

#### 7.15.7 Decision

**v1 (with brief decision 18): method 2 for the limits, plus the jerk-aware online law of section 7.13 as the runtime command**
(default jerkMax = 50 m/s^3; cost +4 to 6% time at EX and +10 to 15% at DEF).

**v1: method 2**, with its refinements: junction speeds from passes across spans; speed-up along
`min(aAcc, 0.9·motor curve)` in closed form; constant speed on every arc (simplification pass) except the ARC_VAR arc of a
`withVelocity` point (user decision); stops as junction 0 with
sub-profiles that restart with normal caps; runtime command = the jerk-aware law (vBack plus the time ramp is its unlimited-jerk
limit). Reasons: it gives exactly the grid's time whenever every arc is driven at its curvature cap; it is never faster than the
grid (always traction-safe); a `withVelocity` on an arc costs +2.5 to 5.5% (ARC_VAR) and a heading-kink cap, which slows the whole
arc, +24.6% in the cases tried (section 7.8); it stays
inside the limiter by construction; it is small, exact and easy to debug. **The grid method stays as the test oracle**
(Appendix B: span time >= grid time, equal within 1e-3 s when every arc is driven at its curvature cap). **v2**: speed zones, non-zero end speed, clothoid sub-spans (the lateral jerk at arc joints; the tangential jerk is
already limited in v1).

### 7.16 Full worked example

Shared example: r = 1.4142, C = (3, 2.5858), sigma = -1; lines 1.5307 m, arc 2.9155 m (sweep -118.12 deg), L = 5.9769 m,
s(P1) = 2.9885. Constraints EX, start at rest, heading of section 6.11. With alphaMax = 4·pi, the wheel cap (R_mod = 0.3889)
and the DEF motor curve added, nothing changes: kink cap 2.818, wheel cap 3.363 and motor knee 3.53 are all above the speeds
reached.

| k | Span | l | Kind | vCap | aA, aD | theta' |
|---|---|---|---|---|---|---|
| 0 | line 0 | 1.5307 | LINE | 4 | 3, 3 | 0.5256 (rotation cap 11.95) |
| 1 | arc to P1 | 1.4578 | ARC | 2.3784 | 0, 0 | 0.5256 |
| 2 | arc from P1 | 1.4577 | ARC | 2.3784 | 0, 0 | 0 |
| 3 | line 2 | 1.5307 | LINE | 4 | 3, 3 | 0 |

Junctions: vB = [3.8524, 2.3784, 2.3784, 2.3784, 0], vF = [0, 2.3784, 2.3784, 2.3784, 0]. The arc speed is the curvature cap 2.3784.

| Phase | s range [m] | Speed [m/s] | Distance [m] | Time [s] |
|---|---|---|---|---|
| Speed up at 3 from rest | 0 -> 1.2368 | 0 -> 2.7241 | 1.2368 | 0.9080 |
| Brake at 3 for the arc | 1.2368 -> 1.5307 | 2.7241 -> 2.3784 | 0.2939 | 0.1152 |
| Arc at cap | 1.5307 -> 4.4462 | 2.3784 | 2.9155 | 1.2258 |
| Speed up at 3 | 4.4462 -> 4.7401 | 2.3784 -> 2.7241 | 0.2939 | 0.1152 |
| Brake at 3 to stop | 4.7401 -> 5.9769 | 2.7241 -> 0 | 1.2368 | 0.9080 |
| **Total** | | | 5.9769 | **3.2723 s** |

- Peak: `v^2 = (2·3·1.5307 + 2.3784^2)/2 = 7.4205`, v = 2.7241 m/s. This is the trapezoid without a jerk limit; the v1 law peaks
  at 2.58 m/s, drives the arc at 2.360 m/s and takes 3.54 s to the 0.03 m tolerance (timeout 6.31 s, section 7.13.7).
- Max abs(omega) = `0.5256·2.7241 = 1.43 rad/s`. Max a_n = 4.000 m/s^2 on the arc.
- Limiter check: 4.0 m/s^2 on the arc against skid 8; speed-up 3 m/s^2 at 2.72 m/s against the motor curve
  `10·(1 - 2.72/5.3) = 4.87`: inside.
- Grid oracle (Appendix B): the same 3.2723 s; arc start 1.0233 s, P1 1.6362 s, arc end 2.2491 s.

Runtime samples:

| s* | Span, phase | vBack | vProf |
|---|---|---|---|
| 0.0 | 0, speed-up | 3.8524 | 0 (the speed law starts the robot) |
| 1.0 | 0, speed-up | 2.9734 | 2.4495 |
| 1.4 | 0, slow-down | `min(4, sqrt(5.6569 + 6·0.1307)) = 2.5379` | 2.5379 |
| 3.0 | 2, cruise | 2.3784 | 2.3784 |
| 5.5 | 3, slow-down | `sqrt(6·0.4769) = 1.6916` | 1.6916 |

Variants, trapezoid profile without a jerk limit (script); these are the limits and lower bounds the law works with. The v1
profileTime and timeout of the old jerk law (v2 reference) are in section 7.13.7 where computed:

| Variant | Key numbers | Trapezoid time | v1 profileTime / timeout |
|---|---|---|---|
| Main | arc 2.3784, peak 2.7241 | 3.2723 s | 3.54 s / 6.31 s |
| P1.stop() | two 2.8284 m lines, peaks 2.9130 at s = 1.4142 and 4.2426 | 3.8839 s | 4.32 s / 7.48 s |
| P1.stop(0.5) | + 0.5 dwell | 4.3839 s | 4.82 s / 8.23 s |
| P1.withVelocity(1.0) | ARC_VAR arc (cap 2.1273, aA = aD = 1.8); junctions [0, 2.1273, 1.0, 2.1273, 0]; braking for the arc from s = 1.1425, for P1 from s = 2.0092 | 3.7751 s (grid oracle 3.5770 s) | 4.06 s / 7.09 s |
| P1 at 180 deg (heading-kink cap) | whole arc at 1.4088 | 4.2812 s (grid oracle 3.4372 s) | 4.54 s / 7.81 s |
| Measured start v0 = 1.5 | peak 2.9188 | 2.9051 s | not computed |
| DEF + motor curve, no heading anchors | arc 3.1910, peak 3.7094 | 2.3101 s | 2.78 s / 5.17 s |
| DEF + motor curve, with the P1 90 deg anchor | kink cap 2.8176 < arc cap 3.191: whole arc at 2.8176, junctions [0, 2.8176, 2.8176, 2.8176, 0] | 2.4549 s | 2.88 s / 5.32 s |
| Tight arc (0,0), (4,0).withRadius(0.3), (4,4) | arc 1.0954 | 4.3210 s | 4.62 s / 7.93 s |
| Tight arc + P1.withVelocity(0.5) | ARC_VAR arc (cap 0.9798, aA = aD = 1.8); junctions [0, 0.9798, 0.5, 0.9798, 0] | 4.5577 s (grid oracle 4.4469 s) | 5.00 s / 8.50 s |

### 7.17 Pseudocode

v1 (the code); the span version it replaced is described in sections 7.3 to 7.12.

```
buildProfile(path(flipped), c: PathConstraints, heading, v0, k_p, pos_tol):
  limits = []
  for each ARC entry g:                      limits += (g.sStart, g.sEnd, vCap = vGrip = sqrt(c.aLat * g.r))
  for each inner point i with withVelocity v: if s_i is inside an arc: that arc's vCap = min(vCap, v)   # whole arc
                                              else limits += (s_i, s_i, v)                              # point limit
  for each heading piece [s_a, s_b] with theta' != 0: limits += (s_a, s_b, c.omegaMax / abs(theta'), rate = c.alphaMax / abs(theta'))
  goals = stops inside (0, L) in order, then L
  profileTime = startDwell; s = 0; v = max(0, v0)                          # simulate the law, ideal tracking
  for each goal: while goal - s > d_h: v1 = speed(v, s, s, goal, inf, 0.02); s += (v + v1)/2 * 0.02; v = v1; t += 0.02
                 profileTime += max(0, ln(d_h/pos_tol)/k_p) + dwell(goal); s = goal; v = 0
  # throws after 10*(L/min(vMax, smallest vCap) + sum dwells) + 10 s

ceiling(s*, s_v, s_goal, dt):                # D = c.aDec; disc(vE, d) = sqrt(vE^2 + (d*dt)^2) - d*dt
  v = min(c.vMax, disc(sqrt(v_h^2 + 2*D*dt*v_h + 2*D*max(0, s_goal - d_h - s_v))))     # hand-over, lands on v_h = k_p*d_h
  for each limit: if sStart <= s_v and sEnd >= s*:   v = min(v, vCap)                  # overlaps [s*, s_v]
                  else if s_v < sStart < s_goal:      d = min(D, rate); v = min(v, disc(sqrt(vCap^2 + 2*d*(sStart - s_v)), d))
  return v

speed(v_prev, s*, s_v, s_goal, cap, dt):      # each loop; cap = sqrt(vMax^2 - c^2), 8.3.9; s_v = min(s* + v_prev*tau_lag, s_goal - d_h)
  vp = max(0, v_prev)
  up = c.aAcc; for each limit overlapping [s*, s_v]: up = min(up, rate, c.aAcc*sqrt(max(0, 1 - (vp^2/vGrip^2)^2)))
  v = min(cap, ceiling(s*, s_v, s_goal, dt), vp + up*dt)
  if v > vp: v = max(vp, min(v, ceiling(s*, s_v + tau_lag*(v - vp), s_goal, dt)))   # the lead moves tau_lag*dv more next loop
  return max(0, v)
```

Tests: section 13.3.

## 8. Follower

This chapter specifies the command that drives the robot along a `Path`. Everything is a function of the projected
progress s*, never of time. The only time-based quantities are the speed law's own state (it integrates its commands), the stop
dwell, the finish settle timer and the timeout.

### 8.1 Inputs from the other chapters

The follower uses only these functions of the active (already flipped) path and its profiles:

| Call | Returns | Used for |
|---|---|---|
| `L` | total length [m] | goal, events, clamps |
| `x(s)`, `T(s)`, `N(s)`, `kappa(s)` | point, tangent, left normal, curvature (s clamped to [0, L]); `evaluateLeft` for left limits | translation command |
| `project(p, sMin, sMax, s_prev)` | `(s*, e_n)` (section 5.10.4) | progress |
| `speed(v_cmd, s*, s_v, s_goal, cap, dt)` | the plain trapezoid speed law of the current sub-profile (sections 7.6.5, 8.3.7): next v_cmd | speed command (8.3.7) |
| `vBack(s)`, `vCap` | braking envelope and span cap (section 7.1); the law's limits | cap, telemetry |
| `aAccEff(v, s)` | effective speed-up limit (section 7.6.5), the law's Amax | CAPTURE speed-up limit (8.3.7e) |
| `vProf(s)` | forward-pass profile | logging only |
| `theta(s)`, `theta'(s)` | heading profile, continuous and unwrapped (section 6) | heading |
| `profileTime` | from 0 to L plus dwells (section 7.10.5) | timeout |
| `segmentIndexAt(s)`, segment end, type and radius | segment lookups (section 5.9.3) | window cap, telemetry |
| `stopPoints()` | sorted s_c of every automatic STOP corner and user stop, each with P_c, T_in, T_out, dwell | stop state machine (8.6) |
| `events()` | (s_event, action), already converted (section 5.9.5) | events (8.8) |
| `path.flipped(config)` | the path flipped for red with the flip mode and field size of the `PathFollowerConfig`, memoized (section 5.13) | alliance (8.10) |

Derived here: `e_t = (p - x(s*))·T(s*)` (0 unless the projection is clamped at a window edge or at the goal).

**Inference:** the last segment is always a line, and every stop point is a zero-length entry with lines on both sides
(automatic stops and user stops alike, section 5.8.7). So the segment into a stop or the end is a line that ends
exactly at P_c. Section 8.3.7 relies on this. At a STOP, the geometry's T and N are right limits (the out-leg); the
follower uses left limits (`evaluateLeft`, section 5.9.3) whenever s* = s_goal, so e_t, e_n and d_go at a stop refer
to the leg the robot arrives on.

Alignment with the speed profile: the profile is built once in `initialize()` from v0 and theta0, which the
follower passes (section 8.14). Stop points split it into sub-profiles; the follower drives one at a time and moves
to the next on arrival (section 8.6). The measured field velocity comes from a second supplier,
`Supplier<ChassisSpeeds>` (section 8.11).

The tunable follower settings are fields of `PathFollowerConfig` (section 9.6); the other follower constants are internal.
Values and single homes are in section 10.
Worked numbers use EX (section 3) and jerkMax = 50 m/s^3; d_h = 0.05 m with both EX and DEF (it depends on jerkMax and k_p only),
and the heading linear zone is `4·pi/4^2 = 0.785 rad` with both.

### 8.2 Loop structure

#### 8.2.1 Order of one execute() (every 20 ms)

```
1. t = Timer.getFPGATimestamp(); dt = t - t_prev; t_prev = t              (8.2.2)
2. pose = poseSupplier.get(); V_meas = speedSupplier.get(); validate        (8.12)
3. project with the window -> s*, e_n; e_t                                 (8.2.3, 8.2.4)
4. fire events with s_event <= s*                                          (8.8)
5. state machine: TRACK / CAPTURE_STOP / ROTATE / DWELL / CAPTURE_END     (8.6, 8.7)
6. translation command V (field frame)                                     (8.3, 8.4)
7. heading command omega                                                   (8.5)
8. output.accept(new ChassisSpeeds(V.x, V.y, omega))                       (8.11)
9. telemetry                                                               (8.13)
```

The follower does no discretization and no acceleration limiting of translation: `DemaciaKinematics` discretizes and
the drivetrain's `SwerveAccelerationLimiter` limits (section 8.11).

#### 8.2.2 Measured dt

```
dt = clamp(t - t_prev, 0.005, 0.10)      // first loop: dt = dt_nom
```

dt is used for the window (8.2.3), the speed law (8.3.7), the omega rate limit (8.5.2), the settle timer (8.7) and the
monitors (8.12). The control laws output velocities, so no gain depends on dt. Example: loops at 0.020, 0.020, 0.061 s;
the third gives dt = 0.061, and the window uses it. A dt above 0.10 s (breakpoint, long GC) is clamped so the window cannot
grow without bound.

#### 8.2.3 Projection window

This is the final projection design (resolution R9); the geometry only projects inside it.

```
v_hat   = max(v_t_prev, abs(V_meas))                  // last commanded tangential speed, measured field speed
ahead   = clamp(k_ahead * v_hat * dt, ahead_min, ahead_max)
sMin    = max(s_start_sub, s_hi - eps_back)            // s_hi = highest accepted s* (8.2.4)
sMax    = min(s_prev + ahead, s_end(k_next), s_goal)
          k_cur = segmentIndexAt(s_prev); k_next = the first positive-length segment after k_cur (STOP skipped),
          or k_cur if it is the last segment; s_end(k) = s_start,k + l_k
          and, for every arc j with s-range meeting [s_prev, sMax]:  sMax = min(sMax, max(s_start_j, s_prev) + max(0.05, (pi/2)*r_j))
(s*, e_n) = project(p, sMin, sMax, s_prev)
e_t     = (p - x(s*))·T(s*)
```

- Capping at the end of the segment after the current one (k_next) means one loop never jumps over a whole segment. Zero-length STOP
  entries are skipped when finding "the next segment".
- `s_goal` = the next stop point s_c, or L (section 8.6). The window never passes a point where the robot must stop.
- Arc cap. An arc can sweep more than 180 deg and then its in-line and out-line can cross (section 5.6.1). Inside 90 deg
  of sweep the closest point to any robot not at the center is unique, and a crossing needs more than 180 deg of turning,
  so one window cannot reach the far branch. **Inference:** the cap never limits real travel: on an arc
  `v <= sqrt(aLat·r)`, so one loop moves `0.02·sqrt(4·r)` m, 0.009 m for r = 0.05 m, against a cap of 0.079 m.
- Why `k_ahead = 3`: three loops of travel covers measured speed spikes, one overrun loop and a small along-track pose
  correction. `ahead_min = 0.15 m` covers a typical vision correction at low speed. `ahead_max = 0.5 m` stops a wrong
  branch being reached on a tight self-crossing.

Worked example (vMax 4):
- v_hat = 2.0 m/s, dt = 0.02: `3·2.0·0.02 = 0.12`, so ahead = 0.15 m (the minimum).
- v_hat = 4.0 m/s: ahead = 0.24 m. With a 0.06 s overrun: 0.72, capped at 0.50 m.
- Arc r = 0.3 m starting 0.05 m ahead of s_prev: cap = `s_prev + 0.05 + max(0.05, 0.471) = s_prev + 0.521`, above
  `s_prev + 0.24`, so the speed term is the limit.
- Teardrop of section 5.10.4: s_hi = s_prev = 2.0, ahead 0.15: window [1.97, 2.15], s* = 1.9723.

Edge cases: s_prev = L gives sMin = sMax = L. sMin > sMax cannot happen (sMin <= s_prev <= sMax). A pose jump of more than
`ahead` along the path clamps the projection at sMax with e_t > 0; it catches up by `ahead` per loop.

#### 8.2.4 Monotonic progress rule

```
s_hi = max(s_hi, s*)            // after each projection
```

The window's lower edge `s_hi - eps_back` enforces the rule: s* can never be more than eps_back = 0.03 m below the
highest progress ever accepted. Why:
1. The profile is a function of s. Letting s* go back moves the reference back too; pose noise around a speed change
   would make the command chatter.
2. On a hairpin or self-crossing path, a free projection can snap back to an earlier branch that is closer in space.
3. Events must fire once (section 8.8 also latches each event).
4. 0.03 m and not 0: pose noise of 1 to 2 cm across a segment boundary would otherwise pin s* a few cm ahead of the robot.
   0.03 m equals the finish tolerance.

When the robot is really pushed back by more than 0.03 m: s* stays at `s_hi - 0.03` and `e_t < 0`. The robot drives along
T(s*) and catches up. Near a stop, d_go uses e_t (8.3.7), so the remaining distance stays correct. Example: s_hi = 4.20, the
robot is pushed back and projects at 4.05; the window floor is 4.17, so s* = 4.17 and e_t ~= -0.12 m.

#### 8.2.5 Start check (in initialize, before the profile is built)

```
d0 = abs(p - P0)                        // P0 of the active (flipped) path
if d0 > startTol (0.5 m): end, result FAILED_START_TOO_FAR, alert A1
s_hi = s_prev = 0
```

**Decision** (user, simplification pass): there is no mid-path start. The robot must start within startTol = 0.5 m of P0;
otherwise the command ends at once with `FAILED_START_TOO_FAR` and alert A1 (section 9.12). A wrong flip, a wrong path or a start
pose that was not set is far likelier than a real start in the middle of a path. There is no global search, no tie-break and no
skipping of events: the profile, the heading (section 6.8), profileTime and the events all start at s = 0.
- Start a little behind or beside P0 (within 0.5 m): no special case. s* stays at 0 while the speed law speeds the robot up along
  T(0); d_go includes -e_t, so the hand-over sees the true distance, and the correction clamp joins the path at <= c_max laterally.
  Simulated 0.5 m behind P0: done at 4.10 s, against 3.94 s from P0 (section 8.15). The extra distance is not in profileTime; the
  timeout margin covers it.
- Start a little ahead of P0 (within 0.5 m): the window (section 8.2.3) catches up by `ahead` (at least 0.15 m) per loop.
- A path that returns to P0 (a loop): s* starts at 0, so the robot does not finish at once.
- Start state of the speed law: `(v_cmd, a_cmd) = (v0, 0)`, `v0 = max(0, V_meas·T(0))` (sections 7.6.4, 7.13.2).

### 8.3 Translation command

#### 8.3.1 Formula (TRACK state)

```
s_L     = min(s* + v_t_prev * tau_lag, s_goal)                    // lookahead point for the direction (8.4)
s_v     = min(s* + v_t_prev * tau_lag, s_goal - d_h)              // lag-led progress for the speed law (8.3.7)
d_go    = (s_goal - s*) - e_t                                     // signed distance to go (8.3.7)
c       = clamp( -k_n * e_n, -c_max, +c_max )                     // 8.3.2, 8.3.8
cap     = min( vCap[span at s_v], sqrt(vMax^2 - c^2) )            // speed budget (8.3.9)
v_cmd = speed(v_cmd, s*, s_v, s_goal, cap, dt)                    // plain online trapezoid (sections 7.6.5, 8.3.7)
v_t     = v_cmd
V       = v_t * T(s_L) + c * N(s*)                                // field frame [m/s]
v_t_prev = v_t
```

The speed law (brief decision 18) replaces the earlier `min(vBack(s_v), v_t_prev + aAccEff·dt, vsqrt(d_go))`: it brakes by position
for every limit ahead (junction speeds vB, the hand-over point before the goal), speeds up along aAccEff, and keeps the tangential
jerk within jerkMax. The speed budget enters as its cap, so a slowdown is jerk-limited too; only a sudden jump of c cuts v_t
instantly to the budget (with the state reset of section 7.13.5).

Sign (stated explicitly): e_n > 0 means the robot is LEFT of the path; `c = -k_n·e_n` is then negative and `c·N` points right,
back toward the path. With tau_lag = 0, no clamp active and the law at the profile speed, this is the brief's
`V = v(s*)·T(s*) - k_n·e_n·N(s*)`.

Worked example (shared example, EX, jerkMax = 50, real geometry) (hand, script):
- Robot at `p = x(1.4849) + 0.0707·N = (1.7028, 3.3099)` on line 0: `T = (0.5142, 0.8577)`, `N = (-0.8577, 0.5142)`.
  Projection: `s* = 1.4849 m`, `e_n = +0.0707 m` (left), e_t = 0.
- Law state v_cmd = 2.3604 m/s, a_cmd = 0: the law reached the arc speed with a = 0 at the arc entry (ideal-run value with jerkMax 50,
  section 7.13.4). tau_lag = 0.05: `s_L = s_v = 1.4849 + 2.3604·0.05 = 1.6029`, inside the arc (`s_goal - d_h = 5.9269` does not clamp).
  `c = -3·0.0707 = -0.2121 m/s`; cap = min(2.3784, sqrt(16 - 0.045) = 3.994) = 2.3784.
- Law: s_v is on a constant-speed arc, so "up" is not tried (section 7.13.4). "toward 0" with a = 0 gives a1 = 0 and v1 = 2.3604, below the cap; the
  next junction (P1, vB = 2.3784) is above v1 with a1 = 0, so it is safe: **v_t = 2.3604 m/s**, a_cmd = 0. The arc is driven at this
  speed.
- `T(s_L) = (0.5572, 0.8303)` (rotated 2.9 deg into the right turn).
  `V = 2.3604·(0.5572, 0.8303) - 0.2121·(-0.8577, 0.5142) = (1.4972, 1.8509) m/s`, `abs(V) = 2.3807 m/s`.
- Check: `N·V = -0.3325 = c + v_t·(T(s_L)·N) = -0.2121 - 0.1204`: the cross-track term plus 0.1204 m/s toward the arc's center from the
  lookahead (the curvature feed-forward of section 8.3.4).

#### 8.3.2 Cross-track dynamics on a line

Assume the drivetrain tracks V exactly (dp/dt = V). Differentiate `e_n = (p - x(s*))·N(s*)`:

```
d/dt e_n = (dp/dt - T·ds*/dt)·N + (p - x)·(dN/ds)·ds*/dt
```

`dN/ds = -kappa·T` (Frenet), and `(p - x)·T = e_t = 0` at an interior projection, so `d/dt e_n = (dp/dt)·N` exactly, for lines
and arcs. On a line, `T(s_L) = T(s*)`, so `(dp/dt)·N = c = -k_n·e_n`:

```
e_n(t) = e_n(0) * exp(-k_n * t),   time constant 1/k_n
```

Stable for any k_n > 0 with an ideal drivetrain. Example: e_n(0) = 0.2 m, k_n = 3: after 0.333 s e_n = 0.074 m; after 1 s,
0.010 m. Simulated with the lag model of section 8.3.5: t63 = 0.38 s, below 4 mm after 1.04 s, no overshoot.

#### 8.3.3 Cross-track dynamics on an arc

`d/dt e_n = (dp/dt)·N(s*)` holds on an arc too; the kappa terms cancel because the projection is radial. So with the arc tangent
and an ideal drivetrain, `d/dt e_n = -k_n·e_n` (no kappa·e_n·v term). Check by geometry: left arc (kappa = +1/r): the center is at
`x + N/kappa` and `e_n = r - dist_C`, where dist_C is the distance to the center; `d(dist_C)/dt = u_out·dp/dt` with the outward unit
vector `u_out = -N`, so `d/dt e_n = N·dp/dt`. The same holds for a right arc.

What kappa changes is the progress rate. Differentiate the projection condition `(p - x(s*))·T(s*) = 0`:

```
d/dt s* = (dp/dt·T) / (1 - kappa*e_n)
```

Example: r = 0.5 m (kappa = 2), robot 0.05 m inside (e_n = +0.05), v = 1.4 m/s: `ds*/dt = 1.4/(1 - 0.1) = 1.556 m/s`; s* runs 11% faster
than the robot because the robot is on a tighter circle. Its lateral acceleration is `v^2/(r - e_n) = 4.36 m/s^2` instead of 3.92.

Edge cases: singular at e_n = 1/kappa (the robot at the arc center); the window and the monotonic rule keep s* local, and v there is
<= `sqrt(4·0.05) = 0.45 m/s` for r = 0.05 m. A robot more than r outside the arc is still well defined.

#### 8.3.4 Curvature feed-forward and lag

Swerve modules take velocity setpoints, so the follower never adds a force term like `m·v^2·kappa`; with an ideal velocity
drivetrain, `V = v·T(s*)` already turns the robot because T(s*) rotates as s* advances. The real issue is lag. The actual velocity is
the command delayed by about tau_d: CAN and motor latency (`KinematicsConfig.latency` = 0.035 s, KinematicsConfig.java:30), half a loop,
and the module velocity-loop time constant. On an arc the command direction rotates at `kappa·v`, so the delayed command points along
`T(s* - v·tau_d)`, rotated outward. To first order `T(s - delta) = T(s) - kappa·delta·N(s)`, so

```
d/dt e_n = -k_n*e_n - kappa*v^2*tau_d
e_n,ss   = -kappa * v^2 * tau_d / k_n         (negative for a left turn: the robot runs wide)
```

Example: `kappa·v^2 = aLat = 4 m/s^2`, tau_d = 0.085 s, k_n = 3: `e_n,ss = -4·0.085/3 = -0.113 m`; simulated (radius 3 m at 3.46 m/s): -0.106 m.

The cure is to aim the tangent ahead by the lag, `T(s* + v·tau_lag)` (section 8.4): to first order
`v·T(s* + v·tau_lag) ~= v·T(s*) + kappa·v^2·tau_lag·N(s*)`, a curvature feed-forward of size `aLat·tau_lag` along N that cancels the
lag term when tau_lag = tau_d. Example: `4·0.05 = 0.2 m/s` toward the center; residual `e_n,ss = -4·(0.085 - 0.05)/3 = -0.047 m`
(simulated -0.043 m). With tau_lag = 0.085: simulated +0.0015 m.

**Recommendation:** use the lookahead form (direction from T(s_L)), not an explicit `kappa·v^2·tau` term: on the G1 jump from a line to an
arc, the lookahead starts turning tau_lag before the arc, which a lagging drivetrain needs; the explicit term would jump at the junction.
**Inference:** the limiter adds no lag while not saturated (SwerveAccelerationLimiter.java:116-126); holding an arc needs `aLat·dt = 0.08 m/s`
of change per loop, below `maxSkidAccel·dt = 0.16 m/s`.

#### 8.3.5 Choosing k_n

The bandwidth is set by the lag, not by dt. For `d/dt e = -k·e(t - tau_d)`: no overshoot iff `k·tau_d <= 1/e ~= 0.368`; stable iff
`k·tau_d < pi/2`. With tau_d ~= 0.085 s (0.035 latency + 0.01 half loop + about 0.04 module velocity loop, the last **Uncertain**): no
overshoot for k_n <= 4.3, unstable above about 18.

Simulated line step, 0.2 m start offset (simulation model of section 8.15: vMax 3, aAcc = aDec = aLat = 4):

| k_n [1/s] | 1/k_n [s] | t63 [s] | min e_n (overshoot) [m] | below 4 mm after [s] |
|---|---|---|---|---|
| 2 | 0.50 | 0.54 | +0.0011 (none) | 1.72 |
| 3 | 0.33 | 0.38 | 0.0000 (none) | 1.04 |
| 4 | 0.25 | 0.32 | 0.0000 (none) | 0.70 |
| 5 | 0.20 | 0.30 | -0.0020 | 0.50 |
| 6 | 0.17 | 0.28 | -0.0078 | 0.66 |
| 8 | 0.125 | 0.28 | -0.0207 | 0.62 |
| 15 | 0.067 | 0.28 | -0.0663 | 1.36 |

From k_n = 6 the correction clamp (`6·0.2 = 1.2 > c_max = 1.0`) makes t63 constant. **Recommendation:** k_n = 3.0 1/s (time constant
0.33 s): a 1.4x margin to the no-overshoot bound if the real lag is up to 0.12 s, and a 0.1 m vision jump gives a 0.3 m/s step that
the limiter spreads over 2 loops. Raise to 4 only after measuring tau_d (section 8.13).

#### 8.3.6 Along-track correction: none

A time-parameterized tracker corrects being "behind schedule". This follower has no schedule: the reference is always vBack at the
robot's own progress (the speed law integrates only its own commands). e_t is not an error to drive to zero; it is non-zero only when the
projection is clamped, and it is used in one place, the distance to go (8.3.7).

#### 8.3.7 Speed reference: jerk-aware law, lag lead, hand-over

**Changed in v1 (jerk limit removed):** (a) is the jerk-free form `v_cmd = min(vBack(s_v), sqrt(v_h^2 + 2·D·(s_goal - d_h - s_v)),
sqrt(vMax^2 - c^2), v_cmd_prev + aAccEff·dt)` (state v_cmd only; reset to `max(0, V_meas·T_out)` on release from a stop); the goal is
the envelope that reaches v_h at s_goal - d_h with D the smallest line aD on the way (both braking terms in the discrete form `sqrt(vE^2 + (D·dt)^2) - D·dt`; the hand-over envelope is `vE^2 = v_h^2 + 2·D·dt·v_h + 2·D·(s_goal - d_h - s_v)`, so the law lands exactly on v_h at the hand-over and the CAPTURE step is continuous); on a constant arc the law speeds up at aAcc while below the planned arc speed vF; d_h is a
fixed 0.05 m; the at-rest hand-over band in (e) is `d_h + 0.01 m`. (b), (c) apart from the d_h formula, (d) and (e) are unchanged.
From rest the commands are 0.06, 0.12, 0.18 m/s (EX). The text below is the jerk-law version (v2 reference).

**(a) The speed law** (section 7.13; brief decision 18). The command state is (v_cmd, a_cmd), initialized to (v0, 0) and reset to
`(max(0, V_meas·T_out), 0)` on release from a stop. Each loop the law takes "up", "toward 0", "keep" or "brake", whichever is the first
that can still meet every limit ahead with jerkMax and the per-span deceleration. This removes the start stall: vBack(start) > 0 at
every sub-profile start, and "up" accelerates along aAccEff. Example (EX, jerkMax = 50): from rest the commands are 0.01, 0.04, 0.09,
0.15, 0.21, 0.27 m/s; the limiter passes these (changes <= 0.06 m/s per loop < 0.16). **Inference:** the law is open loop in time (it
integrates its own commands); if the robot is held back, v_cmd still rises while the robot lags, which is safe because every
constraint is in position (s_v). dt is the measured one; the limiter's dt is capped at 0.02 s (SwerveAccelerationLimiter.java:75).
The jerk-free form `min(vBack(s_v), v_t_prev + aAccEff·dt)` is the limit of the law for jerkMax -> infinity; the earlier simulations
of this chapter (section 8.15) used it.

**(b) Lag lead, required.** The law's position input `s_v = min(s* + v_t_prev·tau_lag, s_goal - d_h)` starts braking tau_lag earlier,
compensating the drivetrain lag. It is the s-domain form of `vBack(s) + a·tau`: `vBack(s + v·tau) ~= vBack(s) + (dv/ds)·v·tau`. Example:
braking zone `vBack(s) = sqrt(2·3·(L - s))`, L - s = 1.0 m, v = 2.449 m/s, tau_lag = 0.05: lead 0.122 m, `vBack(s* + 0.122) = 2.295 m/s`
instead of 2.449. **tau_lag > 0 is part of the v1 law:** without it the plant simulation passes the goal by 2.7 to 6.2 cm
(section 7.13.2).

**(c) Goal: the hand-over point.** The signed distance to go along the path is `d_go = (s_goal - s*) - e_t`. On the last line (or the
line into a stop) this equals `(P_goal - p)·T` exactly; it is negative after an overshoot; it is not the straight-line distance (the
2025 bug, DemaciaTrajectory.java:247-249). The goal is the law's last constraint: reach `v_h = k_p·d_h` at `s_goal - d_h`, with

```
d_h = min(D_H_MAX, jerkMax*dt_nom/k_p^2)        (D_H_MAX = 0.05 m;  jerkMax = 50, k_p = 3: d_h = 0.05 m, v_h = 0.15 m/s)
```

The goal law vsqrt(d_go) is **not** a term of the TRACK command any more: it would cut the jerk-limited speed instantly near the goal.
It survives in CAPTURE (e) and as the shape of the heading feedback (section 8.5.2):

```
vsqrt(d; a, k) = sign(d) * ( k*abs(d)                              if abs(d) <= a/k^2
                             sqrt(2*a*(abs(d) - a/(2*k^2)))        otherwise )
d_lin = aDec / k_p^2
```

It is the standard sqrt controller: value, slope and deceleration are continuous at abs(d) = d_lin; outside it the deceleration is exactly
aDec; inside it is `v = k_p·d`. EX: d_lin = 0.333 m, so anywhere inside d_h = 0.05 m it is the linear law `k_p·d` (values (script):
d = 2.0: 3.317 m/s against the profile's 3.464; 1.0: 2.236 / 2.449; 0.5: 1.414 / 1.732; 0.333: 1.000 / 1.414; 0.2: 0.600 / 1.095;
0.03: 0.090 / 0.424).

**(d) Why s_v is clamped at s_goal - d_h.** Without the clamp, the lead would put the law's position past its own hand-over point
and it would brake to v_h too early. With the clamp the law reaches the hand-over point at v_h.

**(e) CAPTURE (sections 8.6 and 8.7).** When `d_go <= d_h`, or when the law has come to rest (`v_cmd = 0`) with
`d_go <= d_h + 2·STOP_DIST_MARGIN`, the state becomes CAPTURE_STOP or CAPTURE_END and the command becomes the 2D position law. The second
condition (review of the simplification pass) ends a deadlock: the law can stop a few mm before the hand-over point, where "up" from rest
needs `STOP_DIST_MARGIN + (13/12)·jerkMax·dt^3` (0.0063 m at jerkMax 150) and less is left, so TRACK would never hand over and the offline
profileTime loop would not end. It was seen once in 500 random chains (jerkMax 150, at rest 5.2 mm before the hand-over); with the rule the
robot hands over at rest and CAPTURE starts it from 0 (speed-up limit `v_lim_prev + aAccEff·dt`). The band covers jerkMax up to
570 m/s^3 (`STOP_DIST_MARGIN/((13/12)·dt^3)`). It acts only at `v_cmd = 0`, so no other run changes: every time in this document is the same (script). The alternative,
counting the hand-over constraint as met within STOP_DIST_MARGIN of it, also ends the deadlock but changes the last loops of normal runs
(shared example 3.54 -> 3.52 s), so it was not chosen.

```
V_raw  = k_p * (P_goal - p)
v_lim  = min( vsqrt(abs(P_goal - p); aDec, k_p),         // goal law (linear zone here)
              v_h,                                       // the hand-over speed of the law
              v_lim_prev + aAccEff*dt )                  // speed-up limit; v_lim_prev = v_t at entry
V      = V_raw * min(1, v_lim / abs(V_raw))
```

At entry `abs(V) = k_p·d_h = v_h`, so the speed is continuous; the deceleration steps from 0 to `k_p^2·d_h = 0.45 m/s^2` in one loop,
22.5 m/s^3 <= jerkMax (that is the reason for the jerkMax term in d_h). **Inference, seamless hand-off:** on the final line
`P_goal - p = d_go·T - e_n·N`, so `V = k_p·d_go·T - k_p·e_n·N`; with k_p = k_n the lateral part equals the TRACK correction while c is not
clamped (keep k_p = k_n; a different value only adds a small step at the hand-off). The reason to switch: s* saturates at s_goal, and after an overshoot the TRACK law cannot push back;
the 2D law can, in any direction. Example: robot at `P_end + (-0.04, +0.01)` in the path frame with T = x: `V = 3·(0.04, -0.01) = (0.12, -0.03) m/s`,
`abs(V) = 0.124 <= v_h = 0.15` (no scaling).

**Goal tail.** Inside d_h the robot follows `v = k_p·d`, which decays exponentially: from d_h into the 0.03 m tolerance takes
`max(0, ln(d_h/pos_tol)/k_p) = ln(0.05/0.03)/3 = 0.170 s` per goal (0 when d_h < 0.03 m, i.e. jerkMax < 13.5 m/s^3) (with the earlier vsqrt hand-over at d_lin it was about 0.47 s). profileTime
includes it (section 7.13.6); the 0.1 s settle is inside the timeout margin.

#### 8.3.8 Correction clamp

```
c = clamp(-k_n*e_n, -c_max, c_max),     c_max = 1.0 m/s
```

- Clamp: a large error (bump, vision correction, start off the path) should not use the whole speed budget sideways. Above
  `abs(e_n) = c_max/k_n = 0.33 m` the approach is a constant 1 m/s instead of exponential.
- Example: e_n = -0.5 m (right of the path), k_n = 3: `-k_n·e_n = +1.5`, so c = +1.0 m/s (left, toward the path).
- Far off the path the robot keeps its profile speed and rejoins at c_max laterally (**Decision**, user, simplification pass); the
  speed budget (section 8.3.9) keeps abs(V) <= vMax.

#### 8.3.9 Keeping abs(V) <= vMax: tangential first

T and N are perpendicular, so `abs(V)^2 = v_t^2 + c^2`. When that exceeds vMax^2, the design keeps c and reduces v_t: the budget
`sqrt(vMax^2 - c^2)` is part of the speed law's cap, so normally the reduction is jerk-limited. If c jumps (a vision correction), v_t is
cut instantly to the budget and the law's state is reset (`v_cmd` = the cut value, `a_cmd = min(a_cmd, 0)`): rare, and it protects the
modules. Getting back on the path has priority over speed, and reducing speed is always safe in an s-parameterized
follower. Because `abs(c) <= c_max = 1.0 < vMax`, `v_t >= sqrt(16 - 1) = 3.873 m/s` at vMax = 4, so progress never stops. Example: v_t = 3.9,
c = -1.0: v_t = 3.873 and abs(V) = 4.000. The drivetrain still desaturates translation and rotation together (DemaciaKinematics.java:260,
:331-345), keeping the direction; compatible.

### 8.4 Lookahead (lag compensation)

```
s_L = min(s* + v_t_prev*tau_lag, s_goal)                      -> direction T(s_L)
s_v = min(s* + v_t_prev*tau_lag, s_goal - d_h)                -> position input of the speed law
N, e_n, d_go: always at s* (never at the lookahead)
```

The error is measured at s* because `d/dt e_n = (dp/dt)·N(s*)` (8.3.2). Use the direction lookahead always on paths with arcs (it removes the
outward offset of 8.3.4: -0.113 m becomes -0.047 m at tau_lag = 0.05, about 0 at tau_lag = tau_d); the speed lookahead with the same tau_lag is
**required** by the v1 speed law (section 8.3.7b). Default tau_lag = 0.05 s, deliberately below the modelled tau_d = 0.085 s: over-compensation cuts the
inside of arcs, which is as dangerous as running wide, and tau_d is **Uncertain** until measured. Tuning rule, from 8.3.4, on a long constant
arc:

```
tau_lag_new = tau_lag - k_n * mean(e_n) / (kappa * v^2)      (signed kappa and e_n, from e_n,ss = -kappa*v^2*(tau_d - tau_lag)/k_n)
magnitude form:  robot runs wide (outside the arc):  tau_lag += k_n*abs(mean e_n) / (abs(kappa)*v^2)
                 robot runs inside the arc:          tau_lag -= k_n*abs(mean e_n) / (abs(kappa)*v^2)
```

Example: left arc `kappa·v^2 = +4`, k_n = 3, mean e_n = -0.043 m at tau_lag 0.05 (negative on a left arc: the robot runs wide):
`tau_lag_new = 0.05 - 3·(-0.043)/4 = 0.05 + 3·0.043/4 = 0.082 s`.
Alternative not chosen (**Inference**): `() -> RobotPose.getInstance().getFuturePose(tau)` as the supplier (RobotPose.java:291) extrapolates the
pose; the stop and finish checks would then use a predicted pose. The s-lookahead keeps the measured pose for every check.
Edge cases: s_L is clamped to s_goal (no lookahead past a stop); after a stop v_t_prev ~= 0, so the lookahead shrinks to 0.

### 8.5 Heading control

#### 8.5.1 Path-heading mode

```
theta_ref = theta(s*)
omega_ff  = theta'(s*) * v_t          // v_t = commanded tangential speed of this loop (after 8.3.8, 8.3.9)
```

Reasons for v_t and no lookahead: section 6.5. Example (section 8.3.1): `theta(1.4849) = 44.72 deg`,
`omega_ff = 0.5256·2.3604 = 1.241 rad/s`.

#### 8.5.2 Feedback, clamps, rate limit

```
e_theta   = wrap(theta_ref - theta)                            // theta = pose.getRotation().getRadians()
omega_fb  = vsqrt(e_theta; alphaMax, k_theta)                  // same sqrt law as 8.3.7, linear for abs(e) <= alphaMax/k_theta^2
omega_raw = clamp(omega_ff + omega_fb, -omegaMax, omegaMax)
omega     = clamp(omega_raw, omega_prev - alphaMax*dt, omega_prev + alphaMax*dt)
```

- k_theta = 4.0 1/s: same delay argument as 8.3.5, `4·0.085 = 0.34 < 0.368`. Simulated 30 deg step (section 8.15; omegaMax 6, no alpha limit):

  | k_theta | overshoot [deg] | time to stay within 2 deg [s] |
  |---|---|---|
  | 3 | 0.0 | 0.74 |
  | 4 | 0.0 | 0.50 |
  | 5 | 0.33 | 0.36 |
  | 6 | 1.35 | 0.30 |
  | 8 | 4.20 | 0.44 |
- The rate limit is here because `SwerveAccelerationLimiter` passes omega through (SwerveAccelerationLimiter.java:80); alphaMax must be
  applied by the follower. The sqrt law brakes the rotation at alphaMax without overshoot from saturation.
- No deadband (2025 and 2026 had one). The kinematics stands still below `minVelocity = 0.01 rad/s` (KinematicsConfig.java:34,
  DemaciaKinematics.java:271-272) when translation is also below 0.01 m/s; at k_theta = 4 that is abs(e_theta) < 0.14 deg, far inside 2 deg.

Example (EX, continuing 8.5.1): theta_ref - theta = 0.10 rad; `alphaMax/k_theta^2 = 4·pi/16 = 0.785 rad`, so linear: omega_fb = 0.40;
`omega_raw = 1.241 + 0.40 = 1.641 rad/s` (< 2·pi). With omega_prev = 1.40 the band is `+-4·pi·0.02 = +-0.251`, so omega = 1.641.
Saturation: e_theta = 1.5 rad at rest: `vsqrt = sqrt(2·4·pi·(1.5 - 0.393)) = 5.275 rad/s` (< 2·pi); the rate limit raises omega by 0.251 rad/s
per loop until then.

#### 8.5.3 Face-target mode

> Removed in v1 (simplification); kept as v2 reference.

Rule and derivation in section 6.9. Example (EX, robot and V of section 8.3.1, target (8.0, 4.0)) (script): `pT = (6.2972, 0.6901)`,
`abs(pT)^2 = 40.131`; `theta_ref = atan2(0.6901, 6.2972) = 0.1091 rad (6.25 deg)`;
`omega_ff = (0.6901·1.4972 - 6.2972·1.8509)/40.131 = -0.2647 rad/s` (finite difference agrees). The robot moves up and to the left of its line
to the target, so the bearing turns clockwise. Singularity: `omega_ff ~ v/abs(pT)` grows as the robot nears the target, hence the freeze
inside r_face_min = 0.5 m. At 0.5 m and 4 m/s across, omega_ff = 8 rad/s > omegaMax: the clamp applies and the error recovers after the pass.

#### 8.5.4 Angle wrap everywhere

`wrap(a) = MathUtil.angleModulus(a)` is applied to every angle difference: e_theta, the finish test, the stop release, telemetry. theta(s) is
continuous and unwrapped (section 6.3), so the follower never interpolates angles itself. Never compare raw radians (the 2026 bug,
FollowTrajectory.java:45-46). Example: theta_ref = 179 deg, theta = -179 deg: raw difference 358 deg; wrap gives -2.0 deg (turn 2 deg clockwise).

### 8.6 Stop points

A stop point s_c (vBack(s_c) = 0) is an automatic STOP corner (tau_turn > TAU_STOP, radius below R_MIN after shrinking,
`.withRadius(x < R_MIN)`) or a user stop (`.stop()`, `withVelocity(0)`, `.stop(dwellSeconds)`). Both are zero-length
entries with a line on each side (section 5.8.7), so the in-segment is always a line. PASS markers are not stops; `.stop()` on a straight
point makes it a STOP. Each stop has P_c, T_in, T_out, theta(s_c) and a dwell (0 unless `.stop(d)`).

| State | Command | Leaves when | To |
|---|---|---|---|
| TRACK (sub-profile j, s_goal = s_c) | 8.3.1, window capped at s_c | d_go <= d_h (0.05 m), or v_cmd = 0 and d_go <= d_h + 2·STOP_DIST_MARGIN (8.3.7e) | CAPTURE_STOP |
| CAPTURE_STOP | 2D law on P_c (8.3.7e); theta_ref = theta(s_c), omega_ff = 0 | arrival: abs(d_go) <= pos_tol (0.03 m, along T_in) AND abs(V_meas) <= STOP_SPEED (0.1 m/s) | ROTATE or DWELL |
| on arrival | fire stop events (8.8); switch to sub-profile j+1: s_goal = next stop or L; s* = s_hi = s_prev = s_c | | ROTATE if `abs(wrap(theta(s_c) - theta)) > stop_ang_tol` (3 deg), else DWELL |
| ROTATE | 2D law on P_c (holds position); theta_ref = theta(s_c), omega_ff = 0, feedback 8.5.2 | `abs(wrap(theta(s_c) - theta)) <= stop_ang_tol` | DWELL |
| DWELL | same as ROTATE | `now() - t_dwell_start >= dwell` (at once if dwell = 0) | TRACK |
| on release | speed-law state `(v_cmd, a_cmd) = (max(0, V_meas·T_out), 0)` (~= (0, 0)); the law restarts the robot | | TRACK (sub-profile j+1) |

Why:
- The speed condition stops the robot from switching sub-profiles while still sliding through the corner.
- `d_go = (P_c - p)·T_in` (left limit, section 8.1), negative after an overshoot. The window cap keeps s* <= s_c, so TRACK cannot run onto the
  out-leg before release.
- The 2D law is used because the in-leg and out-leg directions differ sharply; a projection near the corner could flip between them.
- No settle time at a stop, and the lateral error is not part of arrival: TRACK on the next leg corrects it within about 0.33 s.
- ROTATE only corrects a residual (section 6.7): theta is continuous through s_c. The modules still steer to the new direction, at most 90 deg
  (DemaciaKinematics.java:194-197); that is not in profileTime (section 7.10.4).

Dwell timing:
- `t_dwell_start = now()` (FPGA time) when DWELL is entered, after any ROTATE. Measured time, not loop counts: `.stop(1.5)` with one 0.06 s
  overrun ends 1.50 to 1.52 s after entry.
- During DWELL the robot holds P_c and theta(s_c); if pushed, the 2D law brings it back; the dwell clock keeps running.
- Order: arrive, (ROTATE), dwell, leave. Stop events fire at arrival, so a 1.5 s dwell gives an event command (shooting, placing) 1.5 s with the
  robot held still.
- `.stop(d)` on the last point: the finish test (8.7.2) passes first, then the command holds in CAPTURE_END for d and ends OK. profileTime and the
  timeout include it.

Overshoot: the window cap keeps s* <= s_c, so an overshoot shows as d_go < 0; the 2D law pulls the robot back at the goal-law speed. Arrival
needs both conditions, so a robot sliding through at 0.5 m/s has not arrived. Stop events do not fire on the crossing. Example: arriving at
0.6 m/s with a lag overshoot of 0.04 m: d_go = -0.04, `V = 3·(P_c - p) = 0.12 m/s` backward along T_in; about 0.3 s later abs(d_go) < 0.03 and the
speed < 0.1: arrived.

Example (EX, user stop `.stop(1.5)`; s_c = 3.20 m, P_c = (4, 2), in-leg along +x) (hand):
- The speed law brings the robot to s_c - d_h = 3.15 m at v_h = 0.15 m/s. At s* = 3.15 (p = (3.95, 2.01)), d_go = 0.05 <= d_h:
  CAPTURE_STOP, `V = 3·(0.05, -0.01) = (0.15, -0.03) m/s`, speed continuous with the law's 0.15 m/s.
- p = (3.98, 2.004), abs(V_meas) = 0.06 m/s: d_go = 0.02 <= 0.03 and 0.06 <= 0.1: arrived. Stop events fire. Sub-profile j+1.
- Residual heading error 4 deg (lag) > 3 deg: ROTATE. `omega_fb = 4·0.0698 = 0.279 rad/s` (linear zone); within 3 deg after about
  `ln(4/3)/4 = 0.072 s` (4 loops; lag not modelled).
- DWELL for 1.5 s measured, then release with the law's state (~0, 0); it commands 0.01, 0.04, 0.09, 0.15, ... m/s along T_out.

Edge cases: two stops closer than about 2·d_h plus the braking distance: TRACK may enter CAPTURE_STOP at once after a release (go-stop-go).
Pushed during ROTATE or DWELL: the 2D
law brings it back; release does not test position again. `.stop()` on P_0: no effect on the path (W4); `.stop(d)` there delays the start by d.

### 8.7 Finish logic

#### 8.7.1 Hand-off to the end-point controller

The end is the last goal: s_goal = L, P_goal = P_end = x(L). TRACK becomes CAPTURE_END when d_go <= d_h (or at rest within d_h + 0.01 m), with the continuous hand-off of
8.3.7e. A position controller is needed because s* saturates at L: past the end, vBack(L) = 0 and the TRACK law cannot pull the robot back;
`V = k_p·(P_end - p)` can, in any direction. The heading target in CAPTURE_END is theta_end = theta(L) (or the face-target value, section 6.9),
with omega_ff = 0 once s* = L.

#### 8.7.2 Finish test

```
inTol = abs(P_end - p) < pos_tol  AND  abs(wrap(theta_end - theta)) < ang_tol      (0.03 m, 2 deg)
t_in  = inTol ? t_in + dt : 0          // dt measured (8.2.2)
finished_ok = (state == CAPTURE_END) AND (t_in >= t_settle)                         (0.1 s)
```

- abs on the wrapped error (the 2025 bug was the missing abs, DemaciaTrajectory.java:270). Strict `<` on both, as in decision 10.
- Measured time, not a count of loops: one 0.06 s overrun counts 0.06 s.
- With the linear law at 0.03 m the commanded speed is `3·0.03 = 0.09 m/s`, so finishing while moving through the tolerance disc is not a
  practical risk; no speed condition is added (decision 10 is fixed).

Example: errors 0.025 m and 1.5 deg for 4 loops (0.08 s), then a 0.031 m sample resets t_in to 0; then 5 good loops: t_in = 0.10, finished.
Simulated (arc path of section 8.15): done at 3.84 to 4.04 s for profileTime 3.62 s (k_n = 3, depending on tau_lag), final error 0.020 m.

#### 8.7.3 Timeout

```
t_timeout = 1.5 * profileTime + 1.0         // decision 10; profileTime with the dwells; 1.5 and 1.0 are PathFollowerConfig fields
if (t - t_start >= t_timeout) -> end, result FAILED_TIMEOUT
```

profileTime is the closed form of section 7.12 with the hand-over tail (v1). Examples (EX): shared example 3.4426 s, timeout 6.16 s;
P1.stop() 4.2245 s, timeout 7.34 s; P1.stop(1.5) 5.7245 s, timeout 9.59 s. The dwell is exact, so 1.5 times it is spare margin. The 0.1 s settle and the modules' turn at a stop (at most 0.13 s)
are not in profileTime; they fit in the margin. The timeout clock never pauses during ROTATE or DWELL; a robot that cannot reach a stop times out the same way.

#### 8.7.4 After a failure

On any failure the command ends and sends **zero ChassisSpeeds once** (resolution R4). Reasons: a follow command that never ends blocks `andThen`
sequences in autonomous; after a timeout the robot is most likely blocked, and pushing with a position loop draws current and can cause contact
fouls; at zero velocity the drive motors resist pushing like a damper, and `DemaciaKinematics` keeps the module angles when standing
(DemaciaKinematics.java:271-277). For active holding, the command offers `holdEndCommand()`, which runs the CAPTURE_END law on P_end until
interrupted: `FollowPathCommand f = new FollowPathCommand(...); f.andThen(f.holdEndCommand())`, made fresh each time it is scheduled
(a WPILib command can belong to only one composition). It is on the command, not on `Path`, because a `Path` has no suppliers. The result is
readable with `getResult()` and is sent to telemetry.

### 8.8 Events

> Removed in v1 (simplification); kept as v2 reference.

Each event has `s_event` in [0, L] (section 5.9.5) and an action, a `Command` scheduled with `CommandScheduler.getInstance().schedule(...)`. Each
event has a `fired` latch, reset in initialize.

```
for events in ascending s_event (same s: in the order added):
    if !fired and !isStopEvent and s_event <= s*:  schedule(action); fired = true
// stop events (s_event == s_c of a stop point) fire in the arrival transition instead (8.6)
```

- Fires once: the latch plus monotonic s*. Skipped crossings: if s* jumps from 0.95 to 1.12 in one loop, every event in (0.95, 1.12] fires in that
  loop, in order. A loop never "misses" an event (the 2026 time window could, FollowTrajectory.java:52).
- s = 0 (fraction 0 or point 0): fires on the first execute.
- Events at a stop point (an event placed by that waypoint's index gives `s_event = s_c` exactly) fire on arrival, before ROTATE and DWELL; the
  crossing rule would fire them while the robot is still sliding through.
- s = L and events within pos_tol of L: the robot can finish with s* = L - 0.02, so on a successful finish every unfired event fires before
  `isFinished` returns true.
- Timeout or interrupt: unfired events do not fire.
- An event command that requires the drivetrain would cancel the follower; the `FollowPathCommand` constructor rejects it (V20).

Example (shared example): events at fraction 0.2 (s = 1.1954, intake down) and at point 2 (s = L = 5.9769, shoot). s* goes 1.10 -> 1.27:
intake fires once. The robot finishes at s* = 5.9569: shoot fires at the finish.

### 8.9 Re-plan hook (design only, v2)

Not in v1 (resolution R5); recorded so v2 can add it without changing the follower's structure.

```
offPath = abs(e_n) > e_replan (0.5 m) OR abs(e_t) > 0.15 m (pushed back past the floor, 8.2.4)
stuck   = v_t > 0.5 m/s AND filtered ds*/dt < 0.1 m/s           // commanded to move, not progressing
t_off   = (offPath OR stuck) ? t_off + dt : 0
if t_off >= t_replan (0.5 s) [1.0 s for stuck] and armed:
    armed = false
    decision = replanCallback.apply(new ReplanRequest(pose, s*, e_n, e_t, reason))   // OFF_PATH or STUCK
    if decision == ABORT: end with result FAILED_REPLAN
re-arm when abs(e_n) < e_replan/2 for 0.5 s
```

The callback (`Function<ReplanRequest, ReplanDecision>`, CONTINUE or ABORT) fires once per episode. The follower never builds a new path
(decision 8); the user's callback may cancel and schedule a new follow command. Example: bumped to e_n = 0.6 m and back under 0.5 m after
0.1 s: no callback; pinned against a wall at 0.6 m for 0.5 s: one callback.

### 8.10 Alliance flip at runtime

> Changed in v1: no flipped-copy memo and no `Path.forCurrentAlliance`; `initialize()` reads the alliance once and calls `path.flipped(config)` on red. Alert A2 is a `DriverStation.reportWarning`.

- In `initialize()`, once: read `DriverStation.getAlliance()`. Red: `activePath = path.flipped(config)`; blue: the path itself; empty (no
  Driver Station yet): blue, alert A2. With `FlipMode.NONE` the path is never flipped.
- `flipped` is memoized inside the `Path`, so scheduling the command again does not rebuild anything. The face target is flipped with the same
  transform and mode (section 5.13).
- Poses from the supplier are always blue-origin (RobotPose.java:267 documents `getEstimatedPose` that way). The flipped path is in the same frame,
  so the follower has no alliance branches after initialize.
- Never per loop: it would cost work every loop, and a mid-run alliance change would make the path jump.
- MIRROR flips the turn direction: sigma, kappa and theta' change sign; the follower uses N from the active path, so the sign rule of 8.3.1 stays
  correct. Event s values are unchanged.

Example: ROTATE, placeholder field 16.54 x 8.07 m: P0 = (1, 2) becomes (15.54, 6.07); heading 0 becomes pi.

### 8.11 Output interface
The follow command takes a `PathDrivetrain` (section 9.10): it reads `getPose()` and `getFieldSpeeds()` and sends the command to `driveFieldRelative(ChassisSpeeds)`; the drivetrain is its requirement.
`Subsystem... requirements`.

Measured speeds: field-relative, blue frame, omega CCW. Used for v0 at initialize, the stop arrival test, the window, stale-pose detection and
telemetry; never fed back into the speed reference (the 2026 bug). In the example drivetrain the value is
`fromRobotRelativeSpeeds(kinematics.toChassisSpeeds(getModuleStates(), getGyroRate()), heading())`, the expression `periodic` uses
(ExampleSwerveWithPose.java:335-336). The example now has `getFieldSpeeds()` for it (section 9.13f); `getGyroRate` stays private.

Each loop: `output.accept(new ChassisSpeeds(vx, vy, omega))`, vx and vy field-relative in the blue frame [m/s], omega CCW-positive [rad/s]. The
drivetrain does the rest; reference **Fact**, `ExampleSwerveWithPose.driveFieldRelative` (:304-313):
1. Desaturate in the robot frame: `fromRobotRelativeSpeeds(kinematics.desaturate(fromFieldRelativeSpeeds(wanted, heading)), heading)`
   (`desaturate` at DemaciaKinematics.java:260).
2. Limit: `limiter.calculate(wanted, lastFieldSpeeds, heading)` (SwerveAccelerationLimiter.java:64): field-relative, omega passed through; dt capped at
   maxDt = 0.02 (:75).
3. Drive: `drive(fromFieldRelativeSpeeds(..., heading))` (:287-297) negates omega and calls `kinematics.toSwerveModuleStates(..., getModuleStates())`
   (DemaciaKinematics.java:183), which takes omega clockwise-positive (:164-168).
4. Next loop starts from `lastFieldSpeeds = fromRobotRelativeSpeeds(kinematics.getSentSpeeds(), heading)` (DemaciaKinematics.java:241;
   ExampleSwerveWithPose.java:311); when something else drove last loop, `periodic` re-syncs from measured speeds (:333-339).

Rules (**Recommendation**): do not negate omega (only the drivetrain's `drive` does it, once; the 2026 trajectory negated it internally,
DemaciaTrajectory.java:193); do not discretize (`DemaciaKinematics` already discretizes over `cycleDt + 2·latency`, DemaciaKinematics.java:84, :280);
do not rate-limit translation (the limiter does), but do rate-limit omega (8.5.2). The limiter is optional for the user but recommended; without it,
correction steps from pose jumps reach the modules unfiltered.

**Inference, consequences for tuning:** the forward limit `maxForwardAccel·(1 - v/freeSpeed)` is 2.45 m/s^2 at 4 m/s with the defaults (10, 5.3);
aAccEff follows 0.9 of it, so the speed law stays inside and the limiter passes the command unchanged. A per-path aAcc above it makes the robot lag the
command, which is harmless (braking is by position) but uses timeout margin. `minVelocity = 0.01` means the robot stands at `abs(d) < 0.0033 m` (k_p = 3) and
abs(e_theta) < 0.14 deg: no interference with the tolerances.

### 8.12 Failure modes

| Failure | Detection | Behavior |
|---|---|---|
| Pose jump (vision correction) | none needed | No mode change. Along track: the window catches up by `ahead` per loop, or the floor holds s*. Across track: an e_n step of x m gives a `k_n·x` command step (clamped at c_max) that the limiter spreads |
| Robot bumped off path | e_n large, or e_t < -eps_back | Correction clamp (8.3.8) |
| Starts away from P0 | `d0 = abs(p - P0)` at initialize (8.2.5) | > 0.5 m: end FAILED_START_TOO_FAR with alert A1. Within 0.5 m behind P0: the law speeds up along T(0), s* = 0 until P0 is passed |
| Self-crossing path / arc > 180 deg | none needed | Window capped at the next segment and 90 deg of arc (8.2.3); monotonic s* (8.2.4); s* starts at 0 (8.2.5) |
| Very short path (L < L_short = 0.10 m) | L at initialize | Skip TRACK: start in CAPTURE_END on P_end with heading theta_end, then the finish test. A zero-length path cannot be built (V6) |
| Disabled mid-path | the scheduler cancels the command (**Inference**, WPILib default without `runsWhenDisabled`) | `end(true)`: zero speeds once, result INTERRUPTED, no more events. Re-scheduling runs the start check again: more than 0.5 m from P0 gives FAILED_START_TOO_FAR (no mid-path start) |
| Pose supplier NaN / infinite | any of x, y, theta not finite | Skip projection and events. Repeat the last command for up to t_stale = 0.1 s, then send zero (the limiter brakes). Resume when valid. Invalid for > 1 s: end FAILED_POSE. The timeout keeps running |
| Pose supplier stale | bit-identical x, y, theta for >= 0.1 s while abs(V_meas) > 0.3 m/s | Same as NaN |
| Speed supplier NaN | any component not finite | Use `abs(V_meas) = v_t_prev` for the window; arrival test uses 0.1 s of `abs(d_go) <= 0.03` instead of the speed; v0 = 0 at initialize. Alert A3 |
| Loop overrun | measured dt > dt_nom | Window and speed law use the measured dt; settle and timeout use measured time; events never skipped |
| Wrong omega sign in the drivetrain | telemetry: eThetaDeg grows instead of shrinking | None automatic. Test FOL-I1 checks it |
| Alliance unknown at initialize | Optional empty | Use blue, alert A2 |
| Pose supplier in a different frame (red origin) | not detectable | Documented: the supplier must be blue origin (decision 12) |

### 8.13 Telemetry

> Changed in v1: there is no `PathLogger` or `PathTelemetry`. The keys below are added by `FollowPathCommand.initSendable`, so `SmartDashboard.putData("Path", cmd)` (or any logger that takes a `Sendable`) shows them; failures use `DriverStation.reportWarning`.

`FollowPathCommand` publishes **one** `Sendable` under `Path/<name>` with `PathLogger.get().putData(...)`, once, the first time the
command is initialized (section 9.11; the same shape as RobotPose's `PoseLogger`, PoseLogger.java:41, :51). Its `initSendable` adds
these read-only properties, read by the dashboard or logger (**Decision**, user, simplification pass: only what tuning needs):

| Key | Unit | Why |
|---|---|---|
| `state` | enum name | TRACK / CAPTURE_STOP / ROTATE / DWELL / CAPTURE_END / DONE / FAILED_* |
| `segment` | int | segment index at s* (section 5.9.3) |
| `s` | m | progress s* |
| `eN` | m | cross-track error (tune k_n and tau_lag) |
| `vRef` | m/s | the speed law's v_cmd |
| `vMeas` | m/s | abs(V_meas); compare with vRef for lag |
| `eThetaDeg` | deg | heading error (tune k_theta) |

Failures and warnings are not telemetry keys: they go through `PathLogger.alert` (section 9.12). The result is also readable with
`getResult()`.

Tuning procedures: tau_d from a line step, fitting eN(t) to `exp(-k_n·(t - tau_d))`, or comparing vRef with vMeas; tau_lag from the mean
eN on a long arc (section 8.4); k_n: raise until eN overshoots on the step, then back off 25%.

### 8.14 Lifecycle pseudocode

```
initialize():
    if first initialize: PathLogger.get().putData("Path/" + path.name(), telemetry)  // one Sendable (8.13)
    alliance = DriverStation.getAlliance()                                         // empty: blue, alert A2
    active   = (alliance == Red && config.flipMode() != NONE) ? path.flipped(config) : path   // once, memoized in Path
    p = pose.get(); Vm = speeds.get()                                              // invalid pose: result = FAILED_POSE
    d0 = abs(p - active.startPoint())
    if d0 > config.startTol(): result = FAILED_START_TOO_FAR; alert A1; return     // isFinished ends it (8.2.5)
    v0 = max(0, Vm·T(0))
    heading = buildHeading(active, p.getRotation(), faceTarget)                    // section 6.13
    profile = buildProfile(active, constraints, kinematicsConfig, R_mod, v0, heading)   // section 7.17
    t_start = t_prev = now(); s_hi = s_prev = 0; (v_cmd, a_cmd) = (v0, 0); v_t_prev = v0; omega_prev = Vm.omega; t_in = 0; t_fin = NaN
    reset event latches; result = RUNNING
    s_goal = first stop point > 0, or L                                             // selects the sub-profile
    state  = (active.L < L_short) ? CAPTURE_END : TRACK

execute():
    dt = clamp(now() - t_prev, 0.005, 0.10); t_prev = now()
    p = pose.get(); Vm = speeds.get(); if invalid(p): handleInvalid(); return       // 8.12
    s* = windowedProjection(p) (8.2.3)
    s_hi = max(s_hi, s*); s_prev = s*; e_t = (p - x(s*))·T(s*); d_go = (s_goal - s*) - e_t
    fireEvents(s*)                                                                 // 8.8
    switch state:
      TRACK:        if d_go <= d_h || (v_cmd == 0 && d_go <= d_h + 2*STOP_DIST_MARGIN): state = (s_goal == L) ? CAPTURE_END : CAPTURE_STOP
      CAPTURE_STOP: if abs(d_go) <= pos_tol && abs(Vm) <= STOP_SPEED:               // arrival (8.6); d_go along T_in
                        s_c = s_goal; fireStopEvents(s_c)
                        s* = s_hi = s_prev = s_c; s_goal = nextStopOrL
                        state = (abs(wrap(theta(s_c) - theta)) > stop_ang_tol) ? ROTATE : DWELL; if DWELL: t_dwell_start = now()
      ROTATE:       if abs(wrap(theta(s_c) - theta)) <= stop_ang_tol: t_dwell_start = now(); state = DWELL
      DWELL:        if now() - t_dwell_start >= dwell(s_c): (v_cmd, a_cmd) = (max(0, Vm·T_out(s_c)), 0); state = TRACK
      CAPTURE_END:  update t_in (8.7.2); if t_in >= t_settle and isNaN(t_fin): t_fin = now()
    V     = (state == TRACK) ? trackLaw() (8.3.1, steps the speed law) : captureLaw(P_goal or P_c) (8.3.7e)
    omega = headingLaw(state) (8.5)
    output.accept(new ChassisSpeeds(V.x, V.y, omega))
    v_t_prev = (state == TRACK) ? v_t : abs(V); omega_prev = omega                  // the Sendable reads the new values

isFinished():
    if result is FAILED_*: return true
    if now() - t_start >= 1.5*profileTime + 1.0: result = FAILED_TIMEOUT; return true   // factor and margin from PathFollowerConfig
    if state == CAPTURE_END && !isNaN(t_fin) && now() - t_fin >= dwell(L):         // dwell(L) = 0 unless .stop(d) on the last point
        fireRemainingEvents(); result = OK; return true
    return false

end(interrupted):
    output.accept(new ChassisSpeeds())          // zero once (8.7.4)
    if interrupted && result == RUNNING: result = INTERRUPTED
    if result is FAILED_*: alert A1 (text per result)
```

### 8.15 What was executed

Python simulations (not part of the repository), run with `python3 -I`:
- A point robot with heading: the command sampled every 20 ms, delayed 0.035 s, then a first-order velocity lag of 0.04 s, with a skid-disk limiter of
  8 m/s^2 on the command change. Path: 3 m line, 90 deg left arc (radius 1 or 3 m), 3 m line; numerically computed profile with vMax 3,
  aAcc = aDec = aLat = 4 (these simulation constraints differ from EX). It implements sections 8.2.3 to 8.2.5 (simplified), 8.3.1, 8.3.7 and the
  position part of 8.7.2. Results: the arc offsets of 8.3.4, all k_n 2 to 6 by tau_lag 0, 0.05, 0.085 finished, final error 0.0005 to 0.024 m, done
  3.56 to 4.64 s against profileTime 3.62 s. With the vBack-plus-ramp law: k_n = 3, tau_lag 0: max abs(e_n) 0.100, arc mean -0.0766, done 3.96 s;
  tau_lag 0.05: 0.041, -0.0313, 3.94 s. Start 0.5 m behind P0: done 4.10 s. These runs used the jerk-free speed law; the jerk-aware law
  was simulated in the same plant model (section 7.13.8): no overshoot of any stop or end, arrival error 0.7 to 1.8 cm.
- The line-step table (8.3.5), the steady-state arc offsets (predicted -0.113 / -0.047 / 0; simulated -0.106 / -0.043 / +0.0015 m), the heading step
  table (8.5.2; omegaMax 6, no alpha limit).
- Every worked number in this chapter, including the face-target omega checked by finite difference (script).

Not executed: the full stop state machine (the jerk-law plant runs include stop arrival only), events, failure handling, the omega rate limit, the real limiter (only its skid disk), and any Java.

Tests: section 13.4.

## 9. Public API

The public Java API of package `frc.robot.Path`. It changes no math; it names the classes, every user error with its
exact text, and the examples. The signatures and examples below were checked against the implemented code, which is the final
reference.

### 9.1 Conventions

- **Frame and units:** WPILib field frame, blue-alliance origin; meters, radians, seconds. Every `Point`, `Translation2d` and
  `Rotation2d` the user passes is in the **blue** frame on both alliances; red is made by a flip (section 9.9).
- **Immutable value objects.** `Point`, `Path`, `PathConstraints`, `PathsConfig` and `PathFollowerConfig` never
  change. Each `withX(...)` returns a **new** object. **Fact**
  (KinematicsConfig.java:61-114): `KinematicsConfig.withX` does the opposite (changes `this` and returns it). Rule to teach:
  "write `p = p.withX(...)` or chain the calls". `FollowPathCommand` is a command, not a value: its `withX` methods configure
  `this` before it is scheduled.
- **Exceptions at construction, warnings at build time, messages at run time.** Bad values throw (`IllegalArgumentException` unless
  stated) when the object is created, so the stack trace points at the student's line. Problems the library repairs (a shrunk
  radius, a created stop, merged points) are reported once with `DriverStation.reportWarning` and
  collected in `Path.warnings()`. Nothing throws at run time; run-time failures end the command with a `Result` and a message (9.12.3).
- **The geometry is built in the `Path` constructor** (blue frame). The heading and speed profiles are built in
  `FollowPathCommand.initialize()`, because they need the measured start speed and heading. Geometry warnings appear
  when `Path` objects are created (robot start-up if they are `static final`).
- **No dependency on RobotPose** (decision 12). The only inputs are suppliers.

### 9.2 Package layout

Directory `Infrastructures/src/main/java/frc/robot/Path`, flat (like `frc/robot/Kinematics`, so installing is "copy one
directory"). **Nine files** (v1, simplified 2026-11-03). There are exactly three public config types,
`PathConstraints`, `PathsConfig` and `PathFollowerConfig` (**Decision**, user, simplification pass).

| File | Kind | Responsibility |
|---|---|---|
| `Point.java` | public final class | one waypoint: position plus optional angle, max speed, radius, stop with dwell |
| `Path.java` | public final class | points, constraints, name; builds and holds the geometry; `flipped(...)`, `warnings()` |
| `PathConstraints.java` | public record | vMax, aAcc, aDec, aLat, omegaMax, alphaMax; `fromConfig(KinematicsConfig, PathsConfig)` |
| `PathsConfig.java` | public record | robot settings the Kinematics module does not have: max omega, max alpha |
| `PathFollowerConfig.java` | public record | follower gains, tolerances, timeout, flip mode and field size; nested `FlipMode`; constants `K_THETA`, `TAU_LAG`, `T_SETTLE`, `FIELD_LENGTH`, `FIELD_WIDTH` |
| `PathDrivetrain.java` | public interface extends `Subsystem` | what the follower needs from the drivetrain: `getPose`, `getFieldSpeeds`, `driveFieldRelative`, default `getKinematicsConfig`, `getPathsConfig` |
| `FollowPathCommand.java` | public class extends `Command` | the follower (the tracker is inside it); nested `Result`; a `Sendable` |
| `PathGeometry.java` | package-private | merging, pass markers, corner circles, radius rule, tangents, sweeps, segment table, projection, flip (sections 5.3 to 5.13), the geometry constants; nested `PathSample` |
| `SpeedProfile.java` | package-private | heading profile (section 6), spans, caps, passes, constant-speed arcs, time, the online speed law (sections 7, 7.13) |

Merged in v1 (earlier design in brackets): `GeometryBuilder`, `SegmentTable`, `Projector`, `PathFlipper` into `PathGeometry`; `HeadingProfile`, `SpeedLaw` into `SpeedProfile`;
`Tracker` into `FollowPathCommand`. Deleted: `FollowResult` (now `FollowPathCommand.Result`), `FlipMode` (now `PathFollowerConfig.FlipMode`), `PathEvent`, `ResolvedEvent`,
`EventSchedule`, `PathLogger`, `PathWarnings`, `PathTelemetry`, `PathMath`.

**Inference:** the class `Path` in package `frc.robot.Path` has the same shape as `frc.robot.RobotPose.RobotPose`. A file that also
imports `java.nio.file.Path` must use the full name for one of them. No global registry and no static state.

### 9.3 Point

```java
public final class Point {
    public Point(double x, double y);                 // blue frame, meters
    public Point(Translation2d position);
    public Point(Pose2d pose);                        // position + withAngle(pose.getRotation())

    // each returns a NEW Point; the last call of a kind wins
    public Point withAngle(Rotation2d angle);         // heading anchor (section 6)
    public Point withVelocity(double metersPerSecond);// max speed at this point; 0 means stop()
    public Point withRadius(double meters);           // corner radius override; 0 = stop corner
    public Point stop();                              // v = 0 here, no wait
    public Point stop(double dwellSeconds);           // v = 0 here, then wait

    public Translation2d position();
    public double x();
    public double y();
    public Optional<Rotation2d> angle();
    public OptionalDouble maxVelocity();              // empty if not set; never 0 (0 is stored as a stop)
    public OptionalDouble radius();
    public boolean isStop();                          // stop() / stop(d) / withVelocity(0)
    public double dwellSeconds();                     // 0 unless stop(d) with d > 0
    @Override public boolean equals(Object o);
    @Override public int hashCode();
    @Override public String toString();               // "Point(3.0, 4.0, angle=90.0deg, stop 1.5 s)"
}
```

Rules:
- `withAngle`: any point, including P_0 and P_{n-1}. The heading is linear in s between anchors, the short way (section 6.3); a point
  without an angle has no anchor; after the last anchor the heading holds; with no angle on P_0 the path starts from the robot's
  measured heading.
- `withVelocity(v > 0)`: a cap at this waypoint's s: the robot passes the point at no more than v; not a speed zone (section 7.4.2). On a
  corner it caps the whole arc through the point (constant speed on the arc; the robot brakes before the arc and speeds up after it). Ignored with warning W4 on P_0 and, in v1, on
  P_{n-1}. `withVelocity(0)` is exactly `stop()`. `v < 0` throws (V11).
- `stop()` / `stop(d)`: any middle point; on the last point the stop is the default and `stop(d)` holds d seconds after the finish test
  passes; on P_0 no effect on the path (W4), `stop(d)` delays the start by d. A stop splits the profile (section 7.10) and runs arrive,
  residual-heading correction, dwell, leave (section 8.6).
- **A stop point is a sharp corner** (section 5.8.7): the arc of that corner is removed; the path arrives on a straight line, stops exactly
  on the point, and leaves on a straight line. Neighboring corners are recomputed with P_i as an endpoint, so their lines and radii can
  change slightly. The same holds for a collinear point: `stop()` makes it a STOP.
- **A point exactly on a straight line is a pass marker** (section 5.8.3): the path is built without it and may miss it by a few cm
  next to a round corner (warning W5 above 0.01 m). Use `stop()` on it if the robot must pass exactly through it.
- **`stop()` and `withRadius(r > 0)` do not combine**: `IllegalArgumentException` from the `Path` constructor (V30). `withRadius(0)` with
  `stop()` is allowed (same meaning). The check is in the `Path` constructor, so the order of the calls does not matter. Reason: the two ask
  for opposite things, and a silent choice would change the path.
- `withRadius(r)` on P_0, P_{n-1} or a point exactly on a straight line: ignored, warning W4 (resolution R8).
- Javadoc summary for `stop()`: "Stops the robot exactly on this point. A stop point is a sharp corner: the robot arrives on a straight line,
  stops on the point, then continues on a straight line. It has no arc, so it cannot be combined with withRadius(r) for r greater than 0."
- Merging (section 5.8.2): if two points are merged, the later attribute wins, with warning W3 if both set one.
- Last call wins within one `Point`: `p.stop(2).withVelocity(1.0)` has max speed 1.0 and no stop; `p.withVelocity(1.0).stop()` is a stop;
  `p.stop(2).withVelocity(0)` is `stop()` with dwell 0.

### 9.4 Path

```java
public final class Path {
    // the geometry is built here (section 5.15). Throws on bad input (9.12), warns for repairs (DriverStation.reportWarning, once).
    public Path(Point... points);
    public Path(PathConstraints constraints, Point... points);
    public static Path of(List<Point> points);
    public static Path of(PathConstraints constraints, List<Point> points);

    // value changes (each returns a NEW Path; the geometry is shared)
    public Path withName(String name);                          // used in the command name and run messages
    public Path withConstraints(PathConstraints constraints);   // per-path override

    // alliance (section 5.13). A new flipped copy on every call; no memo.
    public Path flipped(FlipMode mode, double fieldLength, double fieldWidth);  // NONE returns this
    public Path flipped(PathFollowerConfig config);

    // read-only
    public String name();                                       // default "path" + a counter
    public double length();                                     // L [m]
    public int pointCount();                                    // as given by the user (before merging)
    public List<Point> points();
    public double sAtPoint(int pointIndex);                     // s of waypoint i (5.9.4); merged points map to the kept one
    public Optional<PathConstraints> constraints();             // empty = resolved when the command starts (9.10)
    public List<String> warnings();                             // everything reported at build time
    public Translation2d startPoint();                          // P_0
    public Translation2d endPoint();                            // P_{n-1}
    public Optional<Rotation2d> startHeading();                 // P_0's angle if set
    public List<Pose2d> polyline(double spacingMeters);         // for tests and the user's own Field2d; rotation = direction of travel
}
```

Rules: `new Path(constraints, p0, p1)` and `new Path(p0, p1)` do not clash; `new Path()` fails V1. `withConstraints` and `withName` do not rebuild the
geometry. The name is used in the command name and in run messages; warning texts keep the automatic name the path had when it was built.
Removed in v1 (kept as v2 reference): `withFaceTarget`, `withoutFaceTarget`, `faceTarget()`, `withEvent...`, `events()`, `isFlipped()`, `forCurrentAlliance`,
the flip memo and `sampleAt`.

### 9.5 PathConstraints

```java
public record PathConstraints(double vMax, double aAcc, double aDec, double aLat,
                              double omegaMax, double alphaMax) {
    public static final double PROFILE_MARGIN    = 0.9;

    public PathConstraints { /* every field finite and > 0, else IllegalArgumentException (V16) */ }

    public static PathConstraints fromConfig(KinematicsConfig config, PathsConfig paths);
    public static PathConstraints fromConfig(KinematicsConfig config);   // = fromConfig(config, PathsConfig.defaults())

    public PathConstraints withVMax(double vMax);
    public PathConstraints withAAcc(double aAcc);
    public PathConstraints withADec(double aDec);
    public PathConstraints withALat(double aLat);
    public PathConstraints withOmegaMax(double omegaMax);
    public PathConstraints withAlphaMax(double alphaMax);
}
```

`fromConfig(c, p)` (field names of `c` **Fact**: KinematicsConfig.java:32, :46, :51, :53, :55, :57; formulas section 7.2.1):

```
vMax     = PROFILE_MARGIN * c.maxAllowedModuleVelocity
aAcc     = PROFILE_MARGIN * min(c.maxForwardAccel, c.maxSkidAccel, c.maxTiltAccelFront, c.maxTiltAccelSide)
aDec     = PROFILE_MARGIN * min(c.maxSkidAccel, c.maxTiltAccelFront, c.maxTiltAccelSide)
aLat     = aDec formula
omegaMax = p.maxAngularVelocity,  alphaMax = p.maxAngularAccel     (no margin: these are path limits, not limiter limits)
```

With `new KinematicsConfig()` and `PathsConfig.defaults()`: (4.05, 7.2, 7.2, 7.2, 2·pi, 4·pi). When the path has no constraints, the command takes the `KinematicsConfig` and the
`PathsConfig` from the drivetrain (`getKinematicsConfig()`, `getPathsConfig()`, section 9.10). 

### 9.6 Config records

**Decision** (user, simplification pass): there are exactly three public config types: `PathConstraints` (section 9.5),
`PathsConfig` and `PathFollowerConfig`. The geometry and profile settings of the earlier design (R_MIN, R_MAX, D_BULGE, EPS_DEV,
TAU_STOP, SHRINK, MAX_ITER, HEADING_LAG_ALLOWED, FT_SPAN, ...) and the follower internals (window, correction clamp, freeze radius,
stop speed and angle) are documented `static final` constants inside the library. Section 10 lists them, marked "internal constant",
and is the only place their values are written.

`PathsConfig` holds the robot settings the path needs that `KinematicsConfig` does not have. **Decision** (user): the Kinematics
module is not changed; these live in the Path module. It is an immutable record (`withX` returns a copy), unlike `KinematicsConfig`,
whose `withX` changes the object (KinematicsConfig.java:61-64).

```java
public record PathsConfig(
        double maxAngularVelocity,   // [rad/s]   robot omega limit -> PathConstraints.omegaMax (not maxSteerRate, which is the module steer rate)
        double maxAngularAccel) {    // [rad/s^2] robot alpha limit -> PathConstraints.alphaMax (the limiter passes omega through)
    public PathsConfig { /* every field finite and > 0, else IllegalArgumentException (V16) */ }
    public static PathsConfig defaults();   // (2·pi, 4·pi); both placeholders, measure on the robot (section 12)
    public PathsConfig withMaxAngularVelocity(double radPerSec);
    public PathsConfig withMaxAngularAccel(double radPerSec2);
}
```

`moduleRadius` and `fromModules` (wheel-speed cap) are removed in v1; sections 7.2.2 and 7.11 are the v2 reference.

`PathFollowerConfig` holds the follower's gains, tolerances and timeout, and the alliance flip:

```java
public record PathFollowerConfig(
        double kN,               // [1/s]  cross-track gain k_n (8.3.5)
        double kP,               // [1/s]  goal-capture gain k_p; keep it equal to kN (8.3.7)
        double posTol,           // [m]    finish and stop-arrival position tolerance (8.6, 8.7.2)
        double angTolRad,        // [rad]  finish heading tolerance (8.7.2)
        double startTol,         // [m]    largest start distance from P0 (8.2.5)
        double timeoutFactor,    // [-]    t_timeout = timeoutFactor·profileTime + timeoutMargin (8.7.3)
        double timeoutMargin,    // [s]
        FlipMode flipMode,       //        NONE / MIRROR / ROTATE (5.13, 9.9)
        double fieldLength,      // [m]    FL
        double fieldWidth) {     // [m]    FW
    static final double K_THETA = 4.0, TAU_LAG = 0.05, T_SETTLE = 0.1;   // constants, not settings (section 10)
    public static final double FIELD_LENGTH = 16.54, FIELD_WIDTH = 8.07;
    public enum FlipMode { NONE, MIRROR, ROTATE }

    public PathFollowerConfig { /* every number finite and > 0 (V16), flipMode not null (V3) */ }
    public static PathFollowerConfig defaults();   // values in section 10
    public PathFollowerConfig withKN(double kN);
    public PathFollowerConfig withKP(double kP);
    public PathFollowerConfig withPosTol(double meters);
    public PathFollowerConfig withAngTolDegrees(double degrees);
    public PathFollowerConfig withStartTol(double meters);
    public PathFollowerConfig withTimeout(double factor, double marginSeconds);
    public PathFollowerConfig withFlip(FlipMode mode);
    public PathFollowerConfig withField(double lengthMeters, double widthMeters);
}
```

- The student-facing settings are kN, kP, posTol, angTol and the flip mode; the guide names only these. `kTheta`, `tauLag` and `tSettle` are constants: to tune one, edit the constant.
- `kP != kN` is valid; the hand-off of section 8.3.7e then has a small step. There is no warning for it.

### 9.7 Events

> Removed in v1 (simplification); kept as v2 reference.

There is no `PathEvent`, `ResolvedEvent`, `Path.withEvent...` or `Path.events()` in v1. To act part way along the path, keep the command and use
`FollowPathCommand.progress()` (0 to 1) or `distanceTraveled()` (m) in a `Trigger` (example 9.13 (e)). The design below (section 8.8) is the v2 reference:
`PathEvent.atFraction/atPoint`, scheduled with the `CommandScheduler`, fired once when s* reaches them, events at a stop firing on arrival, event
commands not allowed to require the drivetrain (V20).

### 9.8 Face target

> Removed in v1 (simplification); kept as v2 reference.

There is no `Path.withFaceTarget` or `withoutFaceTarget` in v1; the robot's heading comes from the `withAngle` anchors only (section 6). The v2 design
(section 6.9): the robot's front faces a blue-frame target, the target is flipped with the path, the heading freezes inside r_face_min = 0.5 m, and
`withAngle` anchors are ignored with a warning.

### 9.9 Alliance flip

```java
public enum FlipMode { NONE, MIRROR, ROTATE }   // nested in PathFollowerConfig; MIRROR about x = FL/2; ROTATE 180 deg about the field center (5.13)
```

The mode and the field size are fields of `PathFollowerConfig` (`flipMode`, `fieldLength`, `fieldWidth`, section 9.6). `PathFollowerConfig.defaults()` uses
ROTATE and the constants `FIELD_LENGTH = 16.54`, `FIELD_WIDTH = 8.07` (public static; no `AprilTagFieldLayout` is loaded; `withField(length, width)` overrides
them). The 2027 field is unknown, so the default mode ROTATE is **Uncertain** (section 12); verify the field size against the official layout of the
season. `Path.flipped(FlipMode, fieldLength, fieldWidth)` and `Path.flipped(PathFollowerConfig)` return a flipped copy (a new object on every call; mode
NONE returns `this`); pass the blue path, because flipping a flipped path turns it back. There is no memo and no `forCurrentAlliance`: the command reads
`DriverStation.getAlliance()` once in `initialize()` and flips on red. MIRROR flips the sign of sigma, kappa, dphi, theta' and omega; ROTATE keeps them;
s values and L do not change. `NONE` is for a team that writes separate red paths.

### 9.10 FollowPathCommand

```java
public interface PathDrivetrain extends Subsystem {
    Pose2d getPose();                                // blue-origin field pose
    ChassisSpeeds getFieldSpeeds();                  // measured, field-relative, omega CCW
    void driveFieldRelative(ChassisSpeeds speeds);   // field-relative command, omega CCW
    default KinematicsConfig getKinematicsConfig() { return new KinematicsConfig(); }   // limits when the path has no constraints
    default PathsConfig getPathsConfig() { return PathsConfig.defaults(); }             // omega, alpha when the path has no constraints
}

public class FollowPathCommand extends Command {   // also a Sendable (initSendable adds the keys of 8.13)
    public FollowPathCommand(PathDrivetrain drivetrain, Path path);   // requires the drivetrain; null: NPE (V3)

    // configure before scheduling; each returns this; a change takes effect at the next initialize()
    public FollowPathCommand withConfig(PathFollowerConfig config);        // gains, tolerances, timeout, flip; default PathFollowerConfig.defaults()
    public FollowPathCommand withOnFinish(Consumer<Result> callback);

    @Override public void initialize();
    @Override public void execute();
    @Override public boolean isFinished();
    @Override public void end(boolean interrupted);
    @Override public void initSendable(SendableBuilder builder);

    public Result getResult();
    public double progress();            // 0 at the start to 1 at the end; never decreases during a run
    public double distanceTraveled();    // highest s* reached in this run [m]

    public enum Result {
        RUNNING,
        OK,                      // position and angle in tolerance for t_settle
        FAILED_TIMEOUT,          // t_timeout passed; zero speed sent once
        FAILED_START_TOO_FAR,    // more than startTol (0.5 m) from P0 at start
        FAILED_POSE,             // pose NaN or infinite for more than 1 s (or at the start)
        INTERRUPTED;             // cancelled by the scheduler or the driver
        public boolean isSuccess();          // this == OK
        public boolean isFailure();          // FAILED_*
    }
}
```

Rules:
- **Constraints:** `path.constraints()` if set; else `PathConstraints.fromConfig(drivetrain.getKinematicsConfig(), drivetrain.getPathsConfig())`, read in `initialize()`. No warning (C1 is removed).
- **Flip:** in `initialize()`, once: the alliance is read with `DriverStation.getAlliance()`; on red the command drives `path.flipped(config)`; empty gives blue with warning A2.
- **Requirements:** the drivetrain (it is a `Subsystem`); a null drivetrain is an NPE (V3), so the V24 case cannot happen.
- **Failure:** `end` sends zero speeds once (also on success) and prints the reason with `DriverStation.reportWarning` (9.12.3). There is no `holdEndCommand()`; schedule
  your own hold command after the path.
- **Output:** never negates omega, never discretizes.
- **Logging:** none built in. `SmartDashboard.putData("Path", follow)` (or any logger that takes a `Sendable`) shows the keys of section 8.13.
- **Package-private test hook** (not API): `withClock(DoubleSupplier)`.
- **Scheduling twice:** allowed (a fresh `initialize`); a command can still be in only one composition at a time.
- Not in v1: a re-plan hook (`withReplanHook`, result `FAILED_REPLAN`, section 8.9), starting at the current pose (resolution R5) and a
  mid-path start (section 8.2.5).

### 9.11 PathLogger and PathSample

> Removed in v1 (simplification); kept as v2 reference.

There is no `PathLogger`, `PathSample`, `PathWarnings` or `PathTelemetry` in v1. Warnings and failures use `DriverStation.reportWarning`; build warnings are
also in `Path.warnings()`; the telemetry is the command itself (a `Sendable`, section 9.10). `Path.sampleAt(s)` does not exist; `Path.polyline(spacingMeters)`
does. The v2 design was: a `PathLogger` interface copying RobotPose's `PoseLogger` (`putData(key, Sendable)`, `alert(text, type, active)`, a WPILIB default, a
static holder with `set`/`get`), `FollowPathCommand` publishing one `Sendable` under `Path/<name>`, and warnings as WPILib `Alert`s. A team that wants its own logger passes the command
to it: `logger.putData("Path", follow)`.

### 9.12 Validation

Principles: constructors throw for values that can never be right; the text says what is wrong, shows the value and says how to fix it (`{x}` is
filled at run time). `IllegalArgumentException` (IAE) for bad values, `NullPointerException` (NPE) for null. Everything the library can repair is
a warning. **Decision** (user, simplification pass): about ten exceptions for real user mistakes and about five build and start warnings; the
rest were merged or dropped. The types and wording are those of the code, except where a note below says otherwise.

#### 9.12.1 Exceptions

| ID | Condition | Thrown by | Type | Message |
|---|---|---|---|---|
| V1 | fewer than 2 points (also `new Path()`) | `Path` constructor | IAE | `Path needs at least 2 points (a start and an end), but you gave {n}. Example: new Path(new Point(1, 2), new Point(3, 4)).` |
| V3 | a null argument: the points array or a point, a `Translation2d` or `Pose2d`, an angle, a config, a supplier, the output or the path | the method that receives it | NPE | `{method}: {argument} is null; pass {what to pass} (for example {example}).` ({argument} is for example `point 2` or `the angle`; example: `new Path: point 1 is null; pass a Point for every position (for example new Point(1, 2)).`) |
| V4 | a number NaN or infinite: point x or y, an angle, a radius, a speed, a wait time | `Point` constructors and `with...` / `stop(d)` | IAE | `{method}({value}): {name} must be a finite number ({unit}), but it is NaN or infinite. A NaN usually comes from a division by zero or from a value that was never set.` |
| V6 | all points merge into one (every point within 1 mm of the first), so L = 0 | `Path` constructor | IAE | `All {n} points of the path are at the same place (within 1 mm), so the path has no length. Check the coordinates: two different points are needed, for example new Path(new Point(1, 2), new Point(3, 4)).` |
| V9 | `withRadius(r)` with r < 0 | `Point.withRadius` | IAE | `withRadius({r}): the radius must be 0 or more (meters). Use withRadius(0) to make the robot stop at this point, or leave withRadius out so the library chooses the radius, for example withRadius(0.5).` |
| V11 | `withVelocity(v)` with v < 0 | `Point.withVelocity` | IAE | `withVelocity({v}): the speed must be 0 or more (m/s). withVelocity(0) is the same as stop(). To limit the speed at a point use a positive number, for example withVelocity(1.0).` |
| V13 | `stop(d)` with d < 0 | `Point.stop` | IAE | `stop({d}): the wait time must be 0 or more (seconds). Use stop() to stop without waiting, or stop(1.5) to wait 1.5 seconds.` |
| V16 | a config value out of range: a `PathConstraints` field, a `PathsConfig` angular limit, or a `PathFollowerConfig` number not finite or not > 0 | the record constructors (one shared helper, `PathConstraints.requirePositive`) | IAE | `{Record}.{field} must be a finite number above 0 ({unit}), but it is {v}. Start from {Record factory} and change one value, for example {example}.` Example: `PathConstraints.vMax must be a finite number above 0 (m/s), but it is -1.0. Start from PathConstraints.fromConfig(config) and change one value, for example PathConstraints.fromConfig(config).withVMax(3.0).` The factory is `PathConstraints.fromConfig(config)`, `PathsConfig.defaults()` or `PathFollowerConfig.defaults()`. Null arguments of `PathConstraints.fromConfig` throw NPE (V3): `PathConstraints.fromConfig: the KinematicsConfig is null; pass your robot's config (for example PathConstraints.fromConfig(new KinematicsConfig()))`. |
| V24 | no drivetrain (null) | `FollowPathCommand` constructor | NPE | `FollowPathCommand: the drivetrain is null; pass your drivetrain subsystem, which implements PathDrivetrain (for example new FollowPathCommand(swerve, path)).` (same format as V3; replaces the old no-requirement error) |
| V30 | a point has `stop()`, `stop(d)` or `withVelocity(0)` **and** `withRadius(r)` with r > 0 | `Path` constructor | IAE | `Point {i} has both stop() and withRadius({r}). A stop point is a sharp corner (the robot stops exactly on the point), so it has no arc. Remove withRadius({r}) to stop on the point, or remove stop() to drive through the corner on a round arc.` If the stop and the radius come from two points merged (closer than 1 mm), the text is `Points {i} and {j} are less than 1 mm apart and were merged into one point, but point {a} has stop() and point {b} has withRadius({r}). ...` with the same advice. |

Notes: neighbors closer than 1 mm are merged with warning W3, not rejected; two equal points far apart in the list (a loop back to P_0) are legal.
Not errors, deliberately: a radius larger than fits (shrunk, W2). The IDs V2, V5, V7, V8, V10, V12, V14, V15,
V17, V18, V19 to V23 (events, removed in v1) and V25 to V29 of the earlier design are not used: they were merged into V3, V4, V16, or dropped.

#### 9.12.2 Build warnings (reported once from the `Path` constructor)

Format: `Path "{name}", point {i}: {text}`; each is printed once with `DriverStation.reportWarning(text, false)` and is in `Path.warnings()`. C1/C2-style limit warnings and the face-target part of W4 are removed in v1.

| ID | When | Text | Source |
|---|---|---|---|
| W1 | an automatic corner became a stop: turn above TAU_STOP, radius shrank below R_MIN, short leg `d/4 < R_MIN`, or `withRadius(r)` with 0 < r < R_MIN | `the corner is too sharp or too tight for an arc ({reason}). The robot will stop here. To drive through it with an arc, give a radius of at least {R_MIN} m, for example withRadius(0.3). To stop on purpose, use stop() or withRadius(0).` | 5.6.4, 5.7.3 |
| W2 | a radius was shrunk (automatic or override) | `radius {r0} m does not fit and was shrunk to {r1} m because {reasons}. Move the points farther apart, or give a smaller radius, for example withRadius({r1}).` {reasons} are joined with ` and `: `a neighbor point is too close to the circle`; `the line would run more than 30 degrees off the straight leg`; `the arc would turn much more than the corner angle`. (Checks F1 to F3.) | 5.7.3 |
| W3 | neighbor points closer than 1 mm | `points {i} and {j} are only {d} mm apart. Point {j} was merged into point {i}. Remove one of them from your list, for example delete point {j}.` Extra when both set an attribute: `both set {attribute} ({a} and {b}); using the later one ({b}).` | 5.8.2 |
| W4 | a setting that has no effect: `withVelocity(v > 0)` or `stop()` on P_0; `withVelocity` on the last point; `withRadius` on P_0, P_{n-1} or a straight point | `{setting} is ignored here: {reason}.` Texts: `stop() is ignored here: the robot starts with its measured speed. Remove stop() from this point.`; `withVelocity({v}) is ignored here: the robot starts with its measured speed. Remove withVelocity from this point.`; `withVelocity({v}) is ignored here: the robot always ends at speed 0 in this version. Remove withVelocity from the last point.`; `withRadius({r}) is ignored here: only a point where the path turns has a corner radius. Remove withRadius from this point.` For `stop(d)` on P_0, d > 0: `stop({d}) on the first point does not change the path; the robot waits {d} s before it starts. Remove the wait unless you want it.` | 5.7.3, 6.1, 7.4.2, 8.6 |
| W5 | a straight middle point is missed by more than PASS_TOL = 0.01 m | `this point is on a straight line between its neighbors, so it is only a pass marker, and the path misses it by {d} m (the line next to a round corner is turned outward). Add .stop() to drive exactly through it, or remove the point and use the command's progress() in a Trigger instead.` | 5.8.3 |

`stop()` on a corner that would have had an arc is not a warning: removing the arc is the documented meaning of `stop()`.

#### 9.12.3 Start warnings and run-time messages

v1 has no alert layer. Every message below goes through `DriverStation.reportWarning(text, false)`. The start warnings C1 and C2 of the earlier
design (no constraints; a limit above what has an effect) are removed: with no constraints the command uses `new KinematicsConfig()` and
`PathsConfig.defaults()` without a message.

| ID | When | Text |
|---|---|---|
| A1 | the command ended with a FAILED_* result (from `end`) | `Path "{name}": {result}: {reason}.` Reasons: FAILED_TIMEOUT `did not finish within {t} s ({factor} x the planned time + {margin} s); the robot was blocked, or the limits are above what the drivetrain can do; free the robot, or lower the limits, for example constraints.withVMax(2.0)`; FAILED_START_TOO_FAR `the robot is {d} m from the start point ({x}, {y}) and must start within {startTol} m (startTol); put the robot on the start point, check setStartPose and the alliance flip, or raise the tolerance, for example PathFollowerConfig.defaults().withStartTol(1.0)`; FAILED_POSE `the robot pose was NaN or infinite for more than 1 s; check that getPose() returns a real pose and that odometry keeps updating` |
| A2 | alliance unknown at initialize | `Path "{name}": alliance unknown; driving the blue path. For a red run, connect the Driver Station (or set the alliance in the simulator) before the command starts.` |
| A3 | `getFieldSpeeds()` gives NaN or infinity (once per run) | `Path "{name}": the measured speed is NaN or infinite; check getFieldSpeeds() of your drivetrain (PathDrivetrain) and that the gyro and encoders are connected.` |

### 9.13 Examples

All examples use the implemented API. Imports are omitted except in the first. The wiring example (f) is built into `ExampleSwerveWithPose` (`PathDrivetrain`, `follow(Path)`, `followCommand(Path)`, `examplePath()`).

**(a) Minimal 3-point path**

```java
import frc.robot.Path.Path;
import frc.robot.Path.Point;

Path path = new Path(new Point(1, 2), new Point(3, 4), new Point(5, 2));
```

The corner at (3, 4) gets an automatic radius of 1.4142 m; the path passes exactly through all three points; L = 5.9769 m, segments
1.5307 / 2.9155 / 1.5307 m (section 5.11). No warnings. Speed limits come from the command.

**(b) Headings on some points**

```java
Path path = new Path(
    new Point(1, 2).withAngle(Rotation2d.fromDegrees(0)),
    new Point(3, 4).withAngle(Rotation2d.fromDegrees(90)),
    new Point(5, 2));                       // no angle: keeps 90 degrees
```

The robot turns the short way between angles, in proportion to the distance along the path: 0 to 90 deg over the first 2.9885 m
(0.5256 rad/m), then holds 90 deg (section 6.11). 170 deg then -170 deg is a 20 deg turn, not 340. Without an angle on P_0 the path starts
from the robot's measured heading.

**(c) A slow corner with `withVelocity`**

```java
Path path = new Path(
    new Point(1, 2),
    new Point(3, 4).withVelocity(1.0),      // the whole arc at this corner at 1.0 m/s or less
    new Point(5, 2));
```

(3, 4) is a corner: `withVelocity(1.0)` caps the whole arc through it at 1.0 m/s (arcs are driven at one constant speed). The robot
brakes on the line before the arc and speeds up on the line after it. v1 profileTime: 5.43 s, against 3.44 s without the cap. On a
straight part (a pass marker) the cap acts at the point only: the robot brakes before it and speeds up after it. The earlier ARC_VAR design
(brake to the point inside the arc, section 7.8) would take about 4.06 s; it is not in v1. Zones with a length are v2.

**(c2) Stop points**

```java
Path path = new Path(
    new Point(1, 2),
    new Point(4, 2).stop(1.5),              // stop on this point, wait 1.5 s, then go
    new Point(7, 4).withAngle(Rotation2d.kPi));
```

- A stop point is a sharp corner: the path arrives on a straight line, stops exactly on (4, 2), and leaves on a straight line. `stop()` with
  `withRadius(r > 0)` on the same point is an error (V30). `withVelocity(0)` means the same as `stop()`.
- Order at a stop: arrive (within 0.03 m and below 0.1 m/s), correct a residual heading error of more than 3 deg,
  wait, leave under the speed law (section 8.6). The heading here goes linearly from the measured start heading to 180 deg at (7, 4) (an exact
  half turn from 0 deg turns CCW, section 6.3); it does not jump at the stop.
- With EX and a start heading of 0 (script): legs 3.0 m and 3.6056 m, trapezoid motion 2.0000 + 2.1926 s, dwell
  1.5 s: 5.6926 s (the dwell is the only stop term, section 7.10.4). The v1 profileTime adds the hand-over tail (0.1703 s with the
  default k_p and pos_tol) per goal.
- At the last point the stop is automatic; `.stop(1.0)` there holds 1 s after the finish test passes.

**(d) Face target** (removed in v1; kept as v2 reference)

Not in v1: `Path.withFaceTarget` does not exist. See section 6.9 for the design.

**(e) Progress trigger instead of events**

```java
FollowPathCommand follow = swerve.followCommand(new Path(new Point(1, 1), new Point(4, 1), new Point(7, 3)));
new Trigger(() -> follow.progress() > 0.2).onTrue(intake.deployCommand());   // after 20% of the path
new Trigger(() -> follow.distanceTraveled() > 3.0).onTrue(Commands.print("3 m")); // after 3 m along the path
SmartDashboard.putData("Path", follow);                                     // state, segment, s, eN, vRef, vMeas, eThetaDeg
```

`progress()` is 0 at the start and 1 at the end and never decreases during a run, so each trigger fires once. The triggered command must
not require the drivetrain. Create the trigger from the same command object that you schedule.

**(f) Wiring into a drivetrain with the Kinematics module**

Based on `ExampleSwerveWithPose.java` (**Fact** for every cited line; the example is not used by the robot):

| PathDrivetrain method | In the example |
|---|---|
| `getPose()` | blue-alliance origin |
| `driveFieldRelative(ChassisSpeeds)` | `driveFieldRelative(ChassisSpeeds wanted)`: desaturate, limiter, `drive`, remembers `lastFieldSpeeds` |
| `getFieldSpeeds()` | `getFieldSpeeds()` (added for Path; `periodic()` now calls it) |
| Limiter re-sync when the path starts | `periodic()` does it when the last loop did not use `driveFieldRelative` |
| Start pose for the auto | `setStartPose(Pose2d)` |

`getGyroRate()` and `heading()` are private, so the drivetrain class needs one public method (it is in `ExampleSwerveWithPose`; `follow(Path)` there is the `follow` below):

```java
/** Measured field-relative speeds [m/s, rad/s], omega CCW, blue-origin field frame. For FollowPathCommand. */
public ChassisSpeeds getFieldSpeeds() {
    ChassisSpeeds measured = kinematics.toChassisSpeeds(getModuleStates(), getGyroRate());  // as in periodic() before this method existed
    return ChassisSpeeds.fromRobotRelativeSpeeds(measured, heading());
}
```

In `ExampleSwerveWithPose` (the code is the reference):

```java
private final PathFollowerConfig followerConfig = PathFollowerConfig.defaults();   // ROTATE flip, field 16.54 x 8.07
// ExampleSwerveWithPose implements PathDrivetrain; pathsConfig = PathsConfig.defaults() (omega, alpha placeholders)

public FollowPathCommand followCommand(Path path) {
    return new FollowPathCommand(this, path).withConfig(followerConfig);
}

@Override public KinematicsConfig getKinematicsConfig() { return kinematicsConfig; }
@Override public PathsConfig getPathsConfig() { return pathsConfig; }

public Command follow(Path path) {
    return Commands.defer(() -> {
        Path active = isRedAlliance() ? path.flipped(followerConfig) : path;   // red: the flipped path (start pose)
        Pose2d start = new Pose2d(active.startPoint(), active.startHeading().orElse(getPose().getRotation()));
        return Commands.runOnce(() -> setStartPose(start)).andThen(followCommand(path));
    }, Set.of(this));
}
```

Notes: `getPose()` must return the **blue-origin** pose also on red (RobotPose.java:267 documents `getEstimatedPose` that way).
`driveFieldRelative` is the only place where omega is turned to clockwise-positive (`drive`, :287-297); the follower does not negate it.
`Commands.defer` reads the alliance when autonomous starts, because it can change before (**Inference**). `driveFieldRelative` includes the
acceleration limiter, which the follower expects (section 7.14).

Tests: section 13.5.

## 10. Defaults

Every constant of v1, with its single home. Rows for removed features (moduleRadius, ARC_RHO, FT_SPAN, r_face_min, the jerk warning threshold, the motor-curve inversion, the stale-pose gate; with the jerk limit: maxJerk, STOP_DIST_MARGIN, D_H_MAX, the room-to-brake bisection and the offline profileTime guard) were deleted; `kTheta`, `tauLag` and `tSettle` are constants of `PathFollowerConfig`, not settings. "Home" is the record field (configurable) or the internal class that owns the
constant: an **internal constant** is a documented `static final`, not configurable (**Decision**, user, simplification pass: only
`PathConstraints`, `PathsConfig` and `PathFollowerConfig` are public config types). All values are **Recommendation** starting values,
reasoned and simulated in Python only, not tuned on a robot, unless marked **Fact** or **Decision**.

| Name | Value | Unit | Home | Used in | Notes |
|---|---|---|---|---|---|
| vMax | `0.9·maxAllowedModuleVelocity` = 4.05 | m/s | `PathConstraints.vMax` (via `fromConfig`) | 7.2.1 | from `new KinematicsConfig()` |
| aAcc | `0.9·min(maxForwardAccel, maxSkidAccel, maxTiltAccelFront, maxTiltAccelSide)` = 7.2 | m/s^2 | `PathConstraints.aAcc` | 7.2.1 | |
| aDec | `0.9·min(maxSkidAccel, maxTiltAccelFront, maxTiltAccelSide)` = 7.2 | m/s^2 | `PathConstraints.aDec` | 7.2.1 | |
| aLat | same as aDec = 7.2 | m/s^2 | `PathConstraints.aLat` | 7.2.1 | |
| maxAngularVelocity | 2·pi | rad/s | `PathsConfig.maxAngularVelocity` | 7.2.1, 9.6 | placeholder; measure (section 12) |
| maxAngularAccel | 4·pi | rad/s^2 | `PathsConfig.maxAngularAccel` | 7.2.1, 9.6 | placeholder |
| | PROFILE_MARGIN | 0.9 | - | `PathConstraints.PROFILE_MARGIN` (public constant) | 7.2.1 | v1 has no motor curve or wheel cap |
| HAND_OVER (d_h) | 0.05 (fixed; v_h = k_p·d_h) | m | `SpeedProfile.HAND_OVER` (internal constant) | 8.3.7 | hand-over distance to the 2D capture law |
| REST_HAND_OVER_MARGIN | 0.01 | m | `SpeedProfile.REST_HAND_OVER_MARGIN` (internal constant) | 8.3.7e | at rest the hand-over band is d_h + this |
| R_MIN | 0.05 | m | `PathGeometry` (internal constant) | 5.7.3 | **Decision** (brief decision 2) |
| R_MAX | 2.0 | m | `PathGeometry` (internal constant) | 5.7.2 | |
| D_BULGE | 0.5 | m | `PathGeometry` (internal constant) | 5.7.2 | accepted by the user |
| EPS_DEV | 30 deg | rad | `PathGeometry` (internal constant) | 5.7.1 (F2, F3) | accepted by the user |
| TAU_STOP | 150 deg | rad | `PathGeometry` (internal constant) | 5.6.4 | accepted by the user; `withRadius` bypasses it |
| SHRINK | 0.8 | - | `PathGeometry` (internal constant) | 5.7.3 | |
| MAX_ITER | 1000 | passes | `PathGeometry` (internal constant) | 5.7.3 | |
| PASS_TOL | 0.01 | m | `PathGeometry` (internal constant) | 5.8.3 (W5) | largest miss of a straight point without a warning |
| EPS_DUP | 1e-3 | m | `PathGeometry` (internal constant) | 5.8.2 | merge distance |
| EPS_ANG | 1e-4 | rad | `PathGeometry` (internal constant) | 5.3, 5.8.3 | straight-point threshold |
| EPS_LEN | 1e-6 | m | `PathGeometry` (internal constant) | 5.4 | tangent existence, zero-length segments |
| EPS_WRAP | 1e-9 | rad | `PathGeometry` (internal constant) | 5.5 | sweep wrap guard |
| EPS_CENTER | 1e-9 | m | `PathGeometry` (internal constant) | 5.10.2 | robot at an arc center |
| radius divisors k | 2 (end leg or user-stop leg), 4 (shared leg) | - | `PathGeometry` (internal constant) | 5.7.2 | |
| projection tie | 1e-12 | m | `PathGeometry` (internal constant) | 5.10.4 | |
| half-turn tie | 1e-9 (a change within this of pi turns CCW) | rad | `PathGeometry` (internal constant) | 6.3 | |
| HEADING_LAG_ALLOWED (e_allow) | 5 deg = 0.0873 | rad | `SpeedProfile` (internal constant) | 6.6, 7.4.6 | |
| S_EPS | 1e-6 | m | `SpeedProfile` (internal constant) | 7.3.1 | span merge |
| DEFAULT_DS | 0.02 | m | grid test oracle only (test code) | Appendix B | not in production code |
| dt_nom | 0.02 | s | `FollowPathCommand` (internal constant) | 8.2.2 | the only home of the nominal loop period; equals `KinematicsConfig.cycleDt` (**Fact** :25). Run time uses the measured dt |
| k_n | 3.0 | 1/s | `PathFollowerConfig.kN` | 8.3.5 | |
| k_p | 3.0 (= k_n) | 1/s | `PathFollowerConfig.kP` | 8.3.7 | keep equal to k_n |
| tau_lag | 0.05 | s | `PathFollowerConfig.TAU_LAG` (constant, not a setting) | 8.4 | tuning: edit the constant |
| k_theta | 4.0 | 1/s | `PathFollowerConfig.K_THETA` (constant, not a setting) | 8.5.2 | |
| pos_tol | 0.03 | m | `PathFollowerConfig.posTol` | 8.6, 8.7.2 | **Decision** (brief decision 10); also the stop-arrival tolerance (along T_in) |
| ang_tol | 2 deg | rad | `PathFollowerConfig.angTolRad` | 8.7.2 | **Decision** |
| t_settle | 0.1 | s | `PathFollowerConfig.T_SETTLE` (constant, not a setting) | 8.7.2 | **Decision** |
| startTol | 0.5 | m | `PathFollowerConfig.startTol` | 8.2.5 | **Decision** (simplification pass); FAILED_START_TOO_FAR above it |
| timeout | `1.5·profileTime + 1.0` | s | `PathFollowerConfig.timeoutFactor` = 1.5, `timeoutMargin` = 1.0 | 7.10.5, 8.7.3 | **Decision** (brief decision 10) |
| flip mode | ROTATE | - | `PathFollowerConfig.flipMode` | 5.13, 9.9 | **Uncertain** for 2027 |
| field size | 16.54 x 8.07 constants `FIELD_LENGTH`, `FIELD_WIDTH` (no AprilTag loading; `withField` overrides) | m | `PathFollowerConfig.fieldLength`, `fieldWidth` | 5.13 | verify against the season's official layout |
| c_max | 1.0 | m/s | `FollowPathCommand` (internal constant) | 8.3.8 | |
| ahead_min | 0.15 | m | `FollowPathCommand` (internal constant) | 8.2.3 | |
| k_ahead | 3 | loops | `FollowPathCommand` (internal constant) | 8.2.3 | |
| ahead_max | 0.50 | m | `FollowPathCommand` (internal constant) | 8.2.3 | |
| eps_back | 0.03 | m | `FollowPathCommand` (internal constant) | 8.2.4 | |
| STOP_SPEED | 0.1 | m/s | `FollowPathCommand` (internal constant) | 7.10.3, 8.6 | the only home of the stop speed |
| stop_ang_tol | 3 deg | rad | `FollowPathCommand` (internal constant) | 6.7, 8.6 | ROTATE entry and exit (resolution R1) |
| L_short | 0.10 | m | `FollowPathCommand` (internal constant) | 8.12 | |
| t_stale | 0.10 | s | `FollowPathCommand` (internal constant) | 8.12 | |
| dt clamp | [0.005, 0.10] | s | `FollowPathCommand` (internal constant) | 8.2.2 | |
| arc window cap | `max(0.05, (pi/2)·r_j)` beyond the arc start | m | `FollowPathCommand` (internal constant) | 8.2.3 | |
| pose invalid limit | 1.0 (then FAILED_POSE) | s | `FollowPathCommand` (internal constant) | 8.12 | |
| user stop dwell | 0 | s | `Point.stop(d)` | 8.6 | |
| e_replan, t_replan | 0.50, 0.5 (1.0 for STUCK) | m, s | v2 (future `PathFollowerConfig` fields) | 8.9 | not in v1 |
| re-plan e_t trigger, stuck test, re-arm | 0.15 m; v_t > 0.5 and ds*/dt < 0.1 m/s; abs(e_n) < e_replan/2 for 0.5 s | - | v2 | 8.9 | not in v1 |

## 11. v1 and v2 scope

| Feature | v1 | v2 or later | Source |
|---|---|---|---|
| Waypoints exactly on the path, line + arc (G1) | yes | - | decisions 1, 3 |
| Automatic radius with bulge cap, TAU_STOP stops, shrink loop | yes | per-corner arc-or-stop choice by profile time | decision 14, 5.6.4 |
| Straight middle points | PASS markers, warning W5 above 0.01 m (no kink corners) | - | simplification pass, 5.8.3 |
| Per-point `withRadius` | yes | - | decision 2 |
| `cutCorner()` | no | classic fillet that misses the point | decision 1 |
| Clothoid transitions (G2) | no | if the logged criterion fails (median dE > 0.03 m over >= 20 entries) | decisions 3, 14, 5.12.3 |
| Per-point speed cap `withVelocity(v > 0)` | yes; on a corner it caps the whole arc (constant speed); on a straight part it acts at the point | variable speed inside the arc (ARC_VAR, 7.8) | decision 5, 7.4.2 |
| Speed zones over an s range | no | yes | decision 5 |
| Stops at any middle point: `stop()`, `stop(d)`, `withVelocity(0)` | yes | - | decisions 15, 17 |
| Turn in place at a stop (a heading step) | no: heading continuous, ROTATE corrects residuals only | candidate | resolution R1 |
| Non-zero end speed, chaining paths | no (end speed 0; `withVelocity` on the last point ignored) | yes | decision 5, 7.4.7 |
| Heading anchors (linear in s) | yes | face target (6.9), also for an s range | decision 4, resolution R1, simplification pass |
| Speed profile | per-span closed form with constant acceleration limits, constant speed on every arc (a `withVelocity` point caps the whole arc), junction passes | motor-curve speed-up (7.9), wheel-speed cap (7.11), ARC_VAR (7.8), speed zones, clothoid sub-spans | decision 16, resolution R10, simplification pass |
| Runtime speed command | plain online trapezoid `min(vBack(s_v), v_prev + aAccEff·dt)` with lag lead and hand-over envelope; no jerk limit | jerk-aware law (section 7.13, tangential jerk); lateral jerk at arc joints via clothoids | decision 18, user decision 2026-11-03 |
| Events at a fraction of the path or a waypoint index | no: use `cmd.progress()` / `distanceTraveled()` with a `Trigger` | `PathEvent` (8.8, 9.7) | decision 7, simplification pass |
| Alliance flip NONE / MIRROR / ROTATE | yes | - | decision 9, resolution R6 |
| Finish 0.03 m, 2 deg, 0.1 s; timeout; `FollowPathCommand.Result` | yes | - | decision 10 |
| `holdEndCommand()` on the command | no: schedule your own hold command | yes | resolution R4 |
| Face target, alert/logger layer (`PathLogger`, `PathWarnings`, `PathTelemetry`), C1/C2 warnings, flipped-copy memo, `forCurrentAlliance`, AprilTag field loading | no: the command is a `Sendable`, warnings go to `DriverStation.reportWarning`, the field size is a constant | yes (sections 6.9, 8.8, 9.7, 9.8, 9.11) | simplification 2026-11-03 |
| Mid-path start | no: the robot starts within 0.5 m of P0 | candidate | simplification pass (replaces resolution R5) |
| Settings | three public config records; the rest internal constants (`kTheta`, `tauLag`, `tSettle` are constants too) | more knobs only if testing shows a need | simplification pass |
| Re-plan hook | no (design in 8.9) | callback, `FAILED_REPLAN` | decision 8, resolution R5 |
| Start at the current pose (rebuild geometry) | no | not planned (no on-the-fly paths) | decision 8, resolution R5 |
| On-the-fly paths | no | not planned | decision 8 |
| Obstacle avoidance | no | yes | decision 8 |
| Java API | yes | - | decision 11 |
| JSON | a sketch only (Appendix A), not v1 | schema, reader and writer for a path editor | decision 11, simplification pass |
| Changes to the Kinematics module | none | none; the missing settings are in `PathsConfig` (section 9.6; no `moduleRadius` in v1) | user decision |

## 12. Open items for the user

Only the decisions that remain, and the values that must be measured. Everything else in this document is decided.

1. **Robot settings outside the Kinematics module** (**Decision**, user: `KinematicsConfig` is not changed). They live in `PathsConfig`
   (section 9.6): `maxAngularVelocity` (2·pi), `maxAngularAccel` (4·pi), `maxJerk` (removed in v1 with the jerk limit) and `moduleRadius` (removed in v1; R_mod; the module positions
   are private in `DemaciaKinematics`, DemaciaKinematics.java:58, so `PathsConfig.fromModules` takes the same array the drivetrain uses).
   Nothing to decide; the values must be measured (item 2).
2. **Values to measure on the robot** (none can be decided on paper):
   - **tau_d**, the drivetrain lag (0.085 s modelled, **Uncertain**): fit the line step (section 8.13). It sets k_n and k_p (raise both to 4 if
     tau_d allows; the hand-over tail is `ln(d_h/0.03)/k_p`, section 8.3.7) and tau_lag (rule in section 8.4).
   - **`PathsConfig` values**: maxAngularVelocity and maxAngularAccel (placeholders 2·pi and 4·pi).
   - The `KinematicsConfig` limits themselves (placeholders, KinematicsConfig.java:8-15), from which aAcc, aDec and aLat follow.
   - Whether omega_ff from the commanded speed or from a filtered ds*/dt tracks better (test FOL-H4).
   - The clothoid criterion (section 5.12.3) from match logs, which decides G2 for v2.
3. **Default flip mode and field size for 2027.** The default is ROTATE (**Uncertain**: the 2027 field is unknown). Set the mode once per robot
   (`PathFollowerConfig.withFlip`) when the field is announced, and check the field size of `PathFollowerConfig.defaults()` against the official
   layout; the examples' 16.54 x 8.07 m is a placeholder.
4. **Big-loop checks: decided, F2 and F3 are kept** (**Decision**, user). Removing the 30 deg line check (F2) and the sweep check (F3) would change
   5.76% of random field-sized paths even with the bulge cap and TAU_STOP (section 5.7.3; sweeps up to 709 deg and 45 self-crossing corners
   without them, script). The sweep is about tau_turn + 2·(line tilt), and the line is fixed once the circle is fixed (it starts at the previous
   point and is tangent to the circle), so limiting the tilt means shrinking the radius, which is what F2 and F3 do. No longer open.

**Uncertain** items that need no decision but should be watched in testing: the time cost of a heading-kink cap on an arc (the
whole arc) and of the ARC_VAR share (section 7.8), the one-loop quantization of a = 0 at arc entries (section 7.8), the G1 toy model
(section 5.12.1), the duration of ROTATE (section 8.6), and WPILib `angleModulus` at exactly +-pi (implemented explicitly, section 6.3).

## 13. Test cases

> The IDs below are the test plan of the earlier design. Cases for removed features (face target, events, `PathLogger`, `holdEndCommand`, ARC_VAR, motor curve, wheel cap, C1/C2) are historical v2 reference; the v1 JUnit tests (not committed) cover the 8-file package.

Inputs, then expected outputs. Not committed as code. Tolerance 1e-4 unless stated; "auto" means default config; "shared example" is
P0 = (1,2), P1 = (3,4), P2 = (5,2); EX and DEF as in section 3. IDs of tests removed in the simplification pass are not reused, so the
gaps are intentional.

### 13.1 Geometry

| ID | Input | Expected |
|---|---|---|
| GEO-1 | shared example, auto | r = 1.4142, C = (3, 2.5858), lengths 1.5307 / 2.9155 / 1.5307, L = 5.9769, kappa = 0 / -0.7071 / 0, dphi = -118.12 deg, s(P1) = 2.9885 |
| GEO-2 | same, evaluate(3.0); evaluate(2.9885) | x = (3.0115, 4.0000), T = (1.0000, -0.0082), kappa = -0.7071; x = (3, 4), T = (1, 0) |
| GEO-3 | same, project((2,3)) on line 0; project((3,4.5)) on the arc | s* = 1.3719, e_n = -0.3435; s* = 2.9885, e_n = +0.5000 |
| GEO-4 | same, project(C exactly, s_prev = 2.5) | s* = 2.5, e_n = -1.4142, no NaN |
| GEO-5 | same, segmentIndexAt | s = 3.0 gives 1; 4.4462 gives 2; -1 gives 0; 99 gives 2 (clamped) |
| GEO-6 | (0,0),(4,0),(4,4),(8,4), radius 1 at both | first tangent point (3.2031, -0.2889), arc 1 dphi = +109.21 deg, middle line (4.2629, 0.95) to (3.7371, 3.05), length 2.1648, L = 12.4092 |
| GEO-7 | (0,0),(4,0),(4,4),(0,4), radius 1 at both; radii 1.0 / 0.5 | equal radii: middle line x = 4.2929 from y = 0.7071 to 3.2929, L = 12.3394; radii 1.0 / 0.5: middle line (4.2916, 0.7571) to (4.1458, 3.6714), L = 12.2546 |
| GEO-8 | (0,0),(3,4) | one LINE, L = 5, T = (0.6, 0.8) |
| GEO-9 | (0,0),(2,0),(2.0005,0),(2,2) | same output as (0,0),(2,0),(2,2), L = 4.2263, warning W3 |
| GEO-10 | (0,0),(2,0),(4,0) | one LINE, L = 4; PASS marker at s = 2.0, miss 0, no warning, no stop |
| GEO-11 | (0,0),(1,0),(2,0),(2,2) | (1,0) is a PASS marker; the path equals that of (0,0),(2,0),(2,2) (r = 1.0, L = 4.2263); miss 0.2429 m at s = 0.9700: warning W5 naming point 1 |
| GEO-12 | (0,0),(3,0),(3,3),(3,6),(6,6) | (3,3) set aside; r_1 = r_3 = 1.5; lines 1.6236 / 3.2472 / 1.6236 m, arcs 3.0923 m (sweep 118.12 deg); the inner tangent passes exactly through (3,3): PASS marker at s(P2) = 6.3395, miss 0, no warning; L = 12.6790 |
| GEO-12b | (0,0),(3,0),(3,3),(3,6),(0,6); then with `(3,3).stop()` | middle line x = 3.4393, (3,3) missed by 0.4393 m: warning W5, geometry unchanged; with `.stop()` (3,3) is a STOP on the path |
| GEO-13 | (0,0),(4,0),(0,0), auto; with `.withRadius(0.5)` | STOP at (4,0), L = 8.0; arc C = (3.5, 0), dphi = +196.43 deg, L = 8.6423 |
| GEO-14 | (0,0),(4,0),(2,0) with `.withRadius(0.5)`; robot (2, 0.05), window [1.97, 2.15] | dphi = +207.68 deg, arc length 1.8124, L = 6.6907; s* = 1.9723, e_n = +0.3352 (the global search would give 6.6740) |
| GEO-15 | (0,0),(4,0),(1,0),(1,2) with `.withRadius(0.5)` on P1 | sigma_1 = -1 (copied from the right turn at P2) |
| GEO-16 | corner of 170 deg, legs 4, auto; with `.withRadius(0.5477)` | STOP, warning W1; an arc (withRadius bypasses TAU_STOP), sweep 186.7 deg, bulge 0.5000 |
| GEO-17 | corner of 120 deg, legs 4, auto | r = 1.0000, sweep 138.6 deg, line deviation 9.3 deg |
| GEO-18 | (0,0),(2,0),(2,2) with `.withRadius(5)` | shrunk to 1.3107 after 7 passes, warning W2 |
| GEO-19 | shared example with `.withRadius(0)` on P1 | polyline, STOP at (3,4), L = 5.6569 |
| GEO-20 | (0,0),(4,0),(4.1,0.1),(8,0) | both corners STOP (d/4 = 0.035 < R_MIN), L = 8.0427 |
| GEO-21 | (0,0),(4,0),(4,1),(8,1), radius 1.0 at both; auto | shrunk to 0.3277 after 6 passes; auto r = 0.25, L = 9.0971 |
| GEO-22 | (0,0),(4,0),(4,4),(0,4), radius 2.0 at both | accepted, middle line x = 4.5858, length 1.1716 |
| GEO-23 | GEO-1 under MIRROR, FL = 16.54 (placeholder); under ROTATE, FW = 8.07 | C = (13.54, 2.5858), dphi = +118.12 deg, kappa = +0.7071, same s table; C = (13.54, 5.4842), dphi = -118.12 deg |
| GEO-24 | property: random paths of 3 to 7 points on a 16 x 8 field | every build terminates, F1 to F3 hold, each turning waypoint and stop within 1e-9 m of the path at its s, position and T continuous across every non-STOP joint within 1e-9 |
| GEO-25 | property: random triangles and radii | `d > 2r·sin(tau_turn/2)` agrees with `abs(P0 - C) > r` |
| GEO-26 | property: every arc | `x(s_wp) = P_i` and `abs(dphi) = l/r` |
| GEO-27 | shared example with `P1.stop()` (and with `withVelocity(0)`); (0,0),(4,0),(4,4).stop(),(0,4) | LINE 2.8284, STOP at 2.8284, LINE 2.8284, L = 5.6569, s(P1) = 2.8284, same table for both; r_1 = 1.7071, STOP at s = 8.3596, L = 12.3596 |
| GEO-28 | (0,0),(2,0).stop(),(4,0); (0,0),(1,0).stop(),(2,0),(2,2) | two LINEs and a STOP (not a PASS) at s = 2, L = 4; (1,0) is a STOP, the line to circle 2 heads -14.06 deg, r_2 = 0.5000 (both legs k = 2), L = 4.1023 |
| GEO-29 | shared example, `P1.stop().withRadius(0.5)`; `P1.withRadius(0.5).stop()`; `P1.stop().withRadius(0)` | IAE V30 (both orders); the last is accepted, same as GEO-27 |
| GEO-30 | `withRadius(0.5)` on P0, on P2, and on the straight point of GEO-10 | ignored, warning W4; geometry unchanged |
| GEO-31 | property: random paths with default TAU_STOP | no crossing of an arc's in-line and out-line (0 in 5,000 paths in the script); in a test build with TAU_STOP = pi crossings exist and the build still succeeds |
| GEO-32 | shared example: event at fraction 0.5, at point 1, at point 2 | s = 2.98845, 2.9885, 5.9769 |
| GEO-33 | GEO-27, evaluate and evaluateLeft at s = 2.8284 | T = (0.7071, -0.7071) (right limit) and (0.7071, 0.7071) (left limit) |
| GEO-34 | all points equal, or (0,0),(0.0004,0) | IAE V6 |

### 13.2 Heading profile

| ID | Input | Expected |
|---|---|---|
| HEAD-1 | shared example, theta0 = 0, P1 90 deg | theta' = 0.5256 on [0, 2.9885], 0 after; theta(1.0) = 30.12 deg, theta(1.5307) = 46.10 deg, theta(2.9885) = 90 deg, theta(5.9769) = 90 deg |
| HEAD-2 | HEAD-1 spans | span theta' = [0.5256, 0.5256, 0, 0] |
| HEAD-3 | HEAD-1 omega_ff with vProf | 1.2875 rad/s at s = 1.0; 1.4318 at the peak (s = 1.2368); 1.2501 on span 1; 0 after P1 |
| HEAD-4 | HEAD-1 caps | rotation cap 11.95 m/s; alpha-acceleration cap 23.9 m/s^2; heading-kink cap at P1 2.818 m/s; trapezoid time 3.2723 s and v1 time 3.54 s, both unchanged by the heading |
| HEAD-5 | anchors 170 deg at s = 1 and -170 deg at s = 3 | +20 deg (CCW), theta' = 0.1745 rad/m, stored end value 190 deg |
| HEAD-6 | theta0 = 0, P1 at 180 deg | exactly half a turn: CCW (+pi); theta' = 1.0512; kink cap 1.4088 binds; the whole arc at 1.4088; trapezoid time 4.2812 s, v1 time 4.54 s |
| HEAD-7 | no anchors, theta0 = 0.3 rad | theta(s) = 0.3 for all s, theta' = 0 |
| HEAD-8 | P0.withAngle(0.5 rad), theta0 = 0 | theta(0) = 0.5 (no step inside theta(s)); the robot's error is removed by feedback |
| HEAD-9 | theta0 = 10 deg, shared example | linear 10 deg -> 90 deg over [0, 2.9885], theta' = 0.4672 rad/m |
| HEAD-11 | shared example with P1.stop() | theta' = 0.5554 before the stop, 0 after; ROTATE skipped when the residual is at most 3 deg |
| HEAD-12 | face target (8, 4), robot (1.7028, 3.3099), V = (1.4972, 1.8509) | theta_ref = 0.1091 rad, omega_ff = -0.2647 rad/s, equal to the finite difference within 1e-6 |
| HEAD-13 | face target, straight path passing 1.0 m from it | theta'_max = 1.0, theta''_max = 0.6495, vCap near the target 3.110 m/s, aA cap 6.28 m/s^2 |
| HEAD-14 | face target, path passing 0.5 m from it | freeze inside 0.5 m (omega_ff = 0, theta_ref frozen); kink cap 0.740 m/s at the freeze boundary; no warning |
| HEAD-15 | anchors and face target both set | anchors ignored, warning W4 |
| HEAD-16 | HEAD-1 under MIRROR; HEAD-6 under MIRROR | anchor 90 deg stays 90 deg, theta' sign follows the flipped path; the 180 deg change still turns CCW (documented, not a mirror image) |

### 13.3 Speed profile

| ID | Input | Expected |
|---|---|---|
| PROF-1 | straight (0,0)->(4,0), vMax 4, aAcc = aDec = 3, v0 = 0 | one span; peak `sqrt(3·4) = 3.464` at s = 2; time 2.309 s |
| PROF-2 | straight (0,0)->(10,0), vMax 3, aAcc = aDec = 3 | cruise 3 on [1.5, 8.5]; time `1 + 7/3 + 1 = 4.333 s` |
| PROF-3 | shared example, EX, span profile | spans [1.5307, 1.4578, 1.4577, 1.5307]; vB = [3.8524, 2.3784, 2.3784, 2.3784, 0]; trapezoid peaks 2.7241 at s = 1.2368 and 4.7401; trapezoid time 3.2723 s (no jerk limit) |
| PROF-4 | span method vs grid oracle (ds 0.005, Appendix B), paths where every arc is driven at its curvature cap | times equal within 1e-3 s |
| PROF-5 | span method vs grid, any path | span time >= grid time - 1e-3 s |
| PROF-6 | any path, any s | vProf <= vCap; `vProf^2·abs(kappa) <= aLat + 1e-9`; vProf constant on every constant-speed arc (a_t = 0 there); on ARC_VAR spans `vProf^2·abs(kappa) <= rho_lat·aLat + 1e-9` and `(a_t/aT)^2 + (a_n/aLat)^2 <= 1 + 1e-9` |
| PROF-7 | span boundaries | every segment start, waypoint s and stop s is a boundary (abs(diff) < 1e-9) |
| PROF-8 | naive-trapezoid trap: a 0.3 m span entered at 3 m/s must end at 0, aDec 3 | vB at its entry = 1.3416; the previous span brakes 3 -> 1.3416 over 1.2 m |
| PROF-9 | start stall: v0 = 0, ideal robot (s += v_cmd·dt) | leaves s = 0; v_cmd = 0.06, 0.12, 0.18 m/s after 1, 2, 3 loops (aAcc 3) |
| PROF-12 | sideways start V_meas = (0.5, 1.0), T(0) = (1, 0) | v0 = 0.5; no warning |
| PROF-13 | V_meas = (-1, 0), T(0) = (1, 0) | v0 = 0 |
| PROF-14 | shared example + P1.withVelocity(1.0), span method | both arc spans ARC_VAR: cap 2.1273, aA = aD = 1.8; junctions vF = [0, 2.1273, 1.0, 2.1273, 0]; braking for the arc from s = 1.1425 (peak 2.6182), for P1 from s = 2.0092; total 3.7751 s; v1 time 4.06 s, timeout 7.09 s |
| PROF-15 | same, grid oracle | 3.5770 s |
| PROF-16 | `withVelocity(0)` on P1 | identical to P1.stop() |
| PROF-17 | `withVelocity(-1)` | IAE V11 |
| PROF-18 | P1.stop() on the shared example | two sub-profiles; trapezoid peaks 2.9130 at s = 1.4142 and 4.2426; trapezoid motion 3.8839 s; v1 profileTime 4.2245 s (+ 2 tails), timeout 7.34 s |
| PROF-19 | P1.stop(0.5); P1.stop(1.5) | v1 profileTime 4.7245 s (timeout 8.09 s); 5.7245 s (timeout 9.59 s) |
| PROF-20 | stop restart: sub-profile 2 of P1.stop() | vBack(2.8284) = 4.0 (> 0); law restarts from 0: 0.06, 0.12, 0.18 m/s after release |
| PROF-21 | wheel cap: theta' = 1 rad/m, R_mod = 0.3889, vWheelMax 4.5, margin 0.9 | cap = 4.05/1.3889 = 2.916 |
| PROF-22 | heading 170 deg -> -170 deg over 2 m | see HEAD-5 |
| PROF-23 | heading 0 -> 180 deg at P1, alphaMax 4·pi, e_allow 5 deg | J at P1 = 1.4088 (kink cap); the whole arc at 1.4088, junctions [0, 1.4088, 1.4088, 1.4088, 0]; 4.2812 s (grid oracle 3.4372 s) |
| PROF-24 | motor curve a_0 = 9, v_free = 5.3, 0 -> 2 m/s | ds = 0.3010 m, dt = 0.2790 s; inverting ds = 0.3010 gives 2.000 |
| PROF-25 | `min(7.2, 9·(1 - v/5.3))`, 0 -> 2 m/s | knee 1.06; ds = 0.3068 m, dt = 0.2948 s |
| PROF-26 | shared example geometry, DEF + motor curve, **no heading anchors** (theta0 = 0, no `withAngle`) | span = grid = 2.3101 s; peak 3.7094; arc 3.1910 |
| PROF-26b | shared example as in section 3 (P1 `withAngle(90 deg)`), DEF + motor curve | kink cap 2.8176 at P1; the whole arc at 2.8176; junctions [0, 2.8176, 2.8176, 2.8176, 0]; span time 2.4549 s; v1 time 2.88 s |
| PROF-27 | tight arc (0,0), (4,0).withRadius(0.3), (4,4), EX | spans [3.7819, 0.2426, 0.2426, 3.7819]; arc 1.0954; 4.3210 s (span = grid) |
| PROF-29 | PathConstraints with aLat = 0 or NaN | IAE V16 |
| PROF-30 | limiter pass-through: run the profile through `SwerveAccelerationLimiter` with the default config (simulation) | output equals input every loop (difference < 1e-9) on the shared example |
| PROF-31 | measured start v0 = 1.5, shared example | 2.9051 s |
| PROF-33 | hand-over, k_p 3 | d_h = 0.05 m (fixed), v_h = 0.15 m/s; tail `ln(0.05/0.03)/3 = 0.170 s` per goal |
| PROF-L1 | plain law, ideal tracking: shared example, P1.stop(), P1.withVelocity(1.0), 150 random paths | never above vBack(s_v); per-loop deceleration <= aDec (discrete form `sqrt(vE^2 + (D·dt)^2) - D·dt`); no stall; never past a goal; shared example 3.39 s, profileTime 3.4426 s |
| PROF-L2 | hand-over envelope, shared example, EX | at s_goal - d_h: exactly v_h = 0.15 m/s (envelope `vE^2 = v_h^2 + 2·D·dt·v_h + 2·D·(s_goal - d_h - s_v)` in the discrete form); ideal run with the lag lead hands over at 0.15 m/s (within 1e-3), deceleration <= aDec |
| PROF-L3 | correction-budget cut on a constant arc (v_cmd from 0) | speeds up at aAcc to the planned arc speed vF (2.3784 m/s on the shared example) and holds it; never above vF |
| PROF-L4 | bad input (NaN s, cap or dt; dt = 0) | holds v_prev |
| PROF-38 | ARC_VAR selection: tight arc with P1.withVelocity(v), v = 0.5, 1.0 and 1.2, EX | 0.5 < ARC_VAR cap 0.9798: both arc spans ARC_VAR, 4.5577 s; 1.0 (between 0.9798 and the curvature cap 1.0954): constant arc at 1.0, 4.4075 s; 1.2: constant arc at its cap 1.0954, 4.3210 s, no warning |

The old physics test "turn in place pi/2 in 0.7071 s" is removed: v1 has no turn-in-place step (resolution R1).

#### 13.3.1 v2 reference: jerk-aware law (section 7.13)

Removed in v1; kept as v2 reference. These cases test the jerk-aware law, stopDist, vJerk and the offline profileTime, which are not in the v1 code.

| ID | Input | Expected |
|---|---|---|
| PROF-34 | law, ideal tracking: shared example and tight arc, EX and DEF + motor, jerkMax 30 to 150 | a = 0 at every arc entry (within one loop), no "up" on a constant-speed arc, speed constant on it; traction-ellipse use at the arc entry <= 1 (largest 0.9982 in the script, at jerkMax 10) |
| PROF-35 | two arcs joined by a 0.7729 m line: (0,0), (4,0), (5,1.2).withRadius(1.0), (9,1.2), EX, jerkMax 50 | arc speeds 1.2498 and 2.0; the speed changes only on the line; ellipse use at the second arc entry 0.9639; time to 0.03 m 5.22 s |
| PROF-36 | 6 m straight line cut into 0.1 m spans (face-target style), EX, jerkMax 50 | 3.08 s (the uncut line 3.06 s): line-to-line junctions do not force a = 0 |
| PROF-J1 | stopDist(3, 0, 0, jerkMax = 30, D = 3) | 1.6500 m, 1.1000 s |
| PROF-J2 | stopDist(3, 3, 0, 30, 3) | 2.1212 m, 1.2500 s |
| PROF-J3 | stopDist(1, 0, 0.8, 30, 3) (triangular) | 0.14697 m, 0.1633 s |
| PROF-J4 | stopDist(2, -2, 0.5, 30, 3) | 0.6622 m |
| PROF-J5 | stopDist(1, -2.5, 0.9, 30, 3) (over-braking) | 0.0626 m, 0.0667 s |
| PROF-J6 | stopDist vs numerical integration, random cases | difference < 1e-3 m |
| PROF-J7 | law, ideal tracking, shared example, jerkMax = 50, EX | no junction speed above vB; never past the end; max abs(da)/dt = 50; time to 0.03 m 3.54 s |
| PROF-J8 | law, P1.stop(), jerkMax = 50 | v_cmd = 0 within 0.03 m of P1, no pass; restart from (0, 0); 4.32 s |
| PROF-J9 | law with tau_lag = 0 in the plant model | overshoot > 0 (2.7 to 6.2 cm): the lead is needed |
| PROF-J10 | command hits v = 0 while braking at -6 m/s^2, landing rule off / on | spike / no spike |
| PROF-J11 | d_h for jerkMax = 10, k_p = 3 | 0.0222 m; hand-over step 10 m/s^3 |
| PROF-J13 | jerkMax = 0 or NaN | IAE V16 |
| PROF-J15 | jerkMax = 400 with DEF | warning C2 |
| PROF-J16 | spans: line 0-3 m (aD 3) to vB = 1.0, line 3-4 m with aD 1.8 (heading-limited) to vB = 0.5, line 4-6 m; jerkMax 50, aAcc = aDec = 3, from rest | D_j = 3 for the first junction, 1.8 for the second and the goal; D_brake = 3 while the first junction binds; both junctions reached at or below vB, within the 4 mm/s junction tolerance (0.020 m/s below in the script; not 0.939 m/s above, as with the minimum over all constraints) |
| PROF-J17 | random span chains (lines with random aD, constant-speed arcs, ARC_VAR arcs with a `withVelocity` point, point caps, short lines after arcs into slower arcs, stops, the end), jerkMax <= the C2 threshold `max(aAcc, aDec)/(5·dt_nom)` | speed at any junction at most 4 mm/s above vB and at a `withVelocity` point at most 4 mm/s above v (interpolated; never above in the script), no stall, traction-ellipse use on arcs <= 1 at the end of every loop, except the one-loop constant-arc entry effect (<= about 1.08; section 7.8) (<= 0.99995 in the earlier chains, <= 0.99944 with ARC_VAR arcs, <= 0.9505 on ARC_VAR spans). Also log the use interpolated at junctions next to arcs (up to 1.074 at constant-arc entries in the script, section 7.8). Above the threshold the speed reached 6.7 mm/s above vB and the use 1.97 (documented, section 7.8) |
| PROF-J18 | DEF, line 3 m, arc r = 2 (2 m), 0.75 m line, arc r = 0.5 (0.8 m), line 2 m; jerkMax 50 | vJerk caps the first arc at 3.035 m/s (jerk-free pass: 3.795); no braking on either arc; ellipse use <= 1 (0.869 in the script) |
| PROF-37 | ARC_VAR law, ideal tracking: shared example + P1.withVelocity(1.0) and tight arc + P1.withVelocity(0.5), EX, jerkMax 10 to 150 | a = 0 at the ARC_VAR entry (within one loop); braking and speed-up on the arc within aA = aD = 1.8; speed at P1 at most v + 4 mm/s (0.9746 and 0.5000 m/s at jerkMax 50 in the script); traction-ellipse use on the arc <= 1 (0.9608 and 0.8885); time to 0.03 m 4.06 and 5.00 s at jerkMax 50 |
| PROF-J19 | law comes to rest within d_h + 2·STOP_DIST_MARGIN of a goal (jerkMax 150, rest 5.2 mm before the hand-over point) | hands over to CAPTURE at rest; the offline profileTime run ends; without the rule it never ends (guard reached) |


### 13.4 Follower

Simulation scenarios (WPILib simulation or a Python model like section 8.15).

| ID | Scenario | Expected |
|---|---|---|
| FOL-T1 | straight 4 m path, robot exactly at P0, heading = theta(0) | finishes OK; max abs(e_n) < 0.01 m; done before profileTime + 0.6 s; final abs(d) < 0.03 m, abs(e_theta) < 2 deg |
| FOL-T2 | same, robot starts 0.2 m left | e_n decays with time constant 0.33 to 0.38 s, no sign change beyond 5 mm; correction starts at -0.6 m/s along N |
| FOL-T3 | line, arc (radius 1 m, 90 deg left), line, tau_lag = 0 | arc mean e_n negative (wide), within 30% of `kappa·v^2·tau_d/k_n` |
| FOL-T4 | T3 with tau_lag = measured tau_d | abs(mean e_n) on the arc < 0.01 m |
| FOL-T5 | robot at rest at P0, vProf(0) = 0, EX, jerkMax = 50 | moves at once: 0.01, 0.04, 0.09, 0.15 m/s in the first loops; same e_n as T1 |
| FOL-T6 | robot 0.1 m past the end | CAPTURE_END pushes back: V = k_p·(P_end - p) points backward; finishes OK |
| FOL-T7 | automatic stop corner (tau_turn > 150 deg), heading continuous | arrives with abs(d_go) <= 0.03 m and abs(V_meas) <= 0.1 m/s; switches sub-profile; ROTATE only if the residual is > 3 deg; leaves on the ramp; s* = s_c exactly at release, never past it before |
| FOL-T7b | `.stop(1.5)` on a straight point (a STOP, not a PASS marker), event at that index, one injected 0.06 s loop during the dwell | event fires once, in the arrival loop; robot holds P_c, leaves 1.50 to 1.52 s (measured) after the dwell starts; timeout = 1.5·(profileTime incl. 1.5 s) + 1 |
| FOL-T7c | same, robot arrives 0.6 m/s too fast (lag overshoot 0.04 m) | d_go < 0, the 2D law pulls back along -T_in; arrival only after abs(d_go) <= 0.03 and speed <= 0.1; event not fired before |
| FOL-T7d | straight line with a PASS marker that has an angle (no stop) | s* passes the marker without stalling; the marker is only a span boundary |
| FOL-T7e | `.stop(1.0)` on the last point | finish test passes, the robot holds 1.0 s more, then OK |
| FOL-T8 | path that returns to P0 (loop), robot at P0 | s* starts at 0, not L; does not finish at once |
| FOL-T9 | arc sweeping 220 deg (self-crossing lines) | s* never jumps from the in-line to the out-line; monotonic within 0.03 m |
| FOL-T10 | vision jump of 0.3 m along track at 3 m/s | s* catches up within 2 loops; no event double-fire; no mode change |
| FOL-T11 | vision jump of 0.1 m across track | one 0.3 m/s command step along N, spread by the limiter; e_n recovers with time constant about 0.33 s |
| FOL-T12 | push the robot back 0.5 m mid-leg | s* stays at s_hi - 0.03; e_t ~= -0.47; the robot catches up (re-plan callback: v2) |
| FOL-T13 | (v2) pin the robot against a wall 0.6 m off the path for 1 s | exactly one re-plan callback, reason OFF_PATH; ABORT gives FAILED_REPLAN |
| FOL-T14 | block the robot so it cannot reach the end | ends at `1.5·profileTime + 1 s` with FAILED_TIMEOUT; zero velocity sent once |
| FOL-T15 | pose supplier NaN for 3 loops at 3 m/s | last command repeated (0.06 s < 0.1 s); tracking resumes; no event fired during the NaN loops |
| FOL-T16 | pose supplier frozen for 0.5 s | zero command after 0.1 s; resumes when fresh; FAILED_POSE only after > 1 s |
| FOL-T17 | inject a 0.08 s loop | window = `clamp(3·v·0.08)` (capped at 0.5); settle timer counts 0.08; events in the crossed range fire in order |
| FOL-T18 | path with L = 0.05 m | starts in CAPTURE_END, finishes OK (a zero-length path cannot be built, GEO-34) |
| FOL-T19 | red alliance, ROTATE | drives the flipped path in the blue frame; flip computed once per path object across two schedules |
| FOL-T19b | red alliance, FlipMode.NONE | drives the path as written |
| FOL-T20 | MIRROR, path with a left corner | the corner becomes a right turn; e_n still "left of the flipped path"; converges |
| FOL-T21 | face target 3 m to the side, robot passing at 2 m/s | omega_ff matches the finite-difference d/dt atan2 within 1%; e_theta < 3 deg at the closest pass |
| FOL-T22 | face target, path passing within 0.3 m | theta_ref frozen inside 0.5 m; no omega above omegaMax |
| FOL-T23 | theta_ref = 179 deg, theta = -179 deg | omega_fb is clockwise (negative) for the 2 deg error |
| FOL-T24 | finish heading error -1.9 deg, then -2.1 deg | -1.9: in tolerance; -2.1: not (the 2025 code would accept -2.1) |
| FOL-T25 | events at fraction 0, fraction 0.2 and the last point | fraction 0 fires on loop 1; 0.2 fires once when crossed; the last point fires at the finish even if s* = L - 0.02 |
| FOL-T26 | re-schedule mid-path after a disable, robot 2 m from P0 | ends at once with FAILED_START_TOO_FAR and alert A1 naming the distance; zero speed once |
| FOL-T27 | event command that requires the drivetrain | IAE V20 at construction |
| FOL-T28 | worked example of section 8.3.1 | s* = 1.4849, e_n = +0.0707, v_t = 2.3604, a_cmd = 0 (no "up" on the arc), V = (1.4972, 1.8509) |
| FOL-T30 | CAPTURE entry | at d_go = d_h = 0.05 m: abs(V) = 0.15 m/s = the law's v_h (continuous); deceleration step 22.5 m/s^3 <= jerkMax |
| FOL-T31 | vision jump makes c jump to 1.0 m/s at v_t = 3.95, vMax 4 | v_t cut at once to 3.873, law state reset (a_cmd <= 0) |
| FOL-T29 | holdEndCommand after FAILED_TIMEOUT | holds P_end with the CAPTURE_END law until interrupted |
| FOL-T32 | robot 0.6 m from P0 at start; robot 0.4 m behind P0 | FAILED_START_TOO_FAR with alert A1; drives normally (s* = 0 until P0 is passed) |
| FOL-T33 | telemetry, command scheduled twice | `PathLogger.putData` called once, with one `Sendable` under `Path/<name>` that has the keys state, segment, s, eN, vRef, vMeas, eThetaDeg |
| FOL-H4 | accelerating at 4 m/s with the forward limit active, heading ramp | compare omega_ff from v_t and from filtered ds*/dt: heading error RMS; decide section 6.5 |
| FOL-I1 | real robot: follower commands omega = +1 rad/s | robot turns CCW seen from above (only the drivetrain negates) |

### 13.5 Public API

| ID | Input | Expected |
|---|---|---|
| API-1 | `new Path()`; `new Path(new Point(1, 2))` | IAE V1 with `you gave 0` / `you gave 1` |
| API-2 | `new Point(Double.NaN, 1)`; `new Point(0, Double.POSITIVE_INFINITY)` | IAE V4 |
| API-3 | `new Path(new Point(1, 2), null)` | NPE V3 with `point 1` |
| API-4 | `new Point(1, 2).withRadius(-0.3)`; `withRadius(0)` on a middle corner | IAE V9; accepted, the corner is a stop |
| API-5 | `withVelocity(-1)`; `withVelocity(0)` | IAE V11; `isStop()` true, `dwellSeconds()` 0, `maxVelocity()` empty |
| API-6 | `stop(-1)`; `stop(Double.NaN)`; `stop(1.5)` | IAE V13; IAE V4; `dwellSeconds()` = 1.5 |
| API-7 | `p.stop(2).withVelocity(1.0)`; `p.withVelocity(1.0).stop()` | not a stop, `maxVelocity()` = 1.0; a stop, `maxVelocity()` empty |
| API-9 | `new Path(p(0,0), p(2,0), p(2.0005,0), p(2,2))` | W3; L = 4.2263 |
| API-10 | `new Path(p(0,0), p(0.0004,0))` | IAE V6 |
| API-11 | shared example | L = 5.9769, no warnings, `sAtPoint(1)` = 2.9885 |
| API-12 | event fraction 1.5; index 3 on the shared example | IAE V19 (fraction text); IAE V19 with `numbered 0 to 2` |
| API-13 | event at point 1; at fraction 0.5 | `events().get(0).s()` = 2.9885; 2.98845 |
| API-14 | `withVelocity(2)` on P_0; on the last point | W4 (both, with their reasons) |
| API-15 | `withRadius(0.01)` on a middle corner; `withRadius(5)` on (0,0),(2,0),(2,2) | W1 and a stop; W2, shrunk to 1.3107 |
| API-16 | 170 deg corner, legs 4 m | W1 and a stop |
| API-17 | `new PathConstraints(4, 7.2, 7.2, 0, 2*Math.PI, 4*Math.PI, j)` | IAE V16 naming `aLat` |
| API-18 | `PathConstraints.fromConfig(new KinematicsConfig())` | (4.05, 7.2, 7.2, 7.2, 2·pi, 4·pi, 50) |
| API-19 | `new FollowPathCommand(null, path)` | NPE V24 |
| API-20 | event command requiring the follower's subsystem | IAE V20 |
| API-21 | `PathFollowerConfig.defaults().withKN(0)`; `.withField(16.54, -1)` | IAE V16 naming `kN`; naming `fieldWidth` |
| API-22 | `path.flipped(cfg) == path.flipped(cfg)`; `path.flipped(cfg.withFlip(FlipMode.NONE)) == path` | true (memo); true; lengths and event s equal to the blue path |
| API-23 | `.stop(1.5)` with an event at that point | event once on arrival; leaves 1.50 to 1.52 s after the wait starts; timeout includes 1.5·1.5 s |
| API-24 | `.stop(1.0)` on the last point | finish passes, 1.0 s more, then `OK` |
| API-26 | `FollowResult.values()` in v1 | RUNNING, OK, FAILED_TIMEOUT, FAILED_START_TOO_FAR, FAILED_POSE, INTERRUPTED |
| API-27 | `PathLogger.set(custom)`, then build (0,0),(2,0),(2,2) with `.withRadius(5)` | `custom.alert` called once with the W2 text, kWarning, true; `PathLogger.get() == custom`; `PathLogger.set(null)` throws NPE |
| API-28 | GEO-11 | W5 naming point 1 and 0.2429 m; geometry identical to (0,0),(2,0),(2,2) |

## Appendix A. JSON sketch (future editor, not v1)

**For a future path editor, not part of v1** (**Decision**, user, simplification pass): v1 has no reader, no writer and no reserved
method. A file could look like this (blue frame; meters, degrees, seconds, m/s):

```json
{
  "schemaVersion": 1, "name": "score",
  "points": [
    { "x": 1.0, "y": 2.0, "angle": 0.0 },
    { "x": 3.0, "y": 4.0, "maxVelocity": 1.0 },
    { "x": 4.0, "y": 2.0, "stop": true, "dwell": 1.5 },
    { "x": 7.0, "y": 4.0, "radius": null }
  ],
  "constraints": { "vMax": 3.0 },
  "faceTarget": null,
  "events": [ { "type": "fraction", "value": 0.2, "action": "intakeDown" }, { "type": "point", "value": 2, "action": "shoot" } ]
}
```

Each field maps to the Java call of the same meaning (section 9); a reader would apply the same validation (section 9.12).

## Appendix B. Test oracle: grid passes

Test code only, not production code (resolution R10). The grid is the time-optimal profile of a point mass under the same bounds, with
the exact traction ellipse and motor curve at every sample, so the span method of section 7 must never beat it.

**Equations.**
1. **Samples**: the boundaries of section 7.3.1; each interval of length l gets `N = max(1, ceil(l/ds - 1e-9))` uniform steps
   with DEFAULT_DS = 0.02 m. The passes are exact for constant acceleration, so ds sets only the cap resolution. ds 0.02 vs
   0.005 gives the same 3.2723 s on the shared example; the sampled peak is 2.7194 vs 2.7238 m/s; the exact 2.7241 lies between samples (script).
2. **Curvature at a sample**: the segment's kappa; at a boundary, the larger abs(kappa) of the two sides.
3. **Caps**: as section 7.4, per sample; point caps at their sample; stops and L are 0.
4. **Passes with the ellipse** (explicit, acceleration at the known neighbor):
   ```
   backward: v_i = min(cap_i, sqrt(v_{i+1}^2 + 2*ds_i*aDec*sqrt(max(0, 1 - (v_{i+1}^2*abs(kappa_i)/aLat)^2))))
   forward:  v_i = min(cap_i, sqrt(v_{i-1}^2 + 2*ds_{i-1}*a_fwd(v_{i-1}, i)))      (Heun step for the motor curve)
   a_fwd(v, i) = min(aAcc*sqrt(max(0, 1 - (v^2*abs(kappa_i)/aLat)^2)), alphaMax/abs(theta'_i), 0.9*maxForwardAccel*max(0, 1 - v/freeSpeed))
   ```
   The explicit scheme is off by O(ds) where kappa changes; cap_i is still applied exactly. Unlike the span method, the grid may change
   speed on an arc.
5. **Lookup**: interpolate v^2 linearly (exact for constant a). Example: (0, 0) and (0.02, 0.3464) at s = 0.005 gives 0.1732;
   interpolating v linearly would give 0.0866.
6. **Time**: `dt_i = 2·ds_i/(v_i + v_{i+1})`, exact for constant acceleration; e.g. from rest over 0.02 m at 3 m/s^2:
   `0.04/0.3464 = 0.1155 s = sqrt(2·0.02/3)`.

**How to use it in tests.**
- PROF-4: on paths where every arc is driven at its curvature cap, span time = grid time within 1e-3 s (shared example 3.2723 s both;
  DEF + motor 2.3101 s both; the zero-penalty rows of section 7.8).
- PROF-5: on any path, span time >= grid time - 1e-3 s. With a point cap on an arc the span method is slower: shared example +
  P1.withVelocity(1.0) 3.7751 against 3.5770 s (the ARC_VAR share), P1 at 180 deg 4.2812 against 3.4372 s (the heading-kink cap slows
  the whole arc).
- PROF-15 and the grid columns of sections 6.11 and 7.8 (3.5770 s, 3.4372 s, 4.4469 s) are regression numbers for the oracle code itself.
- Two grid scripts with different arc sampling gave 3.5770 and 3.5761 s for the `withVelocity(1.0)` case; the 0.0009 s difference is the
  O(ds) error of the explicit step, so compare with a tolerance of 1e-3 s.
