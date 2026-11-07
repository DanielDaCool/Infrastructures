# Path: math

The student guide is [Path README](../Infrastructures/src/main/java/frc/robot/Path/README.md). This note holds
only the math the code uses. The earlier long design is in [archive/Path-design.md](archive/Path-design.md).

Units are m, s, rad. `perp(x, y) = (-y, x)`. Constants are in `PathConstants`.

## Corner circle

For an interior point `V`, `u_in = unit(V - V_prev)`, `u_out = unit(V_next - V)`,
`theta = atan2(cross(u_in, u_out), dot(u_in, u_out))`.

- Stop point or `|theta| > MAX_ARC_TURN`: sharp corner, a point node at `V` with speed 0.
- `|theta| < 1e-3` and not a stop: no arc; the heading anchor is placed by projecting `V` onto the path.
- Otherwise: `sigma = sign(theta)` (+1 counter-clockwise), radius `r`, center `C = V + r * unit(u_out - u_in)`.
  The circle passes through `V`.

## Tangent legs

A leg is a common tangent between two nodes. At a tangent point `T` on a circle with center `C`, the travel
direction must be `d = sigma * perp(T - C) / r`.

- Point `P` to circle: `d0 = |C - P| > r`, `alpha = asin(r / d0)`, `dir = rotate(unit(C - P), -sigma * alpha)`,
  `T = P + sqrt(d0^2 - r^2) * dir`. Circle to point is the mirror of this.
- Circle to circle, same `sigma`: `u = unit(C2 - C1)`, `n = rotate(u, -sigma * 90 deg)`, `T1 = C1 + r n`, `T2 = C2 + r n`.
- Circle to circle, opposite `sigma`: `D = |C2 - C1| > 2r`, `a = acos(2r / D)`, `n = rotate(u, -sigma1 * a)`,
  `T1 = C1 + r n`, `T2 = C2 - r n`.

Travel-direction check: the leg vector must satisfy `dot(T_out - T_in, d) > 0`. The arc sweep is
`mod2pi(sigma * (angle(T_out - C) - angle(T_in - C)))`; it must be in `(0, 2 pi)` and contain the polar angle
of `V`, and it must not exceed `|theta| + 90 deg` (close same-direction corners would otherwise loop). If any check fails the path throws `IllegalArgumentException` ("corner arc does not fit").

## Arc projection

For a point `p` with polar angle `phi_p` around the center, and arc start angle `phi0`, sweep `w`:

`rel = angleModulus(sigma * (phi_p - phi0) - w / 2) + w / 2`, `t = r * rel`.

`t` is not clamped: it is negative before the arc start and greater than `r * w` after the end.
For a line, `t = dot(p - start, unit)`.

## Progress and segment switching

With current segment `k`, `t = seg_k.project(p)`. While `t >= length_k`, `k` is not the last segment and the
boundary is not an unreleased stop: `k++` and project again. Then

`s = max(s_prev, segmentStart(k) + clamp(t, 0, length_k))`.

Segments switch by projection, never by a distance threshold, and `s` never decreases.

## Speed law

`vPrev` is the previous commanded speed (0 after a stop). With `s_j` and `v_j` the distance and speed cap of each
later arc or stop, and `s_stop` the next stop:

`v = max(0, min(vPrev + MAX_ACCELERATION * DT, MAX_VELOCITY, arcCap, sqrt(v_j^2 + 2 * MAX_DECELERATION * (s_j - s)), FINAL_KP * (s_stop - s)))`

Near a stop the capture speed uses the signed along-track error `d` (negative past the stop, so an overshoot drives
back) with magnitude at least `MIN_SPEED` until the robot is within `POSITION_TOLERANCE / 2`, then zero. The final
command vector is clamped to `MAX_VELOCITY`.

`arcCap = sqrt(MAX_LATERAL_ACCELERATION * r)`, applied from the start of the arc. The law uses its own previous
command, not the measured speed. A 2-D limit follows: `|vCmd - vCmdPrev| <= MAX_VECTOR_ACCELERATION * DT`.

## Merge law

Lookahead: `s_look = min(s + v * LAG_TIME, s_stop)`; `T` is the tangent at `s_look`, `T_s` the tangent at `s`.
Normal error (left positive): `eN = cross(T_s, p - point(s))`. With `L = MERGE_DISTANCE`:

`psi = -atan(eN / L)`, `vT = v cos(psi)` along `T`, `vN = v sin(psi)` along `N = perp(T_s)`.

At low speed the normal part is raised: `vN = -signum(eN) * max(|vN|, FINAL_KP * |eN|)`, then `|vN| <= MAX_VELOCITY`.
The path is approached asymptotically, never by a straight line to a point.

## Heading interpolation

Anchors: `(0, start heading)` unless point 0 has an angle, then `(s_i, angle_i)` for each angled point.
Between anchors `thetaRef(s)` is linear in `s` along the shortest wrap; after the last anchor it holds.

`omega = (dThetaRef / ds) * v + HEADING_KP * angleModulus(thetaRef(s) - theta)`, `|omega| <= MAX_OMEGA`.

## Finish check

At the end stop the command is finished when

`|p - end| < POSITION_TOLERANCE` and `|angleModulus(theta_end - theta)| < ANGLE_TOLERANCE`.

An intermediate stop is reached when `|p - point(s_j)| < POSITION_TOLERANCE` and the measured speed is under
`STOP_SPEED`; then the dwell counts `dwell / DT` loops and the stop is released.
