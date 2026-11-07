# Path explained

This note explains the whole `frc.robot.Path` package in plain words: what each class does, and the
geometry and physics behind it. It is for students and mentors. It does not replace the short
math note [Path-design.md](Path-design.md) or the package
[README](../Infrastructures/src/main/java/frc/robot/Path/README.md). The code is the source of truth.
Where this note and the older design note differ, see the last section.

All file references are in `Infrastructures/src/main/java/frc/robot/Path/`.

> Status: the code has not been run on a robot. The constants are starting values, not tuned values.

## 1. The big picture

You give the robot a list of **waypoints**: points on the field it should pass through. The module does
two jobs.

**Job A, once, when you build the `Path`:**

1. Look at each waypoint. If the path turns there, put a **corner circle** through the waypoint.
   If the path is straight there, forget the waypoint. If it must stop there (or the turn is very
   sharp), make it a **stop**.
2. Join the corners with straight **legs** that touch the circles at exactly one point each (tangent legs).
3. Cut each circle down to an **arc**, from where the incoming leg touches it to where the outgoing
   leg leaves it.
4. Result: a list of segments, line, arc, line, arc, line, and so on. Each segment is parameterized by
   distance along itself.

**Job B, every 20 ms, while the robot drives (`PathFollower.calculate`):**

1. Find out how far along the path the robot is (its **progress** `progress`), using the pose.
2. Pick a **speed**: as fast as the limits allow, but slow enough for the next arc and for the next stop.
3. Pick a **direction**: along the path, bent gently back toward the path if the robot is off it.
4. Pick a **turning rate** to follow the heading plan.
5. Return field-relative speeds `(vx, vy, turnRate)`. `FollowPathCommand` hands them to the drivetrain.

A leg, an arc through a waypoint, and the next leg (the example used again in section 4):

```
 y
 2 |                                B  (end point)
   |                               /
   |                              /   leg 2
   |                             /    (almost straight up)
   |                          .-'
   |                       .-'  <- arc (center C near (1.82, 0.18))
 0 |  A -----------------'--*  V = waypoint (2, 0), ON the arc
   |        leg 1
   +----------------------------------- x
      0                      2
```

The arc is a piece of a circle that passes through `V`. The legs are tangent to the circle, so the
path has no kinks: the direction changes smoothly. Note that the path does not run on the straight
lines `A-V` and `V-B`. It cuts the corner, and leg 1 even dips a little below `y = 0` (section 4).

## 2. Units and frames

- Distance in **meters**, time in **seconds**, angles in **radians**.
- **Field coordinates**: `x` runs along the field length, `y` across the width. Positive angle is
  **counter-clockwise** (CCW), the WPILib convention. Heading 0 means facing +x.
- A "left turn" is CCW. A "right turn" is CW.
- All speeds that the follower reads and writes are **field-relative**: `vx` is speed toward +x,
  `vy` toward +y, `turnRate` is CCW spin. Not robot-relative.
- Waypoints are written for the **blue** side. On red, the whole path is flipped (section 6).

Why this choice:

- The path is drawn on the field, so the math is simplest in field coordinates. No conversion by the
  current heading is needed while following. The robot can turn and drive along different directions
  at the same time (swerve), so heading and travel direction are independent.
- The pose estimator (RobotPose) already gives field coordinates.
- One coordinate system everywhere means fewer sign mistakes.

The drive method that receives the output must also be field-relative. If your drive method expects
robot-relative speeds, convert in your own wrapper (for example with
`ChassisSpeeds.fromFieldRelativeSpeeds`) before it reaches the module's consumer.

## 3. The classes

### Point (`Point.java`)

A waypoint. It is **immutable**: each `withX` returns a new object (`Point.java:25`, `:31`, `:34`, `:40`).

- `new Point(x, y)` (`:20`): position in meters. NaN or infinity is rejected (`check`, `:70`).
- `withAngle(Rotation2d)` (`:25`): the robot heading wanted at this point.
- `stop()` / `stop(seconds)` (`:31`, `:34`): stop here, wait the dwell time, continue. The dwell is
  ignored on the last point (the last point is always a stop).
- `withRadius(m)` (`:40`): corner radius for this point only. Default is `PathConfig.RADIUS`.
- `flipped(mirror)` (`:52`): the same point for the red side (section 6).

Used by: `Path` reads the points; `FollowPathCommand` never touches them directly.

### Segment (`Segment.java`)

An abstract piece of path. Every segment is parameterized by `distance`, the **distance in meters from its start**.
Five methods (`Segment.java:8-20`): `length()`, `pointAt(distance)`, `tangentAt(distance)` (unit direction of travel),
`project(point)` (closest `distance` to a point `point`, **not clamped**), and `curvature()` (signed, 1/m, positive
means turning left, 0 for a line).

Because `Line` and `Arc` share this shape, the follower never needs to know which one it is on.

### Line (`Line.java`)

A straight segment. Stores the start and a unit direction. The constructor (`:11`) rejects a length of
zero. `pointAt(distance) = start + direction * distance` (`:22`). The tangent is the constant direction (`:25`).
`project(point) = dot(point - start, direction)` (`:28`). Curvature is 0 (`:31`).

### Arc (`Arc.java`)

A piece of a circle: center, `radius`, turn direction, `startAngle`, `sweep`. Constructor at `:22`.
It stores `turn = +1` for a left turn and `-1` for a right turn, and uses that sign in every formula.

- `length() = radius * sweep` (`:36`). Arc length is radius times angle in radians.
- `pointAt(distance)` (`:39`): the angle after moving `distance` along the arc is `angle = startAngle + turn * distance / radius`,
  and the point is `center + radius * (cos angle, sin angle)`.
- `tangentAt(distance)` (`:45`): `turn * (-sin angle, cos angle)`. This is the derivative of the point with
  respect to distance, so it has length 1.
- `project(point)` (`:51`): see section 4.5.
- `curvature() = turn / radius` (`:58`). Curvature is 1/radius; a small circle is a sharp curve.

### Path (`Path.java`)

Builds and holds the segments. The constructor (`Path.java:50`) does four steps: check the points
(`checkPoints`, `:72`), turn waypoints into corners (`cornerAt`, `:82`), join corners with legs and
arcs (`buildSegments`, `:100`), and place the heading anchors (`:64-69`).

Helper records: `Corner` (`:30`, a waypoint turned into a circle, or a point with radius 0), `Stop`
(`:22`, path distance and dwell) and `Anchor` (`:24`, path distance and heading).

Key methods:

| Method | Line | Job |
|---|---|---|
| `cornerAt(i)` | 80 | Corner circle, stop or nothing for waypoint `i` |
| `tangentLeg(from, to)` | 212 | The straight leg between two circles |
| `cornerArc(...)` | 125 | The arc, with validity checks |
| `locate(point, fromDistance)` | 188 | Closest path distance to `point`, not before `fromDistance` |
| `indexAt(distance)` | 165 | Which segment holds path distance `distance` |
| `pointAt(distance)`, `tangentAt(distance)` | 171, 176 | Position and direction at path distance `distance` |
| `flipped()` | 138 | The red-side path |
| `stops()`, `anchors()` | 159, 162 | Stops and heading anchors in path order |
| `length()` | 147 | Total length |

Path distance `distance` is distance from the very start of the whole path. `segmentStarts.get(index)` is the path distance where
segment `index` begins. Local `distanceOnSegment = distance - segmentStarts.get(index)`.

### PathFollower (`PathFollower.java`)

The brain that runs every loop. `calculate(pose, measuredFieldSpeeds)` (`:62`) returns the speeds, as a short sequence of private step methods (5.1 to 5.7).
It remembers: current segment `segmentIndex`, next stop `stopIndex`, `progress`, last commanded speed `previousSpeed`,
last command vector `previousVelocity`, dwell counter `waitLoops` and `finished`. Details in section 5.
`updateTargetHeading(distance)` (`:197`) computes the heading plan. `isFinished()` (`:186`) and `progress()` (`:189`)
are for the command.

### FollowPathCommand (`FollowPathCommand.java`)

A WPILib `Command` around the follower. It picks the alliance, checks the start, creates the follower in
`initialize()` (`:38`), calls it in `execute()` (`:52`), and sends zero speed in `end()` (`:60`).
Section 6.

### PathConfig (`PathConfig.java`)

The values to set for your robot, in one place (`PathConfig.java`). The other numbers are private constants in the class that uses them. Section 7.

### How they connect

```
Point[] --> Path --(builds)--> Segment list (Line, Arc, Line, ...) + stops + anchors
                                        |
 pose, measured speeds --> PathFollower.calculate --> ChassisSpeeds (field-relative)
                                        ^                        |
                          FollowPathCommand (alliance flip,      v
                          start check, suppliers)           drive consumer
```

## 4. The geometry, step by step

Running example for sections 4.1 to 4.6: waypoints `A = (0, 0)`, `V = (2, 0)`, `B = (2, 2)`, `radius = 0.25 m`.
I computed every number in this section with a separate script that repeats the code's formulas.

### 4.1 The turn angle (cross and dot products)

At an interior waypoint `V`, with the previous point `P` and the next point `N`:

- `directionIn = unitVector(V - P)`: the direction we arrive in. (`unit` means divide by length so the length is 1.)
- `directionOut = unitVector(N - V)`: the direction we leave in.

The signed turn angle is

`turn = atan2( cross(directionIn, directionOut), dot(directionIn, directionOut) )` (`Path.java:89`)

where `cross(a, b) = ax*by - ay*bx` and `dot(a, b) = ax*bx + ay*by`.

Why it works: for two unit vectors with angle `turn` between them, `dot = cos(turn)` and
`cross = sin(turn)`. The `cross` is positive when `directionOut` is CCW from `directionIn`. `atan2(sin, cos)` gives back
`turn` with the right sign, in `(-180, 180]` degrees. Using `atan2` of both avoids the problems of
`acos`, which cannot tell left from right and is inaccurate near 0 and 180 degrees.

Example: `directionIn = (1, 0)`, `directionOut = (0, 1)`. `cross = 1*1 - 0*0 = 1`, `dot = 0`, so `turn = atan2(1, 0) = 90` degrees.
Positive, so it is a **left turn**.

- `turn > 0`: left (CCW). `turn < 0`: right (CW).
- `|turn| < 1e-3` rad (about 0.06 degrees): no turn, no corner (section 4.7).
- `|turn| > MAX_ARC_TURN` (150 degrees): too sharp, becomes a stop (section 4.8).

### 4.2 Where the corner circle goes

The center is placed at

`C = V + radius * unitVector(directionOut - directionIn)` (`Path.java:95`).

Why this direction: `directionOut - directionIn` points to the inside of the turn (toward the side the path bends to; for a
left turn it points left of both legs). And the circle passes through `V`
because `C` is exactly `radius` from `V` (the added vector has length `radius`). That is the whole trick: the
path goes **through** the waypoint instead of only near it, so the waypoint is a real point on the path.

Example: `directionOut - directionIn = (-1, 1)`. Its unit vector is `(-0.7071, 0.7071)`. Times `0.25` gives
`(-0.1768, 0.1768)`. So `C = (2 - 0.1768, 0 + 0.1768) = (1.8232, 0.1768)`. Check: distance from `C` to `V` is
`sqrt(0.1768^2 + 0.1768^2) = 0.25`. Good.

The waypoint is on the circle but it is not where the legs touch the circle. The legs touch at other
points (4.3), so the path passes through `V` partway along the arc.

### 4.3 The tangent leg and the signed-radius trick

A leg is a straight line that touches two circles, leaving the first and entering the second. Each end
is a point where the line is tangent to the circle. The robot drives CCW around a left corner circle and CW
around a right one, so the leg must leave in the direction of travel at that point.

Rather than write one formula for each combination, `Path` uses **signed radii** (`Path.java:34`):
`+radius` for a left turn, `-radius` for a right turn, and `0` for a plain point (start, end, stop). A plain point is
just a circle of radius 0.

For circles with centers `C1`, `C2` and signed radii `fromSignedRadius`, `toSignedRadius` (`tangentLeg`, `Path.java:215`):

1. `centerDistance = |C2 - C1|` and `radiusDifference = toSignedRadius - fromSignedRadius`.
2. `legNormal = unitVector(C2 - C1)` rotated CCW by `acos(radiusDifference / centerDistance)`.
3. Tangent points: `T1 = C1 - fromSignedRadius * legNormal` and `T2 = C2 - toSignedRadius * legNormal`.

Here `legNormal` is the unit normal to the leg, pointing to the left of the direction of travel. A point on the
circle where the leg touches it lies `radius` away from the center, along the normal. For a left-turn circle
(the robot goes CCW, the center is on the robot's left) the center is on the left of the leg, so the
touch point is **center minus radius times the left normal**: `C - signedRadius * legNormal` with `signedRadius = +radius`. For a right-turn
circle the center is on the right, so the touch point is center plus `radius` times `legNormal`, which is
`C - signedRadius * legNormal` with `signedRadius = -radius`. The sign in `signedRadius` takes care of it.

Why the angle is `acos(radiusDifference / centerDistance)`: the leg is perpendicular to its normal `legNormal`, so the two tangent points
`T1 = C1 - fromSignedRadius legNormal` and `T2 = C2 - toSignedRadius legNormal` must have the same position along `legNormal`. That gives
`dot(C2 - C1, legNormal) = toSignedRadius - fromSignedRadius = radiusDifference`. The left side is `centerDistance * cos(angle between legNormal and C2 - C1)`, so the angle is
`acos(radiusDifference / centerDistance)`. That one condition covers every case:

| Case | `fromSignedRadius`, `toSignedRadius` | `radiusDifference` | Angle `acos(radiusDifference/centerDistance)` |
|---|---|---|---|
| Point to circle | `0`, `+-radius` | `+-radius` | the leg aims at a tangent point on the circle |
| Same direction (both left or both right) | `+radius`, `+radius` | `0` | exactly 90 degrees: the leg is parallel to the line between centers |
| Opposite directions | `+radius`, `-radius` | `-2r` | more than 90 degrees: the leg crosses between the circles |

Worked numbers (computed):

- Same direction, centers `(0,0)` and `(2,0)`, both `+0.25`: `radiusDifference = 0`, `acos(0) = 90` degrees. `legNormal = (0, 1)`. Tangent
  points `(0, -0.25)` and `(2, -0.25)`. The leg runs along the bottom, parallel to the line between
  centers, going +x. A CCW circle moves toward +x at its bottom. Correct.
- Opposite, same centers, `fromSignedRadius = +0.25`, `toSignedRadius = -0.25`: `radiusDifference = -0.5`, `acos(-0.5 / 2) = 104.5` degrees. Tangent points
  `(0.0625, -0.242)` and `(1.9375, 0.242)`: a leg that crosses from the bottom of the first circle to the
  top of the second (an S-shape).
- Point to circle, `(0,0)` to center `(2, 1)` with `+0.25`: tangent point `(2.086, 0.765)`.

**When the leg does not exist.** The leg exists only when `centerDistance > |radiusDifference|` (otherwise `acos` has no answer). For a
point and a circle this means the point is outside the circle. For two same-direction circles (`radiusDifference = 0`) it
means the circles are not at the same place. For opposite directions it means the circles do not overlap. If
the test fails, `tangentLeg` returns `null` and the path throws "corner arc does not fit"
(`Path.java:105`, `:219`).

For our running example: the start `A = (0,0)` to circle `C = (1.8232, 0.1768)` (left): the tangent point is
`(1.8132, -0.0730)`. The circle `C` to the end `B = (2,2)`: tangent point `(2.0730, 0.1868)`.
Leg lengths: both `1.8146 m`.

Look at the first tangent point: its `y` is **negative**. Leg 1 leaves `A` pointing slightly downward
(about -2.3 degrees), touches the circle near its bottom, and curves up. The path cuts the corner but
stays on a circle through `V`, so it bulges a little on the outside of the straight line near the start. That
is normal for this method.

### 4.4 The arc's start angle and sweep

Once the incoming leg ends at `entry` and the outgoing leg starts at `exit` (both on the circle), the arc
runs between them (`cornerArc`, `Path.java:127`):

- `startAngle = angleOf(entry - C)`, where `angleOf(v) = atan2(vy, vx)` is the angle of a vector.
- `sweep = wrapTo2Pi( sign * (angleOf(exit - C) - startAngle) )`, where `sign = +1` for a left turn and `-1` for a
  right turn, and `wrapTo2Pi` wraps into `[0, 2 pi)`.

Why the sign: a left arc turns CCW, so its angle grows. A right arc turns CW, so the angle shrinks;
multiplying by `-1` makes the sweep positive. `wrapTo2Pi` makes "go the long way around" correct: a sweep
is always the CCW (or CW) angle from entry to exit, never negative.

Example: `entry - C = (1.8132 - 1.8232, -0.0730 - 0.1768) = (-0.0100, -0.2498)`, which points almost straight
down: `startAngle = -92.31` degrees. `exit - C = (0.2498, 0.0100)`, pointing almost along +x: angle `2.31`
degrees. `sweep = 2.31 - (-92.31) = 94.61` degrees. This is the "near 94.6 degrees" in the task: more than the
`90` degree turn, because the legs touch the circle away from where the straight lines would.
Arc length is `radius * sweep = 0.25 * 1.6513 rad = 0.4128 m`. Total path length: `1.8146 + 0.4128 + 1.8146 = 4.042 m`.

The waypoint `V` sits at angle `angleOf(V - C) = angleOf(0.1768, -0.1768) = -45` degrees, which is `47.3` degrees after
the start, inside the `94.6` degree sweep. So the arc really passes through `V`. This check (`toWaypoint`,
`Path.java:131`) is part of the validity test (4.6).

### 4.5 Projection: how close am I, and where along?

`project(point)` returns the `distance` of the point on the segment nearest to `point`. It is **not clamped**: the follower needs
to know when `point` is past the end (`distance > length`) or before the start (`distance < 0`).

- **Line**: `distance = dot(point - start, direction)`. This is the length of the shadow of the vector `point - start` on the
  line. Negative if `point` is behind the start. (`Line.java:28`)
- **Arc**: take `angle`, the polar angle of `point` around the center, and use (`Arc.java:51`)

  `distance = radius * ( angleModulus( turn * (angle - startAngle) - sweep/2 ) + sweep/2 )`.

  Read it from the inside. `turn * (angle - startAngle)` is how far around the circle, in the direction of
  travel, the closest point is from the arc start. `angleModulus` wraps an angle into `(-pi, pi]`. If we
  wrapped the plain angle, a point just before the start would show up as about `2 pi`, not as a small negative
  number. Shifting by `sweep/2` first puts the middle of the arc at 0, so the wrap point is on the far
  side of the circle, away from the arc. Then we shift back. Result: points before the arc get a small
  negative `distance`, points after it get `distance > radius*sweep`, and points on it get the right distance.

The radius of `point` from the center does not matter: only the **angle** is used. That is exactly "the nearest
point on the circle, in the radial direction".

### 4.6 When a corner is rejected

`cornerArc` (`Path.java:134`) accepts an arc only if all three hold:

1. `sweep > 1e-9`: there is an arc at all.
2. `sweep <= turn + 90 degrees` (`turn` is the absolute turn of the corner): a sweep much bigger than the turn
   means the leg went around the circle the long way, a **loop**. This happens when corners are close
   together: the leg from the previous circle touches this one on the wrong side.
3. `toWaypoint <= sweep`: the waypoint must lie on the arc. If it does not, the path would skip the
   waypoint.

Also `tangentLeg` can return `null` (section 4.3): two circles too close, or a point inside a circle.
And `checkPoints` (`Path.java:72`) rejects fewer than 2 points, null points, or two consecutive points
within 1 cm.

All of these throw `IllegalArgumentException` when the `Path` is **built**, before any driving. The
message names the waypoint index. Fix: move waypoints farther apart, or give that point a smaller
`withRadius`. A radius is a size: the larger the radius, the more room the corner needs on both sides.

### 4.7 Dropped straight points

If `|turn| < 1e-3` rad and the point is not a stop, `cornerAt` returns `null` (`Path.java:91`). No arc is made;
the path just runs through. It would be pointless (and numerically bad: `unitVector(out - in)` is almost the zero
vector) to put a circle there.

The waypoint still needs a place on the path, for its heading. `locate` (`Path.java:190`) finds the nearest
path distance to the point, never earlier than the previous waypoint (`fromDistance`), so the order of waypoints is kept.

### 4.8 Sharp corners become stops

If `|turn| > 150` degrees (`MAX_ARC_TURN`), the path almost reverses. A circle for such a turn would be a
near-circle loop and the robot would need to slow to a crawl anyway. So `cornerAt` returns a plain point
(radius 0), the same as a stop (`Path.java:90`). The legs go exactly to `V`, and the follower treats it as a
stop with dwell 0 (it brakes to a halt, then continues). An explicit `.stop()` does the same (`Path.java:84-85`).

## 5. The follower, loop by loop

`PathFollower.calculate` runs once per loop (`DT = TimedRobot.kDefaultPeriod = 0.02 s`). Steps in the order of the code.

### 5.1 Guards (`calculate`, `:63`; `isBadPose`, `:83-86`)

If the follower already finished, or the pose has a NaN or infinite `x`, `y` or heading, return zero speeds.
A NaN would otherwise poison every later number (NaN in a `min` or `max` spreads, and some comparisons with NaN are
false). Zero is the safe answer.

### 5.2 Progress and switching segments (`updateProgress`, `:89-101`)

The follower knows the current segment `segmentIndex`. It projects the robot onto it: `distanceOnSegment = segment.project(position)`.

- If `distanceOnSegment >= length` of the segment, the robot has passed its end, so move to the next one (`segmentIndex++`) and project
  again. This is a `while` loop, so a big jump can skip a short segment.
- It never moves past a segment that **ends at the next stop** (`:92-93`, the `segmentStart(segmentIndex+1)` vs `stop.distance()` test). The
  stop must be reached and released first. Otherwise a robot that overshoots would start the next leg without stopping.
- Then `progress = max(progress, segmentStart(segmentIndex) + clamp(distanceOnSegment, 0, length))` (`:98-99`).

Why switch by projection and not by distance to the end point: the robot never hits an exact point. A distance
threshold can fire early or never. Projection is a clean geometric question: "am I past the end of this segment?".

Why `progress` never goes backward (`max`): pose noise, a bump or a sideways shove could make the nearest point jump back a
few centimeters. If `progress` went back, the speed law would think a stop or arc is farther away than it is, the heading plan
would turn back, and in a path that crosses itself the robot could lock onto the wrong crossing. A monotone `progress` makes
progress a one-way quantity. The cost: the robot cannot retreat along the path (see limits).

Then `updateTargetHeading(distance)` sets the heading plan (5.7) and `headingError = angleModulus(targetHeading - heading)`.

### 5.3 Stops, dwell and finishing (`handleStop`, `:104-130`)

- `measuredSpeed` is the measured speed `hypot(vx, vy)`. If it is NaN, fall back to the last command's length (`:107`).
- `stopDistance` is the straight distance from the robot to the next stop point.
- `atStop = stopDistance < POSITION_TOLERANCE (3 cm) && measuredSpeed < STOP_SPEED (0.1 m/s)`. Both: **close** and **slow**. Close
  alone could be a fly-by at speed.
- **Last stop (the end):** finished when `atStop` and the heading error is under `ANGLE_TOLERANCE` (2 degrees), using
  `Math.abs(angleModulus(finalHeading - heading))` (`:111`). Then return zeros and set `finished`. The end dwell is not used.
- **Middle stop:** when `atStop` (or when already waiting), count loops: `++waitLoops >= round(dwell / DT)` (`:117`).
  When it reaches the count, release the stop: `stopIndex++`, `waitLoops = -1`, set `progress = stop.distance()` and re-find `segmentIndex` with
  `indexAt(progress)`. Setting `progress` exactly fixes the case where the robot was up to 3 cm short. A dwell of 0 releases
  on the first loop. A dwell of 1.0 s counts 50 loops.
- While waiting, or when within `POSITION_TOLERANCE / 2` (1.5 cm) of the stop, return **zero translation** but keep turning
  toward the heading plan with `clamp(HEADING_KP * headingError, +-MAX_OMEGA)` (`:124-128`), and reset `previousSpeed` and
  `previousVelocity` to zero, so the next leg starts from rest.

Why `Math.abs` and `angleModulus` on the angle: a heading error of `-1.5` degrees is a good result, but `-1.5 < 2` is true
for any big negative error too, so the absolute value is required. And `358` degrees and `-2` degrees are the same
heading; `angleModulus` wraps the difference into `(-pi, pi]` so "almost one full turn away" reads as "almost
zero".

### 5.4 The speed law (`speedAlongPath`, `:133-151`)

The speed `speed` is the smallest of several limits. Each one has a physical reason.

**(a) Acceleration limit and top speed (`:138`).**
`speed = min(previousSpeed + MAX_ACCELERATION * DT, MAX_VELOCITY)`. In one loop, speed may rise by `3 * 0.02 = 0.06 m/s`. From rest to
`2 m/s` takes `2 / 3 = 0.67 s` and `0.67 m`. The law builds on its own previous command `previousSpeed`, not on the measured
speed. Measured speed is noisy and lags, and would make the speed "jump".

**(b) Arc speed cap (`:139-145`).** On a circle of radius `radius`, a robot at speed `speed` has centripetal acceleration `speed^2 / radius`
(the sideways acceleration needed to keep turning). Wheels can only give so much sideways force before slipping, so cap
it at `MAX_ACCELERATION` (`a_lat = 3`):

`v_arc = sqrt(a_lat * radius)`, written in the code as `sqrt(a_lat / curvature)` with curvature `1/r`.

For `radius = 0.25`: `sqrt(3 * 0.25) = 0.87 m/s`. For `radius = 0.5`: `1.22 m/s`. Bigger radius, faster corner.

The robot must already be that slow **when it arrives** at the arc. So for every later arc `j` before the next stop
the code limits the speed now so it can still brake in time:

`speed <= sqrt(v_arc_j^2 + 2 * a_dec * d_j)`, with `d_j` the distance to the arc's start (`:143-144`).

This is the braking formula `v^2 = v_j^2 + 2 a d` (constant deceleration: final speed squared = start speed squared plus
twice acceleration times distance), turned around: the fastest we may go `d` meters before the arc and still be at
`v_arc` when we get there, braking at `a_dec`. Example: from `2 m/s` down to `0.87 m/s` at `a_dec = 3` needs `d = (4 - 0.75) / 6 = 0.54 m`.
On the arc itself `d = 0`, so the cap is just `v_arc`.

**(c) Stop approach (`:134-137`, `:146-150`).** `distanceToStop` is the path distance left to the next stop. Two laws, and `speed` takes the
smaller:

- Constant braking: `speed <= sqrt(2 * a_dec * distanceToStop)`. This is the same formula with final speed 0. From `2 m/s` it begins at
  `4 / 6 = 0.67 m` from the stop.
- Proportional: `speed <= FINAL_KP * distanceToStop`, with `FINAL_KP = 3 1/s`. A speed proportional to the distance left is an exponential
  approach with time constant `1/3 s`: smooth, no overshoot, no jerk at the end. The two laws are equal at `0.67 m`
  (both give `2 m/s`). Closer than that, the proportional law is lower.

**MIN_SPEED (`:148`).** As the distance goes to 0, both laws go to 0. But a swerve drive has a **deadband**: tiny
commands do not move the robot, because motors need some minimum output to overcome friction and the controller
ignores values near zero. The robot would stall short of the stop forever. So the magnitude is raised to at least
`MIN_SPEED = 0.15 m/s` (the proportional law falls below it at about 5 cm). This floor stays until the robot is within
1.5 cm (the `POSITION_TOLERANCE / 2` check in 5.3), where the command drops to zero and the stop is declared.

The sign is kept from `distanceToStop` (`copySign`). If the robot **passed** the stop, `distanceToStop` is negative. For the segment that
ends at the stop, `distanceOnSegment` is not clamped (`:136-137`), so the overshoot is visible, `approachSpeed` is negative, and the final line
`speed = max(speed, min(0, approachSpeed))` (`:150`) lets `speed` go to `-0.15`: it drives back toward the stop slowly. Otherwise `speed` is floored at 0.

### 5.5 Direction: the look-ahead and the smooth merge (`velocity`, `:154-178`)

Now the follower has a speed `speed` and needs a direction.

**Look-ahead (`:159-160`).** The drivetrain does not respond instantly: commands pass through a CAN loop, motor controllers,
and the wheel's inertia. Assume a lag of `LAG_TIME = 0.05 s`. In that time the robot moves `speed * LAG_TIME`
(`0.1 m` at `2 m/s`). So the direction is read from the path tangent not at `progress`, but at `progress + speed * LAG_TIME`: we steer
for where the robot will be when the command takes effect. The look point is held just before the stop
(`stop.distance() - 1e-9`), so near a stop the direction does not read from the next leg.

**Normal error (`:156-157`, `:162`).** `tangent` is the unit direction of the path at `progress`. `normal = (-ty, tx)` is the tangent rotated 90 degrees
CCW: it points to the **left** of the path. The cross-track error

`sideError = cross(tangent, position - pathPoint(progress))`

is the signed distance of the robot to the left of the path (the shadow of `position - pathPoint` on the normal). `sideError` (eN) `> 0` means the robot is left of
the path, `sideError < 0` means right.

**The merge angle (`:163`).** Let `L = MERGE_DISTANCE = 0.5 m`. Then

`mergeAngle = -atan(sideError / MERGE_DISTANCE)`.

`mergeAngle` is how much to turn the direction of travel away from the path direction, toward the path. The minus sign: if the robot is
left (`sideError > 0`) we turn right (`mergeAngle < 0`). The numbers: `sideError = 0.1 m` gives `-11.3` degrees; `0.3 m` gives `-31` degrees; `0.5 m` gives `-45` degrees; `1 m` gives
`-63` degrees; far away it tends to `-90` degrees (drive straight at the path). `atan` is the key: it is smooth,
and it saturates, so the angle never exceeds 90 degrees and never flips. The angle depends only on `sideError`. It does not depend on the speed.

**The merge law (`:166`).** The robot keeps its speed and only turns the direction of travel. The velocity along the path is `speed cos(mergeAngle)`
and the sideways part is

`sidewaysSpeed = |speed| * sin(mergeAngle)`

(negative when the robot is left of the path, so it moves right). For a small error, `sin(mergeAngle) ~ -sideError / MERGE_DISTANCE`, so the sideways speed is
`-speed * sideError / MERGE_DISTANCE`: proportional to the error. That is `d(sideError)/dt = -(speed / MERGE_DISTANCE) * sideError`, so the error shrinks
exponentially, with time constant `MERGE_DISTANCE / speed` (0.5 s at `1 m/s`). Picture the path as a river and the robot as a boat: the closer to the path, the more it
points along it. It merges **tangentially**, and ends up on the path moving along it. A "straight line to a point 1 m ahead" is a different
idea: the robot would aim at a fixed target, cut corners on arcs, and the direction would jump as the target moves to
the next segment. The merge law is a vector field defined everywhere, with no target to choose.

**Worked example (the test `PathTest.mergesOntoLegAlongAGentleCurve`).** The robot starts `0.3 m` beside an `8 m` straight leg. Then
`mergeAngle = -atan(0.3 / 0.5) = -31.0` degrees. Whatever the speed is, the sideways part is `0.515` of it and the along-path part is `0.857` of it, so the robot
slides onto the leg in a gentle curve. In the test it crosses the leg by at most `1.4 mm`, with a `0.1 s` drivetrain lag in the simulation.

**Why not a sideways floor everywhere.** An earlier version raised the sideways speed to at least `FINAL_KP * |sideError|` everywhere. At the start the robot is slow (it
accelerates from rest), and `FINAL_KP * 0.3 = 0.9 m/s` is much more than the sideways part of a slow robot. The sideways part dominated the along-path part, and the
robot headed nearly straight at the line: an approach angle of `86.7` degrees instead of `31.0`. The rule that the angle follows only `sideError` fixes that.

**Low-speed correction near a stop (`:167-169`).** The correction rate is `speed / MERGE_DISTANCE`, and a robot that is about to stop is slow
(at `MIN_SPEED = 0.15` it barely corrects). So only when `|distanceToStop| < MERGE_DISTANCE`, the sideways speed is raised to at least a pull straight toward the path:

`sidewaysSpeed = -signum(sideError) * max( |speed sin(mergeAngle)|, FINAL_KP * |sideError| )`, then `|sidewaysSpeed| <= MAX_VELOCITY` (`:169`).

`FINAL_KP * |sideError|` is a rate of 3 per second, the same as the stop approach, so the robot settles on the stop point and does not stay beside it.
Example: `speed = 0.3` (the stop approach allows `3 * 0.1 = 0.3` at 0.1 m from the stop), `sideError = 0.1`: `|speed sin mergeAngle| = 0.3 * 0.196 = 0.059`, so
the floor `3 * 0.1 = 0.3` wins and `sidewaysSpeed = -0.3` (to the right). Farther than `0.5 m` from a stop, with `speed = 1` and `sideError = 0.1`, the law
alone applies: `sidewaysSpeed = 1 * sin(-11.3 degrees) = -0.196`, and the along-path part is `0.981`.

The command is `velocity = lookAheadDirection * (speed cos mergeAngle) + normal * sidewaysSpeed` (`:170-171`). The along-path part uses the look-ahead direction,
the sideways part uses the normal at `progress`.

### 5.6 The 2-D acceleration limit (`:172-176`)

The speed law limited changes of `speed`, a number. But the direction can change too, and a sudden sideways
change is also an acceleration. So the change of the **whole velocity vector** is capped:

`velocityChange = velocity - previousVelocity`. If `|velocityChange| > 2 * MAX_ACCELERATION * DT = 6 * 0.02 = 0.12 m/s`, scale `velocityChange` down to that length.

That is a limit of `6 m/s^2` in any direction, which covers a start, a stop, a turn, a merge. (It is bigger than `MAX_ACCELERATION = 3`
because a vector change can mix a speed change with a direction change.) Finally the length of `velocity` is clamped to `MAX_VELOCITY`.

### 5.7 Heading and turnRate (`turnRate`, `:181-183`; `updateTargetHeading`, `:197`)

**Heading plan.** `anchors` are `(distance, angle)` pairs: each point with `withAngle`. If the path has no angle at `distance = 0`, the
constructor adds an anchor at 0 with the robot's starting heading (`:51-52`). Between two anchors `before` and `after`, the
reference heading changes **linearly with path distance**:

`targetHeadingPerMeter = angleModulus(angle_after - angle_before) / (distance_after - distance_before)` [rad/m], `targetHeading = angle_before + targetHeadingPerMeter * (distance - distance_before)`.

`angleModulus` picks the shortest way around: from 350 to 10 degrees is `+20`, not `-340`. After the last anchor the
heading is held. If two anchors have the same `distance` (within `1e-9`), the heading is held and `targetHeadingPerMeter` is 0, which avoids division by zero.

**Command.**

`turnRate = targetHeadingPerMeter * speed + HEADING_KP * headingError`, clamped to `+-MAX_OMEGA` (`:182`).

- `targetHeadingPerMeter * speed` is a **feedforward**: heading changes `targetHeadingPerMeter` radians per meter, the robot moves `speed` meters per second,
  so the heading must change at `targetHeadingPerMeter * speed` rad/s. The units multiply out to rad/s. This makes the heading follow the plan without lag.
- `HEADING_KP * headingError` is a **P term**: a push of `4 rad/s` per radian of error to fix what the feedforward
  and the drivetrain miss.

Because the plan is in path distance, not time, the heading automatically goes with the speed: slow down, turn
slower; stop, the heading holds.

Last lines of `calculate` (`:75-79`): if any output is NaN, return zero, else store `previousSpeed = speed`, `previousVelocity = velocity`, and return `(velocity.x, velocity.y, turnRate)`.

## 6. FollowPathCommand

**Alliance flip (`initialize`, `:38-40`).** Paths are written for blue. If the Driver Station says red, the command
uses `path.flipped()` (`Path.java:140`), which flips every point (`Point.flipped`, `Point.java:52`) and rebuilds the path.
If the alliance is unknown it defaults to blue. Two ways, chosen by `PathConfig.MIRRORED_FIELD` (default `false`):

| | Position | Heading | Use when |
|---|---|---|---|
| Rotate (`false`) | `(x, y) -> (L - x, W - y)` | `angle + 180 deg` | the red field is the blue field turned 180 degrees about the center |
| Mirror (`true`) | `(x, y) -> (L - x, y)` | `180 deg - angle` | the red field is the blue field reflected across the line `x = L/2` |

`L` and `W` are the field length and width from the AprilTag layout of the current season (`kDefaultField`, loaded once in `Point.Field`; 2026 Rebuilt welded: 16.541 m and 8.069 m. Example, the point `(3, 2)`: rotate gives about `(13.54, 6.07)`, mirror gives
`(13.54, 2)`. A heading of 30 degrees becomes 210 degrees (rotate) or 150 degrees (mirror).

A rotation keeps handedness: a left turn stays a left turn. A **mirror reverses handedness**, like a reflection in a
mirror: a left turn becomes a right turn, and a counter-clockwise spin becomes clockwise. That is why `Path.flipped`
rebuilds from the flipped **points** and recomputes the turn signs from the new geometry, instead of copying the old
arcs. Pick the setting that matches how your field is laid out.

**Start check (`:41-49`).** It compares the robot with the path start. If the distance is more than `START_TOLERANCE` (0.5 m),
it reports an error to the Driver Station and sets `follower = null`. Then `isFinished()` is true at once (`:57`) and the
command ends without moving. Only position is checked, not heading. This guards against running an auto from the wrong
place on the field. Otherwise it creates `new PathFollower(active, start)` with the starting pose, whose heading is
the first heading anchor.

**Running.** `execute()` calls `follower.calculate(pose.get(), fieldSpeeds.get())` and passes the result to the consumer
(`:52-54`). `end()` sends zeros (`:60`). `progress()` returns a 0..1 fraction (`:63`).

**Wiring.** Three things from your own robot code: a pose supplier, a measured field-relative speed supplier, and a
drive method that takes field-relative speeds, plus the drive subsystem for requirements. Example, one call:

`new FollowPathCommand(path, swerve::getPose, swerve::getFieldSpeeds, swerve::driveFieldRelative, swerve)`

The speed supplier must be the **measured** field-relative speed (it is used for the "slow enough to be stopped" test), not
the last commanded one.

## 7. Constants

Seven values are set per robot in `PathConfig.java`. The rest are `private static final` in the one class that uses
them (`PathFollower`, `Path`, `FollowPathCommand`). Values are starting points.

Derived, not separate constants: braking (`a_dec`) and the arc acceleration (`a_lat`) are `MAX_ACCELERATION`, and the
velocity-vector limit is `2 * MAX_ACCELERATION`. The loop period is `TimedRobot.kDefaultPeriod` (0.02 s, so it matches
the real period). The field size comes from WPILib's AprilTag layout for the current season,
`AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)` (`getFieldLength()` / `getFieldWidth()`), loaded once on first flip
in `Point.Field`.

**Set per robot, in `PathConfig`:**

| Constant | Value | Unit | Meaning | Too big | Too small |
|---|---|---|---|---|---|
| `MAX_VELOCITY` | 2.0 | m/s | Top speed | Wheels or pose estimate cannot keep up; harder to stop | Slow auto |
| `MAX_ACCELERATION` | 3.0 | m/s^2 | Speed-up along the path; also braking, arc (lateral) acceleration, and half the vector limit | Wheel slip, tipping, brownout; late braking, arcs entered too fast | Slow starts, early braking, crawls through corners |
| `MAX_OMEGA` | 6.28 | rad/s | Top turn rate | Faster than the drive can turn | Heading lags the plan |
| `RADIUS` | 0.25 | m | Default corner circle radius | Wide, slow-cutting corners; more "does not fit" errors; path strays from the waypoint line | Sharp corners, low arc speed cap (`sqrt(a_lat r)`) |
| `POSITION_TOLERANCE` | 0.03 | m | "At the stop/end" distance (and half of it ends the creep) | Finishes sloppy | May never finish if the robot cannot get that close |
| `ANGLE_TOLERANCE` | 2 deg | rad | Heading error allowed at the end | Finishes with a bad heading | May never finish |
| `MIRRORED_FIELD` | false | none | false: rotate 180; true: mirror across the center line. Set per season | Wrong red path if mismatched | Wrong red path if mismatched |

**Internal, in the class that uses them:**

| Constant (class) | Value | Unit | Meaning | Too big | Too small |
|---|---|---|---|---|---|
| `HEADING_KP` (`PathFollower`) | 4.0 | rad/s per rad | P gain on heading error | Oscillates, hunts | Heading error stays |
| `FINAL_KP` (`PathFollower`) | 3.0 | 1/s | Gain on distance to stop, and on cross-track error within `MERGE_DISTANCE` of a stop | Aggressive, overshoot, twitchy | Slow settle at a stop |
| `MERGE_DISTANCE` (`PathFollower`) | 0.5 | m | Cross-track distance giving a 45 degree merge | Slow, lazy return to the path | Hard snap, near 90 degrees for small errors, may oscillate |
| `LAG_TIME` (`PathFollower`) | 0.05 | s | Drivetrain delay the look-ahead covers | Steers for a point too far ahead, cuts arcs | Does not compensate, trails the path |
| `MIN_SPEED` (`PathFollower`) | 0.15 | m/s | Least approach speed toward a stop until 1.5 cm away; beats the drive deadband | Hits the stop hard; overshoot | Stalls before reaching the stop |
| `STOP_SPEED` (`PathFollower`) | 0.1 | m/s | Measured speed under which a stop counts as reached | Declares a stop while still moving | May never see "stopped"; must stay above `MIN_SPEED`-level noise (see limits) |
| `START_TOLERANCE` (`FollowPathCommand`) | 0.5 | m | Max distance from robot to path start | Allows a start from the wrong place | Refuses small pose errors |
| `MAX_ARC_TURN` (`Path`) | 150 deg | rad | Corners turning more become stops | Loops and near-reversals | Many corners become stops |

Note on `STOP_SPEED` against `MIN_SPEED`: the robot creeps toward a stop at about `0.15 m/s` and must read below `0.1 m/s` to be
"stopped". The creep ends (command zero) within 1.5 cm. It then needs the measured speed to drop under `0.1`. This is
one more thing to check on the real robot.

## 8. Limits and assumptions

Be honest with the students: these are real.

- **Not run on a robot.** All gains and limits are guesses. Tune `HEADING_KP`, `FINAL_KP`, `MERGE_DISTANCE`, `LAG_TIME`,
  `MIN_SPEED` and the accelerations on the real drivetrain.
- **No timeout.** If the robot is stuck (blocked, pose lost), the command never ends by itself. Wrap it with a timeout
  (`withTimeout`) in the auto.
- **End dwell is ignored.** `stop(seconds)` on the last point does nothing extra. The command ends when it is there.
- **The robot is a point.** The path is for the robot center. Bumper size, obstacles and field walls are not considered. A
  bigger corner radius means the real robot sweeps a wider area.
- **Heading is not coupled to speed.** A heading plan that needs a big turn over a short distance is clamped by
  `MAX_OMEGA`; the heading then lags the plan, with no slowdown.
- **Progress never goes back.** If the robot is pushed backward along the path it does not "un-progress"; a long
  detour may leave it far from the point `s` thinks it is at, and the merge law handles only the sideways part.
- **Pose quality.** Everything depends on the pose estimate and on the measured speed (for stopping). A bad or frozen
  pose gives bad commands. The follower only guards against NaN.
- **Middle stops do not check heading.** Only the end checks the angle.
- **Build-time errors.** A path that does not fit throws when constructed. Build paths in a static field or at robot
  init, not in the middle of a match, so a mistake shows up early.
- **Same-turn corners must be far enough apart.** Two close left corners with big radii are the common cause of
  "corner arc does not fit".
