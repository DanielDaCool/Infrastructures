# Path: math

The student guide is [Path README](../Infrastructures/src/main/java/frc/robot/Path/README.md). This note holds
only the math the code uses. The earlier long design is in [archive/Path-design.md](archive/Path-design.md).

Units are m, s, rad. `perp(x, y) = (-y, x)`. Constants are in `PathConstants`.

## Corner circle

For an interior waypoint, `directionIn = unitVector(waypoint - previousWaypoint)`,
`directionOut = unitVector(nextWaypoint - waypoint)`,
`turn = atan2(cross(directionIn, directionOut), dot(directionIn, directionOut))`.

- Stop point or `|turn| > MAX_ARC_TURN`: sharp corner, a point node at the waypoint with speed 0.
- `|turn| < 1e-3` and not a stop: no arc; the heading anchor is placed by projecting the waypoint onto the path.
- Otherwise: `isLeftTurn = turn > 0` (counter-clockwise), `radius`, `center = waypoint + radius * unitVector(directionOut - directionIn)`.
  The circle passes through the waypoint.

## Tangent legs

A leg is a common tangent between two nodes. At a tangent point on a circle, the travel direction must be
`turnSign * perp(tangentPoint - center) / radius`, with `turnSign` +1 for a left turn and -1 for a right turn.

The code (`Path.tangentLeg`) uses one formula for every case. Give each node a signed radius
(`+radius` left turn, `-radius` right turn, `0` for a point; radii may differ per corner). With
`centerToCenter = to.center - from.center`, `centerDistance = |centerToCenter|` and
`radiusDifference = to.signedRadius - from.signedRadius`: the leg exists only if
`centerDistance > |radiusDifference|`; then `legNormal = rotate(unitVector(centerToCenter), acos(radiusDifference / centerDistance))`
is the unit normal on the leg's left, and the tangent points are `from.center - from.signedRadius * legNormal`
and `to.center - to.signedRadius * legNormal`. This covers point-to-circle, circle-to-point, same-direction and
opposite-direction legs.

The arc sweep is
`wrapTo2Pi(turnSign * (angleOf(exit - center) - angleOf(entry - center)))`; it must be in `(0, 2 pi)` and contain the
polar angle of the waypoint, and it must not exceed `|turn| + 90 deg` (close same-direction corners would otherwise
loop). If any check fails the path throws `IllegalArgumentException` ("corner arc does not fit").

## Arc projection

For a point with polar angle `pointAngle` around the center, arc start angle `startAngle`, and `sweep`:

`offset = angleModulus(turnSign * (pointAngle - startAngle) - sweep / 2) + sweep / 2`, `distance = radius * offset`.

`distance` is not clamped: it is negative before the arc start and greater than `radius * sweep` after the end.
For a line, `distance = dot(point - start, direction)`.

## Progress and segment switching

With current segment `segmentIndex`, `distanceOnSegment = segment.project(position)`. While
`distanceOnSegment >= segment.length()`, `segmentIndex` is not the last segment and the boundary is not an
unreleased stop: `segmentIndex++` and project again. Then

`progress = max(progress, segmentStart(segmentIndex) + clamp(distanceOnSegment, 0, segment.length()))`.

Segments switch by projection, never by a distance threshold, and `progress` never decreases.

## Speed law

`previousSpeed` is the previous commanded speed (0 after a stop). With `segmentStart_j` and `arcSpeedLimit_j` the
distance and speed cap of each later arc, and `stop.distance` the path distance of the next stop:

`speed = max(0, min(previousSpeed + MAX_ACCELERATION * DT, MAX_VELOCITY, sqrt(arcSpeedLimit_j^2 + 2 * MAX_DECELERATION * (segmentStart_j - progress)), approachSpeed))`

Near a stop `approachSpeed = min(sqrt(2 * MAX_DECELERATION * distanceToStop), FINAL_KP * distanceToStop)` uses the
signed along-track error `distanceToStop` (negative past the stop, so an overshoot drives back) with magnitude at
least `MIN_SPEED` until the robot is within `POSITION_TOLERANCE / 2`, then zero. The final command vector is clamped
to `MAX_VELOCITY`.

`arcSpeedLimit = sqrt(MAX_LATERAL_ACCELERATION / |curvature|)`, i.e. `sqrt(MAX_LATERAL_ACCELERATION * radius)`,
applied from the start of the arc. The law uses its own previous command, not the measured speed. A 2-D limit
follows: `|velocity - previousVelocity| <= MAX_VECTOR_ACCELERATION * DT`.

## Merge law

Lookahead: `lookAheadDistance = min(progress + speed * LAG_TIME, stop.distance - 1e-9)` (just before the stop, so it
stays on the arriving leg); `lookAheadDirection` is the tangent at `lookAheadDistance`, `tangent` the tangent at
`progress`. Side error (`sideError`, signed distance from the path, left positive):
`sideError = cross(tangent, position - pointAt(progress))`. With `MERGE_DISTANCE`:

`mergeAngle = -atan(sideError / MERGE_DISTANCE)`, along-path part `speed * cos(mergeAngle)` along `lookAheadDirection`,
`sidewaysSpeed = |speed| * sin(mergeAngle)` along `normal = perp(tangent)`. The travel direction is turned toward the
path and the speed is kept, so the robot curves onto the leg; the approach angle depends only on `sideError`, not on speed
(0.3 m beside a leg: 31.0 deg; test `PathTest.mergesOntoLegAlongAGentleCurve`).

Only when `|distanceToStop| < MERGE_DISTANCE` (the robot is slow and must settle) the sideways part is raised:
`sidewaysSpeed = -signum(sideError) * max(|sidewaysSpeed|, FINAL_KP * |sideError|)`. In all cases `|sidewaysSpeed| <= MAX_VELOCITY`.
The path is approached asymptotically, never by a straight line to a point.

## Heading interpolation

Anchors: `(0, start heading)` unless point 0 has an angle, then `(distance, angle)` for each angled point.
Between anchors `targetHeading(distance)` is linear in `distance` along the shortest wrap; after the last anchor it
holds. `targetHeadingPerMeter` is the slope, `angleModulus(after.angle - before.angle) / anchorSpacing`.

`turnRate = targetHeadingPerMeter * speed + HEADING_KP * angleModulus(targetHeading - heading)`, `|turnRate| <= MAX_OMEGA`.

## Finish check

At the end stop the command is finished when

`|position - end| < POSITION_TOLERANCE` and `|angleModulus(finalHeading - heading)| < ANGLE_TOLERANCE`.

An intermediate stop is reached when `|position - pointAt(stop.distance)| < POSITION_TOLERANCE` and the measured
speed (`measuredSpeed`) is under `STOP_SPEED`; then the dwell counts `dwell / DT` loops and the stop is released.
