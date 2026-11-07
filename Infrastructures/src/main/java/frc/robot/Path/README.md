# Path

[![WPILib](https://img.shields.io/badge/WPILib-2026.2.1-blue)](https://docs.wpilib.org/)
[![Java](https://img.shields.io/badge/Java-17-orange)](https://adoptium.net/)
[![Vendor libraries](https://img.shields.io/badge/vendor%20libraries-none-lightgrey)](#install)

Autonomous paths for swerve robots. You write the points the robot must drive through. Path rounds the
corners, picks a speed and a heading for every part of the path, and `FollowPathCommand` drives the robot
along it. It works with any swerve drivetrain and any pose source: you pass a pose supplier, a field-speeds
supplier and a field-relative drive method. It needs no code from outside its own package.

> [!WARNING]
> **Status: not yet run on a robot.** The constants are starting values, not tuned values.

> [!NOTE]
> The math (corner circle, tangents, progress, speed and merge laws) is in
> [Path-design.md](../../../../../../../docs/Path-design.md).

## Install

Copy `Infrastructures/src/main/java/frc/robot/Path` into your robot project, keeping the package `frc.robot.Path`.
Requires WPILib 2026.2.1 and Java 17. No vendor libraries.

## Quick start

Units are meters, seconds, radians. Coordinates are field coordinates (x forward, y left, counter-clockwise
positive) on the blue side. Output speeds are field-relative.

```java
Path path = new Path(
    new Point(1, 2),
    new Point(3, 4).withAngle(Rotation2d.fromDegrees(90)),
    new Point(5, 2).stop(1.0),
    new Point(7, 2));

FollowPathCommand follow = new FollowPathCommand(
    path,
    swerve::getPose,             // Supplier<Pose2d>
    swerve::getFieldSpeeds,      // Supplier<ChassisSpeeds>, measured, field-relative
    swerve::driveFieldRelative,  // Consumer<ChassisSpeeds>
    swerve);                     // required subsystem
follow.schedule();               // or use it in a SequentialCommandGroup

new Trigger(() -> follow.progress() > 0.7).onTrue(intake.deployCommand());   // your command
```

Make a new command each time autonomous starts. Put the robot on the first point first: the command does
not reset your pose. The trigger's command must not require the drivetrain, or it cancels the path.
`progress()` goes from 0 to 1 and never decreases.

## Points API

`Point` is immutable. Every `withX` call returns a new object, so chain the calls.

| Call | What it does |
|---|---|
| `new Point(x, y)` | A point, in meters. Must be finite; consecutive points must be more than 1 cm apart |
| `.withAngle(Rotation2d)` | The robot faces this way at this point; it turns smoothly between angle points |
| `.stop()` | Stop exactly on this point |
| `.stop(seconds)` | Stop, wait that many seconds, then continue |
| `.withRadius(meters)` | Corner circle radius for this point (`>= 0`). Default is `PathConstants.RADIUS` |

The last point is always a stop. A path needs at least 2 points.

## Corners

Each middle point is a corner. The robot drives along a circle that passes through the point.

- **Circle through the point.** The radius is `RADIUS` unless the point has `withRadius`. Both turn directions work.
- **Turn over 150 degrees** (`MAX_ARC_TURN`) or a stop point: a sharp corner; the over-150 case is an automatic stop with no dwell.
- **Points on a straight line** (turn about 0) make no arc; the point is only a pass marker (it still holds an angle).
- **An arc that does not fit** (points too close for the radius, or an arc that would sweep more than the turn
  plus 90 degrees, which happens with close same-direction corners) throws one short `IllegalArgumentException`
  that names the point. Move the points apart or use a smaller radius.

## Speed and following

- The speed is a profile along the path, not a clock. It accelerates with `MAX_ACCELERATION` and brakes with
  `MAX_DECELERATION` by the distance left along the path to the next stop or the end.
- An arc of radius `r` is capped at `sqrt(MAX_LATERAL_ACCELERATION * r)`. The robot brakes before the arc.
- Progress follows the robot's projection onto the path. It never decreases, and segments switch by that
  projection, not by a distance threshold.
- If the robot is off the path, it returns on a smooth curve (merge over `MERGE_DISTANCE`), not a straight line.
- The direction looks ahead `LAG_TIME` seconds to cancel the command delay.
- The commanded velocity vector changes by at most `MAX_VECTOR_ACCELERATION * DT` per loop.
- A NaN pose gives zero speeds.

## Headings

Points with `withAngle` are anchors. The start anchor is the robot's current heading unless the first point
has an angle. The reference angle is linear in distance along the path between anchors, along the shortest
way around, and holds the last angle. Omega = feed-forward from the reference slope + `HEADING_KP` * wrapped error,
limited to `MAX_OMEGA`.

## Alliance flip

`FollowPathCommand.initialize` reads `DriverStation.getAlliance()`. On Red it uses `path.flipped()`.
`PathConstants.MIRRORED_FIELD` picks the mode; set it per season:

- `false`: red is the blue field rotated 180 degrees about the field center: `(L - x, W - y)`, angle + pi.
- `true`: red is the blue field mirrored across `x = L / 2`: `(L - x, y)`, angle becomes `pi - angle`.

`L` and `W` are `FIELD_LENGTH` and `FIELD_WIDTH`. Your pose must be in the same field coordinates on both alliances.

## Constants

| Constant | Default | Meaning |
|---|---|---|
| `RADIUS` | 0.25 m | Default corner circle radius |
| `MAX_VELOCITY` | 2.0 m/s | Speed limit |
| `MAX_ACCELERATION` | 3.0 m/s^2 | Speed-up limit along the path |
| `MAX_DECELERATION` | 3.0 m/s^2 | Braking used by the profile |
| `MAX_LATERAL_ACCELERATION` | 3.0 m/s^2 | Sets the arc speed cap |
| `MAX_VECTOR_ACCELERATION` | 6.0 m/s^2 | Limit on the change of the commanded velocity vector |
| `MAX_OMEGA` | 2 pi rad/s | Rotation speed limit |
| `HEADING_KP` | 4.0 1/s | Heading error gain |
| `FINAL_KP` | 3.0 1/s | Gain near a stop and on the normal error at low speed |
| `MERGE_DISTANCE` | 0.5 m | Scale of the return curve to the path |
| `LAG_TIME` | 0.05 s | Lookahead time for the direction |
| `POSITION_TOLERANCE` | 0.03 m | Position tolerance at a stop and at the end |
| `ANGLE_TOLERANCE` | 2 deg | Heading tolerance at the end |
| `STOP_SPEED` | 0.1 m/s | Measured speed below which a stop counts as reached |
| `MIN_SPEED` | 0.15 m/s | Lowest approach speed to a stop until within `POSITION_TOLERANCE / 2`, so a drive deadband can't stall it |
| `START_TOLERANCE` | 0.5 m | Largest allowed distance from the first point at start |
| `MAX_ARC_TURN` | 150 deg | Larger turns become an automatic stop |
| `DT` | 0.02 s | Loop period |
| `FIELD_LENGTH` / `FIELD_WIDTH` | 16.54 / 8.07 m | Field size for the flip |
| `MIRRORED_FIELD` | false | Flip mode (see above) |

## Finish, stop and start rules

- At a stop the robot reaches the point (within `POSITION_TOLERANCE`, measured speed under `STOP_SPEED`), waits
  the dwell, then continues. During the wait it sends zero translation and keeps its heading.
- The command finishes at the end point within `POSITION_TOLERANCE` and `ANGLE_TOLERANCE`. When it ends,
  it sends zero speeds.
- If the pose is farther than `START_TOLERANCE` from the first point at start, the command reports one error
  line and finishes without driving.

## Troubleshooting

| Symptom | Cause |
|---|---|
| Throws "corner arc does not fit" | Points too close for the radius. Move them apart or use `withRadius` with a smaller value |
| Command ends at once, error in the Driver Station | The robot is more than 0.5 m from the first point, or the pose is not set for this alliance |
| Robot drives the mirrored path on Red | `MIRRORED_FIELD` is wrong for this field, or your pose is in red-origin coordinates |
| Robot stops at a sharp corner | The turn is over 150 degrees. Add a point in between or accept the stop |
| Robot never finishes | The end heading is outside 2 degrees, or the pose is not reaching 3 cm. Check the pose and the drive method |
| A progress trigger never fires | The trigger watches another command object; use the one you schedule |

## Files

All in `frc.robot.Path`.

| File | Description |
|---|---|
| `PathConstants.java` | Every constant above |
| `Point.java` | The immutable point |
| `Path.java` | The path: corner geometry, stops, heading anchors, `flipped()` |
| `Segment.java`, `Line.java`, `Arc.java` | One piece of the path: base class, straight line, circle arc |
| `PathFollower.java` | Progress, speed, direction and heading per loop |
| `FollowPathCommand.java` | The command |
