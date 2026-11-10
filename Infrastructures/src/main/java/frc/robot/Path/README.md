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
> A plain-language walk-through of all the code, with worked numbers, is in
> [Path-explained.md](../../../../../../../docs/Path-explained.md).

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
| `.withRadius(meters)` | Corner circle radius for this point (`> 0`), used as is. Without it the radius is automatic (see [Corners](#corners)) |

The last point is always a stop. A path needs at least 2 points.

## Path files

`PathLoader.load("name")` reads `src/main/deploy/paths/name.json` (written by the path designer app) and returns a `Path`.
Load paths when the robot starts, not in auto: a bad file throws with the file name and point index.

```json
{ "version": 1,
  "limits": { "maxVelocity": 1.5, "maxAcceleration": 2.0, "maxOmegaDeg": 270 },
  "points": [ { "x": 1.0, "y": 2.0 },
              { "x": 3.0, "y": 2.0, "angleDeg": 90, "radius": 0.4 },
              { "x": 3.0, "y": 4.0, "stop": true, "dwell": 0.5 } ] }
```

`x`, `y` in meters (blue field), `angleDeg` in degrees, `radius` in meters, `dwell` in seconds. Only `x` and `y` are required.
Unknown fields are an error, so a typo is not silently ignored.

`limits` is optional, and so is each value in it: a value left out uses the `PathConfig` constant. In code, the same
is `path.withLimits(maxVelocity, maxAcceleration, maxOmega)` [m/s, m/s^2, rad/s]. Tolerances and
`MAX_RADIAL_ACCELERATION` stay global in `PathConfig`. The limits never change the path's shape: the automatic radius
uses the global `PathConfig` values.

`AutoLoader.load("name")` reads `src/main/deploy/autos/name.json` and returns the paths in order, each loaded with
`PathLoader`. The file is `{ "version": 1, "paths": [ "StartToNote", "NoteToShoot" ] }`. Chain them yourself, with a
`FollowPathCommand` per path in a `SequentialCommandGroup`. Each path should start where the last one ends: a path
whose start is more than 0.5 m from the robot (`START_TOLERANCE` in `FollowPathCommand`) ends at once.

## Corners

Each middle point is a corner. The robot drives along a circle that passes through the point.

- **Circle through the point.** Both turn directions work. A point with `withRadius` uses that radius unchanged.
- **Automatic radius.** Without `withRadius` the radius is `r = v^2 / a_r` = `MAX_VELOCITY^2 / MAX_RADIAL_ACCELERATION`
  (4/3 m by default), the tightest circle the robot can drive at full speed. When the points are too close for it
  (short legs, corners back to back), the automatic corners at and next to the one that does not fit shrink by 10% at a
  time, down to 5 cm (`MIN_AUTO_RADIUS`), until the path fits. This is done once, when the path is built.
- **Turn over 150 degrees** (`MAX_ARC_TURN`) or a stop point: a sharp corner; the over-150 case is an automatic stop with no dwell.
- **Points on a straight line** (turn about 0) make no arc; the point is only a pass marker (it still holds an angle).
- **An arc that does not fit** (points too close for the radius, or an arc that would sweep more than the turn
  plus 90 degrees, which happens with close same-direction corners), even with the automatic corners near it at
  5 cm, throws one short `IllegalArgumentException` that names the point. Move the points apart or use a smaller
  `withRadius`.

## Speed and following

- The speed is a profile along the path, not a clock. It accelerates with `MAX_ACCELERATION` and brakes with
  `MAX_ACCELERATION` by the distance left along the path to the next stop or the end.
- An arc of radius `radius` is capped at `sqrt(MAX_RADIAL_ACCELERATION * radius)`. The robot brakes before the arc
  with `MAX_ACCELERATION`. An automatic corner at full size is driven at `MAX_VELOCITY`.
- Progress follows the robot's projection onto the path. It never decreases, and segments switch by that
  projection, not by a distance threshold.
- If the robot is off the path, it returns on a smooth curve (merge over `MERGE_DISTANCE`, turning the travel direction toward the path at the same speed), not a straight line.
- The direction looks ahead `LAG_TIME` seconds to cancel the command delay.
- The commanded velocity vector changes by at most `(MAX_ACCELERATION + MAX_RADIAL_ACCELERATION) * DT` per loop.
- A NaN pose gives zero speeds.

## Headings

Points with `withAngle` are anchors. The start anchor is the robot's current heading unless the first point
has an angle. The reference angle is linear in distance along the path between anchors, along the shortest
way around, and holds the last angle. Omega = feed-forward from the reference slope + `HEADING_KP` * wrapped error,
limited to `MAX_OMEGA`.

## Alliance flip

`FollowPathCommand.initialize` reads `DriverStation.getAlliance()`. On Red it uses `path.flipped()`.
`PathConfig.MIRRORED_FIELD` picks the mode; set it per season:

- `false`: red is the blue field rotated 180 degrees about the field center: `(L - x, W - y)`, angle + pi.
- `true`: red is the blue field mirrored across `x = L / 2`: `(L - x, y)`, angle becomes `pi - angle`.

`L` and `W` are the field length and width of the current season from WPILib's AprilTag layout (`AprilTagFields.kDefaultField`, 16.541 and 8.069 m for 2026). Your pose must be in the same field coordinates on both alliances.

## Constants

Set these in `PathConfig.java` for your robot:

| Constant | Default | Meaning |
|---|---|---|
| `MAX_VELOCITY` | 2.0 m/s | Speed limit (default; a path can override it with `withLimits` or the file's `limits`) |
| `MAX_ACCELERATION` | 3.0 m/s^2 | Speed-up and braking along the path. The commanded velocity vector may change by `MAX_ACCELERATION + MAX_RADIAL_ACCELERATION` per second |
| `MAX_OMEGA` | 2 pi rad/s | Rotation speed limit |
| `MAX_RADIAL_ACCELERATION` | 3.0 m/s^2 | Acceleration toward a corner's center: sets the automatic radius `MAX_VELOCITY^2 / MAX_RADIAL_ACCELERATION` and the arc speed cap. It adds to the along-path acceleration, so keep it conservative and tune it on the robot |
| `POSITION_TOLERANCE` | 0.03 m | Position tolerance at a stop and at the end |
| `ANGLE_TOLERANCE` | 2 deg | Heading tolerance at the end |
| `MIRRORED_FIELD` | false | Flip mode (see above) |

The loop period is `TimedRobot.kDefaultPeriod`. The field size for the flip comes from the season's AprilTag layout.

These are private constants in the class that uses them; change them only when tuning:

| Constant (class) | Default | Meaning |
|---|---|---|
| `HEADING_KP` (`PathFollower`) | 4.0 1/s | Heading error gain |
| `FINAL_KP` (`PathFollower`) | 3.0 1/s | Gain near a stop, and on the normal error within `MERGE_DISTANCE` of a stop |
| `MERGE_DISTANCE` (`PathFollower`) | 0.5 m | Scale of the return curve to the path |
| `LAG_TIME` (`PathFollower`) | 0.05 s | Lookahead time for the direction |
| `STOP_SPEED` (`PathFollower`) | 0.1 m/s | Measured speed below which a stop counts as reached |
| `MIN_SPEED` (`PathFollower`) | 0.15 m/s | Lowest approach speed to a stop until within `POSITION_TOLERANCE / 2`, so a drive deadband can't stall it |
| `START_TOLERANCE` (`FollowPathCommand`) | 0.5 m | Largest allowed distance from the first point at start |
| `MAX_ARC_TURN` (`Path`) | 150 deg | Larger turns become an automatic stop |
| `MIN_AUTO_RADIUS` (`Path`) | 0.05 m | Smallest automatic corner radius before the path is rejected |
| `AUTO_RADIUS_SHRINK` (`Path`) | 0.9 | Factor an automatic radius shrinks by each time the path does not fit |

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
| Throws "corner arc does not fit" | Points too close even with the automatic corners near it shrunk to 5 cm, or a `withRadius` too large. Move the points apart or use a smaller `withRadius` |
| Command ends at once, error in the Driver Station | The robot is more than 0.5 m from the first point, or the pose is not set for this alliance |
| Robot drives the mirrored path on Red | `MIRRORED_FIELD` is wrong for this field, or your pose is in red-origin coordinates |
| Robot stops at a sharp corner | The turn is over 150 degrees. Add a point in between or accept the stop |
| Robot never finishes | The end heading is outside 2 degrees, or the pose is not reaching 3 cm. Check the pose and the drive method |
| A progress trigger never fires | The trigger watches another command object; use the one you schedule |

## Files

All in `frc.robot.Path`.

| File | Description |
|---|---|
| `PathConfig.java` | The values you set for your robot |
| `Point.java` | The immutable point |
| `Path.java` | The path: corner geometry, stops, heading anchors, `flipped()` |
| `Segment.java`, `Line.java`, `Arc.java` | One piece of the path: base class, straight line, circle arc |
| `PathFollower.java` | Progress, speed, direction and heading per loop |
| `FollowPathCommand.java` | The command |
| `PathLoader.java` | Reads a path file from `deploy/paths` |
| `AutoLoader.java` | Reads an auto file (a list of path names) from `deploy/autos` |
