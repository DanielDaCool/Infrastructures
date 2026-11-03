# Path

[![WPILib](https://img.shields.io/badge/WPILib-2026.2.1-blue)](https://docs.wpilib.org/)
[![Java](https://img.shields.io/badge/Java-17-orange)](https://adoptium.net/)
[![Vendor libraries](https://img.shields.io/badge/vendor%20libraries-none-lightgrey)](#requirements)

Autonomous paths for swerve robots. You write the points the robot must drive through. Path rounds
the corners, picks a heading and a speed for every part of the path, and a command drives the robot
along it. It uses the [Kinematics](Kinematics.md) module for the drivetrain and works with any pose
source, for example [RobotPose](RobotPose.md).

> [!WARNING]
> **Status: implemented; unit-tested in simulation (JUnit, not committed); not yet run on a robot.** The
> numbers are starting values from the design report, not tuned values.

> [!NOTE]
> This page is the student guide. The geometry, the heading and speed profiles, the follower and every
> setting are in [Path-design.md](Path-design.md), which also keeps the designs of features that v1 does not have.

## Contents

- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Quick start](#quick-start)
- [The points API](#the-points-api)
- [Headings](#headings)
- [Limits](#limits)
- [Alliance, progress, logging](#alliance-progress-logging)
- [Tuning order](#tuning-order)
- [Troubleshooting](#troubleshooting)
- [Project structure](#project-structure)

## Features

- **Every turning point is on the path.** The robot drives exactly through each point where the path turns, each stop
  and both ends. A point on a straight line is only a pass marker (`Path.warnings()` tells you if the path misses it).
- **Automatic rounded corners.** Each corner is a circle through the point. The radius is chosen for you, and
  you can set it per point.
- **Speed from the path, not from a clock.** The robot slows for corners and for the end, and speeds up
  on straight parts, within the limits you give, with a limit on jerk so the speed changes smoothly. A round corner is
  driven at one steady speed. `.withVelocity(v)` on a corner point caps that whole arc at `v`.
- **Stops where you ask.** `.stop()` on any middle point, with an optional wait.
- **Headings.** An angle on some points; the robot turns smoothly between them.
- **Progress.** `cmd.progress()` (0 to 1) and `cmd.distanceTraveled()` (m) for a `Trigger`.
- **Red alliance.** One setting flips every path (mirror or rotate), or turns flipping off.
- **Clear errors.** A wrong number stops the program with a sentence that says what to change.

## Requirements

| Dependency | Version |
|---|---|
| [WPILib / GradleRIO](https://docs.wpilib.org/) | 2026.2.1 |
| Java | 17 |
| [Kinematics](Kinematics.md) module | For the drivetrain (drive method with the acceleration limiter) |
| Vendor libraries | None |

## Installation

1. Copy `Infrastructures/src/main/java/frc/robot/Path` into your robot project, keeping the same package path.
2. Make sure your drivetrain uses the Kinematics module, and add `getFieldSpeeds()` (see below).

## Quick start

**1. Write the points.** Meters, on the blue-alliance side of the field (x toward the red wall, y to the left).

```java
Path path = new Path(new Point(1, 2), new Point(3, 4), new Point(5, 2));   // also: Path.of(List<Point>)
```

**2. Add `getFieldSpeeds()` to your drivetrain.** The follower needs the real speed of the robot. It is
already in `ExampleSwerveWithPose`, together with `follow(Path)` and `followCommand(Path)` (steps 3 and 4):

```java
public ChassisSpeeds getFieldSpeeds() {
    ChassisSpeeds measured = kinematics.toChassisSpeeds(getModuleStates(), getGyroRate());
    return ChassisSpeeds.fromRobotRelativeSpeeds(measured, heading());
}
```

**3. Write a method that makes the command.** A command can be used in only one composition, so make a new one each
time autonomous starts:

```java
KinematicsConfig kinematicsConfig = ...;      // the same one you give DemaciaKinematics
PathsConfig pathsConfig = PathsConfig.defaults();   // omega, alpha and jerk limits (placeholders)
PathFollowerConfig followerConfig = PathFollowerConfig.defaults();   // gains, tolerances, red-alliance flip (ROTATE)

FollowPathCommand followCommand(Path path) {
    return new FollowPathCommand(path.withConstraints(PathConstraints.fromConfig(kinematicsConfig, pathsConfig)),
            swerve::getPose, swerve::getFieldSpeeds, swerve::driveFieldRelative, swerve)
        .withConfig(followerConfig);
}
```

`swerve::driveFieldRelative` is the method that already has the acceleration limiter. The pose must be the
blue-origin pose also on the red alliance. Instead of `path.withConstraints(...)` you can call
`.withKinematics(kinematicsConfig)` and `.withPathsConfig(pathsConfig)` on the command.

**4. Use it in autonomous.** Set the start pose from the path for the current alliance (on red the command drives the
flipped path, so the start pose must be flipped too):

```java
return Commands.defer(() -> {
    Path active = isRed ? path.flipped(followerConfig) : path;   // flipped() returns a flipped copy
    Pose2d start = new Pose2d(active.startPoint(),
        active.startHeading().orElse(swerve.getPose().getRotation()));
    return Commands.runOnce(() -> swerve.setStartPose(start))
        .andThen(followCommand(path));                 // a new command every time; it flips the path itself
}, Set.of(swerve));
```

`ExampleSwerveWithPose.follow(Path)` does exactly this (its `isRedAlliance()` reads `DriverStation.getAlliance()`).

> [!IMPORTANT]
> - Set every option **before** you schedule the command. A change takes effect only the next time it starts.
> - The robot must start within 0.5 m of the first point (for the current alliance). Otherwise the command ends at once
>   with `FAILED_START_TOO_FAR`. A path cannot be started from the middle.
> - `Point`, `Path` and the config records never change. `p.withAngle(a)` returns a new point; write `p = p.withAngle(a)` or chain the
>   calls. (`KinematicsConfig.withX` is different: it changes the object, and `FollowPathCommand.withX` returns the same command.)

## The points API

| Call | What it does |
|---|---|
| `new Point(x, y)` | A point, meters. Also `new Point(Translation2d)` and `new Point(Pose2d)` (position and angle) |
| `.withAngle(Rotation2d)` | The robot faces this way at this point (see [Headings](#headings)) |
| `.withVelocity(v)` | At most `v` m/s at this point. On a round corner this caps the whole arc at `v` (the robot slows before the arc and speeds up after it) |
| `.withRadius(r)` | Use radius `r` m for this corner. `0` makes the corner a stop. Ignored (with a warning) on the first point, the last point, and a point exactly on a straight line |
| `.stop()` | Stop exactly on this point |
| `.stop(seconds)` | Stop, then wait that many seconds |
| `.withVelocity(0)` | Same as `.stop()` |

Examples:

```java
// headings on some points
new Path(new Point(1, 2).withAngle(Rotation2d.fromDegrees(0)),
         new Point(3, 4).withAngle(Rotation2d.fromDegrees(90)),
         new Point(5, 2));

// the whole arc at corner (3, 4) at most 1 m/s; the robot slows into it and speeds up after it
new Path(new Point(1, 2), new Point(3, 4).withVelocity(1.0), new Point(5, 2));

// stop on a point, wait 1.5 s, continue
new Path(new Point(1, 2), new Point(4, 2).stop(1.5), new Point(7, 4));
```

**A stop point is a sharp corner.** The robot stops exactly on the point and then continues. The path arrives
on a straight line and leaves on a straight line: there is no round arc at a stop. So `.stop()` and
`.withRadius(r)` (r above 0) on the same point is an error. Choose one: a round corner you drive through, or a
sharp corner you stop at.

Good to know:
- Corners are rounded by a circle that passes *through* the point, so the robot never cuts inside a corner.
  The path bulges outward by at most 0.5 m by default.
- A corner that is sharper than 150 degrees becomes a stop automatically (a warning tells you). Give it a radius
  if you want to drive through it: `.withRadius(0.3)`.
- The last point is always a stop. `.stop(1.0)` on it holds the robot for one more second.
- Two points less than 1 mm apart are merged (with a warning). If all points are at the same place, `new Path`
  throws an error.
- A point exactly on the straight line between its neighbors is only a pass marker: the path is built without it, and
  next to a round corner it can miss the point by a few cm (a warning tells you when it is more than 1 cm). Put `.stop()`
  on it if the robot must drive exactly through it.
- Build warnings are printed once with `DriverStation.reportWarning` and kept in `path.warnings()` (a `List<String>`).
- To start or end an action along the path, use `progress()`, not extra points on a straight line.

## Headings

- Points with `.withAngle(...)` are the heading anchors. Between two anchors the robot turns at a steady rate
  per meter of path, the short way (170 degrees then -170 degrees is a 20 degree turn).
- Before the first anchor the robot turns from the heading it has when the command starts. After the last anchor
  it keeps that angle. With no anchors at all it keeps its start heading.
- The heading never jumps, also not at a stop: if a stop point has an angle, the robot arrives with it. At a stop
  the robot only corrects a leftover error of more than 3 degrees before it waits.
- A turn of exactly 180 degrees between two anchors goes counter-clockwise. Avoid it if you use the mirror flip,
  or add an anchor in between.

## Limits

`PathConstraints` has seven numbers: `vMax` (m/s), `aAcc`, `aDec`, `aLat` (m/s^2), `omegaMax` (rad/s),
`alphaMax` (rad/s^2) and `jerkMax` (m/s^3). `PathConstraints.fromConfig(kinematicsConfig, pathsConfig)` computes
the first four from your `KinematicsConfig` (about 10% below the limiter's limits; with the default config 4.05 m/s
and 7.2 m/s^2). The last three come from `PathsConfig`, the Path module's own settings that `KinematicsConfig` does
not have: `maxAngularVelocity`, `maxAngularAccel` and `maxJerk` (placeholders 2 pi, 4 pi and 50 m/s^3; measure
them). `fromConfig(kinematicsConfig)` uses `PathsConfig.defaults()`. The jerk limit makes the speed change smoothly; it costs a few percent of time.

Change one for one path: `path.withConstraints(PathConstraints.fromConfig(c, pathsConfig).withVMax(2.0))`.
If a path has no constraints, the command uses `withKinematics(...)` / `withPathsConfig(...)`, and a default
`KinematicsConfig` / `PathsConfig.defaults()` for what is missing.

## Alliance, progress, logging

```java
// run something part way along the path: keep the command and use a Trigger
FollowPathCommand follow = followCommand(path);
new Trigger(() -> follow.progress() > 0.7).onTrue(intake.deployCommand());
// follow.distanceTraveled() > 2.0  also works (meters)

SmartDashboard.putData("Path", follow);   // or any logger that takes a Sendable
```

- `progress()` is 0 at the start and 1 at the end and never goes down during a run, so the trigger fires once.
  The trigger's command must not require the drivetrain, or it would cancel the path.
- Write all paths for the blue alliance. On red, the flip mode of `PathFollowerConfig` flips them once when the command
  starts: `PathFollowerConfig.FlipMode.ROTATE` (the default) or `FlipMode.MIRROR`; `FlipMode.NONE` never flips (for teams with separate red
  paths): `followerConfig.withFlip(FlipMode.MIRROR)`. The field size for the flip is `PathFollowerConfig.FIELD_LENGTH` x
  `FIELD_WIDTH` (16.54 x 8.07 m); for another field use `followerConfig.withField(length, width)`.
- `command.getResult()` returns a `FollowPathCommand.Result`: `RUNNING`, `OK`, `FAILED_TIMEOUT`, `FAILED_START_TOO_FAR`,
  `FAILED_POSE`, `INTERRUPTED` (`isSuccess()`, `isFailure()`). `withOnFinish(result -> ...)` is called once at the end.
  After a failure the robot gets zero speed once and the reason goes to the Driver Station with `DriverStation.reportWarning`.
  The robot gets zero speed also on a normal finish; to keep holding the end point actively, schedule your own hold command after it.
- The command is a `Sendable`. `SmartDashboard.putData` shows these keys next to the usual command keys:

| Key | What to look for |
|---|---|
| `state` | TRACK, CAPTURE_STOP, ROTATE, DWELL, CAPTURE_END, DONE, IDLE, or the failure/INTERRUPTED name |
| `segment` | Which line or arc of the path the robot is on |
| `s` | Progress along the path (m) |
| `eN` | Distance left (+) or right (-) of the path. Should be a few cm |
| `vRef`, `vMeas` | Commanded and measured speed. A big gap means a lagging drivetrain |
| `eThetaDeg` | Heading error |

## Tuning order

Do these in order. Change one thing at a time. Watch `eN` (above) on the dashboard. The gains and tolerances are
`PathFollowerConfig` settings, for example `PathFollowerConfig.defaults().withKN(3.5)`.
`kTheta` (4.0), `tauLag` (0.05 s) and `tSettle` (0.1 s) are constants in `PathFollowerConfig.java`, not settings: to
change one, edit the constant.

1. **Settings of the drivetrain first.** `KinematicsConfig` (`latency`, speed limits) must be right. Path does not
   fix a drivetrain that does not follow its commands.
2. **Measure the lag.** Drive a straight line, start the robot 0.2 m to the side, and look at how `eN` goes to zero.
   Fit it to `exp(-k_n * (t - lag))`, or compare `vRef` with `vMeas`. The design model value is 0.085 s (not
   measured).
3. **`kN`** (default 3.0). Raise it until `eN` overshoots on the step, then lower it by 25%. Stay below 4 until the lag
   is measured.
4. **`TAU_LAG`** (constant, 0.05 s). Drive a long arc at a steady speed `v` and take the mean `eN`. Use sizes without signs
   (`|eN|`, radius in meters):
   - robot runs wide (outside the arc): `tauLag = tauLag + kN * |mean eN| * radius / v^2`;
   - robot runs inside the arc: `tauLag = tauLag - kN * |mean eN| * radius / v^2`.

   Example: radius 1 m at 2 m/s (`v^2/radius = 4`), `kN = 3`, robot 0.043 m wide at `tauLag = 0.05`:
   `0.05 + 3 * 0.043 / 4 = 0.082 s`.
5. **`K_THETA`** (constant, 4.0) for the heading, using `eThetaDeg`.
6. **`kP`** (default 3.0, equal to `kN`). Raise both to 4 after step 2 if the finish takes too long.
7. **Limits.** Lower `aLat` if the robot slides on arcs. Check that the real speed `vMeas` follows `vRef`.
8. **Check the finish.** Default tolerances: 0.03 m, 2 degrees, 0.1 s. The timeout is 1.5 times the
   planned time plus 1 s (waits at stops are included).

## Troubleshooting

Warnings and failures go to the Driver Station (`DriverStation.reportWarning`). Build warnings start with `Path "name", point i:`.

| Symptom | Likely cause |
|---|---|
| Program stops with `Path needs at least 2 points ...` | `new Path(...)` got fewer than two points |
| `All ... points of the path are at the same place` | Every point is within 1 mm of the first; check the coordinates |
| `new Path: point 1 is null; pass a Point for every position (for example new Point(1, 2)).` (any `... is null; pass ...` text) | A null argument: pass the thing named in the text |
| `PathConstraints.vMax must be a finite number above 0 (m/s), but it is -1.0. Start from PathConstraints.fromConfig(config) and change one value, for example PathConstraints.fromConfig(config).withVMax(3.0).` (same form for `PathsConfig` and `PathFollowerConfig`) | A config value is zero, negative, NaN or infinite; start from the factory named in the text |
| `Point(NaN, ...)` error | A division by zero or a value never set in the coordinates |
| `Point 1 has both stop() and withRadius(0.5). A stop point is a sharp corner ...` | A stop point is a sharp corner; remove one of the two |
| `withVelocity(-1.0): the speed must be 0 or more (m/s). withVelocity(0) is the same as stop(). To limit the speed at a point use a positive number, for example withVelocity(1.0).` (`stop(...)` and `withRadius(...)` are alike) | A negative number. Use `withVelocity(0)` or `stop()` to stop |
| Warning: `the corner is too sharp or too tight for an arc (...). The robot will stop here. ...` | Turn sharper than 150 degrees, or no arc fits. Move the points apart or use `.withRadius(r)` |
| Warning: `radius 5 m does not fit and was shrunk to 1.3107 m because a neighbor point is too close to the circle. Move the points farther apart, or give a smaller radius, for example withRadius(1.3107).` | The radius you asked for does not fit between the neighbors |
| Warning: `withRadius(0.5) is ignored here: only a point where the path turns has a corner radius. Remove withRadius from this point.` (also `withVelocity(...)` and `stop() is ignored here ...` on the first or last point) | `withRadius` on the first or last point or on a straight-line point, or `withVelocity` / `stop()` on the first or last point |
| Warning: `this point is on a straight line between its neighbors, so it is only a pass marker, and the path misses it by ... m ...` | The point is on a straight line and the path misses it; add `.stop()` there or remove the point |
| Robot drives a whole corner slowly | `withVelocity` caps the whole arc; or the heading must turn too fast at the corner point (a big angle change there). Spread the turn over more path or use a larger radius |
| Robot runs wide on arcs | Lookahead `TAU_LAG` too small (tuning step 4), or `latency` in `KinematicsConfig` is wrong |
| Robot weaves around the path | `kN` too high for the lag (tuning steps 2 and 3) |
| Robot ends a little short or late | `kP` low, or the finish tolerance is tight; see `eN` and `s` |
| Warning: `Path "name": alliance unknown; driving the blue path. For a red run, connect the Driver Station (or set the alliance in the simulator) before the command starts.` | The alliance was not known when the command started |
| Warning: `Path "name": the measured speed is NaN or infinite; check the speed supplier you passed to FollowPathCommand (for example swerve::getFieldSpeeds) and that the gyro and encoders are connected.` | The measured-speed supplier returned NaN or infinity |
| `FAILED_TIMEOUT` | The robot was blocked, or the limits are above what the drivetrain can do. The message says: `free the robot, or lower the limits, for example constraints.withVMax(2.0)` |
| `FAILED_START_TOO_FAR` | The robot was more than 0.5 m from the first point: wrong alliance flip, wrong path, or the start pose was not set. The message says: `put the robot on the start point, check setStartPose and the alliance flip, or raise the tolerance, for example PathFollowerConfig.defaults().withStartTol(1.0)` |
| `FAILED_POSE` | The pose was NaN or infinite for more than 1 s (or at the start); check the pose supplier |
| Robot spins the wrong way | The drivetrain flips omega for `toSwerveModuleStates` once; do not flip it again |
| Red alliance drives a mirrored path | Wrong `FlipMode` for this field, or a pose supplier in red-origin coordinates |
| A progress trigger never fires | The robot never reached that fraction, or the trigger watches an old command object (make the trigger from the same command you schedule) |
| Robot stops at every straight-line point | A point has `.stop()`; points exactly on a line are not stops otherwise |
| Robot starts slowly | The limiter and the module steering (up to 0.13 s); this is expected |

## Project structure

All paths are relative to `Infrastructures/src/main/java/frc/robot/Path` (`frc.robot.Path`). Eight files.

| File | Description |
|---|---|
| `Point.java`, `Path.java` | The points and the path (geometry is built in the constructor; `warnings()`, `flipped(...)`) |
| `PathConstraints.java` | The seven limits and `fromConfig` |
| `FollowPathCommand.java` | The command, `FollowPathCommand.Result`, `progress()`, Sendable keys |
| `PathsConfig.java`, `PathFollowerConfig.java` | Robot settings, and follower settings (gains, tolerances, timeout, flip, field size, `FlipMode`), with defaults |
| `PathGeometry.java`, `SpeedProfile.java` | Internal (corner geometry, heading and speed profiles); not for use in robot code |

The full design is in [Path-design.md](Path-design.md).
