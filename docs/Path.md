# Path

[![WPILib](https://img.shields.io/badge/WPILib-2026.2.1-blue)](https://docs.wpilib.org/)
[![Java](https://img.shields.io/badge/Java-17-orange)](https://adoptium.net/)
[![Vendor libraries](https://img.shields.io/badge/vendor%20libraries-none-lightgrey)](#requirements)

Autonomous paths for swerve robots. You write the points the robot must drive through. Path rounds
the corners, picks a heading and a speed for every part of the path, and a command drives the robot
along it. It uses the [Kinematics](Kinematics.md) module for the drivetrain and works with any pose
source, for example [RobotPose](RobotPose.md).

> [!WARNING]
> **Planned API, not built or tested.** Everything below describes the *planned* API. Nothing here has
> been compiled, run in simulation or tested on a robot, and names may still change. The numbers are
> starting values from the design report, not tuned values.

> [!NOTE]
> This page is the student guide. The geometry, the heading and speed profiles, the follower, every
> setting and every error message are in [Path-design.md](Path-design.md).

## Contents

- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Quick start](#quick-start)
- [The points API](#the-points-api)
- [Headings](#headings)
- [Limits](#limits)
- [Face a target, events, alliance](#face-a-target-events-alliance)
- [Tuning order](#tuning-order)
- [Telemetry](#telemetry)
- [Troubleshooting](#troubleshooting)
- [Project structure](#project-structure)

## Features

- **Every turning point is on the path.** The robot drives exactly through each point where the path turns, each stop
  and both ends. A point on a straight line is only a pass marker (a warning tells you if the path misses it).
- **Automatic rounded corners.** Each corner is a circle through the point. The radius is chosen for you, and
  you can set it per point.
- **Speed from the path, not from a clock.** The robot slows for corners and for the end, and speeds up
  on straight parts, within the limits you give, with a limit on jerk so the speed changes smoothly. Round corners are
  driven at a steady speed, except a corner whose point has `.withVelocity(v)`: there the robot slows down to at most `v` at the
  point and speeds up after it.
- **Stops where you ask.** `.stop()` on any middle point, with an optional wait.
- **Headings.** An angle on some points (the robot turns smoothly between them), or "always face this spot".
- **Events at a fraction of the path or at a point.** Run a command when the robot gets there.
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
Path path = new Path(new Point(1, 2), new Point(3, 4), new Point(5, 2));
```

**2. Add `getFieldSpeeds()` to your drivetrain.** The follower needs the real speed of the robot. In
`ExampleSwerveWithPose` the same two lines are in `periodic()` (lines 335-336):

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
PathsConfig pathsConfig = PathsConfig.fromModules(modules);   // same module array as the drivetrain
PathFollowerConfig followerConfig = PathFollowerConfig.defaults();   // gains, tolerances, red-alliance flip (ROTATE)

FollowPathCommand follow(Path path) {
    return new FollowPathCommand(path.withConstraints(PathConstraints.fromConfig(kinematicsConfig, pathsConfig)),
            swerve::getPose, swerve::getFieldSpeeds, swerve::driveFieldRelative, swerve)
        .withKinematics(kinematicsConfig)
        .withPathsConfig(pathsConfig)    // module radius for the wheel-speed cap
        .withConfig(followerConfig);
}
```

`swerve::driveFieldRelative` is the method that already has the acceleration limiter. The pose must be the
blue-origin pose also on the red alliance.

**4. Use it in autonomous.** Set the start pose from the path for the current alliance (on red the command drives the
flipped path, so the start pose must be flipped too), and read the alliance when autonomous starts:

```java
return Commands.defer(() -> {
    Path active = path.forCurrentAlliance(followerConfig); // blue: the path itself; red: the flipped path
    Pose2d start = new Pose2d(active.startPoint(),
        active.startHeading().orElse(swerve.getPose().getRotation()));
    return Commands.runOnce(() -> swerve.setStartPose(start))
        .andThen(follow(path));                            // a new command every time; it flips the path itself
}, Set.of(swerve));
```

> [!IMPORTANT]
> - Set every option **before** you schedule the command. A change takes effect only the next time it starts.
> - The robot must start within 0.5 m of the first point (for the current alliance). Otherwise the command ends at once
>   with `FAILED_START_TOO_FAR`. A path cannot be started from the middle.
> - Value objects never change. `p.withAngle(a)` returns a new point; write `p = p.withAngle(a)` or chain the
>   calls. (`KinematicsConfig.withX` is different: it changes the object.)

## The points API

| Call | What it does |
|---|---|
| `new Point(x, y)` | A point, meters |
| `.withAngle(Rotation2d)` | The robot faces this way at this point (see [Headings](#headings)) |
| `.withVelocity(v)` | At most `v` m/s at this point. If it is faster, the robot slows down before the point and speeds up after it, also on a round corner (that corner then uses about 90% of its grip limit) |
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

// at most 1 m/s at the corner point (3, 4); the robot slows into it and speeds up after it
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
- To start or end an action along the path, use events, not extra points on a straight line.

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
them), plus `moduleRadius` for the wheel-speed cap. The jerk limit makes the speed change smoothly; it costs a few percent of time.

Change one for one path: `path.withConstraints(PathConstraints.fromConfig(c, pathsConfig).withVMax(2.0))`.

## Face a target, events, alliance

```java
// the robot's front faces the hub the whole way
path.withFaceTarget(new Translation2d(4.6, 4.0));

// events: at a fraction of the path or at a point; a Command or a Runnable
path.withEventAtFraction(0.2, intake.deployCommand())
    .withEventAtFraction(0.5, Commands.print("halfway"))
    .withEventAtPoint(2, () -> shooter.spinUp());
```

- With a face target, the point angles are not used (a warning tells you). Within 0.5 m of the target the
  heading stops following it.
- An event at a stop point fires when the robot **arrives**, then the robot waits.
- Event commands must not need the drivetrain, or they would cancel the path (this is an error when you create
  the command).
- Write all paths for the blue alliance. On red, the flip mode of `PathFollowerConfig` flips them once when the command
  starts: `FlipMode.ROTATE` (the default) or `FlipMode.MIRROR`; `FlipMode.NONE` never flips (for teams with separate red
  paths). Choose the one that matches the field: `followerConfig.withFlip(FlipMode.MIRROR)`.
- `command.getResult()` tells how it ended: `OK`, `FAILED_TIMEOUT`, `FAILED_START_TOO_FAR`, `FAILED_POSE`,
  `INTERRUPTED`. After a failure the robot gets zero speed once. To keep holding the end point actively, add
  `FollowPathCommand f = follow(path); f.andThen(f.holdEndCommand())`, with `f` made fresh in the same
  supplier.

## Tuning order

Do these in order. Change one thing at a time. Watch `eN` in `Path/<name>` (how far the robot is left of the path)
on the dashboard. The gains and tolerances are `PathFollowerConfig` settings, for example
`PathFollowerConfig.defaults().withKN(3.5).withTauLag(0.07)`.

1. **Settings of the drivetrain first.** `KinematicsConfig` (`latency`, speed limits) must be right. Path does not
   fix a drivetrain that does not follow its commands.
2. **Measure the lag.** Drive a straight line, start the robot 0.2 m to the side, and look at how `eN` goes to zero.
   Fit it to `exp(-k_n * (t - lag))`, or compare `vRef` with `vMeas`. The planned model value is 0.085 s (not
   measured).
3. **`kN`** (default 3.0). Raise it until `eN` overshoots on the step, then lower it by 25%. Stay below 4 until the lag
   is measured.
4. **`tauLag`** (default 0.05 s). Drive a long arc at a steady speed `v` and take the mean `eN`. Use sizes without signs
   (`|eN|`, radius in meters):
   - robot runs wide (outside the arc): `tauLag = tauLag + kN * |mean eN| * radius / v^2`;
   - robot runs inside the arc: `tauLag = tauLag - kN * |mean eN| * radius / v^2`.

   Example: radius 1 m at 2 m/s (`v^2/radius = 4`), `kN = 3`, robot 0.043 m wide at `tauLag = 0.05`:
   `0.05 + 3 * 0.043 / 4 = 0.082 s`.
5. **`kTheta`** (default 4.0) for the heading, using `eThetaDeg`.
6. **`kP`** (default 3.0, equal to `kN`). Raise both to 4 after step 2 if the finish takes too long.
7. **Limits.** Lower `aLat` if the robot slides on arcs. Check that the real speed `vMeas` follows `vRef`.
8. **Check the finish.** Default tolerances: 0.03 m, 2 degrees, 0.1 s. The timeout is 1.5 times the
   planned time plus 1 s (waits at stops are included).

## Telemetry

The command publishes one entry, `Path/<name>` (set the name with `path.withName("score")`), through `PathLogger`, which
goes to SmartDashboard by default. Its keys:

| Key | What to look for |
|---|---|
| `state` | TRACK, CAPTURE_STOP, ROTATE, DWELL, CAPTURE_END, DONE, or FAILED_* |
| `segment` | Which line or arc of the path the robot is on |
| `s` | Progress along the path (m) |
| `eN` | Distance left (+) or right (-) of the path. Should be a few cm |
| `vRef`, `vMeas` | Commanded and measured speed. A big gap means a lagging drivetrain |
| `eThetaDeg` | Heading error |

Warnings and failures show as alerts. To send everything to your team's logger instead, call `PathLogger.set(...)` once
in `Robot`'s constructor, the same way as `PoseLogger` in RobotPose (example in [Path-design.md](Path-design.md), section 9.11).
Path adds no logging library.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Program stops with `Path needs at least 2 points ...` | `new Path(...)` got fewer than two points |
| `All ... points of the path are at the same place` | Every point is within 1 mm of the first; check the coordinates |
| `Point(NaN, ...)` error | A division by zero or a value never set in the coordinates |
| `... has both stop() and withRadius(...)` | A stop point is a sharp corner; remove one of the two |
| `withVelocity(...)` / `stop(...)` error | A negative number. Use `withVelocity(0)` or `stop()` to stop |
| Warning: corner became a stop | Turn sharper than 150 degrees, or no arc fits. Move the points apart or use `.withRadius(r)` |
| Warning: radius shrunk | The radius you asked for does not fit between the neighbors |
| Warning: ... is ignored | `withRadius` on the first or last point or on a straight-line point, `withVelocity` on the first or last point, or `withAngle` while a face target is set |
| Warning: point is only a pass marker | The point is on a straight line and the path misses it; add `.stop()` there or remove the point |
| Robot drives a whole corner slowly | The heading must turn too fast at the corner point (a big angle change there); spread the turn over more path or use a larger radius. `withVelocity` slows the robot only near its point |
| Robot runs wide on arcs | `tauLag` too small (tuning step 4), or `latency` in `KinematicsConfig` is wrong |
| Robot weaves around the path | `kN` too high for the lag (tuning steps 2 and 3) |
| Robot ends a little short or late | `kP` low, or the finish tolerance is tight; see `eN` and `s` |
| Result `FAILED_TIMEOUT` | The robot was blocked, or `aAcc` is above what the limiter allows |
| Result `FAILED_START_TOO_FAR` | The robot was more than 0.5 m from the first point: wrong alliance flip, wrong path, or the start pose was not set |
| Robot spins the wrong way | The drivetrain flips omega for `toSwerveModuleStates` once; do not flip it again |
| Red alliance drives a mirrored path | Wrong `FlipMode` for this field, or a pose supplier in red-origin coordinates |
| An event never fires | The robot never reached that point or fraction, or it was added after the command was created |
| Robot stops at every straight-line point | A point has `.stop()`; points exactly on a line are not stops otherwise |
| Robot starts slowly | The limiter and the module steering (up to 0.13 s); this is expected |

## Project structure

All paths are relative to `Infrastructures/src/main/java/frc/robot/Path` (planned).

| Path | Description |
|---|---|
| `Point.java`, `Path.java` | The points and the path (geometry is built in the constructor) |
| `PathConstraints.java` | The seven limits and `fromConfig` |
| `FollowPathCommand.java`, `FollowResult.java` | The command and how it ended |
| `PathEvent.java`, `ResolvedEvent.java` | Events |
| `FlipMode.java` | Red-alliance flip mode |
| `PathsConfig.java`, `PathFollowerConfig.java` | Robot settings, and follower settings (gains, tolerances, timeout, flip), with defaults |
| `PathLogger.java`, `PathSample.java` | Logging, read-only samples |
| other files | Internal (geometry, heading and speed profiles, follower); not for use in robot code |

The full design is in [Path-design.md](Path-design.md).
