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

- **Every point is on the path.** The robot drives exactly through each point you write.
- **Automatic rounded corners.** Each corner is a circle through the point. The radius is chosen for you, and
  you can set it per point.
- **Speed from the path, not from a clock.** The robot slows for corners and for the end, and speeds up
  on straight parts, within the limits you give, with a limit on jerk so the speed changes smoothly.
- **Stops where you ask.** `.stop()` on any middle point, with an optional wait.
- **Headings.** An angle on some points (the robot turns smoothly between them), or "always face this spot".
- **Events at a distance, a fraction, or a point.** Run a command when the robot gets there.
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
FlipConfig flip = FlipConfig.rotate();

FollowPathCommand follow(Path path) {
    return new FollowPathCommand(path.withConstraints(PathConstraints.fromConfig(kinematicsConfig, pathsConfig)),
            swerve::getPose, swerve::getFieldSpeeds, swerve::driveFieldRelative, swerve)
        .withKinematics(kinematicsConfig)
        .withPathsConfig(pathsConfig)    // module radius for the wheel-speed cap
        .withFlip(flip);
}
```

`swerve::driveFieldRelative` is the method that already has the acceleration limiter. The pose must be the
blue-origin pose also on the red alliance.

**4. Use it in autonomous.** Set the start pose from the path for the current alliance (on red the command drives the
flipped path, so the start pose must be flipped too), and read the alliance when autonomous starts:

```java
return Commands.defer(() -> {
    Path active = path.forCurrentAlliance(flip);           // blue: the path itself; red: the flipped path
    Pose2d start = new Pose2d(active.startPoint(),
        active.startHeading().orElse(swerve.getPose().getRotation()));
    return Commands.runOnce(() -> swerve.setStartPose(start))
        .andThen(follow(path));                            // a new command every time; it flips the path itself
}, Set.of(swerve));
```

> [!IMPORTANT]
> - Set every option **before** you schedule the command. Setting one later throws an error.
> - Value objects never change. `p.withAngle(a)` returns a new point; write `p = p.withAngle(a)` or chain the
>   calls. (`KinematicsConfig.withX` is different: it changes the object.)

## The points API

| Call | What it does |
|---|---|
| `new Point(x, y)` | A point, meters |
| `.withAngle(Rotation2d)` | The robot faces this way at this point (see [Headings](#headings)) |
| `.withVelocity(v)` | At most `v` m/s *at this point*. It is a limit at one point, not a slow zone |
| `.withRadius(r)` | Use radius `r` m for this corner. `0` makes the corner a stop. Ignored (with a warning) on the first point, the last point, and a point exactly on a straight line |
| `.stop()` | Stop exactly on this point |
| `.stop(seconds)` | Stop, then wait that many seconds |
| `.withVelocity(0)` | Same as `.stop()` |
| `.cutCorner()` | Not available yet |

Examples:

```java
// headings on some points
new Path(new Point(1, 2).withAngle(Rotation2d.fromDegrees(0)),
         new Point(3, 4).withAngle(Rotation2d.fromDegrees(90)),
         new Point(5, 2));

// slow through a gap: pass the middle point at 1 m/s or less
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
// face the hub the whole way (shooter at the back: add Rotation2d.kPi)
path.withFaceTarget(new Translation2d(4.6, 4.0));

// events: a Command or a Runnable
path.withEventAtDistance(1.0, intake.deployCommand())
    .withEventAtFraction(0.5, Commands.print("halfway"))
    .withEventAtPoint(2, () -> shooter.spinUp());
```

- With a face target, the point angles are not used (a warning tells you). Within 0.5 m of the target the
  heading stops following it.
- An event at a stop point fires when the robot **arrives**, then the robot waits.
- Event commands must not need the drivetrain, or they would cancel the path (this is an error when you create
  the command).
- Write all paths for the blue alliance. On red, `FlipConfig.rotate()` (the default) or `FlipConfig.mirror()`
  flips them once when the command starts; `FlipConfig.none()` never flips (for teams with separate red paths).
  Choose the one that matches the field.
- `command.getResult()` tells how it ended: `OK`, `FAILED_TIMEOUT`, `FAILED_START_TOO_FAR`, `FAILED_POSE`,
  `INTERRUPTED`. After a failure the robot gets zero speed once. To keep holding the end point actively, add
  `FollowPathCommand f = follow(path); f.andThen(f.holdEndCommand())`, with `f` made fresh in the same
  supplier.

## Tuning order

Do these in order. Change one thing at a time. Watch `Path/<name>/eN` (how far the robot is left of the path)
on the dashboard.

1. **Settings of the drivetrain first.** `KinematicsConfig` (`latency`, speed limits) must be right. Path does not
   fix a drivetrain that does not follow its commands.
2. **Measure the lag.** Drive a straight line, start the robot 0.2 m to the side, and look at how `eN` goes to zero.
   Fit it to `exp(-k_n * (t - lag))`, or compare `vCmdNorm` with `vMeas`. The planned model value is 0.085 s (not
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
7. **Limits.** Lower `aLat` if the robot slides on arcs. Check that the real speed `vMeas` follows `vCmd`.
8. **Check the finish.** Default tolerances: 0.03 m, 2 degrees, 0.1 s. The timeout is 1.5 times the
   planned time plus 1 s (waits at stops are included).

## Telemetry

Published under `Path/<name>/` (set the name with `path.withName("score")`). The main keys:

| Key | What to look for |
|---|---|
| `state` | TRACK, CAPTURE_STOP, ROTATE, DWELL, CAPTURE_END, DONE, or FAILED_* |
| `eN` | Distance left (+) or right (-) of the path. Should be a few cm |
| `eT`, `dGo` | Along-track error, distance to the next stop or the end |
| `vCmd`, `vMeas` | Commanded and measured speed. A big gap means a lagging drivetrain |
| `eThetaDeg` | Heading error |
| `s`, `L` | Progress and path length |
| `result` | How the command ended |
| `field` | The path and the robot's target point on a Field2d |

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
| Warning: withRadius is ignored | It was on the first or last point, or on a point exactly on a straight line |
| Robot runs wide on arcs | `tauLag` too small (tuning step 4), or `latency` in `KinematicsConfig` is wrong |
| Robot weaves around the path | `kN` too high for the lag (tuning steps 2 and 3) |
| Robot ends a little short or late | `kP` low, or the finish tolerance is tight; see `eN`, `dGo` |
| Result `FAILED_TIMEOUT` | The robot was blocked, or `aAcc` is above what the limiter allows |
| Result `FAILED_START_TOO_FAR` | The pose is more than 3 m from the path: wrong alliance flip, wrong path, or the start pose was not set |
| Robot spins the wrong way | The drivetrain flips omega for `toSwerveModuleStates` once; do not flip it again |
| Red alliance drives a mirrored path | Wrong `FlipMode` for this field, or a pose supplier in red-origin coordinates |
| An event never fires | The robot never reached that distance, or it was added after the command was created |
| Robot stops at every straight-line point | A point has `.stop()`; points exactly on a line are not stops otherwise |
| Robot starts slowly | The limiter and the module steering (up to 0.13 s); this is expected |

## Project structure

All paths are relative to `Infrastructures/src/main/java/frc/robot/Path` (planned).

| Path | Description |
|---|---|
| `Point.java`, `Path.java` | The points and the path (geometry is built in the constructor) |
| `PathConstraints.java` | The seven limits and `fromConfig` |
| `FollowPathCommand.java`, `FollowResult.java` | The command and how it ended |
| `PathEvent.java`, `ResolvedEvent.java`, `FaceTarget.java` | Events and face-target mode |
| `FlipConfig.java`, `FlipMode.java`, `FieldDimensions.java` | Red-alliance flip |
| `PathGeometryConfig.java`, `ProfileConfig.java`, `FollowerConfig.java` | Settings, with defaults |
| `PathLogger.java`, `PathSample.java` | Logging, read-only samples |
| other files | Internal (geometry, heading and speed profiles, follower); not for use in robot code |

The full design is in [Path-design.md](Path-design.md).
