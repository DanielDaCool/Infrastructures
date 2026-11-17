# DemaciaCodePaths

The path code from DemaciaCode `path-daniel` (769e744), copied into this repo so it can be reviewed and fixed
on its own. It lives in [`Infrastructures/src/main/java/frc/robot/Path`](Infrastructures/src/main/java/frc/robot/Path).
The only changes from DemaciaCode are the package (`frc.demacia.path` → `frc.robot.Path`), `Log.log` →
`System.out.println`, and the two fixes in 4ac6d73 listed below.

> [!WARNING]
> **Status: not run on a robot, not run under GradleRIO.** The fixes were checked only in a plain-Java harness
> with stand-ins for the WPILib classes. The constants are starting values.

This is a different design from the `Paths` branch (`Path`, `Point`, `FollowPathCommand`); the two share no code.

## Usage

Units are meters, seconds, radians. Points and speeds are field-relative. There is no alliance flip.

```java
DemaciaTrajectory trajectory = new DemaciaTrajectory(List.of(
    new PathPoint(new Translation2d(1, 2), Rotation2d.kZero),
    new PathPoint(new Translation2d(3, 2), Rotation2d.fromDegrees(90)),        // corner at MAX_VELOCITY
    new PathPoint(new Translation2d(3, 4), Rotation2d.fromDegrees(90), 0),     // velocity 0: stop here
    new PathPoint(new Translation2d(5, 4), Rotation2d.kZero)));

Command follow = new PathCommand(
    trajectory, swerve::getPose, swerve::getFieldSpeeds, swerve::driveFieldRelative, swerve);
```

- `PathPoint(translation, rotation[, velocity])`: `rotation` is the heading the robot turns to on the way to
  this point; `velocity` (default `MAX_VELOCITY`) is the speed through the point, and 0 means stop on it.
  The first and last points' velocities are ignored: the path always ends stopped.
- Pass the drive subsystem as a requirement. `PathCommand` takes requirements as varargs, so leaving it out
  compiles, but then the drive's default command runs at the same time.
- `PathCommand.initialize` calls `startTrajectory()`, so the same trajectory can be run again.

## How it drives

- **Corners.** Each middle point with a turn of at least `MIN_TURN_FOR_ARC` gets a corner arc of radius
  `v² / MAX_LATERAL_ACCELERATION`, capped at `MAX_RADIUS_TO_LEG_RATIO` × the shorter leg (the speed drops to fit).
  The circle passes through the point, so the path bulges slightly outside the drawn corner.
- **Lost arcs become stops (4ac6d73).** If an arc can't be built (no tangent leg) or would sweep more than
  `MAX_ARC_SWEEP`, it is removed and the point's velocity becomes 0, so the robot stops on that sharp corner
  instead of taking it at speed. A near-reversal (turn close to 180°) is also a stop.
- **Speed.** `DemaciaTrapezoid` accelerates at `MAX_ACCELERATION` and brakes to each segment's end velocity.
  End velocities are lowered from the back so every segment can still brake for the next.
- **Stops (4ac6d73).** A stop segment (end velocity 0) counts as reached within
  `MAX_POSITION_THRESHOLD_FINAL_POINT` (0.1 m). From there, the robot keeps following the trapezoid until its measured speed is under
  `MAX_VELOCITY_TO_CONSIDER_STOPPED`, instead of being commanded zero while still moving.
- **Heading.** A `ProfiledPIDController` (`HEADING_KP`, `MAX_ANGULAR_VELOCITY`, `MAX_ANGULAR_ACCELERATION`)
  turns toward the current segment's end heading. The command finishes at the end within `HEADING_TOLERANCE`.

## Constants

In `constants/PathConstants.java`:

| Constant | Value | Meaning |
|---|---|---|
| `MAX_VELOCITY` | 4 m/s | Speed limit, and the default point velocity |
| `MAX_ACCELERATION` | 3 m/s² | Speed-up and braking |
| `MAX_LATERAL_ACCELERATION` | 3 m/s² | Sets the corner radius from the corner speed |
| `MAX_ANGULAR_VELOCITY` | 2 rad/s | Heading profile speed limit |
| `MAX_ANGULAR_ACCELERATION` | 0.6 rad/s² | Heading profile acceleration limit |
| `HEADING_KP` | 1 (rad/s)/rad | Heading gain |
| `HEADING_TOLERANCE` | 5° | Heading tolerance at the end |
| `MIN_TURN_FOR_ARC` | 1° | Smaller turns get no arc |
| `MAX_ARC_SWEEP` | 270° | Larger arcs are removed (the point becomes a stop) |
| `MAX_RADIUS_TO_LEG_RATIO` | 0.25 | Arc radius ≤ this × the shorter leg |
| `MAX_VELOCITY_TO_CONSIDER_STOPPED` | 0.1 m/s | Measured speed under which a stop is done |
| `MAX_POSITION_THRESHOLD_FINAL_POINT` | 0.1 m | Distance at which a stop (end velocity 0) counts as reached |
| `MAX_POSITION_THRESHOLD_DURING_PATH` | 0.1 m | Distance at which any other segment's end counts as reached |
| `MAX_ANGLE_TO_PASS` | 60° | A line segment also counts as passed when the robot is this far off its direction |

## Known open issues

From the review of this branch (harness runs, not hardware). Not fixed yet:

- `DemaciaTrapezoid` jumps straight to the end velocity once under 0.08 s from it, a last step of about 19 m/s².
- Duplicate or very close points skip the arc but keep the corner speed, so the corner is taken at speed.
- A zero-length segment takes its direction as +x and can count as finished from far away.
- The trapezoid integrates from the measured speed, so drivetrain lag makes paths slower.
- A short leg forces a small radius and a slow corner (radius cap of 0.25 × the leg).
- A negative point velocity is not rejected.

## Files

| File | Description |
|---|---|
| `PathPoint.java` | A point: position, heading, speed through it |
| `DemaciaTrajectory.java` | Builds the segments and computes the speeds each loop |
| `segments/SegmentBase.java`, `LineSegment.java`, `ArcSegment.java` | One piece of the path |
| `calculator/ArcCalculator.java`, `LegCalculator.java` | Corner circle and tangent-leg math |
| `trapezoid/DemaciaTrapezoid.java` | Next speed from distance left |
| `commands/PathCommand.java` | The command |
| `constants/PathConstants.java` | The constants above |

## Build

From [`Infrastructures`](Infrastructures): `./gradlew build`. Needs WPILib (GradleRIO) and Java 17.
