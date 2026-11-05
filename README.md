# Path

[![WPILib](https://img.shields.io/badge/WPILib-2026.2.1-blue)](https://docs.wpilib.org/)
[![Java](https://img.shields.io/badge/Java-17-orange)](https://adoptium.net/)
[![Vendor libraries](https://img.shields.io/badge/vendor%20libraries-none-lightgrey)](#requirements)

A distance-based path planner and follower for FRC swerve robots. You give it the points the
robot must drive through. The path passes through every point where it turns (each corner, stop
and end point), with rounded arcs at the corners. A command drives the robot along it, using the
pose and field speeds from your own drivetrain.

> [!WARNING]
> **Status: implemented, unit-tested in simulation, not yet run on a robot.** The default
> numbers are starting values, not tuned values.

> [!NOTE]
> The student guide (full API, headings, limits, tuning, troubleshooting) is
> [Infrastructures/src/main/java/frc/robot/Path/README.md](Infrastructures/src/main/java/frc/robot/Path/README.md).
> The geometry, profiles and every setting are in [docs/Path-design.md](docs/Path-design.md).

## Contents

- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Build](#build)
- [Other modules](#other-modules)
- [Project structure](#project-structure)
- [License](#license)

## Requirements

| Dependency | Version |
|---|---|
| [WPILib / GradleRIO](https://docs.wpilib.org/) | 2026.2.1 |
| Java | 17 |
| Vendor libraries | None |

Path needs no other module from this repository.

## Installation

1. Copy [`Infrastructures/src/main/java/frc/robot/Path`](Infrastructures/src/main/java/frc/robot/Path)
   into your robot project, keeping the package path `frc.robot.Path`.
2. Make your drivetrain implement `PathDrivetrain` (see step 1 below).

## Usage

### 1. Implement `PathDrivetrain`

The follower needs the pose, the measured field-relative speeds, and a way to drive the robot.

```java
public class Swerve extends SubsystemBase implements PathDrivetrain {
    @Override public Pose2d getPose() { ... }                 // blue-origin pose, also on the red alliance
    @Override public ChassisSpeeds getFieldSpeeds() { ... }   // measured, field-relative, omega counter-clockwise
    @Override public void driveFieldRelative(ChassisSpeeds speeds) { ... }   // keep your acceleration limiter here

    // optional: limits used when a path has no constraints of its own (default: PathConstraints.defaults())
    @Override public PathConstraints getPathConstraints() { return myConstraints; }
}
```

### 2. Build a path

Points are in meters, on the blue-alliance side of the field.

```java
Path path = new Path(new Point(1, 2), new Point(3, 4), new Point(5, 2));   // or Path.of(List<Point>)
```

| Call | What it does |
|---|---|
| `new Point(x, y)` | A point. Also `new Point(Translation2d)` and `new Point(Pose2d)` |
| `.withAngle(Rotation2d)` | The robot faces this way at this point. The robot turns smoothly between angle points |
| `.withVelocity(v)` | At most `v` m/s here. On a corner, caps the whole arc at `v` |
| `.withRadius(r)` | Corner radius `r` m. `0` makes the corner a stop |
| `.stop()` | Stop exactly on this point |
| `.stop(seconds)` | Stop, then wait that many seconds |

```java
// heading on a point, and a slow corner
Path path = new Path(
    new Point(1, 2).withAngle(Rotation2d.fromDegrees(0)),
    new Point(3, 4).withVelocity(1.0),
    new Point(5, 2));

// stop on a point for 1.5 s, then continue
Path stopping = new Path(new Point(1, 2), new Point(4, 2).stop(1.5), new Point(7, 4));
```

A stop point is a sharp corner: the path arrives and leaves on straight lines. Use `.stop()` or
`.withRadius(r)` with `r > 0` on a point, not both.

`Point` and `Path` never change. `withAngle` and the other `withX` calls return a new object, so
write `p = p.withAngle(a)` or chain the calls.

### 3. Create and run the command

Make a new command each time autonomous starts. A command can be used in only one composition.

```java
FollowPathCommand follow = new FollowPathCommand(swerve, path);   // requires swerve
```

The robot limits come from `getPathConstraints()` unless the path has its own, set with
`path.withConstraints(PathConstraints.defaults().withVMax(2.0))`.

### 4. Alliance flip

Write every path for the blue alliance. On red, the command flips the path once when it starts.
The flip mode is in `PathFollowerConfig`:

```java
PathFollowerConfig followerConfig = PathFollowerConfig.defaults()
    .withFlip(PathFollowerConfig.FlipMode.MIRROR);   // ROTATE is the default; NONE never flips

FollowPathCommand follow = new FollowPathCommand(swerve, path).withConfig(followerConfig);
```

For another field size, use `followerConfig.withField(lengthMeters, widthMeters)`. The default
is `PathFollowerConfig.FIELD_LENGTH` x `PathFollowerConfig.FIELD_WIDTH` (16.54 x 8.07 m).

Set every option before you schedule the command. The start pose must match the path for the
current alliance, so flip it too. `Path.flipped(followerConfig)` returns a flipped copy:

```java
return Commands.defer(() -> {
    Path active = isRed ? path.flipped(followerConfig) : path;
    Pose2d start = new Pose2d(active.startPoint(),
        active.startHeading().orElse(swerve.getPose().getRotation()));
    return Commands.runOnce(() -> swerve.setStartPose(start))   // your method: reset your pose estimator
        .andThen(new FollowPathCommand(swerve, path).withConfig(followerConfig));
}, Set.of(swerve));
```

`isRed` and `setStartPose` are your own code. `isRed` can read `DriverStation.getAlliance()`.

### 5. Progress triggers and logging

`progress()` goes from 0 at the start to 1 at the end and never decreases during a run.
`distanceTraveled()` returns meters.

```java
FollowPathCommand follow = new FollowPathCommand(swerve, path);
new Trigger(() -> follow.progress() > 0.7).onTrue(intake.deployCommand());   // your command
```

The trigger's command must not require the drivetrain, or it cancels the path.

`FollowPathCommand` is a `Sendable`, so you can log it to the dashboard:

```java
SmartDashboard.putData("Path", follow);
```

`command.getResult()` returns a `FollowPathCommand.Result`. `withOnFinish(result -> ...)` runs
once at the end. Build warnings are available from `path.warnings()`.

## Build

Run these from the [`Infrastructures`](Infrastructures) directory:

```
./gradlew build
./gradlew simulateJava
```

On Windows, set `JAVA_HOME` to the WPILib 2026 JDK before running Gradle.

## Other modules

Each module is on its own branch and has its own README:

- [Kinematics](https://github.com/DanielDaCool/Infrastructures/tree/Kinematics)
- [RobotPose](https://github.com/DanielDaCool/Infrastructures/tree/RobotPose)

## Project structure

```
.
├── README.md
├── docs/
│   └── Path-design.md                 Design notes
└── Infrastructures/src/main/java/frc/robot/
    ├── Path/
    │   ├── README.md                  Student guide
    │   ├── Point.java                 Points: position, angle, velocity, radius, stop
    │   ├── Path.java                  The path: geometry, warnings(), flipped(...)
    │   ├── PathConstraints.java       The six limits, defaults(), withX
    │   ├── PathDrivetrain.java        Interface your drivetrain implements
    │   ├── FollowPathCommand.java     The command, progress(), Result
    │   ├── PathFollowerConfig.java    Gains, tolerances, FlipMode, field size
    │   ├── PathGeometry.java          Internal: corner geometry
    │   └── SpeedProfile.java          Internal: speed profile
    └── example/
        └── ExampleSwerveWithPose.java Path example (also uses Kinematics and RobotPose)
```

## License

The project is built on the WPILib template and uses the WPILib license. See
[WPILib-License.md](Infrastructures/WPILib-License.md).
