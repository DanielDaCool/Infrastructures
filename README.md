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
> The student guide (full API, headings, constants, troubleshooting) is
> [Infrastructures/src/main/java/frc/robot/Path/README.md](Infrastructures/src/main/java/frc/robot/Path/README.md).
> The math is in [docs/Path-design.md](docs/Path-design.md); a plain-language walk-through is in
> [docs/Path-explained.md](docs/Path-explained.md).

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

## Usage

### 1. Build a path

Points are in meters, in field coordinates on the blue-alliance side. The last point is always a stop.

```java
Path path = new Path(new Point(1, 2), new Point(3, 4), new Point(5, 2));
```

| Call | What it does |
|---|---|
| `new Point(x, y)` | A point |
| `.withAngle(Rotation2d)` | The robot faces this way at this point. The robot turns smoothly between angle points |
| `.withRadius(r)` | Corner circle radius `r` m (default `PathConfig.RADIUS`) |
| `.stop()` | Stop exactly on this point |
| `.stop(seconds)` | Stop, then wait that many seconds |

```java
// heading on a point, and a wide corner
Path path = new Path(
    new Point(1, 2).withAngle(Rotation2d.fromDegrees(0)),
    new Point(3, 4).withRadius(0.5),
    new Point(5, 2));

// stop on a point for 1.5 s, then continue
Path stopping = new Path(new Point(1, 2), new Point(4, 2).stop(1.5), new Point(7, 4));
```

A stop point is a sharp corner: the path arrives and leaves on straight lines. `Point` is immutable;
`withAngle` and the other `withX` calls return a new object, so chain the calls.

### 2. Create and run the command

Make a new command each time autonomous starts. A command can be used in only one composition.

```java
FollowPathCommand follow = new FollowPathCommand(
    path, swerve::getPose, swerve::getFieldSpeeds, swerve::driveFieldRelative, swerve);
follow.schedule();
```

The suppliers give the pose and the measured field-relative speeds; the consumer takes field-relative speeds.
The command does not reset your pose: put the robot on the first point. If it is more than 0.5 m away, the
command reports an error and finishes without driving.

### 3. Alliance flip

Write every path for the blue alliance. On red, the command flips the path once when it starts.
`PathConfig.MIRRORED_FIELD` picks the mode and is set per season: `false` rotates the field 180 degrees
about its center, `true` mirrors it across `x = L / 2`, with `L` the field length from WPILib's AprilTag layout for the season.

### 4. Progress triggers

`progress()` goes from 0 at the start to 1 at the end and never decreases during a run.

```java
new Trigger(() -> follow.progress() > 0.7).onTrue(intake.deployCommand());   // your command
```

The trigger's command must not require the drivetrain, or it cancels the path.

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
│   ├── Path-design.md                 Design notes
│   └── Path-explained.md              Plain-language walk-through of the code
└── Infrastructures/src/main/java/frc/robot/
    ├── Path/
    │   ├── README.md                  Student guide
    │   ├── PathConfig.java            Values to set for your robot
    │   ├── Point.java                 Points: position, angle, radius, stop
    │   ├── Path.java                  The path: corners, stops, flipped()
    │   ├── Segment.java, Line.java, Arc.java  Path pieces
    │   ├── PathFollower.java          Progress, speed, direction, heading
    │   └── FollowPathCommand.java     The command, progress()
    └── example/
        └── ExampleSwerveWithPose.java Path example (also uses Kinematics and RobotPose)
```

## License

The project is built on the WPILib template and uses the WPILib license. See
[WPILib-License.md](Infrastructures/WPILib-License.md).
