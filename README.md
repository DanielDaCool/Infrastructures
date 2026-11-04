# Infrastructures

[![WPILib](https://img.shields.io/badge/WPILib-2026.2.1-blue)](https://docs.wpilib.org/)
[![Java](https://img.shields.io/badge/Java-17-orange)](https://adoptium.net/)
[![Phoenix 6](https://img.shields.io/badge/Phoenix%206-26.1.0-green)](https://v6.docs.ctr-electronics.com/)
[![QuestNav](https://img.shields.io/badge/QuestNav-2026--2.2.0-purple)](https://questnav.gg/)

Infrastructures is a set of Java modules for FRC robot code. It currently has three: swerve
kinematics with an acceleration limiter, pose estimation that fuses wheel odometry, Limelight
and Quest data, and a distance-based path planner and follower. Each module can be used on its
own. You copy its package into your robot project and call it from your own drivetrain code.
This branch (`development`) holds all three modules together so they can be tested as one project.

## Modules

| Module | What it does | README and design |
|---|---|---|
| Kinematics | Swerve kinematics (field-relative and robot-relative), feed-forwards, and a swerve acceleration limiter. No vendor libraries. | [README](Infrastructures/src/main/java/frc/robot/Kinematics/README.md), [Design](docs/Kinematics-design.md) |
| RobotPose | Pose estimation from module positions and gyro, plus Limelight and Quest vision sources. Needs Phoenix 6 and QuestNav. | [README](Infrastructures/src/main/java/frc/robot/RobotPose/README.md), [Design](docs/RobotPose-design.md) |
| Path | Autonomous paths: points to rounded corners, heading and speed profiles, and a command that follows the path. Any drivetrain and pose source; needs no other module. No vendor libraries. Not yet run on a robot. | [README](Infrastructures/src/main/java/frc/robot/Path/README.md), [Design](docs/Path-design.md) |

Also in `docs/`: [library-plan.md](docs/library-plan.md), the plan for packaging the modules as one
importable library.

## Using the modules together

[`ExampleSwerveWithPose.java`](Infrastructures/src/main/java/frc/robot/example/ExampleSwerveWithPose.java)
shows the modules in one drivetrain. Kinematics drives field-relative using the gyro heading, which
RobotPose keeps equal to its own heading. RobotPose is fed by the module positions and the gyro, and by vision measurements.
Path follows a path using that pose and the same drivetrain; see the Path README.

For Kinematics without RobotPose, see
[`ExampleChassis.java`](Infrastructures/src/main/java/frc/robot/example/ExampleChassis.java).

## Installation

1. Copy the package you want into your robot project, keeping the same package path:
   - [`frc/robot/Kinematics`](Infrastructures/src/main/java/frc/robot/Kinematics)
   - [`frc/robot/RobotPose`](Infrastructures/src/main/java/frc/robot/RobotPose)
   - [`frc/robot/Path`](Infrastructures/src/main/java/frc/robot/Path) (WPILib only; needs no other module)
2. For RobotPose, install the Phoenix 6 and QuestNav vendor libraries (WPILib: Manage Vendor
   Libraries), or copy the files from [`Infrastructures/vendordeps`](Infrastructures/vendordeps).
   Kinematics and Path need no vendor libraries.
3. Continue with the README of each module (links in the table above).

Pinned versions: WPILib/GradleRIO 2026.2.1, Java 17, Phoenix 6 26.1.0, QuestNavLib 2026-2.2.0.

## Build and simulate

Run these from the [`Infrastructures`](Infrastructures) directory:

```
./gradlew build
./gradlew simulateJava
```

On Windows, set `JAVA_HOME` to the WPILib 2026 JDK before running Gradle.

## Branch workflow

`Kinematics` and `RobotPose` are developed independently, each on its own branch. They are merged
into `development` for integration testing. When the tests pass, `development` is merged into
`main`.

```
git switch development
git merge Kinematics
git merge RobotPose
# build and test
git switch main
git merge development
```

Each module's README lives in its package folder, and its design doc under `docs/` (see the table above).
Module branches that still keep their README at the root conflict with this README on merge: keep
this README and copy the module's change into the module README by hand.

## Project structure

```
.
├── README.md
├── docs/
│   ├── Kinematics-design.md     Kinematics design notes
│   ├── Path-design.md           Path design notes
│   ├── RobotPose-design.md      RobotPose design notes
│   └── library-plan.md          Plan for packaging as one library
└── Infrastructures/
    ├── build.gradle             GradleRIO 2026.2.1, Java 17
    ├── vendordeps/              Phoenix 6, WPILib New Commands, QuestNavLib
    ├── WPILib-License.md
    └── src/main/java/frc/
        └── robot/
            ├── Kinematics/      README.md; Swerve kinematics, acceleration limiter
            ├── Path/            README.md; Distance-based path planner and follower
            ├── RobotPose/       README.md; Estimation/ (odometry, pose estimator), Vision/ (sources, configs)
            ├── example/         ExampleChassis, ExampleSwerveWithPose
            └── Main, Robot, RobotContainer
```

## License

The project is built on the WPILib template and uses the WPILib license. See
[WPILib-License.md](Infrastructures/WPILib-License.md).
