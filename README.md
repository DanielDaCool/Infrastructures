# Infrastructures

[![WPILib](https://img.shields.io/badge/WPILib-2026.2.1-blue)](https://docs.wpilib.org/)
[![Java](https://img.shields.io/badge/Java-17-orange)](https://adoptium.net/)
[![Phoenix 6](https://img.shields.io/badge/Phoenix%206-26.1.0-green)](https://v6.docs.ctr-electronics.com/)
[![QuestNav](https://img.shields.io/badge/QuestNav-2026--2.2.0-purple)](https://questnav.gg/)

Infrastructures is a set of Java modules for FRC robot code. It currently has two: swerve
kinematics with an acceleration limiter, and pose estimation that fuses wheel odometry, Limelight
and Quest data. Each module can be used on its own. You copy its package into your robot project
and call it from your own drivetrain code. This branch (`development`) holds both modules
together so they can be tested as one project.

## Modules

| Module | What it does | Docs |
|---|---|---|
| Kinematics | Swerve kinematics (field-relative and robot-relative), feed-forwards, and a swerve acceleration limiter. No vendor libraries. | [Guide](docs/Kinematics.md), [Design](docs/Kinematics-design.md) |
| RobotPose | Pose estimation from module positions and gyro, plus Limelight and Quest vision sources. Needs Phoenix 6 and QuestNav. | [Guide](docs/RobotPose.md), [Design](docs/RobotPose-design.md) |

Also in `docs/`: [library-plan.md](docs/library-plan.md), the plan for packaging both modules as one
importable library.

## Using both together

[`ExampleSwerveWithPose.java`](Infrastructures/src/main/java/frc/robot/example/ExampleSwerveWithPose.java)
shows the two modules in one drivetrain. Kinematics drives field-relative using the gyro heading, which
RobotPose keeps equal to its own heading. RobotPose is fed by the module positions and the gyro, and by vision measurements.

For Kinematics without RobotPose, see
[`ExampleChassis.java`](Infrastructures/src/main/java/frc/robot/example/ExampleChassis.java).

## Installation

1. Copy the package you want into your robot project, keeping the same package path:
   - [`frc/robot/Kinematics`](Infrastructures/src/main/java/frc/robot/Kinematics)
   - [`frc/robot/RobotPose`](Infrastructures/src/main/java/frc/robot/RobotPose)
2. For RobotPose, install the Phoenix 6 and QuestNav vendor libraries (WPILib: Manage Vendor
   Libraries), or copy the files from [`Infrastructures/vendordeps`](Infrastructures/vendordeps).
   Kinematics needs no vendor libraries.
3. Continue with the guide for each module (links in the table above).

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

In `development`, each module's README and design doc live under `docs/` (see the table above).
Module branches keep editing their own README and design doc where they are on that branch;
the move into `docs/` happened on `development`.

Edits to `docs/design.md` on a module branch follow the move automatically. Edits to a module
branch's root `README.md` do not: git sees them against this README and reports a conflict. When
that happens, keep this README and copy the module's change into `docs/Kinematics.md` or
`docs/RobotPose.md` by hand. Moving each module's README into `docs/` on its own branch removes
the conflict for good.

## Project structure

```
.
├── README.md
├── docs/
│   ├── Kinematics.md            Kinematics guide
│   ├── Kinematics-design.md     Kinematics design notes
│   ├── RobotPose.md             RobotPose guide
│   ├── RobotPose-design.md      RobotPose design notes
│   └── library-plan.md          Plan for packaging as one library
└── Infrastructures/
    ├── build.gradle             GradleRIO 2026.2.1, Java 17
    ├── vendordeps/              Phoenix 6, WPILib New Commands, QuestNavLib
    ├── WPILib-License.md
    └── src/main/java/frc/
        └── robot/
            ├── Kinematics/      Swerve kinematics, acceleration limiter
            ├── RobotPose/       Estimation/ (odometry, pose estimator), Vision/ (sources, configs)
            ├── example/         ExampleChassis, ExampleSwerveWithPose
            └── Main, Robot, RobotContainer
```

## Contributing

Work on the branch of the module you are changing, not on `development` or `main`. Keep changes to
a module's own files and docs. Before asking for a merge into `development`, run
`./gradlew build`.

## License

The project is built on the WPILib template and uses the WPILib license. See
[WPILib-License.md](Infrastructures/WPILib-License.md).
