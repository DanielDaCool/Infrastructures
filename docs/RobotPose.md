# RobotPose

[![WPILib](https://img.shields.io/badge/WPILib-2026.2.1-blue)](https://docs.wpilib.org/)
[![Java](https://img.shields.io/badge/Java-17-orange)](https://adoptium.net/)
[![Phoenix 6](https://img.shields.io/badge/Phoenix%206-26.1.0-green)](https://v6.docs.ctr-electronics.com/)
[![QuestNav](https://img.shields.io/badge/QuestNav-2026--2.2.0-purple)](https://questnav.gg/)

Field-relative pose estimation for FRC swerve robots. RobotPose combines swerve odometry and
the gyro with Limelight cameras (MegaTag2 or 2D) and a Meta Quest running QuestNav, and
returns one pose for autos, aiming and driving.

No filter tuning is required: bad camera frames, a jumping Quest and a wrong starting pose
are handled automatically.

> [!NOTE]
> This README is the student guide. How it works, the design decisions, simulation results,
> calibration and every setting are documented in [RobotPose-design.md](RobotPose-design.md).

## Contents

- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Getting started](#getting-started)
- [Migrating from DemaciaCode](#migrating-from-demaciacode)
- [Telemetry](#telemetry)
- [Troubleshooting](#troubleshooting)
- [Project structure](#project-structure)

## Features

- **Latency-compensated fusion.** Each vision frame is applied at its capture time: the
  estimate is moved back to that time by odometry, corrected, and moved forward again.
- **Jump filter with recovery.** Frames that disagree too much with the estimate are
  rejected. When most recent frames agree with each other but not with the estimate, the
  estimate is the one that is wrong, and the frames are used again.
- **Multiple vision source types.** MegaTag2 Limelight, 2D Limelight (single tag, `tx`/`ty`)
  and Meta Quest (QuestNav), configured through one `VisionConfig`.
- **Automatic Quest handling.** Anchoring at startup and on reconnect, re-alignment after a
  jump or lost tracking, and a cross-check against the MegaTag2 Limelight.
- **Tilt trust.** Wheel motion counts less while the robot is tilted on a bump or ramp.
- **Optional 250 Hz odometry.** `PhoenixOdometryThread` samples the gyro and modules on a
  separate thread over CAN FD (100 Hz on CAN 2.0).
- **Pose prediction.** `getFuturePose` extrapolates the pose for shooting on the move.

## Requirements

| Dependency | Version | Used for |
|---|---|---|
| [WPILib / GradleRIO](https://docs.wpilib.org/) | 2026.2.1 | Core |
| Java | 17 | Core |
| WPILib New Commands | 2026 | Core |
| [CTRE Phoenix 6](https://v6.docs.ctr-electronics.com/) | 26.1.0 | `PhoenixOdometryThread` |
| [QuestNavLib](https://questnav.gg/) | 2026-2.2.0 | `Quest` source |
| [LimelightHelpers](https://github.com/LimelightVision/limelightlib-wpijava) | Bundled | Limelight sources |

## Installation

1. Copy [`Infrastructures/src/main/java/frc/robot/RobotPose`](Infrastructures/src/main/java/frc/robot/RobotPose)
   into your robot project, keeping the same package path.
2. Install the **Phoenix 6** and **QuestNav** vendor libraries (WPILib → Manage Vendor
   Libraries). The vendordep files are in [`Infrastructures/vendordeps`](Infrastructures/vendordeps).
3. Follow [Getting started](#getting-started).

## Getting started

### 1. Describe your cameras

Each camera needs a **name**, its **position on the robot** and how much to trust it
(standard deviations: x m, y m, heading rad).

```java
public final class VisionConstants {
    // Limelight hostname: "limelight-front"
    public static final LimelightTagCamera3dConfig FRONT = new LimelightTagCamera3dConfig(
        "front",
        new Transform3d(0.30, 0, 0.35, new Rotation3d(0, Math.toRadians(15), 0)),
        VecBuilder.fill(0.02, 0.02, Double.POSITIVE_INFINITY));

    public static final QuestConfig QUEST = new QuestConfig(
        "quest",
        new Transform3d(0.10, 0, 0.50, new Rotation3d()),
        VecBuilder.fill(0.05, 0.05, Double.POSITIVE_INFINITY));

    public static final VisionConfig VISION = new VisionConfig(FRONT, QUEST);
}
```

> [!IMPORTANT]
> The position on the robot is measured from the robot's center (on the floor) to the camera
> lens, in meters: **x forward, y left, z up**. Angles are in radians, counter-clockwise
> positive.
>
> **Exception:** for a 2D Limelight (`LimelightTagCamera2dConfig`), pitch is positive when
> the camera is tilted **up**.

### 2. Initialize RobotPose

Call this once in `Robot`'s constructor, **after the chassis is created** (it reads the gyro
and the modules):

```java
RobotPose.initialize(
    () -> List.of(new OdometryData(chassis.getGyroAngle(), chassis.getModulePositions())),
    angle -> chassis.setGyroYaw(angle),     // setYaw() writes the gyro
    angle -> chassis.setGyroYaw(angle),     // resetPose() writes the gyro too
    chassis.getModuleLocations(),           // same order as getModulePositions()
    VecBuilder.fill(0.05, 0.05, 0),         // how much to trust the wheels (x m, y m, heading)
    VisionConstants.VISION);
```

> [!NOTE]
> `RobotPose.initialize` can also be called in the `RobotContainer` constructor, as long as
> it comes after the chassis is created there. `Robot`'s constructor remains the recommended
> place. Either way, call it only once.

### 3. Update every loop

In `Robot.robotPeriodic()`, after the CommandScheduler:

```java
CommandScheduler.getInstance().run();
RobotPose.getInstance().periodic();
```

### 4. Read the pose

```java
Pose2d pose = RobotPose.getInstance().getEstimatedPose();
```

| Method | Description |
|---|---|
| `getEstimatedPose()` | Current pose (blue-alliance origin). |
| `getFuturePose(seconds)` | Predicted pose a moment ahead at the current speed (for shooting on the move). |
| `resetPose(pose)` | Sets the robot's pose (for example, at the start of an auto). |
| `setYaw(angle)` | Sets only the heading, and writes it to the gyro. |
| `getEstimatedPoseAt(time)` | Pose at a past time (last 1.5 s). |

### Optional extras

- **Bumps:** pass the robot's tilt so wheel slip on bumps is not counted:
  `new OdometryData(Timer.getFPGATimestamp(), gyro, modules, OdometryData.tilt(pitchRadians, rollRadians))`.
- **More accurate odometry (250 Hz):** use `PhoenixOdometryThread` instead of the supplier
  above; see [RobotPose-design.md](RobotPose-design.md#250-hz-odometry).
- **Dashboard generator:** `BaseVisionSource.setOnCreate(callback)` is called with every
  vision source as it is created.
- **Your own logger:** all logging goes through `frc.demacia.utils.log.Log` (`putData` for dashboard
  entries, `alert` for alerts). The package ships a small stand-in of the team's `Log` and
  `ConsoleAlert` (plain WPILib: SmartDashboard and Alerts) in `Infrastructures/src/main/java/frc/demacia/utils/log`.
  To use the team logs, delete those two files; no other change is needed.

## Migrating from DemaciaCode

1. Check the 3D Limelight on the robot first: `vision/<name>/source pose sent` on the
   dashboard must match the camera pose in the Limelight web interface.
2. Change the imports from `frc.demacia.RobotPose` to `frc.robot.RobotPose`.
3. Replace the `initialize` call with the one in [step 2](#2-initialize-robotpose), in
   `Robot`'s constructor (use your `STATE_STD` for the wheel trust), and add
   `BaseVisionSource.setOnCreate(ElasticGenerator.getInstance()::registerVisionSource);`.
4. The dashboard keys moved from `chassis/...` to `pose/...`.

## Telemetry

| Key | What to look for |
|---|---|
| `pose/field` | The robot on the field. It should sit where the robot really is. |
| `vision/<name>/is Connected` | The camera or Quest is communicating with the robot. |
| `vision/<name>/seconds since used` | If it keeps growing, that source is not contributing (no tag, or rejected). |
| `vision/<name>/last error m` | Consistently large means that source's position on the robot is wrong. |
| `pose/frames rejected` | Frames that looked wrong. A few is normal; a fast climb means a wrong offset. |
| `pose/reset pose`, `chassis/reset gyro` | Buttons: reset the pose to the origin, or the heading to 0. |

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Pose drifts when driving straight | Module locations in the wrong order, or wrong wheel size / gear ratio. |
| Pose turns the wrong way | The gyro must be counter-clockwise positive. |
| Pose jumps around | A camera's trust is 0 (`VecBuilder.fill(0, ...)`); use a small number instead. |
| Cameras are ignored for a long time | A camera offset is wrong, or the pose was reset to a wrong place. After a few seconds of cameras agreeing, it recovers on its own. |
| Cameras never change the pose | Check `is Connected` and that the camera sees tags (Quest: `is tracking`). |
| One camera keeps pulling the pose | Its offset on the robot is wrong; see [calibration](RobotPose-design.md#calibration-step-by-step). |
| 2D camera is off by meters when turning | Wrong camera yaw, height or pitch. |
| Quest pulls the pose to (0, 0) | Do not call `QuestNav.setPose` directly; use `resetPose` or the dashboard button. |

## Project structure

All paths are relative to `Infrastructures/src/main/java/frc/robot/RobotPose`.

| Path | Description |
|---|---|
| `RobotPose.java` | Public entry point. Singleton that runs the full pipeline each loop. |
| `Estimation/DemaciaOdometry.java` | Swerve odometry (gyro + module positions → twist). |
| `Estimation/DemaciaPoseEstimator.java` | Fuses odometry and vision: latency compensation, jump filter with recovery, tilt trust. |
| `Estimation/PhoenixOdometryThread.java` | Optional 250 Hz (CAN FD) / 100 Hz (CAN 2.0) odometry thread. |
| `Vision/VisionSource.java` | Interface implemented by every vision source. |
| `Vision/BaseVisionSource.java` | Base class for sources: config values and dashboard entries. |
| `Vision/VisionConfig.java` | The list of sources passed to `RobotPose.initialize`. |
| `Vision/TimestampedVisionMeasurement.java` | A single measurement: pose, capture time, standard deviations. |
| `Vision/VisionTypes/LimelightTagCamera2d.java` | Limelight in 2D mode (`tx`/`ty` to one tag). |
| `Vision/VisionTypes/LimelightTagCamera3d.java` | Limelight using MegaTag2. |
| `Vision/VisionTypes/Quest.java` | Meta Quest running QuestNav. |
| `Vision/VisionTypes/QuestFrameFilter.java` | Selects which Quest frames are used (jump and lost-tracking handling, re-alignment, 10 Hz thinning). |
| `Vision/visionConfigs/*Config.java` | One config class per source type. |
| `Vision/LimelightHelpers.java` | Limelight's official helper library. |

Outside this folder: `Infrastructures/src/main/java/frc/demacia/utils/log/Log.java` and `ConsoleAlert.java` are the stand-in for the team logger. Delete them when using the real one.
