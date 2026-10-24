# RobotPose

Knows where the robot is on the field. It combines the swerve wheels and the gyro (odometry)
with Limelight cameras (MegaTag2 or 2D) and a Meta Quest (QuestNav), and gives you one pose to
use for autos, aiming and driving.

You don't have to tune a filter: bad camera frames, a jumping Quest and a wrong start pose are
handled for you.

> Mentors and developers: how it works, why, test results and every setting are in
> [docs/design.md](docs/design.md).

## Install

1. Copy [`Infrastructures/src/main/java/frc/robot/RobotPose`](Infrastructures/src/main/java/frc/robot/RobotPose)
   into your robot project (same package path).
2. Install the vendor libraries: **Phoenix 6** and **QuestNav** (WPILib → Manage Vendor Libraries).

Made for WPILib 2026.

## Getting started

### 1. Describe your cameras

Each camera needs a **name**, **where it is on the robot** and how much to trust it.

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

**Where it is on the robot** = from the robot's center (on the floor) to the camera lens, in
meters: **x forward, y left, z up**. Angles in radians, counter-clockwise positive.
A 2D Limelight (`LimelightTagCamera2dConfig`) is the one exception: its pitch is positive
when the camera is tilted **up**.

### 2. Start it (once)

In `Robot`'s constructor or in `RobotContainer`, **after the chassis is created** (it reads the
gyro and the modules). Not in `Main.java`: that runs before WPILib starts.

```java
RobotPose.initialize(
    () -> List.of(new OdometryData(chassis.getGyroAngle(), chassis.getModulePositions())),
    angle -> chassis.setGyroYaw(angle),     // setYaw() writes the gyro
    angle -> chassis.setGyroYaw(angle),     // resetPose() writes the gyro too
    chassis.getModuleLocations(),           // same order as getModulePositions()
    VecBuilder.fill(0.05, 0.05, 0),         // how much to trust the wheels (x m, y m, heading)
    VisionConstants.VISION);
```

### 3. Run it every loop

In `Robot.robotPeriodic()`, after the CommandScheduler:

```java
CommandScheduler.getInstance().run();
RobotPose.getInstance().periodic();
```

### 4. Use it

| Call | What it does |
|---|---|
| `getEstimatedPose()` | Where the robot is now (blue-alliance origin). |
| `getFuturePose(seconds)` | Where it will be in a moment at the current speed (for shooting on the move). |
| `resetPose(pose)` | Tell it where the robot is (e.g. at the start of an auto). |
| `setYaw(angle)` | Set only the heading (writes the gyro). |
| `getEstimatedPoseAt(time)` | Where it was at a past time (last 1.5 s). |

```java
Pose2d pose = RobotPose.getInstance().getEstimatedPose();
```

### Optional extras

- **Bumps:** pass the robot's tilt so wheel slip on bumps isn't counted:
  `new OdometryData(Timer.getFPGATimestamp(), gyro, modules, OdometryData.tilt(pitchRadians, rollRadians))`.
- **More accurate odometry (250 Hz):** use `PhoenixOdometryThread` instead of the supplier above;
  see [docs/design.md](docs/design.md#250-hz-odometry).
- **Dashboard generator:** `BaseVisionSource.setOnCreate(callback)` gives you every vision source.

## Switching from the old DemaciaCode RobotPose

1. Check the 3D Limelight on the robot first: `vision/<name>/source pose sent` on the dashboard
   must match the camera pose in the Limelight web page.
2. Change the imports from `frc.demacia.RobotPose` to `frc.robot.RobotPose`.
3. Replace the `initialize` call with the one in step 2 above (use your `STATE_STD` for the
   wheel trust), and add
   `BaseVisionSource.setOnCreate(ElasticGenerator.getInstance()::registerVisionSource);`.
4. The dashboard keys moved from `chassis/...` to `pose/...`.

## Before the first match

Do these once, in order, and again after big changes to the robot:

1. **Wheels:** drive a taped 5 m straight line with the cameras covered. If the pose says 4.9 m,
   your wheel size is 2 % off: fix the wheel circumference.
2. **Gyro:** spin the robot 10 turns and stop at the same heading. The gyro should read 3600°.
   Turning left must be positive.
3. **Camera pitch:** park about 1 m from a tag, centered. Adjust the camera pitch until the pose
   distance matches a tape measure.
4. **Camera position:** spin slowly in place in front of a tag. If the camera's pose (on
   `vision/<name>/field`) draws a circle, the camera's x/y on the robot is wrong. Fix it until it
   stays on one spot.
5. **Quest:** same spin test for the Quest offset.
6. **Check:** drive around for a minute and stop on a taped mark. The pose should be a few cm off.

Full procedure with the reasons: [docs/design.md](docs/design.md#calibration-step-by-step).

## Dashboard

| Key | What to look for |
|---|---|
| `pose/field` | The robot on the field. Should sit where the robot really is. |
| `vision/<name>/is Connected` | The camera / Quest is talking to the robot. |
| `vision/<name>/seconds since used` | Keeps growing = that camera isn't helping (no tag, or rejected). |
| `vision/<name>/last error m` | Always large = that camera's position on the robot is wrong. |
| `pose/frames rejected` | Frames that looked wrong. A few is normal; a fast climb means a wrong offset. |
| `pose/reset pose`, `chassis/reset gyro` | Buttons: reset the pose to the origin / the heading to 0. |

## Troubleshooting

| Problem | Try |
|---|---|
| Pose drifts when driving straight | Module locations in the wrong order, or wrong wheel size / gear ratio. |
| Pose turns the wrong way | The gyro must be counter-clockwise positive. |
| Pose jumps around | A camera's trust is 0 (`VecBuilder.fill(0, ...)`) – use a small number instead. |
| Cameras are ignored for a long time | A camera offset is wrong, or the pose was reset to a wrong place. After a few seconds of cameras agreeing it fixes itself. |
| Cameras never change the pose | Check `is Connected` and that the camera sees tags (Quest: `is tracking`). |
| One camera keeps pulling the pose | Its offset on the robot is wrong (see "Before the first match"). |
| 2D camera is off by meters when turning | Wrong camera yaw, height or pitch. |
| Quest pulls the pose to (0, 0) | Don't call `QuestNav.setPose` yourself; use `resetPose` or the dashboard button. |
