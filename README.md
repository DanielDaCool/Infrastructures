# Infrastructures – RobotPose

Field-pose estimation for a swerve robot: odometry from the gyro and swerve modules, fused
with vision from Limelights (2D or MegaTag2) and a Meta Quest (QuestNav).

The code lives in [`Infrastructures/src/main/java/frc/robot/RobotPose`](Infrastructures/src/main/java/frc/robot/RobotPose).
It is kept in sync with `frc.demacia.RobotPose` in DemaciaCode (estimation, collision detection and
the Limelight fixes from `odometry-accelerometer-testing` at `f536d31`, the Quest battery alert and
the rest of the vision code from `mechanism` at `ed68a5c`), with the DemaciaCode-only
dependencies (`Chassis`, `Log`, `ElasticGenerator`) removed so it works in any project.

## Contents

| Path | What it is |
|---|---|
| `RobotPose.java` | The one class the rest of the robot uses. Singleton. Runs everything each loop. |
| `RoboRioOrientation.java` | How the roboRIO is mounted, for collision detection (optional). |
| `Estimation/DemaciaOdometry.java` | Swerve odometry (gyro + module positions → twist). |
| `Estimation/DemaciaPoseEstimator.java` | Fuses odometry and vision with a replayed history (same approach as 6328's estimator). |
| `Estimation/PhoenixOdometryThread.java` | Optional: reads the gyro and modules at 250 Hz (CAN FD) / 100 Hz (CAN 2.0) on a separate thread. |
| `Vision/VisionSource.java` | Interface every vision source implements. |
| `Vision/BaseVisionSource.java` | Base class for sources: config values and dashboard entries. |
| `Vision/VisionConfig.java` | The list of sources passed to `RobotPose.initialize`. |
| `Vision/TimestampedVisionMeasurement.java` | One measurement: pose, capture time, std devs. |
| `Vision/VisionTypes/LimelightTagCamera2d.java` | Limelight in 2D mode (tx/ty to one tag). |
| `Vision/VisionTypes/LimelightTagCamera3d.java` | Limelight using MegaTag2. |
| `Vision/VisionTypes/Quest.java` | Meta Quest running QuestNav. |
| `Vision/visionConfigs/*Config.java` | One config class per source type. |
| `Vision/LimelightHelpers.java` | Limelight's official helper library. |

Vendor dependencies: `questnavlib` (for the Quest), Phoenix 6 (for `PhoenixOdometryThread`)
and WPILib New Commands.

## Quick start

### 1. Describe the vision sources

Each source has a config: a **name**, an **offset** (robot center → device, meters/radians)
and **std devs** (x m, y m, θ rad) saying how much to trust it.

Every offset is in the **WPILib robot frame**: x forward, y left, z up, rotations CCW
positive. This holds for every source type. A device that uses another frame gets the offset
converted by its config class (`BaseVisionSourceConfig.sourceFrame()` /
`getSourceOffset()`); sources never flip signs themselves.

```java
public final class VisionConstants {
    // Limelight hostname must be "limelight-back" for this one.
    public static final LimelightTagCamera2dConfig BACK_2D = new LimelightTagCamera2dConfig(
        "back",
        new Transform3d(-0.25, 0, 0.40, new Rotation3d(0, Math.toRadians(20), Math.PI)),
        VecBuilder.fill(0.5, 0.5, Double.POSITIVE_INFINITY));

    public static final LimelightTagCamera3dConfig FRONT_3D = new LimelightTagCamera3dConfig(
        "front",
        new Transform3d(0.30, 0, 0.35, new Rotation3d(0, Math.toRadians(15), 0)),
        VecBuilder.fill(0.3, 0.3, Double.POSITIVE_INFINITY));

    public static final QuestConfig QUEST = new QuestConfig(
        "quest",
        new Transform3d(0.10, 0.0, 0.50, new Rotation3d()),
        VecBuilder.fill(0.05, 0.05, Double.POSITIVE_INFINITY));

    public static final VisionConfig VISION_CONFIG = new VisionConfig(BACK_2D, FRONT_3D, QUEST);
}
```

`VisionConfig` creates the sources as soon as it is built, so build it once (a `static final`
is fine).

### 2. Initialize RobotPose (once, after the chassis exists)

```java
RobotPose.initialize(
    // Every odometry sample since the last loop. Here: one sample read now, every loop.
    // For 250 Hz odometry pass odometryThread::getNewSamples instead (see below).
    () -> List.of(new OdometryData(chassis.getGyroAngle(), chassis.getModulePositions())),
    angle -> gyro.setYaw(angle.getDegrees()),        // used by setYaw()
    chassis.getModuleLocations(),                    // same order as the module positions
    VecBuilder.fill(0.1, 0.1, 0),                    // odometry std devs (x m, y m, θ rad)
    VisionConstants.VISION_CONFIG);
```

### 3. Run it every loop

In `Robot.robotPeriodic()`, **after** the CommandScheduler:

```java
CommandScheduler.getInstance().run();
RobotPose.getInstance().periodic();
```

### 4. Use the pose

```java
Pose2d pose = RobotPose.getInstance().getEstimatedPose();
```

[`Robot.java`](Infrastructures/src/main/java/frc/robot/Robot.java) has a placeholder setup
(zero gyro, zero modules, no vision) so the project runs without hardware. Replace it with
the real chassis.

## API

| Method | What it does |
|---|---|
| `getEstimatedPose()` | Latest fused field pose (blue-alliance origin). |
| `getEstimatedPoseAt(t)` | Fused pose at a past FPGA time. Only the last 1.5 s are kept. |
| `resetPose(pose)` | Sets position and heading. The gyro is **not** written; an offset is stored instead. Clears the vision history and re-anchors the Quest. |
| `setYaw(angle)` | Sets the heading only and **writes it to the gyro** (through `gyroYawSetter`). Keeps the position. |
| `getGyroAngle()` | Raw gyro reading of the newest odometry sample. Not the field heading after a `resetPose`; use `getEstimatedPose()` for that. |
| `enableCollisionDetection(orientation, position, yawRate)` | Optional. Turns on collision detection (see below). |

## How it works

Each `periodic()`:

1. **Odometry first.** Every sample since the last loop (gyro + modules) is added at its
   own timestamp, so any vision frame captured up to now has odometry around it.
2. **Every source reads its device** (`VisionSource.periodic()`). The MegaTag2 Limelight
   sends the current heading here.
3. **Every source is checked.** A Quest that just (re)connected is re-anchored to the
   current estimate and skipped this loop. Any other source with `shouldUpdate()` true has
   its measurements added at their **capture time**.
4. **Quest vs 3D Limelight.** Before those measurements are added, when a Quest and a
   MegaTag2 Limelight both have a new one, their distance is logged (the Quest's position is
   moved by the odometry between the two capture times first). If it stays over 0.3 m for
   1 s a WPILib alert is raised: one of the offsets is probably wrong.

### Odometry (`DemaciaOdometry`)

- Each module's motion during one loop is treated as a circular arc (its wheel angle
  changed during the loop); the chord of that arc is its displacement.
- The robot's displacement is a weighted average of the modules. Weights start equal and
  can be changed with `changeModuleWeight` (e.g. to trust a slipping module less).
- The heading always comes from the gyro, never from the wheels.

### Fusion (`DemaciaPoseEstimator`)

- The estimator keeps a 1.5 s history of odometry twists. A vision measurement is inserted at
  its capture time (splitting a twist if needed) and the history is replayed from there, so
  late measurements are applied where they were taken and several corrections add up.
- Per field axis (x, y, θ), measurements at the same time are combined by inverse variance,
  then the estimate moves by `K * residual`, with `K = q / (q + sqrt(q * r))` (q = odometry
  variance, r = measurement variance).

### Collision detection (optional)

Driving into a wall or a robot, the wheels keep turning while the robot is stopped, so odometry
counts motion that didn't happen. With collision detection on, a loop where the roboRIO's
built-in accelerometer reads more than `COLLISION_ACCELERATION` (3 g) horizontally is a hit,
and for `COLLISION_HOLD_SECONDS` (60 ms) after the last hit the part of the wheel motion that
goes **into** the hit is dropped. Motion to the side or away from it is kept, and the heading
always comes from the gyro.

It is **off** unless you turn it on, because the roboRIO mounting has to be measured on each robot:

```java
RobotPose.getInstance().enableCollisionDetection(
    RoboRioOrientation.LABEL_LEFT_X_FRONT,   // where the label faces, where its X axis points
    new Translation2d(-0.10, 0.05),          // roboRIO position from the robot center (m)
    () -> Math.toRadians(pigeon.getAngularVelocityZWorld().getValueAsDouble())); // yaw rate, rad/s
```

- The yaw rate removes the w² · r an off-center roboRIO reads while spinning.
- Check the orientation (disabled, on the floor): `pose/acceleration x g` and `y g` read about 0
  at rest, lifting the robot's front makes `x g` positive, lifting its left side makes `y g` positive.
- The 3 g threshold is not tuned yet; tune it from `pose/acceleration g`.
- With 250 Hz odometry the accelerometer is still read once per loop, so a hit applies to every
  sample of that loop.
- In simulation the accelerometer reads 0, so nothing is detected.

## 250 Hz odometry

Odometry adds up small steps, and each step assumes the robot moved in one smooth arc
between two readings. At 50 Hz a step is 20 ms; when the robot accelerates or turns during
it, the guess is off and the error stays in the pose. At 250 Hz a step is 4 ms, so each
guess is much closer. In 6328's testing, 250 Hz autos ended in almost the same place every
run (84% smaller spread than 50 Hz).

`PhoenixOdometryThread` reads the signals on its own thread. Each cycle it waits until the
gyro yaw and every drive/steer position have a new value (`BaseStatusSignal.waitForAll`),
builds a sample, stamps it with the FPGA time minus the signals' CAN latency, and queues it.
Each robot loop `RobotPose` takes the ~5 queued samples and adds them all, then replays the
history once. The pose math stays on the main thread; the thread only reads CAN.

| Bus | Frequency | How it waits |
|---|---|---|
| CAN FD (CANivore) | 250 Hz | `waitForAll`: returns when all frames arrived, so all signals are sampled together. |
| CAN 2.0 (roboRIO) | 100 Hz | Sleeps one period, then `refreshAll` (CAN 2.0 can't wait on several signals). |

```java
// Clones, so the main thread refreshing the originals doesn't interfere with the thread.
StatusSignal<Angle> yaw = pigeon.getYaw().clone();
StatusSignal<Angle>[] drive = ...; // module.driveMotor.getPosition().clone()
StatusSignal<Angle>[] steer = ...; // module.steerMotor.getPosition().clone()

PhoenixOdometryThread odometryThread = new PhoenixOdometryThread(canBus,
    () -> {
        // Runs on the odometry thread: only read the signals' values, never refresh them.
        SwerveModulePosition[] modules = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            modules[i] = new SwerveModulePosition(
                drive[i].getValueAsDouble() * WHEEL_CIRCUMFERENCE,         // wheel rotations -> m
                Rotation2d.fromRotations(steer[i].getValueAsDouble()));   // steer rotations
        }
        return new OdometryData(Rotation2d.fromDegrees(yaw.getValueAsDouble()), modules);
    },
    yaw, drive[0], steer[0], drive[1], steer[1], drive[2], steer[2], drive[3], steer[3]);

RobotPose.initialize(odometryThread::getNewSamples, ...);
```

- The thread starts in the constructor and sets these signals to its frequency. If you use
  `optimizeBusUtilization`, call it after creating the thread.
- The unit conversions must match your motor configs (`SensorToMechanismRatio`). Any extra
  math your module does (e.g. a steer/drive coupling correction) goes in the reader too,
  using only these signals.
- `getFailedCycles()` counts cycles where a signal didn't arrive in time (e.g. an unplugged
  device).
- Vision is unchanged. A frame up to 50 ms newer than the newest odometry sample is added
  to that sample instead of being dropped (the newest sample is a few ms old).

### Choosing std devs

Each vision frame moves the estimate by `K` of the way to the measurement, per axis, with
`K = 1 / (1 + σ_vision / σ_odometry)`. Only the **ratio** matters: lowering the odometry std
is the same as raising every vision std. Examples with odometry σ = 0.1 (WPILib's default):

| Vision σ | K | Each frame |
|---|---|---|
| 0.05 | 0.67 | Pulls two thirds of the way (e.g. a Quest). |
| 0.3 | 0.25 | Pulls a quarter of the way. |
| 0.9 | 0.10 | Small nudge (far or single-tag frames). |

| Value | Meaning |
|---|---|
| Odometry std **larger** | Vision corrects the pose faster. |
| Odometry std **0** on an axis | Vision never changes that axis. Typical for θ (trust the gyro). |
| Measurement std **smaller** | That source pulls harder. |
| Measurement std **0** | That measurement is taken as exact (the pose jumps to it). Avoid. |
| Measurement std **∞** | Ignored on that axis. All current sources use ∞ for θ, since none of them measure heading. |

Start with all vision std devs at ∞ (vision off), check odometry alone, then lower them one
source at a time.

## Vision sources

### `LimelightTagCamera2d` – 2D Limelight

Computes the position from `tx`, `ty` and `tid` of a single tag plus the tag's place on the
field (`AprilTagFields.kDefaultField`).

- NetworkTables table: `"limelight-" + name`. The Limelight hostname must match.
- The offset's **height, pitch and yaw** are used in the distance math, so measure them
  carefully. Pitch is positive = camera tilted up.
- Each frame is read once, atomically, from the `t2d` array, so `tx`, `ty`, `tid` and the
  latency always come from the same frame. Only new arrays with a target and a known tag are used.
- Capture time = the time NetworkTables received the array − (`tl` + `cl`).
- Does not work when the tag is at the camera's height (distance becomes infinite; the
  frame is dropped).

### `LimelightTagCamera3d` – MegaTag2 Limelight

The Limelight computes the pose itself from every tag it sees, using the heading we send
every loop.

- The config offset is sent to the Limelight at startup and **overwrites** the camera pose
  set in the Limelight web UI.
- The Limelight's robot space has y pointing **right**, so `LimelightTagCamera3dConfig`
  mirrors y (`sourceFrame() = diag(1, -1, 1)`): position y, roll and yaw change sign, pitch
  stays. A camera at y = -0.3 facing right (yaw -90°) is sent as side +0.3, yaw +90°. The six
  values sent are on `vision/<name>/source pose sent`; they should match the Limelight web UI.
- Only x and y are used (MegaTag2's yaw is just the heading we sent).

### `Quest` – Meta Quest / QuestNav

The Quest tracks its own motion and reports poses relative to the last pose it was set to,
so it must be **anchored** to the fused estimate first.

- At startup and after every disconnect, RobotPose anchors it to the current estimate
  automatically.
- After an anchor, frames are ignored for 0.25 s (QuestNav doesn't acknowledge the reset).
- `resetPose` and `setYaw` re-anchor it too.
- Offset is robot center → headset.

## Dashboard

All values are read lazily, only when NetworkTables publishes them.

**Pose**

| Key | What |
|---|---|
| `pose/x`, `pose/y`, `pose/heading deg` | The fused pose. |
| `pose/field` | Field2d with the fused pose. |
| `pose/colliding` | True while a collision is limiting the wheel motion (only with collision detection on). |
| `pose/acceleration g`, `x g`, `y g` | Horizontal roboRIO acceleration, robot relative (0 while collision detection is off). |
| `pose/reset pose` | Button: `resetPose` to the origin. |
| `chassis/reset gyro` | Button: `setYaw(0°)`. |
| `chassis/reset gyro 180` | Button: `setYaw(180°)`. |

**Every vision source** (`vision/<name>/...`)

| Key | What |
|---|---|
| `is Connected` | The device is talking to the robot. |
| `seconds since used` | Age of the last used measurement (-1 if none yet). Keeps growing = the source isn't contributing. |
| `last error m` | Distance between the last used measurement and the estimate at its capture time. Consistently large = wrong offset or std devs too small. |
| `field` | Field2d with the robot pose from that source's newest frame. |
| `<quest name> distance m` | MegaTag2 Limelights only, with a Quest: distance between the two sources' newest measurements, when both had one in the same loop. |

**Per type**

| Source | Keys |
|---|---|
| 2D Limelight | `is see` (`tv`), `tag id`, `tag distance m` (of the last tag used). |
| MegaTag2 Limelight | `is see`, `tag count`, `avg tag distance m`, `source pose sent` (forward, side, up m; roll, pitch, yaw deg, in the Limelight's frame). |
| Quest | `should update`, `is tracking`, `is anchored`, `battery`, and a `Reset Quest Pose` button (re-anchors the Quest to the current estimate). A WPILib alert is raised once the battery is at or below 20%. |

## Adding a new kind of vision source

1. Write a class that extends `BaseVisionSource`. Read the device in `periodic()`; return
   that loop's measurements from `getPoseEstimates()` (each frame only once); return
   whether to use them from `shouldUpdate()`.
2. Write a config class that extends `BaseVisionSourceConfig` and sets `visionSourceType`.
3. Add a value to `BaseVisionSourceConfig.VisionSourceType` that creates your source.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Pose drifts when driving straight | Module locations in the wrong order, or wheel diameter / gear ratio wrong in module positions. |
| Pose rotates the wrong way | Gyro sign: the angle must be CCW-positive. |
| Pose jumps to vision every frame | A vision std dev is 0. |
| Vision never changes the pose | `seconds since used` keeps growing: check `is Connected` / `is see` (Quest: `is tracking`, `is anchored`). If it stays small: std devs are ∞, or odometry std is 0 on that axis. |
| Vision keeps pulling the pose around | Large `last error m` on one source: its offset is wrong, or its std devs are too small. |
| 2D camera is off by meters when turned | Wrong offset yaw, or camera height/pitch wrong. |
| Quest pulls the pose to (0, 0) | Something called `QuestNav.setPose` directly. Use `RobotPose.resetPose` or the dashboard button. |
