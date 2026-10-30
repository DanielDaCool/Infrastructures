# RobotPose – design and details (for mentors and developers)

The student guide is the [README](../README.md). This page has everything behind it: how the
estimator works, why it is built this way (with the simulation numbers), each vision source in
detail, the full calibration procedure, every tunable, and how it compares with other teams.

The code started from `frc.demacia.RobotPose` in DemaciaCode (`odometry-accelerometer-testing`
`f536d31`, `mechanism` `ed68a5c`) without the DemaciaCode-only dependencies (`Chassis`, `Log`,
`ElasticGenerator`), so it works in any project. [`Robot.java`](../Infrastructures/src/main/java/frc/robot/Robot.java)
has a placeholder setup (zero gyro and modules, no vision) so the project runs without hardware.

## Contents

| Path | What it is |
|---|---|
| `RobotPose.java` | The one class the rest of the robot uses. Singleton. Runs everything each loop. |
| `Estimation/DemaciaOdometry.java` | Swerve odometry (gyro + module positions → twist). |
| `Estimation/DemaciaPoseEstimator.java` | Fuses odometry and vision: moves the estimate back to each frame, WPILib's gain, jump filter with escape, tilt trust. |
| `Estimation/PhoenixOdometryThread.java` | Optional: reads the gyro and modules at 250 Hz (CAN FD) / 100 Hz (CAN 2.0) on a separate thread. |
| `Vision/VisionSource.java` | Interface every vision source implements. |
| `Vision/BaseVisionSource.java` | Base class for sources: config values and dashboard entries. |
| `Vision/VisionConfig.java` | The list of sources passed to `RobotPose.initialize`. |
| `Vision/TimestampedVisionMeasurement.java` | One measurement: pose, capture time, std devs. |
| `Vision/VisionTypes/LimelightTagCamera2d.java` | Limelight in 2D mode (tx/ty to one tag). |
| `Vision/VisionTypes/LimelightTagCamera3d.java` | Limelight using MegaTag2. |
| `Vision/VisionTypes/Quest.java` | Meta Quest running QuestNav. |
| `Vision/VisionTypes/QuestFrameFilter.java` | Which Quest frames are used: jump / lost tracking handling, re-alignment, 10 Hz thinning. |
| `Vision/visionConfigs/*Config.java` | One config class per source type. |
| `Vision/LimelightHelpers.java` | Limelight's official helper library. |

Vendor dependencies: `questnavlib` (for the Quest), Phoenix 6 (for `PhoenixOdometryThread`)
and WPILib New Commands.

## How it works

Each `periodic()`:

1. **Odometry first.** Every sample since the last loop (gyro + modules) is added at its
   own timestamp, so any vision frame captured up to now has odometry around it.
2. **Every source reads its device** (`VisionSource.periodic()`). The MegaTag2 Limelight
   sends the current heading here.
3. **Every source is checked.** A Quest that just (re)connected is re-anchored to the
   current estimate and skipped this loop. Any other source with `shouldUpdate()` true has
   its measurements added at their **capture time**. If the estimator says the Quest frames
   agree with each other but not with the estimate, the Quest's offset is shifted instead.
4. **Quest vs 3D Limelight.** Before those measurements are added, when a Quest and a
   MegaTag2 Limelight both have a new one, their distance is logged (the Quest's position is
   moved by the odometry between the two capture times first). If it stays over 0.3 m for
   1 s a WPILib alert is raised: one of the offsets is probably wrong.

### Odometry (`DemaciaOdometry`)

- Each module's motion during one loop is treated as a circular arc (its wheel angle
  changed during the loop); the chord of that arc is its displacement.
- The robot's displacement is the average of the modules.
- The heading always comes from the gyro, never from the wheels.
- A sample with a NaN or infinite value moves nothing (the next good one covers the motion).
  A module whose distance jumps more than 0.5 m in one sample (encoder reboot) is left out of
  that sample.

### Fusion (`DemaciaPoseEstimator`)

- Each odometry sample moves the estimate and goes into a 1.5 s history of odometry-only
  poses. A vision frame taken at time t: the estimate is moved back to t by the odometry since
  then, corrected, and moved forward again. One step per frame; frames out of order land
  within a few mm of a full replay.
- Each frame moves the estimate by `K = q / (q + sqrt(q r))` of the way per field axis
  (WPILib's gain; q = odometry variance, r = frame variance).
- **Jump filter:** a frame whose distance from the estimate is unlikely for q + r (chi², 99.9 %)
  is rejected. When 15 of the last 25 frames of one kind (cameras together, Quest separately)
  were rejected but agree within 0.3 m, the estimate is the one that is wrong and the frame is
  used; the Quest is shifted by the same amount, since it was anchored to the wrong estimate.
  When Quest frames escape, only the Quest's offset is shifted. Off while disabled (the robot
  may be carried).
- Dropped: NaN/inf, more than 0.5 m outside the field, older than the history (a reset clears
  it), more than 50 ms newer than the newest sample.
- Yaw rate (for the MegaTag2 spin check): each sample's rotation since the newest sample at
  least 100 ms older (raw times). Not since the previous sample: on CAN 2.0 the thread sleeps
  instead of waiting for the gyro frame, so one sample can hold two gyro frames and the next
  none (up to 1.8x the real rate, MegaTag2 rejected from about 1.1 rad/s). Until there are
  100 ms of samples (startup, and after a reset or `setYaw`, whose first twist spans the
  reset) and for a sample out of order, the previous rate is kept. `getFuturePose` divides
  the first twist after a reset by the time since the sample before the reset.
- θ: only changed by a frame with a finite θ std dev, with the θ odometry std dev as a fixed
  gain (none of the current sources measure heading).

### Tilt trust

On a bump or ramp the wheels spin in the air or count the climb as floor distance. Like 6328's
2026 RobotState, each odometry sample's translation is scaled by
`1 − clamp(tilt / 25°, 0, 1)`, where `tilt = acos(cos(pitch) · cos(roll))`; the heading (gyro)
is not affected. Give the tilt in the sample (`OdometryData.tilt(pitch, roll)` computes it);
samples built with the old constructors are level (no scaling).

Once per loop, from the Pigeon 2 (`getPitch()` / `getRoll()`, degrees):

```java
() -> List.of(new OdometryData(Timer.getFPGATimestamp(), chassis.getGyroAngle(), chassis.getModulePositions(),
    OdometryData.tilt(Math.toRadians(pigeon.getPitch().getValueAsDouble()),
                      Math.toRadians(pigeon.getRoll().getValueAsDouble()))))
```

In the 250 Hz thread: clone the two signals, pass them to the thread with the others, and read
them in the reader (see below).

## What changed (2026-10-22/24) and why

All numbers are from the loop simulation used while writing this version (`LoopSim`, removed
with the test files in commit c4b349f), not from a robot, on fresh seeds never used for tuning:
10 seeds x 17 scenarios x 13 conditions (latency, noise, Limelight outliers, Quest bias,
traction, accelerometer noise). Mean position error over all of them. The current simulation
tests use other scenarios and don't reproduce these numbers:

| Estimator | Mean error |
|---|---|
| WPILib's estimator, wired the way the team wires it | about 20 cm |
| The previous version of this package | about 21 cm |
| Shared fixes only (frame handling below, WPILib gain) | 10.6 cm |
| **This version** (shared fixes + jump filter with escape) | **8.6 cm** |
| With trust that follows an uncertainty and a slip/collision detector (tried, not kept) | 6.5 cm |

**Why not the 6.5 cm version:** it needed 17 tunables (most only tunable against ground truth),
about 20 values kept between loops and the roboRIO accelerometer (mounting, vibration, an
untested 5 g threshold), and without a working detector it was worse than the simple one. This
version has 5 tunables (the odometry std devs, the chi² threshold and the three escape numbers),
keeps 3 values between loops (two escape windows and the enabled flag) and needs no extra sensor.
Most of the gain from 20 to 10 cm comes from how the frames are handled, not from the estimator.

| Change | Why (simulation) |
|---|---|
| Vision frames move the estimate back to their time, correct it, move it forward (no more replay of 1.5 s per frame) | about 100x less work per frame (estimate, not measured; the "problematic cpu" commit); within mm of a full replay; using frames at their capture time is worth 50 % of the error |
| Jump filter: a frame unlikely for the state + frame std devs (chi², 99.9 %) is rejected; 15 of the last 25 rejected frames that agree within 0.3 m are accepted (escape); off while disabled | 10.6 to 8.6 cm; Quest-jump error 13 to 6 cm; without the disabled bypass a carried robot was 28 cm off instead of 12 |
| A camera escape shifts the Quest too (it was anchored to the same wrong estimate) | wrong start pose 31 to 19 cm in the simulation, mean 8.35 to 7.63 cm (nominal); not in the 8.6 cm row above |
| Tilt trust: wheel translation scaled by 1 − tilt/25° (6328, 2026) | bumps: wheels spin or count the climb (end-to-end test: 48 to 17 cm over one bump, odometry only) |
| Safety: NaN/inf, outside the field, too old/new frames | never triggered in simulation; a few lines each |
| Odometry: NaN/inf samples skipped, a module whose encoder jumps > 0.5 m left out | one NaN made the pose NaN forever; an encoder reboot added 1.06-2.5 m for good, now 1 cm |
| Odometry thread: reader can move each signal to the sample time (`valueAt`) | a gyro/module skew costs ~2.5 cm per ms when driving and spinning fast |
| Odometry thread: alert while signals fail; `changeModuleWeight` removed (never called) | a dead signal stopped all samples without a sign |
| Limelight std = hypot(0.008 d² / √tags, 0.01 d) | removing it costs +30 % |
| MegaTag2 frames dropped when spinning > 2 rad/s (peak over 0.1 s) or heading off by > 3° | guards gyro resets, stale heading and a wrong camera yaw |
| Quest: latency subtracted, jump / lost tracking: ignore 0.5 s and re-align, thinned to 10 Hz with std 0.4, disconnects debounced 0.5 s | Quest-jump error 49 cm to 6 cm; without thinning +53 % and 26-28 lock-outs |
| `getFuturePose(t)` from the odometry velocity | a vision correction no longer reads as speed (estimate velocity doubles the 0.5 s error) |
| `BaseVisionSource.setOnCreate`, `Quest(QuestConfig, QuestNav)` | drop-in for the team chassis (below), tests |

**Removed:** collision detection and the roboRIO accelerometer (`enableCollisionDetection`,
`RoboRioOrientation`, the drop of wheel motion into a hit), and the vision drop before the first
sample after a reset (the reset clears the history, so older frames are dropped anyway).

## 250 Hz odometry

Odometry adds up small steps, and each step assumes the robot moved in one smooth arc
between two readings. At 50 Hz a step is 20 ms; when the robot accelerates or turns during
it, the guess is off and the error stays in the pose. At 250 Hz a step is 4 ms, so each
guess is much closer. In 6328's testing, 250 Hz autos ended in almost the same place every
run (84% smaller spread than 50 Hz).

`PhoenixOdometryThread` reads the signals on its own thread. Each cycle it waits until the
gyro yaw and every drive/steer position have a new value (`BaseStatusSignal.waitForAll`),
builds a sample, stamps it with the FPGA time minus the signals' CAN latency, and queues it.
Each robot loop `RobotPose` takes the ~5 queued samples and adds them all. The pose math stays on the main thread; the thread only reads CAN.

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
        double tilt = OdometryData.tilt(Math.toRadians(pitch.getValueAsDouble()),
                Math.toRadians(roll.getValueAsDouble()));             // pitch = pigeon.getPitch().clone(), roll likewise
        return new OdometryData(0, Rotation2d.fromDegrees(yaw.getValueAsDouble()), modules, tilt); // the thread sets the time
    },
    yaw, pitch, roll, drive[0], steer[0], drive[1], steer[1], drive[2], steer[2], drive[3], steer[3]);

RobotPose.initialize(odometryThread::getNewSamples, ...);
```

- The thread starts in the constructor and sets these signals to its frequency. If you use
  `optimizeBusUtilization`, call it after creating the thread.
- The unit conversions must match your motor configs (`SensorToMechanismRatio`). Any extra
  math your module does (e.g. a steer/drive coupling correction) goes in the reader too,
  using only these signals.
- `getFailedCycles()` counts cycles where a signal didn't arrive in time (e.g. an unplugged
  device) or the reader threw; an alert ("odometry thread: a signal stopped updating") is on
  while it keeps going up. A reader exception is also reported to the driver station (at most
  once a second) and the thread keeps running; it stops only when interrupted. If no sample
  arrives for more than 0.1 s (the thread died from an `Error` or the reader blocks), an alert
  ("odometry thread: no samples") is on, since the pose is frozen.
- **Gyro vs module timing.** The gyro and the modules arrive in separate frames and can be a
  few ms apart (about 2.5 cm of error per ms when driving 2.5 m/s and spinning 4.5 rad/s). Use
  the constructor whose reader gets the sample time and move each position to it with its
  velocity signal (add the velocity signals to the thread's signals):

  ```java
  new PhoenixOdometryThread(canBus, PhoenixOdometryThread.CAN_FD_FREQUENCY_HZ,
      time -> new OdometryData(time,
          Rotation2d.fromDegrees(PhoenixOdometryThread.valueAt(yaw, yawRate, time)), // yawRate = pigeon.getAngularVelocityZWorld().clone()
          modules(time)),  // drive: valueAt(drive[i], driveVelocity[i], time) * WHEEL_CIRCUMFERENCE
      yaw, yawRate, drive[0], driveVelocity[0], steer[0], ...);
  ```
- Vision is unchanged. A frame up to 50 ms newer than the newest odometry sample is added
  to that sample instead of being dropped (the newest sample is a few ms old).

### Choosing std devs

Each vision frame moves the estimate by `K` of the way to the measurement, per axis, with
`K = 1 / (1 + σ_vision / σ_odometry)`. The vision σ comes from the tag distance (Limelights) or
`QuestConfig.thinnedStd` (Quest), see the [README](../README.md), step 1. The odometry σ also sets the jump filter:
a frame is rejected when its distance from the estimate is more than about
`3.7 · sqrt(σ_odometry² + σ_vision²)` (0.2 m with 0.05 and 0.02). Examples with odometry σ = 0.1:

| Vision σ | K | Each frame |
|---|---|---|
| 0.05 | 0.67 | Pulls two thirds of the way (e.g. a Quest). |
| 0.3 | 0.25 | Pulls a quarter of the way. |
| 0.9 | 0.10 | Small nudge (far or single-tag frames). |

| Value | Meaning |
|---|---|
| Odometry std **larger** | Vision corrects the pose faster, and frames further off pass the jump filter. |
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
- Only the offset's **height, pitch and yaw** (plus its x and y position) are read, and they
  are used in the distance math, so measure them carefully. Roll is ignored.
- Pitch is positive = camera **tilted up**, the opposite of WPILib's right-hand pitch. This
  is the one exception to the frame contract (see the [README](../README.md), step 1).
- The math is exact for a pinhole camera (Limelight's `tx`/`ty` are each the atan of the
  normalized image coordinate on their axis), including a pitched camera with the tag off to
  the side.
- Each frame is read once, atomically, from the `t2d` array, so `tx`, `ty`, `tid` and the
  latency always come from the same frame. Only new arrays with a target and a known tag are used.
- Capture time = the time NetworkTables received the array − (`tl` + `cl`).
- Does not work when the tag is at the camera's height, or when the ray to the tag never
  reaches the tag's height (the frame is dropped).

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
- A frame is dropped if the robot turned faster than 2 rad/s in the 0.1 s before it, or if
  the heading in the frame (the one the Limelight used) differs from ours at its time by more
  than 3° (after a gyro reset, while spinning, or with a wrong camera yaw).
- x/y std dev = hypot(0.008 · d² / √tags, 0.01 · d), d = average tag distance (the config std
  devs are not used). The 2D Limelight uses the same formula with its tag distance and one tag.

### `Quest` – Meta Quest / QuestNav

The Quest tracks its own motion and reports poses relative to the last pose it was set to,
so it must be **anchored** to the fused estimate first.

- At startup and after every disconnect, RobotPose anchors it to the current estimate
  automatically.
- After an anchor, frames are ignored for 0.25 s (QuestNav doesn't acknowledge the reset).
  This is checked per frame, by its capture time. The first frame after that re-aligns the
  Quest to the estimate: QuestNav's `setPose` has no timestamp and the headset applies it a
  little later, so an anchor sent while moving is off by speed x that delay. If the headset
  applies it later than 0.25 s, the pose change reads as a jump (below) and is re-aligned then.
- `resetPose` and `setYaw` re-anchor it too.
- Offset is robot center → headset.
- Every unread frame of the loop that is tracking is used, each at its own capture time
  (not only the newest). Only x and y are measured; the heading is copied from the estimate
  at that time and sent with an infinite std dev.
- The newest frames are often a few ms newer than the newest odometry sample. A frame up to
  50 ms newer is added to that sample; one further ahead is dropped.
- QuestNav stamps frames with the publish time; `QuestConfig.latencySeconds` (0.03) is
  subtracted.
- If the Quest moves more than 0.3 m differently from the odometry between two frames, or no
  frame comes for 0.2 s (tracking lost), its frames are ignored for 0.5 s and then it is
  re-aligned to the estimate (a software offset; the headset is not reset).
- At most one frame per 0.1 s is used, with std 0.4: Quest errors are the same from frame to
  frame, so 100 Hz frames must not outvote the cameras.
- A disconnect only counts (and re-anchors) after 0.5 s.

## Calibration, step by step

The estimator can only be as good as its inputs. Most pose problems on real robots are a wrong
wheel size, a wrong camera offset or a wrong latency, not the fusion. Calibrate in this order,
because each step relies on the ones before it, and **turn vision off** (std devs ∞) until step 4.

### 1. Wheels (odometry distance)

- Drive a taped straight line of 5-10 m slowly (no slip), with vision off. Compare the odometry
  distance with the tape: `new circumference = old × tape / odometry`. 1 % of wheel size is 1 % of
  every distance, so a 2 % error is 20 cm after 10 m.
- Repeat forwards, backwards and sideways. If sideways differs from forwards by more than ~1 %,
  the carpet is anisotropic; use the average and let vision absorb the rest.
- Re-check after replacing treads: worn treads shrink the wheel.

### 2. Gyro (heading)

- Spin the robot 10 full turns in place and stop at the start heading (use a line on the floor).
  The gyro should read 3600° ± ~5°. A larger error is a gyro scale or mounting problem; the
  heading is never corrected by vision in this package, so it must be right.
- Check that CCW is positive (WPILib convention). A mirrored gyro makes the pose spiral.
- For tilt trust: on flat carpet `OdometryData.tilt(pitch, roll)` must read about 0. A mounting
  tilt of a few degrees would scale every sample down; zero it with the Pigeon's mount pose.

### 3. Camera offsets (each camera)

The offset is robot center → camera lens in the WPILib robot frame (x forward, y left, z up,
rotations CCW positive; the 2D Limelight's pitch is positive = tilted up, see §1 of the Quick
start). Measure x, y, z with a tape from the robot center on the floor. Angles are calibrated
below, because a protractor is not accurate enough: 1° of pitch error is already
decimeters at a few meters.

**Pitch (2D Limelight and MegaTag2).** The 2D distance is
`d = (tag height − camera height) / tan(camera pitch + ty)`. Its sensitivity to the pitch is
`|∂d/∂pitch| = (Δh² + d²) / Δh` (Δh = tag height − camera height):

| Δh | d = 2 m | d = 4 m |
|---|---|---|
| 1.0 m | 9 cm per degree | 30 cm per degree |
| 0.5 m | 15 cm per degree | 57 cm per degree |

So: the smaller the height difference and the farther the tag, the more a pitch error costs.
Procedure (Orbit 1690's): put the robot close to a tag (≈1-1.5 m) with the tag in the image
center (tx ≈ 0, ty ≈ 0, so roll doesn't matter yet), measure the true distance with a tape, and
adjust the pitch until the computed distance matches. Then check at 3-4 m; if it is off there,
the camera height is wrong, not the pitch (pitch errors grow with distance, height errors don't).

**Roll.** A rolled camera mixes tx into ty, so the distance is right in the image center and
wrong at the edges. Two checks: (a) at a fixed distance, turn the robot so the tag moves from the
left edge to the right edge of the image: the computed distance must not change; (b) Orbit's
check: spin the robot slowly in place in front of a tag; the estimated position must stay still.
Adjust the roll until it does.

**Yaw (and U1).** A yaw error δ moves the pose sideways by about `d · δ` (1° = 7 cm at 4 m), in
opposite directions on the left and right of the tag. Check: stand 3 m straight in front of a tag
and read the pose; then turn the robot ±30° (the tag still in view): the pose must not move.
For the MegaTag2 camera this is also the **U1 check**: with the wrong yaw sign the pose is off by
meters and the heading-agreement check (`frames rejected (spin or heading)`) rejects every frame.

**x / y offset.** Spin in place slowly with a tag in view (vision still off in the estimator, but
log the camera pose): with a wrong x/y offset the camera's robot pose traces a **circle** whose
radius is the offset error, centered on the true position. Shrink the circle to a point by
adjusting x and y. Do this after the angles, because a yaw error also bends the circle.

### 4. Vision trust (std devs) and latency

- **Std devs.** Stand still at 1, 2, 3, 4 and 5 m from tags, with 1 tag and with 2+ tags in view,
  10 s each. The spread (std) of the camera pose at each distance gives the curve
  `σ = STD_PER_METER_SQUARED · d² / √tags`; fit `STD_PER_METER_SQUARED` to it. Moving frames are
  noisier, which the spin gate and the jump filter handle.
- **Limelight latency.** The package uses the Limelight's own timestamp (capture time). Check it:
  drive at a constant 2 m/s past a tag with vision fused; if the camera pose consistently trails
  (or leads) the odometry along the direction of travel by x m, the timestamp is off by x / 2 s.
  Also log `now − capture time`: it should be stable (tens of ms), not growing.
- Now turn vision on, one source at a time, and watch `last error m` on the dashboard: it should
  look like noise around 0, not a constant offset (wrong offset) or a lag (wrong latency).

### 5. Quest

- **Offset.** Spin in place slowly: the Quest's robot pose must stay still. A circle means the
  Quest offset (x, y) in `QuestConfig` is wrong, exactly like a camera (§3, x / y).
- **Yaw offset.** Drive a straight taped line: the Quest's path must point the same way as the
  odometry's. An angle between them is a Quest yaw offset error.
- **Latency.** QuestNav stamps frames with the publish time, so `QuestConfig.latencySeconds` is
  subtracted. Drive back and forth at 2 m/s: choose the latency that lines the Quest position up
  with the odometry (no lag in either direction).
- **Drift check.** Drive a 2-minute pattern and return to the start mark: the Quest error at the
  end, with tags covered, is its drift per match; the tags and the escape correct it, but a large
  drift (> ~10 cm/min) means the Quest is not tracking well (lighting, vibration, mounting).

### 6. Whole-system check

Drive a full auto path and a minute of hard teleop (with a few bumps and a light push), then stop
on a taped mark: the pose error at the end should be a few cm. Then repeat with vision covered
for 10 s in the middle: the error should grow while covered (odometry drift) and come back within
about a second when the tags are visible again. Log `pose/frames rejected` and `pose/escapes`:
rejections should be rare, escapes only after real events (a big push, a wrong start pose).

## Tunables and how to measure them

Defaults are the values tested in simulation.

| Where | Value | Measure |
|---|---|---|
| odometry std devs (`initialize`) | 0.05 m, 0.05 m, 0 | Log the distance between each frame and the estimate (`last error m`) while driving with tags in view: most of them should be under 3.7 · sqrt(0.05² + σ_vision²). |
| `DemaciaPoseEstimator.GATE_CHI_SQUARED` | 13.82 | A statistical constant (99.9 %); leave it. |
| `DemaciaPoseEstimator.ESCAPE_REJECTED_FRAMES` / `_WINDOW_FRAMES` / `_SPREAD_METERS` | 15 / 25 / 0.3 m | Judgement: how many agreeing frames prove the estimate wrong. Watch `pose/escapes` after a wrong start pose. |
| `DemaciaPoseEstimator.MAX_TILT_RADIANS` | 25° | 6328's value; log the tilt and the pose error against the Quest over the bumps. |
| `BaseVisionSource.STD_PER_METER_SQUARED` | 0.008 | Robot standing still at 1-5 m, 1 and 2+ tags: spread of the camera pose vs distance. |
| `LimelightTagCamera3d.MAX_YAW_RATE_RADIANS_PER_SECOND` | 2 rad/s | Log the MegaTag2 error against the yaw rate while spinning. |
| `QuestConfig.latencySeconds` | 0.03 s | Drive back and forth; the shift that lines the Quest position up with the odometry. |
| `QuestConfig.thinnedStd`, `periodSeconds` | 0.4 m, 0.1 s | Tuned in simulation (0.4 better than 0.1-0.2 with the fixed gain). |
| `QuestConfig.jumpMeters`, `gapSeconds`, `blackoutSeconds` | 0.3 m, 0.2 s, 0.5 s | Cover the Quest cameras: the jump after it re-localizes must be caught. |

## Robot checks (before relying on it)

1. **U1**: the 3D Limelight yaw sign (see "Plugging it into the team chassis"), before switching the import.
2. **Tilt**: on flat carpet `OdometryData.tilt(pitch, roll)` should read about 0 (a mounting
   offset would scale every sample down); then drive over the bump with vision off and compare
   the end position with a tape measure.
3. **Loop time** before/after (the replay was the suspected CPU problem).
4. **Latencies**: Quest (`latencySeconds`), Limelight (log now − capture time), and the gyro vs
   module offset in the odometry thread (`yaw.getTimestamp().getLatency()` minus a drive
   signal's); over 1 ms, use `valueAt`.
5. **Wheel calibration drive**: drive a taped 5-10 m straight line, compare with the odometry
   and correct the wheel circumference (1 % of wheel size = 1 % of distance).

## Plugging it into the team chassis (`mechanism` / `demacia-odometry-simple-testing`)

1. **Settle U1 first** (3D Limelight yaw sign): the package sends the
   camera yaw mirrored compared to both team branches. Check `vision/<name>/source pose sent`
   against the Limelight web UI on the robot.
2. Change the imports from `frc.demacia.RobotPose.*` to `frc.robot.RobotPose.*` (Chassis,
   RobotContainer, VisionConstants, ShootingValues, ElasticGenerator).
3. Replace the `initialize` call in RobotContainer (the package can't read `Chassis` itself):

   ```java
   var c = Chassis.getInstance();
   RobotPose.initialize(() -> List.of(new OdometryData(c.getGyroAngle(), c.getModulePositions())),
       c::setGyroYaw, c::setGyroYaw /* resetPose also writes the gyro */, c.getModuleLocations(),
       RobotCChassisConstants.STATE_STD, VisionConstants.visionConfig);
   BaseVisionSource.setOnCreate(ElasticGenerator.getInstance()::registerVisionSource);
   ```

   Passing the second `setGyroYaw` matters: `DriveCommand` and `getChassisSpeedsFieldRel` read
   the raw gyro, so a `resetPose` that only stored an offset would rotate field-relative driving.
4. `getFuturePose(double)` exists (ShootingValues keeps compiling). Remove the team
   `BaseVisionSource`'s `registerVisionSource(this)` (the hook above does it).
5. The dashboard keys move from `chassis/field` / `chassis/reset pose` to `pose/field` /
   `pose/reset pose`.
6. For tilt trust, add the Pigeon's pitch/roll to the sample (see "Tilt trust"). The team's
   MegaTag2 std is no longer used (distance-based); the Quest's 0.03 is raised to `thinnedStd`
   0.4. `STATE_STD` 0.05 is what was tested.

## How this compares with other approaches

| | This package | WPILib `PoseEstimator` | 6328 (2026) | 254 (2024) | Orbit 1690 (2024 lecture) |
|---|---|---|---|---|---|
| Odometry | arc per sample, gyro heading, 250 Hz option, per-signal latency, NaN / encoder-jump guards, tilt trust | arc, gyro heading | arc, gyro heading, 250 Hz, tilt trust, wheel-heading fallback | CTRE | per-module arcs averaged, gyro heading |
| Fusion | move back to the frame time, fixed gain, move forward | fixed gain, replays its buffer; an older frame deletes newer corrections | same as this package | CTRE (WPILib-like) | weights 1/FOM²; FOM grows with time, skids, collisions, speed |
| Vision trust | `d² / √tags`, spin gate, heading-agreement check, field check, jump filter + escape | none (user's job) | `d² / tags²`, field and height checks, ambiguity | tiered by distance, spin-rate gates | geometric 2D (tx / ty + gyro) at range, MegaTag only close |
| Slip / collisions | tilt trust; vision recovers the rest (jump filter + escape) | none | tilt trust | none | skid ratio (drop the slipping module), accelerometer vs expected motion |

What was tested here and not adopted, and when it would be worth it:

- **Orbit's skid detection** (translational speed of each module, max / min > 1.25 → drop the
  slipping module): in simulation it only helps when **one** wheel slips or fails (odometry alone:
  19-38 cm → 6-8 cm); it cannot see all four wheels slipping together, and with vision on the gain
  is under 2 %. Worth ~35 lines if robot logs show one module disagreeing by > 25 % outside collisions.
- **Trusting vision more after a collision** (Orbit's FOM idea; accelerometer minus expected
  acceleration > 2 g): defense pushing 18 → 11.5 cm in simulation, but it needs the roboRIO
  accelerometer path back (~150 lines) and lets some bad frames through right after a hit. Worth
  it if real matches show the pose lost after pushes. Mount the roboRIO near the robot center,
  or the spin acceleration eats most of the 2 g margin.
- **Uncertainty tracking (variance per axis, "K")**: the most accurate in simulation (6.3 vs 7.6 cm)
  but 17 tunables, ~20 state values and a slip detector shaped on the simulator; see "What changed".

## Adding a new kind of vision source

1. Write a class that extends `BaseVisionSource`. Read the device in `periodic()`; return
   that loop's measurements from `getPoseEstimates()` (each frame only once); return
   whether to use them from `shouldUpdate()`.
2. Write a config class that extends `BaseVisionSourceConfig` and sets `visionSourceType`.
3. Add a value to `BaseVisionSourceConfig.VisionSourceType` that creates your source.

