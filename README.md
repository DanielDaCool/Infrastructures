# Infrastructures – Kinematics

Swerve kinematics and a velocity-setpoint acceleration limiter for a swerve robot.

The code lives in [`Infrastructures/src/main/java/frc/robot/Kinematics`](Infrastructures/src/main/java/frc/robot/Kinematics).
It started from the kinematics and `SwerveAccelerationLimiter` in DemaciaCode
(`kinematics-acceleration-limit-testing` a54f5a4), with no `Chassis`, `Log` or
`ElasticGenerator` dependencies, so it works in any project. The kinematics math was then fixed
here and no longer matches DemaciaCode (see [Differences from DemaciaCode](#differences-from-demaciacode)).

## Contents

| Path | What it is |
|---|---|
| `DemaciaKinematics.java` | Swerve kinematics, the same math as WPILib's `SwerveDriveKinematics`. Module states from robot speeds (desaturated), robot speeds from module states (the exact inverse). |
| `SwerveAccelerationLimiter.java` | Limits how fast the field-relative velocity setpoint changes (skid, forward and tilt limits). |
| `AccelLimitedSwerve.java` | The class a robot uses: runs the limiter and the kinematics once per loop and keeps the speeds it sent. |
| `ModuleCommand.java`, `ModuleCommandConfig.java` | The steer-aware command for one module and its settings (see [Steer-aware commands](#steer-aware-commands-latency-cosine-scaling-feed-forwards)). |
| `KinematicsConstants.java` | `MAX_ALLOWED_MODULE_VELOCITY` (desaturation), `MIN_VELOCITY` (below this the robot stands), loop time. |
| `KinematicsUtilities.java` | Small vector helpers (`limitVector`, `isInRange`, ...). |

Every module location is in the **WPILib robot frame**: x forward, y left, rotations CCW
positive, meters from the robot center. Speeds are m/s and rad/s, CCW positive.

The limits in `SwerveAccelerationLimiter` and `KinematicsConstants` are placeholders. Tune
them on the robot.

## Using it from a robot

The chassis owns one `AccelLimitedSwerve` and sends every drive command through it. Four
things have to be right:

1. **`calculate` once per loop**, with the wanted field-relative speeds and the gyro heading.
   It keeps the speeds it sent and starts the next call from them, with dt measured from the
   previous call. Two callers in the same loop (e.g. two commands) fight: the second one gets
   almost no time to change anything, so it sends about what the first one did.
2. **`resetTo` when the limiter wasn't used last loop** (robot just enabled, another command
   drove the modules directly, a path follower or a characterization routine). Otherwise
   the limiter starts from whatever it sent the last time it ran, not from what the robot
   is doing now.
3. **`onYawReset` when the gyro yaw is set.** The stored speeds are field-relative; without
   it the same robot motion is suddenly seen from another angle and the limiter turns the
   robot toward the old field direction.
4. **The robot's own module logic sends the states**: optimize, cosine scaling, holding a
   module's angle at low speed. `calculate` only returns the states.

### 1. Create it (once)

```java
// Module locations, robot frame, in the same order as the modules.
private final Translation2d[] moduleLocations = {
    new Translation2d(0.32, 0.27),    // front left
    new Translation2d(0.32, -0.27),   // front right
    new Translation2d(-0.32, 0.27),   // back left
    new Translation2d(-0.32, -0.27) };// back right

private final DemaciaKinematics kinematics = new DemaciaKinematics(moduleLocations);
private final AccelLimitedSwerve accelLimitedSwerve = new AccelLimitedSwerve(kinematics);
private boolean limiterUsedLastLoop = false;
```

### 2. Drive through it

```java
/** Field-relative drive with the acceleration limiter. Call once per loop. */
public void driveFieldRelative(ChassisSpeeds wantedFieldRel) {
    // Heading hold, if the robot has one, goes into wantedFieldRel.omegaRadiansPerSecond
    // before this call.
    SwerveModuleState[] states = accelLimitedSwerve.calculate(wantedFieldRel, getGyroAngle());
    setModuleStates(states);
    limiterUsedLastLoop = true;
}
```

### 3. Re-sync in `periodic`

`SubsystemBase.periodic()` runs before the commands' `execute()` in each
`CommandScheduler.run()`, so here the flag still says whether last loop's command used the
limiter:

```java
@Override
public void periodic() {
    if (!limiterUsedLastLoop) {
        SwerveModuleState[] measured = getModuleStates();
        ChassisSpeeds measuredRobotRel = kinematics.toChassisSpeeds(measured, getGyroRateRadPerSec());
        accelLimitedSwerve.resetTo(
            ChassisSpeeds.fromRobotRelativeSpeeds(measuredRobotRel, getGyroAngle()),
            measured);   // the modules keep these angles while the robot stands
    }
    limiterUsedLastLoop = false;
}
```

Drive methods that don't use the limiter (robot-relative driving, a path follower sending
module states) just don't set the flag, so the next limited command starts from the
measured speeds.

### 4. Gyro reset

```java
public void setYaw(Rotation2d newYaw) {
    Rotation2d oldYaw = getGyroAngle();
    gyro.setYaw(newYaw.getDegrees());
    accelLimitedSwerve.onYawReset(oldYaw, newYaw);
}
```

Call it from every place that writes the gyro yaw (driver button, auto start, a pose reset
that writes the gyro).

### 5. Send the states with the robot's own module logic

```java
public void setModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < modules.length; i++) {
        Rotation2d current = modules[i].getAngle();
        SwerveModuleState state = states[i];
        state.optimize(current);      // turn at most 90 degrees, reverse the wheel instead
        state.cosineScale(current);   // slow down while the module is still turning
        modules[i].setState(state);
    }
}
```

- `optimize` flips both the angle and the speed, so the module still moves along the same
  vector and the speeds `AccelLimitedSwerve` stored stay right.
- When the robot should stand, `calculate` already returns speed 0 with each module's last
  angle, so the modules don't turn back to 0. Anything more (e.g. holding a module's angle
  below some speed while the robot moves) is the robot's own logic, applied here.

### Reading what was sent

| Method | What it returns |
|---|---|
| `getSentSpeedsFieldRel()` | Field-relative speeds sent last loop, after desaturation. |
| `getSentSpeedsRobotRel()` | The same, robot-relative (with the heading of that loop). |

These are the speeds the module states really ask for (read back with
`DemaciaKinematics.toChassisSpeeds`, with the discretize undone), not the wanted ones: the
robot's mean velocity over the loop.

Speeds measured from module states (as in `resetTo` above) are the module vectors, so while the
robot turns their direction is about ω·dt/2 off from the mean velocity. That is small and the
limiter catches up in the next loops.

## How it works

Each `calculate`:

1. **Limit.** `SwerveAccelerationLimiter` moves the last sent field-relative velocity toward
   the wanted one by the closest change that every limit allows in the time since the last
   call (at most `MAX_DT`):
   - skid: total change ≤ `MAX_SKID_ACCEL · dt`;
   - forward: the speed can grow by at most `MAX_FORWARD_ACCEL · (1 - speed / FREE_SPEED) · dt`
     (braking and turning are free here);
   - tilt: change along the robot's front ≤ `MAX_TILT_ACCEL_FRONT · dt`, along its side
     ≤ `MAX_TILT_ACCEL_SIDE · dt`.

   Omega is passed through unchanged.
2. **To the robot.** The limited speeds are turned robot-relative with the heading.
3. **Standing?** Below `MIN_VELOCITY` (both translation and omega) every module gets speed 0
   at its last angle.
4. **Desaturate.** `DemaciaKinematics.toSwerveModuleStates`: module at (x, y) moves at
   (vx - ω·y, vy + ω·x), then all modules are scaled by one factor if any is above
   `MAX_ALLOWED_MODULE_VELOCITY`, so the robot keeps its direction and its turn/drive ratio.
   The states are read back into the speeds the modules can really drive.
5. **Discretize.** `ChassisSpeeds.discretize(speeds, CYCLE_DT)`, then the module states again.
   A state held for one loop while the robot turns moves it along the wanted direction turned
   by ω·dt/2; without this the robot drifts sideways while it spins. The discretize makes the
   translation slightly bigger, so at full speed the states are desaturated again, which leaves
   about a millimeter of drift per 2 s while spinning fast.
6. **Store.** The states are read back into robot speeds, the discretize is undone (the chord
   of one loop's arc, over dt) and they are turned field-relative; the next loop starts there.

A heading that is NaN or infinite (gyro glitch) is replaced by the last heading before the
limiter runs. A gyro that stays NaN freezes the heading.

The stored speeds are field-relative on purpose: kept robot-relative, a robot turning while
driving would see its last speeds turn with it every loop, and the limiter could not keep up.

`DemaciaKinematics` has no lead-angle term (see its class javadoc); `AccelLimitedSwerve`
discretizes instead.

## Steer-aware commands (latency, cosine scaling, feed-forwards)

`calculate(wanted, heading)` returns plain module states. The steer-aware overload returns
`ModuleCommand`s, which also use what the modules are really doing, with four small, independent
additions on top of the same limiter, desaturate and discretize:

| Step | What it fixes | Measured gain (simulation) |
|---|---|---|
| Latency compensation: the heading is advanced by `gyroRate · latencySeconds` | The robot-relative command is rotated by the heading it will have when the command takes effect, so a spinning robot doesn't curve | Second biggest: without it spin-while-driving cross-track error goes from 1.3 to 59 cm (spin25_4), fullspin 1.4 to 70 cm |
| Reverse decision against the previous **commanded** angle, 10° hysteresis, one shared decision for the modules near 90° | `optimize` against the measured angle flips a wheel back and forth when the steer lags; here modules at 89°/91° don't split | Small: mixed, mostly neutral |
| Cosine scaling against the measured angle advanced by `measuredAngleLatencySeconds` | A wheel pointing the wrong way (start from rest, a stick snapped faster than the steer, a stuck module) gets ~0 speed instead of scrubbing | Slow module (x0.25) cross-track 13.2 to 9.9 cm, locked module 19.7 to 10.4 cm, strafe from rest 5.2 to 2.1 cm; neutral for normal driving |
| Drive acceleration feed-forward from the plan (`(speed − previous speed) / dt`, clamped) | The drive velocity loop lags the setpoint; the feed-forward leads it | Largest: without it fullspin goes from 1.4 to 51 cm, spin25_4 1.3 to 17 cm, figure-8 9.5 to 15 cm |
| Steer velocity feed-forward (optional, `PositionVoltage` steer only) | A position-only steer lags a moving angle | With a `PositionVoltage` steer: figure-8 with spin 9 to 23 cm, spin25_4 2.3 to 39 cm when it is missing. With MotionMagic leave it off |

The gains are from a simulation (council/simple.md), not from a robot. Tune on the robot.

### Wiring

```java
private final AccelLimitedSwerve swerve = new AccelLimitedSwerve(kinematics,
    new ModuleCommandConfig(0.035, 0.035, 12, 20, false)); // see "Measure on the robot"

public void driveFieldRelative(ChassisSpeeds wantedFieldRel) {
    ModuleCommand[] commands = swerve.calculate(wantedFieldRel, getGyroAngle(), getGyroRateRadPerSec(),
        getMeasuredAngles(), getMeasuredSteerVelocities());
    for (int i = 0; i < modules.length; i++) {
        ModuleCommand c = commands[i];
        // drive: VelocityVoltage.withVelocity(speed / wheelCircumference).withAcceleration(acceleration / wheelCircumference)
        modules[i].setDrive(c.speedMetersPerSecond(), c.accelerationMetersPerSecondSquared());
        // steer: PositionVoltage.withPosition(angle).withVelocity(steerVelocity); MotionMagic: the angle only
        modules[i].setSteer(c.angle(), c.steerVelocityRadiansPerSecond());
    }
}
```

- The commands are **already optimized and cosine scaled**: don't call `optimize` or
  `cosineScale` again, and send the speed with its sign.
- Send `accelerationMetersPerSecondSquared` as the drive motor's acceleration feed-forward (Phoenix 6
  `VelocityVoltage.withAcceleration`, in rotations per second squared, or `kA · a`). It is computed
  from the plan only, never from the measured drive speed.
- `resetTo(speeds, measuredStates)` also resets the previous commands and the reversed flags from
  the measured angles. Call it in the same places as before. `onYawReset` is unchanged.
- To use only the kinematics: `DemaciaKinematics.toModuleCommands(discretizedRobotRel, measuredAngles,
  measuredSteerVelocities, previous, reversed, config, dt)` is stateless; you keep `previous` and
  the `reversed` flags (in/out: read for the tie break, set to this loop's decision).
- A NaN or infinite measured angle or steer velocity makes that module use its previous angle as
  the measured one with steer velocity 0 (no cosine scaling on garbage); a NaN gyro rate counts as
  0; arrays with the wrong length throw `IllegalArgumentException`.
- When the robot stands every module keeps its angle with speed 0, and the acceleration
  feed-forward brakes from the previous speed.

### Measure on the robot

| `ModuleCommandConfig` | How to measure it | Default |
|---|---|---|
| `latencySeconds` | Total latency from the command to the robot moving: command latency + gyro latency + dt/2. Spin the robot at a constant rate while driving a line and raise it until the path is straight (or log the gyro timestamp against the command timestamp) | 0.035 |
| `measuredAngleLatencySeconds` | Age of the measured module angle: the CAN encoder latency (Phoenix `getLatency`) plus the time to your loop. Check with a steer step: the measured angle vs the angle the motor reports | 0.035 |
| `maxSteerRateRadiansPerSecond` | Free steer speed: command a large steer step and read the peak steer velocity (with the robot on the ground) | 12 |
| `maxDriveAccelMetersPerSecondSquared` | The acceleration where the wheels still don't slip: the same as the skid limit, or the largest feed-forward the drive can follow | 20 |
| `steerVelocityFeedforward` | `true` only for a `PositionVoltage` steer; with MotionMagic leave `false` | false |

Known weak spot: a fully locked steer module makes the robot yaw, because the cosine scaling
stops that wheel and the other three drive on (heading error 34° vs 22-27° without it in the
simulation). Check on the robot that a module with a broken steer is detected elsewhere.

## Using `DemaciaKinematics` on its own

Without the limiter (e.g. robot-relative driving or a path follower), discretize first, as
WPILib does:

```java
ChassisSpeeds discrete = ChassisSpeeds.discretize(robotRelSpeeds, KinematicsConstants.CYCLE_DT);
SwerveModuleState[] states = kinematics.toSwerveModuleStates(discrete); // desaturated
setModuleStates(states);
```

Then don't set `limiterUsedLastLoop`, so the next limited command re-syncs.

Measured robot speeds:

```java
ChassisSpeeds robotRel = kinematics.toChassisSpeeds(getModuleStates(), getGyroRateRadPerSec());
// or, without a gyro rate, omega from the modules too (least squares):
ChassisSpeeds robotRelNoGyro = kinematics.toChassisSpeeds(getModuleStates());
```

## API

| Class | Method | What it does |
|---|---|---|
| `AccelLimitedSwerve` | `AccelLimitedSwerve(DemaciaKinematics)` | One per drivetrain. |
| | `calculate(wantedFieldRel, heading)` | Limit, desaturate, discretize; returns the module states. Once per loop. |
| | `AccelLimitedSwerve(kinematics, config)`, `calculate(wantedFieldRel, heading, gyroRate, measuredAngles, measuredSteerVelocities)` | Steer-aware: returns `ModuleCommand`s (angle, speed, drive acceleration FF, steer velocity FF). |
| | `resetTo(measuredFieldRel[, measuredStates])` | Start the next loop from measured speeds (and keep those module angles while standing). |
| | `onYawReset(oldYaw, newYaw)` | Re-express the stored speeds after the gyro yaw is set. |
| | `getSentSpeedsFieldRel()` / `getSentSpeedsRobotRel()` | What was sent last loop (mean velocity). |
| `DemaciaKinematics` | `DemaciaKinematics(Translation2d...)` | Module locations, robot frame, module order. |
| | `toSwerveModuleStates(robotRelSpeeds)` | Module states, desaturated to `MAX_ALLOWED_MODULE_VELOCITY`. No discretize. |
| | `toChassisSpeeds(states, omegaFromGyro)` | Robot speeds from module states, omega from the gyro. |
| | `toChassisSpeeds(states)` | Same, omega from the modules (least squares). |
| | `toModuleCommands(discretizedSpeeds, measuredAngles, measuredSteerVelocities, previous, reversed, config, dt)` | Reverse hysteresis, cosine scaling, feed-forwards. Stateless. |
| | `getModuleCount()` | Number of modules. |
| `SwerveAccelerationLimiter` | `calculate(wanted, lastCommanded, heading)` | The limiter alone (field-relative, dt from the clock). `AccelLimitedSwerve` calls it; use it directly only if you keep `lastCommanded` yourself. |

## Tuning the limits

All in `SwerveAccelerationLimiter` (m/s²) and `KinematicsConstants`. Start low and raise them.

| Constant | What it limits | How to find it |
|---|---|---|
| `MAX_SKID_ACCEL` | Total velocity change (traction) | Full stick from rest on carpet, raise until the wheels slip (logged module speed jumps above the robot speed), then back off ~20%. About μ·g; 8 is a safe start for most treads. |
| `MAX_FORWARD_ACCEL` | Speed-up at rest | Highest acceleration the drive motors give at low speed without browning out; the limit falls linearly to 0 at `FREE_SPEED`. |
| `FREE_SPEED` | Where the motors run out of torque | Free speed of the drive: motor free rpm / gear ratio · wheel circumference. |
| `MAX_TILT_ACCEL_FRONT` / `_SIDE` | Tipping forward/back and sideways | Raise until the robot lifts a wheel when stopping hard / swerving at speed, then back off. Lower with a high center of mass or an extended mechanism. |
| `MAX_ALLOWED_MODULE_VELOCITY` | Desaturation | The fastest a module can really drive (a bit under the free speed). |
| `MIN_VELOCITY` | Below this the robot stands | Just above stick noise after the controller deadband. |
| `CYCLE_DT` | Discretize time | The robot loop period (0.02 s). |

## Differences from DemaciaCode

The DemaciaCode versions had sign and math bugs (DemaciaCode issues #22–#25). Here:

- **Inverse kinematics** follows WPILib: a module at (x, y) moves at (vx − ω·y, vy + ω·x).
  DemaciaCode has `−vy + ω·r·cos` (strafe mirrored, rotation non-rigid) on some branches and
  `vy − ω·r·cos` (rotation reversed) on others.
- **Forward kinematics** is the exact least-squares inverse; DemaciaCode added a rotation term
  that used the wheel angle (wrong while rotating, and different depending on which modules are
  flipped).
- **No lead-angle term** (`ω·0.02` added to each module angle); `AccelLimitedSwerve`
  discretizes the whole command instead.
- **Desaturation** no longer skips when one module is at 0.
- **The limiter loop** (DemaciaCode keeps it in `Chassis`) is `AccelLimitedSwerve` here: the
  stored speeds are read back with the same convention as the command (no mirrored vy), a gyro
  reset re-expresses them, a re-sync keeps the module angles, and a stop holds the last angles.
- Removed: `ChassisAccel`, `toSwerveModuleStatesWithAccel` and `Limits.limitVelocities` (unused).

Copying code back to DemaciaCode means taking these fixes with it.

## Tests

```bash
cd Infrastructures
./gradlew test
```

| Test | Checks |
|---|---|
| `DemaciaKinematicsTest.moduleStatesMatchWpilib` | Module states equal WPILib's formula. |
| `forwardIsTheInverseOfInverse`, `forwardIsTheInverseForAnOffCenterLayout` | Forward kinematics undoes the inverse, also with the modules not centered. |
| `desaturatesWhenOneModuleIsAtZero` | Desaturation with a module at 0. |
| `zeroSpeedsGiveZeroModules` | Zero speeds give zero modules. |
| `AccelLimitedSwerveTest.reachesTheWantedSpeedFromRest` | Ramps up to the wanted speed. |
| `convergesWhenTheRobotIsTurned`, `convergesWhileSpinning` | Reaches the field-relative target at any heading and while spinning. |
| `yawResetDoesNotJump` | A gyro reset doesn't step the command. |
| `stoppingKeepsTheModuleAngles` | A stop holds the module angles. |
| `resetStartsFromTheMeasuredSpeeds` | `resetTo` starts from the measured speeds. |

## Checking it on the robot

Watch the commanded vs. measured speeds on the dashboard. From DemaciaCode issue #19:

- Standing still 10 s: command stays 0, no creep.
- Full stick from rest: smooth, no slip. Let go at full speed: stops without skid or tipping.
- Full forward, then full backward; same for strafing left and right: no jump.
- Full speed, snap the stick 90°: the robot curves into the new direction.
- Drive while spinning: the path is the same as without spinning (checks the discretize).
- Switch between limited and unlimited driving while moving, disable and enable while moving,
  reset the gyro while moving: no jump.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Robot jumps when a drive command starts | `resetTo` isn't called when the limiter wasn't used last loop. |
| Robot swerves after a gyro reset while driving | `onYawReset` isn't called from that reset. |
| Modules snap to 0 degrees when stopping | The robot's own module code overrides speed-0 states, or `resetTo` without module states at startup. |
| Robot drives the wrong way when turned | Gyro sign: the heading must be CCW positive. |
