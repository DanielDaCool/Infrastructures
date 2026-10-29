# Kinematics – design and details (for mentors and developers)

The student guide is the [README](../README.md). This page has everything behind it: how the
kinematics and the limiter work, what each fix does with the simulation numbers, the full API,
tuning and the differences from DemaciaCode.

Swerve kinematics and a velocity-setpoint acceleration limiter for a swerve robot.

**DemaciaCode chassis: change one import** (see [Quick start](#quick-start-the-demaciacode-chassis)).

The code lives in [`Infrastructures/src/main/java/frc/robot/Kinematics`](../Infrastructures/src/main/java/frc/robot/Kinematics).
It started from the kinematics and `SwerveAccelerationLimiter` in DemaciaCode
(`kinematics-acceleration-limit-testing` a54f5a4), with no `Chassis`, `Log` or
`ElasticGenerator` dependencies, so it works in any project. The kinematics math was then fixed
here and no longer matches DemaciaCode (see [Differences from DemaciaCode](#differences-from-demaciacode)).

## Contents

| Path | What it is |
|---|---|
| `DemaciaKinematics.java` | Swerve kinematics. A drop-in for DemaciaCode's `DemaciaKinematics` (same constructor and methods) that discretizes over the loop and the latency, keeps the angles when standing and, with one added argument, cosine scales against the measured module angles. Also the plain WPILib inverse (`toWpilibModuleStates`) and robot speeds from module states. |
| `SwerveAccelerationLimiter.java` | Limits how fast the field-relative velocity setpoint changes (skid, forward and tilt limits). |
| `KinematicsConstants.java` | `MAX_ALLOWED_MODULE_VELOCITY` (desaturation), `MIN_VELOCITY` (below this the robot stands), loop time, `LATENCY`, `MAX_DRIVE_ACCEL`. |
| `KinematicsUtilities.java` | Small vector helpers (`limitVector`, `isInRange`, ...). |
| `../example/ExampleChassis.java` | Example only (outside the library, not used by the robot): a chassis with the kinematics and the optional acceleration limit. |

Every module location is in the **WPILib robot frame**: x forward, y left, rotations CCW
positive, meters from the robot center. Speeds are m/s and rad/s, CCW positive. **One
exception:** `DemaciaKinematics.toSwerveModuleStates` takes omega **clockwise-positive**, because
that is what the DemaciaCode chassis sends (its old kinematics computed `vy − ω·x`, and the chassis
negates the trajectory's omega to match).

The limits in `SwerveAccelerationLimiter` and `KinematicsConstants` are placeholders. Tune
them on the robot.

## Quick start: the DemaciaCode chassis

`Chassis` keeps working as it is. Change the import:

```java
// import frc.demacia.kinematics.DemaciaKinematics;
import frc.robot.Kinematics.DemaciaKinematics;
```

That's all that's needed. `setVelocities` (and everything that goes through it: the drive command,
`followTrajectory`) now gets module states that are discretized over the loop **plus the
latency**, so a robot that drives and spins stays on its path, and a robot that stops keeps its
module angles instead of turning them to 0.

**Recommended, one argument:** let the kinematics see where the modules really point. In
`Chassis.setVelocities`:

```java
SwerveModuleState[] states = demaciaKinematics.toSwerveModuleStates(speeds, getModuleStates());
```

Each module is then turned to the side closer to its measured angle and its speed is scaled by
the cosine of the angle it still has to turn, so a module that points the wrong way (start from
rest, a stick snapped faster than the steer) doesn't push the robot off the path.
`SwerveModule.setState` stays as it is (it won't reverse the state again, the state is already
within 90° of the module).

**Recommended, the feed-forwards (a few lines in `SwerveModule` and `Chassis`):** after the
two-argument call, `getDriveAccelerations()` [m/s²] and `getSteerVelocities()` [rad/s] give each
module's planned drive acceleration and steer velocity. The motor classes already take them
(`setVelocityWithAcceleration` uses the drive's kA, `setPositionVoltage(position, feedForward)`
takes volts). In the **module code** (`SwerveModule.java`, not the chassis: it uses the module's own
fields), a copy of `setState` without the reverse, with two more arguments:

```java
public void setState(SwerveModuleState state, double acceleration, double steerVelocity) {
    // no reverse here: the two-argument toSwerveModuleStates already decided it
    double diff = MathUtil.angleModulus(state.angle.getRadians() - steerMotor.getCurrentPosition());
    if (Math.abs(diff) <= Math.toRadians(0.7)) {
        setSteerPower(0);
    } else {
        steerMotor.setPositionVoltage(steerMotor.getCurrentPosition() + diff, STEER_KV * steerVelocity);
    }
    if (state.speedMetersPerSecond == 0) {
        setDrivePower(0);
    } else {
        isDriveStop = false;
        // like setDriveVelocity (steer coupling correction), plus the acceleration feed-forward
        driveMotor.setVelocityWithAcceleration(
            state.speedMetersPerSecond - config.steerVelToDriveVel * getSteerVel(), () -> acceleration);
    }
}
```

(`STEER_KV`: the steer's kV in volts per rad/s; leave the steer feed-forward out with a MotionMagic
steer.) In the **chassis code** (`Chassis.setVelocities`):

```java
SwerveModuleState[] states = demaciaKinematics.toSwerveModuleStates(speeds, getModuleStates());
double[] accelerations = demaciaKinematics.getDriveAccelerations();
double[] steerVelocities = demaciaKinematics.getSteerVelocities();
for (int i = 0; i < states.length; i++) {
    modules[i].setState(states[i], accelerations[i], steerVelocities[i]);
}
```

Use the feed-forwards only with the two-argument call: if the module reversed the state again,
their signs would be wrong.

How much each step gives (simulation, not a robot: mean path error over the core scenarios, both
steer types; heading disturbance in degrees):

| What the chassis does | Path error | Heading |
|---|---|---|
| Old kinematics, WPILib style | 36.3 cm | 8.0° |
| Import only | 23.8 cm | 8.2° |
| + measured states + drive feed-forward | 14.8 cm | 8.0° |
| + steer feed-forward (`PositionVoltage` steer, as in DemaciaCode) | 11.6 cm | 7.0° |

With a `PositionVoltage` steer the steer feed-forward is worth as much as the drive feed-forward
(path error 18.0 → 11.5 cm); with MotionMagic it does nothing. The latency compensation uses the
commanded omega, not the gyro; in the simulation that is as good as the gyro rate (14.8 vs 15.3 cm).

Set on the robot (`KinematicsConstants`):

| Constant | What | How to measure it | Default |
|---|---|---|---|
| `LATENCY` | Command latency + gyro latency (no half loop: the discretize over `CYCLE_DT + 2·LATENCY` already adds it) | Drive a straight line while spinning at a constant rate and raise it until the path is straight | 0.035 s (placeholder, tune it) |
| `MAX_ALLOWED_MODULE_VELOCITY` | Desaturation | The fastest a module really drives (DemaciaCode has 4) | 4.5 m/s |
| `MAX_DRIVE_ACCEL` | Clamp of the acceleration feed-forward | The acceleration where the wheels still don't slip | 20 m/s² |
| `MAX_STEER_RATE` | Clamp of the steer velocity feed-forward | Free steer speed: a large steer step, peak steer velocity | 12 rad/s |

Check on the robot after the import change: the robot spins **the same way** as before for the
same stick/trajectory (the omega sign is the one the old kinematics used on `main`; the
`development`/`Quest` branches had a different, non-rigid formula, see
[Differences from DemaciaCode](#differences-from-demaciacode)).

Call one of the `toSwerveModuleStates` methods **once per loop**: they keep the last angles and
speeds for the standing hold and the feed-forward.

### Driving slower than it should while spinning (acceleration limiter)

When the robot drives and spins fast, a module would need `v + ω·r`, more than it can do, so all
modules are cut by one common factor (`desaturate`): driving and spinning slow down **together**
and the path is kept. A chassis with its own acceleration limiter (e.g. `Chassis` on
`kinematics-acceleration-limit-testing`) breaks this: the limiter ramps from the speeds sent last
loop, which the desaturation already cut, and only adds a small step per loop, while omega isn't
limited. Every loop the desaturation takes back more translation than the limiter adds, so the
robot spins at almost full speed and drives much slower (simulation, modules at (±0.32, ±0.27) m and
`MAX_ALLOWED_MODULE_VELOCITY` 4.0: 5 m/s + 4 rad/s wanted gives 2.5 m/s and 3.8 rad/s instead of 3.0 m/s
and 2.4 rad/s). With the current 4.5 m/s and modules at (±0.275, ±0.275) m the kept ratio gives
3.47 m/s and 2.77 rad/s (simulation).

The fix is to desaturate the wanted speeds **before** the limiter, one line before
`accelLimiter.calculate(...)`:

```java
fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
    demaciaKinematics.desaturate(ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, heading)), heading);
```

`ExampleChassis` does this. The DemaciaCode chassis on `main` has no limiter, so it doesn't
need it.

## Adding the acceleration limit to your chassis

Not needed for the DemaciaCode chassis.

There is no wrapper class: the chassis uses `SwerveAccelerationLimiter` directly, in its own
field-relative drive method, and keeps the field-relative speeds really sent last loop
(`lastFieldSpeeds`, from `DemaciaKinematics.getSentSpeeds()`: after cosine scaling, with the
discretize undone). Starting from the limiter's own output instead, the setpoint keeps ramping
while cosine scaling holds a misaligned wheel back, and the wheel jumps to it when the module
aligns (above the skid limit in simulation). The full example is
[`ExampleChassis.java`](../Infrastructures/src/main/java/frc/robot/example/ExampleChassis.java).

```java
private final SwerveAccelerationLimiter limiter = new SwerveAccelerationLimiter(); // one per drivetrain
private ChassisSpeeds lastFieldSpeeds = new ChassisSpeeds();

public void driveFieldRelative(ChassisSpeeds wanted) {
    Rotation2d heading = getGyroAngle();
    wanted = ChassisSpeeds.fromRobotRelativeSpeeds(
        kinematics.desaturate(ChassisSpeeds.fromFieldRelativeSpeeds(wanted, heading)), heading);
    drive(ChassisSpeeds.fromFieldRelativeSpeeds(limiter.calculate(wanted, lastFieldSpeeds, heading), heading));
    lastFieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(kinematics.getSentSpeeds(), heading);
}
```

Four things have to be right:

1. **`calculate` once per loop**, with dt from the clock. Two callers in the same loop (e.g.
   two commands) fight: the second one gets almost no time to change anything.
2. **Desaturate before the limiter** (the first line), see
   [above](#driving-slower-than-it-should-while-spinning-acceleration-limiter).
3. **Re-sync when something else drove the robot last loop** (just enabled, robot-relative
   driving, a path follower): set `lastFieldSpeeds` to the measured field-relative speeds,
   `fromRobotRelativeSpeeds(kinematics.toChassisSpeeds(getModuleStates(), gyroRate), heading)`.
   Otherwise the limiter starts from what it sent the last time it ran. `ExampleChassis` does it
   in `periodic`, which runs before the commands.
4. **Gyro reset:** turn the translation of `lastFieldSpeeds` by `newYaw.minus(oldYaw)`. The
   speeds are field-relative; without it the same motion is suddenly seen from another angle and
   the limiter turns the robot toward the old field direction. Do it in every place that writes
   the gyro yaw.

The limiter passes omega through unchanged, so the drive method flips it to clockwise-positive
for `toSwerveModuleStates` as usual. Speeds measured from module states are the module vectors,
so while the robot turns their direction is about ω·dt/2 off from the mean velocity. That is
small and the limiter catches up in the next loops.

## How it works

Each `SwerveAccelerationLimiter.calculate` moves the last sent field-relative velocity toward
the wanted one by the closest change that every limit allows in the time since the last call
(at most `MAX_DT`):

- skid: total change ≤ `MAX_SKID_ACCEL · dt`;
- forward: the speed can grow by at most `MAX_FORWARD_ACCEL · (1 - speed / FREE_SPEED) · dt`
  (braking and turning are free here);
- tilt: change along the robot's front ≤ `MAX_TILT_ACCEL_FRONT · dt`, along its side
  ≤ `MAX_TILT_ACCEL_SIDE · dt`.

Omega is passed through unchanged. With a NaN or infinite heading the tilt axes are unknown, so
the smaller tilt limit is used on both.

The speeds are field-relative on purpose: kept robot-relative, a robot turning while driving
would see its last speeds turn with it every loop, and the limiter could not keep up.

## What the chassis methods fix

| Step | What it fixes |
|---|---|
| Discretize over `CYCLE_DT + 2·LATENCY` (commanded omega) | The robot turns during the loop and the command arrives late; without it a robot that drives and spins curves off its path |
| Reverse and cosine scale against the measured angle (two-argument call) | A module that still points the wrong way gets ~0 speed instead of scrubbing |
| Drive acceleration feed-forward (`getDriveAccelerations`) | The drive velocity loop lags the setpoint |
| Steer velocity feed-forward (`getSteerVelocities`, `PositionVoltage` steer) | A position-only steer lags a moving angle |
| Hold the angles when standing | Modules don't snap to 0 when the robot stops. Before the first command the measured angles are held (with the measured states); the one-argument call starts at 0, like WPILib |

The numbers are in [Quick start](#quick-start-the-demaciacode-chassis); they come from a simulation, not a robot. Tune on the robot. Also weaker than
the full model: the heading while spinning fast (the reverse decision is against the measured angle, with no hysteresis). Known weak spot: a module
with a locked steer makes the robot yaw (cosine scaling stops that wheel, the other three drive on).

## Using `DemaciaKinematics` on its own

Without the limiter, use `toSwerveModuleStates` (above). For the plain WPILib inverse (omega
counter-clockwise, no discretize, no state), discretize first, as WPILib does:

```java
ChassisSpeeds discrete = ChassisSpeeds.discretize(robotRelSpeeds, KinematicsConstants.CYCLE_DT);
SwerveModuleState[] states = kinematics.toWpilibModuleStates(discrete); // desaturated
setModuleStates(states);
```


Measured robot speeds:

```java
ChassisSpeeds robotRel = kinematics.toChassisSpeeds(getModuleStates(), getGyroRateRadPerSec());
// or, without a gyro rate, omega from the modules too (least squares):
ChassisSpeeds robotRelNoGyro = kinematics.toChassisSpeeds(getModuleStates());
```

## API

| Class | Method | What it does |
|---|---|---|
| `DemaciaKinematics` | `DemaciaKinematics(Translation2d...)` | Module locations, robot frame, module order. |
| | `toSwerveModuleStates(robotRelSpeeds)` | For the chassis (omega clockwise-positive): desaturate, discretize over `CYCLE_DT + 2·LATENCY`, inverse, desaturate; keeps the angles when standing. Once per loop. |
| | `toSwerveModuleStates(robotRelSpeeds, measuredStates)` | Same, then reverse against the measured angle and cosine scale. |
| | `getDriveAccelerations()` | Drive acceleration feed-forward of the last states [m/s²], clamped to `MAX_DRIVE_ACCEL`. |
| | `getSteerVelocities()` | Steer velocity feed-forward of the last states [rad/s], clamped to `MAX_STEER_RATE`. |
| | `toWpilibModuleStates(robotRelSpeeds)` | Plain WPILib inverse (omega CCW), desaturated to `MAX_ALLOWED_MODULE_VELOCITY`. No discretize, no state. |
| | `toChassisSpeeds(states, omegaFromGyro)` | Robot speeds from module states, omega from the gyro. |
| | `toChassisSpeeds(states)` | Same, omega from the modules (least squares). |
| | `desaturate(robotRelSpeeds)` | Robot speeds cut by one common factor so no module is faster than `MAX_ALLOWED_MODULE_VELOCITY` (translation and rotation together). |
| | `getModuleCount()` | Number of modules. |
| `SwerveAccelerationLimiter` | `calculate(wanted, lastCommanded, heading)` | Next field-relative setpoint, dt from the clock. Once per loop; the chassis keeps `lastCommanded` (see [Adding the acceleration limit](#adding-the-acceleration-limit-to-your-chassis)). |

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
| `LATENCY` | Added to the discretize time (twice) in `toSwerveModuleStates` | See [Quick start](#quick-start-the-demaciacode-chassis). |
| `MAX_DRIVE_ACCEL` | Clamp of `getDriveAccelerations` | The acceleration where the wheels still don't slip. |

## Differences from DemaciaCode

The DemaciaCode versions had sign and math bugs (DemaciaCode issues #22–#25). Here:

- **Inverse kinematics** is WPILib's: a module at (x, y) moves at (vx − ω·y, vy + ω·x).
  `toSwerveModuleStates` takes ω clockwise-positive at its input, which gives the same signs as `main`'s
  `vy − ω·r·cos` (rotation reversed), so the chassis on `main` keeps spinning the same way.
  `development`, `Quest`, `mechanism`, `tashtiot` and `kinematics-acceleration-limit-testing`
  have `−vy + ω·r·cos` (strafe mirrored, rotation non-rigid); a chassis tuned on those strafes
  the other way here.
- **Forward kinematics** is the exact least-squares inverse; DemaciaCode added a rotation term
  that used the wheel angle (wrong while rotating, and different depending on which modules are
  flipped).
- **No lead-angle term** (`ω·0.02` added to each module angle); the whole command is
  discretized instead (over the loop plus the latency in `toSwerveModuleStates`).
- **Desaturation** no longer skips when one module is at 0.
- Removed: `ChassisAccel`, `toSwerveModuleStatesWithAccel` and `Limits.limitVelocities` (unused).

Copying code back to DemaciaCode means taking these fixes with it.

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
| Robot jumps when a drive command starts | `lastFieldSpeeds` isn't re-synced to the measured speeds when the limiter wasn't used last loop. |
| Robot swerves after a gyro reset while driving | `lastFieldSpeeds` isn't turned by the yaw change in that reset. |
| Modules snap to 0 degrees when stopping | The robot's own module code overrides speed-0 states. |
| Robot drives the wrong way when turned | Gyro sign: the heading must be CCW positive. |
