# Infrastructures – Kinematics

Swerve kinematics and a velocity-setpoint acceleration limiter for a swerve robot.

The code lives in [`Infrastructures/src/main/java/frc/robot/Kinematics`](Infrastructures/src/main/java/frc/robot/Kinematics).
It is kept in sync with the kinematics and `SwerveAccelerationLimiter` in DemaciaCode, with no
`Chassis`, `Log` or `ElasticGenerator` dependencies, so it works in any project.

## Contents

| Path | What it is |
|---|---|
| `DemaciaKinematics.java` | Swerve kinematics, the same math as WPILib's `SwerveDriveKinematics`. Module states from robot speeds (desaturated), robot speeds from module states (the exact inverse). |
| `SwerveAccelerationLimiter.java` | Limits how fast the field-relative velocity setpoint changes (skid, forward and tilt limits). |
| `AccelLimitedSwerve.java` | The class a robot uses: runs the limiter and the kinematics once per loop and keeps the speeds it sent. |
| `KinematicsConstants.java` | `MAX_ALLOWED_MODULE_VELOCITY` (desaturation), `MIN_VELOCITY` (below this the robot stands), loop time. |
| `KinematicsUtilities.java`, `ChassisAccel.java` | Older helpers (`toSwerveModuleStatesWithAccel`, `Limits.limitVelocities`). Not used by `AccelLimitedSwerve`. |

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

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Robot jumps when a drive command starts | `resetTo` isn't called when the limiter wasn't used last loop. |
| Robot swerves after a gyro reset while driving | `onYawReset` isn't called from that reset. |
| Modules snap to 0 degrees when stopping | The robot's own module code overrides speed-0 states, or `resetTo` without module states at startup. |
| Robot drives the wrong way when turned | Gyro sign: the heading must be CCW positive. |
