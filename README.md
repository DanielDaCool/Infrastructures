# Kinematics

Turns the speeds you want (forward, sideways, turn) into what each swerve module should do. It
replaces DemaciaCode's `DemaciaKinematics`: your chassis keeps working, and the robot stays on
its path better when it drives and spins at the same time.

> Mentors and developers: how it works, why, test results and every setting are in
> [docs/design.md](docs/design.md).

## Install

1. Copy [`Infrastructures/src/main/java/frc/robot/Kinematics`](Infrastructures/src/main/java/frc/robot/Kinematics)
   into your robot project (same package path).
2. Made for WPILib 2026. No vendor libraries needed.

## Getting started

There are three levels. Each one works on its own; each next one makes the robot follow its path
better.

### Level 1 – change the import (nothing else)

```java
// import frc.demacia.kinematics.DemaciaKinematics;
import frc.robot.Kinematics.DemaciaKinematics;
```

The chassis already creates it once (`new DemaciaKinematics(modulePositions)` in the `Chassis`
constructor). If you create it yourself, do it once in `Robot`'s constructor.

### Level 2 – tell it where the wheels point (one line)

In `Chassis.setVelocities`, pass the measured module states:

```java
SwerveModuleState[] states = demaciaKinematics.toSwerveModuleStates(speeds, getModuleStates());
```

A wheel that still points the wrong way (just starting, or the stick moved faster than the
wheel can turn) now gets less speed, so it doesn't push the robot off the path.

### Level 3 – feed-forwards (a few lines)

Tells the motors in advance how the speed and the angle are about to change, so they don't lag.
**1. In the module code (`SwerveModule.java`), not in the chassis:** add a second `setState` with
two more values. It is a copy of the module's `setState` without the reverse, with the
feed-forwards added (written for the module on `mechanism`). It has to be inside `SwerveModule`
because it uses the module's own fields and methods (`isDriveStop`, `steerMotor`, `getSteerVel()`):

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

**2. In the chassis code (`Chassis.setVelocities`):** call it for each module:

```java
SwerveModuleState[] states = demaciaKinematics.toSwerveModuleStates(speeds, getModuleStates());
double[] accelerations = demaciaKinematics.getDriveAccelerations();
double[] steerVelocities = demaciaKinematics.getSteerVelocities();
for (int i = 0; i < states.length; i++) {
    modules[i].setState(states[i], accelerations[i], steerVelocities[i]);
}
```

`STEER_KV` is the steer motor's kV (volts per rad/s). With a MotionMagic steer, leave the steer
feed-forward out. Use level 3 only together with level 2.

How much each level helps (simulation, average distance from the path):

| | Off the path |
|---|---|
| Old kinematics | 36 cm |
| Level 1 | 24 cm |
| Level 2 + drive feed-forward | 15 cm |
| Level 3 (both feed-forwards) | 12 cm |

### Rules

- Call `toSwerveModuleStates` **once per loop** (it remembers the last loop).
- Turning speed is **clockwise-positive** here, the same as the old DemaciaCode kinematics, so
  nothing in the chassis changes.

## Settings

In `KinematicsConstants`. Set them on your robot:

| Setting | What it is | How to find it | Default |
|---|---|---|---|
| `MAX_ALLOWED_MODULE_VELOCITY` | Top speed of a wheel | Drive full stick on the floor, log the wheel speed, use ~10 % less than the highest | 4.5 m/s |
| `LATENCY` | Delay from the command to the wheels | Drive straight while spinning; raise it until the path is straight | 0.035 s |
| `MAX_DRIVE_ACCEL` | Limit of the drive feed-forward | Where the wheels start to slip | 20 m/s² |
| `MAX_STEER_RATE` | Limit of the steer feed-forward | Fastest the steer turns | 12 rad/s |

## Using it with an acceleration limiter

If your chassis has its own acceleration limiter, add this line **before** the limiter, so
driving and turning slow down together when the wheels are at top speed (without it the robot
keeps turning fast but drives much slower):

```java
fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
    demaciaKinematics.desaturate(ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, heading)), heading);
```

`AccelLimitedSwerve` (in this package) already does it; see [docs/design.md](docs/design.md).

## Troubleshooting

| Problem | Try |
|---|---|
| Robot turns the wrong way after the import change | Your chassis used a different version of the old kinematics; see [docs/design.md](docs/design.md#differences-from-demaciacode). |
| Drives slower than expected while turning | `MAX_ALLOWED_MODULE_VELOCITY` is higher than the wheels can really do, or a limiter without the line above. |
| Curves when driving and spinning | `LATENCY` too small (or too big). |
| Wheels snap to 0 when stopping | The module code overrides speed-0 states. |
| Wheel jumps or shakes with level 3 | Level 3 used without level 2, or `STEER_KV` too big. |
