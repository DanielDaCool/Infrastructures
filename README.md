# Kinematics

[![WPILib](https://img.shields.io/badge/WPILib-2026.2.1-blue)](https://docs.wpilib.org/)
[![Java](https://img.shields.io/badge/Java-17-orange)](https://adoptium.net/)
[![Vendor libraries](https://img.shields.io/badge/vendor%20libraries-none-lightgrey)](#requirements)

Swerve kinematics for FRC robots. It converts the requested chassis speeds (forward,
sideways, turn) into a state for each swerve module, and keeps the robot on its path when
it drives and spins at the same time. It is a drop-in replacement for DemaciaCode's
`DemaciaKinematics`: existing chassis code keeps working unchanged.

> [!NOTE]
> This README is the student guide. The full design, the math, simulation results and every
> setting are documented in [docs/design.md](docs/design.md).

## Contents

- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Getting started](#getting-started)
- [Conventions](#conventions)
- [Settings](#settings)
- [Using it with an acceleration limiter](#using-it-with-an-acceleration-limiter)
- [Building](#building)
- [Troubleshooting](#troubleshooting)
- [Project structure](#project-structure)

## Features

- **Drop-in replacement.** Same constructor and methods as DemaciaCode's
  `DemaciaKinematics`; adopting it is a one-line import change.
- **Discretization over the loop and the latency.** Removes the sideways drift a swerve
  robot gets while driving and spinning together.
- **Cosine scaling against measured module angles.** A wheel that still points the wrong
  way gets less speed, so it does not push the robot off the path.
- **Drive and steer feed-forwards.** Per-module acceleration and steer-velocity values so the
  motors do not lag behind the command.
- **Ratio-preserving desaturation.** When a wheel is over its top speed, driving and turning
  slow down together.
- **Optional acceleration limiter.** `SwerveAccelerationLimiter` limits how fast the
  field-relative velocity setpoint changes (skid, forward and tilt limits). A few lines in your
  own drive method.

Average distance from the path in simulation:

| Configuration | Off the path |
|---|---|
| Old kinematics | 36 cm |
| Level 1 | 24 cm |
| Level 2 + drive feed-forward | 15 cm |
| Level 3 (both feed-forwards) | 12 cm |

## Requirements

| Dependency | Version |
|---|---|
| [WPILib / GradleRIO](https://docs.wpilib.org/) | 2026.2.1 |
| Java | 17 |
| Vendor libraries | None |

## Installation

1. Copy [`Infrastructures/src/main/java/frc/robot/Kinematics`](Infrastructures/src/main/java/frc/robot/Kinematics)
   into your robot project, keeping the same package path.
2. Follow [Getting started](#getting-started).

## Getting started

Adoption is split into three levels. Each level works on its own, and each one makes the
robot follow its path more closely.

### Level 1: change the import

```java
// import frc.demacia.kinematics.DemaciaKinematics;
import frc.robot.Kinematics.DemaciaKinematics;
```

The DemaciaCode chassis already creates it once (`new DemaciaKinematics(modulePositions)` in
the `Chassis` constructor). If you create it yourself, do it once in `Robot`'s constructor.
That constructor uses the default settings; to set your robot's values, pass a
`KinematicsConfig` (see [Settings](#settings)).

### Level 2: pass the measured module states

In `Chassis.setVelocities`, pass the measured module states:

```java
SwerveModuleState[] states = demaciaKinematics.toSwerveModuleStates(speeds, getModuleStates());
```

A wheel that still points the wrong way (just starting, or the stick moved faster than the
wheel can turn) now gets less speed, so it does not push the robot off the path.

### Level 3: add the feed-forwards

The feed-forwards tell the motors in advance how the speed and the angle are about to
change, so they do not lag. Level 3 requires level 2.

**1. In the module code (`SwerveModule.java`), not in the chassis,** add a second `setState`
that takes two more values. It is a copy of the module's `setState` without the reverse, with
the feed-forwards added (written for the module on the `mechanism` branch). It must live in
`SwerveModule` because it uses the module's own fields and methods (`isDriveStop`,
`steerMotor`, `getSteerVel()`):

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

**2. In the chassis code (`Chassis.setVelocities`),** call it for each module:

```java
SwerveModuleState[] states = demaciaKinematics.toSwerveModuleStates(speeds, getModuleStates());
double[] accelerations = demaciaKinematics.getDriveAccelerations();
double[] steerVelocities = demaciaKinematics.getSteerVelocities();
for (int i = 0; i < states.length; i++) {
    modules[i].setState(states[i], accelerations[i], steerVelocities[i]);
}
```

`STEER_KV` is the steer motor's kV (volts per rad/s). With a MotionMagic steer, leave the
steer feed-forward out.

## Conventions

> [!IMPORTANT]
> - Call `toSwerveModuleStates` **once per loop**. It remembers the previous loop.
> - The turning speed passed to `toSwerveModuleStates` is **clockwise-positive**, the same as
>   the old DemaciaCode kinematics, so nothing in the chassis changes. Everything else
>   (module locations, `SwerveAccelerationLimiter`) uses the WPILib frame: x forward, y left,
>   counter-clockwise positive.

## Settings

All settings are in a `KinematicsConfig`, one per drivetrain, like `ChassisConfig` in
DemaciaCode. Every setting has a default (placeholders: tune them on your robot); set only the
ones that differ and pass the same config to the kinematics and the limiter:

```java
KinematicsConfig kinematicsConfig = new KinematicsConfig()
    .withMaxAllowedModuleVelocity(4.2)
    .withLatency(0.03);
DemaciaKinematics demaciaKinematics = new DemaciaKinematics(kinematicsConfig, modulePositions);
SwerveAccelerationLimiter limiter = new SwerveAccelerationLimiter(kinematicsConfig);
```

The values are copied when the kinematics and the limiter are created, so finish the config
first. The limiter's settings are in [docs/design.md](docs/design.md#tuning-the-limits).

| Setting | Description | How to find it | Default |
|---|---|---|---|
| `maxAllowedModuleVelocity` | Top speed of a wheel | Drive at full stick on the floor, log the wheel speed, and use about 10% less than the highest value | 4.5 m/s |
| `latency` | Command latency + gyro latency (no half loop) | Drive straight while spinning, and raise it until the path is straight | 0.035 s (placeholder) |
| `maxDriveAccel` | Safety clamp of the drive feed-forward | Above the traction limit (about μg); with the acceleration limiter it never engages | 20 m/s² |
| `maxSteerRate` | Limit of the steer feed-forward | The fastest the steer can turn | 12 rad/s |

## Using it with an acceleration limiter

Optional. Use `SwerveAccelerationLimiter` in your own field-relative drive method. Keep the
field-relative speeds really sent last loop (`lastFieldSpeeds`, from `getSentSpeeds()`) in the chassis:

```java
// desaturate first, so driving and turning slow down together at top speed
wanted = ChassisSpeeds.fromRobotRelativeSpeeds(
    demaciaKinematics.desaturate(ChassisSpeeds.fromFieldRelativeSpeeds(wanted, heading)), heading);
// send ChassisSpeeds.fromFieldRelativeSpeeds(limiter.calculate(wanted, lastFieldSpeeds, heading), heading)
// with your drive method, then start the next loop from what was really sent (after cosine scaling)
lastFieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(demaciaKinematics.getSentSpeeds(), heading);
```

Keep the desaturate line **before** the limiter, also with a limiter of your own. Without it,
the robot keeps turning fast but drives much slower.

Also set `lastFieldSpeeds` to the measured speeds when something else drove the robot last
loop, and turn it on a gyro reset. The example chassis
[`ExampleChassis.java`](Infrastructures/src/main/java/frc/robot/example/ExampleChassis.java)
shows all of it; see also
[docs/design.md](docs/design.md#adding-the-acceleration-limit-to-your-chassis).

## Building

From the `Infrastructures` directory:

```bash
./gradlew build
```

Simulation results do not replace testing on the robot. Check the settings above and the
module directions on real hardware; see
[docs/design.md](docs/design.md#checking-it-on-the-robot).

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Robot turns the wrong way after the import change | Your chassis used a different version of the old kinematics; see [Differences from DemaciaCode](docs/design.md#differences-from-demaciacode). |
| Drives slower than expected while turning | `maxAllowedModuleVelocity` is higher than the wheels can really reach, or a limiter is used without the line above. |
| Curves when driving and spinning | `latency` is too small (or too big). |
| Wheels snap to 0° when stopping | The module code overrides speed-0 states. |
| Wheel jumps or shakes with level 3 | Level 3 is used without level 2, or `STEER_KV` is too big. |

## Project structure

All paths are relative to `Infrastructures/src/main/java/frc/robot/Kinematics`.

| Path | Description |
|---|---|
| `DemaciaKinematics.java` | Swerve kinematics. Drop-in for DemaciaCode's `DemaciaKinematics`, with discretization, cosine scaling and feed-forwards. |
| `SwerveAccelerationLimiter.java` | Limits how fast the field-relative velocity setpoint changes (skid, forward and tilt limits). |
| `KinematicsConfig.java` | Settings for the kinematics and the limiter (module speed limit, standing threshold, loop time, latency, feed-forward limits, acceleration limits). |
| `KinematicsUtilities.java` | Small vector helpers (`limitVector`, `isInRange`, ...). |

The example chassis is outside the library, in
`Infrastructures/src/main/java/frc/robot/example/ExampleChassis.java`: robot-relative and
field-relative drive with the acceleration limit, re-sync and gyro reset, hardware left to your
own code. It is not used by the robot.
