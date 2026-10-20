# Handoff – kinematics-sync branch

Delete this file once the work below is done.

Branch `kinematics-sync` (new, pushed), based on `Kinematics` `0aa10ec`. Now at `e59c5c3`.
Source: DemaciaCode `kinematics-acceleration-limit-testing` a54f5a4 (SwerveAccelerationLimiter, kinematics); compared with
`mechanism` ed68a5c, `development`, `demacia-odometry-simple-testing`, `robot-pose-fixes`, `odometry-accelerometer-testing`.

## Done
| Commit | What | Type |
|---|---|---|
| 4476d7c | `SwerveAccelerationLimiter` from DemaciaCode | sync |
| 57a7653 | `toSwerveModuleStates` from DemaciaCode | sync |
| 673f757 | `DemaciaKinematics` matches WPILib: module i at (x, y) gets `(vx - w*y, vy + w*x)`; `toChassisSpeeds` is the exact inverse (least squares, omega from the gyro). Fixes DemaciaCode #22 / #25 | fix |
| 87b96ae | Desaturation no longer skips when one module is at 0 | fix |
| 5fb644d | `AccelLimitedSwerve`: owns the limiter loop (fixes the Chassis-side bugs from DemaciaCode #23 / #24) | fix |
| e59c5c3 | `DemaciaKinematicsTest`, `AccelLimitedSwerveTest` | tests |

`AccelLimitedSwerve` API (`Infrastructures/src/main/java/frc/robot/Kinematics/AccelLimitedSwerve.java`):
- `calculate(ChassisSpeeds wantedFieldRel, Rotation2d heading)` → module states, once per loop (overload with `dt` for tests).
- `resetTo(ChassisSpeeds measuredFieldRel[, SwerveModuleState[] measuredStates])` → re-sync when the limiter wasn't used last loop (enable, other command drove).
- `onYawReset(Rotation2d oldYaw, Rotation2d newYaw)` → call on gyro reset so the stored command doesn't jump.
- `getSentSpeedsFieldRel()` / `getSentSpeedsRobotRel()` → what was actually sent after desaturation.
- Zero command keeps the last module angles.

## Verification so far
- **No gradle build or test** (WPILib/CTRE maven blocked in that environment).
- **Compiled and ran against stand-ins:** the real `Kinematics` sources and tests were compiled with `javac` against small WPILib/JUnit stand-in classes and the tests run by a small runner: 11 passed, 0 failed (moduleStatesMatchWpilib, forwardIsTheInverseOfInverse, forwardIsTheInverseForAnOffCenterLayout, desaturatesWhenOneModuleIsAtZero, zeroSpeedsGiveZeroModules, reachesTheWantedSpeedFromRest, convergesWhenTheRobotIsTurned, convergesWhileSpinning, stoppingKeepsTheModuleAngles, yawResetDoesNotJump, resetStartsFromTheMeasuredSpeeds). The stand-ins are not WPILib, so this does not prove it compiles against WPILib.
- **Cross-checked with real WPILib math:** `robotpy-wpimath` 2026.2.2 (`SwerveDriveKinematics`, `ChassisSpeeds`) vs the Java output: worst difference 1.8e-15 for module states and forward kinematics; `fromFieldRelativeSpeeds` / `fromRobotRelativeSpeeds` match.

## Left
0. `./gradlew build test` locally; fix anything the stand-ins hid (method names/signatures of `ChassisSpeeds`, `SwerveModuleState`, `Rotation2d`, `Translation2d`, `MathUtil`).
1. The sync and the fixes are separate commits already, but 57a7653 changes `DemaciaKinematics` and 673f757 rewrites it; if a cleaner history is wanted, squash 57a7653 into the sync or leave as is.
2. The old `0.02*w` lead-angle term was removed in 673f757 (documented in the `DemaciaKinematics` class javadoc). If the robot needs it, add it as a rotation of the whole command (e.g. `ChassisSpeeds.discretize`), not per module.
3. Using it from a robot: the Chassis calls `calculate` once per loop instead of its own limiter path, calls `resetTo` from periodic when the limiter wasn't used last loop, and `onYawReset` from its gyro reset; send the states with the robot's own hold-angle/optimize logic.
4. `ChassisAccel` (old Infrastructures code) was kept; check whether it's still needed now that `SwerveAccelerationLimiter` exists.
5. Robot test plan: DemaciaCode issue #19 (strafe, turning at speed, gyro reset while moving, switching drives).

## Risks
- Only compiled against stand-ins, never against WPILib.
- Limiter constants are placeholders (see `KinematicsConstants`).
