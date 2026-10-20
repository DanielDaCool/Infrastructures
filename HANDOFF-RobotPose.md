# Handoff – RobotPose branch

Delete this file once the work below is done.

Branch `RobotPose`. Before this work it was at `77387ed`; now at `fda8d6e` (pushed, fast-forward).
Sources: DemaciaCode branches `mechanism` ed68a5c, `development` fb4599a, `kinematics-acceleration-limit-testing` a54f5a4,
`demacia-odometry-simple-testing` 8323d7b, `robot-pose-fixes` 515a24e, `odometry-accelerometer-testing` f536d31.

## Done (sync only, no new fixes)
Rule used: per file, the newest correct version across the six DemaciaCode branches; Infrastructures-only parts kept
(`PhoenixOdometryThread` 250 Hz odometry, `DemaciaPoseEstimatorTest`, README, standalone design without `Chassis`/`Log`/`ElasticGenerator`).

| Commit | What | From |
|---|---|---|
| 72ca10b | `LimelightTagCamera2d`: assigns the `timestampSeconds` field (was a local that shadowed it, so every 2D measurement got timestamp 0 and was dropped); heading taken at the frame's capture time and used as the pose heading instead of the raw gyro | odometry-accelerometer-testing |
| 3fe16f8 | `LimelightTagCamera3d`: comments/heartbeat docs | odometry-accelerometer-testing |
| ead33bb | `Quest`: low battery alert | mechanism |
| 62c1762 | `DemaciaOdometry` / `DemaciaPoseEstimator`: gyro offset reset | odometry-accelerometer-testing |
| 838b411 | Collision detection (`RoboRioOrientation`, estimator drops wheel motion into a hit) + test. **Opt-in**: off unless `RobotPose.enableCollisionDetection(orientation, roboRioPosition, yawRateRadiansPerSecond)` is called, because its mounting constants are placeholders on that branch. Math unchanged | odometry-accelerometer-testing |
| fda8d6e | README: sync sources and the new parts | — |

Not imported on purpose: `mechanism`/`demacia-odometry-simple-testing` send `-offset.getY()` to the 3D Limelight (half a mirror, DemaciaCode #31). This branch still sends `offset.getY()` (`LimelightTagCamera3d.java:49-55`). It is replaced by item 1 below.

## Verification so far
- No gradle build or test was possible (frcmaven.wpi.edu / maven.ctr-electronics.com blocked in that environment). Nothing on this branch was compiled.
- Everything above was checked by reading only. Treat it as uncompiled code.

## Left (in order)
0. `./gradlew build test` and fix anything that doesn't compile. Watch: imports/signatures in `RobotPose.java` (838b411 is the biggest change), `DemaciaPoseEstimatorTest` (new collision tests), `Quest` battery alert (WPILib `Alert`).
1. **Source frame (DemaciaCode #31).** Every config `offset` is WPILib robot frame (x fwd, y left, z up, CCW+). In `Vision/visionConfigs/BaseVisionSourceConfig.java` add
   `protected Matrix<N3,N3> sourceFrame()` (default identity) and `public final Transform3d getSourceOffset()` = `p' = C·p`, `R' = C·R·Cᵀ` (`new Rotation3d(Matrix)`).
   `LimelightTagCamera3dConfig` overrides `sourceFrame()` with `diag(1,-1,1)`. `LimelightTagCamera3d` sends `getSourceOffset()` with no manual sign flips. `QuestConfig` keeps the default.
   Document the contract in the config javadoc and README. Log the 6 values sent (`vision/<name>/source pose sent`).
2. **Quest vs 3D Limelight check.** When both have a fresh measurement in the same loop, log the distance; WPILib `Alert` if > ~0.3 m for > 1 s.
3. **2D Limelight timing (DemaciaCode #28).** Read `t2d` once with `getAtomic()`; timestamp = `atomic.timestamp / 1e6 - (tl + cl) / 1000`; take tx/ty/tid from the same array (indices per `LimelightHelpers.getT2DArray`; check the repo copy). Currently `now - latency` and separate NT reads (tid is read at the validity check and again in `getTag()`).
4. **resetPose heading (DemaciaCode #26).** `resetPose` only stores an estimator gyro offset. Add an optional gyro-reset callback (e.g. `Consumer<Rotation2d>`) to `RobotPose.initialize`; when set, `resetPose` also sets the gyro (like `setYaw`). Javadoc: otherwise field-relative driving must use `getEstimatedPose().getRotation()`.
5. **Quest frames (DemaciaCode #12 + comment).** Use all unread tracking frames, not only the newest. Frames newer than the last odometry sample are dropped (`ceilingEntry == null` in `DemaciaPoseEstimator.addVisionMeasurement`): clamp/attach them to the last sample, or read vision before stamping odometry.
6. **Battery alert** (`Quest.java:67-74`, `:171-174`): it latches (`set(true)` only) and checks `battery.isPresent()`, but QuestNav 2026-1.0.0 returns 0 (present) before device data arrives, so it can still fire at connect. Set it every loop: `lowBatteryAlert.set(isConnected() && deviceDataArrived && battery <= LOW_BATTERY_PERCENT)`.
7. **Tests:** `getSourceOffset` (back-right camera y -0.295, yaw -90° → side +0.295, yaw +90°; pure pitch unchanged; there-and-back identity), Quest clamp behaviour, 2D timestamp from the atomic value. Run `./gradlew test`.
8. Robot check for #31: gyro reset the real way, Quest std infinite, robot at a measured spot in front of a tag → Limelight pose matches the tape; then Quest on → both agree.

## Risks
- 838b411 adds the most code and was never compiled.
- Collision detection constants (`COLLISION_ACCELERATION`, hold time) are untuned.
