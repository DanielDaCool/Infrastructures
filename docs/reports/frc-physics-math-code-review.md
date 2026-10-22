# Physics and math review – RobotPose and kinematics-sync

Date: 2026-10-20. Scope: the physics, math, geometry, frame and timestamp code on two branches of
`DanielDaCool/Infrastructures`. Motion profiles, controllers, the acceleration limiter's limit
policy and tuning constants were out of scope.

| Branch | Reviewed at | Code |
|---|---|---|
| `RobotPose` | `5a0fc51` | `RobotPose/Estimation/*`, `RobotPose/Vision/**`, `RobotPose.java`, `RoboRioOrientation.java` |
| `kinematics-sync` | `177dea2` | `Kinematics/DemaciaKinematics.java`, `AccelLimitedSwerve.java` (frames, kinematics, read-back), `KinematicsUtilities`/`ChassisAccel` where reachable |

Environment: WPILib/GradleRIO 2026.2.1, Java 17 (WPILib JDK 17.0.16), Phoenix 6 2026, QuestNav lib
2026-2.2.0, LimelightHelpers 1.14. The system Java 21 crashes in `wpiHal.dll` when the tests load
the HAL, so every Gradle command used the WPILib JDK
(`JAVA_HOME=C:/Users/Public/wpilib/2026/jdk`).

## A. Executive summary

- Confirmed bugs: **7** (5 on RobotPose: C1, C2, C3, E1, E2; 2 on kinematics-sync: K1, K2).
- Unresolved: **1** (U1, the Limelight 3D robot-space convention: Limelight's own documentation
  contradicts itself, needs a robot test).
- Fixed and verified: **5** (C1, C2, E1, K1, K2). Documented, not changed: **1** (C3). Not
  fixed by decision: **1** (E2). Overall: partially verified (see G).

How each finding was checked: one investigator per area (estimation, vision, kinematics), then a
separate adversarial skeptic and an independent verifier per area, each in its own detached
worktree. Each derived the expected result on its own and ran tests whose oracle was not the
implementation's formula (WPILib `SwerveDriveKinematics`/`Pose3d.relativeTo`/`Pose2d.exp`
integration, hand derivations).

| ID | Severity | Branch | Where | Defect |
|---|---|---|---|---|
| C1 | High | RobotPose | `LimelightTagCamera2d` constructor / `seesTag()` | NPE at construction: any `VisionConfig` with a 2D Limelight crashes at startup. Introduced by `cd2eb94` in this session. |
| C2 | Medium | RobotPose | `LimelightTagCamera2d.getCameraToTag/getDistanceFromCamera` | Lateral position wrong for a pitched camera and an off-axis tag (5–17 cm at typical geometry). |
| C3 | Medium | RobotPose | `LimelightTagCamera2d`, `LimelightTagCamera2dConfig`, README | 2D pitch is "positive = up", the opposite of the WPILib frame the contract requires (and the Quest uses). |
| E1 | Medium/Low | RobotPose | `DemaciaPoseEstimator.dropMotionIntoHit` | Scales the whole wheel motion during a hit instead of removing only the part into the hit. |
| E2 | Low | RobotPose | `DemaciaPoseEstimator.addVisionMeasurement` | A frame up to 50 ms newer than the newest odometry sample is fused at that sample: steady bias of v·Δ. |
| K1 | Medium | kinematics-sync | `AccelLimitedSwerve.send` | No discretize: the robot drifts sideways when it drives and turns at once (≈ v·sin(ω·dt/2)). |
| K2 | Low | kinematics-sync | `AccelLimitedSwerve.calculate/send` | A NaN heading reaches the module states and the stored speeds, then the next loop brakes hard. |
| U1 | Unresolved | RobotPose | `LimelightTagCamera3dConfig.sourceFrame()` | Whether `camerapose_robotspace_set` wants y right (current mirror) or y left. |

## B. Findings

### C1 – 2D Limelight constructor throws (High)

- **Where:** `LimelightTagCamera2d.java`, `seesTag()` and the constructor; `BaseVisionSource.java:46`.
- **Why it is wrong:** `super(config)` runs `SmartDashboard.putData(this)`, and WPILib 2026.2.1
  `SendableRegistry.publish` calls `builder.update()` straight away (source lines 473-474). That
  evaluates the `is see` property, which calls `seesTag()` while `Table` is still null (it is
  assigned after `super`). The old `shouldUpdate()` had a null check; `cd2eb94` moved the `tv` read
  to `seesTag()` without it.
- **Evidence (executed, three agents independently):**
  `NullPointerException: Cannot invoke "NetworkTable.getEntry(String)" because "this.Table" is null`,
  stack `seesTag ← SendableBuilderImpl.update ← SendableRegistry.publish ← SmartDashboard.putData ← BaseVisionSource.<init>`.
  The 3D source and the Quest construct fine.
- **Fix:** null guard in `seesTag()`.

### C2 – 2D lateral math for a pitched camera (Medium)

- **Where:** `LimelightTagCamera2d.getCameraToTag()` / `getDistanceFromCamera()`.
- **Model:** Limelight tx/ty are pinhole angles, computed per axis as atan of normalized image
  coordinates (Limelight docs "From Pixels to Angles", Release 1.0 §5.4: "assume that we have a
  perfect pinhole camera", tan(ax) = x, tan(ay) = y). The current NetworkTables API page does not
  restate it.
- **Derivation:** the ray in camera axes is (1, −tan tx, tan ty). Tilted up by θ it is
  ∝ (cos(θ+ty), −tan tx·cos ty, sin(θ+ty)). Scaled to the height difference dz:
  - forward = dz / tan(θ+ty) (the code has this right);
  - lateral = −dz·tan tx·cos ty / sin(θ+ty). The code computes −dz·tan tx·cos(θ+ty)/sin(θ+ty)
    (range/cos tx at bearing yaw − tx), short by cos(θ+ty)/cos ty. Exact only when θ = 0 or tx = 0.
- **Evidence (executed, oracle = WPILib `Pose3d.relativeTo` of a placed tag):**

  | Case | Error |
  |---|---|
  | level camera with yaw, any heading | 0.000 m |
  | pitched up 20–25°, tx = 0 | 0.000 m |
  | pitched up 25°, tx −25.9°, ty 7.7° | 0.121 m |
  | pitched up 25°, close, tx 28.5°, ty 23.4° | 0.166 m |
  | README `BACK_2D` mount (back, up 20°) | 0.087–0.119 m |

- **Fix:** rotate the exact ray by the whole offset rotation (this also covers roll and gives the
  WPILib pitch sign of C3) and scale it to the tag height; frames whose ray doesn't reach the tag
  height are dropped by the existing finite check. Verified error 0 (≤ 1e-9 m) in every case above.
- **Assumption:** crosshair at the principal point, undistorted image (same as the old code).

### C3 – 2D pitch sign breaks the frame contract (Medium)

- **Where:** `LimelightTagCamera2d` (`alpha = pitch + ty`), `LimelightTagCamera2dConfig` javadoc,
  README (2D bullet, `BACK_2D`/`FRONT_3D` examples).
- **Why it is wrong:** `BaseVisionSourceConfig` and the README state that every offset is in the
  WPILib robot frame with right-hand rotations and that sources never flip signs. In WPILib a
  positive pitch points the camera **down**: `Rotation3d(0, +20°, 0)` maps (1, 0, 0) to
  (0.940, 0, −0.342) (executed). The Quest uses the same `Transform3d` with WPILib semantics, so a
  single config convention cannot be right for both.
- **Evidence (executed):** the same up-tilted 2D scene written with the WPILib sign is off by
  1.2–48 m in the old code; a Quest pitch written with the 2D convention gives a 0.36 m error that
  turns with the heading.
- **Proposed fix:** use the WPILib rotation directly and change the docs and examples to
  "tilted up = negative pitch". That would change the meaning of existing 2D configs.
- **Decision (owner):** keep "positive pitch = up" for the 2D Limelight and document it as an
  explicit exception to the frame contract. No behaviour change; C2's fix uses the existing sign.
- **Docs check (2026-10-21):**
  - Limelight "Estimating Distance" tutorial (current docs): `d = (h2 - h1) / tan(a1 + a2)` with
    `a2 = ty` and the sample `limelightMountAngleDegrees = 25.0` for a camera looking up at a
    target above it, so the Limelight convention for the 2D mount angle is **positive = tilted
    up**. That is what this class uses.
  - WPILib coordinate system (current docs): NWU axes (x forward, y left, z up), rotations CCW
    positive looking down each positive axis. A rotation about +y (left) with that rule turns +x
    toward −z, so **positive WPILib pitch = tilted down**. Executed with robotpy-wpimath
    2026.2.2: `Translation3d(1,0,0).rotateBy(Rotation3d(0, 20°, 0))` = (0.940, 0, −0.342).
  - So the two documents use opposite signs, and the 2D class follows Limelight's. The decision
    stands; the javadoc and README must keep saying that the 2D Limelight pitch is the one
    exception, entered as "positive = up", while the 3D Limelight and Quest use WPILib pitch.

### E1 – collision handling scales instead of removing the into-hit part (Medium/Low)

- **Where:** `DemaciaPoseEstimator.dropMotionIntoHit`.
- **Why it is wrong:** the javadoc and README say only the motion into the hit is dropped and
  motion to the side is kept. The code multiplies the whole translation by 1 − max(0, cos α).
  Head-on, sideways and away happen to agree; every oblique angle doesn't.
- **Evidence (executed):** wall ahead, 0.1 m of wheel motion at 30° gives x = 0.0116 m into the wall
  (expected 0); at 45° the code keeps (0.0207, 0.0207), the documented behaviour is (0, 0.0707).
- **Fix:** project in the robot frame and remove only the into-hit component.
- **Disagreement:** the skeptic agreed the code contradicts its docs but warned that a bare
  projection leaks |m|·sin ε sideways when the measured hit direction is off by ε (the scaling only
  leaks ≈ ε²/2), and suggested also scaling the remaining part by (1 − intoHit). The verifier
  recommended the bare projection. Resolution: the projection is what the code documents and is the
  physically correct removal; the ε sensitivity is a model limit of a single accelerometer read.
  Only active for 60 ms after a > 3 g hit, and only with collision detection turned on.

### E2 – frames ahead of odometry are biased by v·Δ (Low)

- **Where:** `DemaciaPoseEstimator.addVisionMeasurement`, the `MAX_VISION_AHEAD_OF_ODOMETRY_SECONDS`
  branch.
- **Derivation:** a frame captured at t_L+Δ is compared with the estimate at t_L, then the later
  odometry (which includes the motion during Δ) is replayed on top. With error e at t_L:
  e' = (1 − K)e + K·vΔ, fixed point e* = vΔ for any gain K.
- **Evidence (executed):** 4 m/s, exact 250 Hz odometry, exact frames 8 ms ahead: steady error
  0.03200 m (also 0.0320 m at constant curvature). Frames behind odometry: 0 error.
- **How often:** a timing simulation (Quest at ~120 Hz, 1 ms CAN latency, jittered loops) put 14%
  (250 Hz odometry) to 26% (100 Hz) of Quest frames ahead, with a mean error of 2–7 mm and peaks of
  1–2.4 cm at 4 m/s. Limelight frames are never ahead. Hence Low.
- **Fix options:** (A) hold ahead frames and insert them at their own time once odometry reaches
  them (verifier's choice: zero bias, one loop of latency, ~25 lines, must clear on `resetPose`);
  (B) shift the measurement back by the newest twist rate (skeptic's choice: no state, but can blow
  up when the newest sample is the 1 µs-spaced duplicate, and has an aΔ²/2 error).
  Either fix changes the expected value of the tracked test
  `visionSlightlyNewerThanTheLatestSampleIsUsed`, which asserts the biased behaviour.
- **Decision (owner, confirmed 2026-10-21):** kept as is. The bias is small (mean 2–7 mm, peaks
  1–2.4 cm at 4 m/s), only Quest frames hit it, and the test
  `visionSlightlyNewerThanTheLatestSampleIsUsed` documents the current behaviour. Revisit with
  option A if the Quest std dev is lowered enough for a 1–2 cm bias to matter.

### K1 – no discretize in `AccelLimitedSwerve` (Medium)

- **Where:** `AccelLimitedSwerve.send` (kinematics-sync).
- **Derivation:** module states held for dt while the body turns at ω move the robot along
  dt·sinc(φ/2)·R(θ0)·R(φ/2)·v, φ = ω·dt: the travel direction is turned by +ω·dt/2 (3.4° at 6 rad/s),
  a sideways drift of ≈ |v|·sin(ω·dt/2). WPILib's `ChassisSpeeds.discretize` (2026 source:
  `Pose2d.kZero.log(Pose2d(v·dt, φ))/dt`) cancels it exactly, and the WPILib 2026 swervebot example
  calls it before `toSwerveModuleStates`. `DemaciaKinematics`' javadoc tells callers to discretize;
  `AccelLimitedSwerve` is its only caller and doesn't.
- **Evidence (executed, two independent plants):** field +x at 2 m/s while spinning at 3 rad/s
  drifts y = +0.11996 m in 2 s (prediction 0.11996).
- **Fix:** desaturate, then discretize, then build the states; read back by undoing the discretize
  (`Pose2d.exp` of the module twist / dt) so the limiter's stored speeds aren't turned every loop.
  Without that undo the existing test `convergesWhileSpinning` fails. Verified drift 0.0000 m
  undesaturated, −0.0009 m in 2 s desaturated; ω = 0 unchanged within 4e-15.
- **Context:** the handoff left the lead-angle term as optional future work; the gap is between
  `AccelLimitedSwerve` and the contract its own kinematics class documents.

### K2 – NaN heading poisons the stored speeds (Low)

- **Where:** `AccelLimitedSwerve.calculate` (both overloads) and `send`.
- **Evidence (executed):** one loop with `Rotation2d.fromRadians(NaN)` at 3 m/s: every module gets
  speed NaN at angle 0, the stored speeds become NaN, and the next good loop commands 0.16 m/s (the
  limiter maps NaN to 0), a 2.84 m/s step.
- **Fix:** use the last heading when the heading isn't finite, before the limiter runs. Limits:
  a gyro that stays NaN freezes the heading (field-relative drive goes wrong while the robot turns).
  Realism is low (Phoenix signals don't return NaN), but it is reachable through a NaN fused pose.

### U1 – Limelight robot-space convention (unresolved)

`LimelightTagCamera3dConfig` mirrors y (C = diag(1, −1, 1)) before `setCameraPose_RobotSpace`,
which also flips roll and yaw and keeps pitch. This came from handoff item 1 (DemaciaCode #31).

- For the mirror: Limelight "AprilTag coordinate systems" page (Robot Space Y+ points right),
  MegaTag2 page ("LL Forward, LL Right, LL Up").
- Against: Limelight's LimelightLib page for this exact call ("x forward, y left, z up";
  `side … left of robot center`; `pitch = -30 // camera tilted down`), LimelightHelpers reading
  `camerapose_robotspace` straight into a WPILib `Pose3d`, and public team code (1678 C2025, 2713
  Robot2025) passing the WPILib transform unchanged. A March 2026 Chief Delphi thread reports the
  same contradiction with no official answer.
- Both reviewers lean (60–65%) toward "no mirror", but the documentation is not decisive, so the
  code was **not** changed. The pitch sign Limelight expects is also unsettled.
- **Status (2026-10-21):** the owner will run the robot check below and set `sourceFrame()` from
  its result.
- Robot check: mount a camera off-center (y ≠ 0) and yawed, put a tag at a known spot, and see
  which convention makes the MegaTag2 pose stop moving when the robot turns in place. Reading
  `camerapose_robotspace` back only echoes the input.

## C. Rejected candidates (checked, correct)

- Centripetal removal adds ω²·r (the accelerometer reads −ω²r); pure spin gives 0 at the center.
- `RoboRioOrientation` axes are right-handed and match the documented tilt check.
- Splitting a twist proportionally preserves the composed pose (one-parameter subgroup).
- `getPoseAt` uses the same split as `addVisionMeasurement`; arc interpolation matches R·sinθ.
- Gyro offset after a reset, across the ±180° wrap; theta residual wrap.
- Hit direction and motion are in the same (gyro) frame; the sign drops forward motion into a wall.
- Gain K = q/(q+√(qr)) matches WPILib; q = 0, r = 0, r = ∞ give no NaN.
- Odometry arc/chord math; `PhoenixOdometryThread` latency stamping.
- 2D: level-camera math, bearing yaw − tx, heading rotation and robot = tag − robotToTag; t2d
  indices and capture timestamp; MegaTag2 timestamp and `SetRobotOrientation` units.
- Quest: setPose and measurement transforms are inverses (round trip 0 error with pitch/roll/yaw);
  QuestNav timestamps are in the roboRIO time base.
- Quest vs Limelight disagreement check: motion compensation sign is right.
- Kinematics: inverse kinematics and desaturation match WPILib to 1e-9 on 2000 random cases and
  three layouts; least-squares forward kinematics with or without the gyro ω is exact for
  consistent states, also off-center; field/robot conversions at 0/90/180/−90/wrap; `onYawReset`.

## D. Tests and builds

All commands: `cd Infrastructures && JAVA_HOME=C:/Users/Public/wpilib/2026/jdk ./gradlew …`.
Review worktrees were detached copies; the branches themselves were not modified during the review.

| Where | Command | Result |
|---|---|---|
| RobotPose `5a0fc51` | `./gradlew build` | BUILD SUCCESSFUL, 25 tests passed (9 tracked + 16 untracked) |
| kinematics-sync `177dea2` | `./gradlew build test --rerun-tasks` | BUILD SUCCESSFUL, 11 tests passed |
| rev-est (investigator) | audit tests | 10 run, 2 failed = E1, E2 reproduced |
| ver-est (verifier) | old code + oracle tests / with fixes | exit 1, 3 failed / exit 0, 13 passed |
| rev-vis, ver-vis | 2D construct + oracle tests, old / fixed | exit 1 (NPE, 4 oracle failures) / exit 0, 16 passed |
| rev-kin, ver-kin | oracle and discretize tests, old / fix B + NaN fix | exit 1 (2 failed) / exit 0, 16 passed |

Not tested: anything that needs a robot (U1, real Limelight tx/ty model, collision thresholds,
QuestNav device data on a real headset).

## E. Fixes and git results

Every fix was implemented by one engineer, then checked by a separate reviewer who read the diff,
derived the math again, ran the build, ran the new tests against the old code (they fail) and
mutated the fix (flipped pitch sign, flipped gyro rotation: the tests catch both). Both reviews
ended APPROVED.

| ID | Status | Branch | Commit | Message |
|---|---|---|---|---|
| C1 | Fixed and verified | RobotPose | `67a5b1c` | fix the 2d limelight constructor crashing on the dashboard reading tv before the table is set |
| C2 | Fixed and verified (pinhole model, not checked on a camera) | RobotPose | `077d484` | fix the 2d limelight side offset for a pitched camera and document its pitch sign |
| C3 | Documented as an exception (owner decision) | RobotPose | `077d484` | same commit |
| E1 | Fixed and verified | RobotPose | `ef4163b` | drop only the part of the wheel motion that goes into a hit |
| E2 | Not fixed (owner decision) | – | – | – |
| K1 | Fixed and verified | kinematics-sync | `b1095af` | discretize the module states in AccelLimitedSwerve so the robot doesn't drift while spinning |
| K2 | Fixed and verified | kinematics-sync | `7a5a1c4` | use the last heading in AccelLimitedSwerve when the gyro heading is nan |
| U1 | Unresolved, no change | – | – | – |

Fix details:

- **C2:** ray (1, −tan tx, tan ty) tilted up by the configured pitch (this class's +up sign),
  scaled to the tag height, turned by the camera yaw. A ray that never reaches the tag height is
  dropped. The dashboard distance is the length of that vector. Identical to the old math at
  pitch 0 or tx = 0.
- **E1:** the hit direction is turned into the robot frame and only the into-hit component of
  the twist is removed; heading is kept.
- **K1:** desaturate, discretize with `CYCLE_DT`, build the states; the read-back undoes the
  discretize, so the limiter keeps working on the mean velocity. The stand-still path is
  unchanged. Residual: about 1 mm per 2 s of drift when spinning fast at full speed, because the
  discretized speeds are desaturated again.
- **K2:** a non-finite heading is replaced by the last heading in both `calculate` overloads and
  in `onYawReset`, before the limiter.

Regression tests (left untracked; the repo doesn't commit new tests, per the owner):
`CollisionProjectionTest`, `LimelightTagCamera2dGeometryTest` (RobotPose) and
`AccelLimitedSwerveDiscretizeTest` (kinematics-sync).

Final checks:

| Branch | Command | Result |
|---|---|---|
| RobotPose `ef4163b` | `./gradlew build --rerun-tasks` | BUILD SUCCESSFUL, 33 tests passed, 0 failed |
| kinematics-sync `7a5a1c4` | `./gradlew build test` | BUILD SUCCESSFUL, 16 tests passed, 0 failed |
| RobotPose, old sources + new tests | `./gradlew test` (review) | 8 of 33 failed (C1, C2, E1 caught) |
| kinematics-sync, old sources + new tests | `./gradlew test` (review) | 3 of 16 failed (K1, K2 caught) |

Not tested on a robot: the 2D tx/ty pinhole model, the collision behaviour, K1 on the real
drivetrain.

## E2. Later fix (2026-10-21)

| ID | Severity | Branch | Commit | Message |
|---|---|---|---|---|
| R1 | Medium | RobotPose | `1f1917f` | skip odometry samples measured before a pose or gyro reset |

- **Where:** `DemaciaPoseEstimator.addOdometryData` / `resetPose`, `DemaciaOdometry.setModulePositions`.
- **Why it was wrong:** with `PhoenixOdometryThread`, a sample the gyro measured before
  `setYaw`/`resetPose` can still be on its way (not yet in the thread's queue) when the reset
  runs. It was applied after the reset with the old gyro reading, so the heading jumped by the
  old-to-new difference for one sample (180° after "reset gyro 180") and its motion was put on
  top of the reset pose.
- **Fix:** the estimator stores the FPGA time of the reset and skips samples measured before it,
  only moving the module baseline so their motion isn't counted later.
- **Evidence:** executed model of the heading update: before 0°, 180°, 180° after a reset to
  180°; after the fix 180°, 180°. Not compiled (the environment couldn't reach WPILib/CTRE
  maven); no committed test (new tests stay untracked). Run `./gradlew build test`.

## E3. Pose estimator and odometry update (2026-10-22)

From the simulation study (fresh seeds, see README "What changed"). Commits on `RobotPose`:

| ID | Commit | Message |
|---|---|---|
| O1 | `697b33a` | skip nan odometry samples and modules whose encoder jumped, remove the unused module weights |
| P1 | `3cde284` | move the estimate back to each vision frame instead of replaying the history, trust that follows the uncertainty, jump filter with escape, slip and collision detector |
| O2 | `e8eb55f` | let the odometry thread reader move each signal to the sample time, alert when a signal stops updating |
| P2 | `6a99b01` | add getFuturePose, an initialize overload with adaptiveTrust, a vision source registration hook, log the uncertainty and detector flags |
| V1 | `af81635` | trust limelight frames by tag distance and count, drop megatag2 frames taken while spinning or with another heading |
| V2 | `2cd53dd` | quest: subtract the configured latency, ignore and re-align after a jump or lost tracking, thin to 10 hz, debounce disconnects |

- **E1 follow-up:** the collision threshold is 5 g (3 g gave 5x more false hits with an
  off-center roboRIO) and the hold 0.15 s. The drop into the hit is only used with
  `adaptiveTrust` off; with it on, the slip/collision detector raises the uncertainty instead.
- **E2:** still kept by decision (frames up to 50 ms ahead go on the newest sample).
- **R1:** kept for odometry samples; vision frames are now also dropped until the first sample
  after a reset.
- **Evidence:** compiled and run with javac against stand-in WPILib/CTRE/QuestNav stubs (real
  WPILib 2026.2.1 geometry sources), 44 JUnit tests pass; the Java estimator reproduces the
  simulation's reference (`est_k.py`) to 0.17 mm on 5 scenarios. Not built with Gradle, not run
  on a robot. Run `./gradlew build test`.

## F. Remaining risks

- U1 is open (owner will check on the robot): the 3D Limelight pose sent may have the wrong side/roll/yaw (and possibly pitch) sign.
- E2 is kept by decision: Quest frames ahead of odometry keep a small v·Δ bias.
- R1 and the 2026-10-22 update (E3) were compiled against stubs only, not with Gradle.
- `adaptiveTrust` depends on the slip/collision detector; the push test must show it fires, or
  turn `adaptiveTrust` off.
- C3 changes what a 2D config's pitch means; any existing robot config must flip its 2D pitch sign.
- C2 assumes the pinhole tx/ty model and the crosshair at the principal point.
- E1 and the detector still depend on one accelerometer read per loop.
- Nothing here was run on a robot.

## G. Verdict

**Partially verified.** Every confirmed bug that the owner chose to fix (C1, C2, E1, K1, K2) is
fixed, reviewed and covered by tests that fail on the old code; the builds pass on both branches.
Not complete: E2 is left unfixed by decision, C3 is documented instead of changed, U1 (the 3D
Limelight robot-space convention) can only be settled on the robot, and none of this has run on
hardware.
