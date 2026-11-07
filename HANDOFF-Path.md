# Handoff: Path module redesign (2026-11-07, Israel time)

Branch `Paths` (local, tracks `origin/Paths`), worktree `C:/Users/User/Infrastructures/.claude/worktrees/paths`.
9 local commits on top of `e6d93d5`, **not pushed**. Baseline: `demacia5635/DemaciaCode` branch `path` @ `31b1782`.

## Done
- `Infrastructures/src/main/java/frc/robot/Path/`: 8 files, 602 Java lines (was 2,614; baseline 642):
  `PathConfig`, `Point`, `Segment`, `Line`, `Arc`, `Path` (corner circles through the waypoint, tangent legs, stops, flip),
  `PathFollower` (projected progress, speed, merge, heading, stops, finish), `FollowPathCommand` (suppliers + consumer).
- Removed: `PathGeometry`, `SpeedProfile`, `PathConstraints`, `PathFollowerConfig`, `PathDrivetrain`.
- Examples: `ExampleSwerveWithPose.followCommand/follow`, `ExampleChassis.followCommand` (+ abstract `getPose()`).
- Docs: `Path/README.md` (new API), `docs/Path-design.md` (short math note), old doc in `docs/archive/Path-design.md`, root README.
- Tests: `Infrastructures/src/test/java/frc/robot/Path/PathTest.java`, 13 tests, committed (user approved).
- `.claude/subagents-learnings.md` updated.

## Verified (executed)
- `JAVA_HOME=C:/Users/Public/wpilib/2026/jdk ./gradlew.bat build` from `Infrastructures/`: BUILD SUCCESSFUL, 13/13 tests pass
  (last run 17:03).
- Point-robot sims (tests + reviewer): max distance to path 0.031 m, max command heading jump 5.3 deg (left/right corners,
  0 and 0.1 s lag); stops finish with a 0.1 m/s drive deadband and with 200 ms latency + 0.1 s lag
  (that case overshoots ~0.23 m and settles in ~7 s).
- Baseline bugs reproduced by running the baseline Java (harness: scratchpad `sim/src/Sim.java`, temporary).

## Not verified
- WPILib simulation (`simulateJava`) and the robot. **Core behavior change: test at low speed first.**
- `FollowPathCommand` itself (alliance read, start-too-far check, `end()` zero) has no test: it needs DriverStation/HAL.
- Real-drivetrain lag and the default gains (`MERGE_DISTANCE` 0.5, `LAG_TIME` 0.05, `HEADING_KP` 4, `FINAL_KP` 3).

## Decisions (user)
- Corner circle passes through the waypoint (baseline intent). Drivetrain hookup: pose supplier, field-speeds supplier,
  field-relative drive consumer, subsystem. Flip: rotate or mirror via `PathConfig.MIRRORED_FIELD`, set per season.
- Approved improvements: arc cap sqrt(aLat r), lag look-ahead, smooth merge (`psi = -atan(eN / MERGE_DISTANCE)`),
  heading interpolation, 2-D accel limit; plus `stop(dwell)`, start-too-far check, `withRadius`.
- Later (not in code): per-point velocity, omega slew limit, per-path limits, events, telemetry.

## Gotchas
- Lower capture speed is `MIN_SPEED` 0.15 m/s until within `POSITION_TOLERANCE / 2`; then zero. Needed for drive deadbands.
- Corners whose arc would sweep more than turn + 90 deg throw (close same-direction corners would loop).
- End-point dwell is ignored. A 0-turn middle point is dropped from the geometry (its angle still anchors heading).
- No built-in timeout: use `.withTimeout(...)`.
- `Path.flipped(boolean)` is package-private only so tests can cover both flip modes.
- Commit `706a213` holds only the deletions (a failed `git add`); `1cb4547` adds the files. Not amended.
- Builds need `JAVA_HOME=C:/Users/Public/wpilib/2026/jdk` (system Java 21 crashes the HAL).

## Uncommitted state
- This file only.

## Next
1. Push `Paths` (needs approval). 2. Run in WPILib sim with the real example chassis. 3. Robot test at `MAX_VELOCITY` 0.5,
   then tune `LAG_TIME` and `MERGE_DISTANCE` from logged eN at arc entry/exit.
