# Plan – turning Infrastructures into one importable library

Status: **plan only.** Nothing on `main`, `RobotPose` or `Kinematics` changes for packaging
until the owner has tested both branches and approves. This plan is waiting for the owner's own
approach and will be updated to match it. Branch state last checked 2026-10-31.

## Goal

`main` holds one package with all the code (pose estimation, vision, kinematics, acceleration
limiter). When testing is done, it is published so the team installs it like any vendor library
and uses it **without copying or changing the code**.

## Current state

| Branch | Head | Content |
|---|---|---|
| `main` | `26593b6` | WPILib template only (`frc.robot.Main/Robot/RobotContainer`) and an older copy of this plan. |
| `RobotPose` | `f6637e7` | `frc.robot.RobotPose` (estimation, vision, 250 Hz odometry thread, collision detection, source-frame offsets), `PoseLogger` interface for dashboard entries and alerts, README, `docs/design.md`. Vendordeps: Phoenix 6, QuestNav, WPILib New Commands. Tests and review report removed. |
| `Kinematics` | `84d50a9` | `frc.robot.Kinematics` (`DemaciaKinematics`, `SwerveAccelerationLimiter`, `KinematicsConfig`, `KinematicsUtilities`), `frc.robot.example.ExampleChassis`, README, `docs/design.md`. `AccelLimitedSwerve` and tests removed. |
| `development` | latest merge | Both branches merged, plus `frc.robot.example.ExampleSwerveWithPose`. Module docs in `docs/Kinematics*.md` and `docs/RobotPose*.md`; combined root README. |

`kinematics-sync` no longer exists on GitHub.

## Steps (after approval)

### 1. Merge into `main`
- Done on `development` (merge commits, history kept). After testing, merge `development` into
  `main`; the module branches stay until the owner removes them.
- Module branch READMEs conflict with `development`'s root README on every merge: keep
  `development`'s README and apply the module change to `docs/Kinematics.md` or `docs/RobotPose.md`.

### 2. One package
Root package **`frc.demacia.infrastructures`** (doesn't clash with `frc.robot` in team projects or
with DemaciaCode's own `frc.demacia.RobotPose` / `frc.demacia.kinematics`).

| Now | After |
|---|---|
| `frc.robot.RobotPose` | `frc.demacia.infrastructures.robotpose` |
| `frc.robot.RobotPose.Estimation` | `frc.demacia.infrastructures.robotpose.estimation` |
| `frc.robot.RobotPose.Vision` (+ `VisionTypes`, `visionConfigs`) | `frc.demacia.infrastructures.robotpose.vision` (+ `types`, `configs`) |
| `frc.robot.Kinematics` | `frc.demacia.infrastructures.kinematics` |
| `frc.robot.example` | not published (example project, see open questions) |

- Only `package` / `import` lines and file locations change; no logic changes.
- Logging (decided, done on `RobotPose`): the stand-in `frc.demacia.utils.log` is removed. RobotPose
  sends entries and alerts through its `PoseLogger` interface (SmartDashboard and WPILib Alerts by
  default); a team plugs in its own logger with `PoseLogger.set`. Nothing in the jar clashes with
  DemaciaCode's classes.
- Settings (decided, done on `Kinematics`): `KinematicsConstants` and the limiter's constants are
  replaced by `KinematicsConfig` (like DemaciaCode's `ChassisConfig`). RobotPose already takes its
  robot-specific values through `initialize` and the vision configs; its remaining constants are
  estimator tuning and stay as they are.
- Remove the template `frc.robot.Main`, `Robot`, `RobotContainer` and `src/main/deploy` (a library
  has no robot program). If a test robot project is wanted, it lives outside the library (see
  open questions).
- Move the Gradle project from `Infrastructures/` to the repo root (one project, one README).

### 3. Library build
- Gradle `java-library` + `maven-publish` instead of a GradleRIO robot project (no deploy,
  no `ROBOT_MAIN_CLASS`).
- WPILib (`wpimath`, `wpilibj`, `wpiutil`, `ntcore`, `hal`, `wpiunits`, `wpilibNewCommands`) and
  the vendor libraries (Phoenix 6, QuestNav) are **`compileOnly`**: the team project already has
  them through its own WPILib/vendordeps, so the library never ships a second copy.
- Java 17, the WPILib 2026 versions currently used (GradleRIO 2026.2.1, Phoenix 6 26.1.0,
  QuestNav 2026-2.2.0).
- Outputs: `infrastructures-java-<version>.jar`, `-sources.jar`, `-javadoc.jar`, published to a
  Maven layout (`frc/demacia/infrastructures-java/<version>/…`).
- Tests were removed from both branches. `./gradlew test` with JUnit 5 stays wired in Gradle;
  adding tests back before publishing is an open question.
- Version from one place (`version = "2026.0.1"`), reused in the vendordep.

### 4. Vendordep
`DemaciaInfrastructures.json` at the repo root, generated from the version:

```json
{
  "fileName": "DemaciaInfrastructures.json",
  "name": "DemaciaInfrastructures",
  "version": "2026.0.1",
  "frcYear": "2026",
  "uuid": "<new uuid>",
  "mavenUrls": ["https://danieldacool.github.io/Infrastructures/repo"],
  "jsonUrl": "https://danieldacool.github.io/Infrastructures/DemaciaInfrastructures.json",
  "requires": [
    { "uuid": "e995de00-2c64-4df5-8831-c1441420ff19", "errorMessage": "Phoenix 6 is required", "offlineFileName": "Phoenix6-frc2026-latest.json", "onlineUrl": "<Phoenix 6 2026 json url>" },
    { "uuid": "a706fe68-86e5-4aed-92c5-ce05aca007f0", "errorMessage": "QuestNav is required", "offlineFileName": "questnavlib.json", "onlineUrl": "https://maven.questnav.gg/snapshots/gg/questnav/questnavlib-json/2026-2.2.0/questnavlib-json-2026-2.2.0.json" }
  ],
  "javaDependencies": [
    { "groupId": "frc.demacia", "artifactId": "infrastructures-java", "version": "2026.0.1" }
  ],
  "jniDependencies": [],
  "cppDependencies": []
}
```

Team side: VS Code → WPILib → Manage Vendor Libraries → Install new library (online) → paste the
`jsonUrl`. Updates: change the version, republish, the team updates the vendordep.

### 5. Docs
- Root `README.md` from `development`, changed to: install (vendordep URL) instead of copying
  packages, new package names, no branch-copy instructions.
- `docs/Kinematics.md`, `docs/Kinematics-design.md`, `docs/RobotPose.md`, `docs/RobotPose-design.md`
  kept, with package names and links updated.
- `CHANGELOG.md` per version.

### 6. Publish (only when the owner says)
- Tag `v2026.0.1`.
- Build and publish the Maven files + the vendordep JSON to GitHub Pages (`gh-pages` branch or a
  GitHub Action on tags).
- Check the install on a clean WPILib 2026 project: install the vendordep, `./gradlew build`,
  call `RobotPose.initialize` and `DemaciaKinematics` / `SwerveAccelerationLimiter` from a minimal
  robot.

## Checks before publishing
- `./gradlew build test` passes on `main`.
- A clean robot project with only the vendordep (plus Phoenix 6 and QuestNav) compiles and runs in
  simulation.
- No `frc.robot` package left in the library; no robot-specific constants (robot values come from
  `KinematicsConfig`, `RobotPose.initialize` and the vision configs).
- No `frc.demacia.utils.log` class in the jar.
- Javadoc builds (public API documented).

## Open questions (for the owner)
- Package/maven group: `frc.demacia` group with `frc.demacia.infrastructures` package — keep?
- Where the published files live: GitHub Pages of this repo, or a team-owned repo/organization.
- Version scheme (`2026.0.1`, `2026.1.0`, …) and who publishes.
- Keep a small example robot project (separate folder, not published) for simulation tests? The
  current `ExampleChassis` / `ExampleSwerveWithPose` would move there.
- A ready-made `PoseLogger` adapter for DemaciaCode's logger (in DemaciaCode, not in the library)?
- Add unit tests back before the first release?
- Should DemaciaCode switch to the library and delete its own copies of RobotPose/kinematics?
- Anything in the owner's approach that differs from the above (waiting for the explanation).
