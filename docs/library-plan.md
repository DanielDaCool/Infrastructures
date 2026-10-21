# Plan – turning Infrastructures into one importable library

Status: **plan only, nothing done yet.** Nothing on `main`, `RobotPose` or `Kinematics` changes
until the owner has tested both branches and approves. This plan is waiting for the owner's own
approach and will be updated to match it.

## Goal

`main` holds one package with all the code (pose estimation, vision, kinematics, acceleration
limiter). When testing is done, it is published so the team installs it like any vendor library
and uses it **without copying or changing the code**.

## Current state

| Branch | Head | Content |
|---|---|---|
| `main` | `030bf01` | WPILib template only (`frc.robot.Main/Robot/RobotContainer`). |
| `RobotPose` | `a46b84a` | `frc.robot.RobotPose` (estimation, vision, 250 Hz odometry thread, collision detection, source-frame offsets), tests, README, review report. Vendordeps: Phoenix 6, QuestNav, WPILib New Commands. |
| `Kinematics` | `e6e4988` | `frc.robot.Kinematics` (`DemaciaKinematics`, `SwerveAccelerationLimiter`, `AccelLimitedSwerve`), tests, README. |
| `kinematics-sync` | `8d6b155` | Same commit as an older `Kinematics`; to be deleted on GitHub. |

Both branches start from `030bf01` and both change the root `README.md` and `build.gradle`, so the
merge has two known conflicts.

## Steps (after approval)

### 1. Merge into `main`
- Merge `RobotPose`, then `Kinematics`, into `main` as merge commits (history kept; the branches
  stay until the owner removes them).
- Resolve `README.md` and `build.gradle` by hand (step 4/3 replace them anyway).

### 2. One package
Root package **`frc.demacia.infrastructures`** (doesn't clash with `frc.robot` in team projects or
with DemaciaCode's own `frc.demacia.RobotPose` / `frc.demacia.kinematics`).

| Now | After |
|---|---|
| `frc.robot.RobotPose` | `frc.demacia.infrastructures.robotpose` |
| `frc.robot.RobotPose.Estimation` | `frc.demacia.infrastructures.robotpose.estimation` |
| `frc.robot.RobotPose.Vision` (+ `VisionTypes`, `visionConfigs`) | `frc.demacia.infrastructures.robotpose.vision` (+ `types`, `configs`) |
| `frc.robot.Kinematics` | `frc.demacia.infrastructures.kinematics` |
| tests under `frc.robot...` | same mapping under `src/test/java` |

- Only `package` / `import` lines and file locations change; no logic changes.
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
- Unit tests stay (`./gradlew test`, JUnit 5, WPILib sim HAL as today).
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
- One root `README.md`: what it is, install (vendordep URL), quick start, then the RobotPose and
  Kinematics sections from the two branch READMEs (adjusted to the new package names).
- `docs/reports/frc-physics-math-code-review.md` kept.
- `CHANGELOG.md` per version.

### 6. Publish (only when the owner says)
- Tag `v2026.0.1`.
- Build and publish the Maven files + the vendordep JSON to GitHub Pages (`gh-pages` branch or a
  GitHub Action on tags).
- Check the install on a clean WPILib 2026 project: install the vendordep, `./gradlew build`,
  call `RobotPose.initialize` and `AccelLimitedSwerve` from a minimal robot.

## Checks before publishing
- `./gradlew build test` passes on `main`.
- A clean robot project with only the vendordep (plus Phoenix 6 and QuestNav) compiles and runs in
  simulation.
- No `frc.robot` package left in the library; no robot-specific constants.
- Javadoc builds (public API documented).

## Open questions (for the owner)
- Package/maven group: `frc.demacia` group with `frc.demacia.infrastructures` package — keep?
- Where the published files live: GitHub Pages of this repo, or a team-owned repo/organization.
- Version scheme (`2026.0.1`, `2026.1.0`, …) and who publishes.
- Keep a small example robot project (separate folder, not published) for simulation tests?
- Should DemaciaCode switch to the library and delete its own copies of RobotPose/kinematics?
- Anything in the owner's approach that differs from the above (waiting for the explanation).
