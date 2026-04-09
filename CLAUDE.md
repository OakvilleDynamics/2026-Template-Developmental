# CLAUDE.md — FRC Team 8719 Robot Codebase Context

This file provides persistent context for AI-assisted development on the team 8719 robot codebase.
It is loaded automatically by Claude Code at the start of every session. Keep it current as the codebase evolves.

---

## Project Overview

From-scratch swerve drive robot codebase for **FRC Team 8719**, built using:
- **WPILib 2026** (Java)
- **Phoenix 6** — CTRE motor/sensor library (Kraken X60 drive, Minion steer)
- **PhotonVision** — three-camera vision system (AprilTag pose estimation + game piece detection)
- **PathPlannerLib 2026.1.2** — on-the-fly AD* pathfinding

---

## Hardware

### Drivetrain
- **Modules:** Thrifty Narrow swerve (×4 — FL, FR, BL, BR)
- **Drive motors:** Kraken X60 (TalonFX, Phoenix 6)
- **Steer motors:** Minion (TalonFX, Phoenix 6)
- **Steer encoders:** Thrifty absolute analog encoders (0–3.3V, roboRIO analog ports)
- **IMU:** ADIS16470

### Vision
Two dedicated OrangePi 5 coprocessors — isolated by role for reliability and headroom:

**AprilTag coprocessor** — static IP `10.87.19.11`
- Cameras: `front_cam` (OV9281), `rear_cam` (OV9281) — global shutter, grayscale, optimized for AprilTag
- Use: Robot pose estimation via AprilTag, fused into Kalman filter

**Game piece coprocessor** — static IP `10.87.19.12`
- Camera: `intake_cam` (OV9782) — rolling shutter, color, optimized for ML game piece detection
- Use: Back-project detections to field coordinates for autonomous game piece hunting

`PhotonCamera` resolves by name through NetworkTables — no robot code change needed when cameras are on separate physical devices.

---

## Package Structure

```
src/main/java/frc/robot/
├── Main.java
├── Robot.java
├── RobotContainer.java
├── Constants.java                          (minimal — OperatorConstants only)
│
├── commands/
│   ├── driveWithJoysticks.java
│   ├── xLockCommand.java
│   └── Autos.java
│
├── constants/
│   ├── swerveConstants.java
│   ├── visionConstants.java                (AprilTag coprocessor — 10.87.19.11)
│   ├── gamePieceConstants.java             (game piece coprocessor — 10.87.19.12)
│   ├── pathplannerConstants.java
│   └── AprilTagIgnore.java
│
├── pathplanning/
│   ├── FieldTargets.java
│   ├── pathfindCommand.java
│   └── gamePieceHuntCommand.java           (4-mode vision-guided collection)
│
├── subsystems/
│   ├── swerveDrive/
│   │   ├── swerveDrive.java                (includes 4-entry pose history ring buffer)
│   │   ├── swerveModule.java
│   │   ├── driveInput.java
│   │   └── driveOdometryState.java
│   └── vision/
│       ├── visionSubsystem.java
│       ├── robotPoseEstimate.java
│       ├── visionHealthMonitor.java        (diagnostic counters now persisted to DataLog)
│       ├── AprilTagFieldCalTab.java
│       └── gamePieceVisionSubsystem.java   (intake_cam, back-projection, clustering, health)
│
└── util/
    ├── units.java
    ├── AprilTagFieldCal.java
    ├── motors/                             (mechanism motor abstraction)
    │   ├── ffProvider.java
    │   ├── motorConstants.java
    │   ├── motorModels.java                ← motor datasheet constants (kT, stall, free speed)
    │   ├── mechanismConfig.java
    │   ├── mechanismUnit.java              ← includes static nested class FF (physics FF library)
    │   ├── CTREMechanismUnit.java
    │   ├── REVMechanismUnit.java
    │   └── NovaMechanismUnit.java
    └── mechanisms/                         (pre-packaged mechanism subsystems)
        └── shooterMechanism.java           ← flywheel + optional turret + optional hood
```

---

## Architectural Principles

### 1. `driveInput` is the only way into `swerveDrive` — with one documented exception

No command or subsystem should call `swerveDrive` drive methods directly. All driver-controlled motion routes through `driveInput` → `swerveDrive.drive()` or `swerveDrive.drivePointAt()`.

**Exception — PathPlanner:** `swerveDrive.driveRobotRelative(ChassisSpeeds)` exists solely for PathPlanner's `AutoBuilder` lambda. PathPlanner outputs robot-relative `ChassisSpeeds` directly; routing through `driveInput` would require a coordinate-frame roundtrip for no benefit. This method is registered once in `configureForAutoBuilder()` and never called directly by any command or subsystem.

### 2. `visionSubsystem` is the only vision interface

All camera access, pose estimation, and tag reading is encapsulated behind `visionSubsystem`. Nothing outside it touches PhotonVision internals directly.

### 3. English units at interface boundaries, SI internally

User-facing and configuration values are in English units (ft/s, inches, lbs, lb·in²). Internal calculations use SI. All conversions happen at entry points via `units.java` — never inside control loops.

### 4. Single `GAME_YEAR_FIELD` constant

Defined once in `RobotContainer`. Propagates through the constructor chain to `visionSubsystem` → `AprilTagFieldCal` → PhotonPoseEstimators. Never hardcode field geometry elsewhere.

### 5. Mechanism subsystems never touch vendor motor APIs directly

All mechanism motor control routes through `mechanismUnit` in `util/motors/`. No subsystem imports `com.revrobotics`, `com.ctre`, or `com.thethriftybot` directly. Instantiate via `mechanismUnit.create(mechanismConfig)` — the factory returns the correct vendor implementation transparently.

Feed-forward voltages are computed from first principles using `mechanismUnit.FF` factories and motor constants from `motorModels.java`. Do not use empirical `kG` calibration constants — use the physics-based factories which derive voltages from `stallTorqueNm` and `gearRatio`.

Available factories: `springTurret` (piecewise-linear spring compensation — single calibration table, reverse rotation auto-negates torques, 0V outside calibrated range, deadbanded direction switching), `rotatingArm` (gravity with variable game-piece CG, signed cos output across 0°–180°), `gyroscopicTurret` (Tier 3, compensates flywheel angular momentum resistance to turret rotation — τ = I × ω_flywheel × ω_turret, pairs additively with springTurret), `multiStageElevator` (Tier 3, friction offset deadbanded), `pivotingElevator` (Tier 3, gravity + drivetrain inertia + centripetal, friction offset deadbanded). All unit conversions route through `units.java` — no inline constants in the FF class.

`mechanismConfig` supports per-follower topology via `withFollowerConfig(modes[], inverted[], leaderIndices[])`. Each follower independently declares MECHANICAL or ENCODER_SYNC mode and which motor in `canIds[]` it tracks — enabling mixed-mode and follower-of-follower chains (e.g. a 4-motor flywheel where two independent sides each have a local mechanical follower). `withFollowMode()` remains available as a convenience for simple same-mode topologies. Nova supports MECHANICAL only.

`mechanismUnit.isAtSetpoint()` compares actual position/velocity against the last commanded setpoint using `config.setpointDeadband`. Requires `withSetpointDeadband()` to be set in the config — warns to DS at construction and on first call otherwise.

This makes swapping a motor controller a one-line config change. See `docs/ARCHITECTURE.md` — Mechanism Motor Abstraction for full details.

### 6. Two-phase design before implementation

Architectural decisions are discussed and agreed before any code is written. Explicit signal ("let's write the code" or similar) triggers implementation.

---

## Naming Conventions

| Pattern | Convention | Examples |
|---|---|---|
| Robot-specific subsystem classes | `camelCase` | `swerveDrive`, `driveInput`, `swerveModule` |
| Proper-noun-prefixed classes | Follow proper noun | `AprilTagFieldCal`, `PhotonCamera` |
| Constants | `UPPER_SNAKE_CASE` | `MAX_DRIVE_SPEED_MPS`, `WHEEL_COF` |
| Methods and variables | `camelCase` | `getPose()`, `addVisionMeasurement()` |
| New feature packages | `lowercase` | `pathplanning/` |

---

## Unit Conventions

| Value type | Interface unit | Internal unit | Conversion |
|---|---|---|---|
| Linear velocity | ft/s | m/s | `units.ftps_mps()` |
| Distance / geometry | inches | meters | `units.inches_m()` |
| Field coordinates | feet | meters | `units.fieldFeet_m()` |
| Angle | degrees | radians | `units.deg_rad()` |
| Robot mass | lbs | kg | `units.lbs_kg()` |
| Moment of inertia | lb·in² | kg·m² | `units.lbIn2_kgM2()` |
| Mechanism torque (FF calibration) | lb·in | N·m | `units.lbIn_Nm()` |

---

## Drivetrain Subsystem

### `swerveDrive`

Owns all four modules and the IMU. The only way to move the robot.

**Drive API:**
| Method | Description |
|---|---|
| `drive(driveInput)` | Normal field-relative driver control |
| `drivePointAt(input, x, y, offset)` | Translation from driver, heading PID locks to target |
| `lockWheelsX()` | X-brace defense pattern |
| `stop()` | Zero drive speed, hold steer angles |
| `driveRobotRelative(ChassisSpeeds)` | PathPlanner-only — robot-relative, bypasses field-relative conversion |

**Pose API:**
| Method | Description |
|---|---|
| `getPose()` | Kalman-filtered pose (vision + odometry fused) |
| `resetPose(Pose2d)` | Seed estimator with known position |
| `getRobotRelativeSpeeds()` | Encoder-derived ChassisSpeeds — PathPlanner feedback |
| `addVisionMeasurement(pose, timestamp, tagCount)` | Inject vision into Kalman filter, trust scaled by tag count |
| `getPoseAtTime(timestampSecs)` | Pose closest to given FPGA timestamp from ring buffer — for game piece back-projection |
| `getOdometryState()` | Full `driveOdometryState` snapshot |
| `getDimension(key)` | Geometry query: "wheel-base", "frame-perimeter", "bumper-perimeter" |

**Pose history ring buffer:**
`swerveDrive` maintains a 4-entry ring buffer of `(Pose2d, timestamp)` pairs, written every loop after `poseEstimator.update()`. `getPoseAtTime()` linearly interpolates between bracketing entries; at 50Hz the buffer covers ~80ms, which encompasses typical PhotonVision ML detection latency (20–60ms). Used exclusively by `gamePieceVisionSubsystem` — AprilTag latency compensation is handled internally by WPILib's `SwerveDrivePoseEstimator.addVisionMeasurement()`.

**PathPlanner integration:**
| Method | Description |
|---|---|
| `configureForAutoBuilder()` | One-time setup — call from RobotContainer after construction |

**Pose estimation:**
`SwerveDrivePoseEstimator` (Kalman filter) replaces `SwerveDriveOdometry`. Odometry (encoder + IMU) updates at 50Hz every loop. Vision measurements injected via `addVisionMeasurement()` from `RobotContainer.updatePoseEstimator()` each loop when valid. Trust is scaled by tag count using `visionConstants.VISION_STD_DEV_TAG_SCALE`.

### `swerveModule`

One Thrifty Narrow module. Kraken X60 drive (velocity control), Minion steer (position control). Analog encoder seeded at startup, Minion internal encoder used for closed-loop steering. All PID gains live-tunable from SmartDashboard.

### `driveInput`

Immutable value object. The sole public command interface into `swerveDrive`. Accepts ft/s, converts to m/s internally on construction. CoR in meters, robot-relative.

### `driveOdometryState`

Full motion state snapshot updated every 20ms. Three buckets:
- `encoderState` — from wheel encoders + kinematics (reliable linear velocity)
- `imuState` — from ADIS16470 (reliable angular velocity)
- `blendedState` — complementary filter blend (alpha tunable live from SmartDashboard)

All values SI internally. CoR tracked with velocity and acceleration.

---

## Vision Subsystem

### `visionSubsystem`

Single public interface for all vision data. Two PhotonVision cameras (front + rear).

**Pose API:**
| Method | Description |
|---|---|
| `getBestPose()` | Best accepted `robotPoseEstimate` (higher-confidence of front/rear) |
| `getFrontPose()` / `getRearPose()` | Per-camera estimates |
| `hasValidPose()` | True if best pose passed all filters |
| `bothCamerasValid()` | True if both cameras have valid estimates |
| `getBestPoseZ()` | Robot height above floor (meters) — for climbing/elevation detection |
| `getBestPosePitch()` | Robot pitch angle (radians) — for tilt detection |
| `getBestPoseRoll()` | Robot roll angle (radians) — for tilt detection |

**Filtering pipeline** (each frame):
1. Must have targets
2. Must meet `MIN_TAGS_FOR_ESTIMATE` tag count
3. Ambiguity below `MAX_AMBIGUITY`
4. Pose jump below `MAX_POSE_JUMP_M` from last accepted estimate

### `robotPoseEstimate`

Immutable value record per camera. Fields: `pose` (Pose3d), `timestampSecs`, `ambiguity`, `tagCount`, `cameraName`, `isValid`. Static `best(a, b)` picks lower ambiguity. Static `invalid()` used as null-safe sentinel.

### `visionHealthMonitor`

Pre-match health validation and per-loop health tracking. Checks camera connectivity, tag visibility, and pose consistency. Exposed via `visionSubsystem.isHealthy()` and `getHealthStatus()`. Polled in `Robot.disabledPeriodic()` for pit/field readiness confirmation.

Diagnostic counters (`frontMissFrames`, `rearMissFrames`, `frontHighAmbFrames`, `rearHighAmbFrames`, `interCameraDisagree`) are now persisted to DataLog at `/Vision/Debug/*` every loop — enables frame-level post-match root cause analysis in AdvantageScope.

### `AprilTagFieldCal` / `AprilTagFieldCalTab`

Field calibration system. Measures per-tag position offsets vs. WPILib baseline. Outputs correction offsets three ways: Python script capture to laptop, backup to roboRIO, clipboard copy. Tab is live on Shuffleboard "Field Calibration" tab.

### `gamePieceVisionSubsystem`

Separate subsystem for game piece detection via `intake_cam` on the game piece coprocessor (10.87.19.12). Runs independently of `visionSubsystem`.

**periodic():** Pulls latest PhotonPipeline result → retrieves `drive.getPoseAtTime(captureTimestamp)` → computes camera field pose → back-projects each detected target to field `Translation2d` using ray-casting → merges into tracked list (position averaging) → expires stale entries.

**Back-projection math:**
```
verticalAngle = cameraMountPitch − target.getPitch()    (radians)
horizontalDist = (cameraHeight − GAME_PIECE_HEIGHT_M) / tan(verticalAngle)
bearing = cameraYawField + target.getYaw()
pieceX = cameraX + horizontalDist × cos(bearing)
pieceY = cameraY + horizontalDist × sin(bearing)
```
Guard: `tan(verticalAngle) ≤ 0` → skip (piece above camera horizon).

**Public API:**
| Method | Description |
|---|---|
| `getFieldRelativePieces()` | All freshness-filtered tracked pieces |
| `getNearestPiece(robotPose)` | Closest single `Translation2d` to robot |
| `getClusters(robotPose)` | Greedy distance clusters sorted by centroid proximity |
| `isCameraConnected()` | NT connectivity check |
| `hasPieceDetection()` | True if a piece was detected within `PIECE_STALE_SECS` |
| `publishHealthStatus()` | Called from `Robot.disabledPeriodic()` — posts to Shuffleboard "Vision Health" |

**DataLog paths:** `/GamePieceVision/CameraConnected`, `/GamePieceVision/PieceDetected`, `/GamePieceVision/TrackedPieceCount`, `/GamePieceVision/Debug/MissFrames`.

**Health check:** No field calibration needed (pieces have no fixed reference positions). Pre-match check: place a game piece ~1–2m in front of intake, confirm `PieceDetected = true` on Shuffleboard.

---

## Commands

### `driveWithJoysticks`

Default drive command on `swerveDrive`. Left stick = field-relative translation (ft/s). Right stick twist = rotation (rad/s). Right stick X/Y = dynamic center of rotation (bounded to bumper corners). `enablePointAt()` / `disablePointAt()` toggled by button binding — heading PID takes over omega.

### `xLockCommand`

X-brace defense. Commands all four modules to 45° X pattern, zero drive speed. Held while button is pressed.

### `pathfindCommand`

On-the-fly AD* pathfinding to a field target using PathPlanner. Requires `swerveDrive` — preempts `driveWithJoysticks` via scheduler, restores it on end. Vision staleness is checked at initialize and logged to Shuffleboard "Pathfinding" tab (command still runs on odometry if stale). Currently uses `pathfindToPose()`. TODO: upgrade to `pathfindThenFollowPath()` once `.path` files are authored in PathPlanner GUI.

### `gamePieceHuntCommand`

Vision-guided autonomous game piece collection. Four modes via `HuntMode` enum:
| Mode | Behavior |
|---|---|
| `NEAREST_PIECE` | Drive to closest detected piece, intake, done |
| `SEQUENTIAL_PIECES` | Intake nearest, then next nearest, repeat until none found |
| `NEAREST_CLUSTER` | Drive to centroid of nearest cluster, intake all reachable, done |
| `SEQUENTIAL_CLUSTERS` | Like above but repeats across all clusters |

**State machine:** `FINDING → PATHFINDING → INTAKING → (DONE | FINDING)`

- **FINDING:** Polls `gamePieceVisionSubsystem` for target. Times out after `HUNT_NO_PIECE_TIMEOUT_SECS`. Computes arrival heading: `atan2(dy, dx) + APPROACH_HEADING_OFFSET_DEG` (intake faces piece). Schedules `AutoBuilder.pathfindToPose()` as inner command.
- **PATHFINDING:** Polls inner pathfinder via `!pathfinder.isScheduled()`. On arrival: calls `intakeTrigger.run()`, transitions to INTAKING.
- **INTAKING:** Waits for `intakeComplete.getAsBoolean()` or `INTAKE_TIMEOUT_SECS`. Sequential modes loop back to FINDING; single-target modes transition to DONE.

**Intake decoupling:** Constructor takes `Runnable intakeTrigger` and `BooleanSupplier intakeComplete` — no intake subsystem import. Wire in `RobotContainer` once intake subsystem exists. `addRequirements(drive)` preempts `driveWithJoysticks`.

### `Autos.java`

Stub — returns `Commands.none()`. Autonomous path sequences to be built here once field targets and path files are ready.

---

## PathPlanner Integration

### Architecture

`AutoBuilder` is configured once in `swerveDrive.configureForAutoBuilder()`, called from `RobotContainer` after construction. All PathPlanner dependencies are contained within `swerveDrive` — `RobotContainer` is unaware of PathPlanner internals.

**AutoBuilder lambda wiring:**
| Lambda | Method |
|---|---|
| Pose supplier | `swerveDrive::getPose` (Kalman-filtered) |
| Pose reset | `swerveDrive::resetPose` |
| Speed feedback | `swerveDrive::getRobotRelativeSpeeds` |
| Speed command | `swerveDrive::driveRobotRelative` |

### `FieldTargets` *(new)*

String-keyed `Pose2d` lookup. `FieldTargets.get("speaker")` etc. Throws `IllegalArgumentException` on unknown key. All poses defined from blue-origin; PathPlanner handles alliance flip. TODO: populate with real 2026 field coordinates.

### `pathplannerConstants` *(new)*

Robot mass (lbs → kg), MOI (lb·in² → kg·m²), wheel COF, path velocity/acceleration constraints, AutoBuilder translation/rotation PID defaults, vision staleness threshold, alliance zone exclusion stub.

---

## Robot Periodic Call Order

Every 20ms, in `Robot.robotPeriodic()`:
1. `CommandScheduler.getInstance().run()` — runs subsystem `periodic()` + all active commands
2. `robotContainer.updatePoseEstimator()` — fuses latest valid vision pose into Kalman filter
3. `robotContainer.updateCalibrationTab()` — updates field calibration Shuffleboard tab

---

## Button Map (current)

| Button | Stick | Binding |
|---|---|---|
| Button 2 | Right | Lock-to-target heading (point-at while held) |
| Button 3 | Right | X-lock defense (while held) |
| Button 4 | Right | Pathfind to target (while held) — currently hardcoded to `"speaker"` |

TODO: replace hardcoded pathfind target with a selector (SmartDashboard chooser or button set).

---

## Season Change Checklist

Each new season:
1. `RobotContainer.GAME_YEAR_FIELD` — update `AprilTagFields` enum value
2. `FieldTargets.java` — replace stub coordinates with real field target poses
3. `pathplannerConstants.ALLIANCE_ZONE_EXCLUSIONS` — populate with new field zone polygons
4. `AprilTagIgnore.java` — review which tag IDs to suppress for known problem tags
5. `visionConstants` — re-verify AprilTag camera transforms if robot geometry changed
6. `gamePieceConstants.GAME_PIECE_CAMERA_TRANSFORM` — remeasure intake camera mounting if robot changed
7. `gamePieceConstants.GAME_PIECE_HEIGHT_M` — update for new game piece height above carpet
8. PathPlanner GUI — re-author final approach `.path` files for each target

---

## WPILib 2026 Notes

- `CommandBase` removed — all commands extend `Command` directly
- `Command.schedule()` deprecated — use `CommandScheduler.getInstance().schedule(cmd)`
- Phoenix 6 required for Kraken X60 and Minion — no Phoenix 5 APIs

---

## Known TODOs / Open Items

| Item | Location |
|---|---|
| Weigh robot (mass + MOI) | `pathplannerConstants.java` |
| Tune PathPlanner translation/rotation PID | `pathplannerConstants.java` |
| Tune vision std devs on carpet | `visionConstants.java` |
| Populate 2026 field target coordinates | `FieldTargets.java` |
| Author final approach `.path` files | PathPlanner GUI → `src/main/deploy/pathplanner/paths/` |
| Switch `pathfindToPose()` → `pathfindThenFollowPath()` | `pathfindCommand.java` |
| Alliance zone exclusion polygons | `pathplannerConstants.ALLIANCE_ZONE_EXCLUSIONS` |
| Pathfind target selector (vs. hardcoded `"speaker"`) | `RobotContainer.java` |
| Measure actual steer offset voltages | `swerveConstants.java` |
| Confirm CAN IDs match physical wiring | `swerveConstants.java` |
| Replace `buildTagReading()` delta calc with full reprojection geometry | `visionSubsystem.java` |
| Update PhotonVision API (deprecated `getLatestResult()`, `update()`) | `visionSubsystem.java`, `gamePieceVisionSubsystem.java` |
| Update `getPositionError()` (deprecated in WPILib 2026) | `swerveDrive.java` |
| Measure intake camera mounting position + angle | `gamePieceConstants.GAME_PIECE_CAMERA_TRANSFORM` |
| Update game piece height once 2026 game piece is known | `gamePieceConstants.GAME_PIECE_HEIGHT_M` |
| Wire `gamePieceHuntCommand` button bindings | `RobotContainer.configureButtonBindings()` |
| Wire intake trigger + completion callbacks to hunt command | `RobotContainer` — once intake subsystem exists |
| Validate `mechanismUnit` abstraction on hardware (first real mechanism deploy) | new mechanism subsystem |
| Measure flywheel MOI for `gyroscopicTurret` FF (lb·in² from CAD or physical measurement) | `shooterMechanism` usage site |
| Measure spring torque calibration table for `springTurret` | `shooterMechanism` usage site |

---

*Update this file as the codebase evolves. It is the single source of truth for AI-assisted development context.*
