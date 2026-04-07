# CLAUDE.md — FRC Team 8719 Robot Codebase Context

This file provides persistent context for AI-assisted development on the team 8719 robot codebase.
It is loaded automatically by Claude Code at the start of every session. Keep it current as the codebase evolves.

---

## Project Overview

From-scratch swerve drive robot codebase for **FRC Team 8719**, built using:
- **WPILib 2026** (Java)
- **Phoenix 6** — CTRE motor/sensor library (Kraken X60 drive, Minion steer)
- **PhotonVision** — dual-camera AprilTag pose estimation (OrangePi 5 coprocessor)
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
- **Cameras:** Dual PhotonVision cameras (front + rear)
- **Coprocessor:** OrangePi 5 — static IP 10.87.19.11
- **Use:** AprilTag pose estimation, fused into Kalman filter

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
│   ├── visionConstants.java
│   ├── pathplannerConstants.java           (new — PathPlanner config)
│   └── AprilTagIgnore.java
│
├── pathplanning/                           (new package)
│   ├── FieldTargets.java
│   └── pathfindCommand.java
│
├── subsystems/
│   ├── swerveDrive/
│   │   ├── swerveDrive.java
│   │   ├── swerveModule.java
│   │   ├── driveInput.java
│   │   └── driveOdometryState.java
│   └── vision/
│       ├── visionSubsystem.java
│       ├── robotPoseEstimate.java
│       ├── visionHealthMonitor.java
│       └── AprilTagFieldCalTab.java
│
└── util/
    ├── units.java
    ├── AprilTagFieldCal.java
    └── motors/                             (new — mechanism motor abstraction)
        ├── ffProvider.java
        ├── motorConstants.java
        ├── mechanismConfig.java
        ├── mechanismUnit.java
        ├── CTREMechanismUnit.java
        ├── REVMechanismUnit.java
        └── NovaMechanismUnit.java
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
| `getOdometryState()` | Full `driveOdometryState` snapshot |
| `getDimension(key)` | Geometry query: "wheel-base", "frame-perimeter", "bumper-perimeter" |

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

### `AprilTagFieldCal` / `AprilTagFieldCalTab`

Field calibration system. Measures per-tag position offsets vs. WPILib baseline. Outputs correction offsets three ways: Python script capture to laptop, backup to roboRIO, clipboard copy. Tab is live on Shuffleboard "Field Calibration" tab.

---

## Commands

### `driveWithJoysticks`

Default drive command on `swerveDrive`. Left stick = field-relative translation (ft/s). Right stick twist = rotation (rad/s). Right stick X/Y = dynamic center of rotation (bounded to bumper corners). `enablePointAt()` / `disablePointAt()` toggled by button binding — heading PID takes over omega.

### `xLockCommand`

X-brace defense. Commands all four modules to 45° X pattern, zero drive speed. Held while button is pressed.

### `pathfindCommand` *(new)*

On-the-fly AD* pathfinding to a field target using PathPlanner. Requires `swerveDrive` — preempts `driveWithJoysticks` via scheduler, restores it on end. Vision staleness is checked at initialize and logged to Shuffleboard "Pathfinding" tab (command still runs on odometry if stale). Currently uses `pathfindToPose()`. TODO: upgrade to `pathfindThenFollowPath()` once `.path` files are authored in PathPlanner GUI.

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
5. `visionConstants` — re-verify camera transforms if robot geometry changed
6. PathPlanner GUI — re-author final approach `.path` files for each target

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
| Update PhotonVision API (deprecated `getLatestResult()`, `update()`) | `visionSubsystem.java` |
| Update `getPositionError()` (deprecated in WPILib 2026) | `swerveDrive.java` |
| Write first mechanism subsystem using `mechanismUnit` (validate abstraction on hardware) | new subsystem |

---

*Update this file as the codebase evolves. It is the single source of truth for AI-assisted development context.*
