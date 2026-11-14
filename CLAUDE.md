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
- **IMU:** CTRE Pigeon 2.0 (CAN ID: `SwerveConstants.PIGEON2_CAN_ID`; Phoenix 6 API — `getYaw().getValueAsDouble()`, `getAngularVelocityZDevice()`, `getAccelerationX/Y()`; calibration done via Phoenix Tuner X, not in code)

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

Two top-level packages:

- **`frc.lib`** — reusable on any robot. Copy a whole folder (e.g. `lib/mechanisms/shooter/` plus `lib/control/`, `lib/hardware/`, `lib/util/`) into another project and it compiles. **Never imports `frc.robot`.** Vendor libraries (`com.ctre`, `com.revrobotics`, `com.thethriftybot`) are imported only under `lib/hardware/`.
- **`frc.robot`** — this robot only: lifecycle, wiring, constants, commands, and subsystems that still read robot constants directly (drivetrain, vision, pathplanning).

Both rules are enforced by `src/test/java/frc/lib/ArchitectureTest.java` — `./gradlew test` fails if either is broken.

```
src/main/java/frc/lib/
├── util/
│   ├── Units.java                          ← all unit conversions + normalizeAngleDeg()
│   ├── Dashboard.java                      ← ALL dashboard output (log / tunable / toggle)
│   ├── RobotLogger.java
│   ├── ConfigVerifier.java / ConfigVerifiable.java / ConfigVerifyResult.java
│
├── hardware/                               ← the ONLY place vendor APIs appear
│   ├── motor/
│   │   ├── MotorIO.java                    ← one interface for every motor controller
│   │   ├── MotorIOConfig.java              ← immutable per-controller settings (Builder)
│   │   ├── MotorVendor.java                ← CTRE_TALONFX | REV_SPARKMAX | REV_SPARKFLEX | THRIFTYBOT_NOVA
│   │   ├── CTREMotor.java / REVMotor.java / NovaMotor.java
│   │   └── MotorModels.java                ← motor datasheet constants (kT, stall, free speed)
│   └── encoder/
│       ├── AbsoluteEncoderIO.java
│       └── ThriftyAnalogEncoder.java / CTRECANcoderEncoder.java / REVThroughBoreEncoder.java
│
├── control/
│   ├── MechanismUnit.java                  ← leader + followers on MotorIO; FF, ramps, sync, limits, homing
│   ├── MechanismConfig.java                ← mechanism-level config; derives one MotorIOConfig per motor
│   ├── MotorConstants.java                 ← FollowMode, loop period, sync defaults
│   └── ff/
│       ├── FFProvider.java
│       └── FF.java                         ← physics FF factories (was MechanismUnit.FF)
│
├── swerve/
│   ├── SwerveModule.java                   ← takes MotorIO drive/steer + AbsoluteEncoderIO
│   ├── SwerveHardware.java                 ← motor/encoder TYPE → MotorIOConfig / encoder
│   ├── DriveInput.java
│   └── DriveOdometryState.java
│
└── mechanisms/                             ← one folder per mechanism
    ├── shooter/
    │   ├── ShooterMechanism.java           ← flywheel + optional turret + optional hood; withReadyGate()
    │   ├── ShooterCalculator.java          ← pure-math ballistics; takes a ShotProfile
    │   ├── ShotProfile.java                ← lookup table + pivot geometry + speed warning
    │   ├── ShooterAimController.java       ← orchestration; talks to AimDriveState / AimDriveModes
    │   └── AimDriveState.java / AimDriveModes.java
    └── elevator/
        └── ElevatorMechanism.java

src/test/java/frc/lib/                      ← mirrors main; FakeMotorIO in hardware/motor/

src/main/java/frc/robot/
├── Main.java
├── Robot.java
├── RobotContainer.java
├── Constants.java                          (minimal — OperatorConstants only)
│
├── commands/
│   ├── DriveWithJoysticks.java
│   ├── XLockCommand.java
│   └── Autos.java
│
├── constants/
│   ├── SwerveConstants.java
│   ├── VisionConstants.java                (AprilTag coprocessor — 10.87.19.11)
│   ├── GamePieceConstants.java             (game piece coprocessor — 10.87.19.12)
│   ├── PathPlannerConstants.java
│   ├── ShooterConstants.java               ← shooter lookup table, pivot geometry → ShooterConstants.PROFILE
│   ├── CANIDs.java / PortIDs.java
│   └── AprilTagIgnore.java
│
├── pathplanning/
│   ├── FieldTargets.java
│   ├── PathfindCommand.java
│   └── GamePieceHuntCommand.java           (4-mode vision-guided collection)
│
├── subsystems/
│   ├── swerveDrive/
│   │   └── SwerveDrive.java                (implements AimDriveState; 4-entry pose history ring buffer)
│   └── vision/
│       ├── VisionSubsystem.java
│       ├── RobotPoseEstimate.java
│       ├── VisionHealthMonitor.java        (diagnostic counters now persisted to DataLog)
│       ├── AprilTagFieldCalTab.java
│       └── GamePieceVisionSubsystem.java   (intake_cam, back-projection, clustering, health)
│
└── util/
    └── AprilTagFieldCal.java
```

SwerveDrive, vision and pathplanning stay in `frc.robot` for now because they read `SwerveConstants` / `VisionConstants` / `PathPlannerConstants` directly. Moving them to `frc.lib` means passing those values in through a config object first, the same way `ShotProfile` replaced `ShooterConstants` inside `ShooterCalculator`.

### Adding a new mechanism

1. Create `frc/lib/mechanisms/<name>/<name>Mechanism.java` built on `MechanismUnit` (never a vendor class).
2. Everything robot-specific (CAN IDs, tables, geometry) comes in through the Builder / a config object — no `frc.robot` imports.
3. Add `src/test/java/frc/lib/mechanisms/<name>/` tests using `FakeMotorIO` (see `MechanismUnitTest` for the pattern).
4. Construct it in `RobotContainer` and `ConfigVerifier.register()` it.

---

## Architectural Principles

### 1. `DriveInput` is the only way into `SwerveDrive` — with one documented exception

No command or subsystem should call `SwerveDrive` drive methods directly. All driver-controlled motion routes through `DriveInput` → `SwerveDrive.drive()` or `SwerveDrive.drivePointAt()`.

**Exception — PathPlanner:** `SwerveDrive.driveRobotRelative(ChassisSpeeds)` exists solely for PathPlanner's `AutoBuilder` lambda. PathPlanner outputs robot-relative `ChassisSpeeds` directly; routing through `DriveInput` would require a coordinate-frame roundtrip for no benefit. This method is registered once in `configureForAutoBuilder()` and never called directly by any command or subsystem.

### 2. `VisionSubsystem` is the only vision interface

All camera access, pose estimation, and tag reading is encapsulated behind `VisionSubsystem`. Nothing outside it touches PhotonVision internals directly.

### 3. English units at interface boundaries, SI internally

User-facing and configuration values are in English units (ft/s, inches, lbs, lb·in²). Internal calculations use SI. All conversions happen at entry points via `Units.java` — never inside control loops.

### 4. Single `GAME_YEAR_FIELD` constant

Defined once in `RobotContainer`. Propagates through the constructor chain to `VisionSubsystem` → `AprilTagFieldCal` → PhotonPoseEstimators. Never hardcode field geometry elsewhere.

### 5. Every motor is a `MotorIO` — nothing above `lib/hardware/` touches vendor APIs

Swerve drive, swerve steer, and every mechanism motor go through the single `MotorIO` interface (`lib/hardware/motor/`). The `MotorVendor` in a `MotorIOConfig` / `MechanismConfig` selects `CTREMotor`, `REVMotor`, or `NovaMotor` — swapping any motor to another vendor is a one-value change. Mechanisms instantiate via `MechanismUnit.create(MechanismConfig)`; swerve modules via `SwerveHardware` + `MotorIO.create()` (see `RobotContainer.buildModule()`). Vendor-only features are reached by casting `MechanismUnit.getLeader()` to the concrete class.

`MotorIO.verifyConfig()` re-applies the motor's FULL configuration before reading it back, so `ConfigVerifier.runAll()` never resets gear ratio, gains, or soft limits to factory defaults.

Feed-forward voltages are computed from first principles using `FF` factories (`lib/control/ff/FF.java`) and motor constants from `MotorModels.java`. Do not use empirical `kG` calibration constants — use the physics-based factories which derive voltages from `stallTorqueNm` and `gearRatio`.

Available factories: `springTurret` (piecewise-linear spring compensation — single calibration table, reverse rotation auto-negates torques, 0V outside calibrated range, deadbanded direction switching), `rotatingArm` (gravity with variable game-piece CG, signed cos output across 0°–180°), `gyroscopicTurret` (Tier 3, compensates flywheel angular momentum resistance to turret rotation — τ = I × ω_flywheel × ω_turret, pairs additively with springTurret), `multiStageElevator` (Tier 3, friction offset deadbanded), `pivotingElevator` (Tier 3, gravity + drivetrain inertia + centripetal, friction offset deadbanded). All unit conversions route through `Units.java` — no inline constants in the FF class.

`MechanismConfig` supports per-follower topology via `withFollowerConfig(modes[], inverted[], leaderIndices[])`. Each follower independently declares MECHANICAL or ENCODER_SYNC mode and which motor in `canIds[]` it tracks — enabling mixed-mode and follower-of-follower chains (e.g. a 4-motor flywheel where two independent sides each have a local mechanical follower). `withFollowMode()` remains available as a convenience for simple same-mode topologies. Nova supports MECHANICAL only (`MechanismUnit` throws at construction if a follower's `MotorIO.supportsEncoderSync()` is false). All motors in one mechanism share a vendor — `new MechanismConfig.Builder(name, canIds, MotorVendor.X)` sets it once. Copy a config with `toBuilder()` (never rebuild field-by-field).

`MechanismUnit.isAtSetpoint()` compares actual position/velocity against the last commanded setpoint using `config.setpointDeadband`. Requires `withSetpointDeadband()` to be set in the config — warns to DS at construction and on first call otherwise.

`ShooterMechanism` supports an optional `withReadyGate(BooleanSupplier gate)` builder method. When set, `isReady()` (i.e. `allAtSetpoint()`) additionally requires `gate.getAsBoolean()` to be true — use this to AND an external condition (e.g. robot heading within turret window) onto the READY state. Usage:
```java
.withReadyGate(() -> aimController.getLastResult().isHeadingInBounds())
```

This makes swapping a motor controller a one-line config change. See `docs/ARCHITECTURE.md` — Mechanism Motor Abstraction for full details.

### 6. Two-phase design before implementation

Architectural decisions are discussed and agreed before any code is written. Explicit signal ("let's write the code" or similar) triggers implementation.

---

### 7. All dashboard output goes through `frc.lib.util.Dashboard`

Use `Dashboard.log(key, value)` for values to view and `Dashboard.tunable(key, default)` /
`Dashboard.toggle(key, default)` (created once, read with `get()` each loop) for values an
operator edits live. Never call SmartDashboard / Telemetry / Tunables directly: `Dashboard` is
implemented on SmartDashboard for WPILib 2026 and on Telemetry + Tunables on the 2027 branch,
so switching WPILib years changes only that file. It is also where publishing policy (what to
send during a match vs. while tuning) belongs.

---

## Naming Conventions

| Pattern | Convention | Examples |
|---|---|---|
| Classes, interfaces, records, enums (and their files) | `UpperCamelCase`; acronyms stay uppercase | `SwerveDrive`, `DriveInput`, `CANIDs`, `FFProvider` |
| Proper-noun-prefixed classes | Follow proper noun | `AprilTagFieldCal`, `PhotonCamera` |
| Constants | `UPPER_SNAKE_CASE` | `MAX_DRIVE_SPEED_MPS`, `WHEEL_COF` |
| Methods and variables | `lowerCamelCase` | `getPose()`, `addVisionMeasurement()` |
| Unit conversions in `Units` | `fromUnitToUnit` in `lowerCamelCase` | `Units.degToRad()`, `Units.inchesToMeters()` |
| New feature packages | `lowercase` | `pathplanning/` |

---

## Unit Conventions

| Value type | Interface unit | Internal unit | Conversion |
|---|---|---|---|
| Linear velocity | ft/s | m/s | `Units.ftpsToMps()` / `Units.mpsToFtps()` |
| Distance / geometry | inches | meters | `Units.inchesToMeters()` |
| Field coordinates | feet | meters | `Units.fieldFeetToMeters()` |
| Angle | degrees | radians | `Units.degToRad()` |
| Robot mass | lbs | kg | `Units.lbsToKg()` |
| Moment of inertia | lb·in² | kg·m² | `Units.lbIn2ToKgM2()` |
| Mechanism torque (FF calibration) | lb·in | N·m | `Units.lbInToNm()` |
| Heading normalization | degrees (any range) | degrees (−180°, 180°] | `Units.normalizeAngleDeg()` |

---

## Drivetrain Subsystem

### `SwerveDrive`

Owns all four modules and the IMU. The only way to move the robot.

**Drive API:**
| Method | Description |
|---|---|
| `drive(DriveInput)` | Normal field-relative driver control |
| `drivePointAt(input, x, y, offset)` | Translation from driver, heading PID locks to target |
| `driveWithHeadingBound(input, minDeg, maxDeg)` | Translation from driver; heading PID holds to nearest bound when outside `[minDeg, maxDeg]`; passes through driver omega when inside bounds. Publishes `Drive/HeadingBound/*` telemetry. |
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
| `getOdometryState()` | Full `DriveOdometryState` snapshot |
| `getDimension(key)` | Geometry query: "wheel-base", "frame-perimeter", "bumper-perimeter" |

**Pose history ring buffer:**
`SwerveDrive` maintains a 4-entry ring buffer of `(Pose2d, timestamp)` pairs, written every loop after `poseEstimator.update()`. `getPoseAtTime()` linearly interpolates between bracketing entries; at 50Hz the buffer covers ~80ms, which encompasses typical PhotonVision ML detection latency (20–60ms). Used exclusively by `GamePieceVisionSubsystem` — AprilTag latency compensation is handled internally by WPILib's `SwerveDrivePoseEstimator.addVisionMeasurement()`.

**PathPlanner integration:**
| Method | Description |
|---|---|
| `configureForAutoBuilder()` | One-time setup — call from RobotContainer after construction |

**Pose estimation:**
`SwerveDrivePoseEstimator` (Kalman filter) replaces `SwerveDriveOdometry`. Odometry (encoder + IMU) updates at 50Hz every loop. Vision measurements injected via `addVisionMeasurement()` from `RobotContainer.updatePoseEstimator()` each loop when valid. Trust is scaled by tag count using `VisionConstants.VISION_STD_DEV_TAG_SCALE`.

### `SwerveModule`

One Thrifty Narrow module. Kraken X60 drive (velocity control), Minion steer (position control). Analog encoder seeded at startup, Minion internal encoder used for closed-loop steering. All PID gains live-tunable from the dashboard.

### `DriveInput`

Immutable value object. The sole public command interface into `SwerveDrive`. Accepts ft/s, converts to m/s internally on construction. CoR in meters, robot-relative.

### `DriveOdometryState`

Full motion state snapshot updated every 20ms. Three buckets:
- `encoderState` — from wheel encoders + kinematics (reliable linear velocity)
- `imuState` — from Pigeon 2.0 (reliable angular velocity)
- `blendedState` — complementary filter blend (alpha tunable live from the dashboard)

All values SI internally. CoR tracked with velocity and acceleration.

---

## Vision Subsystem

### `VisionSubsystem`

Single public interface for all vision data. Two PhotonVision cameras (front + rear).

**Pose API:**
| Method | Description |
|---|---|
| `getBestPose()` | Best accepted `RobotPoseEstimate` (higher-confidence of front/rear) |
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

### `RobotPoseEstimate`

Immutable value record per camera. Fields: `pose` (Pose3d), `timestampSecs`, `ambiguity`, `tagCount`, `cameraName`, `isValid`. Static `best(a, b)` picks lower ambiguity. Static `invalid()` used as null-safe sentinel.

### `VisionHealthMonitor`

Pre-match health validation and per-loop health tracking. Checks camera connectivity, tag visibility, and pose consistency. Exposed via `VisionSubsystem.isHealthy()` and `getHealthStatus()`. Polled in `Robot.disabledPeriodic()` for pit/field readiness confirmation.

Diagnostic counters (`frontMissFrames`, `rearMissFrames`, `frontHighAmbFrames`, `rearHighAmbFrames`, `interCameraDisagree`) are now persisted to DataLog at `/Vision/Debug/*` every loop — enables frame-level post-match root cause analysis in AdvantageScope.

### `AprilTagFieldCal` / `AprilTagFieldCalTab`

Field calibration system. Measures per-tag position offsets vs. WPILib baseline. Outputs correction offsets three ways: Python script capture to laptop, backup to roboRIO, clipboard copy. Values are published through `Dashboard` under "Field Calibration/" for the Elastic "Field Calibration" tab.

### `GamePieceVisionSubsystem`

Separate subsystem for game piece detection via `intake_cam` on the game piece coprocessor (10.87.19.12). Runs independently of `VisionSubsystem`.

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
| `publishHealthStatus()` | Called from `Robot.disabledPeriodic()` — posts to the dashboard for the Elastic "Vision Health" tab |

**DataLog paths:** `/GamePieceVision/CameraConnected`, `/GamePieceVision/PieceDetected`, `/GamePieceVision/TrackedPieceCount`, `/GamePieceVision/Debug/MissFrames`.

**Health check:** No field calibration needed (pieces have no fixed reference positions). Pre-match check: place a game piece ~1–2m in front of intake, confirm `Piece detected = true` on the Elastic "Vision Health" tab.

---

## Commands

### `DriveWithJoysticks`

Default drive command on `SwerveDrive`. Left stick = field-relative translation (ft/s). Right stick twist = rotation (rad/s). Right stick X/Y = dynamic center of rotation (bounded to bumper corners).

**Drive mode enum** (`DriveMode`):
| Mode | Trigger | Behavior |
|---|---|---|
| `NORMAL` | Default | Full driver control |
| `POINT_AT` | `enablePointAt(x, y, offsetDeg)` | Driver translates; heading PID locks to field point |
| `HEADING_BOUND` | `enableHeadingBound(minDeg, maxDeg)` | Driver translates; heading PID clamps to window when outside; driver omega passes through when inside |
| `STATIC_SHOT_LOCK` | `enableStaticShotLock()` | X-brace, no translation — robot stops in place |

**Mode control methods:**
- `enablePointAt(x, y, offsetDeg)` — set `POINT_AT` mode (backward compat)
- `disablePointAt()` — alias for `clearAimMode()` (backward compat)
- `enableHeadingBound(minAbsDeg, maxAbsDeg)` — set `HEADING_BOUND` mode
- `enableStaticShotLock()` — set `STATIC_SHOT_LOCK` mode
- `clearAimMode()` — restore `NORMAL`; unified teardown for all aim modes

### `XLockCommand`

X-brace defense. Commands all four modules to 45° X pattern, zero drive speed. Held while button is pressed.

### `PathfindCommand`

On-the-fly AD* pathfinding to a field target using PathPlanner. Requires `SwerveDrive` — preempts `DriveWithJoysticks` via scheduler, restores it on end. Vision staleness is checked at initialize and raised as a dashboard Alert (command still runs on odometry if stale). Currently uses `pathfindToPose()`. TODO: upgrade to `pathfindThenFollowPath()` once `.path` files are authored in PathPlanner GUI.

### `GamePieceHuntCommand`

Vision-guided autonomous game piece collection. Four modes via `HuntMode` enum:
| Mode | Behavior |
|---|---|
| `NEAREST_PIECE` | Drive to closest detected piece, intake, done |
| `SEQUENTIAL_PIECES` | Intake nearest, then next nearest, repeat until none found |
| `NEAREST_CLUSTER` | Drive to centroid of nearest cluster, intake all reachable, done |
| `SEQUENTIAL_CLUSTERS` | Like above but repeats across all clusters |

**State machine:** `FINDING → PATHFINDING → INTAKING → (DONE | FINDING)`

- **FINDING:** Polls `GamePieceVisionSubsystem` for target. Times out after `HUNT_NO_PIECE_TIMEOUT_SECS`. Computes arrival heading: `atan2(dy, dx) + APPROACH_HEADING_OFFSET_DEG` (intake faces piece). Schedules `AutoBuilder.pathfindToPose()` as inner command.
- **PATHFINDING:** Polls inner pathfinder via `!pathfinder.isScheduled()`. On arrival: calls `intakeTrigger.run()`, transitions to INTAKING.
- **INTAKING:** Waits for `intakeComplete.getAsBoolean()` or `INTAKE_TIMEOUT_SECS`. Sequential modes loop back to FINDING; single-target modes transition to DONE.

**Intake decoupling:** Constructor takes `Runnable intakeTrigger` and `BooleanSupplier intakeComplete` — no intake subsystem import. Wire in `RobotContainer` once intake subsystem exists. `addRequirements(drive)` preempts `DriveWithJoysticks`.

### `Autos.java`

Stub — returns `Commands.none()`. Autonomous path sequences to be built here once field targets and path files are ready.

---

## PathPlanner Integration

### Architecture

`AutoBuilder` is configured once in `SwerveDrive.configureForAutoBuilder()`, called from `RobotContainer` after construction. All PathPlanner dependencies are contained within `SwerveDrive` — `RobotContainer` is unaware of PathPlanner internals.

**AutoBuilder lambda wiring:**
| Lambda | Method |
|---|---|
| Pose supplier | `SwerveDrive::getPose` (Kalman-filtered) |
| Pose reset | `SwerveDrive::resetPose` |
| Speed feedback | `SwerveDrive::getRobotRelativeSpeeds` |
| Speed command | `SwerveDrive::driveRobotRelative` |

### `FieldTargets` *(new)*

String-keyed `Pose2d` lookup. `FieldTargets.get("speaker")` etc. Throws `IllegalArgumentException` on unknown key. All poses defined from blue-origin; PathPlanner handles alliance flip. TODO: populate with real 2026 field coordinates.

### `PathPlannerConstants` *(new)*

Robot mass (lbs → kg), MOI (lb·in² → kg·m²), wheel COF, path velocity/acceleration constraints, AutoBuilder translation/rotation PID defaults, vision staleness threshold, alliance zone exclusion stub.

---

## Robot Periodic Call Order

Every 20ms, in `Robot.robotPeriodic()`:
1. `CommandScheduler.getInstance().run()` — runs subsystem `periodic()` + all active commands
2. `robotContainer.updatePoseEstimator()` — fuses latest valid vision pose into Kalman filter
3. `robotContainer.updateCalibrationTab()` — updates field calibration Elastic tab

---

## Shooter Aim System

Three-layer architecture separating math, hardware, and orchestration:

| Layer | File | Role |
|---|---|---|
| Math | `lib/mechanisms/shooter/ShooterCalculator.java` + `ShotProfile.java` | Pure-math ballistics. No hardware refs. StaticShot + OnTheMove modes with iterative ToF refinement. Outputs `ShooterOutput` including `headingBoundsDeg[2]`. |
| Hardware | `lib/mechanisms/shooter/ShooterMechanism.java` | Flywheel + optional turret + optional hood. Manages motor setpoints, PID, feed-forward. Exposes `isReady()` (optionally gated by `readyGate`). |
| Orchestration | `lib/mechanisms/shooter/ShooterAimController.java` | SubsystemBase. Reads drivetrain state via `AimDriveState` (SwerveDrive), calls `ShooterCalculator`, pushes setpoints to `ShooterMechanism`, sets drive mode via `AimDriveModes` (DriveWithJoysticks). |
| Constants | `constants/ShooterConstants.java` | Lookup table (`LookupEntry[]`), pivot geometry, speed warning threshold. Turret soft/hard limits live in `MechanismConfig` to avoid duplication. |

### `ShooterCalculator`

Two constructors:
- **Convenience** `(ShotProfile, hz, iterations, MechanismConfig turretConfig)` — reads `softLimitReverseDeg`/`softLimitForwardDeg` directly from the config. Single source of truth.
- **Full** `(ShotProfile, hz, iterations, softMin, softMax, hardMin, hardMax)` — for use without a `MechanismConfig` (e.g. `ShooterCalculatorTest`).

**`ShooterOutput`** includes:
- `double[] headingBoundsDeg` — `[normalize(aimFieldDeg − softMax), normalize(aimFieldDeg − softMin)]`. The range of robot headings within which the turret stays inside its soft limits.
- `boolean isHeadingInBounds(double robotHdgDeg)` — wrap-aware check using `isInArc()`.
- `static boolean isInArc(double angle, double arcMin, double arcMax)` — handles window crossing ±180°: uses `arcMin <= arcMax` branch for normal case vs. `angle >= arcMin || angle <= arcMax` for wrapping case.

### `ShooterAimController`

**Enums:** `TurretMode { PHYSICAL_TURRET, SWERVE_AS_TURRET }`, `AimMode { STATIC_SHOT, ON_THE_MOVE, IDLE }`

**Key methods:** `setTarget(type, x, y)`, `setAimMode(mode)`, `clearAimMode()`, `getLastResult()` → `ShooterAimOutput`

**`ShooterAimOutput`** nested class: `shooterResult`, `headingChangeRequired`, `shooterReady`, `activeMode`, `isHeadingInBounds()`

**Static shot** drive mode (two-phase):
- `PHYSICAL_TURRET`: Phase 1 → `enableHeadingBound()` until in bounds; Phase 2 → `enableStaticShotLock()` (X-brace).
- `SWERVE_AS_TURRET`: Phase 1 → `enablePointAt(target)` until pointed; Phase 2 → `enableStaticShotLock()`.

**On the Move** drive mode: `enableHeadingBound(result.headingBoundsDeg[0], result.headingBoundsDeg[1])` each loop.

**Turret OTM — `TurretBoundState` machine** (physical turret only):

| State | Turret command | Transition |
|---|---|---|
| `IN_BOUNDS` | Normal aim from `ShooterCalculator` | → `WAITING_AT_FORWARD_LIMIT` when heading goes CCW past `boundsMin`; → `WAITING_AT_REVERSE_LIMIT` when CW past `boundsMax` |
| `WAITING_AT_FORWARD_LIMIT` | Park at `softMax` | → `IN_BOUNDS` if heading returns inside bounds; → `WAITING_AT_REVERSE_LIMIT` when overshoot exceeds `deadZone × flipThresholdPct` |
| `WAITING_AT_REVERSE_LIMIT` | Park at `softMin` | → `IN_BOUNDS` if heading returns inside bounds; → `WAITING_AT_FORWARD_LIMIT` when overshoot exceeds `deadZone × flipThresholdPct` |

Dead zone width = `360° − (softMax − softMin)`. `flipThresholdPct` defaults to 0.5 (midpoint). Overshoot is computed wrap-correctly via `Units.normalizeAngleDeg(boundEdge − robotHdgDeg)`.

### RobotContainer wiring

```java
ShooterCalculator calc = new ShooterCalculator(ShooterConstants.PROFILE, 50, 5, turretMotor.getConfig());
ShooterAimController aimCtrl = new ShooterAimController(calc, drive, shooter, driveCmd,
    ShooterAimController.TurretMode.PHYSICAL_TURRET);

// Gate isReady() on heading-in-bounds
shooter = new ShooterMechanism.Builder(...)
    .withReadyGate(() -> aimCtrl.getLastResult().isHeadingInBounds())
    .build();

// Button bindings
new JoystickButton(stick, 5).whileHeld(() -> {
    aimCtrl.setTarget("Hub", TARGET_X_FT, TARGET_Y_FT);
    aimCtrl.setAimMode(ShooterAimController.AimMode.ON_THE_MOVE);
}).whenReleased(aimCtrl::clearAimMode);
```

---

## Button Map (current)

| Button | Stick | Binding |
|---|---|---|
| Button 2 | Right | Lock-to-target heading (point-at while held) |
| Button 3 | Right | X-lock defense (while held) |
| Button 4 | Right | Pathfind to target (while held) — currently hardcoded to `"speaker"` |

TODO: replace hardcoded pathfind target with a selector (dashboard chooser or button set).

---

## Season Change Checklist

Each new season:
1. `RobotContainer.GAME_YEAR_FIELD` — update `AprilTagFields` enum value
2. `FieldTargets.java` — replace stub coordinates with real field target poses
3. `PathPlannerConstants.ALLIANCE_ZONE_EXCLUSIONS` — populate with new field zone polygons
4. `AprilTagIgnore.java` — review which tag IDs to suppress for known problem tags
5. `VisionConstants` — re-verify AprilTag camera transforms if robot geometry changed
6. `GamePieceConstants.GAME_PIECE_CAMERA_TRANSFORM` — remeasure intake camera mounting if robot changed
7. `GamePieceConstants.GAME_PIECE_HEIGHT_M` — update for new game piece height above carpet
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
| Weigh robot (mass + MOI) | `PathPlannerConstants.java` |
| Tune PathPlanner translation/rotation PID | `PathPlannerConstants.java` |
| Tune vision std devs on carpet | `VisionConstants.java` |
| Populate 2026 field target coordinates | `FieldTargets.java` |
| Author final approach `.path` files | PathPlanner GUI → `src/main/deploy/pathplanner/paths/` |
| Switch `pathfindToPose()` → `pathfindThenFollowPath()` | `PathfindCommand.java` |
| Alliance zone exclusion polygons | `PathPlannerConstants.ALLIANCE_ZONE_EXCLUSIONS` |
| Pathfind target selector (vs. hardcoded `"speaker"`) | `RobotContainer.java` |
| Measure actual steer offset voltages | `SwerveConstants.java` |
| Confirm CAN IDs match physical wiring | `SwerveConstants.java` |
| Replace `buildTagReading()` delta calc with full reprojection geometry | `VisionSubsystem.java` |
| Update PhotonVision API (deprecated `getLatestResult()`, `update()`) | `VisionSubsystem.java`, `GamePieceVisionSubsystem.java` |
| Update `getPositionError()` (deprecated in WPILib 2026) | `SwerveDrive.java` |
| Measure intake camera mounting position + angle | `GamePieceConstants.GAME_PIECE_CAMERA_TRANSFORM` |
| Update game piece height once 2026 game piece is known | `GamePieceConstants.GAME_PIECE_HEIGHT_M` |
| Wire `GamePieceHuntCommand` button bindings | `RobotContainer.configureButtonBindings()` |
| Wire intake trigger + completion callbacks to hunt command | `RobotContainer` — once intake subsystem exists |
| Validate `MechanismUnit` abstraction on hardware (first real mechanism deploy) | new mechanism subsystem |
| Measure flywheel MOI for `gyroscopicTurret` FF (lb·in² from CAD or physical measurement) | `ShooterMechanism` usage site |
| Measure spring torque calibration table for `springTurret` | `ShooterMechanism` usage site |

---

*Update this file as the codebase evolves. It is the single source of truth for AI-assisted development context.*
