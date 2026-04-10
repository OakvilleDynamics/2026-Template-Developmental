# FRC Team 8719 — Robot Codebase Architecture
### 2026 Season | Swerve Drive + Vision + PathPlanner

---

## What This Document Is

A guide to how the robot code is organized, what each piece does, and how they connect. Written for team members who need to understand, modify, or extend the code — not just run it.

---

## Big Picture

The robot is a **swerve drive** controlled by two flight sticks. It tracks its position on the field using a combination of **wheel encoders** and **AprilTag cameras**, fused together by a **Kalman filter**. A driver can press a button to trigger **autonomous pathfinding** — either to a fixed scoring target or to the nearest detected game piece on the field.

```
Driver sticks ──► driveWithJoysticks ──► driveInput ──► swerveDrive ──► 4× swerveModule
                                                              ▲
PathPlanner ──────────────────────────────────────────────────┘
  ▲                                                           ▲
  │ gamePieceHuntCommand                    SwerveDrivePoseEstimator
  │  (piece detection → auto-navigate)       ▲                  ▲
  │                                    wheel encoders      PhotonVision
gamePieceVisionSubsystem                  + IMU              (AprilTags)
  (OV9782 intake camera → field position)
```

---

## Folder Structure

```
src/main/java/frc/robot/
│
├── Robot.java              ← Top-level lifecycle (init, periodic per mode)
├── RobotContainer.java     ← Wires everything together; button bindings
├── Constants.java          ← Minimal operator constants
│
├── commands/               ← Things the robot does (driver commands)
│   ├── driveWithJoysticks  ← Default: read sticks, drive robot
│   ├── xLockCommand        ← Defense: lock wheels in X pattern
│   └── Autos.java          ← Autonomous routines (stub for now)
│
├── constants/              ← All tunable numbers in one place
│   ├── swerveConstants         ← CAN IDs, gear ratios, speed limits, PID defaults
│   ├── visionConstants         ← AprilTag camera names, transforms, filter thresholds
│   ├── gamePieceConstants      ← Game piece camera, clustering, hunt behavior
│   ├── pathplannerConstants    ← PathPlanner config, robot mass/MOI, constraints
│   ├── shooterConstants        ← Shooter lookup table, pivot geometry, speed warning threshold
│   └── AprilTagIgnore          ← Tag IDs to suppress at specific events
│
├── pathplanning/           ← PathPlanner integration
│   ├── FieldTargets            ← "speaker" → field coordinate lookup table
│   ├── pathfindCommand         ← Navigate to fixed target on button press
│   └── gamePieceHuntCommand    ← Vision-guided game piece collection (4 modes)
│
├── subsystems/
│   ├── swerveDrive/        ← Drivetrain
│   │   ├── swerveDrive         ← Manages all 4 modules + pose estimator + pose history
│   │   ├── swerveModule        ← One wheel/motor pair
│   │   ├── driveInput          ← Data object: vx, vy, omega, center of rotation
│   │   └── driveOdometryState  ← Motion state snapshot (velocity, accel, etc.)
│   └── vision/             ← Vision
│       ├── visionSubsystem         ← Public interface: AprilTag pose estimates
│       ├── gamePieceVisionSubsystem ← Game piece detection, field position tracking
│       ├── robotPoseEstimate        ← One camera's pose estimate (AprilTag)
│       ├── visionHealthMonitor      ← Pre-match checks, ongoing health + DataLog
│       └── AprilTagFieldCalTab      ← Shuffleboard calibration interface
│
└── util/
    ├── units.java          ← All unit conversions (inches↔m, lbs↔kg, etc.) + normalizeAngleDeg()
    ├── AprilTagFieldCal    ← Field layout, per-tag offsets, geometry
    ├── ShooterCalculator   ← Pure-math ballistics (StaticShot + OnTheMove), outputs headingBoundsDeg
    ├── shooterAimController ← Orchestration: drivetrain state → calculator → mechanism + drive mode
    └── motors/             ← Vendor-agnostic mechanism motor abstraction
        ├── ffProvider          ← @FunctionalInterface: (pos, vel, accel) → volts
        ├── motorConstants      ← Vendor/FollowMode enums + package constants
        ├── motorModels         ← Motor datasheet constants (kT, stall, free speed)
        ├── mechanismConfig     ← Immutable config (Builder pattern)
        ├── mechanismUnit       ← Abstract base + static factory + FF library
        ├── CTREMechanismUnit   ← TalonFX (Phoenix 6) implementation
        ├── REVMechanismUnit    ← SparkMax / SparkFlex implementation
        └── NovaMechanismUnit   ← ThriftyBot Nova implementation
```

---

## The Drivetrain

### How driving works

The driver moves the left stick to translate (move forward/back/strafe) and twists the right stick to rotate. These inputs go into `driveWithJoysticks`, which reads the joystick axes, applies deadband and input shaping, then builds a `driveInput` object and hands it to `swerveDrive`.

**Field-relative control** means "push stick forward" always moves the robot toward the far field wall — regardless of which way the robot is facing. The code rotates joystick inputs by the robot's current heading before sending them to the wheels.

**Dynamic center of rotation** — the right stick X/Y axis shifts the pivot point of rotation anywhere within the robot's bumper footprint. Centered stick = pivot in place. Full deflection = pivot at a bumper corner. This lets the driver swing around a game piece or defense target.

**Drive modes** (`driveWithJoysticks.DriveMode`): `NORMAL` (default), `POINT_AT` (heading PID locks to field point), `HEADING_BOUND` (heading PID clamps to window when outside, driver omega passes through when inside), `STATIC_SHOT_LOCK` (X-brace, no translation). `shooterAimController` drives these transitions — see Shooter Aim System section.

### `driveInput` — the one interface rule

**The only way to command the drivetrain is through `driveInput`.** No command or subsystem directly tells the motors what to do — they all build a `driveInput` and pass it to `swerveDrive`. This keeps input handling cleanly separated from drive execution.

The one exception: PathPlanner bypasses `driveInput` (see PathPlanner section below).

### Each `swerveModule`

One Kraken X60 (drive) + one Minion (steer) + one Thrifty analog encoder. The analog encoder measures absolute wheel angle and seeds the Minion's built-in encoder at startup. After that, the Minion's high-resolution internal encoder runs the closed-loop steering — the analog sensor is only used once at power-on.

All PID gains are published to SmartDashboard and can be adjusted live during tuning without redeploying.

### The odometry state

Every 20ms, `swerveDrive` computes a `driveOdometryState` — a rich snapshot of motion containing three buckets:

| Bucket | Source | Best for |
|---|---|---|
| `encoderState` | Wheel encoders + kinematics | Linear velocity, distance |
| `imuState` | CTRE Pigeon 2.0 IMU | Angular velocity, heading rate |
| `blendedState` | Complementary filter of both | General-purpose motion monitoring |

The blend weight (how much encoder vs. IMU) is tunable live from SmartDashboard.

---

## Where the Robot Thinks It Is

### Two inputs, one filter

The robot always knows roughly where it is through **wheel odometry** — counting how far each wheel has turned and doing the math. This is accurate short-term but drifts over the course of a match as errors accumulate.

**PhotonVision** gives global position fixes via AprilTag detection. Each accepted camera frame tells the robot exactly where it is on the field — but these come in slower (10–30Hz vs. 50Hz for encoders) and can be noisy.

**`SwerveDrivePoseEstimator`** (a Kalman filter built into WPILib) fuses both. Odometry updates every loop at high rate. Vision measurements are injected with a timestamp — the filter rewinds and re-applies them correctly even if they arrive late. The result is a pose estimate that has the fast update rate of odometry and the global accuracy of vision.

### How vision trust is scaled

Not all vision frames are equally trustworthy. A frame with 3+ tags visible is much more reliable than one with 1 tag. The filter accounts for this:

| Tags visible | Trust level |
|---|---|
| 1 tag | Lower trust (std devs × 2.0) |
| 2 tags | Baseline trust (std devs × 1.0) |
| 3+ tags | Higher trust (std devs × 0.5) |

These multipliers are in `visionConstants.VISION_STD_DEV_TAG_SCALE`.

### 3D pose data

PhotonVision provides full 3D pose (X, Y, Z, roll, pitch, yaw). The Kalman filter only uses 2D (X, Y, heading) since the robot drives on a flat field. However, the Z, pitch, and roll data is preserved and exposed via:

- `visionSubsystem.getBestPoseZ()` — height above floor (useful if robot climbs)
- `visionSubsystem.getBestPosePitch()` — nose-up/down tilt
- `visionSubsystem.getBestPoseRoll()` — side-to-side tilt

### Vision filtering pipeline

Each frame from each camera is checked before being accepted:
1. Must have at least one AprilTag visible
2. Tag count must meet minimum threshold (`visionConstants.MIN_TAGS_FOR_ESTIMATE`)
3. Pose ambiguity must be below threshold (`visionConstants.MAX_AMBIGUITY`)
4. Pose must not jump more than `MAX_POSE_JUMP_M` from last accepted estimate

Frames failing any check are discarded. The filter continues on odometry alone.

### Vision health monitoring

`visionHealthMonitor` runs pre-match checks and ongoing health tracking. During disabled mode, `Robot.disabledPeriodic()` reads the health status and posts it to SmartDashboard so the drive team knows if vision is ready before auto starts.

---

## Camera Hardware

### Camera selection

| Role | Camera | Reason |
|---|---|---|
| AprilTag pose estimation (×2) | **OV9281** | Global shutter — zero rolling shutter distortion on a moving robot. Monochrome sensor is ideal for AprilTag detection (no color processing overhead). Runs up to 120fps for low-latency pose updates. |
| Game piece detection | **OV9782** | Color sensor — required for detecting colored game pieces that AprilTag pipelines can't see. RGB allows ML classifiers to distinguish game pieces from field elements by color and shape. |

### Coprocessor configuration

The robot runs on **two dedicated OrangePi 5 coprocessors** — one per role:

| Coprocessor | Static IP | Cameras | Role |
|---|---|---|---|
| AprilTag OP5 | `10.87.19.11` | `front_cam` (OV9281), `rear_cam` (OV9281) | Robot pose estimation |
| Game piece OP5 | `10.87.19.12` | `intake_cam` (OV9782) | Game piece ML detection |

PhotonVision UI for each: `http://<IP>:5800`

**Why two coprocessors?**

| Reason | Detail |
|---|---|
| Fault isolation | A game piece pipeline crash or frame-rate drop cannot affect AprilTag pose estimation reliability — the two subsystems are physically separate. |
| Resource headroom | Each coprocessor has full CPU and NPU available for its role — no pipeline contention, no priority fighting between AprilTag grayscale pipelines and color ML inference. |
| Scalability | Adding a 2nd game piece camera (e.g. 2nd intake side) stays on the game piece coprocessor; the AprilTag side is untouched. The robot code just adds a 2nd `PhotonCamera` in `gamePieceVisionSubsystem`. |

**`PhotonCamera` and NT transparency:** `PhotonCamera` instances are identified by camera name, not by IP. Both coprocessors connect to the roboRIO as NetworkTables clients. `PhotonCamera("intake_cam")` resolves through NT regardless of which physical device is running it — no robot code structural change is required to split or merge coprocessors.

---

## Game Piece Hunt

### What it does

`gamePieceHuntCommand` autonomously navigates to and collects game pieces detected by the intake-side camera. Four modes are available:

| Mode | `HuntMode` | Behavior |
|---|---|---|
| 1 | `NEAREST_PIECE` | Navigate to the nearest visible piece, trigger intake, stop. |
| 2 | `SEQUENTIAL_PIECES` | Intake nearest piece, then find and intake the next, repeat until none. |
| 3 | `NEAREST_CLUSTER` | Navigate to the centroid of the nearest cluster, intake, stop. |
| 4 | `SEQUENTIAL_CLUSTERS` | Intake nearest cluster, then move to the next, repeat until none. |

Clusters are groups of pieces within `gamePieceConstants.CLUSTER_RADIUS_M` of each other (default 1.5 m). The robot always arrives at a piece with its intake side facing it — PathPlanner delivers it to the computed heading automatically.

### How piece positions are computed

PhotonVision provides pixel-level target bearings (`getYaw()`, `getPitch()`). The code converts these to field coordinates using the camera's pose at the **exact moment the frame was captured** — retrieved via `swerveDrive.getPoseAtTime(timestampSecs)`:

```
verticalAngle = cameraMountPitch − target.getPitch()
horizontalDist = (cameraHeight − PIECE_HEIGHT) / tan(verticalAngle)
bearingField   = cameraYaw + target.getYaw()
pieceX         = cameraX + horizontalDist × cos(bearingField)
pieceY         = cameraY + horizontalDist × sin(bearingField)
```

This compensates for the ~20–60ms camera pipeline latency — the robot may have moved since the frame was captured, and using the current pose instead of the capture-time pose would misplace pieces by several centimeters.

### Pose history ring buffer

`swerveDrive` maintains a 4-entry ring buffer of (timestamp → Pose2d) snapshots, written every loop. At 50Hz this covers ~80ms — enough for typical PhotonVision ML pipeline latency. `getPoseAtTime(t)` linearly interpolates between the two bracketing entries.

This buffer is for game piece back-projection **only** — AprilTag latency compensation is already handled internally by WPILib's `SwerveDrivePoseEstimator` and does not need this buffer.

### State machine

```
FINDING ──► PATHFINDING ──► INTAKING ──► DONE
   ▲                            │
   └────────────────────────────┘  (SEQUENTIAL_* modes only)
```

The command is decoupled from any specific intake subsystem. It takes a `Runnable intakeTrigger` (called on arrival) and a `BooleanSupplier intakeComplete` (polls completion). Wire these in `RobotContainer` once the intake subsystem exists.

### Health monitoring

During disabled mode, `gamePieceVisionSubsystem.publishHealthStatus()` posts to SmartDashboard alongside the AprilTag camera health. Pre-match driver check: place a game piece ~1–2 m in front of the intake and confirm "Piece Detected = true" on the "Vision Health" Shuffleboard tab.

Every loop during a match, four DataLog entries are written: `CameraConnected`, `PieceDetected`, `TrackedPieceCount`, `Debug/MissFrames` — all under `/GamePieceVision/`. These are post-match reviewable in Advantage Scope.

### Extending to a second intake side

A symmetric dual-intake robot (intake on front and rear) requires these changes:
1. **`gamePieceConstants`** — add `GAME_PIECE_CAMERA_2_NAME` and `GAME_PIECE_CAMERA_2_TRANSFORM` for the rear intake camera.
2. **`gamePieceVisionSubsystem`** — add `PhotonCamera intakeCam2`; run the same back-projection loop for both cameras, merging detections into the same `trackedPieces` list (field-relative positions, so both cameras feed the same spatial map).
3. **`gamePieceHuntCommand`** — replace the fixed heading formula with a `findClosestIntakeHeading()` helper that compares the angular distance from the robot's current heading to both intake directions, picks the nearer one, and uses that as the arrival heading.

The state machine, clustering logic, and PathPlanner wiring are all intake-agnostic and require no changes.

---

## PathPlanner Integration

### What it does

When the driver holds a designated button, `pathfindCommand` activates. PathPlanner's AD* algorithm calculates a collision-free path from the robot's current position to the target, executes it, then hands control back to the driver when the button is released. The WPILib command scheduler handles the handoff automatically — `driveWithJoysticks` is interrupted when pathfinding starts and resumed when it ends.

### How it knows where to go

`FieldTargets.java` is a simple lookup table: string key → field position.

```java
FieldTargets.get("speaker")  // returns Pose2d of the speaker scoring position
FieldTargets.get("amp")      // etc.
```

All positions are defined from the blue alliance origin. PathPlanner automatically mirrors them for red alliance.

### Two-phase navigation (in progress)

The current implementation uses `pathfindToPose()` — pure AD* on-the-fly pathfinding. The planned upgrade is `pathfindThenFollowPath()`, which does AD* pathfinding to an approach zone, then executes a pre-planned precise final approach path authored in the PathPlanner GUI. The final approach paths need to be created and tuned per scoring position before this upgrade can happen.

### What PathPlanner needs from the drivetrain

PathPlanner is configured once at startup in `swerveDrive.configureForAutoBuilder()`. It needs four things wired up:
- **Where is the robot?** → `swerveDrive.getPose()` (Kalman-filtered pose)
- **Reset pose** → `swerveDrive.resetPose()` (used at auto start)
- **How fast is it going?** → `swerveDrive.getRobotRelativeSpeeds()` (encoder readout)
- **Make it go this fast** → `swerveDrive.driveRobotRelative(ChassisSpeeds)` (the PathPlanner exception to the driveInput-only rule)

The last one is the only place in the codebase that bypasses `driveInput`. PathPlanner outputs robot-relative speed commands directly — wrapping them in `driveInput` would require an unnecessary coordinate-frame roundtrip. It's intentional, documented in `CLAUDE.md`, and only ever called by PathPlanner's internals.

---

## Mechanism Motor Abstraction (`mechanismUnit`)

### Why it exists

The drivetrain is CTRE every year. Mechanisms are currently REV (SparkMax/SparkFlex + NEO), but the team is actively transitioning to ThriftyBot Nova over the next two seasons — while keeping REV NEO 550 + UltraPlanetary for small/lightweight mechanisms permanently (no Nova equivalent yet). Without an abstraction layer, every subsystem would contain vendor-specific API calls, and swapping a motor controller on a mechanism would require touching subsystem code everywhere.

With `mechanismUnit`, upgrading a mechanism from NEO to Nova is one line change: the `Vendor` enum in its config. The subsystem is untouched.

### The pattern

`mechanismUnit` is an **abstract base class** holding all shared logic once. Vendor subclasses (`CTREMechanismUnit`, `REVMechanismUnit`, `NovaMechanismUnit`) only implement the ~10 hardware calls specific to their controller. A static factory method returns the correct implementation based on the config:

```java
mechanismUnit arm = mechanismUnit.create(
    new mechanismConfig.Builder("Arm",
            new int[]{ 11 },
            new motorConstants.Vendor[]{ motorConstants.Vendor.REV_SPARKMAX })
        .withGearRatio(125.0)
        .withPID(new double[]{ 0.3, 0, 0.01, 0, 0.05, 0 })
        .withSetpointDeadband(1.0)   // ±1°
        .withTier2FF(mechanismUnit.FF.rotatingArm(
            motorModels.NEO, 125.0,
            3.5, 12.0,               // arm: 3.5 lbs, CG 12 in from pivot
            new double[0], new double[0], new Supplier[0]))
        .withMotionProfile(20, 80, 0)
        .build()
);
```

### Follower topology

Followers are configured via `mechanismConfig.Builder`. Two builder methods are available:

**`withFollowMode(mode, invertedArray)`** — convenience for simple topologies where all followers use the same mode and all follow the main leader (canIds[0]):

```java
// Two motors, both MECHANICAL followers of the leader
.withFollowMode(motorConstants.FollowMode.MECHANICAL, new boolean[]{ false, true })
```

**`withFollowerConfig(modes[], inverted[], leaderIndices[])`** — full per-follower control. Each follower independently declares its mode and which motor in canIds[] it tracks. This enables mixed-mode topologies and follower-of-follower chains:

```java
// 4-motor flywheel:
//   canIds[0]=10 (leader A)
//   canIds[1]=11 (B — MECHANICAL to A)
//   canIds[2]=12 (C — ENCODER_SYNC to A, non-rigid coupling on opposite side)
//   canIds[3]=13 (D — MECHANICAL to C, rigidly coupled to C)
new mechanismConfig.Builder("Flywheel",
        new int[]{ 10, 11, 12, 13 },
        new motorConstants.Vendor[]{ REV_SPARKFLEX, REV_SPARKFLEX,
                                     REV_SPARKFLEX, REV_SPARKFLEX })
    .withFollowerConfig(
        new motorConstants.FollowMode[]{
            motorConstants.FollowMode.MECHANICAL,    // B → A
            motorConstants.FollowMode.ENCODER_SYNC,  // C → A
            motorConstants.FollowMode.MECHANICAL },  // D → C
        new boolean[]{ false, false, false },
        new int[]{ 0, 0, 2 })  // B→canIds[0], C→canIds[0], D→canIds[2]
```

`withFollowMode()` is backward-compatible — it fills `followerModes[]` and `followerLeaderIndices[]` uniformly, so existing single-mode configs require no changes.

**Follow mode semantics:**

| Mode | Behavior | Use when |
|---|---|---|
| `MECHANICAL` | Hardware follow — controller mirrors leader output directly, zero Rio CPU after construction | Motors share a rigid mechanical linkage (same shaft, belt, or chain with no slip) |
| `ENCODER_SYNC` | Software follow — Rio reads both encoders each loop, applies proportional duty cycle correction when drift exceeds deadband | Non-rigid coupling where slip is possible and must be detected/corrected |

Nova does not support `ENCODER_SYNC` — its follow API does not expose independent encoder readback on following devices. A construction-time exception is thrown if attempted.

### Units at the public interface

| Value | Unit |
|---|---|
| Position | **degrees** (mechanism shaft, after gear ratio) |
| Velocity | rotations/second (mechanism shaft) |
| Feed-forward | volts |
| Duty cycle | -1.0 to 1.0 |

Internally, positions are converted to rotations (`/360`) before any vendor call. All gear ratio conversion is applied in the vendor layer — subsystems always work in mechanism-shaft degrees/RPS.

### Feed-forward model

Three tiers, each additive:

| Tier | Where it runs | What it covers |
|---|---|---|
| 1 — kS, kV, kA | On the motor controller | Static friction, velocity FF, acceleration FF |
| 2 — lambda in config | Rio (self-contained) | Gravity/spring loads using only this mechanism's own state |
| 3 — lambda in RobotContainer | Rio (cross-subsystem) | Physics depending on other subsystems — variable-mass elevators, drivetrain coupling, etc. |

Tier-2 and tier-3 are `ffProvider` lambdas: `(positionDeg, velocityRps, accelRpss) → volts`. They're called automatically each cycle and summed before injection.

**All physics-based lambdas are built from motor datasheet constants** via `mechanismUnit.FF` factories — no empirical holding-voltage calibration required. The formula used throughout is:

```
V_ff = τ_mechanism × 12V / (gearRatio × motor.stallTorqueNm)
```

Available factories in `mechanismUnit.FF`:

| Factory | Tier | Use case |
|---|---|---|
| `springTurret(motor, gearRatio, points)` | 2 | Piecewise-linear spring/surgical-tubing compensation. Calibration table measured in one direction; reverse rotation auto-negates the torques. Returns 0V outside the calibrated angle range. Deadband prevents direction flip while holding. |
| `rotatingArm(motor, gearRatio, ...)` | 2 | Arm with variable CG (game pieces at different distances). `cos(position)` produces correctly signed output: positive 0°–90°, zero at 90°, negative 90°–180°. |
| `gyroscopicTurret(motor, gearRatio, moiLbIn2, velocitySupplier)` | 3 | Compensates gyroscopic resistance on a turret that rotates a spinning flywheel. τ = I_flywheel × ω_flywheel × ω_turret. Static flywheel MOI provided at construction; flywheel velocity read each cycle via supplier. Pairs additively with `springTurret` on the same mechanism. |
| `multiStageElevator(motor, gearRatio, ...)` | 3 | Multi-stage linear elevator; angle supplied at runtime. Friction offset is deadbanded — holds up-direction bias within ±`FRICTION_DEADBAND_RPS`. |
| `pivotingElevator(motor, gearRatio, ...)` | 3 | Elevator on a pivoting base + full drivetrain inertia/centripetal. Same signed gravity output and friction deadband as above. |

Motor constants live in `motorModels.java` (sourced from vendor datasheets). Pass the appropriate constant as the first argument:

```java
// Rotating arm with game piece CG shift — Tier 2
.withTier2FF(mechanismUnit.FF.rotatingArm(
    motorModels.NEO, 100.0,
    3.5 /*armLbs*/, 12.0 /*armCgIn*/,
    new double[]{ 1.5 } /*pieceLbs*/, new double[]{ 8.0 } /*pieceCgIn*/,
    new Supplier[]{ indexer::getPieceCount }))

// Vertical elevator with game pieces — Tier 3 (built in RobotContainer)
.withTier3FF(mechanismUnit.FF.multiStageElevator(
    motorModels.NEO, 20.0, 0.75 /*spoolIn*/,
    8.0 /*carriageLbs*/, new double[]{ 3.0 } /*stageLbs*/,
    0.05 /*frictionUp*/, -0.02 /*frictionDown*/,
    new double[]{ 1.5 }, new Supplier[]{ indexer::getPieceCount },
    () -> 90.0)) // fixed vertical elevator
```

### What each vendor supports

| Feature | CTRE (TalonFX) | REV (SparkMax/Flex) | Nova |
|---|---|---|---|
| Native FF injection | `.withFeedForward(volts)` | `setSetpoint(..., arbFF, kVoltage)` | `setVelocity/Position(val, volts)` |
| Motion profiling | MotionMagic (trapezoidal + S-curve) | MAXMotion (trapezoidal) | Software step-limiter (Rio) |
| Stator current | ✓ | — (supply current only) | ✓ |
| Hardware duty cycle ramp | ✓ | ✓ | ✓ |
| ENCODER_SYNC following | ✓ | ✓ | ✗ (Nova follow() API blocks independent readback) |
| Per-follower mixed topology | ✓ | ✓ | ✓ (MECHANICAL only) |
| Gear ratio (native) | `SensorToMechanismRatio` | `positionConversionFactor` | Manual scaling in wrapper |

### Live PID tuning

All six gains (kP, kI, kD, kS, kV, kA) are published to SmartDashboard under `[MechanismName]/PID/` at construction. Each loop, `mechanismUnit` reads them back and calls `applyPIDToController()` only if a value changed — minimizing CAN traffic.

### Logging

Each instance writes per-mechanism DataLog entries under `/[MechanismName]/`:

`CommandedVelocity_rps`, `ActualVelocity_rps`, `CommandedPosition_deg`, `ActualPosition_deg`, `DutyCycle`, `AppliedFF_volts`, `SupplyCurrent_A`, `StatorCurrent_A`, `MotorVoltage_V`, `Temp_C`, `ForwardLimit`, `ReverseLimit`

Plus `EncoderSync/Error_rot` and `EncoderSync/Output` when any follower uses ENCODER_SYNC mode.

---

## The Vision Calibration System

The field calibration system lets the team measure and correct per-tag position errors at each competition event:

1. Drive to known positions and observe what the cameras report vs. what WPILib's field layout says
2. The system calculates per-tag offset corrections
3. Output is saved three ways: Python script capture to a laptop file, backup copy to the roboRIO, and clipboard copy
4. Corrections are applied to subsequent pose estimates

This is accessible from the "Field Calibration" tab in Shuffleboard. Run this procedure at each competition event after setup.

---

## Units

The codebase follows a strict convention: **English units at interface boundaries, SI internally.**

When you're reading or writing values:

| If the value is... | You'll see it in... | Converted to... |
|---|---|---|
| Drive speed (sticks, constants) | ft/s | m/s |
| Robot geometry (bumper size, wheel spacing) | inches | meters |
| Field target positions | feet | meters |
| Robot mass | lbs | kg |
| Moment of inertia | lb·in² | kg·m² |
| Mechanism torque (FF calibration) | lb·in | N·m |
| Angles | degrees (at interface) | radians |

All conversion functions live in `util/units.java`. Never convert inline — use those functions.

---

## Robot Lifecycle (every 20ms)

`Robot.robotPeriodic()` runs in this order:
1. **CommandScheduler** — runs subsystem `periodic()` methods + all active commands
2. **updatePoseEstimator()** — injects latest valid vision pose into Kalman filter
3. **updateCalibrationTab()** — refreshes field calibration Shuffleboard tab

Mode-specific behavior:
- **Disabled** — vision health status posted to dashboard
- **Autonomous** — `clearExpectedStartPose()` called on vision, auto command scheduled
- **Teleop** — auto command cancelled, driver takes over
- **Test** — all commands cancelled

---

## Button Map

| Button | Joystick | What it does |
|---|---|---|
| Button 2 | Right | Hold: heading locks to field target while driver still translates |
| Button 3 | Right | Hold: X-lock defense (resist being pushed) |
| Button 4 | Right | Hold: autonomous pathfind to target (currently "speaker") |

---

## Logging

### Two parallel logging systems

| System | Format | Tool | What it captures |
|---|---|---|---|
| Phoenix 6 `SignalLogger` | `.hoot` | AdvantageScope | All TalonFX motor signals — velocity, position, supply/stator current, supply voltage, motor voltage, duty cycle, temperature |
| WPILib `DataLog` via `RobotLogger` | `.wpilog` | AdvantageScope | Computed quantities — robot pose, commanded vs. actual speeds, odometry buckets, vision estimates, pathfinding state, steer encoder voltages |

Both files land on the same USB drive. AdvantageScope opens them simultaneously on a shared timeline.

**AdvantageScope** must be installed on the driver station laptop before the first match. Download from Team 6328's GitHub (search "AdvantageScope FRC"). It is a desktop app — no vendordep, no robot-side installation.

### USB drive requirements

| Requirement | Spec |
|---|---|
| Format | **FAT32** — exFAT is not supported by the roboRIO |
| Speed class | **Class 10 / UHS-I or faster** — slower drives can cause log write delays |
| Capacity | **32GB or larger** — a full competition season generates several GB of logs |
| Label | **`ROBOT_LOG`** recommended — easy to identify in the pit |

### USB drive best practices

- **Insert before power-on.** `SignalLogger` and `RobotLogger` both attempt to open `/u/` at `robotInit()`. A drive inserted after init will not be used until the next reboot.
- **Check SmartDashboard before each match.** `Logger/Storage Path` shows the active path. If it reads `Internal (/home/lvuser/logs/) — INSERT USB`, the drive is missing or unreadable.
- **Eject safely in the pit.** Power down the robot before removing the drive. The roboRIO does not support hot-eject for FAT32.
- **Copy logs immediately after each match.** Don't wait until end of day — a robot reboot with a full internal fallback storage can overwrite earlier logs.
- **One drive per event.** Label drives by event name (e.g. `ROBOT_LOG_DCMP`). Keep previous event drives as backups.
- **Periodically verify drive health.** Run a filesystem check on the drive between events. FAT32 drives can develop errors after repeated hot-unplugs.

### Signal update rates

Drive motor signals (velocity, current, voltage, duty cycle) are logged at **250Hz** for high-fidelity transient capture — wheel slip events and current spikes are visible at this rate. Temperatures are logged at **4Hz** (they change slowly and 250Hz would waste significant CAN bandwidth at full robot scale).

These rates are configured per-subsystem:
- `swerveConstants.SIGNAL_UPDATE_HZ` — 250Hz
- `swerveConstants.SIGNAL_UPDATE_HZ_TEMP` — 4Hz

When adding future mechanisms (intake, shooter, climber), add matching constants to that subsystem's constants file. This lets you tune CAN bandwidth per-mechanism independently.

### CAN bus optimization

`swerveConstants.OPTIMIZE_CAN_UTILIZATION = false` by default. When `false`, all TalonFX signals are broadcast on the CAN bus regardless of whether they are registered — verbose and safe during bring-up. When `true`, `optimizeBusUtilization()` silences all un-registered signals, significantly reducing CAN traffic at full robot scale.

**Flip to `true` per-mechanism only after that mechanism is fully validated on the robot.** Silencing un-registered signals means unexpected data gaps rather than visible errors — not something you want while debugging new hardware.

---

## Shooter Aim System

Three-layer architecture — math, hardware, orchestration — keeps each concern isolated and independently testable.

```
ShooterCalculator  (pure math — no hardware refs)
      │
      │  ShooterOutput: rpm, hood°, turret°, headingBoundsDeg[2]
      ▼
shooterAimController  (SubsystemBase — orchestration)
      │
      ├──► shooterMechanism.setFlywheelRpm() / setHoodDeg() / setTurretDeg()
      └──► driveWithJoysticks.enableHeadingBound() / enableStaticShotLock() / clearAimMode()
```

### `ShooterCalculator`

Pure-math ballistics calculator. No subsystem imports, no hardware access.

**Shot modes:**
- `calculateStaticShot(targetType, tx, ty, rx, ry, headingDeg)` — assumes robot is stopped; iterates for time-of-flight.
- `calculateOnTheMove(targetType, tx, ty, robotState...)` — adds robot velocity vector to ball velocity for moving shots.

**`ShooterOutput` fields of note:**
- `double[] headingBoundsDeg` — `[normalize(aimFieldDeg − softMax), normalize(aimFieldDeg − softMin)]`. The window of robot headings where the turret can reach the aim direction without leaving its soft limits.
- `isHeadingInBounds(double robotHdgDeg)` — wrap-aware check using `isInArc()`, correctly handles windows that cross ±180° (e.g., bounds = [150°, −160°]).

**Heading bounds derivation:** turretAngle = aimFieldDeg − robotHeadingDeg. Rearranging: robotHeadingDeg ∈ [aimFieldDeg − softMax, aimFieldDeg − softMin]. For swerve-as-turret pass softMin = softMax = 0 → bounds collapse to [aimFieldDeg, aimFieldDeg] (tight lock).

### `shooterAimController`

Reads drivetrain state → calls `ShooterCalculator` → pushes setpoints to `shooterMechanism` → sets drive mode on `driveWithJoysticks`. Runs in `periodic()`.

**Aim modes (set by button bindings):**
- `STATIC_SHOT` — two-phase: Phase 1 rotates robot into heading window (`enableHeadingBound`), Phase 2 X-locks once in bounds (`enableStaticShotLock`). Swerve-as-turret variant uses `enablePointAt` for Phase 1.
- `ON_THE_MOVE` — calls `enableHeadingBound(min, max)` each loop; driver retains full translation.
- `IDLE` — `clearAimMode()`, no setpoints sent.

**`TurretBoundState` machine (OTM + physical turret):**

When the robot drifts outside the heading window during on-the-move shooting, the turret can't reach the target — it must wait at its soft limit or commit to the other side.

| State | Turret parks at | Flip to opposite limit when |
|---|---|---|
| `IN_BOUNDS` | Normal aim | Heading crosses soft limit boundary |
| `WAITING_AT_FORWARD_LIMIT` | `softMax` | Overshoot > `deadZone × flipThresholdPct` from forward limit |
| `WAITING_AT_REVERSE_LIMIT` | `softMin` | Overshoot > `deadZone × flipThresholdPct` from reverse limit |

Dead zone = `360° − (softMax − softMin)`. Default `flipThresholdPct` = 0.5 (break-even midpoint — equally fast to rotate back or forward to re-enter the window). Overshoot is computed wrap-correctly via `units.normalizeAngleDeg(boundEdge − robotHdgDeg)`.

### `shooterMechanism` — `readyGate`

The optional `withReadyGate(BooleanSupplier gate)` builder method ANDs an external condition onto `isReady()`. Wire it to `aimCtrl.getLastResult().isHeadingInBounds()` so the robot won't attempt to fire when the heading is outside the turret window — even if flywheel and hood are at setpoint.

### `shooterConstants`

Keeps the lookup table and geometry in a single visible place for in-season tuning:
- `LookupEntry[]` LOOKUP_TABLE — all shot profiles (distance → RPM, hood angle, etc.)
- `PIVOT_OFFSET_X/Y_IN` — shooter pivot position in robot frame (inches)
- `ROBOT_SPEED_WARNING_THRESHOLD_FT_S` — OTM speed-too-high warning threshold

Turret soft/hard limits are **not** in `shooterConstants` — they live in `mechanismConfig` (passed to `shooterMechanism`) to avoid duplication. `ShooterCalculator` reads them from `mechanismConfig` via the convenience constructor.

---

## What's Still TODO

These are known gaps — in priority order for competition readiness:

1. **Weigh the robot** → update `ROBOT_MASS_LBS` and `ROBOT_MOI_LBIN2` in `pathplannerConstants.java`
2. **Measure steer offset voltages** → deploy with offsets = 0, read SmartDashboard, update `swerveConstants.java`
3. **Confirm CAN IDs** → verify `swerveConstants.java` matches physical wiring
4. **Measure camera mounting positions** → update `visionConstants.FRONT_CAMERA_TRANSFORM` and `REAR_CAMERA_TRANSFORM`
5. **Populate 2026 field targets** → update `FieldTargets.java` when field geometry is published
6. **Tune PathPlanner PIDs** → `pathplannerConstants.TRANSLATION_PID_kP/kD` and `ROTATION_PID_kP/kD` on actual carpet
7. **Author PathPlanner final approach paths** → in PathPlanner GUI, then upgrade `pathfindCommand` to `pathfindThenFollowPath()`
8. **Add alliance zone exclusions** → `pathplannerConstants.ALLIANCE_ZONE_EXCLUSIONS` when 2026 field zones are published
9. **Add target selector** → replace hardcoded `"speaker"` in `RobotContainer` with a dashboard chooser or multi-button map
10. **Update PhotonVision API** → `visionSubsystem.java` uses deprecated `getLatestResult()` and `update()` — update when PhotonVision publishes new API

---

*Last updated: 2026 preseason — Ryan + Claude (Code) | mechanismUnit: per-follower mixed topology (withFollowerConfig), follower-of-follower leader indices; FF library: gyroscopicTurret, springTurret direction-aware, friction deadbanding, signed gravity output; shooterMechanism pre-packaged subsystem; setpointDeadband + isAtSetpoint on mechanismUnit; shooter aim system: ShooterCalculator + shooterAimController + shooterConstants, headingBoundsDeg, TurretBoundState OTM machine, readyGate; IMU changed to Pigeon 2.0; driveWithJoysticks DriveMode enum + HEADING_BOUND + STATIC_SHOT_LOCK modes; driveWithHeadingBound() on swerveDrive; units.normalizeAngleDeg()*
