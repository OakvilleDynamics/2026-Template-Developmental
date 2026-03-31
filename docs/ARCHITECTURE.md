# FRC Team 8719 — Robot Codebase Architecture
### 2026 Season | Swerve Drive + Vision + PathPlanner

---

## What This Document Is

A guide to how the robot code is organized, what each piece does, and how they connect. Written for team members who need to understand, modify, or extend the code — not just run it.

---

## Big Picture

The robot is a **swerve drive** controlled by two flight sticks. It tracks its position on the field using a combination of **wheel encoders** and **AprilTag cameras**, fused together by a **Kalman filter**. A driver can press a button to trigger **autonomous pathfinding** to a target location on the field — the robot navigates there on its own and hands control back when the button is released.

```
Driver sticks ──► driveWithJoysticks ──► driveInput ──► swerveDrive ──► 4× swerveModule
                                                              ▲
PathPlanner ──────────────────────────────────────────────────┘
                                                              ▲
                                              SwerveDrivePoseEstimator
                                               ▲                  ▲
                                         wheel encoders      PhotonVision
                                           + IMU              (AprilTags)
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
│   ├── swerveConstants     ← CAN IDs, gear ratios, speed limits, PID defaults
│   ├── visionConstants     ← Camera names, transforms, filter thresholds
│   ├── pathplannerConstants ← PathPlanner config, robot mass/MOI, constraints
│   └── AprilTagIgnore      ← Tag IDs to suppress at specific events
│
├── pathplanning/           ← PathPlanner integration
│   ├── FieldTargets        ← "speaker" → field coordinate lookup table
│   └── pathfindCommand     ← Navigate to target on button press
│
├── subsystems/
│   ├── swerveDrive/        ← Drivetrain
│   │   ├── swerveDrive     ← Manages all 4 modules + pose estimator
│   │   ├── swerveModule    ← One wheel/motor pair
│   │   ├── driveInput      ← Data object: vx, vy, omega, center of rotation
│   │   └── driveOdometryState ← Motion state snapshot (velocity, accel, etc.)
│   └── vision/             ← Vision
│       ├── visionSubsystem ← Public interface: pose estimates, tag data
│       ├── robotPoseEstimate ← One camera's pose estimate
│       ├── visionHealthMonitor ← Pre-match checks, ongoing health tracking
│       └── AprilTagFieldCalTab ← Shuffleboard calibration interface
│
└── util/
    ├── units.java          ← All unit conversions (inches↔m, lbs↔kg, etc.)
    └── AprilTagFieldCal    ← Field layout, per-tag offsets, geometry
```

---

## The Drivetrain

### How driving works

The driver moves the left stick to translate (move forward/back/strafe) and twists the right stick to rotate. These inputs go into `driveWithJoysticks`, which reads the joystick axes, applies deadband and input shaping, then builds a `driveInput` object and hands it to `swerveDrive`.

**Field-relative control** means "push stick forward" always moves the robot toward the far field wall — regardless of which way the robot is facing. The code rotates joystick inputs by the robot's current heading before sending them to the wheels.

**Dynamic center of rotation** — the right stick X/Y axis shifts the pivot point of rotation anywhere within the robot's bumper footprint. Centered stick = pivot in place. Full deflection = pivot at a bumper corner. This lets the driver swing around a game piece or defense target.

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
| `imuState` | ADIS16470 IMU | Angular velocity, heading rate |
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

*Last updated: 2026 preseason — Ryan + Claude (Web + Code)*
