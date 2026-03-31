# CLAUDE.md — FRC Team 8719 Robot Codebase Context

This file provides persistent context for AI-assisted development on the team 8719 robot codebase.
Place this file in the project root so Claude Code picks it up automatically.

---

## Project Overview

This is a from-scratch swerve drive robot codebase for **FRC Team 8719**, built using:
- **WPILib 2026** (Java)
- **Phoenix 6** vendor library (CTRE motors/sensors)
- **PhotonVision** for vision processing
- **VSCode** as the development environment

---

## Hardware

### Drivetrain
- **Swerve modules:** Thrifty Narrow swerve modules
- **Drive motors:** Kraken X60 (CTRE, controlled via Phoenix 6)
- **Steering motors:** Minion (CTRE)
- **Absolute encoders:** Analog absolute encoders (for swerve module steering)
- **IMU:** ADIS16470

### Vision
- **Cameras:** Dual PhotonVision cameras
- **Coprocessor:** OrangePi 5
- **Use:** AprilTag pose estimation

---

## Codebase Architecture

### Key Classes

| Class | Role |
|---|---|
| `swerveModule` | Low-level control of a single swerve module (drive + steer) |
| `swerveDrive` | Manages all four modules; exposes drive interface |
| `driveInput` | **Sole public interface to `swerveDrive`** — all drive commands go through here |
| `driveOdometryState` | Tracks robot pose via wheel odometry |
| `visionSubsystem` | **Sole public interface to all vision internals** |
| `units.java` | Unit conversion constants and helpers |

### Architectural Principles

1. **`driveInput` is the only way in.** No subsystem or command should call `swerveDrive` directly — all drive commands are routed through `driveInput`. This enforces a clean separation between input handling and drive execution.

   **Exception — PathPlanner:** `swerveDrive.driveRobotRelative(ChassisSpeeds)` is an intentional exception. PathPlanner outputs robot-relative `ChassisSpeeds` directly; routing through `driveInput` would require a coordinate-frame roundtrip for no benefit. This method is called exclusively via the `AutoBuilder` lambda registered in `swerveDrive.configureForAutoBuilder()` — never called directly by any command or subsystem.

2. **`visionSubsystem` is the only vision interface.** All camera access, pose estimation, and tag reading is encapsulated behind `visionSubsystem`. Nothing outside it should touch PhotonVision internals directly.

3. **English units at interface boundaries, SI internally.** User-facing and robot-configuration values (speeds in ft/s, distances in inches, etc.) are expressed in English units at the interface. Internal calculations use SI units. Conversion is handled via `units.java`.

4. **Single `GAME_YEAR_FIELD` constant.** The field geometry constant is defined once and propagates throughout the codebase. Never hardcode field dimensions elsewhere.

---

## Naming Conventions

- **Subsystem classes that are robot-specific:** `camelCase` (e.g., `swerveDrive`, `swerveModule`, `driveInput`)
- **Classes prefixed with a proper noun:** Uppercase-first (e.g., `PhotonVisionCamera`, `ADISGyro` — follow the proper noun's own capitalization)
- **Constants:** `UPPER_SNAKE_CASE`
- **Methods and variables:** `camelCase`

---

## Joystick & Command Mapping

- Joystick commands are mapped with **dynamic center of rotation**, bounded to the bumper corners of the robot. This allows the driver to shift the rotation pivot during maneuvers.
- **X-lock defense** is implemented — commands the modules to an X pattern to resist being pushed.
- **Point-at-target heading lock** is implemented via PID — holds a specific field-relative heading while allowing translational control.

---

## Vision System

- Dual cameras provide overlapping AprilTag coverage.
- Pose estimates from both cameras are fused into the odometry state.
- The `buildTagReading()` method currently uses a **simplified delta calculation** for tag pose offset. This is a known approximation — full tag reprojection geometry is a noted future refinement.
- A **field calibration system** is implemented, outputting offset files via three redundant paths:
  1. Python script output to laptop
  2. Backup copy to roboRIO
  3. Clipboard copy

---

## WPILib 2026 Notes

- `CommandBase` has been removed in WPILib 2026. All commands extend `Command` directly.
- Phoenix 6 vendor library is installed and required for Kraken X60 and Minion motor control.

---

## Development Conventions

- **Explicit "let's write the code" signal** before implementation begins — used to separate design discussion from active coding sessions.
- Prefer discussing architecture and tradeoffs first; only move to code when the approach is agreed upon.
- When in doubt about a design decision, check this file and existing class interfaces for established patterns before introducing new ones.

---

## Known Future Work / Open Items

- `buildTagReading()` — replace simplified delta calculation with full tag reprojection geometry for more accurate vision pose deltas.
- Path planning integration (in progress — reason this file exists).

---

*This file was generated to carry forward context from a web-based Claude conversation into Claude Code. Update it as the codebase evolves.*