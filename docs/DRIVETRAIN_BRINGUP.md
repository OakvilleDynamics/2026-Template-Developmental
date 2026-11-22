# Drivetrain Bring-Up and Calibration

Goal: calibrate the swerve drivetrain and confirm it drives correctly with both the Xbox controller and the two-flight-stick layout.

Hardware: Kraken X60 drive (TalonFX), Minion steer on Thrifty Nova, Thrifty analog absolute encoders, Pigeon 2.

Future drivetrain software changes are tracked in `TODOs.md`.

---

## Part 1: Hardware and config checks (before deploying)

- [ ] **Phoenix Tuner X:** Krakens 2/4/6/8 and Pigeon 1 are on the roboRIO CAN bus (not a CANivore), firmware is current, and you know the license status.
- [ ] **Phoenix Pro license / FOC:** `CTREMotor` turns on FOC for every control request, and FOC needs a Phoenix Pro license. If the Krakens are unlicensed, look for an `UnlicensedFeatureInUse` fault and turn FOC off.
- [ ] **Thrifty app:** Novas 3/5/7/9 match the corner they're on, firmware is current, and motor type is Minion.
- [x] **Gear ratios:** 5.79:1 drive (installed pinion) and 24:1 steer, confirmed from last season's code. Still confirm the wheels are 4".
- [x] **Wheelbase:** measure pivot-to-pivot center distance and confirm 22" × 22" (`WHEEL_BASE_IN` in `RobotContainer`).
- [ ] **Analog wiring:** encoders go FL→AI0, FR→AI1, BL→AI2, BR→AI3.
- [ ] **Driver Station:** the Xbox controller, or the left flight stick, is on USB port 0, and the right stick is on port 1. The code doesn't detect the controller type: set `DRIVER_CONTROLLER` in `RobotContainer` and redeploy to switch.

---

## Part 2: Calibration and validation at the robot

Keep the robot on blocks with the wheels off the ground until step 7. Have someone on the E-stop and use a charged battery (over 12.5 V).

Right flight-stick buttons 4–8 start pathfinding and game-piece hunting, which need cameras. Don't press them during bring-up. Vision will log missing-camera warnings; that's harmless.

- [ ] **1. Deploy and look at the startup output.** Read the ConfigVerifier results in the Driver Station console: every module's drive and steer should pass with no mismatches. Also check for Alerts and the "PathPlanner unavailable" message.
- [ ] **2. Absolute encoder range.** Disabled, turn each wheel through a full turn slowly and watch `Swerve/XX/Raw Volts` and `Swerve/XX/Raw Rot`. Volts should run from about 0 to about 5 V and Raw Rot from 0 to 1. If volts top out near 3.3 V, check the encoder's 3.3 V jumper. It should be in the default 5 V position.
- [ ] **3. Directions and encoder sync, still disabled.** Viewed from above:
  - Turning a wheel counter-clockwise should increase both `Abs Enc Rot` and `Angle Deg`. If one goes the wrong way, flip `ABS_ENCODER_INVERTED` or `STEER_MOTOR_INVERTED` in `SwerveConstants`.
  - Turning the whole robot counter-clockwise should increase `Drive/Heading (deg)`. If it decreases, the gyro sign in `SwerveDrive.getYaw()` is wrong.
  - **Steer sync: the internal encoder matches the absolute encoder.** Turn each wheel slowly by hand to a few angles and stop. `Angle Deg` (Nova internal encoder) should equal `Abs Enc Rot` × 360 within about 2°, after wrapping both to ±180°. `Steer Resyncs` should go up once or twice at boot and then stay flat. If `Angle Deg` barely moves, reads about 1/42 of the expected angle, or `Steer Resyncs` climbs every loop, `NovaMotor.setEncoderPosition` isn't sending ticks. On ThriftyLib 2026.1.2 it must pass motor rotations × 42 (internal ticks/rev). This applies only to 2026.1.2: 2026.3.2 `setIntPosition` takes rotations.
- [ ] **4. Steer offsets.**
  - Point every wheel straight forward with all bevel gears facing the same side, e.g. the robot's left. Using the same side for all four means all drive inversions should end up the same.
  - Record `Raw Rot` for each module into `XX_STEER_OFFSET_ROT` in `SwerveConstants` and redeploy. `Raw Rot` ignores the current offsets, so there's no need to zero them first. The starting values are last season's offsets (ticks ÷ 4096), so compare against them.
  - Restart robot code and check that `Angle Deg` reads about 0 on all four. Repeat after a full power cycle. `Swerve/XX/Steer Resyncs` should settle at a small number while disabled; a count that keeps climbing means a noisy encoder or a Nova not accepting the new position.

  | Module | Last season (rot) | Measured `Raw Rot` |
  |---|---|---|
  | FL | 0.7612 (3118) | |
  | FR | 0.6472 (2651) | |
  | BL | 0.2280 (934) | |
  | BR | 0.9360 (3834) | |

- [ ] **5. Steer tuning, enabled on blocks.**
  - Give small rotate inputs; the wheels should form a rotation pattern. X-lock (right stick button 3) should give FL 45°, FR −45°, BL −45°, BR 45°.
  - Raise steer kP from about 0.1 until the wheels snap to angle without oscillating, then add kD if needed. The values are live-tunable under `Swerve/XX/PID/`. Nova gains are per motor rotation (before the 24:1 steer ratio); steer kS should stay 0, since Nova applies it as a setpoint feedforward, not static friction.
- [ ] **6. Drive direction and feedforward, on blocks.**
  - Push forward gently: all four wheels should spin "forward". If a wheel spins backward, flip its `XX_DRIVE_INVERTED` in `SwerveConstants`. Adding half a rotation to its offset (about 1.65 V) also works.
  - Hold a steady half-speed input and compare `Speed MPS` with `Desired Speed MPS`. Adjust kV (starts at 0.7, in wheel rotations per second) until they match, then set kS to the smallest value that just starts the wheel moving. Add kP last.
- [ ] **7. On the ground, at the reduced speed limit.**
  - Zero the heading with the robot facing away from you (Xbox Start, or left flight stick button 2).
  - Test forward/back, strafe, and rotate in place.
  - Field-relative check: turn the robot 90° and push forward; it should still drive away from you.
  - Finally, translate while rotating.
- [ ] **8. Flight-stick mode tests.**
  - Run the step 7 tests again.
  - Center of rotation: deflecting the right stick should pivot the robot about that bumper corner.
  - X-lock (right stick button 3) should hold position when pushed.
  - Skip lock-to-target (right stick button 2) for now, since it needs a reliable pose.
- [ ] **9. Xbox mode tests.** Switch `DRIVER_CONTROLLER` to `XBOX` in `RobotContainer`, redeploy, and repeat step 7. Check that the right stick turns the robot the same direction you push it.
- [ ] **10. Raise speed and calibrate.**
  - Push the robot exactly 10 ft by hand: `Drive/Pose X (ft)` should read 10. This checks gear ratio and wheel diameter together; adjust `WHEEL_DIAMETER_INCHES` for tread wear.
  - Raise `Drive/Teleop Speed Scale` and `Drive/Teleop Rotation Scale` on the dashboard in steps (no redeploy). At 1.0, measure the real top speed and set `MAX_DRIVE_SPEED_FPS` to about 90% of it. Then set `TELEOP_SPEED_SCALE` and `TELEOP_ROTATION_SCALE` to 1.0 in `SwerveConstants`.
  - Tune each controller's deadband (`INPUT_DEADBAND`); flight-stick twist often drifts more than Xbox sticks.

---

## Values you can only get at the robot

| Value | Where it goes | Measured |
|---|---|---|
| Steer offsets ×4 | `SwerveConstants.XX_STEER_OFFSET_ROT` | |
| Drive inversions ×4 | `SwerveConstants.XX_DRIVE_INVERTED` | |
| Steer motor / encoder inversion | `SwerveConstants.STEER_MOTOR_INVERTED` / `ABS_ENCODER_INVERTED` | |
| Gyro sign (confirm only) | `SwerveDrive.getYaw()` | |
| Steer kP / kD | `RobotContainer.XX_STEER_PID` | |
| Drive kS / kV / kP | `RobotContainer.XX_DRIVE_PID` | |
| Effective wheel diameter | `SwerveConstants.WHEEL_DIAMETER_INCHES` | |
| Top speed | `SwerveConstants.MAX_DRIVE_SPEED_FPS` | |
| Deadband per controller | `SwerveConstants.INPUT_DEADBAND` | |
| Heading PID (later, for lock-to-target) | `SwerveConstants.HEADING_PID_*` | |
