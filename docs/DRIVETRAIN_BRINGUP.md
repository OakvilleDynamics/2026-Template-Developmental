# Drivetrain Bring-Up Plan

Goal for this week: drive the robot with the Xbox controller and the two-flight-stick layout.

Hardware: Kraken X60 drive (TalonFX), Minion steer on Thrifty Nova, Thrifty analog absolute encoders, Pigeon 2.

---

## Part 1: Code fixes before the robot

### Must fix: the robot will misbehave without these

- [x] **1. Gyro sign is probably backwards.** `SwerveDrive.getYaw()` negates the Pigeon yaw. The comment there says the Pigeon is already counter-clockwise-positive, which is what WPILib expects. With the negation, field-relative driving gets mirrored as soon as the robot turns. Remove the negation, then confirm on the robot (Part 3, step 3).
- [x] **2. The Nova steer controllers are never told they drive a Minion.** `SwerveHardware.steerMotorConfig` never calls `.withNovaMotorType(MINION)`. Each Nova uses whatever motor type is saved on it, and `NovaMotor.verifyConfig` skips the motor-type check when none is set.
- [x] **3. The steer gains are TalonFX values but run on a Nova.** Steer PID is `{60, 0, 2, 0.25, 0.12}`.
  - Nova gains are per motor rotation (ThriftyLib `PIDConfig.translate`), and `setPosition` runs before the 24:1 steer gear ratio. A TalonFX kP of 60 (volts per azimuth rotation) works out to roughly **kP ≈ 0.2** on the Nova, assuming its output is a duty fraction. **Start at about 0.1** and raise it.
  - `kS` is passed to Nova's `setFF()`, which is a feedforward on the setpoint, not a static-friction term. **Set steer kS to 0** for bring-up. Nova ignores `kV`.
- [x] **4. Drive kV is about 6× too low.** CTRE runs with `SensorToMechanismRatio = 5.79`, so velocities are in wheel rotations per second, not motor rotations. A Kraken's free speed is about 100 motor rot/s, which is about 17.3 wheel rot/s, so **kV ≈ 0.7**, not 0.12.
- [x] **5. No way to zero the heading.** `SwerveDrive.zeroYaw()` exists but isn't bound to any button. Bind it on both controllers, for example Xbox Start and a flight-stick button.

### Should add: makes testing on the robot much faster

- [x] **6. Add inversion settings to `SwerveConstants`:**
  - per-module drive inversion (replaces the hardcoded `true/false/true/false` in `RobotContainer`, which has a TODO)
  - steer motor inversion (currently hardcoded in `SwerveHardware`)
  - absolute encoder inversion (`ThriftyAnalogEncoder` has no invert option)
- [x] **7. Publish the commanded angle and speed for each module** (e.g. `Swerve/FL/Desired Angle Deg`, `Swerve/FL/Desired Speed MPS`) so commanded can be compared with actual.
- [x] **8. Add a bring-up speed limit.** Done as live-tunable `Drive/Teleop Speed Scale` (0.3) and `Drive/Teleop Rotation Scale` (0.5), defaults in `SwerveConstants.TELEOP_*_SCALE`.
- [x] **9. Re-seed steer angles while disabled.** The steer encoders are seeded from the absolute encoders once, in the `SwerveModule` constructor. If a Nova hasn't finished booting when the code starts, that seed is lost and the module is off by some angle. Done: `SwerveDrive.periodic()` calls `SwerveModule.syncSteerToAbsolute()` while disabled (2° threshold).

### Later

- [ ] **Choose the motor a Nova steer drives.** `SwerveHardware.steerMotorConfig` assumes a `NOVA` steer drives a Minion. Let the robot pick NEO or Minion, e.g. split `SteerMotorType.NOVA` into `NOVA_MINION` / `NOVA_NEO`. (TODO in `SwerveHardware`.)

- [ ] **Consider wiring the encoders to the Novas.** The Nova has an absolute-encoder mode (`EncoderType.ABS`, `setAbsOffset`, `setPositionAbs`). It would steer directly from the absolute encoder with no startup seeding. That needs rewiring all four modules and confirming the Thrifty encoder works on the Nova's port.
  - **Code needed:** `NovaMotor` always calls `useEncoderType(EncoderType.INTERNAL)`, and neither `MotorIOConfig` nor `IMotorIO` can select a different feedback sensor. Add a vendor-neutral feedback setting to `MotorIOConfig` (e.g. internal vs. absolute, plus the sensor-to-mechanism ratio). With ABS feedback on the azimuth shaft, `NovaMotor` must not apply the 24:1 gear ratio to setpoints and readback, `getVelocity()` returns 0, wrapping uses `setAbsoluteWrapping`, and steer gains need re-tuning (error is in azimuth rotations, not motor rotations).

- [ ] **Choose Xbox or flight sticks automatically.** `DriverStation.getJoystickIsXbox(0)` isn't reliable when `RobotContainer` is built, because the Driver Station often isn't connected yet at boot. Detecting it later (e.g. at enable) means rebuilding the drive command and button bindings then. For now, switch `DRIVER_CONTROLLER` in `RobotContainer` and redeploy.

### Doesn't block driving, but be aware

- Flight-stick buttons 4–8 start pathfinding and game-piece hunting, which need cameras. Don't press them, or leave them unbound for now. Vision will log missing-camera warnings; that's harmless.
- `CTREMotor` turns on FOC for every control request. FOC needs a Phoenix Pro license, so check the Krakens' license in Tuner X. If they're unlicensed, look for an `UnlicensedFeatureInUse` fault and turn FOC off.

---

## Part 2: Hardware and config checks (before deploying)

- [ ] **Phoenix Tuner X:** Krakens 2/4/6/8 and Pigeon 1 are on the roboRIO CAN bus (not a CANivore), firmware is current, and you know the license status.
- [ ] **Thrifty app:** Novas 3/5/7/9 match the corner they're on, firmware is current, and motor type is Minion.
- [x] **Gear ratios:** 5.79:1 drive (installed pinion) and 24:1 steer, confirmed from last season's code. Still confirm the wheels are 4".
- [X] **Wheelbase:** measure pivot-to-pivot center distance and confirm 22" × 22" (`WHEEL_BASE_IN` in `RobotContainer`).
- [ ] **Analog wiring:** encoders go FL→AI0, FR→AI1, BL→AI2, BR→AI3.
- [ ] **Driver Station:** the Xbox controller, or the left flight stick, is on USB port 0, and the right stick is on port 1.

---

## Part 3: Testing order at the robot

Keep the robot on blocks with the wheels off the ground until step 7. Have someone on the E-stop and use a charged battery (over 12.5 V).

- [ ] **1. Deploy and look at the startup output.** Read the ConfigVerifier results in the Driver Station console: every module's drive and steer should pass with no mismatches. Also check for Alerts and the "PathPlanner unavailable" message.
- [ ] **2. Absolute encoder range.** Disabled, turn each wheel through a full turn slowly and watch `Swerve/XX/Raw Volts` and `Swerve/XX/Raw Rot`. Volts should run from about 0 to about 5 V and Raw Rot from 0 to 1. If volts top out near 3.3 V, check the encoder's 3.3 V jumper. It should be in the default 5 V position.
- [ ] **3. Directions, still disabled.** Viewed from above:
  - Turning a wheel counter-clockwise should increase both `Abs Enc Rot` and `Angle Deg`. If one goes the wrong way, flip the encoder or steer inversion (Part 1, item 6).
  - Turning the whole robot counter-clockwise should increase `Drive/Heading (deg)`. This confirms the gyro sign (Part 1, item 1).
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
  - Raise steer kP from about 0.1 until the wheels snap to angle without oscillating, then add kD if needed. The values are live-tunable under `Swerve/XX/PID/`.
- [ ] **6. Drive direction and feedforward, on blocks.**
  - Push forward gently: all four wheels should spin "forward". If a wheel spins backward, flip its drive inversion. Adding half a rotation to its offset (about 1.65 V) also works.
  - Hold a steady half-speed input and compare `Speed MPS` with `Desired Speed MPS`. Adjust kV until they match, then set kS to the smallest value that just starts the wheel moving. Add kP last.
- [ ] **7. On the ground, at the reduced speed limit.**
  - Zero the heading with the robot facing away from you.
  - Test forward/back, strafe, and rotate in place.
  - Field-relative check: turn the robot 90° and push forward; it should still drive away from you.
  - Finally, translate while rotating.
- [ ] **8. Flight-stick mode tests.**
  - Run the step 7 tests again.
  - Center of rotation: deflecting the right stick should pivot the robot about that bumper corner.
  - X-lock (button 3) should hold position when pushed.
  - Skip lock-to-target (button 2) for now, since it needs a reliable pose.
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
| Drive inversions ×4 | `SwerveConstants` (after Part 1, item 6) | |
| Steer motor / encoder inversion | `SwerveConstants` (after Part 1, item 6) | |
| Gyro sign | `SwerveDrive.getYaw()` | |
| Steer kP / kD | `RobotContainer.XX_STEER_PID` | |
| Drive kS / kV / kP | `RobotContainer.XX_DRIVE_PID` | |
| Effective wheel diameter | `SwerveConstants.WHEEL_DIAMETER_INCHES` | |
| Top speed | `SwerveConstants.MAX_DRIVE_SPEED_FPS` | |
| Deadband per controller | `SwerveConstants.INPUT_DEADBAND` | |
| Heading PID (later, for lock-to-target) | `SwerveConstants.HEADING_PID_*` | |
