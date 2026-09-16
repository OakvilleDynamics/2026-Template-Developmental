package frc.robot.constants;

import frc.robot.util.units;

/**
 * swerveConstants.java
 * PATH: src/main/java/frc/robot/constants/swerveConstants.java
 *
 * Hardware-fixed constants that don't change between modules or at runtime.
 *
 * ─── WHAT LIVES HERE ─────────────────────────────────────────────────────────
 * CAN IDs, analog ports, steer offsets, gear ratios, speed limits,
 * heading PID defaults, blend alpha default.
 *
 * ─── WHAT DOES NOT LIVE HERE ─────────────────────────────────────────────────
 * Per-module PID values → RobotContainer (passed into each swerveModule constructor)
 * Robot geometry → RobotContainer (passed into swerveDrive constructor)
 */
public final class swerveConstants {

    // ── Motor / encoder type selection ────────────────────────────────────────
    /**
     * Select the drive and steer motor types and absolute encoder type for this
     * robot build. All four modules always use the same hardware. Change these
     * three constants when swapping hardware — no other code changes required.
     */
    public enum DriveMotorType  { KRAKEN_X60, KRAKEN_X44, NEO_VORTEX, NEO, NOVA_PULSAR }
    public enum SteerMotorType  { KRAKEN_X60, KRAKEN_X44, MINION, NOVA, NEO_VORTEX, NEO, NEO_550 }
    public enum AbsoluteEncoderType { THRIFTY_ANALOG, CTRE_CANCODER, REV_THROUGH_BORE }

    public static final DriveMotorType      DRIVE_MOTOR_TYPE  = DriveMotorType.KRAKEN_X60;
    public static final SteerMotorType      STEER_MOTOR_TYPE  = SteerMotorType.MINION;
    public static final AbsoluteEncoderType ABS_ENCODER_TYPE  = AbsoluteEncoderType.THRIFTY_ANALOG;



    // ── Wheel ─────────────────────────────────────────────────────────────────
    /**
     * Drive wheel diameter in INCHES.
     * Used only in swerveModule for unit conversion to meters.
     * Thrifty Narrow ships with a 4" billet wheel.
     * TODO: confirm with your actual wheel.
     */
    public static final double WHEEL_DIAMETER_INCHES = 4.0;

    // ── Gear ratios ───────────────────────────────────────────────────────────
    /**
     * Drive motor rotations per wheel rotation.
     * Thrifty Narrow: L1=8.14, L2=6.75, L3=6.12
     * TODO: confirm your module order.
     */
    public static final double DRIVE_GEAR_RATIO = 6.75;

    /** Minion motor rotations per module azimuth rotation. Thrifty Narrow = 12.8 */
    public static final double STEER_GEAR_RATIO = 12.8;

    // ── Speed limits ──────────────────────────────────────────────────────────
    /**
     * Maximum linear drive speed in ft/s (driver-facing unit).
     * Stored in both ft/s and m/s — m/s used by kinematics, ft/s used by driveInput scaling.
     * Kraken X60 at L2 theoretical max ≈ 15.5 ft/s. Cap at ~90% for control headroom.
     */
    public static final double MAX_DRIVE_SPEED_FPS = 14.0;
    public static final double MAX_DRIVE_SPEED_MPS = units.ftps_mps(MAX_DRIVE_SPEED_FPS);

    /** Maximum angular speed (rad/s). ~1 full rotation per second. */
    public static final double MAX_ANGULAR_SPEED_RPS = 2.0 * Math.PI;

    // ── CAN IDs ───────────────────────────────────────────────────────────────
    // TODO: update to match your robot's actual wiring
    public static final int FL_DRIVE_CAN_ID = 1;
    public static final int FL_STEER_CAN_ID = 2;
    public static final int FR_DRIVE_CAN_ID = 3;
    public static final int FR_STEER_CAN_ID = 4;
    public static final int BL_DRIVE_CAN_ID = 5;
    public static final int BL_STEER_CAN_ID = 6;
    public static final int BR_DRIVE_CAN_ID = 7;
    public static final int BR_STEER_CAN_ID = 8;

    // ── IMU ───────────────────────────────────────────────────────────────────
    /**
     * CAN ID of the CTRE Pigeon 2.0 IMU.
     * TODO: confirm this matches the physical CAN bus wiring.
     */
    public static final int PIGEON2_CAN_ID  = 9;

    // ── Analog encoder ports (roboRIO) ────────────────────────────────────────
    // Used when ABS_ENCODER_TYPE = THRIFTY_ANALOG
    // Thrifty absolute encoders → roboRIO analog ports 0–3
    // TODO: confirm your physical wiring
    public static final int FL_ANALOG_PORT = 0;
    public static final int FR_ANALOG_PORT = 1;
    public static final int BL_ANALOG_PORT = 2;
    public static final int BR_ANALOG_PORT = 3;

    /** Thrifty encoder full-scale output voltage */
    public static final double ANALOG_FULL_SCALE_VOLTS = 3.3;

    // ── CANcoder CAN IDs ──────────────────────────────────────────────────────
    // Used when ABS_ENCODER_TYPE = CTRE_CANCODER
    // TODO: update to match your robot's actual wiring
    public static final int FL_CANCODER_CAN_ID = 11;
    public static final int FR_CANCODER_CAN_ID = 12;
    public static final int BL_CANCODER_CAN_ID = 13;
    public static final int BR_CANCODER_CAN_ID = 14;

    // ── Steering zero offsets ─────────────────────────────────────────────────

    /**
     * Analog voltage read when each module wheel points straight forward.
     * Used when ABS_ENCODER_TYPE = THRIFTY_ANALOG.
     *
     * HOW TO MEASURE:
     *   1. Deploy with all offsets = 0.0
     *   2. Rotate each wheel by hand to face straight forward
     *   3. Read "Swerve/FL/Raw Volts" etc. from SmartDashboard
     *   4. Paste those values here and redeploy
     *
     * TODO: measure on your physical robot
     */
    public static final double FL_STEER_OFFSET_VOLTS = 0.0;
    public static final double FR_STEER_OFFSET_VOLTS = 0.0;
    public static final double BL_STEER_OFFSET_VOLTS = 0.0;
    public static final double BR_STEER_OFFSET_VOLTS = 0.0;

    /**
     * Absolute encoder offset in rotations when wheel points straight forward.
     * Used when ABS_ENCODER_TYPE = CTRE_CANCODER or REV_THROUGH_BORE.
     * Range: [-0.5, 0.5]. Subtract from raw encoder reading; result is normalized.
     *
     * HOW TO MEASURE:
     *   1. Deploy with all offsets = 0.0
     *   2. Rotate each wheel by hand to face straight forward
     *   3. Read "Swerve/FL/Abs Enc Rot" from SmartDashboard
     *   4. Paste those values here and redeploy
     *
     * TODO: measure on your physical robot
     */
    public static final double FL_STEER_OFFSET_ROT = 0.0;
    public static final double FR_STEER_OFFSET_ROT = 0.0;
    public static final double BL_STEER_OFFSET_ROT = 0.0;
    public static final double BR_STEER_OFFSET_ROT = 0.0;

    // ── Input deadband ────────────────────────────────────────────────────────
    /** Applied to all joystick axes before scaling. Eliminates stick drift. */
    public static final double INPUT_DEADBAND = 0.08;

    // ── Complementary filter blend weight ─────────────────────────────────────
    /**
     * Alpha weight for blendedState in driveOdometryState.
     * 0.0 = pure IMU, 1.0 = pure encoder.
     * Linear velocity uses alpha (encoder-dominant).
     * Angular velocity uses (1-alpha) (IMU-dominant).
     * Tunable live from SmartDashboard: "Drive/Blend Alpha"
     */
    public static final double ODOMETRY_BLEND_ALPHA = 0.7;

    // ── CAN signal logging ────────────────────────────────────────────────────

    /**
     * Phoenix 6 signal update rate for drivetrain motor telemetry (Hz).
     * Applied to velocity, position, current, voltage, duty cycle signals.
     * 250Hz gives high-fidelity transient capture (wheel slip, current spikes).
     * Lower this if CAN bus utilization becomes a problem at full robot scale.
     */
    public static final double SIGNAL_UPDATE_HZ = 250.0;

    /**
     * Phoenix 6 signal update rate for temperature signals (Hz).
     * Temperatures change slowly — 4Hz is sufficient and saves CAN bandwidth.
     * At 20 motors, 4Hz temps vs. 250Hz saves meaningful headroom.
     */
    public static final double SIGNAL_UPDATE_HZ_TEMP = 4.0;

    /**
     * When true, calls optimizeBusUtilization() on each TalonFX after configuring
     * its signals. This silences all un-registered signals — reducing CAN traffic
     * significantly at full robot scale, but hiding any signal not explicitly
     * registered. Set to false during initial bring-up so nothing fails silently.
     * Flip to true per-mechanism once that mechanism is fully validated on robot.
     */
    public static final boolean OPTIMIZE_CAN_UTILIZATION = false;

    // ── Pose estimator — odometry trust ──────────────────────────────────────
    /**
     * State standard deviations for SwerveDrivePoseEstimator [x (m), y (m), theta (rad)].
     * Smaller = trust wheel odometry more, larger = trust it less.
     * Odometry is reliable short-term; keep these small.
     */
    public static final double ODOMETRY_STD_DEV_X     = 0.1;
    public static final double ODOMETRY_STD_DEV_Y     = 0.1;
    public static final double ODOMETRY_STD_DEV_THETA = 0.1;

    // ── Point-at-target heading PID defaults ──────────────────────────────────
    /**
     * Default gains for the heading-lock PID controller in swerveDrive.
     * Input: heading error (radians). Output: omega correction (rad/s).
     * Tunable live from SmartDashboard: "Drive/HeadingPID/kP" etc.
     *
     * Tuning: increase kP until robot snaps to heading quickly without oscillating.
     * Add kD to damp overshoot. kI is rarely needed for heading control.
     */
    public static final double HEADING_PID_kP       = 3.0;
    public static final double HEADING_PID_kI       = 0.0;
    public static final double HEADING_PID_kD       = 0.2;
    public static final double HEADING_PID_MAX_OMEGA = MAX_ANGULAR_SPEED_RPS;
}