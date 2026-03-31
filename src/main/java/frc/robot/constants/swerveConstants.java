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

    // ── Analog encoder ports (roboRIO) ────────────────────────────────────────
    // Thrifty absolute encoders → roboRIO analog ports 0–3
    // TODO: confirm your physical wiring
    public static final int FL_ANALOG_PORT = 0;
    public static final int FR_ANALOG_PORT = 1;
    public static final int BL_ANALOG_PORT = 2;
    public static final int BR_ANALOG_PORT = 3;

    /** Thrifty encoder full-scale output voltage */
    public static final double ANALOG_FULL_SCALE_VOLTS = 3.3;

    // ── Steering zero offsets ─────────────────────────────────────────────────
    /**
     * Analog voltage read when each module wheel points straight forward.
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