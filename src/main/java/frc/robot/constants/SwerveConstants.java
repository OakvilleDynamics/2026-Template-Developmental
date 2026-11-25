package frc.robot.constants;

import frc.lib.swerve.SwerveHardware.AbsoluteEncoderType;
import frc.lib.swerve.SwerveHardware.DriveMotorType;
import frc.lib.swerve.SwerveHardware.SteerMotorType;
import frc.lib.util.Units;

/**
 * Hardware-fixed constants that don't change between modules or at runtime.
 *
 * <h2>What lives here</h2>
 *
 * <p>CAN IDs, analog ports, steer offsets, gear ratios, speed limits,
 * heading PID defaults, blend alpha default.
 *
 * <h2>What does not live here</h2>
 *
 * <p>Per-module PID values → RobotContainer (passed into each SwerveModule constructor)
 * Robot geometry → RobotContainer (passed into SwerveDrive constructor)
 */
public final class SwerveConstants {

    // ── Motor / encoder type selection ────────────────────────────────────────
    /**
     * Select the drive and steer motor types and absolute encoder type for this
     * robot build. All four modules always use the same hardware. Change these
     * three constants when swapping hardware — no other code changes required.
     * The type → vendor / current limit mapping lives in frc.lib.swerve.SwerveHardware.
     */
    public static final DriveMotorType      DRIVE_MOTOR_TYPE  = DriveMotorType.KRAKEN_X60;
    // NOVA = Thrifty Nova controller (driving a Minion motor in this build)
    public static final SteerMotorType      STEER_MOTOR_TYPE  = SteerMotorType.NOVA;
    public static final AbsoluteEncoderType ABS_ENCODER_TYPE  = AbsoluteEncoderType.THRIFTY_ANALOG;



    // ── Wheel ─────────────────────────────────────────────────────────────────
    /**
     * Drive wheel diameter in INCHES.
     * Used only in SwerveModule for unit conversion to meters.
     * Thrifty Narrow ships with a 4" billet wheel.
     * TODO: confirm with your actual wheel.
     */
    public static final double WHEEL_DIAMETER_INCHES = 4.0;

    /**
     * Wheel coefficient of friction on FRC carpet (used by PathPlanner).
     * Typical range: 1.0–1.2 for billet wheels on competition carpet.
     * TODO: verify on actual field carpet.
     */
    public static final double WHEEL_FRICTION_COEFFICIENT = 1.19;

    // ── Gear ratios ───────────────────────────────────────────────────────────
    /**
     * Drive motor rotations per wheel rotation.
     * 5.79:1 is set by the pinion installed on these Thrifty Narrow modules
     * (taken from last season's code for the same modules).
     */
    public static final double DRIVE_GEAR_RATIO = 5.79;

    /** Minion motor rotations per module azimuth rotation. Thrifty Narrow = 24:1 */
    public static final double STEER_GEAR_RATIO = 24.0;

    // ── Inversion ─────────────────────────────────────────────────────────────
    /**
     * Drive motor inversion, per module. With every steer offset measured with
     * the bevel gears on the same side, all four modules are physically
     * identical and should share one value. If one wheel drives backward,
     * flip it here (or add half a rotation to its steer offset).
     */
    public static final boolean FL_DRIVE_INVERTED = false;
    public static final boolean FR_DRIVE_INVERTED = false;
    public static final boolean BL_DRIVE_INVERTED = false;
    public static final boolean BR_DRIVE_INVERTED = false;

    /**
     * Steer motor inversion, all modules. Positive steer output must turn the
     * wheel counter-clockwise viewed from above (WPILib convention), so that
     * "Angle Deg" increases when the wheel turns CCW.
     * Nova + Minion: false. A TalonFX steer has previously needed true.
     */
    public static final boolean STEER_MOTOR_INVERTED = false;

    /**
     * Absolute encoder inversion, all modules. Set true if "Abs Enc Rot"
     * decreases when the wheel turns counter-clockwise viewed from above.
     * Supported for THRIFTY_ANALOG only. Last season's code did not invert.
     */
    public static final boolean ABS_ENCODER_INVERTED = false;

    // ── Speed limits ──────────────────────────────────────────────────────────
    /**
     * Maximum linear drive speed in ft/s (driver-facing unit).
     * Stored in both ft/s and m/s — m/s used by kinematics, ft/s used by DriveInput scaling.
     * Kraken X60 at 5.79:1 with 4" wheels: theoretical free speed ≈ 18.1 ft/s
     * (≈ 17.5 ft/s with FOC). Cap at ~90% of the FOC figure for control headroom.
     */
    public static final double MAX_DRIVE_SPEED_FPS = 15.5;
    public static final double MAX_DRIVE_SPEED_MPS = Units.ftpsToMps(MAX_DRIVE_SPEED_FPS);

    /** Maximum angular speed (rad/s). ~1 full rotation per second. */
    public static final double MAX_ANGULAR_SPEED_RPS = 2.0 * Math.PI;

    /**
     * Driver speed scaling for bring-up, applied in TeleopDrive after deadband
     * and squaring. Full stick = scale × max speed. Only affects the driver —
     * kinematics desaturation and PathPlanner still use the max speeds above.
     * Live-tunable from the dashboard: "Drive/Teleop Speed Scale" and
     * "Drive/Teleop Rotation Scale" (clamped to [0, 1]).
     *
     * <p>TODO: set both to 1.0 once the drivetrain is validated.
     */
    public static final double TELEOP_SPEED_SCALE    = 0.3;  // ≈ 4.7 ft/s at full stick
    public static final double TELEOP_ROTATION_SCALE = 0.5;  // ≈ 0.5 rot/s at full stick

    // ── CAN IDs and port assignments ─────────────────────────────────────────
    // All CAN IDs and roboRIO port assignments are in CANIDs.java.

    // ── Steering zero offsets ─────────────────────────────────────────────────

    /**
     * Absolute encoder reading, in rotations, when each wheel points straight
     * forward. Used by every ABS_ENCODER_TYPE. Subtracted from the raw reading;
     * the result is wrapped, so any value in [0, 1) or [-0.5, 0.5) works.
     *
     * <p>HOW TO MEASURE:
     *
     * <ol>
     *   <li>Rotate each wheel by hand to face straight forward, all bevel gears on the same side</li>
     *   <li>THRIFTY_ANALOG: read "Swerve/FL/Raw Rot" etc. (offset-free, any offsets). Other types: deploy with offsets = 0.0 and read "Swerve/FL/Abs Enc Rot"</li>
     *   <li>Paste those values here and redeploy</li>
     * </ol>
     *
     * <p>Starting values are last season's AnalogEncoder offsets for these modules
     * (ticks of 4096). TODO: re-measure on the robot — they only hold if the
     * encoders and magnets haven't been remounted.
     */
    public static final double FL_STEER_OFFSET_ROT = 3118.0 / 4096.0;  // 0.7612
    public static final double FR_STEER_OFFSET_ROT = 2651.0 / 4096.0;  // 0.6472
    public static final double BL_STEER_OFFSET_ROT =  934.0 / 4096.0;  // 0.2280
    public static final double BR_STEER_OFFSET_ROT = 3834.0 / 4096.0;  // 0.9360

    // ── Input deadband ────────────────────────────────────────────────────────
    /** Applied to all joystick axes before scaling. Eliminates stick drift. */
    public static final double INPUT_DEADBAND = 0.08;

    // ── Complementary filter blend weight ─────────────────────────────────────
    /**
     * Alpha weight for blendedState in DriveOdometryState.
     * 0.0 = pure IMU, 1.0 = pure encoder.
     * Linear velocity uses alpha (encoder-dominant).
     * Angular velocity uses (1-alpha) (IMU-dominant).
     * Tunable live from the dashboard: "Drive/Blend Alpha"
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
     * Default gains for the heading-lock PID controller in SwerveDrive.
     * Input: heading error (radians). Output: omega correction (rad/s).
     * Tunable live from the dashboard: "Drive/HeadingPID/kP" etc.
     *
     * <p>Tuning: increase kP until robot snaps to heading quickly without oscillating.
     * Add kD to damp overshoot. kI is rarely needed for heading control.
     */
    public static final double HEADING_PID_kP       = 3.0;
    public static final double HEADING_PID_kI       = 0.0;
    public static final double HEADING_PID_kD       = 0.2;
    public static final double HEADING_PID_MAX_OMEGA = MAX_ANGULAR_SPEED_RPS;
}