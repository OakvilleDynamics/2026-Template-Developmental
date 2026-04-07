package frc.robot.util.motors;

/**
 * motorConstants.java
 * PATH: src/main/java/frc/robot/util/motors/motorConstants.java
 *
 * Enumerations and package-level constants shared across all mechanismUnit
 * implementations. No vendor-specific imports.
 */
public final class motorConstants {

    // ── Vendor enum ───────────────────────────────────────────────────────────

    /**
     * Motor controller vendor. All motors in a single mechanismUnit must share
     * the same vendor — mixed-vendor mechanisms are not supported.
     *
     * Vendor assignment by robot subsystem:
     *   CTRE_TALONFX   — drivetrain (all years); Kraken X60, X44, Minion
     *   REV_SPARKMAX    — mechanisms using NEO / NEO 550 (current season)
     *   REV_SPARKFLEX   — mechanisms using NEO Vortex
     *   THRIFTYBOT_NOVA — mechanisms transitioning from REV (go-forward platform)
     */
    public enum Vendor {
        CTRE_TALONFX,
        REV_SPARKMAX,
        REV_SPARKFLEX,
        THRIFTYBOT_NOVA
    }

    // ── Follow mode enum ──────────────────────────────────────────────────────

    /**
     * How follower motors track the leader.
     *
     *   NONE          — single-motor mechanism; no followers
     *   MECHANICAL    — hardware follower; controller mirrors leader output directly.
     *                   Use when motors share a rigid mechanical linkage (same shaft,
     *                   belt, or chain with no possibility of slip).
     *                   Zero Rio CPU per cycle after construction.
     *   ENCODER_SYNC  — software follower; Rio reads both encoders each loop and
     *                   applies a proportional duty cycle correction when the follower
     *                   drifts beyond the deadband. Use for non-rigid couplings where
     *                   slip is possible and must be detected/corrected.
     *                   NOTE: Not supported for THRIFTYBOT_NOVA — Nova's follow() API
     *                   does not expose independent encoder readback on the follower.
     */
    public enum FollowMode {
        NONE,
        MECHANICAL,
        ENCODER_SYNC
    }

    // ── Loop timing ───────────────────────────────────────────────────────────

    /**
     * Robot control loop period (seconds). Matches WPILib default 20ms loop.
     * Used for:
     *   - Software duty cycle ramp rate calculations
     *   - Finite-difference acceleration estimation (dv/dt)
     *   - Nova software setpoint step limiter
     */
    public static final double LOOP_PERIOD_SECS = 0.020;

    // ── Encoder sync defaults ─────────────────────────────────────────────────

    /**
     * Default deadband for ENCODER_SYNC follow mode (rotations, mechanism shaft).
     * Correction is not applied when |leaderPos - followerPos| <= this value.
     * Tune per-mechanism via mechanismConfig.Builder.withEncoderSync().
     */
    public static final double ENCODER_SYNC_DEADBAND_ROT = 0.05;

    /**
     * Default proportional gain for ENCODER_SYNC correction.
     * Output = kP × error (duty cycle per rotation of error).
     * Result is clamped to [-1.0, 1.0] before being applied.
     * Tune per-mechanism via mechanismConfig.Builder.withEncoderSync().
     */
    public static final double ENCODER_SYNC_KP_DEFAULT = 0.5;

    // ── Nova FF approximation ─────────────────────────────────────────────────

    /**
     * Minimum battery voltage used as divisor when converting FF volts to duty
     * cycle for ThriftyBot Nova (which has no native per-cycle voltage injection).
     *
     * Actual divisor = max(RobotController.getBatteryVoltage(), MIN_BATTERY_VOLTAGE)
     *
     * Clamped to prevent divide-by-near-zero during simulation or deep brownout.
     * Actual battery voltage is read live from RobotController each cycle, so the
     * approximation tracks sag under load rather than assuming a fixed 12V.
     */
    public static final double MIN_BATTERY_VOLTAGE = 7.0;

    private motorConstants() {}
}
