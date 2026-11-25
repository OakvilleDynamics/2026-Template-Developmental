package frc.lib.control;

/**
 * Enumerations and constants shared by MechanismUnit and MechanismConfig.
 * No vendor-specific imports. The vendor enum lives in
 * frc.lib.hardware.motor.MotorVendor.
 */
public final class MotorConstants {

    // ── Follow mode enum ──────────────────────────────────────────────────────

    /**
     * How follower motors track the leader.
     *
     * <pre>{@code
     * NONE          — single-motor mechanism; no followers
     * MECHANICAL    — hardware follower; controller mirrors leader output directly.
     *                 Use when motors share a rigid mechanical linkage (same shaft,
     *                 belt, or chain with no possibility of slip).
     *                 Zero Rio CPU per cycle after construction.
     * ENCODER_SYNC  — software follower; Rio reads both encoders each loop and
     *                 applies a proportional duty cycle correction when the follower
     *                 drifts beyond the deadband. Use for non-rigid couplings where
     *                 slip is possible and must be detected/corrected.
     *                 NOTE: Not supported for THRIFTYBOT_NOVA — Nova's follow() API
     *                 does not expose independent encoder readback on the follower.
     * }</pre>
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
     *
     * <ul>
     *   <li>Software duty cycle ramp rate calculations</li>
     *   <li>Finite-difference acceleration estimation (dv/dt)</li>
     *   <li>Software motion-profile step limiter (controllers without one, e.g. Nova)</li>
     * </ul>
     */
    public static final double LOOP_PERIOD_SECS = 0.020;

    // ── Encoder sync defaults ─────────────────────────────────────────────────

    /**
     * Default deadband for ENCODER_SYNC follow mode (rotations, mechanism shaft).
     * Correction is not applied when |leaderPos - followerPos| ≤ this value.
     * Tune per-mechanism via MechanismConfig.Builder.withEncoderSync().
     */
    public static final double ENCODER_SYNC_DEADBAND_ROT = 0.05;

    /**
     * Default proportional gain for ENCODER_SYNC correction.
     * Output = kP × error (duty cycle per rotation of error).
     * Result is clamped to [-1.0, 1.0] before being applied.
     * Tune per-mechanism via MechanismConfig.Builder.withEncoderSync().
     */
    public static final double ENCODER_SYNC_KP_DEFAULT = 0.5;

    private MotorConstants() {}
}
