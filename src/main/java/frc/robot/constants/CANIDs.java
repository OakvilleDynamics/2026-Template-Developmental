package frc.robot.constants;

/**
 * Single source of truth for ALL CAN bus IDs.
 * Update this file when rewiring the robot — no other files need to change.
 * roboRIO analog/digital/PWM port assignments live in PortIDs.java.
 *
 * <h2>Verification procedure</h2>
 *
 * <p>Before first deploy, verify each ID against the physical device:
 *
 * <ul>
 *   <li><b>CTRE devices (Kraken, Pigeon 2)</b>: use Phoenix Tuner X (Self-Test Snapshot)</li>
 *   <li><b>Thrifty Nova</b>: use ThriftyBot app or Nova's built-in LED blink pattern</li>
 * </ul>
 *
 * <p>TODO: confirm all IDs match physical wiring before first deploy.
 */
public final class CANIDs {

    // ── Swerve drive motors — Kraken X60 (TalonFX, Phoenix 6) ────────────────
    public static final int FL_DRIVE = 2;
    public static final int FR_DRIVE = 4;
    public static final int BL_DRIVE = 6;
    public static final int BR_DRIVE = 8;

    // ── Swerve steer motors — Thrifty Nova ───────────────────────────────────
    public static final int FL_STEER = 3;
    public static final int FR_STEER = 5;
    public static final int BL_STEER = 7;
    public static final int BR_STEER = 9;

    // ── IMU — CTRE Pigeon 2.0 ────────────────────────────────────────────────
    // TODO: Verify this CAN ID
    public static final int PIGEON2 = 1;

    // ── CANcoder CAN IDs — unused (ABS_ENCODER_TYPE = THRIFTY_ANALOG) ────────
    // Populated here for reference if encoder type is ever changed.
    // TODO: assign real IDs if switching to CTRE_CANCODER
    public static final int FL_CANCODER_UNUSED = 21;
    public static final int FR_CANCODER_UNUSED = 22;
    public static final int BL_CANCODER_UNUSED = 23;
    public static final int BR_CANCODER_UNUSED = 24;
}
