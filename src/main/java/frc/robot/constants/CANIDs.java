package frc.robot.constants;

/**
 * canIDs.java
 * PATH: src/main/java/frc/robot/constants/canIDs.java
 *
 * Single source of truth for ALL CAN bus IDs.
 * Update this file when rewiring the robot — no other files need to change.
 * roboRIO analog/digital/PWM port assignments live in portIDs.java.
 *
 * ─── VERIFICATION PROCEDURE ──────────────────────────────────────────────────
 * Before first deploy, verify each ID against the physical device:
 *   CTRE devices (Kraken, Pigeon 2): use Phoenix Tuner X (Self-Test Snapshot)
 *   Thrifty Nova: use ThriftyBot app or Nova's built-in LED blink pattern
 *
 * TODO: confirm all IDs match physical wiring before first deploy.
 */
public final class canIDs {

    // ── Swerve drive motors — Kraken X60 (TalonFX, Phoenix 6) ────────────────
    // TODO: confirm CAN IDs match physical wiring
    public static final int FL_DRIVE = 1;
    public static final int FR_DRIVE = 2;
    public static final int BL_DRIVE = 3;
    public static final int BR_DRIVE = 4;

    // ── Swerve steer motors — Thrifty Nova ───────────────────────────────────
    // TODO: confirm CAN IDs match physical wiring
    public static final int FL_STEER = 11;
    public static final int FR_STEER = 12;
    public static final int BL_STEER = 13;
    public static final int BR_STEER = 14;

    // ── IMU — CTRE Pigeon 2.0 ────────────────────────────────────────────────
    public static final int PIGEON2 = 0;

    // ── CANcoder CAN IDs — unused (ABS_ENCODER_TYPE = THRIFTY_ANALOG) ────────
    // Populated here for reference if encoder type is ever changed.
    // TODO: assign real IDs if switching to CTRE_CANCODER
    public static final int FL_CANCODER = 21;
    public static final int FR_CANCODER = 22;
    public static final int BL_CANCODER = 23;
    public static final int BR_CANCODER = 24;
}
