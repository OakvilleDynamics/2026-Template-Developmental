package frc.robot.constants;

/**
 * portIDs.java
 * PATH: src/main/java/frc/robot/constants/portIDs.java
 *
 * Single source of truth for ALL roboRIO non-CAN port assignments:
 * analog inputs (AI), analog outputs (AO), digital inputs (DI),
 * digital outputs (DO), and PWM.
 *
 * Update this file when rewiring the robot — no other files need to change.
 * CAN bus IDs live in canIDs.java.
 *
 * ─── VERIFICATION PROCEDURE ──────────────────────────────────────────────────
 * Trace wires from each sensor/device to the physical roboRIO port before deploy.
 * roboRIO port layout: https://docs.wpilib.org/en/stable/docs/hardware/hardware-basics/
 *
 * TODO: confirm all ports match physical wiring before first deploy.
 */
public final class portIDs {

    // ── Analog Inputs (AI 0–3) ────────────────────────────────────────────────
    // Thrifty absolute encoders for swerve steer.
    // Used when swerveConstants.ABS_ENCODER_TYPE = THRIFTY_ANALOG.
    // TODO: confirm ports match physical wiring.
    public static final int AI_FL_STEER_ENCODER = 0;
    public static final int AI_FR_STEER_ENCODER = 1;
    public static final int AI_BL_STEER_ENCODER = 2;
    public static final int AI_BR_STEER_ENCODER = 3;

    // ── Analog Outputs (AO 0–1) ──────────────────────────────────────────────
    // None currently assigned.

    // ── Digital Inputs (DI 0–9) ──────────────────────────────────────────────
    // None currently assigned. Add limit switches, beam breaks, etc. here.
    // Example: public static final int DI_INTAKE_BEAM_BREAK = 0;

    // ── Digital Outputs (DO 0–9) ─────────────────────────────────────────────
    // None currently assigned. Add LED control, solenoid signals, etc. here.
    // Example: public static final int DO_INDICATOR_LED = 1;

    // ── PWM (0–9) ────────────────────────────────────────────────────────────
    // None currently assigned. Add servos or PWM-controlled devices here.
    // Example: public static final int PWM_HOOD_SERVO = 0;
}
