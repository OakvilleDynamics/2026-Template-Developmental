package frc.robot.subsystems.swerveDrive.steer;

import com.ctre.phoenix6.hardware.CANcoder;

/**
 * CTRECANcoderEncoder.java
 * PATH: src/main/java/frc/robot/subsystems/swerveDrive/steer/CTRECANcoderEncoder.java
 *
 * SwerveAbsoluteEncoder for the CTRE CANcoder (Phoenix 6).
 * Operates on the standard roboRIO CAN bus — no CANivore required.
 *
 * getAbsolutePosition() returns rotations in [-0.5, 0.5). The mounting-angle
 * offset is subtracted and the result is normalized to [0, 1) before converting
 * to radians, so zero radians always corresponds to straight forward.
 *
 * NOTE: For pure seed-at-startup use, FusedCANcoder / SyncCANcoder (which provide
 * continuous position tracking) are not configured here. If the steer motor is also
 * CTRE TalonFX, consider enabling FusedCANcoder in CTRESteerMotor to eliminate
 * encoder drift over long matches (TODO).
 */
public class CTRECANcoderEncoder implements SwerveAbsoluteEncoder {

    private final CANcoder canCoder;
    private final double   offsetRotations;

    /**
     * @param canId           CANcoder CAN bus ID
     *                        (from swerveConstants.FL/FR/BL/BR_CANCODER_CAN_ID)
     * @param offsetRotations rotations to subtract so 0 rot = straight forward
     *                        (from swerveConstants.FL/FR/BL/BR_STEER_OFFSET_ROT)
     */
    public CTRECANcoderEncoder(int canId, double offsetRotations) {
        this.canCoder        = new CANcoder(canId);
        this.offsetRotations = offsetRotations;
    }

    @Override
    public double getAbsoluteAngleRad() {
        // getAbsolutePosition() returns [-0.5, 0.5) rotations
        double rawRot     = canCoder.getAbsolutePosition().getValueAsDouble();
        double adjusted   = rawRot - offsetRotations;
        // Normalize to [0, 1) using modulo — handles negative values correctly
        double normalized = ((adjusted % 1.0) + 1.0) % 1.0;
        return normalized * 2.0 * Math.PI;
    }
}
