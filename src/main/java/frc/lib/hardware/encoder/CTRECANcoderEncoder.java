package frc.lib.hardware.encoder;

import com.ctre.phoenix6.hardware.CANcoder;

/**
 * CTRECANcoderEncoder.java
 * PATH: src/main/java/frc/lib/hardware/encoder/CTRECANcoderEncoder.java
 *
 * AbsoluteEncoderIO for the CTRE CANcoder (Phoenix 6).
 * Operates on the standard roboRIO CAN bus — no CANivore required.
 *
 * getAbsolutePosition() returns rotations in [-0.5, 0.5). The mounting-angle
 * offset is subtracted and the result is normalized to [0, 1) before converting
 * to radians, so zero radians always corresponds to the zero direction.
 *
 * NOTE: For pure seed-at-startup use, FusedCANcoder / SyncCANcoder (which provide
 * continuous position tracking) are not configured here. If the steer motor is also
 * CTRE TalonFX, consider enabling FusedCANcoder in CTREMotor to eliminate
 * encoder drift over long matches (TODO).
 */
public class CTRECANcoderEncoder implements AbsoluteEncoderIO {

    private final CANcoder canCoder;
    private final double   offsetRotations;

    /**
     * @param canId           CANcoder CAN bus ID
     * @param offsetRotations rotations to subtract so 0 rot = zero direction
     */
    public CTRECANcoderEncoder(int canId, double offsetRotations) {
        this.canCoder        = new CANcoder(canId);
        this.offsetRotations = offsetRotations;
    }

    @Override
    public double getAbsoluteAngleRad() {
        return rotationsToAngleRad(canCoder.getAbsolutePosition().getValueAsDouble(), offsetRotations);
    }

    /**
     * Pure conversion shared by rotation-reporting encoders, exposed for unit tests.
     * Subtracts the offset and wraps into [0, 1) rotations before scaling to radians.
     */
    public static double rotationsToAngleRad(double rawRot, double offsetRotations) {
        double adjusted   = rawRot - offsetRotations;
        // Normalize to [0, 1) using modulo — handles negative values correctly
        double normalized = ((adjusted % 1.0) + 1.0) % 1.0;
        return normalized * 2.0 * Math.PI;
    }
}
