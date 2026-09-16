package frc.robot.subsystems.swerveDrive.steer;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkAbsoluteEncoder;

/**
 * REVThroughBoreEncoder.java
 * PATH: src/main/java/frc/robot/subsystems/swerveDrive/steer/REVThroughBoreEncoder.java
 *
 * SwerveAbsoluteEncoder for the REV Through Bore encoder wired to the data port
 * of a SparkMax, SparkFlex, or compatible Nova controller.
 *
 * The encoder is accessed via the motor controller — both share a single CAN device.
 * The swerveModule factory creates the SparkBase first, passes it to both this class
 * and to REVSteerMotor, so there is exactly one controller instance per CAN ID.
 *
 * NOT compatible with Nova steer motor (ThriftyNova is not a SparkBase). Use
 * ThriftyAnalogEncoder with Nova steer instead.
 *
 * getPosition() returns [0, 1) rotations. The mounting-angle offset is subtracted
 * and the result is normalized to [0, 1) before converting to radians, so zero
 * radians always corresponds to straight forward.
 */
public class REVThroughBoreEncoder implements SwerveAbsoluteEncoder {

    private final SparkAbsoluteEncoder encoder;
    private final double               offsetRotations;

    /**
     * @param spark           the SparkBase this encoder is wired to
     *                        (same instance passed to REVSteerMotor)
     * @param offsetRotations rotations to subtract so 0 rot = straight forward
     *                        (from swerveConstants.FL/FR/BL/BR_STEER_OFFSET_ROT)
     */
    public REVThroughBoreEncoder(SparkBase spark, double offsetRotations) {
        this.encoder         = spark.getAbsoluteEncoder();
        this.offsetRotations = offsetRotations;
    }

    @Override
    public double getAbsoluteAngleRad() {
        // getPosition() returns [0, 1) rotations
        double rawRot     = encoder.getPosition();
        double adjusted   = rawRot - offsetRotations;
        // Normalize to [0, 1) using modulo — handles negative values correctly
        double normalized = ((adjusted % 1.0) + 1.0) % 1.0;
        return normalized * 2.0 * Math.PI;
    }
}
