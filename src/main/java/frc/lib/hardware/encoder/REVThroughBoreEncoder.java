package frc.lib.hardware.encoder;

import com.revrobotics.spark.SparkAbsoluteEncoder;

import frc.lib.hardware.motor.IMotorIO;
import frc.lib.hardware.motor.REVMotor;

/**
 * IAbsoluteEncoderIO for the REV Through Bore encoder wired to the data port of
 * a SparkMax or SparkFlex.
 *
 * <p>The encoder is read through the motor controller it is plugged into, so it is
 * built from that controller's IMotorIO — there is exactly one controller
 * instance per CAN ID. Only valid with a REV IMotorIO (REV_SPARKMAX / REV_SPARKFLEX).
 *
 * <p>getPosition() returns [0, 1) rotations. The mounting-angle offset is subtracted
 * and the result is normalized to [0, 1) before converting to radians, so zero
 * radians always corresponds to the zero direction.
 */
public class REVThroughBoreEncoder implements IAbsoluteEncoderIO {

    private final SparkAbsoluteEncoder encoder;
    private final double               offsetRotations;

    /**
     * @param host            the REV IMotorIO this encoder is wired to
     * @param offsetRotations rotations to subtract so 0 rot = zero direction
     * @throws IllegalArgumentException if {@code host} is not a REV controller
     */
    public REVThroughBoreEncoder(IMotorIO host, double offsetRotations) {
        if (!(host instanceof REVMotor revHost))
            throw new IllegalArgumentException(
                "REVThroughBoreEncoder requires a REV SparkMax/SparkFlex IMotorIO, got " + host.getConfig());
        this.encoder         = revHost.getSpark().getAbsoluteEncoder();
        this.offsetRotations = offsetRotations;
    }

    @Override
    public double getAbsoluteAngleRad() {
        return CTRECANcoderEncoder.rotationsToAngleRad(encoder.getPosition(), offsetRotations);
    }
}
