package frc.lib.hardware.encoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

/**
 * IAbsoluteEncoderIO for the Thrifty absolute magnetic encoder wired to a
 * roboRIO analog port.
 *
 * <p>The encoder runs on the roboRIO's 5 V rail and its output is ratiometric:
 * one full turn spans 0 V to the rail voltage. (A jumper selects 3.3 V
 * operation, mainly for FTC — leave it in the default 5 V position.)
 *
 * <p>Reading is delegated to WPILib's AnalogEncoder, which divides by the
 * measured 5 V rail each read, so rail sag does not shift the angle. Units are
 * rotations (fullRange = 1); the offset is the raw rotation reading when the
 * module points in its zero direction.
 */
public class ThriftyAnalogEncoder implements IAbsoluteEncoderIO {

    private final AnalogInput   input;
    private final AnalogEncoder encoder;

    /**
     * @param analogPort roboRIO analog port (0–3)
     * @param offsetRot  raw reading in rotations [0, 1) at the zero direction
     * @param inverted   true if the reading rises when the module turns clockwise
     *                   viewed from above
     */
    public ThriftyAnalogEncoder(int analogPort, double offsetRot, boolean inverted) {
        this.input   = new AnalogInput(analogPort);
        this.encoder = new AnalogEncoder(input, 1.0, offsetRot);
        this.encoder.setInverted(inverted);
    }

    @Override
    public double getAbsoluteAngleRad() {
        // AnalogEncoder returns [0, 1], with exactly 1 at zero when inverted — wrap to [0, 1)
        return MathUtil.inputModulus(encoder.get(), 0.0, 1.0) * 2.0 * Math.PI;
    }

    /**
     * Raw reading in rotations [0, 1), before offset and inversion. Read this
     * with the wheel pointed forward to measure the steer offset.
     */
    public double getRawRotations() {
        return input.getVoltage() / RobotController.getVoltage5V();
    }

    /** Raw voltage for telemetry. Should span about 0 to 5 V over one turn. */
    public double getRawVolts() {
        return input.getVoltage();
    }
}
