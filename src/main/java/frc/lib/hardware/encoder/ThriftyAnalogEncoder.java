package frc.lib.hardware.encoder;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * ThriftyAnalogEncoder.java
 * PATH: src/main/java/frc/lib/hardware/encoder/ThriftyAnalogEncoder.java
 *
 * IAbsoluteEncoderIO for the Thrifty absolute analog encoder (0–3.3 V output).
 * Wired to a roboRIO analog port.
 *
 * The mounting-angle offset (the voltage read at the zero direction) is
 * subtracted and the result is normalized to [0, fullScaleVolts) before
 * converting to radians, so zero radians always corresponds to the zero direction.
 * When inverted, the angle runs the other way around the same zero.
 */
public class ThriftyAnalogEncoder implements IAbsoluteEncoderIO {

    /** Thrifty encoder full-scale output voltage. */
    public static final double DEFAULT_FULL_SCALE_VOLTS = 3.3;

    private final AnalogInput encoder;
    private final double      offsetVolts;
    private final double      fullScaleVolts;
    private final boolean     inverted;

    /**
     * @param analogPort   roboRIO analog port (0–3)
     * @param offsetVolts  voltage reading at the zero direction
     */
    public ThriftyAnalogEncoder(int analogPort, double offsetVolts) {
        this(analogPort, offsetVolts, DEFAULT_FULL_SCALE_VOLTS);
    }

    /**
     * @param analogPort     roboRIO analog port (0–3)
     * @param offsetVolts    voltage reading at the zero direction
     * @param fullScaleVolts voltage corresponding to one full rotation
     */
    public ThriftyAnalogEncoder(int analogPort, double offsetVolts, double fullScaleVolts) {
        this(analogPort, offsetVolts, fullScaleVolts, false);
    }

    /**
     * @param analogPort     roboRIO analog port (0–3)
     * @param offsetVolts    voltage reading at the zero direction
     * @param fullScaleVolts voltage corresponding to one full rotation
     * @param inverted       true if the voltage rises when the module turns clockwise
     */
    public ThriftyAnalogEncoder(int analogPort, double offsetVolts, double fullScaleVolts,
                                boolean inverted) {
        this.encoder        = new AnalogInput(analogPort);
        this.offsetVolts    = offsetVolts;
        this.fullScaleVolts = fullScaleVolts;
        this.inverted       = inverted;
    }

    @Override
    public double getAbsoluteAngleRad() {
        return voltsToAngleRad(encoder.getVoltage(), offsetVolts, fullScaleVolts, inverted);
    }

    /**
     * Pure conversion with optional inversion, exposed for unit tests.
     * Inverted mirrors the angle around the zero direction: θ → (2π − θ) mod 2π.
     */
    public static double voltsToAngleRad(double rawVolts, double offsetVolts, double fullScaleVolts,
                                         boolean inverted) {
        double angle = voltsToAngleRad(rawVolts, offsetVolts, fullScaleVolts);
        if (!inverted || angle == 0.0) return angle;
        return 2.0 * Math.PI - angle;
    }

    /**
     * Pure conversion, exposed for unit tests.
     * Subtracts the offset and wraps into [0, fullScaleVolts) before scaling to radians.
     */
    public static double voltsToAngleRad(double rawVolts, double offsetVolts, double fullScaleVolts) {
        double normalizedVolts = (rawVolts - offsetVolts) % fullScaleVolts;
        if (normalizedVolts < 0) normalizedVolts += fullScaleVolts;
        return (normalizedVolts / fullScaleVolts) * 2.0 * Math.PI;
    }

    /** Raw voltage for telemetry and offset calibration. */
    public double getRawVolts() {
        return encoder.getVoltage();
    }
}
