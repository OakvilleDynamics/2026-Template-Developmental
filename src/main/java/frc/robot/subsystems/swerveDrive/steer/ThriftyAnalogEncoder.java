package frc.robot.subsystems.swerveDrive.steer;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.constants.swerveConstants;

/**
 * ThriftyAnalogEncoder.java
 * PATH: src/main/java/frc/robot/subsystems/swerveDrive/steer/ThriftyAnalogEncoder.java
 *
 * SwerveAbsoluteEncoder for the Thrifty absolute analog encoder (0–3.3 V output).
 * Wired to a roboRIO analog port.
 *
 * The mounting-angle offset (the voltage read when the wheel faces straight forward)
 * is subtracted and the result is normalized to [0, 3.3 V) before converting to
 * radians, so zero radians always corresponds to straight forward.
 */
public class ThriftyAnalogEncoder implements SwerveAbsoluteEncoder {

    private final AnalogInput encoder;
    private final double      offsetVolts;

    /**
     * @param analogPort   roboRIO analog port (0–3)
     * @param offsetVolts  voltage reading when wheel points straight forward
     *                     (from swerveConstants.FL/FR/BL/BR_STEER_OFFSET_VOLTS)
     */
    public ThriftyAnalogEncoder(int analogPort, double offsetVolts) {
        this.encoder     = new AnalogInput(analogPort);
        this.offsetVolts = offsetVolts;
    }

    @Override
    public double getAbsoluteAngleRad() {
        double rawVolts        = encoder.getVoltage();
        double offsetVolts     = rawVolts - this.offsetVolts;
        // Normalize to [0, ANALOG_FULL_SCALE_VOLTS) using modulo
        double normalizedVolts = offsetVolts % swerveConstants.ANALOG_FULL_SCALE_VOLTS;
        if (normalizedVolts < 0) normalizedVolts += swerveConstants.ANALOG_FULL_SCALE_VOLTS;
        return (normalizedVolts / swerveConstants.ANALOG_FULL_SCALE_VOLTS) * 2.0 * Math.PI;
    }

    /** Raw voltage for telemetry — published by swerveModule.publishTelemetry(). */
    public double getRawVolts() {
        return encoder.getVoltage();
    }
}
