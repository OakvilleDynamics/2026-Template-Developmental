package frc.lib.hardware.encoder;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

/**
 * EncoderMathTest.java
 * PATH: src/test/java/frc/lib/hardware/encoder/EncoderMathTest.java
 *
 * Offset + wrap conversions used by the absolute encoders. These decide the
 * swerve steer seed angle at power-on, so a sign or wrap error here points
 * every wheel the wrong way.
 */
class EncoderMathTest {

    private static final double EPS = 1e-9;

    @Test
    void rotationOffsetReadsZero() {
        assertEquals(0.0, CTRECANcoderEncoder.rotationsToAngleRad(0.37, 0.37), EPS);
    }

    @Test
    void rotationWrapsNegativeIntoZeroToTwoPi() {
        // CANcoder reports [-0.5, 0.5); -0.25 rot with no offset → 270°
        assertEquals(1.5 * Math.PI, CTRECANcoderEncoder.rotationsToAngleRad(-0.25, 0.0), EPS);
        // Offset larger than reading wraps the same way
        assertEquals(1.5 * Math.PI, CTRECANcoderEncoder.rotationsToAngleRad(0.1, 0.35), EPS);
    }
}
