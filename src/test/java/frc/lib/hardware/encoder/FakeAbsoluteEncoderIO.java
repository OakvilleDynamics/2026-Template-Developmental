package frc.lib.hardware.encoder;

/**
 * FakeAbsoluteEncoderIO.java
 * PATH: src/test/java/frc/lib/hardware/encoder/FakeAbsoluteEncoderIO.java
 *
 * In-memory IAbsoluteEncoderIO for unit tests — no HAL, no analog input, no
 * vendor library. Tests set the angle in degrees; it is reported in radians
 * wrapped to [0, 2π), matching the interface contract.
 */
public class FakeAbsoluteEncoderIO implements IAbsoluteEncoderIO {

    private double angleRad;

    public FakeAbsoluteEncoderIO(double angleDeg) {
        setAngleDeg(angleDeg);
    }

    public void setAngleDeg(double angleDeg) {
        double wrappedDeg = ((angleDeg % 360.0) + 360.0) % 360.0;
        angleRad = Math.toRadians(wrappedDeg);
    }

    @Override
    public double getAbsoluteAngleRad() {
        return angleRad;
    }
}
