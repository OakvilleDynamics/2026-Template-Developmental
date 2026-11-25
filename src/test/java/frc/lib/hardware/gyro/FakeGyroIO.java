package frc.lib.hardware.gyro;

/**
 * In-memory IGyroIO for unit tests — no HAL, no CAN, no vendor library.
 * Readings are plain fields the test sets directly.
 */
public class FakeGyroIO implements IGyroIO {

    public double yawDeg;
    public double yawRateDps;
    public double accelXG;
    public double accelYG;
    public int    setYawCount;

    @Override public double getYawDeg()     { return yawDeg; }
    @Override public double getYawRateDps() { return yawRateDps; }
    @Override public double getAccelXG()    { return accelXG; }
    @Override public double getAccelYG()    { return accelYG; }

    @Override
    public void setYawDeg(double yawDeg) {
        this.yawDeg = yawDeg;
        setYawCount++;
    }
}
