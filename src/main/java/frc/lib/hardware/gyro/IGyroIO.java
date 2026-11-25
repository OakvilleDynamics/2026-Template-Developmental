package frc.lib.hardware.gyro;

/**
 * Vendor-agnostic interface for the drivetrain IMU.
 *
 * <p>Yaw and yaw rate are counter-clockwise positive viewed from above (WPILib's
 * NWU convention). Acceleration is in g along the device X and Y axes.
 *
 * <p>Implementations: CTREPigeon2Gyro.
 */
public interface IGyroIO {

    /** @return yaw in degrees, CCW positive. Continuous; not wrapped to one turn. */
    double getYawDeg();

    /** Sets the current yaw reading, in degrees. */
    void setYawDeg(double yawDeg);

    /** @return yaw rate about the device Z axis in degrees per second, CCW positive. */
    double getYawRateDps();

    /** @return acceleration along the device X axis, in g. */
    double getAccelXG();

    /** @return acceleration along the device Y axis, in g. */
    double getAccelYG();
}
