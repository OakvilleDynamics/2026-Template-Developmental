package frc.lib.hardware.gyro;

import com.ctre.phoenix6.hardware.Pigeon2;

/**
 * CTREPigeon2Gyro.java
 * PATH: src/main/java/frc/lib/hardware/gyro/CTREPigeon2Gyro.java
 *
 * IGyroIO for the CTRE Pigeon 2.0 (Phoenix 6). The Pigeon reports yaw in degrees,
 * CCW positive (NWU), which already matches WPILib.
 *
 * The Pigeon self-calibrates on power-up. Temperature calibration is performed
 * once in Phoenix Tuner X, so no runtime calibrate call is needed.
 */
public class CTREPigeon2Gyro implements IGyroIO {

    private final Pigeon2 pigeon;

    /** @param canId Pigeon 2 CAN bus ID */
    public CTREPigeon2Gyro(int canId) {
        this.pigeon = new Pigeon2(canId);
    }

    @Override public double getYawDeg()            { return pigeon.getYaw().getValueAsDouble(); }
    @Override public void   setYawDeg(double yawDeg) { pigeon.setYaw(yawDeg); }
    @Override public double getYawRateDps()        { return pigeon.getAngularVelocityZDevice().getValueAsDouble(); }
    @Override public double getAccelXG()           { return pigeon.getAccelerationX().getValueAsDouble(); }
    @Override public double getAccelYG()           { return pigeon.getAccelerationY().getValueAsDouble(); }
}
