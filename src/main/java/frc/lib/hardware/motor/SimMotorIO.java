package frc.lib.hardware.motor;

import java.util.List;

import frc.lib.util.ConfigVerifyResult;

/**
 * SimMotorIO.java
 * PATH: src/main/java/frc/lib/hardware/motor/SimMotorIO.java
 *
 * Simulation-only MotorIO with no hardware behind it. Velocity and position
 * setpoints are reached instantly (position takes the shortest path when
 * continuousWrap is set); currents, voltage and temperature read zero.
 *
 * MotorIO.create() returns this in simulation when the configured vendor has
 * no library for this WPILib year (currently ThriftyBot Nova), so the rest of
 * the robot code can still run in the simulator. Never constructed on a real
 * robot — there, create() throws instead.
 */
public class SimMotorIO implements MotorIO {

    private final MotorIOConfig config;
    private double positionRot = 0.0;
    private double velocityRps = 0.0;

    public SimMotorIO(MotorIOConfig config) {
        this.config = config;
    }

    @Override public MotorIOConfig getConfig() { return config; }

    @Override public void setDutyCycle(double duty) { velocityRps = 0.0; }

    @Override public void setVelocity(double velocityRps, double ffVolts) { this.velocityRps = velocityRps; }

    @Override
    public void setPosition(double positionRot, double ffVolts) {
        double target = positionRot;
        if (config.continuousWrap) {
            // Shortest path: move by the wrapped error, in [-0.5, 0.5] rotations
            double error = positionRot - this.positionRot;
            error -= Math.rint(error);
            target = this.positionRot + error;
        }
        this.positionRot = target;
        this.velocityRps = 0.0;
    }

    @Override public void stop() { velocityRps = 0.0; }

    @Override public void follow(MotorIO leader, boolean opposeLeader) {}

    @Override public void setEncoderPosition(double positionRot) { this.positionRot = positionRot; }

    @Override public void applyGains(double kP, double kI, double kD, double kS, double kV, double kA) {}

    @Override public double  getPositionRot()               { return positionRot; }
    @Override public double  getVelocityRps()               { return velocityRps; }
    @Override public double  getSupplyCurrentAmps()         { return 0.0; }
    @Override public double  getStatorCurrentAmps()         { return 0.0; }
    @Override public double  getMotorVoltage()              { return 0.0; }
    @Override public double  getTemperatureC()              { return 0.0; }
    @Override public boolean isForwardSoftLimitHit()        { return false; }
    @Override public boolean isReverseSoftLimitHit()        { return false; }
    @Override public double  getStallDetectionCurrentAmps() { return 0.0; }
    @Override public boolean hasHardwareMotionProfile()     { return true; }
    @Override public boolean supportsEncoderSync()          { return false; }

    @Override
    public ConfigVerifyResult verifyConfig(String label) {
        return new ConfigVerifyResult(label, config.canId, "Simulated " + config.vendor.displayName,
                                      true, List.of());
    }
}
