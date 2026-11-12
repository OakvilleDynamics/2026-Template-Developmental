package frc.lib.hardware.motor;

import java.util.List;

import frc.lib.util.ConfigVerifyResult;

/**
 * FakeMotorIO.java
 * PATH: src/test/java/frc/lib/hardware/motor/FakeMotorIO.java
 *
 * In-memory MotorIO for unit tests — no HAL, no CAN, no vendor library.
 *
 * Records every command (last duty cycle / velocity / position / FF volts,
 * follow target, encoder resets, gain updates). Feedback values (position,
 * velocity, currents) are plain fields the test sets directly to simulate the
 * mechanism, e.g. stalling against a hard stop:
 *
 *   leader.velocityRps = 0.0;
 *   leader.stallCurrentAmps = 45.0;
 *
 * Capabilities (hardware motion profile, encoder-sync support) are settable so
 * tests can exercise both the CTRE/REV-style and Nova-style code paths.
 */
public class FakeMotorIO implements MotorIO {

    private final MotorIOConfig config;

    // ── Simulated feedback (tests write these) ────────────────────────────────
    public double  positionRot;
    public double  velocityRps;
    public double  supplyCurrentAmps;
    public double  statorCurrentAmps;
    public double  stallCurrentAmps;
    public double  motorVoltage;
    public double  temperatureC = 25.0;
    public boolean forwardSoftLimitHit;
    public boolean reverseSoftLimitHit;

    // ── Capabilities (tests may flip) ─────────────────────────────────────────
    public boolean hardwareMotionProfile = true;
    public boolean encoderSyncSupported  = true;

    // ── Recorded commands (tests read these) ─────────────────────────────────
    public enum Mode { NONE, DUTY_CYCLE, VELOCITY, POSITION, STOPPED }
    public Mode    lastMode            = Mode.NONE;
    public double  lastDutyCycle       = Double.NaN;
    public double  lastVelocityRps     = Double.NaN;
    public double  lastPositionRot     = Double.NaN;
    public double  lastFFVolts         = Double.NaN;
    public MotorIO followedLeader;
    public boolean followOpposed;
    public double  lastEncoderReset    = Double.NaN;
    public double[] lastGains;
    public int     commandCount;
    public int     verifyCount;

    public FakeMotorIO(MotorIOConfig config) {
        this.config = config;
    }

    /** Convenience: a CTRE-vendor fake with default settings on the given CAN ID. */
    public static FakeMotorIO withId(int canId) {
        return new FakeMotorIO(MotorIOConfig.builder(canId, MotorVendor.CTRE_TALONFX).build());
    }

    @Override public MotorIOConfig getConfig() { return config; }

    @Override
    public void setDutyCycle(double duty) {
        lastMode = Mode.DUTY_CYCLE; lastDutyCycle = duty; commandCount++;
    }

    @Override
    public void setVelocity(double velocityRps, double ffVolts) {
        lastMode = Mode.VELOCITY; lastVelocityRps = velocityRps; lastFFVolts = ffVolts; commandCount++;
    }

    @Override
    public void setPosition(double positionRot, double ffVolts) {
        lastMode = Mode.POSITION; lastPositionRot = positionRot; lastFFVolts = ffVolts; commandCount++;
    }

    @Override
    public void stop() {
        lastMode = Mode.STOPPED; commandCount++;
    }

    @Override
    public void follow(MotorIO leader, boolean opposeLeader) {
        followedLeader = leader; followOpposed = opposeLeader;
    }

    @Override
    public void setEncoderPosition(double positionRot) {
        lastEncoderReset = positionRot;
        this.positionRot = positionRot;
    }

    @Override
    public void applyGains(double kP, double kI, double kD, double kS, double kV, double kA) {
        lastGains = new double[]{ kP, kI, kD, kS, kV, kA };
    }

    @Override public double  getPositionRot()               { return positionRot; }
    @Override public double  getVelocityRps()               { return velocityRps; }
    @Override public double  getSupplyCurrentAmps()         { return supplyCurrentAmps; }
    @Override public double  getStatorCurrentAmps()         { return statorCurrentAmps; }
    @Override public double  getMotorVoltage()              { return motorVoltage; }
    @Override public double  getTemperatureC()              { return temperatureC; }
    @Override public boolean isForwardSoftLimitHit()        { return forwardSoftLimitHit; }
    @Override public boolean isReverseSoftLimitHit()        { return reverseSoftLimitHit; }
    @Override public double  getStallDetectionCurrentAmps() { return stallCurrentAmps; }
    @Override public boolean hasHardwareMotionProfile()     { return hardwareMotionProfile; }
    @Override public boolean supportsEncoderSync()          { return encoderSyncSupported; }

    @Override
    public ConfigVerifyResult verifyConfig(String label) {
        verifyCount++;
        return new ConfigVerifyResult(label, config.canId, "Fake", true, List.of());
    }
}
