package frc.lib.hardware.motor;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.util.ArrayList;
import java.util.List;

import frc.lib.util.ConfigVerifyResult;

/**
 * CTREMotor.java
 * PATH: src/main/java/frc/lib/hardware/motor/CTREMotor.java
 *
 * MotorIO for CTRE TalonFX (Phoenix 6). Covers Kraken X60, Kraken X44, Minion.
 *
 * ─── GEAR RATIO ──────────────────────────────────────────────────────────────
 * Applied via SensorToMechanismRatio, so every Phoenix 6 position/velocity
 * signal reports in mechanism-shaft units natively.
 *
 * ─── FEED-FORWARD ────────────────────────────────────────────────────────────
 * kS/kV/kA run on-controller in Slot0. Rio-computed FF is injected each cycle
 * via .withFeedForward() on every velocity/position request.
 *
 * ─── MOTION MAGIC ────────────────────────────────────────────────────────────
 * setPosition() uses MotionMagicVoltage when motionCruiseVelocityRps > 0,
 * PositionVoltage otherwise. S-curve when motionJerkRpss3 > 0.
 *
 * ─── CONTINUOUS WRAP ─────────────────────────────────────────────────────────
 * ClosedLoopGeneral.ContinuousWrap — Phoenix 6 takes the shortest path.
 *
 * ─── CTRE-ONLY EXTRAS ────────────────────────────────────────────────────────
 * setTorqueCurrent(), setMotionMagicExpoPosition(), setNeutralMode(),
 * getTalonFX() — reach them by casting a MotorIO to CTREMotor.
 */
public class CTREMotor implements MotorIO {

    private static final String VENDOR = MotorVendor.CTRE_TALONFX.displayName;

    private final MotorIOConfig config;
    private final TalonFX       motor;

    // Live gains — updated by applyGains() so verifyConfig() re-applies tuned values
    private double kP, kI, kD, kS, kV, kA;

    // Pre-allocated control requests — never re-allocated in a loop
    private final DutyCycleOut           dutyCycleRequest       = new DutyCycleOut(0).withEnableFOC(true);
    private final VelocityVoltage        velocityRequest        = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
    private final PositionVoltage        positionRequest        = new PositionVoltage(0).withSlot(0).withEnableFOC(true);
    private final MotionMagicVoltage     motionMagicRequest     = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
    private final TorqueCurrentFOC       torqueCurrentRequest   = new TorqueCurrentFOC(0);
    private final MotionMagicExpoVoltage motionMagicExpoRequest = new MotionMagicExpoVoltage(0).withSlot(0).withEnableFOC(true);

    public CTREMotor(MotorIOConfig config) {
        this.config = config;
        kP = config.kP; kI = config.kI; kD = config.kD;
        kS = config.kS; kV = config.kV; kA = config.kA;

        motor = new TalonFX(config.canId);
        motor.getConfigurator().apply(buildConfiguration());

        BaseStatusSignal.setUpdateFrequencyForAll(config.signalUpdateHz,
            motor.getVelocity(), motor.getPosition(),
            motor.getSupplyCurrent(), motor.getStatorCurrent(),
            motor.getSupplyVoltage(), motor.getMotorVoltage(), motor.getDutyCycle());
        BaseStatusSignal.setUpdateFrequencyForAll(config.tempSignalUpdateHz,
            motor.getDeviceTemp());
        if (config.optimizeCanUtilization) motor.optimizeBusUtilization();
    }

    /** The complete TalonFX configuration for this motor — used at construction and by verifyConfig(). */
    private TalonFXConfiguration buildConfiguration() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.Inverted = invertedValue(config.inverted);
        cfg.MotorOutput.NeutralMode = neutralValue(config.brakeOnNeutral);

        cfg.Feedback.SensorToMechanismRatio = config.gearRatio;

        cfg.Slot0.kP = kP; cfg.Slot0.kI = kI; cfg.Slot0.kD = kD;
        cfg.Slot0.kS = kS; cfg.Slot0.kV = kV; cfg.Slot0.kA = kA;

        if (!Double.isNaN(config.softLimitForwardRot)) {
            cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
            cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = config.softLimitForwardRot;
        }
        if (!Double.isNaN(config.softLimitReverseRot)) {
            cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
            cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = config.softLimitReverseRot;
        }

        cfg.CurrentLimits.SupplyCurrentLimit       = config.supplyCurrentLimitAmps;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit       = config.statorCurrentLimitAmps;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        if (config.openLoopRampSecs > 0.0) {
            cfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = config.openLoopRampSecs;
            cfg.OpenLoopRamps.VoltageOpenLoopRampPeriod   = config.openLoopRampSecs;
        }
        if (config.closedLoopRampSecs > 0.0) {
            cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = config.closedLoopRampSecs;
        }

        if (config.hasMotionProfile()) {
            cfg.MotionMagic.MotionMagicCruiseVelocity = config.motionCruiseVelocityRps;
            cfg.MotionMagic.MotionMagicAcceleration   = config.motionAccelerationRpss;
            cfg.MotionMagic.MotionMagicJerk           = config.motionJerkRpss3;
        }

        cfg.ClosedLoopGeneral.ContinuousWrap = config.continuousWrap;
        return cfg;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // MotorIO — commands
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public MotorIOConfig getConfig() { return config; }

    @Override
    public void setDutyCycle(double duty) {
        motor.setControl(dutyCycleRequest.withOutput(duty));
    }

    @Override
    public void setVelocity(double velocityRps, double ffVolts) {
        motor.setControl(velocityRequest.withVelocity(velocityRps).withFeedForward(ffVolts));
    }

    @Override
    public void setPosition(double positionRot, double ffVolts) {
        if (config.hasMotionProfile()) {
            motor.setControl(motionMagicRequest.withPosition(positionRot).withFeedForward(ffVolts));
        } else {
            motor.setControl(positionRequest.withPosition(positionRot).withFeedForward(ffVolts));
        }
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void follow(MotorIO leader, boolean opposeLeader) {
        // Hardware follower — zero Rio CPU per cycle after this call
        motor.setControl(new Follower(leader.getConfig().canId,
            opposeLeader ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
    }

    @Override
    public void setEncoderPosition(double positionRot) {
        // With SensorToMechanismRatio set, setPosition() takes mechanism shaft rotations
        motor.setPosition(positionRot);
    }

    @Override
    public void applyGains(double kP, double kI, double kD, double kS, double kV, double kA) {
        this.kP = kP; this.kI = kI; this.kD = kD;
        this.kS = kS; this.kV = kV; this.kA = kA;
        // Slot0 only — avoids touching any other config
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = kP; slot0.kI = kI; slot0.kD = kD;
        slot0.kS = kS; slot0.kV = kV; slot0.kA = kA;
        motor.getConfigurator().apply(slot0);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // MotorIO — feedback
    // ─────────────────────────────────────────────────────────────────────────

    @Override public double getPositionRot()        { return motor.getPosition().getValueAsDouble(); }
    @Override public double getVelocityRps()        { return motor.getVelocity().getValueAsDouble(); }
    @Override public double getSupplyCurrentAmps()  { return motor.getSupplyCurrent().getValueAsDouble(); }
    @Override public double getStatorCurrentAmps()  { return motor.getStatorCurrent().getValueAsDouble(); }
    @Override public double getMotorVoltage()       { return motor.getMotorVoltage().getValueAsDouble(); }
    @Override public double getTemperatureC()       { return motor.getDeviceTemp().getValueAsDouble(); }
    @Override public boolean isForwardSoftLimitHit() { return motor.getFault_ForwardSoftLimit().getValue(); }
    @Override public boolean isReverseSoftLimitHit() { return motor.getFault_ReverseSoftLimit().getValue(); }

    /** Stator current is directly proportional to motor torque — ideal for stall detection. */
    @Override
    public double getStallDetectionCurrentAmps() { return getStatorCurrentAmps(); }

    @Override public boolean hasHardwareMotionProfile() { return true; }
    @Override public boolean supportsEncoderSync()      { return true; }

    // ─────────────────────────────────────────────────────────────────────────
    // CTRE-only extras — reach by casting MotorIO to CTREMotor
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * TorqueCurrentFOC control — directly commands stator current (torque).
     * Requires Phoenix Pro license. Set kV = 0 when using this.
     *
     * @param amps         target stator current in amps (positive = forward)
     * @param maxDutyCycle maximum duty cycle magnitude allowed during control [0, 1]
     */
    public void setTorqueCurrent(double amps, double maxDutyCycle) {
        motor.setControl(torqueCurrentRequest.withOutput(amps).withMaxAbsDutyCycle(maxDutyCycle));
    }

    /**
     * MotionMagicExpo position control — exponential velocity profile.
     * Configured via MotionMagicExpo_kV / _kA in the TalonFX config.
     *
     * @param positionRot target position (mechanism shaft rotations)
     * @param ffVolts     feed-forward voltage addend (volts)
     */
    public void setMotionMagicExpoPosition(double positionRot, double ffVolts) {
        motor.setControl(motionMagicExpoRequest.withPosition(positionRot).withFeedForward(ffVolts));
    }

    /** Change neutral mode at runtime (e.g. coast while disabled for manual handling). */
    public void setNeutralMode(boolean brake) {
        MotorOutputConfigs cfg = new MotorOutputConfigs();
        cfg.Inverted    = invertedValue(config.inverted);
        cfg.NeutralMode = neutralValue(brake);
        motor.getConfigurator().apply(cfg);
    }

    /** Direct access to the TalonFX. Advanced use only — prefer the MotorIO API. */
    public TalonFX getTalonFX() {
        return motor;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Startup verification
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public ConfigVerifyResult verifyConfig(String label) {
        TalonFXConfiguration cfg = buildConfiguration();

        boolean applyOk = false;
        for (int attempt = 0; attempt < VERIFY_RETRIES; attempt++) {
            StatusCode sc = motor.getConfigurator().apply(cfg);
            if (sc.isOK()) { applyOk = true; break; }
            try { Thread.sleep(VERIFY_DELAY_MS); } catch (InterruptedException ignored) {}
        }
        if (!applyOk) return new ConfigVerifyResult(label, config.canId, VENDOR, false, List.of());

        TalonFXConfiguration rb = new TalonFXConfiguration();
        motor.getConfigurator().refresh(rb);
        List<String> mismatches = new ArrayList<>();

        InvertedValue wantInv = invertedValue(config.inverted);
        if (rb.MotorOutput.Inverted != wantInv)
            mismatches.add("inversion: expected " + wantInv + " got " + rb.MotorOutput.Inverted);
        NeutralModeValue wantNeutral = neutralValue(config.brakeOnNeutral);
        if (rb.MotorOutput.NeutralMode != wantNeutral)
            mismatches.add("neutralMode: expected " + wantNeutral + " got " + rb.MotorOutput.NeutralMode);
        if (Math.abs(rb.Feedback.SensorToMechanismRatio - config.gearRatio) > 1e-6)
            mismatches.add("sensorToMechanismRatio: expected " + config.gearRatio
                           + " got " + rb.Feedback.SensorToMechanismRatio);
        if (Math.abs(rb.CurrentLimits.SupplyCurrentLimit - config.supplyCurrentLimitAmps) > 0.5)
            mismatches.add("supplyCurrentLimit: expected " + config.supplyCurrentLimitAmps
                           + "A got " + rb.CurrentLimits.SupplyCurrentLimit + "A");
        if (Math.abs(rb.CurrentLimits.StatorCurrentLimit - config.statorCurrentLimitAmps) > 0.5)
            mismatches.add("statorCurrentLimit: expected " + config.statorCurrentLimitAmps
                           + "A got " + rb.CurrentLimits.StatorCurrentLimit + "A");

        return new ConfigVerifyResult(label, config.canId, VENDOR, true, mismatches);
    }

    private static InvertedValue invertedValue(boolean inverted) {
        return inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    }

    private static NeutralModeValue neutralValue(boolean brake) {
        return brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    }
}
