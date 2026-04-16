package frc.robot.util.motors;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.constants.swerveConstants;

/**
 * CTREMechanismUnit.java
 * PATH: src/main/java/frc/robot/util/motors/CTREMechanismUnit.java
 *
 * mechanismUnit implementation for CTRE TalonFX (Phoenix 6).
 * Covers: Kraken X60, Kraken X44, Minion.
 *
 * ─── CTRE-SPECIFIC FEATURES ──────────────────────────────────────────────────
 * The base mechanismUnit API covers all cross-vendor functionality.
 * Additional CTRE-only methods (accessible by storing the concrete type):
 *   setTorqueCurrent()          — Phoenix Pro torque/current control
 *   setMotionMagicExpoPosition()— exponential velocity profile
 *   setNeutralMode()            — runtime coast/brake toggle
 *   applyFullConfiguration()    — replace entire TalonFX config at runtime
 *
 * ─── FEED-FORWARD INJECTION ──────────────────────────────────────────────────
 * Tier-2 + Tier-3 FF (computed by abstract base) injected via .withFeedForward()
 * on every VelocityVoltage, PositionVoltage, and MotionMagicVoltage request.
 * Tier-1 (kS/kV/kA) configured in Slot0 at construction and runs on-controller.
 *
 * ─── GEAR RATIO ──────────────────────────────────────────────────────────────
 * Applied via FeedbackConfigs.SensorToMechanismRatio = config.gearRatio.
 * After this, all Phoenix 6 position/velocity signals report in mechanism-shaft
 * rotations natively — no manual scaling needed anywhere in this class.
 *
 * ─── MOTION MAGIC ────────────────────────────────────────────────────────────
 * Activated automatically when config.motionCruiseVelocityRps > 0.
 * setPosition() uses MotionMagicVoltage in that case, PositionVoltage otherwise.
 * S-curve profile enabled when config.motionJerkRpss3 > 0.
 *
 * ─── SIGNAL UPDATE RATES ─────────────────────────────────────────────────────
 * Follows the same swerveConstants.SIGNAL_UPDATE_HZ / SIGNAL_UPDATE_HZ_TEMP
 * pattern established in swerveModule.java.
 */
public class CTREMechanismUnit extends mechanismUnit {

    // ── Leader motor ──────────────────────────────────────────────────────────
    private final TalonFX leader;

    // ── Follower motors (may be empty) ────────────────────────────────────────
    private final TalonFX[] followers;

    // ── Pre-allocated control requests (never re-allocated in loops) ──────────
    private final DutyCycleOut      dutyCycleRequest;
    private final VelocityVoltage   velocityRequest;
    private final PositionVoltage   positionRequest;
    private final MotionMagicVoltage motionMagicRequest;
    // CTRE-only extras
    private final TorqueCurrentFOC       torqueCurrentRequest;
    private final MotionMagicExpoVoltage motionMagicExpoRequest;

    // ── Hardware ramp flag ────────────────────────────────────────────────────
    private final boolean hardwareRampConfigured;

    // ─────────────────────────────────────────────────────────────────────────
    // Constructor
    // ─────────────────────────────────────────────────────────────────────────

    public CTREMechanismUnit(mechanismConfig config) {
        super(config);

        hardwareRampConfigured = config.openLoopRampSecs > 0.0;

        // Instantiate motors
        leader    = new TalonFX(config.canIds[0]);
        followers = new TalonFX[config.canIds.length - 1];
        for (int i = 0; i < followers.length; i++) {
            followers[i] = new TalonFX(config.canIds[i + 1]);
        }

        // Pre-allocate control requests — reused every loop, no heap allocation
        dutyCycleRequest     = new DutyCycleOut(0).withEnableFOC(true);
        velocityRequest      = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
        positionRequest      = new PositionVoltage(0).withSlot(0).withEnableFOC(true);
        motionMagicRequest   = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
        torqueCurrentRequest = new TorqueCurrentFOC(0);
        motionMagicExpoRequest = new MotionMagicExpoVoltage(0).withSlot(0).withEnableFOC(true);

        configureLeader();
        configureFollowers();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Motor configuration
    // ─────────────────────────────────────────────────────────────────────────

    private void configureLeader() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        // Direction
        cfg.MotorOutput.Inverted = config.inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        // Neutral mode
        cfg.MotorOutput.NeutralMode = config.brakeOnNeutral
            ? NeutralModeValue.Brake
            : NeutralModeValue.Coast;

        // Gear ratio — Phoenix 6 applies this so all signals are in mechanism-shaft units
        cfg.Feedback.SensorToMechanismRatio = config.gearRatio;

        // PID slot 0
        cfg.Slot0.kP = kP; cfg.Slot0.kI = kI; cfg.Slot0.kD = kD;
        cfg.Slot0.kS = kS; cfg.Slot0.kV = kV; cfg.Slot0.kA = kA;

        // Soft limits (degrees → rotations)
        if (!Double.isNaN(config.softLimitForwardDeg)) {
            cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
            cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = config.softLimitForwardDeg / 360.0;
        }
        if (!Double.isNaN(config.softLimitReverseDeg)) {
            cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
            cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = config.softLimitReverseDeg / 360.0;
        }

        // Current limits
        cfg.CurrentLimits.SupplyCurrentLimit       = config.supplyCurrentLimitAmps;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit       = config.statorCurrentLimitAmps;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        // Ramp rates
        if (config.openLoopRampSecs > 0.0) {
            cfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = config.openLoopRampSecs;
            cfg.OpenLoopRamps.VoltageOpenLoopRampPeriod   = config.openLoopRampSecs;
        }
        if (config.closedLoopRampSecs > 0.0) {
            cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = config.closedLoopRampSecs;
        }

        // Motion Magic
        if (config.motionCruiseVelocityRps > 0.0) {
            cfg.MotionMagic.MotionMagicCruiseVelocity = config.motionCruiseVelocityRps;
            cfg.MotionMagic.MotionMagicAcceleration   = config.motionAccelerationRpss;
            cfg.MotionMagic.MotionMagicJerk            = config.motionJerkRpss3;
        }

        leader.getConfigurator().apply(cfg);

        // CAN signal update rates — matches swerveModule pattern
        BaseStatusSignal.setUpdateFrequencyForAll(swerveConstants.SIGNAL_UPDATE_HZ,
            leader.getVelocity(),
            leader.getPosition(),
            leader.getSupplyCurrent(),
            leader.getStatorCurrent(),
            leader.getMotorVoltage(),
            leader.getDutyCycle()
        );
        BaseStatusSignal.setUpdateFrequencyForAll(swerveConstants.SIGNAL_UPDATE_HZ_TEMP,
            leader.getDeviceTemp()
        );
        if (swerveConstants.OPTIMIZE_CAN_UTILIZATION) leader.optimizeBusUtilization();
    }

    private void configureFollowers() {
        for (int i = 0; i < followers.length; i++) {
            boolean invert = i < config.followerInverted.length && config.followerInverted[i];
            motorConstants.FollowMode mode = i < config.followerModes.length
                ? config.followerModes[i] : motorConstants.FollowMode.NONE;
            // Resolve the CAN ID of this follower's designated leader
            int leaderCanId = config.canIds[config.followerLeaderIndices[i]];

            if (mode == motorConstants.FollowMode.MECHANICAL) {
                // Hardware follower — zero Rio CPU per cycle after this call.
                // Follows the designated leader CAN ID (may be another follower).
                MotorAlignmentValue alignment = invert
                    ? MotorAlignmentValue.Opposed
                    : MotorAlignmentValue.Aligned;
                followers[i].setControl(new Follower(leaderCanId, alignment));
            } else if (mode == motorConstants.FollowMode.ENCODER_SYNC) {
                // Software follower — encoder-sync loop in abstract base handles correction.
                // Configure independently (no Follower request).
                TalonFXConfiguration followerCfg = new TalonFXConfiguration();
                followerCfg.MotorOutput.Inverted = invert
                    ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive;
                followerCfg.MotorOutput.NeutralMode = config.brakeOnNeutral
                    ? NeutralModeValue.Brake : NeutralModeValue.Coast;
                followerCfg.Feedback.SensorToMechanismRatio = config.gearRatio;
                followerCfg.CurrentLimits.SupplyCurrentLimit       = config.supplyCurrentLimitAmps;
                followerCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
                followerCfg.CurrentLimits.StatorCurrentLimit       = config.statorCurrentLimitAmps;
                followerCfg.CurrentLimits.StatorCurrentLimitEnable = true;
                followers[i].getConfigurator().apply(followerCfg);

                BaseStatusSignal.setUpdateFrequencyForAll(swerveConstants.SIGNAL_UPDATE_HZ,
                    followers[i].getPosition(), followers[i].getVelocity(),
                    followers[i].getStatorCurrent());
                if (swerveConstants.OPTIMIZE_CAN_UTILIZATION) followers[i].optimizeBusUtilization();
            }
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // mechanismUnit abstract method implementations
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    protected void applyDutyCycleImpl(double duty) {
        leader.setControl(dutyCycleRequest.withOutput(duty));
    }

    @Override
    protected void applyVelocityImpl(double rps, double ffVolts) {
        leader.setControl(velocityRequest.withVelocity(rps).withFeedForward(ffVolts));
    }

    @Override
    protected void applyPositionImpl(double rotations, double ffVolts) {
        if (config.motionCruiseVelocityRps > 0.0) {
            leader.setControl(motionMagicRequest.withPosition(rotations).withFeedForward(ffVolts));
        } else {
            leader.setControl(positionRequest.withPosition(rotations).withFeedForward(ffVolts));
        }
    }

    @Override
    protected void stopImpl() {
        leader.stopMotor();
    }

    @Override
    protected void applyPIDToController() {
        // Update Slot0 only — avoids touching any other config
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = kP; slot0.kI = kI; slot0.kD = kD;
        slot0.kS = kS; slot0.kV = kV; slot0.kA = kA;
        leader.getConfigurator().apply(slot0);
    }

    @Override
    protected double getVelocityImpl() {
        return leader.getVelocity().getValueAsDouble();
    }

    @Override
    protected double getPositionImpl() {
        return leader.getPosition().getValueAsDouble();
    }

    @Override
    protected double getSupplyCurrentImpl() {
        return leader.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    protected double getStatorCurrentImpl() {
        return leader.getStatorCurrent().getValueAsDouble();
    }

    @Override
    protected double getMotorVoltageImpl() {
        return leader.getMotorVoltage().getValueAsDouble();
    }

    @Override
    protected double getTemperatureImpl() {
        return leader.getDeviceTemp().getValueAsDouble();
    }

    @Override
    protected boolean isForwardLimitHitImpl() {
        return leader.getFault_ForwardSoftLimit().getValue();
    }

    @Override
    protected boolean isReverseLimitHitImpl() {
        return leader.getFault_ReverseSoftLimit().getValue();
    }

    @Override
    protected double getFollowerPositionImpl(int followerIndex) {
        return followers[followerIndex - 1].getPosition().getValueAsDouble();
    }

    @Override
    protected void applyFollowerCorrectionImpl(int followerIndex, double duty) {
        followers[followerIndex - 1].setControl(new DutyCycleOut(duty).withEnableFOC(true));
    }

    @Override
    protected double getDetectionCurrentImpl() {
        // Stator current is directly proportional to motor torque — ideal for stall detection
        return leader.getStatorCurrent().getValueAsDouble();
    }

    @Override
    protected double getFollowerDetectionCurrentImpl(int followerIndex) {
        return followers[followerIndex - 1].getStatorCurrent().getValueAsDouble();
    }

    @Override
    protected double getFollowerVelocityImpl(int followerIndex) {
        return followers[followerIndex - 1].getVelocity().getValueAsDouble();
    }

    @Override
    protected void resetLeadEncoderImpl(double positionRot) {
        leader.setPosition(positionRot);
    }

    @Override
    protected void resetFollowerEncoderImpl(int followerIndex, double positionRot) {
        followers[followerIndex - 1].setPosition(positionRot);
    }

    @Override
    protected boolean usesHardwareDutyCycleRamp() {
        return hardwareRampConfigured;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // CTRE-specific extras — accessible by storing the concrete type
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * TorqueCurrentFOC control — directly commands stator current (torque).
     * Requires Phoenix Pro license.
     * Back-EMF is handled internally by FOC; set kV = 0 in pid[] when using this.
     *
     * @param amps          target stator current in amps (positive = forward)
     * @param maxDutyCycle  maximum duty cycle magnitude allowed during control [0, 1]
     */
    public void setTorqueCurrent(double amps, double maxDutyCycle) {
        leader.setControl(torqueCurrentRequest.withOutput(amps)
            .withMaxAbsDutyCycle(maxDutyCycle));
    }

    /**
     * MotionMagicExpo position control — uses an exponential velocity profile
     * that automatically scales velocity to the available remaining distance.
     * Smoother than the trapezoidal profile for point-to-point moves.
     * Configured via MotionMagicExpo_kV and MotionMagicExpo_kA in TalonFX config.
     *
     * @param positionDeg target position (degrees, mechanism shaft)
     * @param ffVolts     feed-forward voltage addend (volts)
     */
    public void setMotionMagicExpoPosition(double positionDeg, double ffVolts) {
        leader.setControl(motionMagicExpoRequest
            .withPosition(positionDeg / 360.0)
            .withFeedForward(ffVolts));
    }

    /**
     * Change neutral (idle) mode at runtime.
     * Useful for switching to coast during disabled to allow manual manipulation.
     *
     * @param brake true = brake mode, false = coast mode
     */
    public void setNeutralMode(boolean brake) {
        MotorOutputConfigs cfg = new MotorOutputConfigs();
        cfg.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        leader.getConfigurator().apply(cfg);
    }

    /**
     * Replace the entire TalonFX configuration at runtime.
     * Use sparingly — full configs take longer to apply over CAN than slot-only updates.
     * Does not update the internal kP/kI/kD/kS/kV/kA live-tune state.
     *
     * @param newConfig fully-populated TalonFXConfiguration to apply
     */
    public void applyFullConfiguration(TalonFXConfiguration newConfig) {
        leader.getConfigurator().apply(newConfig);
    }

    /**
     * Direct access to the underlying leader TalonFX.
     * For advanced use only — prefer the mechanismUnit public API where possible.
     */
    public TalonFX getLeaderMotor() {
        return leader;
    }
}
