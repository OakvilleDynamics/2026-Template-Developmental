package frc.robot.util.motors;

import com.revrobotics.REVLibError;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfigAccessor;

import java.util.ArrayList;
import java.util.List;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * REVMechanismUnit.java
 * PATH: src/main/java/frc/robot/util/motors/REVMechanismUnit.java
 *
 * mechanismUnit implementation for REV Robotics SparkMax and SparkFlex (REVLib 2025+).
 * Covers: NEO, NEO 550 (SparkMax), NEO Vortex (SparkFlex).
 *
 * ─── FEED-FORWARD INJECTION ──────────────────────────────────────────────────
 * Tier-2 + Tier-3 FF (computed by abstract base) injected via the 4th/5th
 * arguments to setReference(..., arbFF, ArbFFUnits.kVoltage) each cycle.
 * Tier-1 (kS/kV/kA) configured in ClosedLoopConfig slot 0 and runs on-controller.
 *
 * NOTE: REV does not use kG or kCos on-controller in this implementation.
 * Those functions are handled more consistently by the tier-2 lambda in the
 * abstract base, which runs across all vendors from one code path.
 *
 * ─── GEAR RATIO ──────────────────────────────────────────────────────────────
 * Applied via positionConversionFactor = 1.0 / gearRatio (rotations)
 * and velocityConversionFactor = 1.0 / (gearRatio × 60) (RPS, not RPM).
 * After this, all encoder reads return mechanism-shaft units natively.
 *
 * ─── MOTION PROFILE ──────────────────────────────────────────────────────────
 * MAXMotion activated when config.motionCruiseVelocityRps > 0.
 * setPosition() uses ControlType.kMAXMotionPositionControl in that case.
 * motionJerkRpss3 is CTRE-only; ignored here.
 *
 * ─── PID LIVE TUNING ─────────────────────────────────────────────────────────
 * Applying new gains requires a configure() call on REV controllers, which
 * briefly reconfigures over CAN. PersistMode.kNoPersistParameters is used to
 * avoid flash write wear. This is acceptable for development tuning sessions
 * but should not be triggered repeatedly at competition with time-critical code.
 *
 * ─── FOLLOWERS ───────────────────────────────────────────────────────────────
 * MECHANICAL: native REV follow() with optional invert.
 * ENCODER_SYNC: followers run independently; abstract base reads encoder positions
 *               and applies duty cycle corrections.
 */
public class REVMechanismUnit extends mechanismUnit {

    // ── Leader motor ──────────────────────────────────────────────────────────
    private final SparkBase leader;
    private final SparkClosedLoopController leaderController;

    // ── Follower motors (may be empty) ────────────────────────────────────────
    private final SparkBase[] followers;

    // ── Flags ─────────────────────────────────────────────────────────────────
    private final boolean isFlex;
    private final boolean hardwareRampConfigured;
    private final boolean motionProfileEnabled;

    // ─────────────────────────────────────────────────────────────────────────
    // Constructor
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * @param config mechanismConfig for this mechanism
     * @param isFlex true = SparkFlex (NEO Vortex); false = SparkMax (NEO / NEO 550)
     */
    public REVMechanismUnit(mechanismConfig config, boolean isFlex) {
        super(config);
        this.isFlex                 = isFlex;
        this.hardwareRampConfigured = config.openLoopRampSecs > 0.0;
        this.motionProfileEnabled   = config.motionCruiseVelocityRps > 0.0;

        // Instantiate leader
        leader = isFlex
            ? new SparkFlex(config.canIds[0], MotorType.kBrushless)
            : new SparkMax(config.canIds[0], MotorType.kBrushless);
        leaderController = leader.getClosedLoopController();

        // Instantiate followers
        followers = new SparkBase[config.canIds.length - 1];
        for (int i = 0; i < followers.length; i++) {
            followers[i] = isFlex
                ? new SparkFlex(config.canIds[i + 1], MotorType.kBrushless)
                : new SparkMax(config.canIds[i + 1], MotorType.kBrushless);
        }

        configureLeader();
        configureFollowers();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Motor configuration
    // ─────────────────────────────────────────────────────────────────────────

    private void configureLeader() {
        SparkBaseConfig cfg = isFlex ? new SparkFlexConfig() : new SparkMaxConfig();

        // Direction and neutral mode
        cfg.inverted(config.inverted);
        cfg.idleMode(config.brakeOnNeutral ? IdleMode.kBrake : IdleMode.kCoast);

        // Gear ratio — converts native motor rotations/RPM to mechanism-shaft rotations/RPS
        cfg.encoder
            .positionConversionFactor(1.0 / config.gearRatio)
            .velocityConversionFactor(1.0 / (config.gearRatio * 60.0));

        // Current limits
        cfg.smartCurrentLimit((int) config.supplyCurrentLimitAmps);

        // Ramp rates
        if (config.openLoopRampSecs > 0.0)   cfg.openLoopRampRate(config.openLoopRampSecs);
        if (config.closedLoopRampSecs > 0.0)  cfg.closedLoopRampRate(config.closedLoopRampSecs);

        // Soft limits
        if (!Double.isNaN(config.softLimitForwardDeg)) {
            cfg.softLimit
                .forwardSoftLimit((float)(config.softLimitForwardDeg / 360.0))
                .forwardSoftLimitEnabled(true);
        }
        if (!Double.isNaN(config.softLimitReverseDeg)) {
            cfg.softLimit
                .reverseSoftLimit((float)(config.softLimitReverseDeg / 360.0))
                .reverseSoftLimitEnabled(true);
        }

        // PID slot 0 — kS/kV/kA run on controller; kG/kCos intentionally not set
        // (gravity compensation handled by tier-2 lambda in the abstract base)
        cfg.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kP, kI, kD)
            .apply(new FeedForwardConfig().kV(kV))
            .iZone(0);

        // MAXMotion profile
        if (motionProfileEnabled) {
            cfg.closedLoop.maxMotion
                .cruiseVelocity(config.motionCruiseVelocityRps)
                .maxAcceleration(config.motionAccelerationRpss)
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        }

        leader.configure(cfg,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    private void configureFollowers() {
        for (int i = 0; i < followers.length; i++) {
            boolean invert = i < config.followerInverted.length && config.followerInverted[i];
            motorConstants.FollowMode mode = i < config.followerModes.length
                ? config.followerModes[i] : motorConstants.FollowMode.NONE;
            // Resolve the SparkBase instance for this follower's designated leader.
            // leaderIndex 0 → main leader; N → followers[N-1].
            int leaderIndex = config.followerLeaderIndices[i];
            SparkBase leaderMotor = leaderIndex == 0 ? leader : followers[leaderIndex - 1];

            if (mode == motorConstants.FollowMode.MECHANICAL) {
                // Native REV hardware follow against designated leader motor instance.
                SparkBaseConfig followerCfg = isFlex ? new SparkFlexConfig() : new SparkMaxConfig();
                followerCfg.follow(leaderMotor, invert);
                followers[i].configure(followerCfg,
                    ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);

            } else if (mode == motorConstants.FollowMode.ENCODER_SYNC) {
                // Independent configuration — abstract base handles sync corrections.
                SparkBaseConfig followerCfg = isFlex ? new SparkFlexConfig() : new SparkMaxConfig();
                followerCfg.inverted(invert);
                followerCfg.idleMode(config.brakeOnNeutral ? IdleMode.kBrake : IdleMode.kCoast);
                followerCfg.encoder
                    .positionConversionFactor(1.0 / config.gearRatio)
                    .velocityConversionFactor(1.0 / (config.gearRatio * 60.0));
                followerCfg.smartCurrentLimit((int) config.supplyCurrentLimitAmps);
                followers[i].configure(followerCfg,
                    ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);
            }
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // mechanismUnit abstract method implementations
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    protected void applyDutyCycleImpl(double duty) {
        leader.set(duty);
    }

    @Override
    protected void applyVelocityImpl(double rps, double ffVolts) {
        leaderController.setSetpoint(rps, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVolts, ArbFFUnits.kVoltage);
    }

    @Override
    protected void applyPositionImpl(double rotations, double ffVolts) {
        ControlType ct = motionProfileEnabled
            ? ControlType.kMAXMotionPositionControl
            : ControlType.kPosition;
        leaderController.setSetpoint(rotations, ct, ClosedLoopSlot.kSlot0, ffVolts, ArbFFUnits.kVoltage);
    }

    @Override
    protected void stopImpl() {
        leader.stopMotor();
    }

    @Override
    protected void applyPIDToController() {
        // REV requires a full configure() call to update PID gains.
        // kNoPersistParameters avoids flash write wear during live tuning.
        // kNoResetSafeParameters preserves all other settings.
        SparkBaseConfig cfg = isFlex ? new SparkFlexConfig() : new SparkMaxConfig();
        cfg.closedLoop
            .pid(kP, kI, kD)
            .apply(new FeedForwardConfig().kV(kV));
        leader.configure(cfg,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters);
    }

    @Override
    protected double getVelocityImpl() {
        // Encoder velocity conversion factor already converts to mechanism-shaft RPS
        return leader.getEncoder().getVelocity();
    }

    @Override
    protected double getPositionImpl() {
        // Encoder position conversion factor already converts to mechanism-shaft rotations
        return leader.getEncoder().getPosition();
    }

    @Override
    protected double getSupplyCurrentImpl() {
        return leader.getOutputCurrent();
    }

    @Override
    protected double getStatorCurrentImpl() {
        // REV does not expose a separate stator current signal
        return 0.0;
    }

    @Override
    protected double getMotorVoltageImpl() {
        return leader.getBusVoltage() * leader.getAppliedOutput();
    }

    @Override
    protected double getTemperatureImpl() {
        return leader.getMotorTemperature();
    }

    @Override
    protected boolean isForwardLimitHitImpl() {
        return leader.getForwardSoftLimit().isReached();
    }

    @Override
    protected boolean isReverseLimitHitImpl() {
        return leader.getReverseSoftLimit().isReached();
    }

    @Override
    protected double getFollowerPositionImpl(int followerIndex) {
        return followers[followerIndex - 1].getEncoder().getPosition();
    }

    @Override
    protected void applyFollowerCorrectionImpl(int followerIndex, double duty) {
        followers[followerIndex - 1].set(duty);
    }

    @Override
    protected double getDetectionCurrentImpl() {
        // REV does not expose a separate stator current; output current is the
        // closest approximation and works well for stall detection in practice
        return leader.getOutputCurrent();
    }

    @Override
    protected double getFollowerDetectionCurrentImpl(int followerIndex) {
        return followers[followerIndex - 1].getOutputCurrent();
    }

    @Override
    protected double getFollowerVelocityImpl(int followerIndex) {
        // Encoder conversion factor already converts to mechanism-shaft RPS
        return followers[followerIndex - 1].getEncoder().getVelocity();
    }

    @Override
    protected void resetLeadEncoderImpl(double positionRot) {
        // Motor must be stopped before this is called (homing stops it first)
        leader.getEncoder().setPosition(positionRot);
    }

    @Override
    protected void resetFollowerEncoderImpl(int followerIndex, double positionRot) {
        followers[followerIndex - 1].getEncoder().setPosition(positionRot);
    }

    @Override
    protected boolean usesHardwareDutyCycleRamp() {
        return hardwareRampConfigured;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // REV-specific extras
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Direct access to the underlying leader SparkBase.
     * For advanced use only — prefer the mechanismUnit public API where possible.
     */
    public SparkBase getLeaderMotor() {
        return leader;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // ConfigVerifiable implementation
    // ─────────────────────────────────────────────────────────────────────────

    private static final int VERIFY_RETRIES  = 5;
    private static final int VERIFY_DELAY_MS = 50;

    @Override
    public List<ConfigVerifyResult> verifyConfig() {
        List<ConfigVerifyResult> results = new ArrayList<>();
        results.add(verifyDevice("Leader", leader, config.inverted, config.brakeOnNeutral,
                                 config.supplyCurrentLimitAmps));

        for (int i = 0; i < followers.length; i++) {
            if (i < config.followerModes.length
                    && config.followerModes[i] == motorConstants.FollowMode.ENCODER_SYNC) {
                boolean followerInverted = i < config.followerInverted.length
                                          && config.followerInverted[i];
                results.add(verifyDevice(
                    "Follower[" + (i + 1) + "]",
                    followers[i],
                    followerInverted,
                    config.brakeOnNeutral,
                    config.supplyCurrentLimitAmps));
            }
        }
        return results;
    }

    private ConfigVerifyResult verifyDevice(
            String role, SparkBase motor,
            boolean expectedInverted, boolean expectedBrake,
            double expectedSupplyAmps) {

        String label  = config.name + " " + role;
        int    canId  = motor.getDeviceId();
        String vendor = isFlex ? "REV SparkFlex" : "REV SparkMax";

        SparkBaseConfig cfg = buildVerifyConfig(expectedInverted, expectedBrake, expectedSupplyAmps);

        // ── Retry apply ───────────────────────────────────────────────────────
        boolean applyOk = false;
        for (int attempt = 0; attempt < VERIFY_RETRIES; attempt++) {
            REVLibError err = motor.configure(cfg,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
            if (err == REVLibError.kOk) { applyOk = true; break; }
            try { Thread.sleep(VERIFY_DELAY_MS); } catch (InterruptedException ignored) {}
        }

        if (!applyOk) {
            return new ConfigVerifyResult(label, canId, vendor, false, List.of());
        }

        // ── Read back and diff via configAccessor ─────────────────────────────
        // configAccessor lives on SparkMax/SparkFlex, not SparkBase — cast to reach it.
        List<String> mismatches = new ArrayList<>();
        SparkBaseConfigAccessor accessor = getAccessor(motor);

        if (accessor != null) {
            boolean readInverted = accessor.getInverted();
            if (readInverted != expectedInverted) {
                mismatches.add("inversion: expected " + expectedInverted + " got " + readInverted);
            }

            IdleMode wantIdle = expectedBrake ? IdleMode.kBrake : IdleMode.kCoast;
            IdleMode readIdle = accessor.getIdleMode();
            if (readIdle != wantIdle) {
                mismatches.add("idleMode: expected " + wantIdle + " got " + readIdle);
            }

            int readLimit = accessor.getSmartCurrentLimit();
            int wantLimit = (int) expectedSupplyAmps;
            if (Math.abs(readLimit - wantLimit) > 1) {
                mismatches.add("smartCurrentLimit: expected " + wantLimit + "A got " + readLimit + "A");
            }
        }

        return new ConfigVerifyResult(label, canId, vendor, true, mismatches);
    }

    /** Returns the SparkBaseConfigAccessor for readback — field lives on SparkMax/SparkFlex, not SparkBase. */
    private static SparkBaseConfigAccessor getAccessor(SparkBase motor) {
        if (motor instanceof SparkMax)  return ((SparkMax)  motor).configAccessor;
        if (motor instanceof SparkFlex) return ((SparkFlex) motor).configAccessor;
        return null;
    }

    private SparkBaseConfig buildVerifyConfig(boolean inverted, boolean brake, double supplyAmps) {
        SparkBaseConfig cfg = isFlex ? new SparkFlexConfig() : new SparkMaxConfig();
        cfg.inverted(inverted);
        cfg.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
        cfg.smartCurrentLimit((int) supplyAmps);
        return cfg;
    }
}
