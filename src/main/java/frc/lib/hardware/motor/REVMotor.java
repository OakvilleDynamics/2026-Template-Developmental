package frc.lib.hardware.motor;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfigAccessor;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.util.ArrayList;
import java.util.List;

import frc.lib.util.ConfigVerifyResult;

/**
 * REVMotor.java
 * PATH: src/main/java/frc/lib/hardware/motor/REVMotor.java
 *
 * MotorIO for REV SparkMax (NEO, NEO 550) and SparkFlex (NEO Vortex), REVLib 2026.
 * The vendor value selects the controller: REV_SPARKMAX or REV_SPARKFLEX.
 *
 * ─── GEAR RATIO ──────────────────────────────────────────────────────────────
 * positionConversionFactor = 1 / gearRatio (rotations)
 * velocityConversionFactor = 1 / (gearRatio × 60) (RPS, not RPM)
 * All encoder reads return mechanism-shaft units natively.
 *
 * ─── FEED-FORWARD ────────────────────────────────────────────────────────────
 * kP/kI/kD and kV run on-controller in slot 0. kS/kA are not set on-controller.
 * Rio-computed FF injected via setSetpoint(..., arbFF, ArbFFUnits.kVoltage).
 *
 * ─── MOTION PROFILE ──────────────────────────────────────────────────────────
 * MAXMotion (trapezoidal) when motionCruiseVelocityRps > 0. Jerk is ignored.
 *
 * ─── CONTINUOUS WRAP ─────────────────────────────────────────────────────────
 * positionWrappingEnabled over [-0.5, 0.5) mechanism rotations.
 *
 * ─── LIVE PID TUNING ─────────────────────────────────────────────────────────
 * Gain updates use kNoResetSafeParameters + kNoPersistParameters — no flash
 * wear and no other settings touched. A configure() call briefly occupies CAN,
 * so avoid repeated gain changes in time-critical match code.
 *
 * ─── THROUGH BORE ENCODER ────────────────────────────────────────────────────
 * getSpark() exposes the controller so REVThroughBoreEncoder can share it.
 */
public class REVMotor implements MotorIO {

    private final MotorIOConfig             config;
    private final boolean                   isFlex;
    private final SparkBase                 spark;
    private final SparkClosedLoopController controller;
    private final String                    vendorName;

    // Live gains — updated by applyGains() so verifyConfig() re-applies tuned values
    private double kP, kI, kD, kV;

    // Hardware-follow state — kept so verifyConfig() re-applies it
    private SparkBase followLeader;
    private boolean   followOpposed;

    public REVMotor(MotorIOConfig config) {
        if (config.vendor != MotorVendor.REV_SPARKMAX && config.vendor != MotorVendor.REV_SPARKFLEX)
            throw new IllegalArgumentException("REVMotor: vendor must be REV_SPARKMAX or REV_SPARKFLEX, got "
                                               + config.vendor);
        this.config     = config;
        this.isFlex     = config.vendor == MotorVendor.REV_SPARKFLEX;
        this.vendorName = config.vendor.displayName;
        kP = config.kP; kI = config.kI; kD = config.kD; kV = config.kV;

        spark = isFlex
            ? new SparkFlex(config.canId, MotorType.kBrushless)
            : new SparkMax(config.canId, MotorType.kBrushless);
        controller = spark.getClosedLoopController();

        spark.configure(buildConfiguration(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** The complete Spark configuration for this motor — used at construction and by verifyConfig(). */
    private SparkBaseConfig buildConfiguration() {
        SparkBaseConfig cfg = newConfig();

        cfg.inverted(config.inverted);
        cfg.idleMode(config.brakeOnNeutral ? IdleMode.kBrake : IdleMode.kCoast);

        cfg.encoder
            .positionConversionFactor(1.0 / config.gearRatio)
            .velocityConversionFactor(1.0 / (config.gearRatio * 60.0));

        cfg.smartCurrentLimit((int) config.supplyCurrentLimitAmps);

        if (config.openLoopRampSecs > 0.0)   cfg.openLoopRampRate(config.openLoopRampSecs);
        if (config.closedLoopRampSecs > 0.0) cfg.closedLoopRampRate(config.closedLoopRampSecs);

        if (!Double.isNaN(config.softLimitForwardRot)) {
            cfg.softLimit
                .forwardSoftLimit((float) config.softLimitForwardRot)
                .forwardSoftLimitEnabled(true);
        }
        if (!Double.isNaN(config.softLimitReverseRot)) {
            cfg.softLimit
                .reverseSoftLimit((float) config.softLimitReverseRot)
                .reverseSoftLimitEnabled(true);
        }

        cfg.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kP, kI, kD)
            .apply(new FeedForwardConfig().kV(kV))
            .iZone(0);

        if (config.continuousWrap) {
            cfg.closedLoop
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(-0.5, 0.5);
        }

        if (config.hasMotionProfile()) {
            cfg.closedLoop.maxMotion
                .cruiseVelocity(config.motionCruiseVelocityRps)
                .maxAcceleration(config.motionAccelerationRpss)
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        }

        if (followLeader != null) cfg.follow(followLeader, followOpposed);
        return cfg;
    }

    private SparkBaseConfig newConfig() {
        return isFlex ? new SparkFlexConfig() : new SparkMaxConfig();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // MotorIO — commands
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public MotorIOConfig getConfig() { return config; }

    @Override
    public void setDutyCycle(double duty) {
        spark.set(duty);
    }

    @Override
    public void setVelocity(double velocityRps, double ffVolts) {
        controller.setSetpoint(velocityRps, ControlType.kVelocity, ClosedLoopSlot.kSlot0,
            ffVolts, ArbFFUnits.kVoltage);
    }

    @Override
    public void setPosition(double positionRot, double ffVolts) {
        ControlType ct = config.hasMotionProfile()
            ? ControlType.kMAXMotionPositionControl
            : ControlType.kPosition;
        controller.setSetpoint(positionRot, ct, ClosedLoopSlot.kSlot0, ffVolts, ArbFFUnits.kVoltage);
    }

    @Override
    public void stop() {
        spark.stopMotor();
    }

    @Override
    public void follow(MotorIO leader, boolean opposeLeader) {
        if (!(leader instanceof REVMotor revLeader))
            throw new IllegalArgumentException("REVMotor CAN " + config.canId
                + ": can only hardware-follow another REV controller, not " + leader.getConfig());
        followLeader  = revLeader.spark;
        followOpposed = opposeLeader;
        SparkBaseConfig cfg = newConfig();
        cfg.follow(followLeader, followOpposed);
        spark.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setEncoderPosition(double positionRot) {
        spark.getEncoder().setPosition(positionRot);
    }

    @Override
    public void applyGains(double kP, double kI, double kD, double kS, double kV, double kA) {
        this.kP = kP; this.kI = kI; this.kD = kD; this.kV = kV;
        SparkBaseConfig cfg = newConfig();
        cfg.closedLoop
            .pid(kP, kI, kD)
            .apply(new FeedForwardConfig().kV(kV));
        spark.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // MotorIO — feedback
    // ─────────────────────────────────────────────────────────────────────────

    @Override public double getPositionRot()        { return spark.getEncoder().getPosition(); }
    @Override public double getVelocityRps()        { return spark.getEncoder().getVelocity(); }
    @Override public double getSupplyCurrentAmps()  { return spark.getOutputCurrent(); }
    /** REV does not expose a separate stator current signal. */
    @Override public double getStatorCurrentAmps()  { return 0.0; }
    @Override public double getMotorVoltage()       { return spark.getBusVoltage() * spark.getAppliedOutput(); }
    @Override public double getTemperatureC()       { return spark.getMotorTemperature(); }
    @Override public boolean isForwardSoftLimitHit() { return spark.getForwardSoftLimit().isReached(); }
    @Override public boolean isReverseSoftLimitHit() { return spark.getReverseSoftLimit().isReached(); }

    /** Output current is the closest stall-detection signal available on SparkMax/Flex. */
    @Override
    public double getStallDetectionCurrentAmps() { return spark.getOutputCurrent(); }

    @Override public boolean hasHardwareMotionProfile() { return true; }
    @Override public boolean supportsEncoderSync()      { return true; }

    /** Direct access to the controller — used by REVThroughBoreEncoder. Advanced use only. */
    public SparkBase getSpark() {
        return spark;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Startup verification
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public ConfigVerifyResult verifyConfig(String label) {
        SparkBaseConfig cfg = buildConfiguration();

        boolean applyOk = false;
        for (int attempt = 0; attempt < VERIFY_RETRIES; attempt++) {
            REVLibError err = spark.configure(cfg,
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            if (err == REVLibError.kOk) { applyOk = true; break; }
            try { Thread.sleep(VERIFY_DELAY_MS); } catch (InterruptedException ignored) {}
        }
        if (!applyOk) return new ConfigVerifyResult(label, config.canId, vendorName, false, List.of());

        // configAccessor lives on SparkMax/SparkFlex, not SparkBase
        SparkBaseConfigAccessor accessor = isFlex
            ? ((SparkFlex) spark).configAccessor
            : ((SparkMax)  spark).configAccessor;
        List<String> mismatches = new ArrayList<>();

        if (accessor.getInverted() != config.inverted)
            mismatches.add("inversion: expected " + config.inverted + " got " + accessor.getInverted());
        IdleMode wantIdle = config.brakeOnNeutral ? IdleMode.kBrake : IdleMode.kCoast;
        if (accessor.getIdleMode() != wantIdle)
            mismatches.add("idleMode: expected " + wantIdle + " got " + accessor.getIdleMode());
        int wantLimit = (int) config.supplyCurrentLimitAmps;
        if (Math.abs(accessor.getSmartCurrentLimit() - wantLimit) > 1)
            mismatches.add("smartCurrentLimit: expected " + wantLimit
                           + "A got " + accessor.getSmartCurrentLimit() + "A");

        return new ConfigVerifyResult(label, config.canId, vendorName, true, mismatches);
    }
}
