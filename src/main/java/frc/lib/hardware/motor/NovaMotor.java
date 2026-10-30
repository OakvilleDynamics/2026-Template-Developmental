package frc.lib.hardware.motor;

import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.EncoderType;
import com.thethriftybot.devices.ThriftyNova.MotorType;
import com.thethriftybot.devices.ThriftyNova.PIDSlot;

import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.List;

import frc.lib.util.ConfigVerifyResult;

/**
 * NovaMotor.java
 * PATH: src/main/java/frc/lib/hardware/motor/NovaMotor.java
 *
 * MotorIO for the ThriftyBot Nova controller (NEO, Minion, Pulsar, brushed).
 *
 * ─── GEAR RATIO — MANUAL SCALING ─────────────────────────────────────────────
 * Nova has no conversion-factor API. Setpoints are multiplied by gearRatio
 * before being sent; feedback is divided by gearRatio on readback.
 *
 * ─── FEED-FORWARD ────────────────────────────────────────────────────────────
 * kS maps to Nova's pid0 FF term (closest static-FF equivalent). kV/kA are not
 * configurable on-controller. Rio-computed FF is injected natively as volts via
 * setVelocity(rps, ffVolts) / setPosition(rot, ffVolts).
 *
 * ─── MOTION PROFILE ──────────────────────────────────────────────────────────
 * No MotionMagic/MAXMotion equivalent — hasHardwareMotionProfile() is false,
 * so mechanismUnit applies its software setpoint step-limiter instead.
 *
 * ─── CONTINUOUS WRAP ─────────────────────────────────────────────────────────
 * No hardware wrap — setPosition() computes the shortest path in software.
 *
 * ─── FOLLOWERS ───────────────────────────────────────────────────────────────
 * follow() mirrors the leader's CAN output. Inverted follow is not supported in
 * software (use physical mounting). ENCODER_SYNC is not supported — independent
 * encoder readback on a following Nova is not reliable.
 */
public class NovaMotor implements MotorIO {

    private static final String VENDOR = MotorVendor.THRIFTYBOT_NOVA.displayName;

    private final MotorIOConfig config;
    private final ThriftyNova   motor;
    private final double        gearRatio;

    // Live gains — updated by applyGains() so verifyConfig() re-applies tuned values
    private double kP, kI, kD, kS;

    public NovaMotor(MotorIOConfig config) {
        if (config.vendor != MotorVendor.THRIFTYBOT_NOVA)
            throw new IllegalArgumentException("NovaMotor: vendor must be THRIFTYBOT_NOVA, got " + config.vendor);
        this.config    = config;
        this.gearRatio = config.gearRatio;
        kP = config.kP; kI = config.kI; kD = config.kD; kS = config.kS;

        motor = new ThriftyNova(config.canId);
        motor.useEncoderType(EncoderType.INTERNAL);
        applyConfiguration();

        if (!Double.isNaN(config.softLimitReverseRot) || !Double.isNaN(config.softLimitForwardRot)) {
            // setSoftLimits(reverse, forward) — reverse is the first argument
            double revRot = Double.isNaN(config.softLimitReverseRot)
                ? -Float.MAX_VALUE : config.softLimitReverseRot * gearRatio;
            double fwdRot = Double.isNaN(config.softLimitForwardRot)
                ? Float.MAX_VALUE  : config.softLimitForwardRot * gearRatio;
            motor.setSoftLimits(revRot, fwdRot).enableSoftLimits(true);
        }

        if (config.openLoopRampSecs > 0.0) {
            motor.setRampUp(config.openLoopRampSecs);
            motor.setRampDown(config.openLoopRampSecs);
        }
    }

    /** The settings re-applied by verifyConfig(). Nova has no single apply() call. */
    private void applyConfiguration() {
        motor.setInverted(config.inverted);
        motor.setBrakeMode(config.brakeOnNeutral);
        motor.setMaxCurrent(ThriftyNova.CurrentType.SUPPLY, config.supplyCurrentLimitAmps);
        motor.pid0.setP(kP).setI(kI).setD(kD).setFF(kS);
        if (config.novaMotorType != null) motor.setMotorType(toNovaType(config.novaMotorType));
    }

    // ─────────────────────────────────────────────────────────────────────────
    // MotorIO — commands
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public MotorIOConfig getConfig() { return config; }

    @Override
    public void setDutyCycle(double duty) {
        motor.setPercent(duty);
    }

    @Override
    public void setVelocity(double velocityRps, double ffVolts) {
        motor.setVelocity(velocityRps * gearRatio, ffVolts);
    }

    @Override
    public void setPosition(double positionRot, double ffVolts) {
        double targetRot = config.continuousWrap ? shortestPathTarget(positionRot) : positionRot;
        motor.setPosition(targetRot * gearRatio, ffVolts);
    }

    /**
     * Software continuous wrap: returns the continuous mechanism position that
     * reaches {@code wrappedTargetRot} (in [-0.5, 0.5)) by the shortest path.
     */
    private double shortestPathTarget(double wrappedTargetRot) {
        double mechPos = getPositionRot();
        // Where is the current position within one full rotation? → [-0.5, 0.5)
        double mechMod = mechPos - Math.floor(mechPos);
        if (mechMod >= 0.5) mechMod -= 1.0;
        double delta = wrappedTargetRot - mechMod;
        if (delta >  0.5) delta -= 1.0;
        if (delta < -0.5) delta += 1.0;
        return mechPos + delta;
    }

    @Override
    public void stop() {
        motor.setPercent(0.0);
    }

    @Override
    public void follow(MotorIO leader, boolean opposeLeader) {
        if (opposeLeader) {
            DriverStation.reportWarning("[NovaMotor] CAN " + config.canId
                + ": inverted follow requested, but Nova's follow() API does not support "
                + "inversion in software. Use physical motor mounting instead.", false);
        }
        motor.follow(leader.getConfig().canId);
    }

    @Override
    public void setEncoderPosition(double positionRot) {
        motor.setEncoderPosition(positionRot * gearRatio);
    }

    @Override
    public void applyGains(double kP, double kI, double kD, double kS, double kV, double kA) {
        // Nova gains update live — no configure() call. kV/kA not configurable.
        this.kP = kP; this.kI = kI; this.kD = kD; this.kS = kS;
        motor.pid0.setP(kP).setI(kI).setD(kD).setFF(kS);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // MotorIO — feedback
    // ─────────────────────────────────────────────────────────────────────────

    @Override public double getPositionRot()        { return motor.getPosition() / gearRatio; }
    @Override public double getVelocityRps()        { return motor.getVelocity() / gearRatio; }
    @Override public double getSupplyCurrentAmps()  { return motor.getSupplyCurrent(); }
    @Override public double getStatorCurrentAmps()  { return motor.getStatorCurrent(); }
    @Override public double getMotorVoltage()       { return motor.getAppliedVoltage(); }
    @Override public double getTemperatureC()       { return motor.getTemperature(); }
    @Override public boolean isForwardSoftLimitHit() { return motor.getForwardLimit().getAsBoolean(); }
    @Override public boolean isReverseSoftLimitHit() { return motor.getReverseLimit().getAsBoolean(); }

    @Override
    public double getStallDetectionCurrentAmps() { return motor.getStatorCurrent(); }

    @Override public boolean hasHardwareMotionProfile() { return false; }
    @Override public boolean supportsEncoderSync()      { return false; }

    // ─────────────────────────────────────────────────────────────────────────
    // Nova-only extras — reach by casting MotorIO to NovaMotor
    // ─────────────────────────────────────────────────────────────────────────

    /** Switch the active PID slot (SLOT0 is configured from MotorIOConfig). */
    public void usePIDSlot(PIDSlot slot) {
        motor.usePIDSlot(slot);
    }

    /** Configure the alternate PID slot (slot 1). ff = static FF term (kS, volts). */
    public void configureAlternatePIDSlot(double p, double i, double d, double ff) {
        motor.pid1.setP(p).setI(i).setD(d).setFF(ff);
    }

    /** Direct access to the ThriftyNova. Advanced use only — prefer the MotorIO API. */
    public ThriftyNova getNova() {
        return motor;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Startup verification
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public ConfigVerifyResult verifyConfig(String label) {
        boolean applyOk = false;
        for (int attempt = 0; attempt < VERIFY_RETRIES; attempt++) {
            motor.clearErrors();
            applyConfiguration();
            try { Thread.sleep(VERIFY_DELAY_MS); } catch (InterruptedException ignored) {}
            if (motor.getErrors().isEmpty()) { applyOk = true; break; }
        }
        if (!applyOk) return new ConfigVerifyResult(label, config.canId, VENDOR, false, List.of());

        List<String> mismatches = new ArrayList<>();
        if (motor.getInverted() != config.inverted)
            mismatches.add("inversion: expected " + config.inverted + " got " + motor.getInverted());
        if (motor.getBrakeMode() != config.brakeOnNeutral)
            mismatches.add("brakeMode: expected " + config.brakeOnNeutral + " got " + motor.getBrakeMode());
        if (config.novaMotorType != null) {
            MotorType want = toNovaType(config.novaMotorType);
            MotorType read = motor.getMotorType();
            if (read != want)
                mismatches.add("motorType: expected " + want + " got " + read
                               + " — wrong motor type is a known silent-failure mode");
        }

        return new ConfigVerifyResult(label, config.canId, VENDOR, true, mismatches);
    }

    private static MotorType toNovaType(MotorIOConfig.NovaMotorType type) {
        return MotorType.valueOf(type.name());
    }
}
