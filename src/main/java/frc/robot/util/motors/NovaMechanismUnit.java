package frc.robot.util.motors;

import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.EncoderType;
import com.thethriftybot.devices.ThriftyNova.PIDSlot;

/**
 * NovaMechanismUnit.java
 * PATH: src/main/java/frc/robot/util/motors/NovaMechanismUnit.java
 *
 * mechanismUnit implementation for ThriftyBot Nova motor controller.
 *
 * ─── FEED-FORWARD INJECTION ──────────────────────────────────────────────────
 * Nova supports native per-cycle FF injection via the second parameter of
 * setVelocity(rps, ffVolts) and setPosition(rotations, ffVolts).
 * Tier-2 + Tier-3 FF is passed directly as volts — no battery-voltage
 * approximation or duty-cycle conversion required.
 *
 * ─── GEAR RATIO — MANUAL SCALING ─────────────────────────────────────────────
 * Nova has no native conversion factor API. All setpoints are multiplied by
 * gearRatio before sending to Nova; feedback is divided by gearRatio on
 * readback.
 *
 * ─── MOTION PROFILE ──────────────────────────────────────────────────────────
 * Nova has no MotionMagic or MAXMotion equivalent. When motionCruiseVelocityRps
 * > 0, the abstract base class software setpoint step-limiter is used instead:
 *   - Velocity: setpoint steps at max motionAccelerationRpss * LOOP_PERIOD_SECS
 *   - Position: setpoint steps toward target at the same rate
 * This runs on the Rio and is less precise than hardware profiling, but provides
 * adequate soft-start protection for most mechanism use cases.
 *
 * ─── ENCODER_SYNC FOLLOW MODE NOT SUPPORTED ──────────────────────────────────
 * Nova's follow() API causes the follower to mirror the leader's CAN output;
 * independent encoder readback on a following device is not reliable in this
 * mode. Attempting to use ENCODER_SYNC will throw at construction time.
 *
 * ─── FOLLOWERS ───────────────────────────────────────────────────────────────
 * MECHANICAL: native Nova follow(leaderCanId) — mirrors leader output over CAN.
 * Inversion via software is not supported by Nova's follow() API; use physical
 * motor mounting for opposing-direction followers.
 *
 * ─── PID LIVE TUNING ─────────────────────────────────────────────────────────
 * Nova PID gains are updated live via the public pid0/pid1 PIDConfig fields.
 * No configure() call required — changes apply on the next control cycle.
 */
public class NovaMechanismUnit extends mechanismUnit {

    // ── Leader and followers ──────────────────────────────────────────────────
    private final ThriftyNova leader;
    private final ThriftyNova[] followers;

    // ── Gear ratio cached for setpoint/readback scaling ───────────────────────
    private final double gearRatio;

    // ── Flags ─────────────────────────────────────────────────────────────────
    private final boolean motionProfileEnabled;

    // ─────────────────────────────────────────────────────────────────────────
    // Constructor
    // ─────────────────────────────────────────────────────────────────────────

    public NovaMechanismUnit(mechanismConfig config) {
        super(config);

        if (config.followMode == motorConstants.FollowMode.ENCODER_SYNC) {
            throw new IllegalArgumentException(
                "NovaMechanismUnit '" + config.name + "': ENCODER_SYNC follow mode is not "
                + "supported for ThriftyBot Nova. Nova's follow() API mirrors the leader "
                + "output over CAN; independent encoder readback on following devices is "
                + "not reliable in this mode. Use MECHANICAL follow mode instead.");
        }

        this.gearRatio            = config.gearRatio;
        this.motionProfileEnabled = config.motionCruiseVelocityRps > 0.0;

        leader = new ThriftyNova(config.canIds[0]);
        followers = new ThriftyNova[config.canIds.length - 1];
        for (int i = 0; i < followers.length; i++) {
            followers[i] = new ThriftyNova(config.canIds[i + 1]);
        }

        configureLeader();
        configureFollowers();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Motor configuration
    // ─────────────────────────────────────────────────────────────────────────

    private void configureLeader() {
        // Inversion and neutral mode
        leader.setInverted(config.inverted);
        leader.setBrakeMode(config.brakeOnNeutral);

        // Encoder type — internal encoder (Hall-effect, built into Nova)
        leader.useEncoderType(EncoderType.INTERNAL);

        // PID slot 0 — kS used as combined static FF term (closest Nova equivalent)
        // kV and kA are not separately configurable; tier-2/tier-3 FF lambdas
        // handle velocity and acceleration feed-forward via native per-cycle injection.
        leader.pid0.setP(kP).setI(kI).setD(kD).setFF(kS);

        // Soft limits (degrees → motor rotations via gear ratio)
        // setSoftLimits(reverseLimit, forwardLimit) — note: reverse is first arg
        if (!Double.isNaN(config.softLimitReverseDeg) || !Double.isNaN(config.softLimitForwardDeg)) {
            double revRot = Double.isNaN(config.softLimitReverseDeg)
                ? -Float.MAX_VALUE
                : (config.softLimitReverseDeg / 360.0) * gearRatio;
            double fwdRot = Double.isNaN(config.softLimitForwardDeg)
                ? Float.MAX_VALUE
                : (config.softLimitForwardDeg / 360.0) * gearRatio;
            leader.setSoftLimits(revRot, fwdRot).enableSoftLimits(true);
        }

        // Current limit
        leader.setMaxCurrent(ThriftyNova.CurrentType.SUPPLY, config.supplyCurrentLimitAmps);

        // Ramp rate
        if (config.openLoopRampSecs > 0.0) {
            leader.setRampUp(config.openLoopRampSecs);
            leader.setRampDown(config.openLoopRampSecs);
        }
    }

    private void configureFollowers() {
        for (int i = 0; i < followers.length; i++) {
            if (config.followMode == motorConstants.FollowMode.MECHANICAL) {
                boolean invert = i < config.followerInverted.length && config.followerInverted[i];
                if (invert) {
                    System.out.println("[WARNING] NovaMechanismUnit '" + config.name
                        + "': follower " + i + " requested inverted follow, but Nova's "
                        + "follow() API does not support inversion in software. "
                        + "Use physical motor mounting (opposite orientation) instead.");
                }
                // Nova follow() mirrors the leader's output over CAN
                followers[i].follow(config.canIds[0]);
            }
            // NONE: followers are unused — no configuration needed
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // mechanismUnit abstract method implementations
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    protected void applyDutyCycleImpl(double duty) {
        leader.setPercent(duty);
    }

    @Override
    protected void applyVelocityImpl(double rps, double ffVolts) {
        // Apply software setpoint step-limiter if motion profile is configured
        double targetRps = motionProfileEnabled ? applyVelocityRamp(rps) : rps;

        // Scale to motor-native RPS; inject FF as voltage natively
        leader.setVelocity(targetRps * gearRatio, ffVolts);
    }

    @Override
    protected void applyPositionImpl(double rotations, double ffVolts) {
        // Apply software setpoint step-limiter if motion profile is configured
        double targetRot = motionProfileEnabled ? applyPositionRamp(rotations) : rotations;

        // Scale to motor-native rotations; inject FF as voltage natively
        leader.setPosition(targetRot * gearRatio, ffVolts);
    }

    @Override
    protected void stopImpl() {
        leader.setPercent(0.0);
    }

    @Override
    protected void applyPIDToController() {
        // Nova PID gains update live — no configure() call required
        leader.pid0.setP(kP).setI(kI).setD(kD).setFF(kS);
    }

    @Override
    protected double getVelocityImpl() {
        // Nova returns motor-native RPS; divide by gearRatio for mechanism shaft RPS
        return leader.getVelocity() / gearRatio;
    }

    @Override
    protected double getPositionImpl() {
        // Nova returns motor-native rotations; divide by gearRatio for mechanism shaft
        return leader.getPosition() / gearRatio;
    }

    @Override
    protected double getSupplyCurrentImpl() {
        return leader.getSupplyCurrent();
    }

    @Override
    protected double getStatorCurrentImpl() {
        return leader.getStatorCurrent();
    }

    @Override
    protected double getMotorVoltageImpl() {
        return leader.getAppliedVoltage();
    }

    @Override
    protected double getTemperatureImpl() {
        return leader.getTemperature();
    }

    @Override
    protected boolean isForwardLimitHitImpl() {
        return leader.getForwardLimit().getAsBoolean();
    }

    @Override
    protected boolean isReverseLimitHitImpl() {
        return leader.getReverseLimit().getAsBoolean();
    }

    @Override
    protected double getFollowerPositionImpl(int followerIndex) {
        // ENCODER_SYNC is rejected at construction — this should never be called
        throw new UnsupportedOperationException(
            "NovaMechanismUnit: ENCODER_SYNC follow mode is not supported. "
            + "getFollowerPositionImpl() should never be called on a Nova mechanism.");
    }

    @Override
    protected void applyFollowerCorrectionImpl(int followerIndex, double duty) {
        // ENCODER_SYNC is rejected at construction — this should never be called
        throw new UnsupportedOperationException(
            "NovaMechanismUnit: ENCODER_SYNC follow mode is not supported. "
            + "applyFollowerCorrectionImpl() should never be called on a Nova mechanism.");
    }

    @Override
    protected boolean usesHardwareDutyCycleRamp() {
        // Nova has ramp rate configuration; use hardware ramp when configured.
        return config.openLoopRampSecs > 0.0;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Nova-specific extras
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Switch the active PID slot at runtime.
     * Nova supports two PID slots (SLOT0, SLOT1). Slot 0 is configured at
     * construction from config.pid[]. Use this to switch to a pre-configured
     * alternate slot (e.g. softer gains for a manual control mode).
     *
     * @param slot ThriftyNova.PIDSlot.SLOT0 or SLOT1
     */
    public void usePIDSlot(PIDSlot slot) {
        leader.usePIDSlot(slot);
    }

    /**
     * Configure the alternate PID slot (slot 1).
     * Slot 0 is always configured from config.pid[] at construction.
     * Call this to set up slot 1 with different gains (e.g. for a softer
     * control mode). Switch between slots with usePIDSlot().
     *
     * @param p  proportional gain
     * @param i  integral gain
     * @param d  derivative gain
     * @param ff combined static feed-forward term (kS, in volts)
     */
    public void configureAlternatePIDSlot(double p, double i, double d, double ff) {
        leader.pid1.setP(p).setI(i).setD(d).setFF(ff);
    }

    /**
     * Direct access to the underlying leader ThriftyNova.
     * For advanced use only — prefer the mechanismUnit public API where possible.
     */
    public ThriftyNova getLeaderMotor() {
        return leader;
    }
}
