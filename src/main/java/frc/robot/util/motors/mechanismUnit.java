package frc.robot.util.motors;

import edu.wpi.first.util.datalog.*;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * mechanismUnit.java
 * PATH: src/main/java/frc/robot/util/motors/mechanismUnit.java
 *
 * Abstract base class for all motor controller abstractions.
 *
 * Instantiate via factory:
 *   mechanismUnit arm = mechanismUnit.create(config);
 *
 * Or directly for CTRE-specific features:
 *   CTREMechanismUnit arm = new CTREMechanismUnit(config);
 *
 * ─── SHARED LOGIC IN THIS CLASS ──────────────────────────────────────────────
 *   • Static factory with vendor dispatch and mixed-vendor validation
 *   • Tier-2 + Tier-3 FF computation (lambda calls + sum)
 *   • Finite-difference acceleration estimation (dv/dt)
 *   • Software duty cycle ramp (fallback when hardware ramp unavailable)
 *   • PID live-tuning loop (SmartDashboard read-back → applyPIDToController)
 *   • ENCODER_SYNC follower correction (position error → duty cycle output)
 *   • DataLog entry lifecycle (per-instance, not static)
 *   • SmartDashboard telemetry publishing
 *
 * ─── CALL PATTERN IN SUBSYSTEM PERIODIC ──────────────────────────────────────
 *   // Once per loop — in the subsystem's periodic():
 *   arm.updateEncoderSync();   // only if FollowMode.ENCODER_SYNC
 *   arm.setPosition(targetDeg);
 *   arm.logTelemetry();
 *
 * ─── THREAD SAFETY ───────────────────────────────────────────────────────────
 *   All methods are intended to be called from the WPILib robot periodic thread
 *   (CommandScheduler.run() context). No synchronization is performed.
 *   The tier-2/3 FF lambdas and all state fields assume single-threaded access.
 */
public abstract class mechanismUnit {

    // ── Config ────────────────────────────────────────────────────────────────
    protected final mechanismConfig config;

    // ── Live PID gains (read back from SmartDashboard each loop) ─────────────
    protected double kP, kI, kD, kS, kV, kA;

    // ── Acceleration estimation (finite difference) ───────────────────────────
    private double prevVelocityRps = 0.0;
    private double accelRpss       = 0.0;

    // ── Software duty cycle ramp state ────────────────────────────────────────
    private double rampedDutyCycle = 0.0;

    // ── Nova software setpoint ramp state ─────────────────────────────────────
    private double rampedVelocitySetpoint  = 0.0;
    private double rampedPositionSetpoint  = 0.0;
    private boolean positionInitialized    = false;

    // ── SmartDashboard key prefixes ───────────────────────────────────────────
    protected final String dashPrefix;   // "[Name]/"
    private   final String pidPrefix;    // "[Name]/PID/"

    // ── DataLog entries (per-instance) ────────────────────────────────────────
    private DoubleLogEntry  logCommandedVelocityRps;
    private DoubleLogEntry  logActualVelocityRps;
    private DoubleLogEntry  logCommandedPositionDeg;
    private DoubleLogEntry  logActualPositionDeg;
    private DoubleLogEntry  logDutyCycle;
    private DoubleLogEntry  logAppliedFFVolts;
    private DoubleLogEntry  logSupplyCurrentA;
    private DoubleLogEntry  logStatorCurrentA;
    private DoubleLogEntry  logMotorVoltageV;
    private DoubleLogEntry  logTempC;
    private BooleanLogEntry logForwardLimit;
    private BooleanLogEntry logReverseLimit;
    // ENCODER_SYNC only (null otherwise):
    private DoubleLogEntry  logSyncErrorRot;
    private DoubleLogEntry  logSyncOutput;

    // ─────────────────────────────────────────────────────────────────────────
    // Static factory
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Create a mechanismUnit for the given config.
     * Validates that all motors in the config share the same vendor.
     * Returns the correct vendor implementation.
     *
     * @throws IllegalArgumentException if canIds and vendors arrays are
     *         mismatched, or if mixed vendors are specified.
     */
    public static mechanismUnit create(mechanismConfig config) {
        motorConstants.Vendor v = config.vendors[0];
        for (int i = 1; i < config.vendors.length; i++) {
            if (config.vendors[i] != v) {
                throw new IllegalArgumentException(
                    "mechanismUnit '" + config.name + "': mixed vendors are not supported. "
                    + "Motor 0 is " + v + " but motor " + i + " is " + config.vendors[i]
                    + ". All motors in a mechanism must use the same vendor.");
            }
        }
        switch (v) {
            case CTRE_TALONFX:    return new CTREMechanismUnit(config);
            case REV_SPARKMAX:    return new REVMechanismUnit(config, false);
            case REV_SPARKFLEX:   return new REVMechanismUnit(config, true);
            case THRIFTYBOT_NOVA: return new NovaMechanismUnit(config);
            default:
                throw new IllegalArgumentException(
                    "mechanismUnit '" + config.name + "': unknown vendor " + v);
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Constructor
    // ─────────────────────────────────────────────────────────────────────────

    protected mechanismUnit(mechanismConfig config) {
        this.config     = config;
        this.dashPrefix = config.name + "/";
        this.pidPrefix  = config.name + "/PID/";

        kP = config.pid[0];
        kI = config.pid[1];
        kD = config.pid[2];
        kS = config.pid[3];
        kV = config.pid[4];
        kA = config.pid[5];

        initLogEntries();
        publishPIDToDashboard();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Public API — called by subsystem periodic()
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Command duty cycle output.
     * Applies hardware ramp (if configured via openLoopRampSecs) or falls back
     * to software ramp in this base class.
     * Reads back SmartDashboard PID values and applies any changes.
     *
     * @param duty -1.0 to 1.0
     */
    public final void setDutyCycle(double duty) {
        applyDashboardPIDUpdates();
        double ramped = usesHardwareDutyCycleRamp() ? duty : applyDutyCycleRamp(duty);
        applyDutyCycleImpl(ramped);
        logDutyCycle.append(ramped);
    }

    /**
     * Command velocity setpoint (mechanism shaft, rotations/second).
     * Computes and injects tier-2 + tier-3 feed-forward each cycle.
     * Reads back SmartDashboard PID values and applies any changes.
     *
     * @param velocityRps target velocity (rotations/second, mechanism shaft)
     */
    public final void setVelocity(double velocityRps) {
        applyDashboardPIDUpdates();
        updateAcceleration();
        double ffVolts = computeTotalFF();
        applyVelocityImpl(velocityRps, ffVolts);
        logCommandedVelocityRps.append(velocityRps);
        logAppliedFFVolts.append(ffVolts);
    }

    /**
     * Command position setpoint (mechanism shaft, degrees).
     * Converts to rotations internally before passing to vendor implementation.
     * Computes and injects tier-2 + tier-3 feed-forward each cycle.
     * Reads back SmartDashboard PID values and applies any changes.
     *
     * @param positionDeg target position (degrees, mechanism shaft)
     */
    public final void setPosition(double positionDeg) {
        applyDashboardPIDUpdates();
        updateAcceleration();
        double ffVolts      = computeTotalFF();
        double positionRot  = positionDeg / 360.0;
        applyPositionImpl(positionRot, ffVolts);
        logCommandedPositionDeg.append(positionDeg);
        logAppliedFFVolts.append(ffVolts);
    }

    /**
     * Stop the motor and reset ramp state.
     * After calling stop(), the next duty cycle ramp will start from zero.
     */
    public final void stop() {
        rampedDutyCycle          = 0.0;
        rampedVelocitySetpoint   = 0.0;
        positionInitialized      = false;
        stopImpl();
    }

    /**
     * Run encoder-sync correction for all ENCODER_SYNC followers.
     * Call once per periodic() before the control command.
     * No-op if followMode != ENCODER_SYNC.
     */
    public final void updateEncoderSync() {
        if (config.followMode != motorConstants.FollowMode.ENCODER_SYNC) return;
        double leaderPosRot = getPositionImpl();
        for (int i = 1; i < config.canIds.length; i++) {
            double followerPosRot = getFollowerPositionImpl(i);
            double errorRot       = leaderPosRot - followerPosRot;
            if (logSyncErrorRot != null) logSyncErrorRot.append(errorRot);
            if (Math.abs(errorRot) > config.encoderSyncDeadbandRot) {
                double correction = config.encoderSyncKp * errorRot;
                correction = Math.max(-1.0, Math.min(1.0, correction));
                applyFollowerCorrectionImpl(i, correction);
                if (logSyncOutput != null) logSyncOutput.append(correction);
            } else {
                if (logSyncOutput != null) logSyncOutput.append(0.0);
            }
        }
    }

    /**
     * Log all telemetry for this mechanism to DataLog and SmartDashboard.
     * Call once per periodic() — typically after the control command.
     * Not called automatically to avoid double-logging when a subsystem
     * issues multiple commands per loop.
     */
    public final void logTelemetry() {
        double vel = getVelocityImpl();
        double pos = getPositionImpl() * 360.0; // rotations → degrees

        logActualVelocityRps.append(vel);
        logActualPositionDeg.append(pos);
        logSupplyCurrentA.append(getSupplyCurrentImpl());
        logStatorCurrentA.append(getStatorCurrentImpl());
        logMotorVoltageV.append(getMotorVoltageImpl());
        logTempC.append(getTemperatureImpl());
        logForwardLimit.append(isForwardLimitHitImpl());
        logReverseLimit.append(isReverseLimitHitImpl());

        SmartDashboard.putNumber(dashPrefix + "Velocity_rps",    vel);
        SmartDashboard.putNumber(dashPrefix + "Position_deg",    pos);
        SmartDashboard.putNumber(dashPrefix + "SupplyCurrent_A", getSupplyCurrentImpl());
        SmartDashboard.putNumber(dashPrefix + "MotorVoltage_V",  getMotorVoltageImpl());
        SmartDashboard.putBoolean(dashPrefix + "FwdLimit",       isForwardLimitHitImpl());
        SmartDashboard.putBoolean(dashPrefix + "RevLimit",       isReverseLimitHitImpl());
    }

    // ── Getters ───────────────────────────────────────────────────────────────

    /** Current mechanism shaft velocity (rotations/second, signed). */
    public final double getVelocityRps()    { return getVelocityImpl(); }

    /** Current mechanism shaft position (degrees). */
    public final double getPositionDeg()    { return getPositionImpl() * 360.0; }

    /** Supply (input) current in amps. */
    public final double getSupplyCurrent()  { return getSupplyCurrentImpl(); }

    /** Stator (output) current in amps. CTRE only; returns 0 for REV/Nova. */
    public final double getStatorCurrent()  { return getStatorCurrentImpl(); }

    /** Motor terminal voltage in volts. */
    public final double getMotorVoltage()   { return getMotorVoltageImpl(); }

    /** True if the forward soft/hard limit is currently triggered. */
    public final boolean isForwardLimit()   { return isForwardLimitHitImpl(); }

    /** True if the reverse soft/hard limit is currently triggered. */
    public final boolean isReverseLimit()   { return isReverseLimitHitImpl(); }

    /** The immutable config this unit was built from. */
    public final mechanismConfig getConfig() { return config; }

    // ─────────────────────────────────────────────────────────────────────────
    // Shared internal logic — runs in this base, not duplicated in vendor classes
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Sum tier-2 + tier-3 FF for the current cycle.
     * Both lambdas receive the same (position, velocity, acceleration) snapshot.
     */
    protected final double computeTotalFF() {
        double positionDeg = getPositionImpl() * 360.0;
        double velocityRps = getVelocityImpl();
        double t2 = config.tier2FF.compute(positionDeg, velocityRps, accelRpss);
        double t3 = config.tier3FF.compute(positionDeg, velocityRps, accelRpss);
        return t2 + t3;
    }

    /**
     * Update the finite-difference acceleration estimate.
     * Must be called once per cycle, before computeTotalFF().
     * Uses the previous velocity sample stored in prevVelocityRps.
     */
    private void updateAcceleration() {
        double currentVel = getVelocityImpl();
        accelRpss     = (currentVel - prevVelocityRps) / motorConstants.LOOP_PERIOD_SECS;
        prevVelocityRps = currentVel;
    }

    /**
     * Software duty cycle ramp.
     * Steps rampedDutyCycle toward target by at most maxDelta per loop.
     * maxDelta = LOOP_PERIOD_SECS / openLoopRampSecs (full swing per second).
     * Returns target unchanged when openLoopRampSecs <= 0.
     */
    private double applyDutyCycleRamp(double target) {
        if (config.openLoopRampSecs <= 0.0) {
            rampedDutyCycle = target;
            return target;
        }
        double maxDelta = motorConstants.LOOP_PERIOD_SECS / config.openLoopRampSecs;
        double delta    = target - rampedDutyCycle;
        delta           = Math.max(-maxDelta, Math.min(maxDelta, delta));
        rampedDutyCycle += delta;
        return rampedDutyCycle;
    }

    /**
     * Software velocity setpoint step-limiter.
     * Used by NovaMechanismUnit for soft-start on closed-loop velocity.
     * Steps rampedVelocitySetpoint toward target by at most
     * motionAccelerationRpss * LOOP_PERIOD_SECS per cycle.
     * Returns target unchanged when motionAccelerationRpss <= 0.
     */
    protected final double applyVelocityRamp(double targetRps) {
        if (config.motionAccelerationRpss <= 0.0) return targetRps;
        double maxDelta = config.motionAccelerationRpss * motorConstants.LOOP_PERIOD_SECS;
        double delta    = targetRps - rampedVelocitySetpoint;
        delta           = Math.max(-maxDelta, Math.min(maxDelta, delta));
        rampedVelocitySetpoint += delta;
        return rampedVelocitySetpoint;
    }

    /**
     * Software position setpoint step-limiter.
     * Used by NovaMechanismUnit for soft-start on closed-loop position.
     * Seeds from current position on first call after construction or stop().
     */
    protected final double applyPositionRamp(double targetRot) {
        if (config.motionAccelerationRpss <= 0.0) return targetRot;
        if (!positionInitialized) {
            rampedPositionSetpoint = getPositionImpl();
            positionInitialized    = true;
        }
        double maxDelta = config.motionAccelerationRpss * motorConstants.LOOP_PERIOD_SECS;
        double delta    = targetRot - rampedPositionSetpoint;
        delta           = Math.max(-maxDelta, Math.min(maxDelta, delta));
        rampedPositionSetpoint += delta;
        return rampedPositionSetpoint;
    }

    /**
     * Read PID gains from SmartDashboard and apply to controller if any changed.
     * Called at the start of every setDutyCycle/setVelocity/setPosition call.
     * Changes are only written to the controller when a value actually changes,
     * avoiding unnecessary CAN/SPI traffic.
     *
     * NOTE: Live tuning via SmartDashboard is a development tool.
     * For REV controllers, applying new gains triggers a configure() call
     * which may briefly interrupt closed-loop control. Do not rely on this
     * at competition with time-critical mechanisms.
     */
    private void applyDashboardPIDUpdates() {
        double nkP = SmartDashboard.getNumber(pidPrefix + "kP", kP);
        double nkI = SmartDashboard.getNumber(pidPrefix + "kI", kI);
        double nkD = SmartDashboard.getNumber(pidPrefix + "kD", kD);
        double nkS = SmartDashboard.getNumber(pidPrefix + "kS", kS);
        double nkV = SmartDashboard.getNumber(pidPrefix + "kV", kV);
        double nkA = SmartDashboard.getNumber(pidPrefix + "kA", kA);
        if (nkP != kP || nkI != kI || nkD != kD || nkS != kS || nkV != kV || nkA != kA) {
            kP = nkP; kI = nkI; kD = nkD; kS = nkS; kV = nkV; kA = nkA;
            applyPIDToController();
        }
    }

    private void publishPIDToDashboard() {
        SmartDashboard.putNumber(pidPrefix + "kP", kP);
        SmartDashboard.putNumber(pidPrefix + "kI", kI);
        SmartDashboard.putNumber(pidPrefix + "kD", kD);
        SmartDashboard.putNumber(pidPrefix + "kS", kS);
        SmartDashboard.putNumber(pidPrefix + "kV", kV);
        SmartDashboard.putNumber(pidPrefix + "kA", kA);
    }

    private void initLogEntries() {
        DataLog log = DataLogManager.getLog();
        String p = "/" + config.name + "/";
        logCommandedVelocityRps = new DoubleLogEntry(log,  p + "CommandedVelocity_rps");
        logActualVelocityRps    = new DoubleLogEntry(log,  p + "ActualVelocity_rps");
        logCommandedPositionDeg = new DoubleLogEntry(log,  p + "CommandedPosition_deg");
        logActualPositionDeg    = new DoubleLogEntry(log,  p + "ActualPosition_deg");
        logDutyCycle            = new DoubleLogEntry(log,  p + "DutyCycle");
        logAppliedFFVolts       = new DoubleLogEntry(log,  p + "AppliedFF_volts");
        logSupplyCurrentA       = new DoubleLogEntry(log,  p + "SupplyCurrent_A");
        logStatorCurrentA       = new DoubleLogEntry(log,  p + "StatorCurrent_A");
        logMotorVoltageV        = new DoubleLogEntry(log,  p + "MotorVoltage_V");
        logTempC                = new DoubleLogEntry(log,  p + "Temp_C");
        logForwardLimit         = new BooleanLogEntry(log, p + "ForwardLimit");
        logReverseLimit         = new BooleanLogEntry(log, p + "ReverseLimit");
        if (config.followMode == motorConstants.FollowMode.ENCODER_SYNC) {
            logSyncErrorRot = new DoubleLogEntry(log, p + "EncoderSync/Error_rot");
            logSyncOutput   = new DoubleLogEntry(log, p + "EncoderSync/Output");
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Abstract methods — vendor implementations must provide these
    // ─────────────────────────────────────────────────────────────────────────

    /** Apply duty cycle to the leader motor. Value is already ramped. */
    protected abstract void applyDutyCycleImpl(double duty);

    /**
     * Apply velocity setpoint to the leader motor.
     * @param rps      mechanism shaft RPS target
     * @param ffVolts  tier-2 + tier-3 summed feed-forward (volts)
     */
    protected abstract void applyVelocityImpl(double rps, double ffVolts);

    /**
     * Apply position setpoint to the leader motor.
     * @param rotations mechanism shaft rotations target (already converted from degrees)
     * @param ffVolts   tier-2 + tier-3 summed feed-forward (volts)
     */
    protected abstract void applyPositionImpl(double rotations, double ffVolts);

    /** Neutral-stop the motor. */
    protected abstract void stopImpl();

    /**
     * Apply the current kP/kI/kD/kS/kV/kA values to the motor controller.
     * Called by applyDashboardPIDUpdates() when a change is detected.
     */
    protected abstract void applyPIDToController();

    /** @return current mechanism shaft velocity (rotations/second, signed) */
    protected abstract double getVelocityImpl();

    /** @return current mechanism shaft position (rotations) */
    protected abstract double getPositionImpl();

    /** @return supply (input) current in amps */
    protected abstract double getSupplyCurrentImpl();

    /** @return stator (output) current in amps. Return 0 if not available. */
    protected abstract double getStatorCurrentImpl();

    /** @return motor terminal voltage in volts */
    protected abstract double getMotorVoltageImpl();

    /** @return motor temperature in degrees Celsius */
    protected abstract double getTemperatureImpl();

    /** @return true if the forward limit is currently triggered */
    protected abstract boolean isForwardLimitHitImpl();

    /** @return true if the reverse limit is currently triggered */
    protected abstract boolean isReverseLimitHitImpl();

    /**
     * Get position of a follower motor (ENCODER_SYNC mode only).
     * @param followerIndex 1-based index into config.canIds[]
     * @return follower mechanism shaft position (rotations)
     */
    protected abstract double getFollowerPositionImpl(int followerIndex);

    /**
     * Apply a duty cycle correction to a follower motor (ENCODER_SYNC mode only).
     * @param followerIndex 1-based index into config.canIds[]
     * @param duty          correction duty cycle [-1.0, 1.0]
     */
    protected abstract void applyFollowerCorrectionImpl(int followerIndex, double duty);

    /**
     * Returns true when the vendor implementation has configured a hardware duty
     * cycle ramp, making the software ramp in this base class redundant.
     * CTRE and REV override this to return true when openLoopRampSecs > 0.
     * Nova returns false — its ramp support is less reliable and the software
     * fallback is preferred.
     */
    protected boolean usesHardwareDutyCycleRamp() { return false; }
}
