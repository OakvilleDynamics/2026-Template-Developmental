package frc.lib.control;

import java.util.function.Supplier;

import edu.wpi.first.util.datalog.*;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.lib.swerve.driveOdometryState;
import frc.lib.util.units;

import java.util.List;
import frc.lib.util.ConfigVerifiable;
import frc.lib.util.ConfigVerifyResult;
import frc.lib.hardware.motor.motorModels;
import frc.lib.control.ff.ffProvider;
import frc.robot.util.motors.CTREMechanismUnit;
import frc.robot.util.motors.REVMechanismUnit;
import frc.robot.util.motors.NovaMechanismUnit;

/**
 * mechanismUnit.java
 * PATH: src/main/java/frc/lib/control/mechanismUnit.java
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
 *   // One call per loop — encoder sync, FF, PID tuning, and telemetry are automatic:
 *   arm.setPosition(targetDeg);
 *
 * ─── THREAD SAFETY ───────────────────────────────────────────────────────────
 *   All methods are intended to be called from the WPILib robot periodic thread
 *   (CommandScheduler.run() context). No synchronization is performed.
 *   The tier-2/3 FF lambdas and all state fields assume single-threaded access.
 */
public abstract class mechanismUnit implements ConfigVerifiable {

    // ── Homing direction (public — used in startHoming() calls) ───────────────

    /**
     * Direction to drive during a homing operation.
     * POSITIVE = positive duty cycle (increasing position reading).
     * NEGATIVE = negative duty cycle (decreasing position reading).
     * Physical direction depends on the leader motor's inversion config.
     */
    public enum HomingDirection { POSITIVE, NEGATIVE }

    /** Homing state machine states (private — implementation detail). */
    private enum HomingState { IDLE, IN_PROGRESS, COMPLETE }

    // ── Config ────────────────────────────────────────────────────────────────
    protected final mechanismConfig config;

    // ── Live PID gains (read back from SmartDashboard each loop) ─────────────
    protected double kP, kI, kD, kS, kV, kA;

    // ── Setpoint tracking (for isAtSetpoint) ─────────────────────────────────
    private double  lastSetpoint              = 0.0;
    private boolean lastSetpointIsVelocity    = false;
    private boolean atSetpointWarningLogged   = false;

    // ── Acceleration estimation (finite difference) ───────────────────────────
    private double prevVelocityRps = 0.0;
    private double accelRpss       = 0.0;

    // ── Software duty cycle ramp state ────────────────────────────────────────
    private double rampedDutyCycle = 0.0;

    // ── Nova software setpoint ramp state ─────────────────────────────────────
    private double rampedVelocitySetpoint  = 0.0;
    private double rampedPositionSetpoint  = 0.0;
    private boolean positionInitialized    = false;

    // ── Cached ENCODER_SYNC follower index ────────────────────────────────────
    // 1-based index into config.canIds[]. -1 when no ENCODER_SYNC follower exists.
    // Computed once at construction; avoids scanning followerModes[] every cycle.
    private final int encoderSyncFollowerIndex;

    // ── Travel limit latch state ──────────────────────────────────────────────
    // Each stall counter tracks consecutive loops where both current and speed
    // thresholds are met. When counter >= stallDetectionCycles, the latch sets.
    private int     leadLowerStallCount      = 0;
    private int     leadUpperStallCount      = 0;
    private int     followerLowerStallCount  = 0;
    private int     followerUpperStallCount  = 0;

    private boolean leadLowerLimitLatched      = false;
    private boolean leadUpperLimitLatched      = false;
    private boolean followerLowerLimitLatched  = false;
    private boolean followerUpperLimitLatched  = false;

    // ── Homing state machine ──────────────────────────────────────────────────
    private HomingState     homingState               = HomingState.IDLE;
    private HomingDirection homingDirection           = HomingDirection.NEGATIVE;
    private double          homingDutyCycle           = 0.0;
    private double          homingLeadCurrentAmps     = 0.0;
    private double          homingLeadSpeedRps        = 0.0;
    private double          homingFollowerCurrentAmps = 0.0;
    private double          homingFollowerSpeedRps    = 0.0;
    private int             homingLeadStallCount      = 0;
    private int             homingFollowerStallCount  = 0;
    private boolean         leadHomed                 = false;
    private boolean         followerHomed             = false;

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
    // Travel limits (null when limits not configured):
    private BooleanLogEntry logLeadLowerLimit;
    private BooleanLogEntry logLeadUpperLimit;
    private BooleanLogEntry logFollowerLowerLimit;
    private BooleanLogEntry logFollowerUpperLimit;
    // Homing (always allocated):
    private BooleanLogEntry logIsHoming;

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

        // Cache the ENCODER_SYNC follower index once — avoids a per-cycle array scan.
        // Must be set before initLogEntries(), which uses it to allocate log entries.
        int syncIdx = -1;
        for (int i = 0; i < config.followerModes.length; i++) {
            if (config.followerModes[i] == motorConstants.FollowMode.ENCODER_SYNC) {
                syncIdx = i + 1;  // 1-based to match getFollowerPositionImpl convention
                break;
            }
        }
        this.encoderSyncFollowerIndex = syncIdx;

        initLogEntries();
        publishPIDToDashboard();

        if (config.setpointDeadband <= 0.0) {
            DriverStation.reportWarning(
                "[mechanismUnit] '" + config.name + "': setpointDeadband is not configured. "
                + "isAtSetpoint() will always return false. "
                + "Call withSetpointDeadband() in the mechanismConfig.Builder to enable it.", false);
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Public API — called by subsystem periodic()
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Command duty cycle output.
     * Runs encoder-sync correction, applies hardware or software ramp, then
     * logs telemetry — all automatically. No additional calls needed in periodic().
     *
     * @param duty -1.0 to 1.0
     */
    public final void setDutyCycle(double duty) {
        if (isHoming()) {
            DriverStation.reportWarning("[mechanismUnit] '" + config.name
                + "': setDutyCycle() called while homing — command ignored.", false);
            return;
        }
        runEncoderSync();
        checkTravelLimits();
        duty = applyTravelLimitBlock(duty, Math.signum(duty));
        applyDashboardPIDUpdates();
        double ramped = usesHardwareDutyCycleRamp() ? duty : applyDutyCycleRamp(duty);
        applyDutyCycleImpl(ramped);
        logDutyCycle.append(ramped);
        logTelemetry();
        logTravelLimitState();
    }

    /**
     * Command velocity setpoint (mechanism shaft, rotations/second).
     * Runs encoder-sync correction, computes and injects tier-2 + tier-3 FF,
     * updates PID from SmartDashboard, and logs telemetry — all automatically.
     * No additional calls needed in periodic().
     *
     * @param velocityRps target velocity (rotations/second, mechanism shaft)
     */
    public final void setVelocity(double velocityRps) {
        if (isHoming()) {
            DriverStation.reportWarning("[mechanismUnit] '" + config.name
                + "': setVelocity() called while homing — command ignored.", false);
            return;
        }
        runEncoderSync();
        checkTravelLimits();
        velocityRps = applyTravelLimitBlock(velocityRps, Math.signum(velocityRps));
        applyDashboardPIDUpdates();
        updateAcceleration();
        double ffVolts = computeTotalFF();
        applyVelocityImpl(velocityRps, ffVolts);
        lastSetpoint           = velocityRps;
        lastSetpointIsVelocity = true;
        logCommandedVelocityRps.append(velocityRps);
        logAppliedFFVolts.append(ffVolts);
        logTelemetry();
        logTravelLimitState();
    }

    /**
     * Command position setpoint (mechanism shaft, degrees).
     * Runs encoder-sync correction, computes and injects tier-2 + tier-3 FF,
     * updates PID from SmartDashboard, and logs telemetry — all automatically.
     * No additional calls needed in periodic().
     *
     * @param positionDeg target position (degrees, mechanism shaft)
     */
    public final void setPosition(double positionDeg) {
        if (isHoming()) {
            DriverStation.reportWarning("[mechanismUnit] '" + config.name
                + "': setPosition() called while homing — command ignored.", false);
            return;
        }
        runEncoderSync();
        checkTravelLimits();
        // Position blocking: substitute current position to hold in place when
        // commanded into a latched limit. Clear the opposite latch when moving away.
        double currentDeg = getPositionImpl() * 360.0;
        double direction  = Math.signum(positionDeg - currentDeg);
        if (direction > 0 && isUpperLimitReached()) {
            positionDeg = currentDeg;
        } else if (direction < 0 && isLowerLimitReached()) {
            positionDeg = currentDeg;
        } else if (direction > 0) {
            clearLowerLatches();
        } else if (direction < 0) {
            clearUpperLatches();
        }
        applyDashboardPIDUpdates();
        updateAcceleration();
        double ffVolts      = computeTotalFF();
        double positionRot  = positionDeg / 360.0;
        applyPositionImpl(positionRot, ffVolts);
        lastSetpoint           = positionDeg;
        lastSetpointIsVelocity = false;
        logCommandedPositionDeg.append(positionDeg);
        logAppliedFFVolts.append(ffVolts);
        logTelemetry();
        logTravelLimitState();
    }

    /**
     * Stop the motor and reset ramp state. Logs final telemetry.
     * After calling stop(), the next duty cycle ramp will start from zero.
     */
    public final void stop() {
        rampedDutyCycle          = 0.0;
        rampedVelocitySetpoint   = 0.0;
        positionInitialized      = false;
        stopImpl();
        logTelemetry();
    }

    /**
     * Log all telemetry for this mechanism to DataLog and SmartDashboard.
     * Called automatically by setDutyCycle(), setVelocity(), setPosition(),
     * and stop() — no need to call this from subsystem periodic() directly.
     * Available as public for any case where telemetry is needed without
     * issuing a new command (e.g. reading state during disabled mode).
     */
    public final void logTelemetry() {
        double vel = getVelocityImpl();
        double pos = getPositionImpl() * 360.0;

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

    /**
     * Run encoder-sync correction for all ENCODER_SYNC followers.
     * Called automatically at the start of every set*() command.
     * No-op when no followers use ENCODER_SYNC mode.
     */
    private void runEncoderSync() {
        for (int i = 0; i < config.followerModes.length; i++) {
            if (config.followerModes[i] != motorConstants.FollowMode.ENCODER_SYNC) continue;

            int leaderIndex = config.followerLeaderIndices[i];
            double leaderPosRot   = leaderIndex == 0
                ? getPositionImpl()
                : getFollowerPositionImpl(leaderIndex);
            double followerPosRot = getFollowerPositionImpl(i + 1);
            double errorRot       = leaderPosRot - followerPosRot;

            if (logSyncErrorRot != null) logSyncErrorRot.append(errorRot);
            if (Math.abs(errorRot) > config.encoderSyncDeadbandRot) {
                double correction = config.encoderSyncKp * errorRot;
                correction = Math.max(-1.0, Math.min(1.0, correction));
                applyFollowerCorrectionImpl(i + 1, correction);
                if (logSyncOutput != null) logSyncOutput.append(correction);
            } else {
                if (logSyncOutput != null) logSyncOutput.append(0.0);
            }
        }
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

    /**
     * Returns true when the mechanism is within its configured setpoint deadband.
     * Uses the most recently commanded setpoint from setVelocity() or setPosition().
     *
     * Velocity mechanisms: compares actual RPS against the last setVelocity() value.
     * Position mechanisms: compares actual degrees against the last setPosition() value.
     *
     * Always returns false if config.setpointDeadband was not set (default 0.0).
     * A warning is logged to the driver station the first time this is called
     * without a deadband configured — and once at construction time.
     */
    public final boolean isAtSetpoint() {
        if (config.setpointDeadband <= 0.0) {
            if (!atSetpointWarningLogged) {
                DriverStation.reportWarning(
                    "[mechanismUnit] '" + config.name + "': isAtSetpoint() called but "
                    + "setpointDeadband is not configured — returning false. "
                    + "Call withSetpointDeadband() in the mechanismConfig.Builder.", false);
                atSetpointWarningLogged = true;
            }
            return false;
        }
        double actual = lastSetpointIsVelocity ? getVelocityImpl() : getPositionImpl() * 360.0;
        return Math.abs(actual - lastSetpoint) <= config.setpointDeadband;
    }

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
        // Allocate sync log entries if any follower uses ENCODER_SYNC
        boolean hasEncoderSync = false;
        for (motorConstants.FollowMode m : config.followerModes)
            if (m == motorConstants.FollowMode.ENCODER_SYNC) { hasEncoderSync = true; break; }
        if (hasEncoderSync) {
            logSyncErrorRot = new DoubleLogEntry(log, p + "EncoderSync/Error_rot");
            logSyncOutput   = new DoubleLogEntry(log, p + "EncoderSync/Output");
        }
        // Travel limit log entries — allocated only when limits are configured
        boolean hasTravelLimits = !Double.isNaN(config.leadLowerCurrentAmps)
                               || !Double.isNaN(config.leadUpperCurrentAmps);
        if (hasTravelLimits) {
            logLeadLowerLimit = new BooleanLogEntry(log, p + "TravelLimits/LeadLower");
            logLeadUpperLimit = new BooleanLogEntry(log, p + "TravelLimits/LeadUpper");
            if (encoderSyncFollowerIndex > 0) {
                logFollowerLowerLimit = new BooleanLogEntry(log, p + "TravelLimits/FollowerLower");
                logFollowerUpperLimit = new BooleanLogEntry(log, p + "TravelLimits/FollowerUpper");
            }
        }
        // Homing log entry — always allocated (homing is always available)
        logIsHoming = new BooleanLogEntry(log, p + "Homing/IsHoming");
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Travel limit detection — private helpers
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Update stall counters for lead and (if present) ENCODER_SYNC follower.
     * A limit latches when both detection current >= threshold AND
     * |velocity| <= threshold hold for stallDetectionCycles consecutive loops.
     * No-op when no travel limits are configured (early return).
     * Called at the start of every set*() invocation.
     */
    private void checkTravelLimits() {
        if (Double.isNaN(config.leadLowerCurrentAmps) && Double.isNaN(config.leadUpperCurrentAmps))
            return;

        double leadCurrent = getDetectionCurrentImpl();
        double leadAbsVel  = Math.abs(getVelocityImpl());

        if (!Double.isNaN(config.leadLowerCurrentAmps) && !leadLowerLimitLatched) {
            if (leadCurrent >= config.leadLowerCurrentAmps && leadAbsVel <= config.leadLowerMaxSpeedRps) {
                if (++leadLowerStallCount >= config.stallDetectionCycles) leadLowerLimitLatched = true;
            } else {
                leadLowerStallCount = 0;
            }
        }
        if (!Double.isNaN(config.leadUpperCurrentAmps) && !leadUpperLimitLatched) {
            if (leadCurrent >= config.leadUpperCurrentAmps && leadAbsVel <= config.leadUpperMaxSpeedRps) {
                if (++leadUpperStallCount >= config.stallDetectionCycles) leadUpperLimitLatched = true;
            } else {
                leadUpperStallCount = 0;
            }
        }

        if (encoderSyncFollowerIndex < 0) return;

        // Resolve effective follower thresholds — NaN inherits the lead value
        double effLowerCurrent = Double.isNaN(config.followerLowerCurrentAmps) ? config.leadLowerCurrentAmps : config.followerLowerCurrentAmps;
        double effLowerSpeed   = Double.isNaN(config.followerLowerMaxSpeedRps)  ? config.leadLowerMaxSpeedRps  : config.followerLowerMaxSpeedRps;
        double effUpperCurrent = Double.isNaN(config.followerUpperCurrentAmps) ? config.leadUpperCurrentAmps : config.followerUpperCurrentAmps;
        double effUpperSpeed   = Double.isNaN(config.followerUpperMaxSpeedRps)  ? config.leadUpperMaxSpeedRps  : config.followerUpperMaxSpeedRps;

        double followerCurrent = getFollowerDetectionCurrentImpl(encoderSyncFollowerIndex);
        double followerAbsVel  = Math.abs(getFollowerVelocityImpl(encoderSyncFollowerIndex));

        if (!Double.isNaN(effLowerCurrent) && !followerLowerLimitLatched) {
            if (followerCurrent >= effLowerCurrent && followerAbsVel <= effLowerSpeed) {
                if (++followerLowerStallCount >= config.stallDetectionCycles) followerLowerLimitLatched = true;
            } else {
                followerLowerStallCount = 0;
            }
        }
        if (!Double.isNaN(effUpperCurrent) && !followerUpperLimitLatched) {
            if (followerCurrent >= effUpperCurrent && followerAbsVel <= effUpperSpeed) {
                if (++followerUpperStallCount >= config.stallDetectionCycles) followerUpperLimitLatched = true;
            } else {
                followerUpperStallCount = 0;
            }
        }
    }

    /** Clear lower-limit latches and reset their stall counters. */
    private void clearLowerLatches() {
        leadLowerLimitLatched     = false;
        leadLowerStallCount       = 0;
        followerLowerLimitLatched = false;
        followerLowerStallCount   = 0;
    }

    /** Clear upper-limit latches and reset their stall counters. */
    private void clearUpperLatches() {
        leadUpperLimitLatched     = false;
        leadUpperStallCount       = 0;
        followerUpperLimitLatched = false;
        followerUpperStallCount   = 0;
    }

    /**
     * Block a duty-cycle or velocity command in a latched direction.
     * Returns 0.0 when {@code direction} points into a latched limit.
     * Auto-clears the opposite latch when the command moves away from it.
     *
     * @param value     raw command value (duty cycle or velocity)
     * @param direction Math.signum of value (+1, -1, or 0)
     * @return the original value, or 0.0 if blocked
     */
    private double applyTravelLimitBlock(double value, double direction) {
        if (direction > 0 && isUpperLimitReached()) return 0.0;
        if (direction < 0 && isLowerLimitReached()) return 0.0;
        // Moving away from a limit: auto-clear the opposite latch
        if (direction > 0) clearLowerLatches();
        if (direction < 0) clearUpperLatches();
        return value;
    }

    /** Append travel limit latch state to DataLog and SmartDashboard. */
    private void logTravelLimitState() {
        if (logLeadLowerLimit != null) {
            logLeadLowerLimit.append(leadLowerLimitLatched);
            logLeadUpperLimit.append(leadUpperLimitLatched);
            SmartDashboard.putBoolean(dashPrefix + "TravelLimits/LeadLower", leadLowerLimitLatched);
            SmartDashboard.putBoolean(dashPrefix + "TravelLimits/LeadUpper", leadUpperLimitLatched);
            if (logFollowerLowerLimit != null) {
                logFollowerLowerLimit.append(followerLowerLimitLatched);
                logFollowerUpperLimit.append(followerUpperLimitLatched);
                SmartDashboard.putBoolean(dashPrefix + "TravelLimits/FollowerLower", followerLowerLimitLatched);
                SmartDashboard.putBoolean(dashPrefix + "TravelLimits/FollowerUpper", followerUpperLimitLatched);
            }
        }
        logIsHoming.append(isHoming());
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Travel limit state reporting — public API
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * True if the lower (reverse) travel limit is latched on either the lead or
     * the ENCODER_SYNC follower. Cleared automatically when a positive-direction
     * command is issued after the limit was reached.
     */
    public final boolean isLowerLimitReached() {
        return leadLowerLimitLatched || followerLowerLimitLatched;
    }

    /**
     * True if the upper (forward) travel limit is latched on either the lead or
     * the ENCODER_SYNC follower.
     */
    public final boolean isUpperLimitReached() {
        return leadUpperLimitLatched || followerUpperLimitLatched;
    }

    /** True if the lead motor's lower travel limit is currently latched. */
    public final boolean isLeadLowerLimitReached()     { return leadLowerLimitLatched; }

    /** True if the lead motor's upper travel limit is currently latched. */
    public final boolean isLeadUpperLimitReached()     { return leadUpperLimitLatched; }

    /**
     * True if the ENCODER_SYNC follower's lower travel limit is latched.
     * Always false on mechanisms without an ENCODER_SYNC follower.
     */
    public final boolean isFollowerLowerLimitReached() { return followerLowerLimitLatched; }

    /**
     * True if the ENCODER_SYNC follower's upper travel limit is latched.
     * Always false on mechanisms without an ENCODER_SYNC follower.
     */
    public final boolean isFollowerUpperLimitReached() { return followerUpperLimitLatched; }

    // ─────────────────────────────────────────────────────────────────────────
    // Homing — public API
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Begin homing with the same current/speed thresholds for both lead and follower.
     * Drives the mechanism in {@code dir} at {@code dutyCycle} magnitude until
     * stall is detected on each side independently.
     *
     * <p>Call {@link #updateHoming()} from the subsystem periodic every loop
     * while {@link #isHoming()} returns true. Do NOT call set*() while homing.</p>
     *
     * @param dir          direction to drive during homing
     * @param dutyCycle    magnitude of duty cycle to apply (will be sign-adjusted by dir)
     * @param minCurrentAmps minimum detection current (amps) for stall on both sides
     * @param maxSpeedRps  maximum |velocity| (RPS) for stall on both sides
     */
    public final void startHoming(HomingDirection dir, double dutyCycle,
                                  double minCurrentAmps, double maxSpeedRps) {
        startHoming(dir, dutyCycle, minCurrentAmps, maxSpeedRps, minCurrentAmps, maxSpeedRps);
    }

    /**
     * Begin homing with separate current/speed thresholds for lead and follower.
     * Useful when the two sides have different mechanical loads.
     *
     * @param dir                  direction to drive during homing
     * @param dutyCycle            magnitude of duty cycle (will be sign-adjusted by dir)
     * @param leadCurrentAmps      minimum detection current (amps) for lead-side stall
     * @param leadSpeedRps         maximum |velocity| (RPS) for lead-side stall
     * @param followerCurrentAmps  minimum detection current (amps) for follower-side stall
     * @param followerSpeedRps     maximum |velocity| (RPS) for follower-side stall
     */
    public final void startHoming(HomingDirection dir, double dutyCycle,
                                  double leadCurrentAmps, double leadSpeedRps,
                                  double followerCurrentAmps, double followerSpeedRps) {
        homingDirection           = dir;
        homingDutyCycle           = Math.abs(dutyCycle);
        homingLeadCurrentAmps     = leadCurrentAmps;
        homingLeadSpeedRps        = leadSpeedRps;
        homingFollowerCurrentAmps = followerCurrentAmps;
        homingFollowerSpeedRps    = followerSpeedRps;
        homingLeadStallCount      = 0;
        homingFollowerStallCount  = 0;
        leadHomed                 = false;
        // For mechanisms with no ENCODER_SYNC follower, consider follower vacuously homed
        followerHomed             = (encoderSyncFollowerIndex < 0);
        homingState               = HomingState.IN_PROGRESS;
    }

    /**
     * Advance the homing state machine. Must be called from the subsystem's
     * {@code periodic()} every loop while {@link #isHoming()} returns true.
     *
     * <p>Drives each side toward its hard stop, stops it when stall is confirmed,
     * and resets encoders once both sides complete. ENCODER_SYNC correction is
     * naturally suppressed (this method drives the follower directly and does
     * not call {@code runEncoderSync()}).</p>
     *
     * <p>Do NOT call setDutyCycle(), setVelocity(), or setPosition() while homing.</p>
     */
    public final void updateHoming() {
        if (homingState != HomingState.IN_PROGRESS) return;

        double leadDuty = (homingDirection == HomingDirection.POSITIVE)
            ? +homingDutyCycle : -homingDutyCycle;

        // ── Lead side ────────────────────────────────────────────────────────
        if (!leadHomed) {
            double leadCurrent = getDetectionCurrentImpl();
            double leadAbsVel  = Math.abs(getVelocityImpl());
            boolean leadStalled = leadCurrent >= homingLeadCurrentAmps
                               && leadAbsVel  <= homingLeadSpeedRps;
            if (leadStalled) {
                if (++homingLeadStallCount >= config.stallDetectionCycles) {
                    applyDutyCycleImpl(0.0);
                    leadHomed = true;
                }
            } else {
                homingLeadStallCount = 0;
                applyDutyCycleImpl(leadDuty);
            }
        }

        // ── Follower side (ENCODER_SYNC only) ─────────────────────────────────
        if (encoderSyncFollowerIndex > 0 && !followerHomed) {
            double followerCurrent = getFollowerDetectionCurrentImpl(encoderSyncFollowerIndex);
            double followerAbsVel  = Math.abs(getFollowerVelocityImpl(encoderSyncFollowerIndex));
            boolean followerStalled = followerCurrent >= homingFollowerCurrentAmps
                                   && followerAbsVel  <= homingFollowerSpeedRps;
            if (followerStalled) {
                if (++homingFollowerStallCount >= config.stallDetectionCycles) {
                    applyFollowerCorrectionImpl(encoderSyncFollowerIndex, 0.0);
                    followerHomed = true;
                }
            } else {
                homingFollowerStallCount = 0;
                // Drive follower in same mechanism direction.
                // The follower's own hardware inversion config handles physical direction;
                // no extra sign flip is needed here.
                applyFollowerCorrectionImpl(encoderSyncFollowerIndex, leadDuty);
            }
        }

        // ── Completion check ──────────────────────────────────────────────────
        if (leadHomed && followerHomed) {
            double zeroRot = Double.isNaN(config.homingZeroOffsetDeg)
                ? 0.0 : config.homingZeroOffsetDeg / 360.0;
            resetLeadEncoderImpl(zeroRot);
            if (encoderSyncFollowerIndex > 0) resetFollowerEncoderImpl(encoderSyncFollowerIndex, zeroRot);
            homingState = HomingState.COMPLETE;
        }

        // ── Telemetry ─────────────────────────────────────────────────────────
        SmartDashboard.putBoolean(dashPrefix + "Homing/IsHoming",      isHoming());
        SmartDashboard.putBoolean(dashPrefix + "Homing/LeadHomed",     leadHomed);
        if (encoderSyncFollowerIndex > 0)
            SmartDashboard.putBoolean(dashPrefix + "Homing/FollowerHomed", followerHomed);
        logIsHoming.append(isHoming());
    }

    /**
     * Abort an in-progress homing operation. Both motors are stopped immediately.
     * Encoders are NOT reset. {@link #isHoming()} returns false after this call.
     */
    public final void cancelHoming() {
        if (homingState != HomingState.IN_PROGRESS) return;
        applyDutyCycleImpl(0.0);
        if (encoderSyncFollowerIndex > 0)
            applyFollowerCorrectionImpl(encoderSyncFollowerIndex, 0.0);
        homingState   = HomingState.IDLE;
        leadHomed     = false;
        followerHomed = false;
    }

    /** True while a homing operation is in progress (started but not complete or cancelled). */
    public final boolean isHoming() { return homingState == HomingState.IN_PROGRESS; }

    /** True once homing has successfully completed and encoders have been reset. */
    public final boolean isHomingComplete() { return homingState == HomingState.COMPLETE; }

    /** True if the lead motor's side has finished homing. */
    public final boolean isLeadHomed() { return leadHomed; }

    /**
     * True if the ENCODER_SYNC follower's side has finished homing.
     * Always returns false when no ENCODER_SYNC follower is present
     * (use {@link #isHomingComplete()} as the canonical completion check).
     */
    public final boolean isFollowerHomed() {
        return encoderSyncFollowerIndex > 0 && followerHomed;
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
     * Return the current reading best suited for stall detection on the lead motor.
     * Vendor contracts:
     *   CTRE  — stator current (directly proportional to torque, ideal for stall)
     *   REV   — output current (best approximation available on SparkMax/Flex)
     *   Nova  — stator current (ThriftyNova exposes stator current natively)
     *
     * @return detection current in amps
     */
    protected abstract double getDetectionCurrentImpl();

    /**
     * Return the stall-detection current for an ENCODER_SYNC follower motor.
     * Same vendor-selection logic as {@link #getDetectionCurrentImpl()}.
     * Only called when encoderSyncFollowerIndex > 0.
     *
     * @param followerIndex 1-based index into config.canIds[]
     * @return detection current in amps
     */
    protected abstract double getFollowerDetectionCurrentImpl(int followerIndex);

    /**
     * Return the velocity of an ENCODER_SYNC follower motor (mechanism shaft RPS).
     * Only called when encoderSyncFollowerIndex > 0.
     *
     * @param followerIndex 1-based index into config.canIds[]
     * @return follower velocity in rotations/second (signed)
     */
    protected abstract double getFollowerVelocityImpl(int followerIndex);

    /**
     * Write a new encoder position to the lead motor.
     * Called once by the homing system after both sides have confirmed contact.
     *
     * @param positionRot mechanism shaft rotations to set as the current position
     */
    protected abstract void resetLeadEncoderImpl(double positionRot);

    /**
     * Write a new encoder position to an ENCODER_SYNC follower motor.
     * Called in the same homing completion sequence as {@link #resetLeadEncoderImpl}.
     * Only called when encoderSyncFollowerIndex > 0.
     *
     * @param followerIndex 1-based index into config.canIds[]
     * @param positionRot   mechanism shaft rotations to set as the current position
     */
    protected abstract void resetFollowerEncoderImpl(int followerIndex, double positionRot);

    /**
     * Returns true when the vendor implementation has configured a hardware duty
     * cycle ramp, making the software ramp in this base class redundant.
     * CTRE and REV override this to return true when openLoopRampSecs > 0.
     * Nova returns false — its ramp support is less reliable and the software
     * fallback is preferred.
     */
    protected boolean usesHardwareDutyCycleRamp() { return false; }

    /**
     * Verify this mechanism's motor controller configuration(s).
     * Called once by ConfigVerifier.runAll() in robotInit(), before any
     * motor is commanded. Retries apply up to 5×, then reads back and
     * diffs key fields (inversion, brake mode, current limits, etc.).
     *
     * @return one ConfigVerifyResult per physical motor controller verified
     *         (leader + any ENCODER_SYNC followers)
     */
    @Override
    public abstract List<ConfigVerifyResult> verifyConfig();

}
