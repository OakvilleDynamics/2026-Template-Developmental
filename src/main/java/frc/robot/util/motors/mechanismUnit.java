package frc.robot.util.motors;

import java.util.function.Supplier;

import edu.wpi.first.util.datalog.*;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.swerveDrive.driveOdometryState;

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

    // =========================================================================
    // FF — Static factory library for physics-based feed-forward providers
    // =========================================================================

    /**
     * mechanismUnit.FF
     *
     * Static factory library for physics-based ffProvider lambdas.
     * All voltage calculations are derived from motor datasheet constants
     * (via motorModels) rather than empirical calibration voltages, so
     * values are grounded in vendor-verified specs and require no per-mechanism
     * holding-voltage measurement.
     *
     * ─── VOLTAGE FORMULA ─────────────────────────────────────────────────────
     * For a load requiring torque τ_mech at the mechanism shaft:
     *
     *   V_ff = τ_mech × 12V / (gearRatio × motor.stallTorqueNm)
     *
     * On-controller kV (Tier 1) already handles dynamic back-EMF; these
     * factories target the static/quasi-static gravity and spring loads only.
     *
     * ─── UNITS AT THE INTERFACE ──────────────────────────────────────────────
     * Masses    : lbs  (converted to kg internally)
     * Distances : inches  (converted to meters internally)
     * Angles    : degrees  (matches ffProvider positionDeg convention)
     * Returns   : volts
     *
     * ─── TIER PLACEMENT ──────────────────────────────────────────────────────
     * Tier 2 (self-contained): springTurret, rotatingArm
     * Tier 3 (cross-system):   multiStageElevator, pivotingElevator
     *   Tier-3 factories accept Supplier arguments that close over other
     *   subsystem references; build these in RobotContainer.
     */
    public static final class FF {

        private static final double LBS_TO_KG  = 0.453592;
        private static final double IN_TO_M    = 0.0254;
        private static final double G_MPS2     = 9.80665;
        private static final double V_NOMINAL  = 12.0;

        // No instances
        private FF() {}

        // ─────────────────────────────────────────────────────────────────────
        // Shared helper
        // ─────────────────────────────────────────────────────────────────────

        /**
         * Convert torque at the mechanism shaft to a feed-forward voltage.
         * V = τ_mech × V_nominal / (gearRatio × stallTorqueNm)
         */
        private static double torqueToVolts(double torqueNm, double gearRatio,
                                            motorModels.MotorModel motor) {
            return torqueNm * V_NOMINAL / (gearRatio * motor.stallTorqueNm);
        }

        // ─────────────────────────────────────────────────────────────────────
        // TIER 2 — Self-contained (no external subsystem references)
        // ─────────────────────────────────────────────────────────────────────

        /**
         * Piecewise-linear feed-forward for a mechanism with a non-linear
         * restoring force (constant-force spring, surgical tubing, gas spring).
         *
         * Calibration points are measured torques at the mechanism shaft in lb·in.
         * Measure by holding a known force (e.g. a fish scale) at a known distance
         * from the pivot at each angle, then multiply: torque = force_lbs × distance_in.
         * The sign convention matches the motor output: positive torque = forward direction.
         *
         * Internally converts lb·in → N·m, then uses the same torque-to-volts formula
         * as all other FF factories: V = τ × 12V / (gearRatio × motor.stallTorqueNm).
         *
         * Between points: linear interpolation.
         * Outside the calibrated range: returns 0.0 (intentionally conspicuous —
         * the PID will visibly fight the spring, flagging the gap during tuning).
         *
         * @param motor     motor model from motorModels (e.g. motorModels.NEO)
         * @param gearRatio motor rotations per mechanism shaft rotation
         * @param points    Calibration pairs: double[][]{ {angleDeg, torqueLbIn}, ... }
         *                  Minimum 2 points, sorted ascending by angle.
         */
        public static ffProvider springTurret(motorModels.MotorModel motor, double gearRatio,
                                              double[][] points) {
            if (points == null || points.length < 2)
                throw new IllegalArgumentException(
                    "mechanismUnit.FF.springTurret: requires at least 2 calibration points");
            for (int i = 0; i < points.length; i++) {
                if (points[i].length != 2)
                    throw new IllegalArgumentException(
                        "mechanismUnit.FF.springTurret: each point must be double[]{ angleDeg, torqueLbIn }");
                if (i > 0 && points[i][0] <= points[i - 1][0])
                    throw new IllegalArgumentException(
                        "mechanismUnit.FF.springTurret: points must be sorted ascending by angleDeg. "
                        + "Found " + points[i][0] + " at index " + i + " after " + points[i - 1][0]);
            }

            // Convert torques to N·m and defensive-copy angles — avoid repeated math per cycle
            final double LB_IN_TO_NM = 0.112985;
            final double[] angles    = new double[points.length];
            final double[] torquesNm = new double[points.length];
            for (int i = 0; i < points.length; i++) {
                angles[i]    = points[i][0];
                torquesNm[i] = points[i][1] * LB_IN_TO_NM;
            }

            return (positionDeg, velocityRps, accelRpss) -> {
                // Out of range → 0 intentionally (see javadoc)
                if (positionDeg < angles[0] || positionDeg > angles[angles.length - 1]) return 0.0;

                // Binary search for the containing bracket
                int lo = 0, hi = angles.length - 2;
                while (lo < hi) {
                    int mid = (lo + hi + 1) / 2;
                    if (angles[mid] <= positionDeg) lo = mid; else hi = mid - 1;
                }

                double t          = (positionDeg - angles[lo]) / (angles[lo + 1] - angles[lo]);
                double torqueNm   = torquesNm[lo] + t * (torquesNm[lo + 1] - torquesNm[lo]);
                return torqueToVolts(torqueNm, gearRatio, motor);
            };
        }

        /**
         * Gravity compensation for a rotating arm whose center of mass changes
         * as game pieces are acquired or released.
         *
         * Physics:
         *   τ_mech = [armMass × armCg + Σ(count_i × pieceMass_i × pieceCg_i)] × g × cos(pos°)
         *   V_ff   = τ_mech × 12V / (gearRatio × motor.stallTorqueNm)
         *
         * positionDeg convention: 0° = horizontal (full gravity load), 90° = vertical (zero load).
         *
         * For a fixed-CG arm with no game pieces, pass empty arrays for the
         * gamePiece* parameters.
         *
         * @param motor               motor model from motorModels (e.g. motorModels.NEO)
         * @param gearRatio           motor rotations per mechanism shaft rotation
         * @param armMassLbs          mass of the arm structure itself (lbs)
         * @param armCgDistanceIn     distance from pivot to arm structure CG (inches)
         * @param gamePieceTypesLbs   mass of each game piece type (lbs). Length = N types.
         * @param pieceCgDistancesIn  distance from pivot to each piece type's CG when held (inches).
         *                            Parallel to gamePieceTypesLbs.
         * @param pieceCountSuppliers runtime count for each game piece type, parallel to above.
         *                            Each supplier returns the integer count currently held.
         */
        @SuppressWarnings("unchecked") // new Supplier[0] safe — only read as Supplier<Integer>
        public static ffProvider rotatingArm(
                motorModels.MotorModel motor,
                double gearRatio,
                double armMassLbs,
                double armCgDistanceIn,
                double[] gamePieceTypesLbs,
                double[] pieceCgDistancesIn,
                Supplier<Integer>[] pieceCountSuppliers) {

            if (gamePieceTypesLbs == null)    gamePieceTypesLbs    = new double[0];
            if (pieceCgDistancesIn == null)   pieceCgDistancesIn   = new double[0];
            if (pieceCountSuppliers == null)  pieceCountSuppliers  = new Supplier[0];

            if (gamePieceTypesLbs.length != pieceCgDistancesIn.length
                    || gamePieceTypesLbs.length != pieceCountSuppliers.length)
                throw new IllegalArgumentException(
                    "mechanismUnit.FF.rotatingArm: gamePieceTypesLbs, pieceCgDistancesIn, "
                    + "and pieceCountSuppliers must all be the same length.");

            final double armKg   = armMassLbs * LBS_TO_KG;
            final double armCgM  = armCgDistanceIn * IN_TO_M;
            final double[] pieceKg  = new double[gamePieceTypesLbs.length];
            final double[] pieceCgM = new double[pieceCgDistancesIn.length];
            for (int i = 0; i < pieceKg.length; i++) {
                pieceKg[i]  = gamePieceTypesLbs[i] * LBS_TO_KG;
                pieceCgM[i] = pieceCgDistancesIn[i] * IN_TO_M;
            }
            final Supplier<Integer>[] counts = pieceCountSuppliers;

            return (positionDeg, velocityRps, accelRpss) -> {
                // Effective moment arm = mass × CG distance, summed for all contributors
                double momentKgM = armKg * armCgM;
                for (int i = 0; i < pieceKg.length; i++) {
                    momentKgM += counts[i].get() * pieceKg[i] * pieceCgM[i];
                }
                double torqueNm = momentKgM * G_MPS2 * Math.cos(Math.toRadians(positionDeg));
                return torqueToVolts(torqueNm, gearRatio, motor);
            };
        }

        // ─────────────────────────────────────────────────────────────────────
        // TIER 3 — Cross-system (require external subsystem references)
        // ─────────────────────────────────────────────────────────────────────

        /**
         * Gravity compensation for a multi-stage linear elevator.
         *
         * The elevator angle relative to ground is read each cycle via
         * {@code elevatorAngleDegSupplier}:
         *   - Permanently vertical:  {@code () -> 90.0}
         *   - Pivoting base:         {@code pivotArm::getPositionDeg}
         *
         * This makes the factory Tier 3 — even a "fixed vertical" elevator uses a
         * supplier so the angle can be driven from a base pivot without code changes.
         *
         * Physics:
         *   F_gravity = totalMass × g × sin(elevatorAngle)
         *     (sin: 90° = vertical = full load, 0° = horizontal = no load)
         *   τ_mech    = F_gravity × spoolRadius
         *   V_ff_base = τ_mech × 12V / (gearRatio × motor.stallTorqueNm)
         *   V_ff      = V_ff_base + frictionOffset (direction-dependent)
         *
         * The {@code frictionOffsetVolts*} parameters absorb chain/belt asymmetry
         * that the physics model cannot capture. Set both to 0 for a first pass;
         * tune by observing position hold error at different load conditions.
         *
         * @param motor                      motor model from motorModels
         * @param gearRatio                  motor rotations per mechanism shaft rotation
         * @param spoolRadiusIn              spool or sprocket radius (inches) — converts linear
         *                                   force to shaft torque
         * @param carriageMassLbs            carriage mass (lbs)
         * @param stageMassesLbs             additional stage masses (lbs). Empty array = single-stage.
         * @param frictionOffsetVoltsUp      constant voltage addend when moving up or holding
         * @param frictionOffsetVoltsDown    constant voltage addend when moving down
         * @param gamePieceTypesLbs          mass of each game piece type (lbs)
         * @param pieceCountSuppliers        runtime count per type, parallel to gamePieceTypesLbs
         * @param elevatorAngleDegSupplier   supplies elevator angle from ground (degrees) each cycle.
         *                                   90° = vertical, 0° = horizontal.
         */
        @SuppressWarnings("unchecked") // new Supplier[0] safe — only read as Supplier<Integer>
        public static ffProvider multiStageElevator(
                motorModels.MotorModel motor,
                double gearRatio,
                double spoolRadiusIn,
                double carriageMassLbs,
                double[] stageMassesLbs,
                double frictionOffsetVoltsUp,
                double frictionOffsetVoltsDown,
                double[] gamePieceTypesLbs,
                Supplier<Integer>[] pieceCountSuppliers,
                Supplier<Double> elevatorAngleDegSupplier) {

            if (stageMassesLbs == null)       stageMassesLbs    = new double[0];
            if (gamePieceTypesLbs == null)    gamePieceTypesLbs = new double[0];
            if (pieceCountSuppliers == null)  pieceCountSuppliers = new Supplier[0];

            if (gamePieceTypesLbs.length != pieceCountSuppliers.length)
                throw new IllegalArgumentException(
                    "mechanismUnit.FF.multiStageElevator: gamePieceTypesLbs and "
                    + "pieceCountSuppliers must be the same length.");

            final double spoolM     = spoolRadiusIn * IN_TO_M;
            final double carriageKg = carriageMassLbs * LBS_TO_KG;
            final double[] stageKg  = new double[stageMassesLbs.length];
            for (int i = 0; i < stageKg.length; i++) stageKg[i] = stageMassesLbs[i] * LBS_TO_KG;
            final double[] pieceKg  = new double[gamePieceTypesLbs.length];
            for (int i = 0; i < pieceKg.length; i++) pieceKg[i] = gamePieceTypesLbs[i] * LBS_TO_KG;
            final Supplier<Integer>[] counts = pieceCountSuppliers;

            return (positionDeg, velocityRps, accelRpss) -> {
                // Total mass this cycle
                double totalKg = carriageKg;
                for (double s : stageKg) totalKg += s;
                for (int i = 0; i < pieceKg.length; i++) totalKg += pieceKg[i] * counts[i].get();

                // Gravity force projected onto lift axis
                double elevAngleRad = Math.toRadians(elevatorAngleDegSupplier.get());
                double forceN       = totalKg * G_MPS2 * Math.sin(elevAngleRad);
                double torqueNm     = forceN * spoolM;
                double baseVolts    = torqueToVolts(torqueNm, gearRatio, motor);
                double friction     = velocityRps >= 0 ? frictionOffsetVoltsUp : frictionOffsetVoltsDown;
                return baseVolts + friction;
            };
        }

        /**
         * Combined gravity + drivetrain inertia feed-forward for an elevator
         * mounted on a pivot at its base.
         *
         * positionDeg: angle of the base pivot (0° = horizontal, 90° = vertical).
         *
         * Three voltage components are summed:
         *
         * 1. GRAVITY — effective load varies with cos(pivotAngle):
         *      τ_grav = totalMass × g × cgDist × cos(pivotAngle)
         *
         * 2. DRIVETRAIN LINEAR INERTIA — forward/backward robot acceleration
         *    creates a pseudo-force in the robot frame that projects onto the
         *    pivot load axis via sin(pivotAngle):
         *      accelFwd = linearAccelMag × cos(accelHeading − robotHeading)
         *      τ_inertia = totalMass × accelFwd × cgDist × sin(pivotAngle)
         *
         * 3. CENTRIPETAL — robot rotation causes the elevator CG to trace an
         *    arc, adding inward centripetal acceleration:
         *      a_centripetal = ω² × cgOffsetFromCenter
         *      τ_centripetal = totalMass × a_centripetal × cgDist × sin(pivotAngle)
         *
         * Robot state is read from driveOdometryState.blendedState each cycle.
         *
         * @param motor                      motor model from motorModels
         * @param gearRatio                  motor rotations per mechanism shaft rotation
         * @param cgDistanceFromPivotIn      distance from base pivot to assembly CG (inches)
         * @param cgOffsetFromRobotCenterIn  distance from robot center to elevator CG,
         *                                   projected onto the rotation plane (inches)
         * @param carriageMassLbs            carriage mass (lbs)
         * @param stageMassesLbs             additional stage masses (lbs)
         * @param frictionOffsetVoltsUp      constant addend when pivoting up or holding
         * @param frictionOffsetVoltsDown    constant addend when pivoting down
         * @param gamePieceTypesLbs          mass of each game piece type (lbs)
         * @param pieceCountSuppliers        runtime count per type, parallel to gamePieceTypesLbs
         * @param odometryStateSupplier      supplies driveOdometryState each cycle
         * @param robotHeadingSupplier       supplies robot heading (radians, field-relative, CCW+)
         */
        @SuppressWarnings("unchecked") // new Supplier[0] safe — only read as Supplier<Integer>
        public static ffProvider pivotingElevator(
                motorModels.MotorModel motor,
                double gearRatio,
                double cgDistanceFromPivotIn,
                double cgOffsetFromRobotCenterIn,
                double carriageMassLbs,
                double[] stageMassesLbs,
                double frictionOffsetVoltsUp,
                double frictionOffsetVoltsDown,
                double[] gamePieceTypesLbs,
                Supplier<Integer>[] pieceCountSuppliers,
                Supplier<driveOdometryState> odometryStateSupplier,
                Supplier<Double> robotHeadingSupplier) {

            if (stageMassesLbs == null)       stageMassesLbs      = new double[0];
            if (gamePieceTypesLbs == null)    gamePieceTypesLbs   = new double[0];
            if (pieceCountSuppliers == null)  pieceCountSuppliers = new Supplier[0];

            if (gamePieceTypesLbs.length != pieceCountSuppliers.length)
                throw new IllegalArgumentException(
                    "mechanismUnit.FF.pivotingElevator: gamePieceTypesLbs and "
                    + "pieceCountSuppliers must be the same length.");

            final double cgDistM       = cgDistanceFromPivotIn * IN_TO_M;
            final double cgOffsetM     = cgOffsetFromRobotCenterIn * IN_TO_M;
            final double carriageKg    = carriageMassLbs * LBS_TO_KG;
            final double[] stageKg     = new double[stageMassesLbs.length];
            for (int i = 0; i < stageKg.length; i++) stageKg[i] = stageMassesLbs[i] * LBS_TO_KG;
            final double[] pieceKg     = new double[gamePieceTypesLbs.length];
            for (int i = 0; i < pieceKg.length; i++) pieceKg[i] = gamePieceTypesLbs[i] * LBS_TO_KG;
            final Supplier<Integer>[] counts = pieceCountSuppliers;

            return (positionDeg, velocityRps, accelRpss) -> {
                // ── Total mass this cycle ─────────────────────────────────────
                double totalKg = carriageKg;
                for (double s : stageKg) totalKg += s;
                for (int i = 0; i < pieceKg.length; i++) totalKg += pieceKg[i] * counts[i].get();

                double pivotRad = Math.toRadians(positionDeg);

                // ── 1. Gravity ────────────────────────────────────────────────
                // cos(pivot): max at 0° (horizontal), zero at 90° (vertical)
                double torqueGrav    = totalKg * G_MPS2 * cgDistM * Math.cos(pivotRad);
                double voltsGrav     = torqueToVolts(torqueGrav, gearRatio, motor);

                // ── 2. Drivetrain linear inertia ──────────────────────────────
                driveOdometryState odom = odometryStateSupplier.get();
                double robotHeading     = robotHeadingSupplier.get();
                double accelMag         = odom.blendedState.linearAccelerationMagnitude;
                double accelHeading     = odom.blendedState.linearAccelerationHeading;
                // Project field-frame acceleration onto robot forward axis
                double accelFwdMps2     = accelMag * Math.cos(accelHeading - robotHeading);
                // sin(pivot): zero at 0° (horizontal, inertia along arm), max at 90° (vertical)
                double torqueInertia    = totalKg * accelFwdMps2 * cgDistM * Math.sin(pivotRad);
                double voltsInertia     = torqueToVolts(torqueInertia, gearRatio, motor);

                // ── 3. Centripetal ────────────────────────────────────────────
                double omega            = odom.blendedState.angularVelocity;
                double aCentripetal     = omega * omega * cgOffsetM;
                double torqueCentripetal = totalKg * aCentripetal * cgDistM * Math.sin(pivotRad);
                double voltsCentripetal = torqueToVolts(torqueCentripetal, gearRatio, motor);

                double friction = velocityRps >= 0 ? frictionOffsetVoltsUp : frictionOffsetVoltsDown;
                return voltsGrav + voltsInertia + voltsCentripetal + friction;
            };
        }
    }
}
