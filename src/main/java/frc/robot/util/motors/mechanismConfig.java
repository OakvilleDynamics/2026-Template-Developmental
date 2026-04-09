package frc.robot.util.motors;

/**
 * mechanismConfig.java
 * PATH: src/main/java/frc/robot/util/motors/mechanismConfig.java
 *
 * Immutable configuration object for a mechanismUnit. Build one with the
 * inner Builder, then pass it to mechanismUnit.create().
 *
 * ─── UNIT CONTRACT AT THIS INTERFACE ─────────────────────────────────────────
 *   Position         : degrees  (mechanism shaft, after gear ratio)
 *   Velocity         : rotations/second  (mechanism shaft)
 *   Feed-forward     : volts
 *   Duty cycle       : -1.0 to 1.0
 *   Current          : amps
 *   Time / ramp rate : seconds
 *
 * ─── GEAR RATIO CONVENTION ───────────────────────────────────────────────────
 *   gearRatio = motor rotations per mechanism shaft rotation
 *   A 10:1 reduction  → gearRatio = 10.0
 *   A 1:2 upduction   → gearRatio = 0.5
 *   This matches the Phoenix 6 SensorToMechanismRatio convention.
 *   Applied internally per vendor; all setpoints and feedback are in
 *   mechanism-shaft units at the public API layer.
 *
 * ─── SOFT LIMITS ─────────────────────────────────────────────────────────────
 *   softLimitForwardDeg / softLimitReverseDeg use Double.NaN to mean "disabled".
 *   Converted to rotations internally before being written to motor controllers.
 *
 * ─── FOLLOWER RULES ──────────────────────────────────────────────────────────
 *   canIds[0]          — always the leader
 *   canIds[1..n]       — followers; all must be the same vendor as the leader
 *   followerInverted[] — parallel to canIds[1..n]; true = opposite direction
 *   Mixed-vendor mechanisms are not supported and will throw at factory time.
 *
 * ─── FEED-FORWARD ────────────────────────────────────────────────────────────
 *   Tier 1 (on-controller kS/kV/kA): in the pid[] array at indices 3/4/5.
 *   Tier 2 (Rio-computed, self-contained): tier2FF lambda.
 *   Tier 3 (Rio-computed, cross-system): tier3FF lambda.
 *   See ffProvider.java for full documentation and examples.
 *   Physics-based factories: mechanismUnit.FF.rotatingArm(), multiStageElevator(),
 *     pivotingElevator(), springTurret().
 *   Convenience factory for no-FF case: mechanismConfig.noFF().
 *
 * ─── MOTION PROFILE ──────────────────────────────────────────────────────────
 *   Set motionCruiseVelocityRps > 0 to activate motion profiling.
 *   CTRE: MotionMagic (trapezoidal; set motionJerkRpss3 > 0 for S-curve).
 *   REV:  MAXMotion.
 *   Nova: Software setpoint step-limiter using motionAccelerationRpss.
 *   Leave at 0 to use raw closed-loop without profile (velocity control
 *   typically does not need a cruise velocity, only acceleration).
 */
public final class mechanismConfig {

    // ── Identity ──────────────────────────────────────────────────────────────

    /** Human-readable mechanism name. Used as DataLog path prefix and
     *  SmartDashboard namespace. Example: "Shooter", "Elevator", "Wrist". */
    public final String name;

    // ── Motor roster ──────────────────────────────────────────────────────────

    /** CAN IDs. Index 0 = leader; indices 1..n = followers. */
    public final int[] canIds;

    /** Vendor for each motor, parallel to canIds. All must be identical. */
    public final motorConstants.Vendor[] vendors;

    /** Whether each follower is inverted relative to its designated leader.
     *  Length = canIds.length - 1. Parallel to followerModes and followerLeaderIndices. */
    public final boolean[] followerInverted;

    /**
     * Follow mode per follower. Length = canIds.length - 1.
     * Index 0 = first follower (canIds[1]), etc.
     * Use withFollowMode() to set all followers to the same mode,
     * or withFollowerConfig() for per-follower topology.
     */
    public final motorConstants.FollowMode[] followerModes;

    /**
     * Leader index per follower (0-based index into canIds[]).
     * Length = canIds.length - 1. Parallel to followerModes.
     *
     * 0 = follows the main leader (canIds[0]) — the default for all followers.
     * N = follows canIds[N], enabling follower-of-follower topologies.
     *
     * Example 4-motor topology:
     *   canIds = {10, 11, 12, 13}
     *   followerLeaderIndices = {0, 0, 2}
     *   → 11 follows 10, 12 follows 10, 13 follows canIds[2]=12
     */
    public final int[] followerLeaderIndices;

    // ── Mechanical ────────────────────────────────────────────────────────────

    /** Motor rotations per mechanism shaft rotation. >1 = speed reduction.
     *  Applied via SensorToMechanismRatio (CTRE), conversion factors (REV),
     *  or manual scaling (Nova). Default: 1.0 (direct drive). */
    public final double gearRatio;

    /** Invert leader motor output direction. Default: false. */
    public final boolean inverted;

    // ── PID gains ─────────────────────────────────────────────────────────────

    /**
     * On-controller PID/FF gains: { kP, kI, kD, kS, kV, kA }
     *   kP : proportional gain
     *   kI : integral gain
     *   kD : derivative gain
     *   kS : static friction voltage (volts) — kick to overcome stiction
     *   kV : velocity feed-forward (volts per RPS) — back-EMF compensation
     *   kA : acceleration feed-forward (volts per RPS²) — inertia compensation
     * All run on the motor controller. kA availability varies by vendor.
     * Live-tunable from SmartDashboard during development.
     */
    public final double[] pid;

    // ── Soft limits ───────────────────────────────────────────────────────────

    /** Forward soft limit (degrees, mechanism shaft). Double.NaN = disabled.
     *  Motor output is cut to neutral if position exceeds this and a forward
     *  command is applied. */
    public final double softLimitForwardDeg;

    /** Reverse soft limit (degrees, mechanism shaft). Double.NaN = disabled. */
    public final double softLimitReverseDeg;

    // ── Current limits ────────────────────────────────────────────────────────

    /** Maximum supply (input) current in amps. Protects breakers.
     *  CTRE: SupplyCurrentLimit. REV: SmartCurrentLimit. Nova: current limit. */
    public final double supplyCurrentLimitAmps;

    /** Maximum stator (output) current in amps. Controls torque / prevents slip.
     *  CTRE only: StatorCurrentLimit. Ignored by REV and Nova implementations. */
    public final double statorCurrentLimitAmps;

    // ── Ramp rates ────────────────────────────────────────────────────────────

    /** Seconds from zero to full output for open-loop (duty cycle) control.
     *  0 = no ramp. Applied as hardware ramp where supported (CTRE, REV).
     *  Nova falls back to software ramp in the abstract base. */
    public final double openLoopRampSecs;

    /** Seconds from zero to full output for closed-loop control.
     *  0 = no ramp. Applied as hardware ramp where supported. */
    public final double closedLoopRampSecs;

    // ── Motion profile (soft start for closed-loop) ───────────────────────────

    /** Maximum cruise velocity for motion profile (mechanism shaft RPS).
     *  0 = motion profile disabled; raw closed-loop used instead.
     *  Set > 0 to activate MotionMagic (CTRE), MAXMotion (REV), or
     *  software step-limiter (Nova). */
    public final double motionCruiseVelocityRps;

    /** Maximum acceleration for motion profile (mechanism shaft RPS²). */
    public final double motionAccelerationRpss;

    /** Jerk limit for CTRE MotionMagic S-curve profile (mechanism shaft RPS³).
     *  0 = trapezoidal profile (no jerk limiting).
     *  Ignored by REV and Nova implementations. */
    public final double motionJerkRpss3;

    // ── Feed-forward lambdas ──────────────────────────────────────────────────

    /**
     * Tier-2 feed-forward: Rio-computed, self-contained.
     * Receives (positionDeg, velocityRps, accelRpss) — all from this mechanism.
     * Must not capture external subsystem state. Default: noFF() — returns 0.
     * See ffProvider.java for examples and mechanismConfig.armFF() / elevatorFF()
     * for convenience factories.
     */
    public final ffProvider tier2FF;

    /**
     * Tier-3 feed-forward: Rio-computed, may capture external subsystem references.
     * Same signature as tier-2. External references should be captured via lambda
     * closure at construction time in RobotContainer.
     * Default: noFF() — returns 0.
     */
    public final ffProvider tier3FF;

    // ── Setpoint tracking ─────────────────────────────────────────────────────

    /**
     * Tolerance used by mechanismUnit.isAtSetpoint().
     * For position mechanisms: degrees (mechanism shaft).
     * For velocity mechanisms: rotations/second (mechanism shaft).
     * Default: 0.0 (disabled — isAtSetpoint() always returns false until set).
     */
    public final double setpointDeadband;

    // ── Neutral behavior ──────────────────────────────────────────────────────

    /** true = brake mode (resist movement when not commanded).
     *  false = coast mode. Default: true. */
    public final boolean brakeOnNeutral;

    // ── Encoder sync (ENCODER_SYNC follow mode only) ──────────────────────────

    /** Error threshold before sync correction activates (rotations, mechanism shaft).
     *  Default: motorConstants.ENCODER_SYNC_DEADBAND_ROT */
    public final double encoderSyncDeadbandRot;

    /** Proportional gain for sync correction (duty cycle per rotation of error).
     *  Output clamped to [-1.0, 1.0]. Default: motorConstants.ENCODER_SYNC_KP_DEFAULT */
    public final double encoderSyncKp;

    // ─────────────────────────────────────────────────────────────────────────
    // Convenience FF factory
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * FF provider that always returns zero.
     * Use for flat mechanisms with no gravity component, or when
     * tier-2/3 feed-forward is not needed.
     *
     * For physics-based gravity and spring compensation, use the factories in
     * {@link mechanismUnit.FF}: rotatingArm(), multiStageElevator(),
     * pivotingElevator(), springTurret().
     */
    public static ffProvider noFF() {
        return (pos, vel, accel) -> 0.0;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Private constructor — use Builder
    // ─────────────────────────────────────────────────────────────────────────

    private mechanismConfig(Builder b) {
        this.name                    = b.name;
        this.canIds                  = b.canIds;
        this.vendors                 = b.vendors;
        this.followerInverted        = b.followerInverted;
        this.followerModes           = b.followerModes;
        this.followerLeaderIndices   = b.followerLeaderIndices;
        this.gearRatio               = b.gearRatio;
        this.inverted                = b.inverted;
        this.pid                     = b.pid;
        this.softLimitForwardDeg     = b.softLimitForwardDeg;
        this.softLimitReverseDeg     = b.softLimitReverseDeg;
        this.supplyCurrentLimitAmps  = b.supplyCurrentLimitAmps;
        this.statorCurrentLimitAmps  = b.statorCurrentLimitAmps;
        this.openLoopRampSecs        = b.openLoopRampSecs;
        this.closedLoopRampSecs      = b.closedLoopRampSecs;
        this.motionCruiseVelocityRps = b.motionCruiseVelocityRps;
        this.motionAccelerationRpss  = b.motionAccelerationRpss;
        this.motionJerkRpss3         = b.motionJerkRpss3;
        this.tier2FF                 = b.tier2FF;
        this.tier3FF                 = b.tier3FF;
        this.setpointDeadband        = b.setpointDeadband;
        this.brakeOnNeutral          = b.brakeOnNeutral;
        this.encoderSyncDeadbandRot  = b.encoderSyncDeadbandRot;
        this.encoderSyncKp           = b.encoderSyncKp;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Builder
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Fluent builder for mechanismConfig.
     *
     * Required: name, canIds, vendors.
     * All other fields have safe defaults appropriate for a single-motor mechanism
     * with no profile, no soft limits, and no feed-forward.
     *
     * Example — single CTRE leader, arm with gravity compensation:
     * <pre>{@code
     * mechanismConfig cfg = new mechanismConfig.Builder(
     *         "Shoulder",
     *         new int[]{ 15 },
     *         new motorConstants.Vendor[]{ motorConstants.Vendor.CTRE_TALONFX })
     *     .withGearRatio(100.0)
     *     .withPID(new double[]{ 0.5, 0, 0.01, 0.15, 0.12, 0 })
     *     .withMotionProfile(80, 200, 0)
     *     .withSoftLimits(-10.0, 220.0)
     *     .withCurrentLimits(40, 80)
     *     .withTier2FF(mechanismUnit.FF.rotatingArm(
     *         motorModels.NEO, 100.0, 3.5, 12.0, new double[0], new double[0], null))
     *     .build();
     * }</pre>
     *
     * Example — two REV SparkMax followers (mechanical), vertical elevator:
     * <pre>{@code
     * mechanismConfig cfg = new mechanismConfig.Builder(
     *         "Elevator",
     *         new int[]{ 20, 21 },
     *         new motorConstants.Vendor[]{ REV_SPARKMAX, REV_SPARKMAX })
     *     .withFollowMode(motorConstants.FollowMode.MECHANICAL, new boolean[]{ false })
     *     .withGearRatio(20.0)
     *     .withPID(new double[]{ 0.1, 0, 0, 0.1, 0, 0 })
     *     .withMotionProfile(40, 80, 0)
     *     .withTier3FF(mechanismUnit.FF.multiStageElevator(
     *         motorModels.NEO, 20.0, 0.75, 8.0, new double[0], 0.0, 0.0,
     *         new double[0], null, () -> 90.0))
     *     .build();
     * }</pre>
     */
    public static class Builder {

        // Required
        private final String name;
        private final int[] canIds;
        private final motorConstants.Vendor[] vendors;

        // Optional — defaults
        private boolean[] followerInverted             = new boolean[0];
        private motorConstants.FollowMode[] followerModes        = new motorConstants.FollowMode[0];
        private int[] followerLeaderIndices            = new int[0];
        private double gearRatio                  = 1.0;
        private boolean inverted                  = false;
        private double[] pid                      = { 0, 0, 0, 0, 0, 0 };
        private double softLimitForwardDeg        = Double.NaN;
        private double softLimitReverseDeg        = Double.NaN;
        private double supplyCurrentLimitAmps     = 40.0;
        private double statorCurrentLimitAmps     = 60.0;
        private double openLoopRampSecs           = 0.0;
        private double closedLoopRampSecs         = 0.0;
        private double motionCruiseVelocityRps    = 0.0;
        private double motionAccelerationRpss     = 0.0;
        private double motionJerkRpss3            = 0.0;
        private ffProvider tier2FF                = noFF();
        private ffProvider tier3FF                = noFF();
        private double setpointDeadband           = 0.0;
        private boolean brakeOnNeutral            = true;
        private double encoderSyncDeadbandRot     = motorConstants.ENCODER_SYNC_DEADBAND_ROT;
        private double encoderSyncKp              = motorConstants.ENCODER_SYNC_KP_DEFAULT;

        /**
         * @param name    mechanism name — used for logging and dashboard
         * @param canIds  CAN IDs: index 0 = leader, 1..n = followers
         * @param vendors vendor per motor, parallel to canIds; all must match
         */
        public Builder(String name, int[] canIds, motorConstants.Vendor[] vendors) {
            if (name == null || name.isEmpty())
                throw new IllegalArgumentException("mechanismConfig: name must not be empty");
            if (canIds == null || canIds.length == 0)
                throw new IllegalArgumentException("mechanismConfig: canIds must not be empty");
            if (vendors == null || vendors.length != canIds.length)
                throw new IllegalArgumentException(
                    "mechanismConfig: vendors[] must be same length as canIds[]");
            this.name    = name;
            this.canIds  = canIds;
            this.vendors = vendors;
        }

        /**
         * Convenience method — sets all followers to the same follow mode with
         * all following canIds[0] (the main leader). Use for simple 1-leader topologies.
         *
         * For mixed-mode or follower-of-follower topologies, use withFollowerConfig().
         *
         * @param mode             follow mode applied to every follower
         * @param followerInverted inversion per follower, parallel to canIds[1..n]
         */
        public Builder withFollowMode(motorConstants.FollowMode mode, boolean[] followerInverted) {
            int count = canIds.length - 1;
            this.followerInverted       = followerInverted;
            this.followerModes          = new motorConstants.FollowMode[count];
            this.followerLeaderIndices  = new int[count];
            for (int i = 0; i < count; i++) {
                this.followerModes[i]         = mode;
                this.followerLeaderIndices[i] = 0;  // all follow canIds[0]
            }
            return this;
        }

        /**
         * Full per-follower topology configuration.
         * Use when followers have different modes or a follower must follow another
         * follower rather than the main leader.
         *
         * All three arrays must be the same length = canIds.length - 1.
         *
         * @param modes              follow mode per follower
         * @param followerInverted   inversion per follower
         * @param leaderIndices      0-based index into canIds[] that each follower tracks.
         *                           0 = main leader. N = canIds[N] (follower-of-follower).
         */
        public Builder withFollowerConfig(motorConstants.FollowMode[] modes,
                                          boolean[] followerInverted,
                                          int[] leaderIndices) {
            int count = canIds.length - 1;
            if (modes.length != count || followerInverted.length != count
                    || leaderIndices.length != count)
                throw new IllegalArgumentException(
                    "mechanismConfig '" + name + "': withFollowerConfig arrays must each have "
                    + "length canIds.length - 1 (" + count + ").");
            for (int i = 0; i < count; i++) {
                if (leaderIndices[i] < 0 || leaderIndices[i] >= canIds.length)
                    throw new IllegalArgumentException(
                        "mechanismConfig '" + name + "': followerLeaderIndices[" + i + "] = "
                        + leaderIndices[i] + " is out of range for canIds length "
                        + canIds.length + ".");
                if (leaderIndices[i] == i + 1)
                    throw new IllegalArgumentException(
                        "mechanismConfig '" + name + "': followerLeaderIndices[" + i
                        + "] points to itself (canIds[" + (i + 1) + "]).");
            }
            this.followerModes         = modes;
            this.followerInverted      = followerInverted;
            this.followerLeaderIndices = leaderIndices;
            return this;
        }

        /** Motor rotations per mechanism shaft rotation. >1 = reduction. */
        public Builder withGearRatio(double ratio) {
            this.gearRatio = ratio;
            return this;
        }

        /** Invert leader output direction. */
        public Builder withInverted(boolean inverted) {
            this.inverted = inverted;
            return this;
        }

        /**
         * Set on-controller PID/FF gains.
         * @param pid { kP, kI, kD, kS, kV, kA }
         */
        public Builder withPID(double[] pid) {
            if (pid == null || pid.length != 6)
                throw new IllegalArgumentException(
                    "mechanismConfig: pid[] must have exactly 6 elements {kP, kI, kD, kS, kV, kA}");
            this.pid = pid;
            return this;
        }

        /**
         * Set position soft limits (degrees, mechanism shaft).
         * Pass Double.NaN for either value to leave it disabled.
         * @param reverseDeg lower bound (most negative position)
         * @param forwardDeg upper bound (most positive position)
         */
        public Builder withSoftLimits(double reverseDeg, double forwardDeg) {
            this.softLimitReverseDeg = reverseDeg;
            this.softLimitForwardDeg = forwardDeg;
            return this;
        }

        /**
         * Set current limits.
         * @param supplyAmps supply (input) current limit — protects breakers
         * @param statorAmps stator (output) current limit — CTRE only, controls torque
         */
        public Builder withCurrentLimits(double supplyAmps, double statorAmps) {
            this.supplyCurrentLimitAmps = supplyAmps;
            this.statorCurrentLimitAmps = statorAmps;
            return this;
        }

        /**
         * Set ramp rates.
         * @param openLoopSecs   seconds 0→full for duty cycle (0 = no ramp)
         * @param closedLoopSecs seconds 0→full for closed-loop (0 = no ramp)
         */
        public Builder withRampRates(double openLoopSecs, double closedLoopSecs) {
            this.openLoopRampSecs   = openLoopSecs;
            this.closedLoopRampSecs = closedLoopSecs;
            return this;
        }

        /**
         * Enable motion profiling (MotionMagic / MAXMotion / software step-limiter).
         * @param cruiseVelocityRps max cruise velocity (mechanism shaft RPS)
         * @param accelerationRpss  max acceleration (mechanism shaft RPS²)
         * @param jerkRpss3         jerk limit for CTRE S-curve (0 = trapezoidal)
         */
        public Builder withMotionProfile(double cruiseVelocityRps, double accelerationRpss,
                                         double jerkRpss3) {
            this.motionCruiseVelocityRps = cruiseVelocityRps;
            this.motionAccelerationRpss  = accelerationRpss;
            this.motionJerkRpss3         = jerkRpss3;
            return this;
        }

        /**
         * Set tier-2 feed-forward lambda (self-contained, no external subsystem refs).
         * Use mechanismConfig.armFF(), elevatorFF(), or noFF() for common cases.
         */
        public Builder withTier2FF(ffProvider provider) {
            this.tier2FF = provider;
            return this;
        }

        /**
         * Set tier-3 feed-forward lambda (may close over external subsystem references).
         * Build this lambda in RobotContainer where all subsystem references exist.
         */
        public Builder withTier3FF(ffProvider provider) {
            this.tier3FF = provider;
            return this;
        }

        /**
         * Set the setpoint tolerance for isAtSetpoint().
         * Position mechanisms: degrees (mechanism shaft).
         * Velocity mechanisms: rotations/second (mechanism shaft).
         * Default: 0.0 (isAtSetpoint() always returns false until this is set).
         */
        public Builder withSetpointDeadband(double deadband) {
            this.setpointDeadband = deadband;
            return this;
        }

        /** Set neutral mode. true = brake, false = coast. Default: true. */
        public Builder withBrakeOnNeutral(boolean brake) {
            this.brakeOnNeutral = brake;
            return this;
        }

        /**
         * Tune ENCODER_SYNC follow parameters.
         * @param deadbandRot error threshold before correction fires (rotations)
         * @param kp          proportional gain (duty cycle per rotation of error)
         */
        public Builder withEncoderSync(double deadbandRot, double kp) {
            this.encoderSyncDeadbandRot = deadbandRot;
            this.encoderSyncKp          = kp;
            return this;
        }

        /** Build and return the immutable mechanismConfig. */
        public mechanismConfig build() {
            return new mechanismConfig(this);
        }
    }
}
