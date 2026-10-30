package frc.lib.hardware.motor;

/**
 * MotorIOConfig.java
 * PATH: src/main/java/frc/lib/hardware/motor/MotorIOConfig.java
 *
 * Immutable hardware settings for ONE motor controller. Build with
 * MotorIOConfig.builder(canId, vendor), then pass to MotorIO.create().
 *
 * Everything a vendor implementation needs is in this object — no MotorIO
 * implementation reads robot constants. That is what lets a mechanism be copied
 * into another robot project without dragging this robot's constants along.
 *
 * ─── UNIT CONTRACT ───────────────────────────────────────────────────────────
 *   Position      : rotations (mechanism shaft, after gear ratio)
 *   Velocity      : rotations/second (mechanism shaft)
 *   Soft limits   : rotations (mechanism shaft); Double.NaN = disabled
 *   Current       : amps
 *   Ramp / time   : seconds
 *
 * ─── GEAR RATIO CONVENTION ───────────────────────────────────────────────────
 *   gearRatio = motor rotations per mechanism shaft rotation (10:1 reduction → 10.0).
 *   Matches Phoenix 6 SensorToMechanismRatio. Applied per vendor so every
 *   MotorIO reads and commands in mechanism-shaft units.
 *
 * ─── VENDOR SUPPORT NOTES ────────────────────────────────────────────────────
 *   statorCurrentLimitAmps : CTRE only (REV/Nova use the supply limit)
 *   motionJerkRpss3        : CTRE MotionMagic S-curve only
 *   kS / kA                : REV and Nova do not run these on-controller
 *                            (Nova maps kS to its FF term)
 *   continuousWrap         : hardware on CTRE/REV, software on Nova
 *   signal rates / optimizeCanUtilization : CTRE only
 *   novaMotorType          : Nova only — checked by ConfigVerifier
 */
public final class MotorIOConfig {

    /** Motor attached to a ThriftyBot Nova. Mirrors ThriftyNova.MotorType. */
    public enum NovaMotorType { NEO, MINION, BRUSHED }

    // ── Identity ──────────────────────────────────────────────────────────────
    public final int         canId;
    public final MotorVendor vendor;

    // ── Direction / neutral ───────────────────────────────────────────────────
    public final boolean inverted;
    public final boolean brakeOnNeutral;

    // ── Mechanical ────────────────────────────────────────────────────────────
    public final double gearRatio;

    // ── On-controller gains (slot 0) ──────────────────────────────────────────
    public final double kP, kI, kD, kS, kV, kA;

    // ── Soft limits (rotations, mechanism shaft) ─────────────────────────────
    public final double softLimitForwardRot;
    public final double softLimitReverseRot;

    // ── Current limits ────────────────────────────────────────────────────────
    public final double supplyCurrentLimitAmps;
    public final double statorCurrentLimitAmps;

    // ── Ramps ─────────────────────────────────────────────────────────────────
    public final double openLoopRampSecs;
    public final double closedLoopRampSecs;

    // ── Motion profile (0 cruise = disabled) ─────────────────────────────────
    public final double motionCruiseVelocityRps;
    public final double motionAccelerationRpss;
    public final double motionJerkRpss3;

    // ── Position wrap (swerve steer, turrets with slip rings) ────────────────
    public final boolean continuousWrap;

    // ── CAN signals ───────────────────────────────────────────────────────────
    public final double  signalUpdateHz;
    public final double  tempSignalUpdateHz;
    public final boolean optimizeCanUtilization;

    // ── Nova readback check ───────────────────────────────────────────────────
    /** null = skip the motor-type check during ConfigVerifier readback. */
    public final NovaMotorType novaMotorType;

    private MotorIOConfig(Builder b) {
        this.canId                   = b.canId;
        this.vendor                  = b.vendor;
        this.inverted                = b.inverted;
        this.brakeOnNeutral          = b.brakeOnNeutral;
        this.gearRatio               = b.gearRatio;
        this.kP = b.kP; this.kI = b.kI; this.kD = b.kD;
        this.kS = b.kS; this.kV = b.kV; this.kA = b.kA;
        this.softLimitForwardRot     = b.softLimitForwardRot;
        this.softLimitReverseRot     = b.softLimitReverseRot;
        this.supplyCurrentLimitAmps  = b.supplyCurrentLimitAmps;
        this.statorCurrentLimitAmps  = b.statorCurrentLimitAmps;
        this.openLoopRampSecs        = b.openLoopRampSecs;
        this.closedLoopRampSecs      = b.closedLoopRampSecs;
        this.motionCruiseVelocityRps = b.motionCruiseVelocityRps;
        this.motionAccelerationRpss  = b.motionAccelerationRpss;
        this.motionJerkRpss3         = b.motionJerkRpss3;
        this.continuousWrap          = b.continuousWrap;
        this.signalUpdateHz          = b.signalUpdateHz;
        this.tempSignalUpdateHz      = b.tempSignalUpdateHz;
        this.optimizeCanUtilization  = b.optimizeCanUtilization;
        this.novaMotorType           = b.novaMotorType;
    }

    /** True when a motion profile (MotionMagic / MAXMotion / software) is requested. */
    public boolean hasMotionProfile() {
        return motionCruiseVelocityRps > 0.0;
    }

    /** Start a new config for one controller. */
    public static Builder builder(int canId, MotorVendor vendor) {
        return new Builder(canId, vendor);
    }

    /** Copy every field into a new Builder — used to derive follower configs. */
    public Builder toBuilder() {
        Builder b = new Builder(canId, vendor);
        b.inverted                = inverted;
        b.brakeOnNeutral          = brakeOnNeutral;
        b.gearRatio               = gearRatio;
        b.kP = kP; b.kI = kI; b.kD = kD; b.kS = kS; b.kV = kV; b.kA = kA;
        b.softLimitForwardRot     = softLimitForwardRot;
        b.softLimitReverseRot     = softLimitReverseRot;
        b.supplyCurrentLimitAmps  = supplyCurrentLimitAmps;
        b.statorCurrentLimitAmps  = statorCurrentLimitAmps;
        b.openLoopRampSecs        = openLoopRampSecs;
        b.closedLoopRampSecs      = closedLoopRampSecs;
        b.motionCruiseVelocityRps = motionCruiseVelocityRps;
        b.motionAccelerationRpss  = motionAccelerationRpss;
        b.motionJerkRpss3         = motionJerkRpss3;
        b.continuousWrap          = continuousWrap;
        b.signalUpdateHz          = signalUpdateHz;
        b.tempSignalUpdateHz      = tempSignalUpdateHz;
        b.optimizeCanUtilization  = optimizeCanUtilization;
        b.novaMotorType           = novaMotorType;
        return b;
    }

    @Override
    public String toString() {
        return vendor.displayName + " CAN " + canId;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Builder
    // ─────────────────────────────────────────────────────────────────────────

    public static final class Builder {
        private int         canId;
        private MotorVendor vendor;

        private boolean inverted                = false;
        private boolean brakeOnNeutral          = true;
        private double  gearRatio               = 1.0;
        private double  kP, kI, kD, kS, kV, kA;
        private double  softLimitForwardRot     = Double.NaN;
        private double  softLimitReverseRot     = Double.NaN;
        private double  supplyCurrentLimitAmps  = 40.0;
        private double  statorCurrentLimitAmps  = 60.0;
        private double  openLoopRampSecs        = 0.0;
        private double  closedLoopRampSecs      = 0.0;
        private double  motionCruiseVelocityRps = 0.0;
        private double  motionAccelerationRpss  = 0.0;
        private double  motionJerkRpss3         = 0.0;
        private boolean continuousWrap          = false;
        private double  signalUpdateHz          = 250.0;
        private double  tempSignalUpdateHz      = 4.0;
        private boolean optimizeCanUtilization  = false;
        private NovaMotorType novaMotorType     = null;

        private Builder(int canId, MotorVendor vendor) {
            if (vendor == null)
                throw new IllegalArgumentException("MotorIOConfig: vendor must not be null");
            this.canId  = canId;
            this.vendor = vendor;
        }

        /** Replace the CAN ID — used when deriving a follower from a leader config. */
        public Builder withCanId(int canId) {
            this.canId = canId;
            return this;
        }

        public Builder withInverted(boolean inverted) {
            this.inverted = inverted;
            return this;
        }

        public Builder withBrakeOnNeutral(boolean brake) {
            this.brakeOnNeutral = brake;
            return this;
        }

        /** Motor rotations per mechanism shaft rotation. */
        public Builder withGearRatio(double ratio) {
            if (ratio <= 0.0)
                throw new IllegalArgumentException("MotorIOConfig: gearRatio must be > 0, got " + ratio);
            this.gearRatio = ratio;
            return this;
        }

        public Builder withGains(double kP, double kI, double kD, double kS, double kV, double kA) {
            this.kP = kP; this.kI = kI; this.kD = kD;
            this.kS = kS; this.kV = kV; this.kA = kA;
            return this;
        }

        /**
         * @param gains { kP, kI, kD, kS, kV } or { kP, kI, kD, kS, kV, kA }
         *              (kA defaults to 0 when only five values are given)
         */
        public Builder withGains(double[] gains) {
            if (gains == null || (gains.length != 5 && gains.length != 6))
                throw new IllegalArgumentException(
                    "MotorIOConfig: gains[] must be {kP, kI, kD, kS, kV} or {kP, kI, kD, kS, kV, kA}");
            return withGains(gains[0], gains[1], gains[2], gains[3], gains[4],
                             gains.length == 6 ? gains[5] : 0.0);
        }

        /** Soft limits in mechanism shaft rotations. Double.NaN leaves a side disabled. */
        public Builder withSoftLimits(double reverseRot, double forwardRot) {
            this.softLimitReverseRot = reverseRot;
            this.softLimitForwardRot = forwardRot;
            return this;
        }

        public Builder withCurrentLimits(double supplyAmps, double statorAmps) {
            this.supplyCurrentLimitAmps = supplyAmps;
            this.statorCurrentLimitAmps = statorAmps;
            return this;
        }

        public Builder withRampRates(double openLoopSecs, double closedLoopSecs) {
            this.openLoopRampSecs   = openLoopSecs;
            this.closedLoopRampSecs = closedLoopSecs;
            return this;
        }

        public Builder withMotionProfile(double cruiseVelocityRps, double accelerationRpss,
                                         double jerkRpss3) {
            this.motionCruiseVelocityRps = cruiseVelocityRps;
            this.motionAccelerationRpss  = accelerationRpss;
            this.motionJerkRpss3         = jerkRpss3;
            return this;
        }

        /** Shortest-path position control across the ±0.5 rotation boundary. */
        public Builder withContinuousWrap(boolean wrap) {
            this.continuousWrap = wrap;
            return this;
        }

        /**
         * CAN status signal rates (CTRE only).
         * @param updateHz         velocity / position / current / voltage signals
         * @param tempUpdateHz     temperature signal
         * @param optimizeCanUsage true = silence un-registered signals (flip only
         *                         after the mechanism is validated on the robot)
         */
        public Builder withCanSignals(double updateHz, double tempUpdateHz, boolean optimizeCanUsage) {
            this.signalUpdateHz         = updateHz;
            this.tempSignalUpdateHz     = tempUpdateHz;
            this.optimizeCanUtilization = optimizeCanUsage;
            return this;
        }

        /** Expected Nova motor type, checked by ConfigVerifier. Nova only. */
        public Builder withNovaMotorType(NovaMotorType type) {
            this.novaMotorType = type;
            return this;
        }

        public MotorIOConfig build() {
            return new MotorIOConfig(this);
        }
    }
}
