package frc.robot.util.mechanisms;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.motors.ConfigVerifiable;
import frc.robot.util.motors.ConfigVerifyResult;
import frc.robot.util.motors.mechanismConfig;
import frc.robot.util.motors.mechanismUnit;
import frc.robot.util.motors.motorConstants;

/**
 * shooterMechanism.java
 * PATH: src/main/java/frc/robot/util/mechanisms/shooterMechanism.java
 *
 * Pre-packaged shooter mechanism supporting up to three sub-mechanisms:
 *   - Flywheel  (required) — velocity controlled, RPM at interface
 *   - Turret    (optional) — position controlled, degrees
 *   - Hood      (optional) — position controlled, degrees
 *
 * Each sub-mechanism is configured via a full mechanismConfig (motor vendor,
 * CAN IDs, PID, FF, soft limits, setpointDeadband, etc.). The shooterMechanism
 * instantiates mechanismUnit instances internally — callers never touch the
 * vendor layer directly.
 *
 * ─── READY STATE ─────────────────────────────────────────────────────────────
 * Two states: SEEKING and READY.
 *
 * SEEKING (initial, and after any setpoint change):
 *   isReady() = false
 *   Transitions to READY when all configured sub-mechanisms are simultaneously
 *   within their mechanismConfig.setpointDeadband.
 *
 * READY:
 *   isReady() = true
 *   Returns to SEEKING if flywheel actual RPM drops below
 *     (flywheelSetpointRpm - flywheelHoldThresholdRpm).
 *   Returns to SEEKING if any setpoint is changed.
 *
 * ─── UNITS AT THIS INTERFACE ─────────────────────────────────────────────────
 *   Flywheel setpoint / default : RPM  (converted to RPS internally)
 *   Turret setpoint / default   : degrees
 *   Hood setpoint / default     : degrees
 *   Hold threshold              : RPM  (converted to RPS internally)
 *
 * ─── CALL PATTERN ────────────────────────────────────────────────────────────
 * Instantiate in RobotContainer. Register as a subsystem — periodic() is called
 * automatically by CommandScheduler each loop.
 *
 * Optional: set Tier-3 FF on the turret config (e.g. gyroscopicTurret) in
 * RobotContainer before passing the config to the Builder, since RobotContainer
 * is where both flywheel and turret references co-exist.
 */
public class shooterMechanism extends SubsystemBase implements ConfigVerifiable {

    // ── Sub-mechanisms ────────────────────────────────────────────────────────
    private final mechanismUnit flywheel;
    private final mechanismUnit turret;   // null if not configured
    private final mechanismUnit hood;     // null if not configured

    // ── Default setpoints ─────────────────────────────────────────────────────
    private final double defaultFlywheelRps;
    private final double defaultTurretDeg;  // 0 if turret not configured
    private final double defaultHoodDeg;    // 0 if hood not configured

    // ── Active setpoints ──────────────────────────────────────────────────────
    private double flywheelSetpointRps;
    private double turretSetpointDeg;
    private double hoodSetpointDeg;

    // ── Hold threshold ────────────────────────────────────────────────────────
    private final double holdThresholdRps;

    // ── Ready gate ────────────────────────────────────────────────────────────
    // Optional external condition that must be true before isReady() can return true.
    // Use case: gate on robot heading being within headingBoundsDeg so the shooter
    // never reports READY while the turret would have to fire past a soft stop.
    //   .withReadyGate(() -> aimController.getLastResult().isHeadingInBounds())
    private final BooleanSupplier readyGate;

    // ── Ready state ───────────────────────────────────────────────────────────
    private enum ReadyState { SEEKING, READY }
    private ReadyState state = ReadyState.SEEKING;

    // ─────────────────────────────────────────────────────────────────────────
    // Constructor (private — use Builder)
    // ─────────────────────────────────────────────────────────────────────────

    private shooterMechanism(Builder b) {
        flywheel           = mechanismUnit.create(b.flywheelConfig);
        turret             = b.turretConfig != null ? mechanismUnit.create(b.turretConfig) : null;
        hood               = b.hoodConfig   != null ? mechanismUnit.create(b.hoodConfig)   : null;

        defaultFlywheelRps = b.defaultFlywheelRpm / 60.0;
        defaultTurretDeg   = b.defaultTurretDeg;
        defaultHoodDeg     = b.defaultHoodDeg;
        holdThresholdRps   = b.holdThresholdRpm / 60.0;
        readyGate          = b.readyGate;

        flywheelSetpointRps = defaultFlywheelRps;
        turretSetpointDeg   = defaultTurretDeg;
        hoodSetpointDeg     = defaultHoodDeg;

        printConfigStatus(b);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // SubsystemBase — periodic
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        // Each set*() call automatically runs encoder sync, injects FF, and logs telemetry
        flywheel.setVelocity(flywheelSetpointRps);
        if (turret != null) turret.setPosition(turretSetpointDeg);
        if (hood   != null) hood.setPosition(hoodSetpointDeg);

        // Update ready state machine
        updateReadyState();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Public API — setpoints
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Set flywheel target speed.
     * Resets ready state to SEEKING — isReady() will return false until all
     * sub-mechanisms reach their new setpoints.
     *
     * @param rpm flywheel shaft speed (RPM)
     */
    public void setFlywheelRpm(double rpm) {
        flywheelSetpointRps = rpm / 60.0;
        state = ReadyState.SEEKING;
    }

    /**
     * Set turret target position. No-op if turret was not configured.
     * Resets ready state to SEEKING.
     *
     * @param degrees turret angle (degrees, mechanism shaft)
     */
    public void setTurretDeg(double degrees) {
        if (turret == null) return;
        turretSetpointDeg = degrees;
        state = ReadyState.SEEKING;
    }

    /**
     * Set hood target position. No-op if hood was not configured.
     * Resets ready state to SEEKING.
     *
     * @param degrees hood angle (degrees, mechanism shaft)
     */
    public void setHoodDeg(double degrees) {
        if (hood == null) return;
        hoodSetpointDeg = degrees;
        state = ReadyState.SEEKING;
    }

    /**
     * Restore all sub-mechanisms to their construction default setpoints.
     * Resets ready state to SEEKING.
     */
    public void setDefaultSetpoints() {
        flywheelSetpointRps = defaultFlywheelRps;
        turretSetpointDeg   = defaultTurretDeg;
        hoodSetpointDeg     = defaultHoodDeg;
        state = ReadyState.SEEKING;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Public API — status
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Returns true when all configured sub-mechanisms have reached their
     * setpoints and the flywheel has not dropped below the hold threshold.
     */
    public boolean isReady() {
        return state == ReadyState.READY;
    }

    // ── Flywheel state ────────────────────────────────────────────────────────

    /** Flywheel shaft velocity (RPS). Also used by gyroscopicTurret FF supplier. */
    public double getFlywheelVelocityRps()     { return flywheel.getVelocityRps(); }

    /** Flywheel shaft position (degrees). */
    public double getFlywheelPositionDeg()     { return flywheel.getPositionDeg(); }

    /** Flywheel shaft acceleration (RPS²), estimated via finite difference. */
    public double getFlywheelAccelRpss() {
        // Derived each loop inside mechanismUnit — re-expose via velocity delta if needed.
        // For now returns 0; extend mechanismUnit to expose accelRpss if required.
        return 0.0;
    }

    // ── Turret state (0.0 / false if turret not configured) ──────────────────

    /** Turret shaft position (degrees). Returns 0 if turret not configured. */
    public double getTurretPositionDeg()       { return turret != null ? turret.getPositionDeg()  : 0.0; }

    /** Turret shaft velocity (RPS). Returns 0 if turret not configured. */
    public double getTurretVelocityRps()       { return turret != null ? turret.getVelocityRps()  : 0.0; }

    // ── Hood state (0.0 if hood not configured) ───────────────────────────────

    /** Hood shaft position (degrees). Returns 0 if hood not configured. */
    public double getHoodPositionDeg()         { return hood != null ? hood.getPositionDeg()  : 0.0; }

    /** Hood shaft velocity (RPS). Returns 0 if hood not configured. */
    public double getHoodVelocityRps()         { return hood != null ? hood.getVelocityRps()  : 0.0; }

    // ─────────────────────────────────────────────────────────────────────────
    // ConfigVerifiable implementation
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Delegates to each configured sub-mechanism's verifyConfig().
     * Returns results for flywheel, turret (if present), and hood (if present).
     */
    @Override
    public java.util.List<ConfigVerifyResult> verifyConfig() {
        java.util.List<ConfigVerifyResult> results = new java.util.ArrayList<>(flywheel.verifyConfig());
        if (turret != null) results.addAll(turret.verifyConfig());
        if (hood   != null) results.addAll(hood.verifyConfig());
        return results;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Internal — ready state machine
    // ─────────────────────────────────────────────────────────────────────────

    private void updateReadyState() {
        switch (state) {
            case SEEKING:
                if (allAtSetpoint()) state = ReadyState.READY;
                break;

            case READY:
                // Drop back to SEEKING if flywheel falls below hold threshold
                double actualRps = flywheel.getVelocityRps();
                if (actualRps < flywheelSetpointRps - holdThresholdRps) {
                    state = ReadyState.SEEKING;
                }
                break;
        }
    }

    /**
     * True when every configured sub-mechanism is within its setpoint deadband
     * AND the optional ready gate (if set) returns true.
     */
    private boolean allAtSetpoint() {
        if (!flywheel.isAtSetpoint()) return false;
        if (turret != null && !turret.isAtSetpoint()) return false;
        if (hood   != null && !hood.isAtSetpoint())   return false;
        if (readyGate != null && !readyGate.getAsBoolean()) return false;
        return true;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Internal — startup status message
    // ─────────────────────────────────────────────────────────────────────────

    private void printConfigStatus(Builder b) {
        StringBuilder sb = new StringBuilder();
        sb.append("[ShooterMechanism] '").append(b.name).append("' initialized:\n");
        sb.append("  Flywheel : ").append(describeConfig(b.flywheelConfig))
          .append(" | default ").append((int)(b.defaultFlywheelRpm)).append(" RPM\n");

        if (b.turretConfig != null) {
            sb.append("  Turret   : ").append(describeConfig(b.turretConfig))
              .append(" | default ").append(b.defaultTurretDeg).append("°\n");
        } else {
            sb.append("  Turret   : NOT configured\n");
        }

        if (b.hoodConfig != null) {
            sb.append("  Hood     : ").append(describeConfig(b.hoodConfig))
              .append(" | default ").append(b.defaultHoodDeg).append("°\n");
        } else {
            sb.append("  Hood     : NOT configured\n");
        }

        sb.append("  Hold threshold : ").append((int) b.holdThresholdRpm).append(" RPM");
        System.out.println(sb);
    }

    /** Produces a short human-readable description of a mechanismConfig's motor roster. */
    private static String describeConfig(mechanismConfig config) {
        int count = config.canIds.length;
        motorConstants.Vendor vendor = config.vendors[0];
        String vendorStr = switch (vendor) {
            case CTRE_TALONFX    -> "CTRE TalonFX";
            case REV_SPARKMAX    -> "REV SparkMax";
            case REV_SPARKFLEX   -> "REV SparkFlex";
            case THRIFTYBOT_NOVA -> "ThriftyBot Nova";
            default              -> vendor.toString();
        };
        return count + "× " + vendorStr + " | CAN " + java.util.Arrays.toString(config.canIds);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Builder
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Fluent builder for shooterMechanism.
     *
     * Required: withFlywheel(), withFlywheelHoldThresholdRpm().
     * Optional: withTurret(), withHood().
     *
     * Example — flywheel + turret + hood:
     * <pre>{@code
     * shooterMechanism shooter = new shooterMechanism.Builder("Shooter")
     *     .withFlywheel(flywheelConfig,  3500.0)
     *     .withTurret(turretConfig,         0.0)
     *     .withHood(hoodConfig,            45.0)
     *     .withFlywheelHoldThresholdRpm(  200.0)
     *     .build();
     * }</pre>
     *
     * Example — flywheel only:
     * <pre>{@code
     * shooterMechanism shooter = new shooterMechanism.Builder("Shooter")
     *     .withFlywheel(flywheelConfig, 3500.0)
     *     .withFlywheelHoldThresholdRpm(200.0)
     *     .build();
     * }</pre>
     */
    public static class Builder {

        private final String name;

        // Flywheel — required
        private mechanismConfig flywheelConfig;
        private double defaultFlywheelRpm = 0.0;

        // Turret — optional
        private mechanismConfig turretConfig   = null;
        private double defaultTurretDeg        = 0.0;

        // Hood — optional
        private mechanismConfig hoodConfig     = null;
        private double defaultHoodDeg          = 0.0;

        // Hold threshold — required
        private double holdThresholdRpm        = 0.0;
        private boolean holdThresholdSet       = false;

        // Ready gate — optional external condition
        private BooleanSupplier readyGate      = null;

        public Builder(String name) {
            this.name = name;
        }

        /**
         * Configure the flywheel sub-mechanism.
         * The mechanismConfig must have setpointDeadband set (in RPS).
         *
         * @param config        full mechanismConfig for the flywheel motor(s)
         * @param defaultRpm    default flywheel speed (RPM)
         */
        public Builder withFlywheel(mechanismConfig config, double defaultRpm) {
            this.flywheelConfig    = config;
            this.defaultFlywheelRpm = defaultRpm;
            return this;
        }

        /**
         * Configure the turret sub-mechanism (optional).
         * The mechanismConfig must have setpointDeadband set (in degrees).
         *
         * @param config        full mechanismConfig for the turret motor(s)
         * @param defaultDeg    default turret position (degrees)
         */
        public Builder withTurret(mechanismConfig config, double defaultDeg) {
            this.turretConfig    = config;
            this.defaultTurretDeg = defaultDeg;
            return this;
        }

        /**
         * Configure the hood sub-mechanism (optional).
         * The mechanismConfig must have setpointDeadband set (in degrees).
         *
         * @param config        full mechanismConfig for the hood motor(s)
         * @param defaultDeg    default hood position (degrees)
         */
        public Builder withHood(mechanismConfig config, double defaultDeg) {
            this.hoodConfig    = config;
            this.defaultHoodDeg = defaultDeg;
            return this;
        }

        /**
         * Set an optional external condition that must be true before isReady()
         * can return true. All configured sub-mechanisms must still be at their
         * setpoints — this gate is an additional AND condition on top of that.
         *
         * Typical use: prevent firing when robot heading is outside the turret's
         * allowed window.
         *
         * <pre>{@code
         * .withReadyGate(() -> aimController.getLastResult().isHeadingInBounds())
         * }</pre>
         *
         * @param gate  Returns true when the external condition is satisfied.
         *              Pass null to disable (equivalent to not calling this method).
         */
        public Builder withReadyGate(BooleanSupplier gate) {
            this.readyGate = gate;
            return this;
        }

        /**
         * Set the flywheel RPM drop allowed before isReady() returns to false.
         * Applied relative to the active flywheel setpoint each cycle.
         * Independent from the flywheel setpointDeadband in mechanismConfig.
         *
         * @param thresholdRpm  maximum RPM drop below setpoint before losing READY state
         */
        public Builder withFlywheelHoldThresholdRpm(double thresholdRpm) {
            this.holdThresholdRpm = thresholdRpm;
            this.holdThresholdSet = true;
            return this;
        }

        /** Build and return the shooterMechanism instance. */
        public shooterMechanism build() {
            if (flywheelConfig == null)
                throw new IllegalStateException(
                    "shooterMechanism.Builder '" + name + "': withFlywheel() is required.");
            if (!holdThresholdSet)
                DriverStation.reportWarning(
                    "[shooterMechanism] '" + name + "': withFlywheelHoldThresholdRpm() was not "
                    + "called. Hold threshold defaults to 0 RPM — isReady() will drop to false "
                    + "any time flywheel velocity is below its setpoint.", false);
            return new shooterMechanism(this);
        }
    }
}
