package frc.robot.util.mechanisms;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.swerveDrive.driveOdometryState;
import frc.robot.util.motors.ConfigVerifiable;
import frc.robot.util.motors.ConfigVerifyResult;
import frc.robot.util.motors.ffProvider;
import frc.robot.util.motors.mechanismConfig;
import frc.robot.util.motors.mechanismUnit;
import frc.robot.util.motors.motorConstants;
import frc.robot.util.motors.motorModels;
import frc.robot.util.units;

/**
 * elevatorMechanism.java
 * PATH: src/main/java/frc/robot/util/mechanisms/elevatorMechanism.java
 *
 * Pre-packaged elevator mechanism supporting:
 *   - Elevator  (required) — linear lift driven by a fixed-radius timing belt pulley.
 *                            Position interface in inches; converted to shaft degrees
 *                            internally via pulley radius.
 *   - Pivot     (optional) — rotational base. When present, gravity and drivetrain
 *                            inertia FF automatically account for the changing angle.
 *
 * ─── LINEAR ↔ ROTATIONAL CONVERSION ────────────────────────────────────────────
 * The elevator motor drives a timing belt via a pulley of fixed radius.
 * No spool accumulation — radius is constant.
 *
 *   linear travel (in) = shaft rotations × 2π × pulleyRadiusIn
 *   shaft degrees       = (positionIn / (2π × pulleyRadiusIn)) × 360
 *
 * pulleyRadiusIn is provided in the Builder and never touches mechanismConfig.
 *
 * ─── ANGLE RESOLUTION ───────────────────────────────────────────────────────────
 * The elevator angle relative to ground drives the gravity FF calculation.
 * Three cases, all resolved to the same Supplier<Double> at construction:
 *
 *   Fixed vertical (no pivot, no external supplier):  () -> 90.0
 *   Internal pivot sub-mechanism:                     pivot::getPositionDeg
 *   External pivot / angle source:                    withExternalAngleSupplier(...)
 *
 * ─── FEED-FORWARD ───────────────────────────────────────────────────────────────
 * All FF is wired inside build() — no external wiring in RobotContainer required.
 *
 * Elevator FF (Tier 3 on elevator motor):
 *   - Gravity: totalMass × g × sin(elevatorAngle) via pulley torque
 *   - Drivetrain linear inertia projected onto lift axis
 *   - Centripetal term
 *   - Friction offset (deadbanded)
 *
 * Pivot FF (Tier 3 on pivot motor, when present):
 *   - Gravity: totalMass × g × cgDist × cos(pivotAngle)
 *   - Drivetrain linear inertia projected onto pivot load axis
 *   - Centripetal term
 *
 * Both use the drivetrain odometry and heading suppliers provided at construction.
 *
 * ─── READY STATE ────────────────────────────────────────────────────────────────
 * SEEKING (initial, after any setpoint change):
 *   isReady() = false
 *   → READY when all configured sub-mechanisms are within setpointDeadband
 *
 * READY:
 *   isReady() = true
 *   → SEEKING if elevator position drops more than holdThresholdIn below setpoint
 *   → SEEKING if any setpoint is changed
 *
 * ─── UNITS AT THIS INTERFACE ────────────────────────────────────────────────────
 *   Elevator position / default  : inches
 *   Pivot position / default     : degrees
 *   Hold threshold               : inches
 *
 * ─── CALL PATTERN ───────────────────────────────────────────────────────────────
 * Instantiate in RobotContainer. Register as a subsystem — periodic() is called
 * automatically by CommandScheduler each loop. No additional per-loop calls needed;
 * encoder sync, FF, PID tuning, and telemetry are all automatic inside set*().
 */
public class elevatorMechanism extends SubsystemBase implements ConfigVerifiable {

    // ── Sub-mechanisms ────────────────────────────────────────────────────────
    private final mechanismUnit elevator;
    private final mechanismUnit pivot;      // null if not configured

    // ── Linear ↔ rotational conversion ───────────────────────────────────────
    // shaft degrees per inch of linear travel
    private final double degsPerIn;
    // shaft RPS per inch/sec of linear velocity
    private final double rpsPerInPs;

    // ── Default setpoints ─────────────────────────────────────────────────────
    private final double defaultElevatorIn;
    private final double defaultPivotDeg;

    // ── Active setpoints ──────────────────────────────────────────────────────
    private double elevatorSetpointDeg;
    private double pivotSetpointDeg;

    // ── Hold threshold ────────────────────────────────────────────────────────
    private final double holdThresholdDeg;   // converted from inches at construction

    // ── Ready state ───────────────────────────────────────────────────────────
    private enum ReadyState { SEEKING, READY }
    private ReadyState state = ReadyState.SEEKING;

    // ─────────────────────────────────────────────────────────────────────────
    // Constructor (private — use Builder)
    // ─────────────────────────────────────────────────────────────────────────

    private elevatorMechanism(Builder b) {
        // Conversion factor: shaft degrees per inch of linear travel
        // travel = rotations × 2π × r  →  rotations = travel / (2π × r)
        // degrees = rotations × 360
        double circumferenceIn  = 2.0 * Math.PI * b.pulleyRadiusIn;
        degsPerIn               = 360.0 / circumferenceIn;
        rpsPerInPs              = 1.0  / circumferenceIn;

        defaultElevatorIn  = b.defaultElevatorIn;
        defaultPivotDeg    = b.defaultPivotDeg;
        holdThresholdDeg   = b.holdThresholdIn * degsPerIn;

        // Resolve angle supplier before building FF (pivot unit doesn't exist yet
        // for the internal-pivot case, so we use a holder filled after construction)
        final mechanismUnit[] pivotHolder = { null };

        Supplier<Double> angleSupplier;
        if (b.externalAngleSupplier != null) {
            angleSupplier = b.externalAngleSupplier;
        } else if (b.pivotConfig != null) {
            // Internal pivot — forward reference resolved after pivot is created below
            angleSupplier = () -> pivotHolder[0] != null
                ? pivotHolder[0].getPositionDeg() : 90.0;
        } else {
            angleSupplier = () -> 90.0;
        }

        // Wire elevator Tier-3 FF — all physics computed inside the factory
        mechanismConfig elevatorConfigWithFF = applyElevatorFF(
            b, angleSupplier);

        elevator = mechanismUnit.create(elevatorConfigWithFF);

        // Wire pivot Tier-3 FF and create pivot unit
        if (b.pivotConfig != null) {
            mechanismConfig pivotConfigWithFF = applyPivotFF(b, angleSupplier);
            pivot = mechanismUnit.create(pivotConfigWithFF);
            pivotHolder[0] = pivot;
        } else {
            pivot = null;
        }

        elevatorSetpointDeg = defaultElevatorIn * degsPerIn;
        pivotSetpointDeg    = defaultPivotDeg;

        printConfigStatus(b);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // SubsystemBase — periodic
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        // Each set*() call automatically runs encoder sync, injects FF, and logs telemetry
        elevator.setPosition(elevatorSetpointDeg);
        if (pivot != null) pivot.setPosition(pivotSetpointDeg);

        updateReadyState();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Public API — setpoints
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Set elevator target position.
     * Resets ready state to SEEKING.
     *
     * @param positionIn target position (inches, mechanism travel from home)
     */
    public void setElevatorPositionIn(double positionIn) {
        elevatorSetpointDeg = positionIn * degsPerIn;
        state = ReadyState.SEEKING;
    }

    /**
     * Set pivot target angle. No-op if pivot was not configured.
     * Resets ready state to SEEKING.
     *
     * @param degrees pivot angle (degrees, mechanism shaft)
     */
    public void setPivotDeg(double degrees) {
        if (pivot == null) return;
        pivotSetpointDeg = degrees;
        state = ReadyState.SEEKING;
    }

    /**
     * Restore all sub-mechanisms to their construction default setpoints.
     * Resets ready state to SEEKING.
     */
    public void setDefaultSetpoints() {
        elevatorSetpointDeg = defaultElevatorIn * degsPerIn;
        pivotSetpointDeg    = defaultPivotDeg;
        state = ReadyState.SEEKING;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Public API — status
    // ─────────────────────────────────────────────────────────────────────────

    /** True when all configured sub-mechanisms are at setpoint and hold is maintained. */
    public boolean isReady() {
        return state == ReadyState.READY;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Public API — state getters
    // ─────────────────────────────────────────────────────────────────────────

    /** Actual elevator position (inches). */
    public double getElevatorPositionIn() {
        return elevator.getPositionDeg() / degsPerIn;
    }

    /** Actual elevator velocity (inches/second). */
    public double getElevatorVelocityInPs() {
        return elevator.getVelocityRps() / rpsPerInPs;
    }

    /** Actual pivot position (degrees). Returns 0 if pivot not configured. */
    public double getPivotDeg() {
        return pivot != null ? pivot.getPositionDeg() : 0.0;
    }

    /** Actual pivot velocity (RPS). Returns 0 if pivot not configured. */
    public double getPivotVelocityRps() {
        return pivot != null ? pivot.getVelocityRps() : 0.0;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // ConfigVerifiable implementation
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Delegates to each configured sub-mechanism's verifyConfig().
     * Returns results for elevator motor(s) and pivot motor(s) if present.
     */
    @Override
    public java.util.List<ConfigVerifyResult> verifyConfig() {
        java.util.List<ConfigVerifyResult> results = new java.util.ArrayList<>(elevator.verifyConfig());
        if (pivot != null) results.addAll(pivot.verifyConfig());
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
                // Drop back if elevator falls below hold threshold
                if (elevator.getPositionDeg() < elevatorSetpointDeg - holdThresholdDeg) {
                    state = ReadyState.SEEKING;
                }
                break;
        }
    }

    private boolean allAtSetpoint() {
        if (!elevator.isAtSetpoint()) return false;
        if (pivot != null && !pivot.isAtSetpoint()) return false;
        return true;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Internal — FF wiring
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Returns a copy of the elevator mechanismConfig with Tier-3 FF wired.
     * Any existing tier3FF in the provided config is summed with the physics FF
     * via lambda composition so user-provided FF is not silently overwritten.
     */
    private static mechanismConfig applyElevatorFF(Builder b,
                                                    Supplier<Double> angleSupplier) {
        motorModels.MotorModel motor   = b.elevatorMotorModel;
        double gearRatio               = b.elevatorConfig.gearRatio;
        double spoolM                  = units.inches_m(b.pulleyRadiusIn);
        double carriageKg              = units.lbs_kg(b.carriageLbs);
        double[] stageKg               = toKgArray(b.stageMassesLbs);
        double[] pieceKg               = toKgArray(b.gamePieceTypesLbs);

        // Friction offsets sourced from existing tier2FF slot of elevator config.
        // Elevator friction is a fixed addend — not physics-derived — so we keep
        // it separate from the drivetrain-coupled Tier-3 physics below.
        // Users set friction via kS in the PID array (Tier 1) or via a tier2FF lambda.
        // Tier-3 here covers only gravity + drivetrain inertia + centripetal.

        ffProvider physicsFF = (positionDeg, velocityRps, accelRpss) -> {
            double totalKg = carriageKg;
            for (double s : stageKg) totalKg += s;
            for (int i = 0; i < pieceKg.length; i++)
                totalKg += pieceKg[i] * b.pieceCountSuppliers[i].get();

            double elevAngleRad = Math.toRadians(angleSupplier.get());

            // 1. Gravity — projected onto lift axis via sin(angle)
            double forceGravN   = totalKg * 9.80665 * Math.sin(elevAngleRad);
            double torqueGravNm = forceGravN * spoolM;
            double voltsGrav    = torqueGravNm * 12.0 / (gearRatio * motor.stallTorqueNm);

            // 2. Drivetrain linear inertia — forward accel projects onto lift axis
            driveOdometryState odom  = b.odometrySupplier.get();
            double robotHeading      = b.headingSupplier.get();
            double accelMag          = odom.blendedState.linearAccelerationMagnitude;
            double accelHeading      = odom.blendedState.linearAccelerationHeading;
            double accelFwd          = accelMag * Math.cos(accelHeading - robotHeading);
            double torqueInertNm     = totalKg * accelFwd
                                     * Math.cos(elevAngleRad) * spoolM;
            double voltsInert        = torqueInertNm * 12.0 / (gearRatio * motor.stallTorqueNm);

            // 3. Centripetal — robot rotation swings elevator CG in an arc
            double omega             = odom.blendedState.angularVelocity;
            double cgOffsetM         = units.inches_m(b.cgOffsetFromRobotCenterIn);
            double aCentripetal      = omega * omega * cgOffsetM;
            double torqueCentNm      = totalKg * aCentripetal
                                     * Math.cos(elevAngleRad) * spoolM;
            double voltsCent         = torqueCentNm * 12.0 / (gearRatio * motor.stallTorqueNm);

            return voltsGrav + voltsInert + voltsCent;
        };

        // Compose with any existing tier3FF so user-provided FF is preserved
        ffProvider existing = b.elevatorConfig.tier3FF;
        ffProvider combined = (pos, vel, accel) ->
            physicsFF.compute(pos, vel, accel) + existing.compute(pos, vel, accel);

        return rebuildWithTier3FF(b.elevatorConfig, combined);
    }

    /**
     * Returns a copy of the pivot mechanismConfig with Tier-3 FF wired.
     * Only called when a pivot is configured.
     */
    private static mechanismConfig applyPivotFF(Builder b,
                                                 Supplier<Double> angleSupplier) {
        motorModels.MotorModel motor   = b.pivotMotorModel;
        double gearRatio               = b.pivotConfig.gearRatio;
        double cgDistM                 = units.inches_m(b.cgDistanceFromPivotIn);
        double cgOffsetM               = units.inches_m(b.cgOffsetFromRobotCenterIn);
        double carriageKg              = units.lbs_kg(b.carriageLbs);
        double[] stageKg               = toKgArray(b.stageMassesLbs);
        double[] pieceKg               = toKgArray(b.gamePieceTypesLbs);

        ffProvider physicsFF = (positionDeg, velocityRps, accelRpss) -> {
            double totalKg = carriageKg;
            for (double s : stageKg) totalKg += s;
            for (int i = 0; i < pieceKg.length; i++)
                totalKg += pieceKg[i] * b.pieceCountSuppliers[i].get();

            double pivotRad = Math.toRadians(angleSupplier.get());

            // 1. Gravity — cos(pivot): max at 0° (horizontal), zero at 90° (vertical)
            //    Sign is automatic: positive 0°–90°, negative 90°–180°
            double torqueGravNm  = totalKg * 9.80665 * cgDistM * Math.cos(pivotRad);
            double voltsGrav     = torqueGravNm * 12.0 / (gearRatio * motor.stallTorqueNm);

            // 2. Drivetrain linear inertia projected onto pivot load axis via sin(pivot)
            driveOdometryState odom = b.odometrySupplier.get();
            double robotHeading     = b.headingSupplier.get();
            double accelMag         = odom.blendedState.linearAccelerationMagnitude;
            double accelHeading     = odom.blendedState.linearAccelerationHeading;
            double accelFwd         = accelMag * Math.cos(accelHeading - robotHeading);
            double torqueInertNm    = totalKg * accelFwd * cgDistM * Math.sin(pivotRad);
            double voltsInert       = torqueInertNm * 12.0 / (gearRatio * motor.stallTorqueNm);

            // 3. Centripetal
            double omega            = odom.blendedState.angularVelocity;
            double aCentripetal     = omega * omega * cgOffsetM;
            double torqueCentNm     = totalKg * aCentripetal * cgDistM * Math.sin(pivotRad);
            double voltsCent        = torqueCentNm * 12.0 / (gearRatio * motor.stallTorqueNm);

            return voltsGrav + voltsInert + voltsCent;
        };

        ffProvider existing = b.pivotConfig.tier3FF;
        ffProvider combined = (pos, vel, accel) ->
            physicsFF.compute(pos, vel, accel) + existing.compute(pos, vel, accel);

        return rebuildWithTier3FF(b.pivotConfig, combined);
    }

    /**
     * Rebuild a mechanismConfig with a replacement tier3FF, preserving all other fields.
     */
    private static mechanismConfig rebuildWithTier3FF(mechanismConfig src, ffProvider ff) {
        return new mechanismConfig.Builder(src.name, src.canIds, src.vendors)
            .withFollowerConfig(src.followerModes, src.followerInverted,
                                src.followerLeaderIndices)
            .withGearRatio(src.gearRatio)
            .withInverted(src.inverted)
            .withPID(src.pid)
            .withSoftLimits(src.softLimitReverseDeg, src.softLimitForwardDeg)
            .withCurrentLimits(src.supplyCurrentLimitAmps, src.statorCurrentLimitAmps)
            .withRampRates(src.openLoopRampSecs, src.closedLoopRampSecs)
            .withMotionProfile(src.motionCruiseVelocityRps,
                               src.motionAccelerationRpss, src.motionJerkRpss3)
            .withTier2FF(src.tier2FF)
            .withTier3FF(ff)
            .withSetpointDeadband(src.setpointDeadband)
            .withBrakeOnNeutral(src.brakeOnNeutral)
            .withEncoderSync(src.encoderSyncDeadbandRot, src.encoderSyncKp)
            .build();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Internal — helpers
    // ─────────────────────────────────────────────────────────────────────────

    private static double[] toKgArray(double[] lbsArray) {
        if (lbsArray == null) return new double[0];
        double[] kg = new double[lbsArray.length];
        for (int i = 0; i < lbsArray.length; i++) kg[i] = units.lbs_kg(lbsArray[i]);
        return kg;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Internal — startup status message
    // ─────────────────────────────────────────────────────────────────────────

    private void printConfigStatus(Builder b) {
        StringBuilder sb = new StringBuilder();
        sb.append("[ElevatorMechanism] '").append(b.name).append("' initialized:\n");
        sb.append("  Elevator : ").append(describeConfig(b.elevatorConfig))
          .append(" | pulley ").append(b.pulleyRadiusIn).append(" in")
          .append(" | default ").append(b.defaultElevatorIn).append(" in\n");

        if (b.pivotConfig != null) {
            sb.append("  Pivot    : ").append(describeConfig(b.pivotConfig))
              .append(" | default ").append(b.defaultPivotDeg).append("°\n");
        } else if (b.externalAngleSupplier != null) {
            sb.append("  Pivot    : external angle supplier\n");
        } else {
            sb.append("  Pivot    : NOT configured (fixed vertical)\n");
        }

        sb.append("  Carriage : ").append(b.carriageLbs).append(" lbs");
        if (b.stageMassesLbs != null && b.stageMassesLbs.length > 0) {
            sb.append(" | ").append(b.stageMassesLbs.length).append(" stage(s)");
        }
        if (b.gamePieceTypesLbs != null && b.gamePieceTypesLbs.length > 0) {
            sb.append(" | ").append(b.gamePieceTypesLbs.length).append(" game piece type(s)");
        }
        sb.append("\n");
        sb.append("  Hold threshold : ").append(b.holdThresholdIn).append(" in");
        System.out.println(sb);
    }

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
     * Fluent builder for elevatorMechanism.
     *
     * Required: withElevator(), withMasses(), withDrivetrainSuppliers().
     * Optional: withPivot() or withExternalAngleSupplier(), withElevatorHoldThresholdIn().
     *
     * Example — fixed vertical, single stage, one game piece type:
     * <pre>{@code
     * elevatorMechanism elevator = new elevatorMechanism.Builder("Elevator")
     *     .withElevator(elevatorConfig, motorModels.NEO_VORTEX, 0.75, 0.0)
     *     .withMasses(8.0, new double[]{ 3.0 },
     *                 new double[]{ 1.5 }, new Supplier[]{ indexer::getPieceCount })
     *     .withDrivetrainSuppliers(drive::getOdometryState, drive::getHeadingRad,
     *                              12.0)
     *     .withElevatorHoldThresholdIn(0.25)
     *     .build();
     * }</pre>
     *
     * Example — pivoting base, two stages:
     * <pre>{@code
     * elevatorMechanism elevator = new elevatorMechanism.Builder("PivotElevator")
     *     .withElevator(elevatorConfig, motorModels.NEO_VORTEX, 0.75, 0.0)
     *     .withPivot(pivotConfig, motorModels.NEO, 18.0, 6.0, 90.0)
     *     .withMasses(8.0, new double[]{ 3.0, 2.5 },
     *                 new double[]{ 1.5 }, new Supplier[]{ indexer::getPieceCount })
     *     .withDrivetrainSuppliers(drive::getOdometryState, drive::getHeadingRad,
     *                              12.0)
     *     .withElevatorHoldThresholdIn(0.25)
     *     .build();
     * }</pre>
     */
    public static class Builder {

        private final String name;

        // Elevator — required
        mechanismConfig elevatorConfig;
        motorModels.MotorModel elevatorMotorModel;
        double pulleyRadiusIn    = 0.0;
        double defaultElevatorIn = 0.0;

        // Pivot — optional
        mechanismConfig pivotConfig              = null;
        motorModels.MotorModel pivotMotorModel   = null;
        double cgDistanceFromPivotIn             = 0.0;
        double defaultPivotDeg                   = 90.0;

        // External angle supplier — optional alternative to internal pivot
        Supplier<Double> externalAngleSupplier   = null;

        // Masses — required
        double carriageLbs                       = 0.0;
        double[] stageMassesLbs                  = new double[0];
        double[] gamePieceTypesLbs               = new double[0];
        @SuppressWarnings("unchecked")
        Supplier<Integer>[] pieceCountSuppliers  = new Supplier[0];

        // Drivetrain coupling — required
        Supplier<driveOdometryState> odometrySupplier = null;
        Supplier<Double> headingSupplier               = null;
        double cgOffsetFromRobotCenterIn               = 0.0;

        // Hold threshold — optional
        double holdThresholdIn                   = 0.0;
        private boolean holdThresholdSet         = false;

        public Builder(String name) {
            this.name = name;
        }

        /**
         * Configure the elevator sub-mechanism.
         * setpointDeadband must be set in the mechanismConfig (in shaft degrees,
         * converted from inches: deadbandIn × degsPerIn).
         *
         * @param config          full mechanismConfig for the elevator motor(s)
         * @param motorModel      motor model for FF calculation (e.g. motorModels.NEO_VORTEX)
         * @param pulleyRadiusIn  timing belt pulley output radius (inches, fixed)
         * @param defaultIn       default elevator position (inches)
         */
        public Builder withElevator(mechanismConfig config,
                                    motorModels.MotorModel motorModel,
                                    double pulleyRadiusIn,
                                    double defaultIn) {
            this.elevatorConfig    = config;
            this.elevatorMotorModel = motorModel;
            this.pulleyRadiusIn    = pulleyRadiusIn;
            this.defaultElevatorIn = defaultIn;
            return this;
        }

        /**
         * Configure an internal pivot sub-mechanism. When present, the elevator
         * angle is read from the pivot motor's position each cycle.
         *
         * @param config              full mechanismConfig for the pivot motor(s)
         * @param motorModel          motor model for pivot FF calculation
         * @param cgDistFromPivotIn   distance from pivot axis to assembly CG (inches)
         * @param cgOffsetFromCenterIn distance from robot center to assembly CG
         *                             projected onto rotation plane (inches)
         * @param defaultDeg          default pivot angle (degrees)
         */
        public Builder withPivot(mechanismConfig config,
                                 motorModels.MotorModel motorModel,
                                 double cgDistFromPivotIn,
                                 double cgOffsetFromCenterIn,
                                 double defaultDeg) {
            this.pivotConfig            = config;
            this.pivotMotorModel        = motorModel;
            this.cgDistanceFromPivotIn  = cgDistFromPivotIn;
            this.cgOffsetFromRobotCenterIn = cgOffsetFromCenterIn;
            this.defaultPivotDeg        = defaultDeg;
            return this;
        }

        /**
         * Supply elevator angle from an external source instead of an internal pivot.
         * Use when the pivot is a separate subsystem managed elsewhere.
         * Mutually exclusive with withPivot() — last call wins.
         *
         * @param supplier supplies elevator angle from ground (degrees) each cycle.
         *                 90° = vertical, 0° = horizontal.
         */
        public Builder withExternalAngleSupplier(Supplier<Double> supplier) {
            this.externalAngleSupplier = supplier;
            return this;
        }

        /**
         * Set elevator and stage masses and game piece types.
         *
         * @param carriageLbs         carriage mass (lbs)
         * @param stageMassesLbs      mass of each additional stage (lbs). Empty = single stage.
         * @param gamePieceTypesLbs   mass of each game piece type (lbs)
         * @param pieceCountSuppliers runtime count per piece type, parallel to gamePieceTypesLbs
         */
        @SuppressWarnings("unchecked")
        public Builder withMasses(double carriageLbs,
                                  double[] stageMassesLbs,
                                  double[] gamePieceTypesLbs,
                                  Supplier<Integer>[] pieceCountSuppliers) {
            this.carriageLbs          = carriageLbs;
            this.stageMassesLbs       = stageMassesLbs   != null ? stageMassesLbs   : new double[0];
            this.gamePieceTypesLbs    = gamePieceTypesLbs != null ? gamePieceTypesLbs : new double[0];
            this.pieceCountSuppliers  = pieceCountSuppliers != null
                ? pieceCountSuppliers : new Supplier[0];
            if (this.gamePieceTypesLbs.length != this.pieceCountSuppliers.length)
                throw new IllegalArgumentException(
                    "elevatorMechanism.Builder '" + name + "': gamePieceTypesLbs and "
                    + "pieceCountSuppliers must be the same length.");
            return this;
        }

        /**
         * Provide drivetrain state suppliers for inertia and centripetal FF.
         *
         * @param odometrySupplier       supplies driveOdometryState each cycle
         * @param headingSupplier        supplies robot heading (radians, field-relative, CCW+)
         * @param cgOffsetFromCenterIn   distance from robot center to elevator assembly CG,
         *                               projected onto rotation plane (inches).
         *                               Used for centripetal FF. Also used for pivot FF
         *                               if withPivot() does not set its own offset.
         */
        public Builder withDrivetrainSuppliers(Supplier<driveOdometryState> odometrySupplier,
                                               Supplier<Double> headingSupplier,
                                               double cgOffsetFromCenterIn) {
            this.odometrySupplier           = odometrySupplier;
            this.headingSupplier            = headingSupplier;
            this.cgOffsetFromRobotCenterIn  = cgOffsetFromCenterIn;
            return this;
        }

        /**
         * Set the elevator position drop allowed before isReady() returns to false.
         * Applied relative to the active elevator setpoint each cycle.
         *
         * @param thresholdIn maximum drop below setpoint (inches) before losing READY state
         */
        public Builder withElevatorHoldThresholdIn(double thresholdIn) {
            this.holdThresholdIn = thresholdIn;
            this.holdThresholdSet = true;
            return this;
        }

        /** Build and return the elevatorMechanism instance. */
        public elevatorMechanism build() {
            if (elevatorConfig == null)
                throw new IllegalStateException(
                    "elevatorMechanism.Builder '" + name + "': withElevator() is required.");
            if (elevatorMotorModel == null)
                throw new IllegalStateException(
                    "elevatorMechanism.Builder '" + name + "': motor model must be provided "
                    + "in withElevator().");
            if (odometrySupplier == null || headingSupplier == null)
                throw new IllegalStateException(
                    "elevatorMechanism.Builder '" + name + "': withDrivetrainSuppliers() "
                    + "is required.");
            if (pivotConfig != null && externalAngleSupplier != null)
                throw new IllegalStateException(
                    "elevatorMechanism.Builder '" + name + "': withPivot() and "
                    + "withExternalAngleSupplier() are mutually exclusive.");
            if (!holdThresholdSet)
                DriverStation.reportWarning(
                    "[elevatorMechanism] '" + name + "': withElevatorHoldThresholdIn() was not "
                    + "called. Hold threshold defaults to 0 in — isReady() will drop to false "
                    + "any time elevator position is below setpoint.", false);
            return new elevatorMechanism(this);
        }
    }
}
