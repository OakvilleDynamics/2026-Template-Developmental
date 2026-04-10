package frc.robot.util;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

import frc.robot.constants.shooterConstants;
import frc.robot.constants.shooterConstants.LookupEntry;
import frc.robot.util.motors.mechanismConfig;

/**
 * ShooterCalculator.java
 * PATH: src/main/java/frc/robot/util/ShooterCalculator.java
 *
 * Pure-math shooting calculator — no hardware, no WPILib dependencies.
 * Testable in isolation from a standard Java main().
 *
 * Two entry points:
 *   calculateStaticShot()  — robot is fully stopped when firing
 *   calculateOnTheMove()   — robot is moving; compensates for robot and
 *                            turret motion during ball flight
 *
 * Shot profiles (RPM, hood angle, ToF) are read from shooterConstants.LOOKUP_TABLE.
 * Pivot geometry and turret limits are also sourced from shooterConstants but
 * passed in as constructor parameters so they can be overridden for testing.
 *
 * ─── UNITS AT THIS INTERFACE ─────────────────────────────────────────────────
 * Angles           : degrees
 * Linear distances : feet
 * Linear velocities: ft/s
 * Linear accels    : ft/s²
 * Angular velocities  : deg/s
 * Angular accelerations: deg/s²
 *
 * ─── OUTPUT ──────────────────────────────────────────────────────────────────
 * ShooterOutput contains:
 *   rpm, hoodAngle         — feed directly to shooterMechanism
 *   turretAngleRobotDeg    — commanded turret angle (robot-relative)
 *   shooterHeadingFieldDeg — absolute field heading from pivot to target
 *   shooterHeadingRobotDeg — same, robot-relative
 *   headingBoundsDeg       — [min, max] absolute robot headings that keep the
 *                            turret within its soft stops; used by
 *                            shooterAimController to bound drive heading
 *   warnings               — set of ShooterWarning flags
 */
public class ShooterCalculator {

    // =========================================================================
    // Constructor / instance configuration
    // =========================================================================

    private final double controlLoopHz;
    private final int    maxFlightTimeIterations;
    private final double turretSoftLimitMinDeg;
    private final double turretSoftLimitMaxDeg;
    private final double turretHardLimitMinDeg;
    private final double turretHardLimitMaxDeg;

    /**
     * Convenience constructor — reads turret soft limits directly from the turret's
     * mechanismConfig so the values stay in one place.
     *
     * mechanismConfig.softLimitReverseDeg → soft min (most-negative allowed angle)
     * mechanismConfig.softLimitForwardDeg → soft max (most-positive allowed angle)
     *
     * Hard limits are set equal to the soft limits since mechanismConfig has no
     * separate hard-stop field — motor-controller hard limits are enforced in hardware,
     * not here. The TURRET_ANGLE_EXCEEDS_LIMITS warning will therefore fire at the
     * same threshold as the soft limits.
     *
     * For SWERVE_AS_TURRET (no physical turret), use the full constructor and pass
     * softMin = softMax = hardMin = hardMax = 0.0. The heading bounds will collapse
     * to a tight window around the exact aim heading.
     *
     * @param controlLoopHz           Control loop rate (Hz).
     * @param maxFlightTimeIterations Maximum ToF refinement iterations for OnTheMove.
     * @param turretConfig            mechanismConfig of the turret sub-mechanism.
     */
    public ShooterCalculator(
            double controlLoopHz,
            int    maxFlightTimeIterations,
            mechanismConfig turretConfig) {
        this(controlLoopHz, maxFlightTimeIterations,
             turretConfig.softLimitReverseDeg,
             turretConfig.softLimitForwardDeg,
             turretConfig.softLimitReverseDeg,
             turretConfig.softLimitForwardDeg);
    }

    /**
     * Full constructor — use when turret limits are not backed by a mechanismConfig
     * (e.g. SWERVE_AS_TURRET, or standalone smoke testing).
     *
     * @param controlLoopHz            Control loop rate (Hz) — used as convergence
     *                                 tolerance for ToF iteration (1 loop period).
     * @param maxFlightTimeIterations  Maximum ToF refinement iterations for OnTheMove.
     * @param turretSoftLimitMinDeg    Software minimum turret angle (robot-relative °).
     * @param turretSoftLimitMaxDeg    Software maximum turret angle (robot-relative °).
     * @param turretHardLimitMinDeg    Physical hard-stop minimum (robot-relative °).
     * @param turretHardLimitMaxDeg    Physical hard-stop maximum (robot-relative °).
     */
    public ShooterCalculator(
            double controlLoopHz,
            int    maxFlightTimeIterations,
            double turretSoftLimitMinDeg,
            double turretSoftLimitMaxDeg,
            double turretHardLimitMinDeg,
            double turretHardLimitMaxDeg) {

        this.controlLoopHz           = controlLoopHz;
        this.maxFlightTimeIterations = maxFlightTimeIterations;
        this.turretSoftLimitMinDeg   = turretSoftLimitMinDeg;
        this.turretSoftLimitMaxDeg   = turretSoftLimitMaxDeg;
        this.turretHardLimitMinDeg   = turretHardLimitMinDeg;
        this.turretHardLimitMaxDeg   = turretHardLimitMaxDeg;
    }

    // =========================================================================
    // Shot Mode
    // =========================================================================

    public enum ShotMode {
        /** Robot is fully stopped before firing. */
        STATIC_SHOT,
        /** Robot is moving while firing; uses motion compensation. */
        ON_THE_MOVE
    }

    // =========================================================================
    // Warning Flags
    // =========================================================================

    public enum ShooterWarning {
        /** Calculated distance is outside the lookup table range; result is extrapolated. */
        DISTANCE_OUT_OF_TABLE_RANGE,
        /** ToF iterations did not converge to a stable solution. */
        FLIGHT_TIME_DID_NOT_CONVERGE,
        /** Commanded turret angle exceeds one or more mechanical limits. */
        TURRET_ANGLE_EXCEEDS_LIMITS,
        /** Robot speed is high enough that motion compensation reliability is reduced. */
        ROBOT_SPEED_TOO_HIGH,
        /** No lookup entries found for the requested target type. */
        UNKNOWN_TARGET_TYPE
    }

    // =========================================================================
    // Shooter Output
    // =========================================================================

    public static class ShooterOutput {

        /** Hood angle (degrees). Feed to shooterMechanism.setHoodDeg(). */
        public final double hoodAngle;

        /** Flywheel speed (RPM). Feed to shooterMechanism.setFlywheelRpm(). */
        public final double rpm;

        /** Which calculation path produced this output. */
        public final ShotMode shotMode;

        /** Any degraded-accuracy or range flags raised during calculation. */
        public final Set<ShooterWarning> warnings;

        /**
         * Absolute field-frame angle from shooter pivot to target (degrees).
         * 0° = field +X axis, CCW positive, range (-180°, 180°].
         */
        public final double shooterHeadingFieldDeg;

        /**
         * Angle from shooter pivot to target relative to robot heading (degrees).
         * 0° = robot forward, positive = left of robot, range (-180°, 180°].
         * For a non-turret robot this is the rotation the whole robot must execute.
         */
        public final double shooterHeadingRobotDeg;

        /**
         * Commanded turret angle relative to the robot body (degrees).
         * 0° = turret aligned with robot forward.
         * For OnTheMove this accounts for where the turret will have drifted by
         * the time the ball arrives. For StaticShot equals shooterHeadingRobotDeg.
         */
        public final double turretAngleRobotDeg;

        /**
         * Commanded turret angle in the field frame (degrees).
         * 0° = field +X axis, CCW positive.
         */
        public final double turretAngleFieldDeg;

        /**
         * Absolute robot heading bounds [min, max] (degrees, field frame) within
         * which the turret can reach the target without exceeding its soft stops.
         *
         * Derivation: turretAngle_robot = aimFieldDeg − robotHeadingDeg.
         * Rearranging for robotHeadingDeg within [softMin, softMax]:
         *   bounds[0] = normalize(aimFieldDeg − softLimitMax)  ← min robot heading
         *   bounds[1] = normalize(aimFieldDeg − softLimitMin)  ← max robot heading
         *
         * Pass these to driveWithJoysticks.enableHeadingBound() so the driver
         * retains translation freedom while heading stays within the valid window.
         *
         * For swerve-as-turret (softMin == softMax == 0°) the bounds collapse to
         * [aimFieldDeg, aimFieldDeg] — robot must face the target exactly.
         */
        public final double[] headingBoundsDeg;   // [min, max]

        public ShooterOutput(
                double hoodAngle,
                double rpm,
                ShotMode shotMode,
                double shooterHeadingFieldDeg,
                double shooterHeadingRobotDeg,
                double turretAngleRobotDeg,
                double turretAngleFieldDeg,
                double[] headingBoundsDeg,
                Set<ShooterWarning> warnings) {

            this.hoodAngle              = hoodAngle;
            this.rpm                    = rpm;
            this.shotMode               = shotMode;
            this.shooterHeadingFieldDeg = shooterHeadingFieldDeg;
            this.shooterHeadingRobotDeg = shooterHeadingRobotDeg;
            this.turretAngleRobotDeg    = turretAngleRobotDeg;
            this.turretAngleFieldDeg    = turretAngleFieldDeg;
            this.headingBoundsDeg       = headingBoundsDeg;
            this.warnings               = warnings;
        }

        public boolean hasWarnings() {
            return !warnings.isEmpty();
        }

        /** True when the robot heading is within headingBoundsDeg. */
        public boolean isHeadingInBounds(double robotHeadingDeg) {
            return robotHeadingDeg >= headingBoundsDeg[0]
                && robotHeadingDeg <= headingBoundsDeg[1];
        }

        @Override
        public String toString() {
            return String.format(
                "[%s]  Hood: %.2f°  RPM: %.1f  " +
                "Heading(field): %.2f°  Heading(robot): %.2f°  " +
                "Turret(robot): %.2f°  Turret(field): %.2f°  " +
                "HeadingBounds: [%.2f°, %.2f°]  " +
                "Warnings: %s",
                shotMode, hoodAngle, rpm,
                shooterHeadingFieldDeg, shooterHeadingRobotDeg,
                turretAngleRobotDeg, turretAngleFieldDeg,
                headingBoundsDeg[0], headingBoundsDeg[1],
                warnings.isEmpty() ? "none" : warnings.toString());
        }
    }

    // =========================================================================
    // Primary entry point — StaticShot
    //
    // @param targetType       Must match a targetType key in shooterConstants.LOOKUP_TABLE
    // @param targetX          Field X of target (feet)
    // @param targetY          Field Y of target (feet)
    // @param robotX           Field X of robot center (feet)
    // @param robotY           Field Y of robot center (feet)
    // @param robotHeadingDeg  Robot heading (degrees, 0° = field +X, CCW positive)
    // =========================================================================
    public ShooterOutput calculateStaticShot(
            String targetType,
            double targetX,    double targetY,
            double robotX,     double robotY,
            double robotHeadingDeg) {

        Set<ShooterWarning> warnings = EnumSet.noneOf(ShooterWarning.class);

        if (!targetTypeExists(targetType)) {
            warnings.add(ShooterWarning.UNKNOWN_TARGET_TYPE);
            return emptyOutput(ShotMode.STATIC_SHOT, warnings);
        }

        // --- 1. Project pivot to field frame ---
        double[] pivotField = pivotToFieldFrame(robotX, robotY, robotHeadingDeg);
        double pivotFieldX  = pivotField[0];
        double pivotFieldY  = pivotField[1];

        // --- 2. Distance and headings from pivot to target ---
        double dx                    = targetX - pivotFieldX;
        double dy                    = targetY - pivotFieldY;
        double distanceFt            = Math.sqrt(dx * dx + dy * dy);
        double headingFieldDeg       = Math.toDegrees(Math.atan2(dy, dx));
        double headingRobotDeg       = normalizeAngleDeg(headingFieldDeg - robotHeadingDeg);

        System.out.printf("[StaticShot] Pivot field pos      : (%.3f, %.3f)%n", pivotFieldX, pivotFieldY);
        System.out.printf("[StaticShot] Distance to target   : %.3f ft%n",      distanceFt);
        System.out.printf("[StaticShot] Heading (field)      : %.2f°%n",         headingFieldDeg);
        System.out.printf("[StaticShot] Heading (robot)      : %.2f°%n",         headingRobotDeg);

        // --- 3. Lookup and interpolate ---
        InterpolationResult result = interpolate(targetType, distanceFt, warnings);

        // For StaticShot turret angle equals the shooter heading (robot aims at target)
        double turretAngleRobotDeg = headingRobotDeg;
        double turretAngleFieldDeg = headingFieldDeg;

        checkTurretLimits(turretAngleRobotDeg, warnings);

        // --- 4. Robot heading bounds ---
        // Range of robot headings within which the turret stays inside soft stops.
        double[] headingBoundsDeg = computeHeadingBounds(headingFieldDeg);

        return new ShooterOutput(
                result.hoodAngle, result.rpm,
                ShotMode.STATIC_SHOT,
                headingFieldDeg, headingRobotDeg,
                turretAngleRobotDeg, turretAngleFieldDeg,
                headingBoundsDeg,
                warnings);
    }

    // =========================================================================
    // Primary entry point — OnTheMove
    //
    // All angles in degrees, distances in feet, velocities in ft/s,
    // accelerations in ft/s², angular velocities in deg/s,
    // angular accelerations in deg/s².
    //
    // @param targetType            Must match a targetType key in LOOKUP_TABLE
    // @param targetX               Field X of target (feet)
    // @param targetY               Field Y of target (feet)
    // @param robotX                Field X of robot center (feet)
    // @param robotY                Field Y of robot center (feet)
    // @param robotHeadingDeg       Robot heading (0° = field +X)
    // @param robotLinVelX          Robot linear velocity — field X component (ft/s)
    // @param robotLinVelY          Robot linear velocity — field Y component (ft/s)
    // @param robotLinAccelX        Robot linear acceleration — field X component (ft/s²)
    // @param robotLinAccelY        Robot linear acceleration — field Y component (ft/s²)
    // @param robotAngVelDegS       Robot angular velocity (deg/s, CCW positive)
    // @param robotAngAccelDegS2    Robot angular acceleration (deg/s², CCW positive)
    // @param corX                  Center of rotation — robot-relative X (feet, 0 = robot center)
    // @param corY                  Center of rotation — robot-relative Y (feet, 0 = robot center)
    // @param turretAngleRobotDeg   Current turret angle relative to robot body (0°=fwd)
    // @param turretAngVelDegS      Turret angular velocity (deg/s, CCW positive, robot-relative)
    // @param turretAngAccelDegS2   Turret angular acceleration (deg/s², CCW positive, robot-relative)
    // =========================================================================
    public ShooterOutput calculateOnTheMove(
            String targetType,
            double targetX,           double targetY,
            double robotX,            double robotY,
            double robotHeadingDeg,
            double robotLinVelX,      double robotLinVelY,
            double robotLinAccelX,    double robotLinAccelY,
            double robotAngVelDegS,   double robotAngAccelDegS2,
            double corX,              double corY,
            double turretAngleRobotDeg,
            double turretAngVelDegS,  double turretAngAccelDegS2) {

        Set<ShooterWarning> warnings = EnumSet.noneOf(ShooterWarning.class);

        if (!targetTypeExists(targetType)) {
            warnings.add(ShooterWarning.UNKNOWN_TARGET_TYPE);
            return emptyOutput(ShotMode.ON_THE_MOVE, warnings);
        }

        // Flag if robot speed is high enough to reduce compensation reliability
        double robotSpeed = Math.sqrt(robotLinVelX * robotLinVelX + robotLinVelY * robotLinVelY);
        if (robotSpeed > shooterConstants.ROBOT_SPEED_WARNING_THRESHOLD_FT_S) {
            warnings.add(ShooterWarning.ROBOT_SPEED_TOO_HIGH);
        }

        // --- 1. Current pivot field position ---
        double[] pivotField  = pivotToFieldFrame(robotX, robotY, robotHeadingDeg);
        double pivotFieldX   = pivotField[0];
        double pivotFieldY   = pivotField[1];

        // --- 2. Initial distance estimate using current pivot position ---
        double dxNow         = targetX - pivotFieldX;
        double dyNow         = targetY - pivotFieldY;
        double distanceNow   = Math.sqrt(dxNow * dxNow + dyNow * dyNow);

        // --- 3. Iterative ToF refinement ---
        // Seed with ToF from the table at the current distance, then refine
        // by projecting the pivot position forward by dt and recomputing distance.
        double estimatedToF      = lookupToF(targetType, distanceNow);
        double compensatedDistFt = distanceNow;
        double projPivotX        = pivotFieldX;
        double projPivotY        = pivotFieldY;
        double projRobotHeadDeg  = robotHeadingDeg;
        boolean converged        = false;

        for (int i = 0; i < maxFlightTimeIterations; i++) {
            double dt = estimatedToF;

            // Project robot heading at t+dt (constant-acceleration rotational kinematics)
            projRobotHeadDeg = normalizeAngleDeg(
                    robotHeadingDeg
                    + robotAngVelDegS    * dt
                    + 0.5 * robotAngAccelDegS2 * dt * dt);

            // Project robot center position at t+dt using both linear and
            // rotational motion about the provided center of rotation.
            //
            // Step A: linear displacement from robot center
            double linearDispX = robotLinVelX * dt + 0.5 * robotLinAccelX * dt * dt;
            double linearDispY = robotLinVelY * dt + 0.5 * robotLinAccelY * dt * dt;

            // Step B: rotational displacement of robot center about the CoR.
            // corX/corY are robot-relative — convert to field frame first.
            double corHeadingRad = Math.toRadians(robotHeadingDeg);
            double corFieldX     = robotX + corX * Math.cos(corHeadingRad)
                                          - corY * Math.sin(corHeadingRad);
            double corFieldY     = robotY + corX * Math.sin(corHeadingRad)
                                          + corY * Math.cos(corHeadingRad);

            double robotAngDispDeg = robotAngVelDegS * dt
                                   + 0.5 * robotAngAccelDegS2 * dt * dt;
            double robotAngDispRad = Math.toRadians(robotAngDispDeg);

            // Rotate robot center about the field-frame CoR
            double rX = robotX - corFieldX;
            double rY = robotY - corFieldY;
            double projRobotX = corFieldX
                    + rX * Math.cos(robotAngDispRad) - rY * Math.sin(robotAngDispRad)
                    + linearDispX;
            double projRobotY = corFieldY
                    + rX * Math.sin(robotAngDispRad) + rY * Math.cos(robotAngDispRad)
                    + linearDispY;

            // Project pivot to field frame using projected robot pose
            double[] projPivot = pivotToFieldFrame(projRobotX, projRobotY, projRobotHeadDeg);
            projPivotX = projPivot[0];
            projPivotY = projPivot[1];

            // Compensated distance from projected pivot to target
            double dxComp = targetX - projPivotX;
            double dyComp = targetY - projPivotY;
            compensatedDistFt = Math.sqrt(dxComp * dxComp + dyComp * dyComp);

            // Look up refined ToF at compensated distance
            double newToF = lookupToF(targetType, compensatedDistFt);

            // Converged if ToF changed by less than one control loop period
            if (Math.abs(newToF - estimatedToF) < (1.0 / controlLoopHz)) {
                estimatedToF = newToF;
                converged    = true;
                break;
            }

            estimatedToF = newToF;
        }

        if (!converged) {
            warnings.add(ShooterWarning.FLIGHT_TIME_DID_NOT_CONVERGE);
        }

        // --- 4. Project turret aim direction at t+dt ---
        double turretAngDispDeg   = turretAngVelDegS    * estimatedToF
                                  + 0.5 * turretAngAccelDegS2 * estimatedToF * estimatedToF;
        double projTurretRobotDeg = normalizeAngleDeg(turretAngleRobotDeg + turretAngDispDeg);
        // projTurretFieldDeg is not used downstream but kept for potential telemetry
        @SuppressWarnings("unused")
        double projTurretFieldDeg = normalizeAngleDeg(projRobotHeadDeg + projTurretRobotDeg);

        // --- 5. Required aim heading at t+dt ---
        double dxComp              = targetX - projPivotX;
        double dyComp              = targetY - projPivotY;
        double aimHeadingFieldDeg  = Math.toDegrees(Math.atan2(dyComp, dxComp));
        double aimHeadingRobotDeg  = normalizeAngleDeg(aimHeadingFieldDeg - projRobotHeadDeg);

        // --- 6. Commanded turret angle ---
        double cmdTurretRobotDeg = normalizeAngleDeg(aimHeadingRobotDeg);
        double cmdTurretFieldDeg = normalizeAngleDeg(projRobotHeadDeg + cmdTurretRobotDeg);

        checkTurretLimits(cmdTurretRobotDeg, warnings);

        // --- 7. Lookup and interpolate RPM and hood angle ---
        InterpolationResult result = interpolate(targetType, compensatedDistFt, warnings);

        System.out.printf("[OnTheMove] Current pivot pos         : (%.3f, %.3f)%n", pivotFieldX,   pivotFieldY);
        System.out.printf("[OnTheMove] Projected pivot pos       : (%.3f, %.3f)%n", projPivotX,    projPivotY);
        System.out.printf("[OnTheMove] Compensated distance      : %.3f ft%n",      compensatedDistFt);
        System.out.printf("[OnTheMove] Estimated ToF             : %.4f s%n",        estimatedToF);
        System.out.printf("[OnTheMove] Converged                 : %b%n",            converged);
        System.out.printf("[OnTheMove] Aim heading (field)       : %.2f°%n",         aimHeadingFieldDeg);
        System.out.printf("[OnTheMove] Aim heading (robot)       : %.2f°%n",         aimHeadingRobotDeg);
        System.out.printf("[OnTheMove] Cmd turret angle (robot)  : %.2f°%n",         cmdTurretRobotDeg);
        System.out.printf("[OnTheMove] Cmd turret angle (field)  : %.2f°%n",         cmdTurretFieldDeg);

        // --- 8. Robot heading bounds ---
        double[] headingBoundsDeg = computeHeadingBounds(aimHeadingFieldDeg);

        return new ShooterOutput(
                result.hoodAngle, result.rpm,
                ShotMode.ON_THE_MOVE,
                aimHeadingFieldDeg, aimHeadingRobotDeg,
                cmdTurretRobotDeg, cmdTurretFieldDeg,
                headingBoundsDeg,
                warnings);
    }

    // =========================================================================
    // Interpolation
    //
    // Filters by targetType, then by hood angle groups, interpolates RPM and
    // ToF on distance, selects the hood angle group whose bracket midpoint is
    // closest to the measured distance.
    //
    // Exact-distance hits: use that entry's values directly. If multiple hood
    // angles share the exact distance, the lower hood angle is preferred.
    //
    // Out-of-range distance: extrapolate using the two nearest entries on the
    // closest side rather than snapping to a single edge value.
    // =========================================================================

    private static class InterpolationResult {
        final double hoodAngle;
        final double rpm;
        final double timeOfFlight;

        InterpolationResult(double hoodAngle, double rpm, double timeOfFlight) {
            this.hoodAngle    = hoodAngle;
            this.rpm          = rpm;
            this.timeOfFlight = timeOfFlight;
        }
    }

    private InterpolationResult interpolate(
            String targetType,
            double distanceFt,
            Set<ShooterWarning> warnings) {

        double[] uniqueAngles = getUniqueHoodAngles(targetType);

        // --- Exact-distance match: use the entry with the lowest hood angle ---
        LookupEntry exactMatch = null;
        for (LookupEntry e : shooterConstants.LOOKUP_TABLE) {
            if (!e.targetType.equals(targetType)) continue;
            if (e.distance == distanceFt) {
                if (exactMatch == null || e.hoodAngle < exactMatch.hoodAngle) {
                    exactMatch = e;
                }
            }
        }
        if (exactMatch != null) {
            return new InterpolationResult(
                    exactMatch.hoodAngle,
                    exactMatch.rpm,
                    exactMatch.timeOfFlight);
        }

        // --- Bracketing interpolation ---
        double bestAngle     = Double.NaN;
        double bestRPM       = Double.NaN;
        double bestToF       = Double.NaN;
        double bestDeviation = Double.MAX_VALUE;
        boolean outOfRange   = false;

        for (double angle : uniqueAngles) {
            LookupEntry[] group = getEntriesForAngle(targetType, angle);
            if (group.length < 2) continue;

            LookupEntry lower = null;
            LookupEntry upper = null;

            for (LookupEntry e : group) {
                if (e.distance <= distanceFt) {
                    if (lower == null || e.distance > lower.distance) lower = e;
                }
                if (e.distance >= distanceFt) {
                    if (upper == null || e.distance < upper.distance) upper = e;
                }
            }

            // Out-of-range: extrapolate using the two nearest entries on the closest side
            boolean thisGroupOutOfRange = false;
            if (lower == null) {
                lower = group[0];
                upper = group[1];
                thisGroupOutOfRange = true;
            } else if (upper == null) {
                lower = group[group.length - 2];
                upper = group[group.length - 1];
                thisGroupOutOfRange = true;
            }

            double t               = (distanceFt - lower.distance) / (upper.distance - lower.distance);
            double interpolatedRPM = lower.rpm          + t * (upper.rpm          - lower.rpm);
            double interpolatedToF = lower.timeOfFlight + t * (upper.timeOfFlight - lower.timeOfFlight);

            double midpoint  = (lower.distance + upper.distance) / 2.0;
            double deviation = Math.abs(distanceFt - midpoint);

            if (deviation < bestDeviation) {
                bestDeviation = deviation;
                bestAngle     = angle;
                bestRPM       = interpolatedRPM;
                bestToF       = interpolatedToF;
                outOfRange    = thisGroupOutOfRange;
            }
        }

        if (outOfRange) {
            warnings.add(ShooterWarning.DISTANCE_OUT_OF_TABLE_RANGE);
        }

        return new InterpolationResult(bestAngle, bestRPM, bestToF);
    }

    // =========================================================================
    // ToF lookup helper for OnTheMove seeding
    // =========================================================================
    private double lookupToF(String targetType, double distanceFt) {
        Set<ShooterWarning> dummy = EnumSet.noneOf(ShooterWarning.class);
        return interpolate(targetType, distanceFt, dummy).timeOfFlight;
    }

    // =========================================================================
    // Heading bounds helper
    //
    // Returns [minRobotHeading, maxRobotHeading] in degrees (field frame) such
    // that the turret stays within its soft stops while aimed at the target.
    //
    //   turretAngle_robot = aimFieldDeg - robotHeadingDeg
    //   Solving for robotHeadingDeg within [softMin, softMax]:
    //     robotHeadingDeg ∈ [aimFieldDeg - softMax, aimFieldDeg - softMin]
    // =========================================================================
    private double[] computeHeadingBounds(double aimHeadingFieldDeg) {
        return new double[] {
            normalizeAngleDeg(aimHeadingFieldDeg - turretSoftLimitMaxDeg),
            normalizeAngleDeg(aimHeadingFieldDeg - turretSoftLimitMinDeg)
        };
    }

    // =========================================================================
    // Lookup table helpers
    // =========================================================================

    private boolean targetTypeExists(String targetType) {
        for (LookupEntry e : shooterConstants.LOOKUP_TABLE) {
            if (e.targetType.equals(targetType)) return true;
        }
        return false;
    }

    private double[] getUniqueHoodAngles(String targetType) {
        LinkedHashSet<Double> seen = new LinkedHashSet<>();
        for (LookupEntry e : shooterConstants.LOOKUP_TABLE) {
            if (e.targetType.equals(targetType)) seen.add(e.hoodAngle);
        }
        double[] result = new double[seen.size()];
        int i = 0;
        for (double a : seen) result[i++] = a;
        return result;
    }

    private LookupEntry[] getEntriesForAngle(String targetType, double angle) {
        List<LookupEntry> list = new ArrayList<>();
        for (LookupEntry e : shooterConstants.LOOKUP_TABLE) {
            if (e.targetType.equals(targetType) && Double.compare(e.hoodAngle, angle) == 0) {
                list.add(e);
            }
        }
        list.sort((a, b) -> Double.compare(a.distance, b.distance));
        return list.toArray(new LookupEntry[0]);
    }

    // =========================================================================
    // Geometry helpers
    // =========================================================================

    /**
     * Rotates the robot-local pivot offset (from shooterConstants) into the
     * field frame and adds it to the robot center position.
     * Returns [pivotFieldX, pivotFieldY] in feet.
     */
    private static double[] pivotToFieldFrame(
            double robotX, double robotY, double robotHeadingDeg) {

        double offsetX_ft = shooterConstants.PIVOT_OFFSET_X_IN / 12.0;
        double offsetY_ft = shooterConstants.PIVOT_OFFSET_Y_IN / 12.0;
        double headingRad = Math.toRadians(robotHeadingDeg);

        double pivotFieldX = robotX + offsetX_ft * Math.cos(headingRad)
                                    - offsetY_ft * Math.sin(headingRad);
        double pivotFieldY = robotY + offsetX_ft * Math.sin(headingRad)
                                    + offsetY_ft * Math.cos(headingRad);

        return new double[]{ pivotFieldX, pivotFieldY };
    }

    /**
     * Adds TURRET_ANGLE_EXCEEDS_LIMITS to warnings if the commanded turret angle
     * falls outside the soft or hard limit band.
     */
    private void checkTurretLimits(double turretAngleRobotDeg, Set<ShooterWarning> warnings) {
        if (turretAngleRobotDeg < turretHardLimitMinDeg
         || turretAngleRobotDeg > turretHardLimitMaxDeg
         || turretAngleRobotDeg < turretSoftLimitMinDeg
         || turretAngleRobotDeg > turretSoftLimitMaxDeg) {
            warnings.add(ShooterWarning.TURRET_ANGLE_EXCEEDS_LIMITS);
        }
    }

    /**
     * Returns a zeroed ShooterOutput carrying only the provided warnings.
     * Used when a fatal input error prevents calculation.
     */
    private static ShooterOutput emptyOutput(ShotMode mode, Set<ShooterWarning> warnings) {
        return new ShooterOutput(0, 0, mode, 0, 0, 0, 0,
                new double[]{0, 0}, warnings);
    }

    /**
     * Normalizes an angle in degrees to the range (-180°, 180°].
     */
    private static double normalizeAngleDeg(double angleDeg) {
        angleDeg = angleDeg % 360.0;
        if (angleDeg >  180.0) angleDeg -= 360.0;
        if (angleDeg <= -180.0) angleDeg += 360.0;
        return angleDeg;
    }

    // =========================================================================
    // Smoke test — run as a standalone Java main to validate math without a robot
    // =========================================================================
    public static void main(String[] args) {
        // Full constructor used here — no mechanismConfig available in a standalone test.
        // Replace these with your actual turret soft/hard stop angles when characterizing.
        ShooterCalculator calc = new ShooterCalculator(
                50,     // 50 Hz control loop
                2,      // 2 ToF iterations
                -120,   // turret soft limit min (deg, robot-relative)
                 120,   // turret soft limit max
                -135,   // turret hard limit min
                 135);  // turret hard limit max

        // StaticShot: target at field origin, robot at (15, 10) facing 45°
        System.out.println("=== StaticShot ===");
        ShooterOutput staticOut = calc.calculateStaticShot(
                "Hub", 0, 0, 15, 10, 45);
        System.out.println("Result → " + staticOut);

        System.out.println();

        // OnTheMove: same pose, robot moving at 5 ft/s in field +X,
        // slight CCW rotation at 10 deg/s, turret currently at 0° robot-relative
        System.out.println("=== OnTheMove ===");
        ShooterOutput otmOut = calc.calculateOnTheMove(
                "Hub",
                0, 0,       // target
                15, 10, 45, // robot pose
                5, 0,       // linear vel (ft/s)
                0, 0,       // linear accel (ft/s²)
                10, 0,      // angular vel/accel (deg/s, deg/s²)
                0, 0,       // center of rotation — robot-relative (0,0 = robot center)
                0,          // current turret angle (robot-relative)
                5, 0);      // turret angular vel/accel (deg/s, deg/s²)
        System.out.println("Result → " + otmOut);
    }
}
