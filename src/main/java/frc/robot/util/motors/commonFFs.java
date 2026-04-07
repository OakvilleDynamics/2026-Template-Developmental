package frc.robot.util.motors;

import java.util.function.Supplier;

import frc.robot.subsystems.swerveDrive.driveOdometryState;

/**
 * commonFFs.java
 * PATH: src/main/java/frc/robot/util/motors/commonFFs.java
 *
 * Library of static factory methods that return ffProvider lambdas for common
 * mechanism physics. Organized by tier:
 *
 *   Tier 1 — On-controller (kS/kV/kA in mechanismConfig.pid[]). Not here.
 *   Tier 2 — Self-contained: physics using only this mechanism's own state.
 *   Tier 3 — Cross-system: physics that depend on other subsystems.
 *
 * ─── USAGE ───────────────────────────────────────────────────────────────────
 * Tier-2 factories return an ffProvider directly and belong in the mechanism's
 * mechanismConfig:
 *
 *   .withTier2FF(commonFFs.springTurret(points))
 *
 * Tier-3 factories take Supplier/lambda arguments that close over subsystem
 * references and must be called from RobotContainer where those references exist:
 *
 *   .withTier3FF(commonFFs.pivotingElevator(
 *       carriageMassLbs, stageMassesLbs,
 *       cgHeightMeters, cgOffsetMeters,
 *       drive::getOdometryState,
 *       () -> new double[]{ intake.getPieceCount(), shooter.getPieceCount() }
 *   ))
 *
 * ─── UNIT CONTRACT ───────────────────────────────────────────────────────────
 * All masses passed to these factories: lbs (converted to kg internally).
 * positionDeg: mechanism shaft degrees (after gear ratio), matches ffProvider.
 * Returns: volts (added to motor command each cycle).
 *
 * ─── THREAD SAFETY ───────────────────────────────────────────────────────────
 * All lambdas are called on the robot periodic thread. No synchronization needed.
 */
public final class commonFFs {

    private static final double LBS_TO_KG  = 0.453592;
    private static final double G_MPS2     = 9.80665;

    // No instances
    private commonFFs() {}

    // =========================================================================
    // TIER 2 — Self-contained (no external subsystem references)
    // =========================================================================

    /**
     * Piecewise-linear spring torque compensation for a turret or any rotary
     * mechanism with a non-linear restoring force (e.g. constant-force spring,
     * surgical tubing, gas spring).
     *
     * The user provides N calibration points as (angleDeg, holdingVolts) pairs,
     * measured by holding the mechanism at each angle and reading the voltage
     * required to maintain that position. Points must be provided in ascending
     * angle order.
     *
     * Between points: linear interpolation.
     * Outside the calibrated range: returns 0.0 — if this happens in practice,
     * it means the mechanism is operating outside its expected travel and the
     * behavior is intentionally conspicuous (the PID will fight the spring
     * visibly, making the gap easy to spot during tuning).
     *
     * @param points  Calibration points: each element is double[]{ angleDeg, holdingVolts }.
     *                Minimum 2 points. Must be sorted ascending by angleDeg.
     *                Example: new double[][]{ {0, 0.4}, {45, 0.25}, {90, 0.0}, {135, -0.3} }
     * @return ffProvider lambda performing piecewise-linear interpolation
     * @throws IllegalArgumentException if fewer than 2 points are provided or
     *                                  points are not sorted ascending
     */
    public static ffProvider springTurret(double[][] points) {
        if (points == null || points.length < 2)
            throw new IllegalArgumentException(
                "commonFFs.springTurret: requires at least 2 calibration points");
        for (int i = 0; i < points.length; i++) {
            if (points[i].length != 2)
                throw new IllegalArgumentException(
                    "commonFFs.springTurret: each point must be double[]{ angleDeg, holdingVolts }");
            if (i > 0 && points[i][0] <= points[i - 1][0])
                throw new IllegalArgumentException(
                    "commonFFs.springTurret: points must be sorted ascending by angleDeg. "
                    + "Found angle " + points[i][0] + " at index " + i
                    + " after " + points[i - 1][0]);
        }

        // Defensive copy so the caller can't mutate after construction
        final double[][] pts = new double[points.length][2];
        for (int i = 0; i < points.length; i++) {
            pts[i][0] = points[i][0];
            pts[i][1] = points[i][1];
        }

        return (positionDeg, velocityRps, accelRpss) -> {
            // Out of range — return 0 intentionally (see javadoc)
            if (positionDeg < pts[0][0] || positionDeg > pts[pts.length - 1][0]) {
                return 0.0;
            }

            // Binary search for the bracket containing positionDeg
            int lo = 0;
            int hi = pts.length - 2;
            while (lo < hi) {
                int mid = (lo + hi + 1) / 2;
                if (pts[mid][0] <= positionDeg) lo = mid;
                else hi = mid - 1;
            }

            // Linear interpolation within the bracket [lo, lo+1]
            double angleLo   = pts[lo][0];
            double angleHi   = pts[lo + 1][0];
            double voltLo    = pts[lo][1];
            double voltHi    = pts[lo + 1][1];
            double t = (positionDeg - angleLo) / (angleHi - angleLo);
            return voltLo + t * (voltHi - voltLo);
        };
    }

    /**
     * Gravity compensation for a multi-stage linear elevator (vertical travel).
     * Handles a carriage, N independent stages, and N game piece types — each
     * with their own mass and a runtime quantity supplier.
     *
     * This is a Tier-2 FF because it does not depend on other subsystems — the
     * game piece counts are passed in as Suppliers that close over whatever sensor
     * or tracker the user provides (beam breaks, indexer count, etc.).
     *
     * ─── PHYSICS ─────────────────────────────────────────────────────────────
     * F_gravity = g × (carriageMass + Σ stageMasses + Σ pieceMass_i × pieceCount_i)
     * V_ff = F_gravity / (motorKt × gearRatio / wheelRadius)
     *
     * Since the "motor force constant" is opaque and varies by vendor, this
     * factory works in calibrated volts-per-kg directly:
     *
     *   voltsPerKg is measured by loading a known mass and reading the holding
     *   voltage from SmartDashboard. Typical starting point: 12V / liftCapacityKg.
     *
     * Direction-awareness: upward travel (vel >= 0) uses voltsPerKgUp; downward
     * uses voltsPerKgDown. For symmetric mechanisms, pass the same value for both.
     * The asymmetry handles friction and counterweighted stages.
     *
     * @param carriageMassLbs    mass of the elevator carriage (lbs)
     * @param stageMassesLbs     mass of each additional stage (lbs). Empty array if single-stage.
     * @param voltsPerKgUp       calibrated volts needed per kg of payload, moving up or holding
     * @param voltsPerKgDown     calibrated volts needed per kg of payload, moving down
     * @param gamePieceTypesLbs  mass of each game piece type (lbs). Length = N types.
     * @param pieceCountSuppliers runtime piece count for each type, parallel to gamePieceTypesLbs.
     *                           Each supplier returns the integer count of that piece type held.
     * @return ffProvider lambda computing gravity compensation each cycle
     * @throws IllegalArgumentException if gamePieceTypesLbs and pieceCountSuppliers differ in length
     */
    @SuppressWarnings("unchecked") // new Supplier[0] is safe — only ever read as Supplier<Integer>
    public static ffProvider multiStageElevator(
            double carriageMassLbs,
            double[] stageMassesLbs,
            double voltsPerKgUp,
            double voltsPerKgDown,
            double[] gamePieceTypesLbs,
            Supplier<Integer>[] pieceCountSuppliers) {

        if (gamePieceTypesLbs == null) gamePieceTypesLbs = new double[0];
        if (pieceCountSuppliers == null) pieceCountSuppliers = new Supplier[0];
        if (gamePieceTypesLbs.length != pieceCountSuppliers.length)
            throw new IllegalArgumentException(
                "commonFFs.multiStageElevator: gamePieceTypesLbs and pieceCountSuppliers "
                + "must be the same length (one entry per game piece type). "
                + "Got " + gamePieceTypesLbs.length + " mass entries and "
                + pieceCountSuppliers.length + " count suppliers.");

        // Pre-convert fixed masses to kg — avoid repeated multiplication per cycle
        final double carriageKg = carriageMassLbs * LBS_TO_KG;
        final double[] stageKg  = new double[stageMassesLbs == null ? 0 : stageMassesLbs.length];
        for (int i = 0; i < stageKg.length; i++) stageKg[i] = stageMassesLbs[i] * LBS_TO_KG;
        final double[] pieceKg  = new double[gamePieceTypesLbs.length];
        for (int i = 0; i < pieceKg.length; i++) pieceKg[i] = gamePieceTypesLbs[i] * LBS_TO_KG;

        // Capture final refs for lambda
        final Supplier<Integer>[] countSuppliers = pieceCountSuppliers;

        return (positionDeg, velocityRps, accelRpss) -> {
            // Sum all fixed masses
            double totalKg = carriageKg;
            for (double s : stageKg) totalKg += s;

            // Add game piece contribution (runtime query each cycle)
            for (int i = 0; i < pieceKg.length; i++) {
                totalKg += pieceKg[i] * countSuppliers[i].get();
            }

            // Direction-aware voltage
            double voltsPerKg = velocityRps >= 0 ? voltsPerKgUp : voltsPerKgDown;
            return totalKg * voltsPerKg;
        };
    }

    // =========================================================================
    // TIER 3 — Cross-system (require external subsystem references)
    // =========================================================================

    /**
     * Gravity + drivetrain inertia compensation for an elevator mounted on a
     * pivot at its base — a "pivoting elevator" or "four-bar" style arm where
     * the entire elevator assembly rotates about a base pivot.
     *
     * positionDeg is the angle of the pivot (0° = horizontal, 90° = vertical).
     *
     * ─── GRAVITY COMPONENT ───────────────────────────────────────────────────
     * The effective gravity load on the motor varies with cos(pivotAngle):
     *   F_grav = totalMass × g × cos(pivotDeg) × (cgDistanceFromPivotM)
     * Converted to volts via the same voltsPerKg calibration as multiStageElevator.
     *
     * ─── DRIVETRAIN INERTIA COMPONENT ────────────────────────────────────────
     * When the robot accelerates forward/backward, the elevator assembly
     * experiences a pseudo-force equal to mass × robotAccel in the robot frame.
     * This projects onto the elevator's load axis as:
     *   F_inertia = totalMass × robotAccelMps2 × sin(pivotDeg)
     *   (sin because forward accel tilts the "felt gravity" rearward)
     *
     * Only the forward/backward (X) component of robot acceleration is used,
     * consistent with "front-to-back rotation only" pivot geometry.
     *
     * ─── CENTRIPETAL COMPONENT ───────────────────────────────────────────────
     * When the robot rotates, the elevator CG traces an arc and experiences
     * centripetal acceleration directed inward toward the robot center:
     *   a_centripetal = ω² × r_CG
     * where r_CG is the distance from the robot's center of rotation to the
     * elevator assembly CG (approximately cgOffsetFromCenterM). This projects
     * similarly to the inertia term.
     *
     * ─── DRIVETRAIN STATE ────────────────────────────────────────────────────
     * Robot acceleration is read from driveOdometryState.blendedState each cycle
     * via the odometryStateSupplier. The blended state combines encoder (linear)
     * and IMU (angular) — best general-purpose source for both components.
     *
     * To extract forward acceleration from the blended state's polar representation:
     *   accel_forward = linearAccelMagnitude × cos(accelHeading - robotHeading)
     * Robot heading is tracked via the same odometry state angular integration.
     * For simplicity, we project using the acceleration vector directly against
     * the robot's forward axis: requires robotHeadingSupplier.
     *
     * @param carriageMassLbs        elevator carriage mass (lbs)
     * @param stageMassesLbs         additional stage masses (lbs). Empty array if single-stage.
     * @param voltsPerKgUp           calibrated volts/kg, upward or holding
     * @param voltsPerKgDown         calibrated volts/kg, downward
     * @param cgDistanceFromPivotM   distance from base pivot to assembly CG (meters)
     * @param cgOffsetFromCenterM    distance from robot center to elevator CG projected
     *                               onto the rotation plane (meters) — used for centripetal term
     * @param gamePieceTypesLbs      mass of each game piece type (lbs)
     * @param pieceCountSuppliers    runtime piece count per type, parallel to gamePieceTypesLbs
     * @param odometryStateSupplier  supplies driveOdometryState each cycle
     * @param robotHeadingSupplier   supplies robot heading in radians (field-relative, CCW+)
     *                               — used to project field-frame acceleration into robot frame
     * @return ffProvider lambda computing combined gravity + drivetrain FF each cycle
     */
    @SuppressWarnings("unchecked") // new Supplier[0] is safe — only ever read as Supplier<Integer>
    public static ffProvider pivotingElevator(
            double carriageMassLbs,
            double[] stageMassesLbs,
            double voltsPerKgUp,
            double voltsPerKgDown,
            double cgDistanceFromPivotM,
            double cgOffsetFromCenterM,
            double[] gamePieceTypesLbs,
            Supplier<Integer>[] pieceCountSuppliers,
            Supplier<driveOdometryState> odometryStateSupplier,
            Supplier<Double> robotHeadingSupplier) {

        if (gamePieceTypesLbs == null) gamePieceTypesLbs = new double[0];
        if (pieceCountSuppliers == null) pieceCountSuppliers = new Supplier[0];
        if (gamePieceTypesLbs.length != pieceCountSuppliers.length)
            throw new IllegalArgumentException(
                "commonFFs.pivotingElevator: gamePieceTypesLbs and pieceCountSuppliers "
                + "must be the same length. Got " + gamePieceTypesLbs.length
                + " mass entries and " + pieceCountSuppliers.length + " count suppliers.");

        final double carriageKg = carriageMassLbs * LBS_TO_KG;
        final double[] stageKg  = new double[stageMassesLbs == null ? 0 : stageMassesLbs.length];
        for (int i = 0; i < stageKg.length; i++) stageKg[i] = stageMassesLbs[i] * LBS_TO_KG;
        final double[] pieceKg  = new double[gamePieceTypesLbs.length];
        for (int i = 0; i < pieceKg.length; i++) pieceKg[i] = gamePieceTypesLbs[i] * LBS_TO_KG;

        final Supplier<Integer>[] countSuppliers = pieceCountSuppliers;

        return (positionDeg, velocityRps, accelRpss) -> {
            // ── Total mass this cycle ─────────────────────────────────────────
            double totalKg = carriageKg;
            for (double s : stageKg) totalKg += s;
            for (int i = 0; i < pieceKg.length; i++) {
                totalKg += pieceKg[i] * countSuppliers[i].get();
            }

            double pivotRad = Math.toRadians(positionDeg);

            // ── Gravity component ─────────────────────────────────────────────
            // Effective load is max at 0° (horizontal), zero at 90° (vertical)
            double gravForce   = totalKg * G_MPS2 * Math.cos(pivotRad);
            double voltsPerKg  = velocityRps >= 0 ? voltsPerKgUp : voltsPerKgDown;
            double gravVolts   = gravForce * cgDistanceFromPivotM * (voltsPerKg / (G_MPS2 * cgDistanceFromPivotM));
            // Simplified: volts = totalKg × voltsPerKg × cos(pivotRad)
            gravVolts = totalKg * voltsPerKg * Math.cos(pivotRad);

            // ── Drivetrain inertia component ──────────────────────────────────
            // Project field-frame linear acceleration onto robot forward axis
            driveOdometryState odom   = odometryStateSupplier.get();
            double robotHeading       = robotHeadingSupplier.get();
            double accelMag           = odom.blendedState.linearAccelerationMagnitude;
            double accelHeading       = odom.blendedState.linearAccelerationHeading;

            // Component of acceleration in the robot's forward direction
            double accelForwardMps2   = accelMag * Math.cos(accelHeading - robotHeading);

            // Forward accel creates a rearward pseudo-force in the robot frame.
            // This projects onto the elevator load axis via sin(pivotAngle):
            //   0° pivot (horizontal) → full inertial load (elevator is like a pendulum)
            //   90° pivot (vertical)  → no inertial load (acceleration is along the axis)
            double inertiaForce       = totalKg * accelForwardMps2 * Math.sin(pivotRad);
            double inertiaVolts       = inertiaForce * (voltsPerKg / G_MPS2);

            // ── Centripetal component ─────────────────────────────────────────
            // ω² × r_CG directed inward — projects onto elevator axis via sin(pivotAngle)
            double omega              = odom.blendedState.angularVelocity;
            double centripetalAccel   = omega * omega * cgOffsetFromCenterM;
            double centripetalForce   = totalKg * centripetalAccel * Math.sin(pivotRad);
            double centripetalVolts   = centripetalForce * (voltsPerKg / G_MPS2);

            return gravVolts + inertiaVolts + centripetalVolts;
        };
    }
}
