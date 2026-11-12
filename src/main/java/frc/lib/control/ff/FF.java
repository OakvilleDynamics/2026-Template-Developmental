package frc.lib.control.ff;

import java.util.function.Supplier;

import frc.lib.hardware.motor.motorModels;
import frc.lib.swerve.driveOdometryState;
import frc.lib.util.units;

/**
 * FF.java
 * PATH: src/main/java/frc/lib/control/ff/FF.java
 *
 * Static factory library for physics-based ffProvider lambdas.
 * (Formerly mechanismUnit.FF — pulled out so it can be unit-tested and reused
 * without constructing any motor hardware.)
 *
 * All voltage calculations are derived from motor datasheet constants
 * (via motorModels) rather than empirical calibration voltages, so
 * values are grounded in vendor-verified specs and require no per-mechanism
 * holding-voltage measurement.
 *
 * ─── VOLTAGE FORMULA ─────────────────────────────────────────────────────────
 * For a load requiring torque τ_mech at the mechanism shaft:
 *
 *   V_ff = τ_mech × 12V / (gearRatio × motor.stallTorqueNm)
 *
 * On-controller kV (Tier 1) already handles dynamic back-EMF; these
 * factories target the static/quasi-static gravity and spring loads only.
 *
 * ─── UNITS AT THE INTERFACE ──────────────────────────────────────────────────
 * Masses    : lbs  (converted to kg internally)
 * Distances : inches  (converted to meters internally)
 * Angles    : degrees  (matches ffProvider positionDeg convention)
 * Returns   : volts
 *
 * ─── TIER PLACEMENT ──────────────────────────────────────────────────────────
 * Tier 2 (self-contained): springTurret, rotatingArm
 * Tier 3 (cross-system):   gyroscopicTurret, multiStageElevator, pivotingElevator
 *   Tier-3 factories accept Supplier arguments that close over other
 *   subsystem references; build these in RobotContainer.
 */
public final class FF {

    private static final double G_MPS2     = 9.80665;
    private static final double V_NOMINAL  = 12.0;
    /**
     * Velocity below which the mechanism is considered "at rest" for friction
     * offset direction selection. Prevents the friction addend from flipping sign
     * due to encoder noise when the mechanism is holding position.
     * Units: mechanism shaft rotations per second.
     */
    static final double FRICTION_DEADBAND_RPS = 0.05;

    // No instances
    private FF() {}

    /** Provider that always returns zero — flat mechanisms or no Rio-side FF. */
    public static ffProvider none() {
        return (pos, vel, accel) -> 0.0;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Shared helper
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Convert torque at the mechanism shaft to a feed-forward voltage.
     * V = τ_mech × V_nominal / (gearRatio × stallTorqueNm)
     */
    static double torqueToVolts(double torqueNm, double gearRatio,
                                motorModels.MotorModel motor) {
        return torqueNm * V_NOMINAL / (gearRatio * motor.stallTorqueNm);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // TIER 2 — Self-contained (no external subsystem references)
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Piecewise-linear feed-forward for a mechanism with a non-linear
     * restoring force (constant-force spring, surgical tubing, gas spring).
     *
     * Calibration points are measured torques at the mechanism shaft in lb·in,
     * for the forward direction of rotation. When the motor rotates in reverse,
     * the torque values are negated automatically — a spring that assists forward
     * motion at a given angle inherently resists the same reverse motion by the
     * same magnitude.
     *
     * Between calibration points: linear interpolation.
     * Outside the calibrated angle range: returns 0.0 (intentionally conspicuous —
     * the PID will visibly fight the spring, flagging the gap during tuning).
     * Within FRICTION_DEADBAND_RPS of zero velocity: applies forward-direction
     * convention to prevent sign-flip jitter while holding position.
     *
     * Calibration procedure: hold a fish scale at a known distance from the pivot
     * at each angle stop and record force_lbs × distance_in as torqueLbIn.
     * Positive torque = spring assists the forward-rotation direction at that angle.
     *
     * Internally converts lb·in → N·m via units.lbIn_Nm(), then:
     *   V = τ × 12V / (gearRatio × motor.stallTorqueNm)
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
                "FF.springTurret: requires at least 2 calibration points");
        for (int i = 0; i < points.length; i++) {
            if (points[i].length != 2)
                throw new IllegalArgumentException(
                    "FF.springTurret: each point must be double[]{ angleDeg, torqueLbIn }");
            if (i > 0 && points[i][0] <= points[i - 1][0])
                throw new IllegalArgumentException(
                    "FF.springTurret: points must be sorted ascending by angleDeg. "
                    + "Found " + points[i][0] + " at index " + i + " after " + points[i - 1][0]);
        }

        // Convert torques to N·m and defensive-copy angles — avoid repeated math per cycle
        final double[] angles    = new double[points.length];
        final double[] torquesNm = new double[points.length];
        for (int i = 0; i < points.length; i++) {
            angles[i]    = points[i][0];
            torquesNm[i] = units.lbIn_Nm(points[i][1]);
        }

        return (positionDeg, velocityRps, accelRpss) -> {
            // Out of range → 0 intentionally (see javadoc)
            if (positionDeg < angles[0] || positionDeg > angles[angles.length - 1]) return 0.0;

            // Binary search for the containing bracket, then linear interpolation
            int lo = 0, hi = angles.length - 2;
            while (lo < hi) {
                int mid = (lo + hi + 1) / 2;
                if (angles[mid] <= positionDeg) lo = mid; else hi = mid - 1;
            }
            double t        = (positionDeg - angles[lo]) / (angles[lo + 1] - angles[lo]);
            double torqueNm = torquesNm[lo] + t * (torquesNm[lo + 1] - torquesNm[lo]);

            // Reverse rotation: negate — spring that assists forward inherently
            // resists the same motion in reverse. Within the deadband, hold the
            // forward convention to prevent jitter while holding position.
            if (velocityRps < -FRICTION_DEADBAND_RPS) torqueNm = -torqueNm;

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
                "FF.rotatingArm: gamePieceTypesLbs, pieceCgDistancesIn, "
                + "and pieceCountSuppliers must all be the same length.");

        final double armKg   = units.lbs_kg(armMassLbs);
        final double armCgM  = units.inches_m(armCgDistanceIn);
        final double[] pieceKg  = new double[gamePieceTypesLbs.length];
        final double[] pieceCgM = new double[pieceCgDistancesIn.length];
        for (int i = 0; i < pieceKg.length; i++) {
            pieceKg[i]  = units.lbs_kg(gamePieceTypesLbs[i]);
            pieceCgM[i] = units.inches_m(pieceCgDistancesIn[i]);
        }
        final Supplier<Integer>[] counts = pieceCountSuppliers;

        return (positionDeg, velocityRps, accelRpss) -> {
            // Effective moment arm = mass × CG distance, summed for all contributors
            double momentKgM = armKg * armCgM;
            for (int i = 0; i < pieceKg.length; i++) {
                momentKgM += counts[i].get() * pieceKg[i] * pieceCgM[i];
            }
            // cos(positionDeg) produces the correct signed output automatically:
            //   0°      → cos = +1.0 → positive volts (arm horizontal, full gravity load)
            //   90°     → cos =  0.0 → zero volts (arm vertical, no gravity moment)
            //   90–180° → cos negative → negative volts (arm past vertical; gravity now
            //              pulls the other way, so FF must push back the other way too)
            double torqueNm = momentKgM * G_MPS2 * Math.cos(Math.toRadians(positionDeg));
            return torqueToVolts(torqueNm, gearRatio, motor);
        };
    }

    // ─────────────────────────────────────────────────────────────────────────
    // TIER 3 — Cross-system (require external subsystem references)
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Gyroscopic resistance compensation for a turret that rotates a spinning flywheel.
     *
     * A spinning flywheel has angular momentum (L = I × ω). When the turret rotates,
     * it changes the direction of that angular momentum vector, which requires a torque
     * (the gyroscopic reaction torque). Without FF compensation, the turret PID must
     * fight this load reactively, causing heading lag proportional to flywheel speed.
     *
     * Physics:
     *   τ_gyro = I_flywheel × ω_flywheel × ω_turret
     *   V_ff   = τ_gyro × 12V / (turretGearRatio × turretMotor.stallTorqueNm)
     *
     * Sign is self-consistent: positive turret velocity with positive flywheel spin
     * produces positive τ (FF assists through the resistance). If flywheel direction
     * reverses, FF sign flips automatically.
     *
     * I_flywheel is a fixed physical property of the flywheel assembly — provide it
     * once at construction from a CAD model or measurement. Flywheel angular velocity
     * is read each cycle via the supplier.
     *
     * This is Tier 3 because it closes over the flywheel mechanismUnit's velocity.
     * Wire it in RobotContainer where both mechanism references exist.
     *
     * @param turretMotor              motor model for the turret (e.g. motorModels.MINION)
     * @param turretGearRatio          turret motor rotations per turret shaft rotation
     * @param flywheelMoiLbIn2         flywheel moment of inertia (lb·in²) — static, measured once
     * @param flywheelVelocityRpsSupplier  supplies flywheel shaft RPS each cycle
     */
    public static ffProvider gyroscopicTurret(
            motorModels.MotorModel turretMotor,
            double turretGearRatio,
            double flywheelMoiLbIn2,
            Supplier<Double> flywheelVelocityRpsSupplier) {

        final double flywheelMoiKgM2 = units.lbIn2_kgM2(flywheelMoiLbIn2);

        return (positionDeg, velocityRps, accelRpss) -> {
            double omegaFlywheel = flywheelVelocityRpsSupplier.get() * (2.0 * Math.PI);
            double omegaTurret   = velocityRps * (2.0 * Math.PI);
            double torqueNm      = flywheelMoiKgM2 * omegaFlywheel * omegaTurret;
            return torqueToVolts(torqueNm, turretGearRatio, turretMotor);
        };
    }

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
                "FF.multiStageElevator: gamePieceTypesLbs and "
                + "pieceCountSuppliers must be the same length.");

        final double spoolM     = units.inches_m(spoolRadiusIn);
        final double carriageKg = units.lbs_kg(carriageMassLbs);
        final double[] stageKg  = new double[stageMassesLbs.length];
        for (int i = 0; i < stageKg.length; i++) stageKg[i] = units.lbs_kg(stageMassesLbs[i]);
        final double[] pieceKg  = new double[gamePieceTypesLbs.length];
        for (int i = 0; i < pieceKg.length; i++) pieceKg[i] = units.lbs_kg(gamePieceTypesLbs[i]);
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
            // Deadband prevents the friction addend from flipping due to encoder
            // noise while holding. Within ±FRICTION_DEADBAND_RPS, treat as holding
            // and apply the up-direction bias.
            double friction = velocityRps < -FRICTION_DEADBAND_RPS
                              ? frictionOffsetVoltsDown : frictionOffsetVoltsUp;
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
                "FF.pivotingElevator: gamePieceTypesLbs and "
                + "pieceCountSuppliers must be the same length.");

        final double cgDistM       = units.inches_m(cgDistanceFromPivotIn);
        final double cgOffsetM     = units.inches_m(cgOffsetFromRobotCenterIn);
        final double carriageKg    = units.lbs_kg(carriageMassLbs);
        final double[] stageKg     = new double[stageMassesLbs.length];
        for (int i = 0; i < stageKg.length; i++) stageKg[i] = units.lbs_kg(stageMassesLbs[i]);
        final double[] pieceKg     = new double[gamePieceTypesLbs.length];
        for (int i = 0; i < pieceKg.length; i++) pieceKg[i] = units.lbs_kg(gamePieceTypesLbs[i]);
        final Supplier<Integer>[] counts = pieceCountSuppliers;

        return (positionDeg, velocityRps, accelRpss) -> {
            // ── Total mass this cycle ─────────────────────────────────────
            double totalKg = carriageKg;
            for (double s : stageKg) totalKg += s;
            for (int i = 0; i < pieceKg.length; i++) totalKg += pieceKg[i] * counts[i].get();

            double pivotRad = Math.toRadians(positionDeg);

            // ── 1. Gravity ────────────────────────────────────────────────
            // cos(pivotRad) produces the correct signed output automatically:
            //   0°      → cos = +1.0 → positive volts (horizontal, full gravity load)
            //   90°     → cos =  0.0 → zero volts (vertical, no gravity moment)
            //   90–180° → cos negative → negative volts (past vertical; gravity
            //              now pulls the other way, FF must oppose it the other way)
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

            // Deadband prevents the friction addend from flipping due to encoder
            // noise while holding. Within ±FRICTION_DEADBAND_RPS, treat as holding
            // and apply the up-direction bias.
            double friction = velocityRps < -FRICTION_DEADBAND_RPS
                              ? frictionOffsetVoltsDown : frictionOffsetVoltsUp;
            return voltsGrav + voltsInertia + voltsCentripetal + friction;
        };
    }
}
