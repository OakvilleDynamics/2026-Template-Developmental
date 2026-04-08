package frc.robot.util.motors;

/**
 * motorModels.java
 * PATH: src/main/java/frc/robot/util/motors/motorModels.java
 *
 * Catalog of approved FRC motors with vendor-datasheet specifications.
 * Used by mechanismUnit.FF to derive feed-forward voltages from first principles
 * rather than empirical calibration constants.
 *
 * ─── HOW TO USE ──────────────────────────────────────────────────────────────
 * Pass a MotorModel constant as the first argument to any mechanismUnit.FF factory:
 *
 *   mechanismUnit.FF.rotatingArm(motorModels.NEO, gearRatio, ...)
 *   mechanismUnit.FF.multiStageElevator(motorModels.KRAKEN_X60, gearRatio, ...)
 *
 * ─── HOW TO VERIFY SPECS ─────────────────────────────────────────────────────
 * Before competition, cross-check each constant against the current vendor
 * datasheet. Motor specs occasionally change between manufacturing runs.
 * Source URLs are noted per constant below.
 *
 * ─── ADDING NEW MOTORS ───────────────────────────────────────────────────────
 * Add a new public static final MotorModel field following the pattern below.
 * All spec values must come from the official vendor datasheet, not derived
 * or estimated. If a value is uncertain, leave a // TODO comment.
 *
 * ─── DERIVED VALUES ──────────────────────────────────────────────────────────
 * MotorModel computes the following at construction — no need to supply them:
 *   kT          = stallTorqueNm / stallCurrentA       (N·m per amp)
 *   resistance  = 12.0 / stallCurrentA                (Ω, at 12V nominal)
 *   kE          = 12.0 / (freeSpeedRpm × 2π/60)       (V·s/rad, back-EMF constant)
 *
 * ─── FF VOLTAGE FORMULA ──────────────────────────────────────────────────────
 * For a gravity or spring load requiring torque τ_mech at the mechanism shaft:
 *
 *   V_ff = τ_mech × 12V / (gearRatio × stallTorqueNm)
 *
 * This reduces to τ_mech × R / kT / gearRatio, using the motor's 12V
 * rated stall point. On-controller kV (Tier 1) handles dynamic back-EMF;
 * these factories cover the static/quasi-static load component only.
 *
 * All spec values are at 12V nominal unless otherwise noted.
 */
public final class motorModels {

    // No instances
    private motorModels() {}

    // =========================================================================
    // MotorModel — data class
    // =========================================================================

    /**
     * Specifications for a single motor model.
     * All values at 12V nominal from official vendor datasheets.
     */
    public static final class MotorModel {

        /** Human-readable motor name (e.g. "Kraken X60"). */
        public final String name;

        /** Peak torque at stall (N·m). From vendor datasheet at 12V. */
        public final double stallTorqueNm;

        /** Current draw at stall (A). From vendor datasheet at 12V. */
        public final double stallCurrentA;

        /** No-load speed (RPM). From vendor datasheet at 12V. */
        public final double freeSpeedRpm;

        /** No-load current draw (A). From vendor datasheet at 12V. */
        public final double freeCurrentA;

        // ── Derived ───────────────────────────────────────────────────────────

        /** Torque constant: stallTorqueNm / stallCurrentA  (N·m/A) */
        public final double kT;

        /** Winding resistance: 12.0 / stallCurrentA  (Ω) */
        public final double resistance;

        /** Back-EMF constant: 12.0 / (freeSpeedRpm × 2π/60)  (V·s/rad) */
        public final double kE;

        public MotorModel(String name, double stallTorqueNm, double stallCurrentA,
                          double freeSpeedRpm, double freeCurrentA) {
            this.name          = name;
            this.stallTorqueNm = stallTorqueNm;
            this.stallCurrentA = stallCurrentA;
            this.freeSpeedRpm  = freeSpeedRpm;
            this.freeCurrentA  = freeCurrentA;

            this.kT         = stallTorqueNm / stallCurrentA;
            this.resistance = 12.0 / stallCurrentA;
            this.kE         = 12.0 / (freeSpeedRpm * (2.0 * Math.PI / 60.0));
        }

        @Override
        public String toString() {
            return String.format(
                "%s [stallτ=%.3fN·m, stallI=%.0fA, freeRPM=%.0f, kT=%.5fN·m/A, R=%.4fΩ]",
                name, stallTorqueNm, stallCurrentA, freeSpeedRpm, kT, resistance);
        }
    }

    // =========================================================================
    // CTRE motors
    // =========================================================================

    /**
     * CTRE Kraken X60 (FOC-capable, used in drive and steer on this robot).
     * Source: https://store.ctr-electronics.com/kraken-x60/
     * Verify: stall torque 9.37 N·m, stall current 483 A, free speed 6000 RPM.
     */
    public static final MotorModel KRAKEN_X60 = new MotorModel(
        "Kraken X60", 9.37, 483, 6000, 2.0);

    /**
     * CTRE Kraken X44 (compact form factor, same controller as X60).
     * Source: https://store.ctr-electronics.com/kraken-x44/
     * Verify: stall torque 5.6 N·m, stall current 344 A, free speed 7530 RPM.
     */
    public static final MotorModel KRAKEN_X44 = new MotorModel(
        "Kraken X44", 5.6, 344, 7530, 2.0);

    /**
     * CTRE Minion (used for swerve steer on this robot).
     * Source: https://store.ctr-electronics.com/minion/
     * Verify: stall torque 2.14 N·m, stall current 111 A, free speed 7000 RPM.
     */
    public static final MotorModel MINION = new MotorModel(
        "Minion", 2.14, 111, 7000, 1.4);

    /**
     * CTRE Falcon 500 (legacy — included for brownfield mechanisms).
     * Source: https://store.ctr-electronics.com/falcon-500/
     * Verify: stall torque 4.69 N·m, stall current 257 A, free speed 6380 RPM.
     */
    public static final MotorModel FALCON_500 = new MotorModel(
        "Falcon 500", 4.69, 257, 6380, 1.5);

    // =========================================================================
    // REV Robotics motors
    // =========================================================================

    /**
     * REV NEO (primary mechanism motor; paired with SparkMax or SparkFlex).
     * Source: https://www.revrobotics.com/rev-21-1650/
     * Verify: stall torque 3.36 N·m, stall current 166 A, free speed 5880 RPM.
     */
    public static final MotorModel NEO = new MotorModel(
        "REV NEO", 3.36, 166, 5880, 1.8);

    /**
     * REV NEO 550 (small/lightweight mechanisms + UltraPlanetary gearbox).
     * Permanent exception — no Nova equivalent for this weight class.
     * Source: https://www.revrobotics.com/rev-21-1651/
     * Verify: stall torque 0.97 N·m, stall current 100 A, free speed 11000 RPM.
     */
    public static final MotorModel NEO_550 = new MotorModel(
        "REV NEO 550", 0.97, 100, 11000, 1.4);

    /**
     * REV NEO Vortex (paired with SparkFlex).
     * Source: https://www.revrobotics.com/rev-21-1652/
     * Verify: stall torque 3.60 N·m, stall current 211 A, free speed 6784 RPM.
     */
    public static final MotorModel NEO_VORTEX = new MotorModel(
        "REV NEO Vortex", 3.60, 211, 6784, 3.6);

    // =========================================================================
    // ThriftyBot motors
    // =========================================================================

    /**
     * ThriftyBot Pulsar 775 (go-forward mechanism motor; paired with Nova controller).
     * Source: https://www.thethriftybot.com/products/pulsar-775
     * Verify: stall torque 3.1 N·m, stall current 189 A, free speed 7500 RPM,
     *         free current 2.7 A.
     */
    public static final MotorModel PULSAR_775 = new MotorModel(
        "ThriftyBot Pulsar 775", 3.1, 189, 7500, 2.7);
}
