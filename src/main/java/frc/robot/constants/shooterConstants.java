package frc.robot.constants;

/**
 * shooterConstants.java
 * PATH: src/main/java/frc/robot/constants/shooterConstants.java
 *
 * Shot profile lookup table and shooter geometry constants.
 *
 * ─── WHAT LIVES HERE ─────────────────────────────────────────────────────────
 * LookupEntry      — data class for one row of the shot profile table
 * LOOKUP_TABLE     — RPM / hood angle / ToF indexed by distance per target type
 * Pivot geometry   — shooter pivot offset in robot frame (inches)
 * Turret limits    — soft and hard stop angles (robot-relative degrees)
 * Warning threshold — robot speed above which ROBOT_SPEED_TOO_HIGH is flagged
 *
 * ─── IN-SEASON TUNING ────────────────────────────────────────────────────────
 * Edit LOOKUP_TABLE entries to tune shot profiles without touching the calculator.
 *
 * Columns: targetType | rpm | hoodAngle | distance (ft) | xExitVel (ft/s) | ToF (s)
 * ToF column: populate with measured values — 0.0 is a placeholder.
 *
 * Multiple hood angle groups per target type are supported. The interpolator
 * picks the group whose bracket midpoint best matches the actual distance.
 *
 * ─── TURRET LIMITS ───────────────────────────────────────────────────────────
 * Robot-relative degrees. 0° = turret aligned with robot forward, CCW positive.
 * Pass TURRET_SOFT/HARD constants into the ShooterCalculator constructor from
 * RobotContainer so they remain overridable for testing.
 */
public final class shooterConstants {

    // =========================================================================
    // LookupEntry — data class for one shot profile row
    // =========================================================================

    /**
     * One row of the shot profile lookup table.
     *
     * @param targetType    String key — must match the targetType passed to
     *                      ShooterCalculator.calculate*() at the call site.
     * @param rpm           Flywheel shaft speed (RPM) at this distance.
     * @param hoodAngle     Hood angle (degrees) at this distance.
     * @param distance      Distance from shooter pivot to target (feet).
     * @param xExitVelocity Horizontal ball exit velocity (ft/s) — for future use.
     * @param timeOfFlight  Ball time-of-flight (seconds). Use 0.0 as a placeholder
     *                      until measured; OnTheMove accuracy requires real values.
     */
    public static class LookupEntry {

        public final String targetType;
        public final double rpm;
        public final double hoodAngle;
        public final double distance;        // feet
        public final double xExitVelocity;   // ft/s — stored for future use
        public final double timeOfFlight;    // seconds (0.0 = placeholder)

        public LookupEntry(
                String targetType,
                double rpm,
                double hoodAngle,
                double distance,
                double xExitVelocity,
                double timeOfFlight) {

            this.targetType    = targetType;
            this.rpm           = rpm;
            this.hoodAngle     = hoodAngle;
            this.distance      = distance;
            this.xExitVelocity = xExitVelocity;
            this.timeOfFlight  = timeOfFlight;
        }
    }

    // =========================================================================
    // Shot Profile Lookup Table
    //
    // Add or remove rows as distances are characterized on the field.
    // Rows need not be sorted — the interpolator handles arbitrary ordering.
    // =========================================================================
    public static final LookupEntry[] LOOKUP_TABLE = {
        new LookupEntry("Hub", 4825, 58.00, 24, 20.04, 0.0),
        new LookupEntry("Hub", 4250, 58.00, 20, 16.67, 0.0),
        new LookupEntry("Hub", 3750, 58.00, 16, 14.24, 0.0),
        new LookupEntry("Hub", 3250, 58.00, 12, 12.48, 0.0),
        new LookupEntry("Hub", 2750, 58.00,  8, 11.34, 0.0),
        new LookupEntry("Hub", 3000, 62.54,  8,  8.42, 0.0),
        new LookupEntry("Hub", 2500, 62.54,  4,  7.83, 0.0),
    };

    // =========================================================================
    // Shooter Pivot Geometry
    //
    // Position of the shooter pivot (and turret center of rotation) in the
    // robot-local frame, in INCHES.
    //   +X = right of robot center
    //   +Y = forward of robot center
    //
    // All LOOKUP_TABLE distances are measured from this point.
    // This point is fixed in the robot frame; turret rotation only changes aim
    // direction — it does not translate the pivot.
    // =========================================================================
    /** Shooter pivot X offset from robot center (inches, right positive). */
    public static final double PIVOT_OFFSET_X_IN =  6.0;   // TODO: measure actual position

    /** Shooter pivot Y offset from robot center (inches, forward positive). */
    public static final double PIVOT_OFFSET_Y_IN = -6.0;   // TODO: measure actual position

    // =========================================================================
    // Robot Speed Warning Threshold
    //
    // Above this speed (ft/s) the ROBOT_SPEED_TOO_HIGH warning is added to
    // ShooterOutput.warnings. Tune to match the reliable compensation envelope.
    // =========================================================================
    public static final double ROBOT_SPEED_WARNING_THRESHOLD_FT_S = 10.0;

}
