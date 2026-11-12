package frc.lib.mechanisms.shooter;

/**
 * ShotProfile.java
 * PATH: src/main/java/frc/lib/mechanisms/shooter/ShotProfile.java
 *
 * Robot-specific shooter data handed to ShooterCalculator: the shot lookup
 * table, the shooter pivot location, and the on-the-move speed warning limit.
 * The robot defines one in its constants (e.g. shooterConstants.PROFILE); the
 * calculator itself holds no robot data, so it can be reused and unit-tested
 * with any table.
 *
 * ─── LOOKUP TABLE ────────────────────────────────────────────────────────────
 * Columns: targetType | rpm | hoodAngle | distance (ft) | xExitVel (ft/s) | ToF (s)
 * Rows need not be sorted. Multiple hood-angle groups per target type are
 * supported; the interpolator picks the group whose bracket best fits.
 *
 * ─── PIVOT FRAME ─────────────────────────────────────────────────────────────
 * Robot-local inches: +X = right of robot center, +Y = forward of robot center.
 * All table distances are measured from this point.
 */
public final class ShotProfile {

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

    /** Shot profile rows. */
    public final LookupEntry[] table;
    /** Shooter pivot X offset from robot center (inches, right positive). */
    public final double pivotOffsetXIn;
    /** Shooter pivot Y offset from robot center (inches, forward positive). */
    public final double pivotOffsetYIn;
    /** Robot speed (ft/s) above which ROBOT_SPEED_TOO_HIGH is flagged. */
    public final double robotSpeedWarningFtS;

    public ShotProfile(LookupEntry[] table,
                       double pivotOffsetXIn, double pivotOffsetYIn,
                       double robotSpeedWarningFtS) {
        if (table == null || table.length == 0)
            throw new IllegalArgumentException("ShotProfile: table must not be empty");
        this.table                = table.clone();
        this.pivotOffsetXIn       = pivotOffsetXIn;
        this.pivotOffsetYIn       = pivotOffsetYIn;
        this.robotSpeedWarningFtS = robotSpeedWarningFtS;
    }
}
