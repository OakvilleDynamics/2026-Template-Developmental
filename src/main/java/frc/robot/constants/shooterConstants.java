package frc.robot.constants;

import frc.lib.mechanisms.shooter.ShotProfile;
import frc.lib.mechanisms.shooter.ShotProfile.LookupEntry;

/**
 * shooterConstants.java
 * PATH: src/main/java/frc/robot/constants/shooterConstants.java
 *
 * Shot profile lookup table and shooter geometry constants for THIS robot.
 * Bundled into PROFILE and passed to ShooterCalculator from RobotContainer.
 *
 * ─── WHAT LIVES HERE ─────────────────────────────────────────────────────────
 * LOOKUP_TABLE     — RPM / hood angle / ToF indexed by distance per target type
 * Pivot geometry   — shooter pivot offset in robot frame (inches)
 * Warning threshold — robot speed above which ROBOT_SPEED_TOO_HIGH is flagged
 * PROFILE          — the three above as one ShotProfile
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
 * Turret soft limits live in the turret's mechanismConfig (single source of
 * truth); ShooterCalculator reads them from there.
 */
public final class shooterConstants {

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

    /** Everything above, bundled for ShooterCalculator. */
    public static final ShotProfile PROFILE = new ShotProfile(
        LOOKUP_TABLE, PIVOT_OFFSET_X_IN, PIVOT_OFFSET_Y_IN, ROBOT_SPEED_WARNING_THRESHOLD_FT_S);

    private shooterConstants() {}
}
