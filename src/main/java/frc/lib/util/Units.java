package frc.lib.util;

/**
 * Units.java
 * PATH: src/main/java/frc/lib/util/Units.java
 *
 * Single source of truth for all unit conversions in this codebase.
 * All methods are static — no instantiation needed.
 *
 * ─── NAMING CONVENTION ───────────────────────────────────────────────────────
 * fromUnitToUnit(value):
 *   inchesToMeters()    — inches → meters
 *   feetToMeters()      — feet → meters
 *   metersToInches()    — meters → inches
 *   metersToFeet()      — meters → feet
 *   ftpsToMps()         — feet/s → meters/s
 *   mpsToFtps()         — meters/s → feet/s
 *   ftps2ToMps2()       — ft/s² → m/s²
 *   mps2ToFtps2()       — m/s² → ft/s²
 *   degToRad()          — degrees → radians
 *   radToDeg()          — radians → degrees
 *   lbsToKg()           — pounds → kilograms
 *   lbIn2ToKgM2()       — lb·in² → kg·m²
 *   lbInToNm()          — lb·in → N·m
 *   fieldFeetToMeters() — field coordinate in feet → meters (same math as
 *                         feetToMeters, named distinctly so call sites are
 *                         self-documenting)
 *
 * ─── WHERE CONVERSIONS HAPPEN ────────────────────────────────────────────────
 * Conversion occurs ONLY at entry points — never inside control loops:
 *   DriveInput constructor    (ft/s → m/s)
 *   SwerveDrive constructor   (inches → meters)
 *   drivePointAt()            (feet → meters for field target)
 *
 * WPILib, PathPlanner, Choreo, and PhotonVision all operate in meters.
 * Do not call these conversions anywhere inside the kinematics or PID math.
 */
public final class Units {

    private Units() {}

    // ── Length ────────────────────────────────────────────────────────────────

    public static double inchesToMeters(double inches)         { return inches * 0.0254; }
    public static double feetToMeters(double feet)             { return feet * 0.3048; }
    public static double metersToInches(double meters)         { return meters / 0.0254; }
    public static double metersToFeet(double meters)           { return meters / 0.3048; }

    /**
     * Field coordinate in feet → meters.
     * Same math as feetToMeters() but named to make call sites self-documenting.
     * Use when converting field target positions for drivePointAt().
     */
    public static double fieldFeetToMeters(double feet)        { return feet * 0.3048; }

    // ── Velocity ──────────────────────────────────────────────────────────────

    public static double ftpsToMps(double ftps)                { return ftps * 0.3048; }
    public static double mpsToFtps(double mps)                 { return mps / 0.3048; }

    // ── Acceleration ──────────────────────────────────────────────────────────

    public static double ftps2ToMps2(double ftps2)             { return ftps2 * 0.3048; }
    public static double mps2ToFtps2(double mps2)              { return mps2 / 0.3048; }

    // ── Angle ─────────────────────────────────────────────────────────────────

    public static double degToRad(double degrees)              { return Math.toRadians(degrees); }
    public static double radToDeg(double radians)              { return Math.toDegrees(radians); }

    /**
     * Normalizes an angle in degrees to the range (-180°, 180°].
     * Used for wrap-aware angular arithmetic throughout the shooter aim system.
     */
    public static double normalizeAngleDeg(double angleDeg) {
        angleDeg = angleDeg % 360.0;
        if (angleDeg >  180.0) angleDeg -= 360.0;
        if (angleDeg <= -180.0) angleDeg += 360.0;
        return angleDeg;
    }

    // ── Mass ──────────────────────────────────────────────────────────────────

    public static double lbsToKg(double lbs)                   { return lbs * 0.453592; }

    // ── Moment of inertia ─────────────────────────────────────────────────────

    public static double lbIn2ToKgM2(double lbIn2)             { return lbIn2 * 2.92639e-4; }

    // ── Torque ────────────────────────────────────────────────────────────────

    /** Pound-inches → Newton-meters. Used for spring/mechanism FF calibration. */
    public static double lbInToNm(double lbIn)                 { return lbIn * 0.112985; }
}