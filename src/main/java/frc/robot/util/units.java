package frc.robot.util;

/**
 * units.java
 * PATH: src/main/java/frc/robot/util/units.java
 *
 * Single source of truth for all unit conversions in this codebase.
 * All methods are static — no instantiation needed.
 *
 * ─── NAMING CONVENTION ───────────────────────────────────────────────────────
 * fromUnit_toUnit(value):
 *   inches_m()    — inches → meters
 *   feet_m()      — feet → meters
 *   m_inches()    — meters → inches
 *   m_feet()      — meters → feet
 *   ftps_mps()    — feet/s → meters/s
 *   mps_ftps()    — meters/s → feet/s
 *   ftps2_mps2()  — ft/s² → m/s²
 *   mps2_ftps2()  — m/s² → ft/s²
 *   deg_rad()     — degrees → radians
 *   rad_deg()     — radians → degrees
 *   fieldFeet_m() — field coordinate in feet → meters (same math as feet_m,
 *                   named distinctly so call sites are self-documenting)
 *
 * ─── WHERE CONVERSIONS HAPPEN ────────────────────────────────────────────────
 * Conversion occurs ONLY at entry points — never inside control loops:
 *   driveInput constructor    (ft/s → m/s)
 *   swerveDrive constructor   (inches → meters)
 *   drivePointAt()            (feet → meters for field target)
 *
 * WPILib, PathPlanner, Choreo, and PhotonVision all operate in meters.
 * Do not call these conversions anywhere inside the kinematics or PID math.
 */
public final class units {

    private units() {}

    // ── Length ────────────────────────────────────────────────────────────────

    public static double inches_m(double inches)   { return inches * 0.0254; }
    public static double feet_m(double feet)        { return feet * 0.3048; }
    public static double m_inches(double meters)    { return meters / 0.0254; }
    public static double m_feet(double meters)      { return meters / 0.3048; }

    /**
     * Field coordinate in feet → meters.
     * Same math as feet_m() but named to make call sites self-documenting.
     * Use when converting field target positions for drivePointAt().
     */
    public static double fieldFeet_m(double feet)  { return feet * 0.3048; }

    // ── Velocity ──────────────────────────────────────────────────────────────

    public static double ftps_mps(double ftps)     { return ftps * 0.3048; }
    public static double mps_ftps(double mps)      { return mps / 0.3048; }

    // ── Acceleration ──────────────────────────────────────────────────────────

    public static double ftps2_mps2(double ftps2)  { return ftps2 * 0.3048; }
    public static double mps2_ftps2(double mps2)   { return mps2 / 0.3048; }

    // ── Angle ─────────────────────────────────────────────────────────────────

    public static double deg_rad(double degrees)   { return Math.toRadians(degrees); }
    public static double rad_deg(double radians)   { return Math.toDegrees(radians); }
}