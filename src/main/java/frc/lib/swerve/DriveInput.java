package frc.lib.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.util.Units;

/**
 * DriveInput.java
 * PATH: src/main/java/frc/lib/swerve/DriveInput.java
 *
 * Immutable data object passed into SwerveDrive.drive() each loop.
 * The only way to move the robot is to build one of these and hand it in.
 *
 * ─── UNITS AT THIS INTERFACE ─────────────────────────────────────────────────
 * Input  : vx, vy in FEET/SECOND
 *          omega in RADIANS/SECOND
 *          centerOfRotation in METERS (robot-relative from robot center)
 * Stored : all values in METERS/SECOND internally after conversion
 *
 * The conversion happens once here. SwerveDrive never sees ft/s.
 *
 * ─── FIELD-RELATIVE CONVENTION ───────────────────────────────────────────────
 * vx  positive → toward the far field wall (away from your alliance station)
 * vy  positive → field left (from driver's perspective facing the field)
 * omega CCW positive when viewed from above
 *
 * ─── CENTER OF ROTATION ──────────────────────────────────────────────────────
 * (0, 0) = robot geometric center — robot pivots in place
 * Positive X = toward robot front
 * Positive Y = toward robot left
 * Bounded to bumper corners by DriveWithJoysticks before being passed here
 */
public class DriveInput {

    private final double      vxMps;
    private final double      vyMps;
    private final double      omegaRadps;
    private final Translation2d centerOfRotation;

    /**
     * @param vxFtps           Forward velocity in FEET/SECOND
     * @param vyFtps           Left velocity in FEET/SECOND
     * @param omegaRadps       Rotation rate in RADIANS/SECOND (CCW positive)
     * @param centerOfRotation Robot-relative pivot point in METERS
     */
    public DriveInput(double vxFtps, double vyFtps, double omegaRadps,
                      Translation2d centerOfRotation) {
        this.vxMps            = Units.ftpsToMps(vxFtps);
        this.vyMps            = Units.ftpsToMps(vyFtps);
        this.omegaRadps       = omegaRadps;
        this.centerOfRotation = centerOfRotation;
    }

    /** Convenience constructor — CoR defaults to robot center (0,0) */
    public DriveInput(double vxFtps, double vyFtps, double omegaRadps) {
        this(vxFtps, vyFtps, omegaRadps, new Translation2d());
    }

    /** Fully stopped, pivot at robot center */
    public static DriveInput stopped() {
        return new DriveInput(0.0, 0.0, 0.0);
    }

    // ── Getters (SI — used by SwerveDrive internally) ─────────────────────────
    public double        getVxMps()             { return vxMps; }
    public double        getVyMps()             { return vyMps; }
    public double        getOmegaRadps()        { return omegaRadps; }
    public Translation2d getCenterOfRotation()  { return centerOfRotation; }

    // ── Getters (English — available for telemetry/logging) ───────────────────
    public double getVxFtps()  { return Units.mpsToFtps(vxMps); }
    public double getVyFtps()  { return Units.mpsToFtps(vyMps); }
}