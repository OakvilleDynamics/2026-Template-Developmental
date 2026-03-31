package frc.robot.pathplanning;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.HashMap;
import java.util.Map;

/**
 * FieldTargets.java
 * PATH: src/main/java/frc/robot/pathplanning/FieldTargets.java
 *
 * String-keyed lookup table of field target poses for pathfinding.
 * All poses are in field coordinates (meters, WPILib blue-origin convention).
 *
 * ─── USAGE ───────────────────────────────────────────────────────────────────
 * Pose2d target = FieldTargets.get("speaker");
 * Throws IllegalArgumentException on unknown key — fail fast, fail loudly.
 *
 * ─── ADDING TARGETS ──────────────────────────────────────────────────────────
 * Add entries to the static initializer below.
 * Key: short lowercase string matching how RobotContainer will reference it.
 * Pose: field X (m), field Y (m), heading the robot should face on arrival.
 *
 * ─── COORDINATE CONVENTION ───────────────────────────────────────────────────
 * Origin (0, 0) = blue alliance corner (driver station side, left corner).
 * +X toward red alliance wall, +Y toward left when standing at blue station.
 * Heading 0° = facing red alliance wall (field forward).
 * PathPlanner handles alliance flipping — define all targets from blue origin.
 *
 * TODO: populate with real 2026 field target coordinates once field geometry is published.
 */
public final class FieldTargets {

    private static final Map<String, Pose2d> TARGETS = new HashMap<>();

    static {
        // ── Stub targets — replace X/Y/heading with real 2026 field coordinates ──
        TARGETS.put("speaker",  new Pose2d(1.45, 5.55, Rotation2d.fromDegrees(180.0)));
        TARGETS.put("amp",      new Pose2d(1.84, 7.70, Rotation2d.fromDegrees( 90.0)));
        TARGETS.put("source",   new Pose2d(15.4, 1.00, Rotation2d.fromDegrees(-60.0)));
        TARGETS.put("stage",    new Pose2d(5.32, 4.11, Rotation2d.fromDegrees(  0.0)));
    }

    private FieldTargets() {}

    /**
     * Returns the Pose2d for the named target.
     *
     * @param key  Target name (e.g. "speaker", "amp")
     * @return     Field pose in meters, blue-origin
     * @throws IllegalArgumentException if key is not in the table
     */
    public static Pose2d get(String key) {
        Pose2d pose = TARGETS.get(key);
        if (pose == null) {
            throw new IllegalArgumentException(
                "FieldTargets: unknown target key '" + key + "'");
        }
        return pose;
    }
}
