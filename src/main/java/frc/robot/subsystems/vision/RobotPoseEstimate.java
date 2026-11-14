package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * RobotPoseEstimate.java
 * PATH: src/main/java/frc/robot/subsystems/vision/RobotPoseEstimate.java
 *
 * Immutable snapshot of a single robot pose estimate from one camera.
 * Carries the full 3D pose plus metadata about the quality of the estimate.
 *
 * ─── FIELDS ──────────────────────────────────────────────────────────────────
 * pose           — full 3D pose: x, y, z (meters) + roll, pitch, yaw (radians)
 * timestampSecs  — FPGA time when this estimate was captured (for latency compensation)
 * ambiguity      — PhotonVision ambiguity score (0.0 = perfect, 1.0 = worst)
 * tagCount       — number of AprilTags used to produce this estimate
 * cameraName     — which camera produced this estimate ("front_cam" / "rear_cam")
 * isValid        — false if no tags were visible or estimate was filtered out
 *
 * ─── ACCESSING INDIVIDUAL VALUES ─────────────────────────────────────────────
 * pose.getX()                       — field X position (meters, field-relative)
 * pose.getY()                       — field Y position (meters, field-relative)
 * pose.getZ()                       — height above field floor (meters)
 * pose.getRotation().getX()         — roll  (radians)
 * pose.getRotation().getY()         — pitch (radians)
 * pose.getRotation().getZ()         — yaw   (radians, field-relative heading)
 * pose.toPose2d()                   — flattened 2D pose for odometry fusion later
 *
 * ─── USAGE ───────────────────────────────────────────────────────────────────
 * RobotPoseEstimate best = vision.getBestPose();
 * if (best.isValid) {
 *     double x   = best.pose.getX();               // meters
 *     double y   = best.pose.getY();               // meters
 *     double yaw = best.pose.getRotation().getZ(); // radians
 * }
 */
public class RobotPoseEstimate {

    public final Pose3d  pose;
    public final double  timestampSecs;
    public final double  ambiguity;
    public final int     tagCount;
    public final String  cameraName;
    public final boolean isValid;

    // ─────────────────────────────────────────────────────────────────────────
    // Valid estimate constructor
    // ─────────────────────────────────────────────────────────────────────────

    public RobotPoseEstimate(
            Pose3d pose,
            double timestampSecs,
            double ambiguity,
            int tagCount,
            String cameraName) {
        this.pose          = pose;
        this.timestampSecs = timestampSecs;
        this.ambiguity     = ambiguity;
        this.tagCount      = tagCount;
        this.cameraName    = cameraName;
        this.isValid       = true;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Invalid/empty estimate
    // ─────────────────────────────────────────────────────────────────────────

    private RobotPoseEstimate() {
        this.pose          = new Pose3d();
        this.timestampSecs = 0.0;
        this.ambiguity     = 1.0;
        this.tagCount      = 0;
        this.cameraName    = "none";
        this.isValid       = false;
    }

    /**
     * Returns a sentinel invalid estimate.
     * Always check .isValid before reading any other field.
     * Safe to pass around and store — will never throw.
     */
    public static RobotPoseEstimate invalid() {
        return new RobotPoseEstimate();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Selection utility
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Returns the better of two estimates.
     *
     * Selection priority:
     *   1. Valid over invalid
     *   2. More tags over fewer tags (multi-tag is more reliable)
     *   3. Lower ambiguity over higher ambiguity
     *
     * Used by VisionSubsystem to pick the best estimate from two cameras.
     */
    public static RobotPoseEstimate best(RobotPoseEstimate a, RobotPoseEstimate b) {
        if (!a.isValid) return b;
        if (!b.isValid) return a;
        if (a.tagCount != b.tagCount) return a.tagCount > b.tagCount ? a : b;
        return a.ambiguity <= b.ambiguity ? a : b;
    }
}