package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AprilTagIgnore;
import frc.robot.constants.visionConstants;
import frc.robot.util.AprilTagFieldCal;
import frc.robot.util.units;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

/**
 * visionSubsystem.java
 * PATH: src/main/java/frc/robot/subsystems/vision/visionSubsystem.java
 *
 * Single public interface for all vision data.
 * AprilTagFieldCalTab, RobotContainer, and all commands/middleware
 * read from this class only — nothing reaches into internal objects directly.
 *
 * ─── CONSTRUCTOR ─────────────────────────────────────────────────────────────
 * @param gameYearField  AprilTagFields enum — passed from RobotContainer.
 *                       One change in RobotContainer propagates everywhere.
 *
 * ─── INTERNAL OBJECTS ────────────────────────────────────────────────────────
 * AprilTagFieldCal    — field layout, per-tag offsets, geometry
 * visionHealthMonitor — pre-match validation, health tracking
 * These are not accessible from outside — all access goes through
 * the public methods below.
 */
public class visionSubsystem extends SubsystemBase {

    // ── Internal objects — not accessible from outside ────────────────────────
    private final AprilTagFieldCal    fieldCal;
    private final visionHealthMonitor healthMonitor;

    // ── Cameras ───────────────────────────────────────────────────────────────
    private final PhotonCamera frontCamera;
    private final PhotonCamera rearCamera;

    // ── Pose estimators (rebuilt when calibration changes) ────────────────────
    private PhotonPoseEstimator frontEstimator;
    private PhotonPoseEstimator rearEstimator;

    // ── Current pose estimates ────────────────────────────────────────────────
    private robotPoseEstimate frontPose = robotPoseEstimate.invalid();
    private robotPoseEstimate rearPose  = robotPoseEstimate.invalid();
    private robotPoseEstimate bestPose  = robotPoseEstimate.invalid();

    // ── Per-tag readings for AprilTagFieldCalTab ──────────────────────────────
    private final Map<Integer, Map<String, tagReading>> tagReadings = new HashMap<>();

    // ── Outlier rejection ─────────────────────────────────────────────────────
    private Pose3d lastAcceptedFront = null;
    private Pose3d lastAcceptedRear  = null;

    // ── Alliance change detection ─────────────────────────────────────────────
    private Optional<DriverStation.Alliance> lastKnownAlliance = Optional.empty();

    // ─────────────────────────────────────────────────────────────────────────
    // Inner record: one camera's reading of one tag
    // ─────────────────────────────────────────────────────────────────────────

    public record tagReading(
        int     tagId,
        String  cameraName,
        boolean visible,
        Pose3d  readPose,
        Pose3d  baselinePose,
        double  deltaXInches,
        double  deltaYInches,
        double  deltaZInches,
        double  deltaYawDeg,
        double  ambiguity
    ) {
        public static tagReading notVisible(int tagId, String cameraName, Pose3d baseline) {
            return new tagReading(tagId, cameraName, false,
                null, baseline, 0, 0, 0, 0, 1.0);
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Constructor
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * @param gameYearField  AprilTagFields enum for current game year.
     *                       Set once in RobotContainer — propagates everywhere.
     */
    public visionSubsystem(AprilTagFields gameYearField) {
        fieldCal      = new AprilTagFieldCal(gameYearField);
        healthMonitor = new visionHealthMonitor();

        frontCamera = new PhotonCamera(visionConstants.FRONT_CAMERA_NAME);
        rearCamera  = new PhotonCamera(visionConstants.REAR_CAMERA_NAME);

        buildEstimators();

        SmartDashboard.putString("Vision/Tag Ignore Rules", AprilTagIgnore.getSummary());
    }

    // ─────────────────────────────────────────────────────────────────────────
    // periodic
    // ─────────────────────────────────────────────────────────────────────────

@Override
    public void periodic() {
        Optional<DriverStation.Alliance> currentAlliance = DriverStation.getAlliance();
        if (!currentAlliance.equals(lastKnownAlliance)) {
            lastKnownAlliance = currentAlliance;
            fieldCal.rebuildLayout();
            buildEstimators();
            SmartDashboard.putString("Vision/Alliance",
                currentAlliance.isPresent() ? currentAlliance.get().name() : "Unknown");
        }

        frontPose = processCamera(
            frontCamera, frontEstimator,
            visionConstants.FRONT_CAMERA_NAME, lastAcceptedFront
        );
        if (frontPose.isValid) lastAcceptedFront = frontPose.pose;

        rearPose = processCamera(
            rearCamera, rearEstimator,
            visionConstants.REAR_CAMERA_NAME, lastAcceptedRear
        );
        if (rearPose.isValid) lastAcceptedRear = rearPose.pose;

        bestPose = robotPoseEstimate.best(frontPose, rearPose);

        updateTagReadings();
        healthMonitor.update(frontPose, rearPose);
        publishTelemetry();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Public API — pose data
    // ─────────────────────────────────────────────────────────────────────────

    public robotPoseEstimate getBestPose()     { return bestPose; }
    public robotPoseEstimate getFrontPose()    { return frontPose; }
    public robotPoseEstimate getRearPose()     { return rearPose; }
    public boolean           hasValidPose()    { return bestPose.isValid; }
    public boolean           bothCamerasValid(){ return frontPose.isValid && rearPose.isValid; }

    // ── 3D pose components — available when relevant (climbing, shooting, etc.) ──

    /**
     * Robot height above floor from best vision estimate (meters).
     * Useful when robot is on an elevated surface or climbing structure.
     * Returns 0.0 if no valid pose.
     */
    public double getBestPoseZ() {
        return bestPose.isValid ? bestPose.pose.getZ() : 0.0;
    }

    /**
     * Robot pitch angle from best vision estimate (radians, positive = nose up).
     * Useful for detecting tilt during climbing or on uneven surfaces.
     * Returns 0.0 if no valid pose.
     */
    public double getBestPosePitch() {
        return bestPose.isValid ? bestPose.pose.getRotation().getY() : 0.0;
    }

    /**
     * Robot roll angle from best vision estimate (radians, positive = right side up).
     * Useful for detecting tilt during climbing or on uneven surfaces.
     * Returns 0.0 if no valid pose.
     */
    public double getBestPoseRoll() {
        return bestPose.isValid ? bestPose.pose.getRotation().getX() : 0.0;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Public API — per-tag readings (for AprilTagFieldCalTab)
    // ─────────────────────────────────────────────────────────────────────────

    public Map<Integer, Map<String, tagReading>> getTagReadings() {
        return java.util.Collections.unmodifiableMap(tagReadings);
    }

    public Optional<tagReading> getTagReading(int tagId, String cameraName) {
        Map<String, tagReading> byCamera = tagReadings.get(tagId);
        if (byCamera == null) return Optional.empty();
        return Optional.ofNullable(byCamera.get(cameraName));
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Public API — field geometry
    // ─────────────────────────────────────────────────────────────────────────

    public double[] getFarCornerFt()   { return fieldCal.getFarCornerFt(); }
    public double   getFieldLengthFt() { return fieldCal.getFieldLengthFt(); }
    public double   getFieldWidthFt()  { return fieldCal.getFieldWidthFt(); }
    public double   getFieldLengthM()  { return fieldCal.getFieldLengthM(); }
    public double   getFieldWidthM()   { return fieldCal.getFieldWidthM(); }

    // ─────────────────────────────────────────────────────────────────────────
    // Public API — health
    // ─────────────────────────────────────────────────────────────────────────

    public boolean                          isHealthy()       { return healthMonitor.isOverallHealthy(); }
    public visionHealthMonitor.HealthStatus getHealthStatus() { return healthMonitor.getOverallStatus(); }

    // ─────────────────────────────────────────────────────────────────────────
    // Public API — calibration
    // ─────────────────────────────────────────────────────────────────────────

    public void setTagOffset(int tagId,
                              double xInches, double yInches, double zInches,
                              double rollDeg,  double pitchDeg, double yawDeg) {
        fieldCal.setTagOffset(tagId, xInches, yInches, zInches, rollDeg, pitchDeg, yawDeg);
    }

    public void clearTagOffset(int tagId)  { fieldCal.clearTagOffset(tagId); }
    public void clearAllOffsets()           { fieldCal.clearAllOffsets(); }

    public Map<Integer, edu.wpi.first.math.geometry.Transform3d> getTagOffsets() {
        return fieldCal.getTagOffsets();
    }

    public void rebuildEstimators() {
        fieldCal.rebuildLayout();
        buildEstimators();
        SmartDashboard.putString("FieldCal/Status", "Estimators rebuilt");
    }

    /**
     * Generates a formatted Java code snippet representing all current
     * non-zero tag offsets. Published to three destinations:
     *   1. NetworkTables key (AprilTagFieldCalWatch.py saves to file on laptop)
     *   2. NetworkTables clipboard key (AprilTagFieldCalWatch.py copies to clipboard)
     *   3. File on roboRIO at visionConstants.OFFSETS_ROBORIO_PATH
     */
    public void generateOffsetsOutput() {
        String snippet = buildOffsetsSnippet();

        SmartDashboard.putString(visionConstants.OFFSETS_OUTPUT_NT_KEY,    snippet);
        SmartDashboard.putString(visionConstants.OFFSETS_CLIPBOARD_NT_KEY, snippet);

        writeToRoboRIO(snippet);

        SmartDashboard.putString("FieldCal/Last Generated",
            new java.text.SimpleDateFormat("yyyy-MM-dd HH:mm:ss")
                .format(new java.util.Date()));
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Public API — pre-match validation
    // ─────────────────────────────────────────────────────────────────────────

    public void setExpectedStartPose(Pose3d pose) { healthMonitor.setExpectedStartPose(pose); }
    public void clearExpectedStartPose()           { healthMonitor.clearExpectedStartPose(); }

    // ─────────────────────────────────────────────────────────────────────────
    // Internal — camera processing
    // ─────────────────────────────────────────────────────────────────────────

    private robotPoseEstimate processCamera(
            PhotonCamera camera,
            PhotonPoseEstimator estimator,
            String name,
            Pose3d lastAccepted) {

        PhotonPipelineResult result = camera.getLatestResult();

        if (!result.hasTargets()) {
            SmartDashboard.putBoolean("Vision/" + name + "/Has Target", false);
            return robotPoseEstimate.invalid();
        }

        SmartDashboard.putBoolean("Vision/" + name + "/Has Target", true);

        int    tagCount  = result.getTargets().size();
        double ambiguity = result.getBestTarget().getPoseAmbiguity();
        SmartDashboard.putNumber("Vision/" + name + "/Ambiguity", ambiguity);

        if (tagCount < visionConstants.MIN_TAGS_FOR_ESTIMATE) {
            SmartDashboard.putString("Vision/" + name + "/Rejected",
                "Too few tags: " + tagCount);
            return robotPoseEstimate.invalid();
        }

        if (ambiguity > visionConstants.MAX_AMBIGUITY) {
            SmartDashboard.putString("Vision/" + name + "/Rejected",
                "High ambiguity: " + String.format("%.3f", ambiguity));
            return robotPoseEstimate.invalid();
        }

        var estimated = estimator.update(result);
        if (estimated.isEmpty()) {
            SmartDashboard.putString("Vision/" + name + "/Rejected", "Estimator empty");
            return robotPoseEstimate.invalid();
        }

        Pose3d pose3d    = estimated.get().estimatedPose;
        double timestamp = estimated.get().timestampSeconds;

        if (lastAccepted != null) {
            double jump = lastAccepted.getTranslation()
                .getDistance(pose3d.getTranslation());
            if (jump > visionConstants.MAX_POSE_JUMP_M) {
                SmartDashboard.putString("Vision/" + name + "/Rejected",
                    "Pose jump: " + String.format("%.2f", jump) + "m");
                return robotPoseEstimate.invalid();
            }
        }

        SmartDashboard.putString("Vision/" + name + "/Rejected", "none");
        return new robotPoseEstimate(pose3d, timestamp, ambiguity, tagCount, name);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Internal — per-tag reading updates
    // ─────────────────────────────────────────────────────────────────────────

    private void updateTagReadings() {
        PhotonPipelineResult frontResult = frontCamera.getLatestResult();
        PhotonPipelineResult rearResult  = rearCamera.getLatestResult();

        for (AprilTag tag : fieldCal.getAllBaseTags()) {
            int tagId = tag.ID;
            Optional<Pose3d> baseline = fieldCal.getBaselineTagPose(tagId);
            if (baseline.isEmpty()) continue;

            Map<String, tagReading> byCamera = tagReadings.computeIfAbsent(
                tagId, k -> new HashMap<>()
            );

            byCamera.put(visionConstants.FRONT_CAMERA_NAME,
                buildTagReading(tagId, visionConstants.FRONT_CAMERA_NAME,
                    frontResult, frontPose, baseline.get()));

            byCamera.put(visionConstants.REAR_CAMERA_NAME,
                buildTagReading(tagId, visionConstants.REAR_CAMERA_NAME,
                    rearResult, rearPose, baseline.get()));
        }
    }

    private tagReading buildTagReading(
            int tagId, String cameraName,
            PhotonPipelineResult result,
            robotPoseEstimate poseEstimate,
            Pose3d baseline) {

        Optional<PhotonTrackedTarget> targetOpt = result.hasTargets()
            ? result.getTargets().stream()
                .filter(t -> t.getFiducialId() == tagId)
                .findFirst()
            : Optional.empty();

        if (targetOpt.isEmpty() || !poseEstimate.isValid) {
            return tagReading.notVisible(tagId, cameraName, baseline);
        }

        double ambiguity = targetOpt.get().getPoseAmbiguity();

        // Simplified delta — full tag reprojection is a future refinement
        return new tagReading(tagId, cameraName, true,
            baseline, baseline, 0, 0, 0, 0, ambiguity);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Internal — estimator construction
    // ─────────────────────────────────────────────────────────────────────────

    private void buildEstimators() {
        var layout = fieldCal.getCalibratedLayout();

        frontEstimator = new PhotonPoseEstimator(
            layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            visionConstants.FRONT_CAMERA_TRANSFORM
        );
        frontEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        rearEstimator = new PhotonPoseEstimator(
            layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            visionConstants.REAR_CAMERA_TRANSFORM
        );
        rearEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Internal — offset file generation
    // ─────────────────────────────────────────────────────────────────────────

    private String buildOffsetsSnippet() {
        var offsets = fieldCal.getTagOffsets();
        StringBuilder sb = new StringBuilder();
        String timestamp = new java.text.SimpleDateFormat("yyyy-MM-dd HH:mm:ss")
            .format(new java.util.Date());

        sb.append("// AprilTag field calibration offsets generated: ")
          .append(timestamp).append("\n");
        sb.append("// Paste into RobotContainer after constructing visionSubsystem:\n\n");

        if (offsets.isEmpty()) {
            sb.append("// No offsets currently applied — all tags at WPILib baseline.\n");
        } else {
            offsets.forEach((tagId, transform) -> {
                double xIn  = units.m_inches(transform.getX());
                double yIn  = units.m_inches(transform.getY());
                double zIn  = units.m_inches(transform.getZ());
                double yDeg = units.rad_deg(transform.getRotation().getZ());
                double pDeg = units.rad_deg(transform.getRotation().getY());
                double rDeg = units.rad_deg(transform.getRotation().getX());
                sb.append(String.format(
                    "vision.setTagOffset(%d, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f);\n",
                    tagId, xIn, yIn, zIn, rDeg, pDeg, yDeg));
            });
            sb.append("vision.rebuildEstimators();\n");
        }

        return sb.toString();
    }

    private void writeToRoboRIO(String content) {
        try {
            java.nio.file.Files.writeString(
                java.nio.file.Path.of(visionConstants.OFFSETS_ROBORIO_PATH),
                content
            );
            SmartDashboard.putString("FieldCal/RoboRIO Save", "OK");
        } catch (Exception e) {
            SmartDashboard.putString("FieldCal/RoboRIO Save", "FAILED: " + e.getMessage());
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Internal — telemetry
    // ─────────────────────────────────────────────────────────────────────────

    private void publishTelemetry() {
        SmartDashboard.putBoolean("Vision/Best/Valid",     bestPose.isValid);
        SmartDashboard.putString( "Vision/Best/Source",    bestPose.cameraName);
        SmartDashboard.putNumber( "Vision/Best/Tag Count", bestPose.tagCount);

        if (bestPose.isValid) {
            Pose3d p = bestPose.pose;
            SmartDashboard.putNumber("Vision/Best/X (ft)",      units.m_feet(p.getX()));
            SmartDashboard.putNumber("Vision/Best/Y (ft)",      units.m_feet(p.getY()));
            SmartDashboard.putNumber("Vision/Best/Z (in)",      units.m_inches(p.getZ()));
            SmartDashboard.putNumber("Vision/Best/Yaw (deg)",   units.rad_deg(p.getRotation().getZ()));
            SmartDashboard.putNumber("Vision/Best/Pitch (deg)", units.rad_deg(p.getRotation().getY()));
            SmartDashboard.putNumber("Vision/Best/Roll (deg)",  units.rad_deg(p.getRotation().getX()));
        }
    }
}