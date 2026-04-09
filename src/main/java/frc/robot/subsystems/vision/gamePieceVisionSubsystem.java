package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.gamePieceConstants;
import frc.robot.subsystems.swerveDrive.swerveDrive;

/**
 * gamePieceVisionSubsystem.java
 * PATH: src/main/java/frc/robot/subsystems/vision/gamePieceVisionSubsystem.java
 *
 * Tracks game pieces on the field using an intake-mounted PhotonVision camera.
 * Converts raw pixel detections to field-relative positions using robot pose
 * history from swerveDrive.getPoseAtTime() for latency compensation.
 *
 * ─── DETECTION PIPELINE ──────────────────────────────────────────────────────
 * Every loop:
 *   1. Pull the latest PhotonPipelineResult from the intake camera.
 *   2. Retrieve the robot pose at the frame's capture timestamp.
 *   3. Back-project each detected target from pixel angles (yaw, pitch) to a
 *      field-relative Translation2d using floor-intersection geometry.
 *   4. Merge new detections into the tracked list (update existing nearby entries
 *      or add new ones). Expire stale entries.
 *
 * ─── BACK-PROJECTION MATH ────────────────────────────────────────────────────
 *   verticalAngleRad = cameraMountPitchRad - toRadians(target.getPitch())
 *   horizontalDist   = (cameraHeight - GAME_PIECE_HEIGHT) / tan(verticalAngle)
 *   bearingField     = cameraYawField + toRadians(target.getYaw())
 *   pieceX           = cameraX + horizontalDist * cos(bearingField)
 *   pieceY           = cameraY + horizontalDist * sin(bearingField)
 *
 * ─── HEALTH MONITORING ───────────────────────────────────────────────────────
 * isCameraConnected() — true if the camera has streamed any result this session.
 * hasPieceDetection() — true if at least one piece is currently tracked.
 * publishHealthStatus() — called from Robot.disabledPeriodic() for pre-match UI.
 *
 * Pre-match driver check: place a game piece 1–2 m in front of the intake and
 * confirm "Piece Detected" shows true on the Shuffleboard "Vision Health" tab.
 *
 * ─── CLUSTERING ──────────────────────────────────────────────────────────────
 * getClusters() groups tracked pieces by proximity (CLUSTER_RADIUS_M).
 * Uses a greedy sweep: nearest unassigned piece starts each cluster; remaining
 * pieces within CLUSTER_RADIUS_M join it. Clusters are sorted by centroid
 * distance from the robot, nearest first.
 */
public class gamePieceVisionSubsystem extends SubsystemBase {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final PhotonCamera   intakeCam;
    private final swerveDrive    drive;

    // ── Tracked pieces ────────────────────────────────────────────────────────

    /** A game piece detection projected to field coordinates. */
    public static final class DetectedPiece {
        /** Field-relative position (meters, WPILib field origin). */
        public final Translation2d fieldPos;
        /** FPGA timestamp of the most recent detection that contributed to this entry. */
        public final double        timestampSecs;
        /** Detection confidence from PhotonVision (0.0–1.0). */
        public final double        confidence;

        public DetectedPiece(Translation2d fieldPos, double timestampSecs, double confidence) {
            this.fieldPos      = fieldPos;
            this.timestampSecs = timestampSecs;
            this.confidence    = confidence;
        }
    }

    private final List<DetectedPiece> trackedPieces = new ArrayList<>();

    // ── Health state ──────────────────────────────────────────────────────────
    private boolean cameraConnected  = false;
    private int     missFrameCount   = 0;

    // ── DataLog entries ───────────────────────────────────────────────────────
    private final BooleanLogEntry logCameraConnected;
    private final BooleanLogEntry logPieceDetected;
    private final IntegerLogEntry logTrackedPieceCount;
    private final IntegerLogEntry logMissFrames;

    // ── Shuffleboard ──────────────────────────────────────────────────────────
    private edu.wpi.first.networktables.GenericEntry sbCameraConnected;
    private edu.wpi.first.networktables.GenericEntry sbPieceDetected;
    private edu.wpi.first.networktables.GenericEntry sbTrackedCount;

    // ─────────────────────────────────────────────────────────────────────────
    // Constructor
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * @param drive  swerveDrive reference — used for getPoseAtTime() during
     *               back-projection. Must be constructed before this subsystem.
     */
    public gamePieceVisionSubsystem(swerveDrive drive) {
        this.drive     = drive;
        this.intakeCam = new PhotonCamera(gamePieceConstants.GAME_PIECE_CAMERA_NAME);

        var log = DataLogManager.getLog();
        logCameraConnected   = new BooleanLogEntry(log, "/GamePieceVision/CameraConnected");
        logPieceDetected     = new BooleanLogEntry(log, "/GamePieceVision/PieceDetected");
        logTrackedPieceCount = new IntegerLogEntry(log, "/GamePieceVision/TrackedPieceCount");
        logMissFrames        = new IntegerLogEntry(log, "/GamePieceVision/Debug/MissFrames");

        buildShuffleboardWidgets();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // SubsystemBase — periodic
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        PhotonPipelineResult result = intakeCam.getLatestResult();

        // Track connectivity: PhotonCamera.isConnected() checks network presence
        cameraConnected = intakeCam.isConnected();

        if (!result.hasTargets()) {
            missFrameCount++;
        } else {
            missFrameCount = 0;

            // Get robot pose at the exact moment this frame was captured
            double captureTimestamp = result.getTimestampSeconds();
            Pose2d robotPoseAtCapture = drive.getPoseAtTime(captureTimestamp);

            // Compute camera pose in field frame
            Pose3d robotPose3d = new Pose3d(
                robotPoseAtCapture.getX(),
                robotPoseAtCapture.getY(),
                0.0,
                new edu.wpi.first.math.geometry.Rotation3d(
                    0.0, 0.0, robotPoseAtCapture.getRotation().getRadians())
            );
            Pose3d cameraPose = robotPose3d.transformBy(
                gamePieceConstants.GAME_PIECE_CAMERA_TRANSFORM);

            double cameraX         = cameraPose.getX();
            double cameraY         = cameraPose.getY();
            double cameraHeight    = cameraPose.getZ();
            double cameraYawField  = cameraPose.getRotation().getZ();
            // Mount pitch: how far the camera is tilted down from horizontal.
            // Stored as negative in the Transform3d (tilt down = negative pitch),
            // so negate to get the positive downward angle used in the formula.
            double cameraMountPitch = -gamePieceConstants.GAME_PIECE_CAMERA_TRANSFORM
                                          .getRotation().getY();

            for (PhotonTrackedTarget target : result.getTargets()) {
                // Filter by confidence — PhotonVision exposes it as "pose ambiguity"
                // for AprilTags, but for object detection we use the raw detection
                // confidence score from the pipeline.
                double confidence = 1.0 - target.getPoseAmbiguity();
                if (confidence < gamePieceConstants.DETECTION_CONFIDENCE_MIN) continue;

                // Back-project target pixel angles to field Translation2d
                double verticalAngleRad = cameraMountPitch
                                          - Math.toRadians(target.getPitch());
                if (Math.tan(verticalAngleRad) <= 0.0) continue;  // piece above horizon — skip

                double horizontalDist = (cameraHeight - gamePieceConstants.GAME_PIECE_HEIGHT_M)
                                        / Math.tan(verticalAngleRad);
                double bearingField   = cameraYawField + Math.toRadians(target.getYaw());

                double pieceX = cameraX + horizontalDist * Math.cos(bearingField);
                double pieceY = cameraY + horizontalDist * Math.sin(bearingField);
                Translation2d fieldPos = new Translation2d(pieceX, pieceY);

                mergeDetection(fieldPos, captureTimestamp, confidence);
            }
        }

        // Expire stale entries
        double now = Timer.getFPGATimestamp();
        trackedPieces.removeIf(p -> now - p.timestampSecs > gamePieceConstants.PIECE_STALE_SECS);

        // DataLog
        boolean pieceDetected = !trackedPieces.isEmpty();
        logCameraConnected  .append(cameraConnected);
        logPieceDetected    .append(pieceDetected);
        logTrackedPieceCount.append(trackedPieces.size());
        logMissFrames       .append(missFrameCount);

        // Shuffleboard (live during match)
        if (sbCameraConnected != null) sbCameraConnected.setBoolean(cameraConnected);
        if (sbPieceDetected   != null) sbPieceDetected  .setBoolean(pieceDetected);
        if (sbTrackedCount    != null) sbTrackedCount   .setDouble(trackedPieces.size());
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Public API — piece and cluster queries
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * All currently tracked field-relative game pieces, freshness-filtered.
     * Safe to call from commands each loop.
     */
    public List<DetectedPiece> getFieldRelativePieces() {
        return List.copyOf(trackedPieces);
    }

    /**
     * Returns the field position of the nearest tracked game piece to the robot.
     *
     * @param robotPose  Current robot pose (from swerveDrive.getPose())
     * @return  Nearest piece position, or empty if no pieces are tracked.
     */
    public Optional<Translation2d> getNearestPiece(Pose2d robotPose) {
        if (trackedPieces.isEmpty()) return Optional.empty();
        Translation2d robotPos = robotPose.getTranslation();
        return trackedPieces.stream()
            .min((a, b) -> Double.compare(
                a.fieldPos.getDistance(robotPos),
                b.fieldPos.getDistance(robotPos)))
            .map(p -> p.fieldPos);
    }

    /**
     * Returns game pieces grouped into clusters, sorted by cluster centroid
     * distance from the robot (nearest cluster first).
     *
     * Clustering: greedy sweep — nearest unassigned piece starts each cluster,
     * then all remaining pieces within CLUSTER_RADIUS_M join it. Repeats until
     * all pieces are assigned.
     *
     * Each inner list contains the field positions of all pieces in that cluster.
     * An empty outer list means no pieces are tracked.
     *
     * @param robotPose  Current robot pose (from swerveDrive.getPose())
     */
    public List<List<Translation2d>> getClusters(Pose2d robotPose) {
        if (trackedPieces.isEmpty()) return List.of();

        Translation2d robotPos = robotPose.getTranslation();

        // Collect all piece positions sorted by distance from robot
        List<Translation2d> sorted = new ArrayList<>();
        for (DetectedPiece p : trackedPieces) sorted.add(p.fieldPos);
        sorted.sort((a, b) -> Double.compare(
            a.getDistance(robotPos), b.getDistance(robotPos)));

        boolean[] assigned = new boolean[sorted.size()];
        List<List<Translation2d>> clusters = new ArrayList<>();

        for (int i = 0; i < sorted.size(); i++) {
            if (assigned[i]) continue;

            // Start a new cluster with this (nearest unassigned) piece
            List<Translation2d> cluster = new ArrayList<>();
            cluster.add(sorted.get(i));
            assigned[i] = true;

            // Sweep remaining pieces and absorb those within cluster radius
            for (int j = i + 1; j < sorted.size(); j++) {
                if (!assigned[j]
                        && sorted.get(j).getDistance(sorted.get(i))
                           <= gamePieceConstants.CLUSTER_RADIUS_M) {
                    cluster.add(sorted.get(j));
                    assigned[j] = true;
                }
            }
            clusters.add(cluster);
        }

        // Sort clusters by centroid distance from robot
        clusters.sort((a, b) -> {
            double distA = centroid(a).getDistance(robotPos);
            double distB = centroid(b).getDistance(robotPos);
            return Double.compare(distA, distB);
        });

        return clusters;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Public API — health
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * True if the intake camera is connected and streaming to the roboRIO.
     * Updated every loop.
     */
    public boolean isCameraConnected() { return cameraConnected; }

    /**
     * True if at least one game piece is currently tracked.
     * Driver pre-match check: place a piece in front of the intake and confirm
     * this returns true on Shuffleboard before the match starts.
     */
    public boolean hasPieceDetection() { return !trackedPieces.isEmpty(); }

    /**
     * Human-readable health summary for Shuffleboard / driver station.
     * Called from Robot.disabledPeriodic() pre-match.
     */
    public String getHealthStatus() {
        if (!cameraConnected) return "FAULT: intake camera not connected";
        if (missFrameCount > 50) return "WARNING: no detections for ~1s";
        if (!trackedPieces.isEmpty()) return "GOOD: " + trackedPieces.size() + " piece(s) detected";
        return "GOOD: camera connected, no pieces in view";
    }

    /**
     * Publishes health status string to Shuffleboard.
     * Call from Robot.disabledPeriodic() so the drive team sees it pre-match.
     */
    public void publishHealthStatus() {
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString(
            "GamePieceVision/HealthStatus", getHealthStatus());
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Internal helpers
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Merges a new detection into the tracked list.
     * If a tracked piece is within CLUSTER_RADIUS_M / 3 of the new position,
     * the existing entry is updated (averaged position, refreshed timestamp).
     * Otherwise a new entry is added.
     */
    private void mergeDetection(Translation2d fieldPos, double timestampSecs, double confidence) {
        double mergeRadius = gamePieceConstants.CLUSTER_RADIUS_M / 3.0;
        for (int i = 0; i < trackedPieces.size(); i++) {
            DetectedPiece existing = trackedPieces.get(i);
            if (existing.fieldPos.getDistance(fieldPos) <= mergeRadius) {
                // Average position and refresh timestamp
                double avgX = (existing.fieldPos.getX() + fieldPos.getX()) / 2.0;
                double avgY = (existing.fieldPos.getY() + fieldPos.getY()) / 2.0;
                trackedPieces.set(i, new DetectedPiece(
                    new Translation2d(avgX, avgY), timestampSecs, confidence));
                return;
            }
        }
        trackedPieces.add(new DetectedPiece(fieldPos, timestampSecs, confidence));
    }

    /** Average position of all pieces in a cluster. */
    private static Translation2d centroid(List<Translation2d> cluster) {
        double x = 0, y = 0;
        for (Translation2d p : cluster) { x += p.getX(); y += p.getY(); }
        return new Translation2d(x / cluster.size(), y / cluster.size());
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Shuffleboard
    // ─────────────────────────────────────────────────────────────────────────

    private void buildShuffleboardWidgets() {
        ShuffleboardTab tab = Shuffleboard.getTab("Vision Health");

        ShuffleboardLayout layout = tab
            .getLayout("Game Piece Camera", BuiltInLayouts.kList)
            .withSize(2, 3)
            .withPosition(8, 0);

        sbCameraConnected = layout.add("Intake cam connected", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "red"))
            .getEntry();

        sbPieceDetected = layout.add("Piece detected", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "yellow"))
            .getEntry();

        sbTrackedCount = layout.add("Tracked pieces", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
    }
}
