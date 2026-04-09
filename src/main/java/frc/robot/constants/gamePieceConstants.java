package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.units;

/**
 * gamePieceConstants.java
 * PATH: src/main/java/frc/robot/constants/gamePieceConstants.java
 *
 * Configuration for the game piece detection camera and hunt behavior.
 * This camera is separate from the two AprilTag pose estimation cameras and
 * runs on its own dedicated OrangePi 5 (GAME_PIECE_COPROCESSOR_IP).
 * It is mounted on the intake side of the robot and points toward the floor.
 *
 * ─── COPROCESSOR NETWORK ─────────────────────────────────────────────────────
 * Game piece OrangePi 5 static IP: 10.87.19.12
 *   Camera: intake_cam (OV9782 — color sensor for game piece ML detection)
 *   PhotonVision UI: http://10.87.19.12:5800
 *
 * The AprilTag coprocessor (10.87.19.11) is configured in visionConstants.java.
 * Both coprocessors connect to the roboRIO's NetworkTables server as NT clients.
 * PhotonCamera("intake_cam") resolves through NT transparently — no robot code
 * structural change is required to run cameras on separate physical devices.
 *
 * ─── SEASON SETUP CHECKLIST ──────────────────────────────────────────────────
 * Each new season:
 *   1. GAME_PIECE_HEIGHT_M   — update for the game piece's height above carpet
 *   2. GAME_PIECE_CAMERA_TRANSFORM — remeasure if robot geometry changed
 *   3. CLUSTER_RADIUS_M      — tune based on how pieces are distributed on field
 *   4. DETECTION_CONFIDENCE_MIN — adjust based on PhotonVision pipeline accuracy
 *
 * ─── EVENT SETUP CHECKLIST ───────────────────────────────────────────────────
 * At each competition event:
 *   1. Verify GAME_PIECE_CAMERA_NAME matches exactly what is set in PhotonVision UI
 *   2. Confirm stream URL is reachable from driver station
 *   3. Pre-match check: place a game piece ~1–2 m in front of intake, confirm
 *      "Piece Detected" reads true on Shuffleboard "Vision Health" tab
 *
 * ─── CAMERA TRANSFORM CONVENTION ─────────────────────────────────────────────
 * Transform3d encodes camera position and orientation relative to robot center:
 *   Translation3d(x, y, z) — robot center to camera (meters, robot frame)
 *     +x = forward, +y = left, +z = up
 *   Rotation3d(roll, pitch, yaw) — camera pointing direction
 *     Negative pitch = camera tilted down (pointing toward floor)
 *     Yaw 0 = facing forward (same as robot)
 *
 * ─── BACK-PROJECTION MATH ────────────────────────────────────────────────────
 * GamePieceVisionSubsystem uses getYaw() and getPitch() from PhotonVision targets
 * to project detected pieces to field coordinates:
 *
 *   verticalAngleRad = cameraMountPitchRad - Math.toRadians(target.getPitch())
 *   horizontalDist   = (cameraHeight_m - GAME_PIECE_HEIGHT_M) / Math.tan(verticalAngleRad)
 *   bearingFieldRad  = cameraYawFieldRad + Math.toRadians(target.getYaw())
 *   pieceX           = cameraX + horizontalDist * cos(bearingFieldRad)
 *   pieceY           = cameraY + horizontalDist * sin(bearingFieldRad)
 */
public final class gamePieceConstants {

    // No instances
    private gamePieceConstants() {}

    // ─────────────────────────────────────────────────────────────────────────
    // Camera identity
    // ─────────────────────────────────────────────────────────────────────────

    /** Static IP of the game piece detection OrangePi 5. */
    public static final String GAME_PIECE_COPROCESSOR_IP = "10.87.19.12";

    /** PhotonVision web UI for the game piece coprocessor — for pit diagnostics */
    public static final String GAME_PIECE_PHOTONVISION_UI_URL = "http://10.87.19.12:5800";

    /** Must match the camera name configured in PhotonVision UI on the game piece coprocessor exactly. */
    public static final String GAME_PIECE_CAMERA_NAME = "intake_cam";

    /** MJPEG stream URL for driver station dashboard. Verify port in PhotonVision UI. */
    public static final String GAME_PIECE_CAMERA_STREAM_URL = "http://10.87.19.12:1182/stream.mjpg";

    // ─────────────────────────────────────────────────────────────────────────
    // Camera mounting geometry
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Pose of the camera relative to robot center.
     *
     * TODO: Fill in actual mounting position after robot is built.
     *   Translation: distance along intake side (forward) and height above carpet.
     *   Rotation: pitch angle (negative = tilted down toward floor).
     *
     * Example below: camera 14 inches forward, 0 lateral, 18 inches up,
     * tilted 35° downward toward the intake zone.
     */
    public static final Transform3d GAME_PIECE_CAMERA_TRANSFORM = new Transform3d(
        new Translation3d(
            units.inches_m(14.0),   // forward from robot center (intake side)
            units.inches_m(0.0),    // lateral offset (0 = centered)
            units.inches_m(18.0)    // height above carpet
        ),
        new Rotation3d(
            0.0,                    // roll
            units.deg_rad(-35.0),   // pitch — negative = tilted down toward floor
            0.0                     // yaw — 0 = facing same direction as intake
        )
    );

    // ─────────────────────────────────────────────────────────────────────────
    // Game piece geometry
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Height of the game piece center above carpet (meters).
     * Used in the floor-intersection back-projection calculation.
     * Set to 0.0 for flat pieces resting on carpet.
     *
     * TODO: Update for the 2026 game piece once known.
     */
    public static final double GAME_PIECE_HEIGHT_M = 0.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Detection filtering
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Minimum PhotonVision detection confidence (0.0–1.0) to accept a target.
     * Targets below this threshold are discarded before back-projection.
     * Increase if false positives are observed; decrease if pieces are missed.
     */
    public static final double DETECTION_CONFIDENCE_MIN = 0.5;

    /**
     * Seconds after last detection before a tracked piece is discarded.
     * Pieces not confirmed by a new frame within this window are dropped.
     * 1.0 s at 50 Hz = 50 frames of tolerance for brief camera occlusion.
     */
    public static final double PIECE_STALE_SECS = 1.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Clustering
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Maximum distance (meters) between two pieces for them to be grouped
     * into the same cluster. Tune based on how field pieces are distributed.
     *
     * 1.5 m is appropriate for most FRC note/ring-style stacks at half-field.
     * Individual pieces separated by more than this form their own clusters.
     */
    public static final double CLUSTER_RADIUS_M = 1.5;

    // ─────────────────────────────────────────────────────────────────────────
    // Hunt behavior
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Heading offset (degrees) applied to the robot's target arrival pose.
     * Adjusts for intake not being exactly on the robot's forward axis.
     * 0.0 if intake faces forward. Positive = CCW rotation of intake heading.
     *
     * TODO: Set after measuring intake direction on physical robot.
     */
    public static final double APPROACH_HEADING_OFFSET_DEG = 0.0;

    /**
     * Seconds without detecting a game piece before gamePieceHuntCommand
     * transitions to DONE and ends the hunt. Prevents the robot from
     * waiting indefinitely if all pieces were collected or out of view.
     */
    public static final double HUNT_NO_PIECE_TIMEOUT_SECS = 3.0;

    /**
     * Timeout (seconds) for the intake to signal completion after the robot
     * arrives at a game piece. If intakeComplete does not return true within
     * this window, the hunt advances to the next target anyway.
     */
    public static final double INTAKE_TIMEOUT_SECS = 2.0;
}
