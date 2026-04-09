package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.units;

/**
 * visionConstants.java
 * PATH: src/main/java/frc/robot/constants/visionConstants.java
 *
 * All PhotonVision and field vision configuration in one place.
 *
 * ─── SEASON SETUP CHECKLIST ──────────────────────────────────────────────────
 * Each new season, update:
 *   1. GAME_YEAR_FIELD  — change the AprilTagFields enum value in RobotContainer
 *                         (not here — visionConstants holds the default reference only)
 *   2. Camera transforms — remeasure if robot geometry changed
 *   3. Stream URLs       — verify port numbers in PhotonVision UI
 *   4. AprilTagIgnore.java — review tag IDs for new field layout
 *
 * ─── EVENT SETUP CHECKLIST ───────────────────────────────────────────────────
 * At each competition event:
 *   1. Verify stream URLs are reachable from driver station laptop
 *   2. Confirm camera names match exactly what is set in PhotonVision UI
 *   3. Run field calibration procedure via "Field Calibration" Shuffleboard tab
 *   4. Apply any needed tag offsets and generate the offsets file
 *
 * ─── CAMERA TRANSFORM COORDINATE FRAME ───────────────────────────────────────
 * Robot center (0,0,0) is at floor level, geometric center of frame.
 * X = forward, Y = left, Z = up (WPILib right-hand rule)
 * Rotation: roll (X axis), pitch (Y axis), yaw (Z axis) — all in radians
 * Negative pitch = camera tilted downward toward field (typical mounting)
 * Yaw of 180° = camera faces rearward
 *
 * ─── COPROCESSOR NETWORK ─────────────────────────────────────────────────────
 * This file covers the AprilTag pose estimation coprocessor only.
 * Game piece detection runs on a separate coprocessor — see gamePieceConstants.
 *
 * AprilTag OrangePi 5 static IP: 10.87.19.11
 *   Cameras: front_cam (OV9281), rear_cam (OV9281)
 *   PhotonVision UI: http://10.87.19.11:5800
 *   Stream ports assigned by PhotonVision: 1182, 1184, etc.
 *   Verify exact ports in the PhotonVision UI under camera settings.
 *
 * Replace 87.19 with your team number digits (team 8719 → 87.19).
 *
 * ─── WHY TWO COPROCESSORS ────────────────────────────────────────────────────
 * AprilTag detection (OV9281, global shutter, grayscale) and game piece ML
 * detection (OV9782, color) are on separate OrangePi 5 units for:
 *   1. Isolation — a game piece pipeline crash or overload cannot affect pose
 *      estimation reliability.
 *   2. Headroom — leaves full NPU/CPU available per role; no pipeline contention.
 *   3. Scalability — adding a 2nd game piece camera (2nd intake side) stays on
 *      the game piece coprocessor; AprilTag side is untouched.
 *
 * Both coprocessors connect to the roboRIO's NetworkTables server as NT clients.
 * PhotonCamera() constructor uses camera name only — it resolves through NT
 * transparently regardless of which physical device the camera is on.
 * No robot code structural change is required to split coprocessors.
 */
public final class visionConstants {

    // ═════════════════════════════════════════════════════════════════════════
    // GAME YEAR — reference value
    // The authoritative value lives in RobotContainer as GAME_YEAR_FIELD.
    // This constant is kept here for reference and as a fallback default.
    //
    // 2026 REBUILT:   AprilTagFields.kDefaultField (update when published)
    // 2025 Reefscape: AprilTagFields.k2025Reefscape
    // 2024 Crescendo: AprilTagFields.k2024Crescendo
    // ═════════════════════════════════════════════════════════════════════════

    public static final AprilTagFields GAME_YEAR_FIELD = AprilTagFields.kDefaultField;

    // ═════════════════════════════════════════════════════════════════════════
    // Camera names
    // Must match the camera name set in PhotonVision UI exactly (case-sensitive)
    // Set in PhotonVision UI first, then update these to match
    // ═════════════════════════════════════════════════════════════════════════

    public static final String FRONT_CAMERA_NAME = "front_cam";
    public static final String REAR_CAMERA_NAME  = "rear_cam";

    // ═════════════════════════════════════════════════════════════════════════
    // Camera MJPEG stream URLs for Shuffleboard display
    // Displayed on the "Field Calibration" tab via AprilTagFieldCalTab
    // Verify port numbers in PhotonVision UI → camera settings
    // ═════════════════════════════════════════════════════════════════════════

    /** Static IP of the AprilTag pose estimation OrangePi 5. */
    public static final String APRILTAG_COPROCESSOR_IP = "10.87.19.11";

    public static final String FRONT_CAMERA_STREAM_URL = "http://10.87.19.11:1182/stream.mjpg";
    public static final String REAR_CAMERA_STREAM_URL  = "http://10.87.19.11:1184/stream.mjpg";

    /** PhotonVision web UI for the AprilTag coprocessor — for pit diagnostics */
    public static final String PHOTONVISION_UI_URL = "http://10.87.19.11:5800";

    // ═════════════════════════════════════════════════════════════════════════
    // Camera transforms — robot-relative mounting positions
    //
    // TODO: Measure on your actual robot and update before first use.
    // These placeholder values assume cameras mounted on centerline,
    // 12" forward/rear of robot center, 18" above floor, 15° down tilt.
    // ═════════════════════════════════════════════════════════════════════════

    /** Front camera — faces forward, mounted toward robot front */
    public static final Transform3d FRONT_CAMERA_TRANSFORM = new Transform3d(
        new Translation3d(
            units.inches_m( 12.0),   // X:  12" forward of robot center
            units.inches_m(  0.0),   // Y:  on centerline
            units.inches_m( 18.0)    // Z:  18" above floor
        ),
        new Rotation3d(
            0.0,                     // roll:  level
            units.deg_rad(-15.0),    // pitch: 15° down toward field
            0.0                      // yaw:   facing straight forward
        )
    );

    /** Rear camera — faces backward (yaw = 180°), mounted toward robot rear */
    public static final Transform3d REAR_CAMERA_TRANSFORM = new Transform3d(
        new Translation3d(
            units.inches_m(-12.0),   // X:  12" behind robot center
            units.inches_m(  0.0),   // Y:  on centerline
            units.inches_m( 18.0)    // Z:  18" above floor
        ),
        new Rotation3d(
            0.0,
            units.deg_rad(-15.0),    // pitch: 15° down toward field
            units.deg_rad(180.0)     // yaw:   facing rearward
        )
    );

    // ═════════════════════════════════════════════════════════════════════════
    // Pose estimation filtering
    // ═════════════════════════════════════════════════════════════════════════

    /**
     * Maximum pose ambiguity accepted from PhotonVision (0.0–1.0).
     * Estimates above this value are discarded as unreliable.
     * Start at 0.2. Loosen to 0.3 if losing valid detections in
     * challenging lighting conditions at competition.
     */
    public static final double MAX_AMBIGUITY = 0.2;

    /**
     * Maximum distance (meters) a new pose estimate can jump from the
     * last accepted estimate before being rejected as an outlier.
     * Protects against single-frame misdetections.
     */
    public static final double MAX_POSE_JUMP_M = 1.5;

    /**
     * Minimum number of AprilTags that must be visible for a pose estimate
     * to be accepted.
     * 1 = accept single-tag estimates (less reliable, more coverage)
     * 2 = multi-tag only (more reliable, less coverage)
     */
    public static final int MIN_TAGS_FOR_ESTIMATE = 1;

    // ═════════════════════════════════════════════════════════════════════════
    // Field calibration thresholds
    // These are DEFAULT values. All are overridable live from the
    // "Field Calibration" Shuffleboard tab without redeploying.
    // ═════════════════════════════════════════════════════════════════════════

    /**
     * Threshold (inches) below which a tag delta is considered within tolerance.
     * Used by AprilTagFieldCalTab recommendation logic.
     *
     * Above threshold — recommendation depends on camera agreement:
     *   Both cameras agree but both off from WPILib → apply field offset
     *   Cameras disagree with each other → check camera transform
     *
     * Default: 1.0 inch. Adjustable live from "Cal Tolerance (in)" slider
     * on the Field Calibration Shuffleboard tab.
     */
    public static final double CALIBRATION_TOLERANCE_INCHES = 1.0;

    // ═════════════════════════════════════════════════════════════════════════
    // NetworkTables keys for offset file generation
    // Must match the keys watched by aprilTagFieldCalWatch.py
    // ═════════════════════════════════════════════════════════════════════════

    /**
     * Key where generated Java snippet is published.
     * aprilTagFieldCalWatch.py watches this key and saves to file on laptop.
     */
    public static final String OFFSETS_OUTPUT_NT_KEY = "/FieldCalibration/GeneratedOffsets";

    /**
     * Key where generated snippet is also published for clipboard copy.
     * aprilTagFieldCalWatch.py copies this to system clipboard.
     */
    public static final String OFFSETS_CLIPBOARD_NT_KEY = "/FieldCalibration/ClipboardOffsets";

    /**
     * Path on the roboRIO where a backup copy of the offsets file is written.
     * Recoverable via FTP/WinSCP if laptop copy is lost.
     */
    public static final String OFFSETS_ROBORIO_PATH = "/home/lvuser/field_offsets_latest.java";

    // ═════════════════════════════════════════════════════════════════════════
    // Pose estimator — vision measurement trust
    // ═════════════════════════════════════════════════════════════════════════

    /**
     * Base standard deviations for vision measurements [x (m), y (m), theta (rad)].
     * These are scaled per-measurement by VISION_STD_DEV_TAG_SCALE based on tag count.
     * Larger = trust vision less. Vision is reliable globally but noisy frame-to-frame.
     * TODO: tune on carpet with actual camera placement.
     */
    public static final double VISION_STD_DEV_X     = 0.9;
    public static final double VISION_STD_DEV_Y     = 0.9;
    public static final double VISION_STD_DEV_THETA = 0.9;

    /**
     * Multipliers applied to base std devs based on number of tags visible.
     * Index 0 = 1 tag, index 1 = 2 tags, index 2 = 3+ tags.
     * More tags → lower multiplier → higher trust in that measurement.
     */
    public static final double[] VISION_STD_DEV_TAG_SCALE = { 2.0, 1.0, 0.5 };

    // ═════════════════════════════════════════════════════════════════════════
    // Complementary filter blend weight
    // ═════════════════════════════════════════════════════════════════════════

    /**
     * Alpha weight for blended pose in driveOdometryState.
     * 0.0 = pure IMU, 1.0 = pure encoder.
     * Linear velocity/accel: encoder-dominant (alpha weight).
     * Angular velocity/accel: IMU-dominant (1-alpha weight).
     * Tunable live from SmartDashboard: "Drive/Blend Alpha"
     */
    public static final double ODOMETRY_BLEND_ALPHA = 0.7;
}