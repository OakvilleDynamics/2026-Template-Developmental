package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.lib.util.Dashboard;
import frc.lib.util.Units;

/**
 * VisionHealthMonitor.java
 * PATH: src/main/java/frc/robot/subsystems/vision/VisionHealthMonitor.java
 *
 * Pre-match validation and continuous vision health tracking.
 * Updated every loop by VisionSubsystem via update().
 *
 * ─── HEALTH STATUS ───────────────────────────────────────────────────────────
 * UNKNOWN  — not enough data yet (cameras just connected)
 * GOOD     — all checks passing, both cameras healthy
 * WARNING  — degraded but usable (intermittent tag loss, elevated ambiguity)
 * FAULT    — camera lost or pose implausible (action required)
 *
 * ─── DASHBOARD ───────────────────────────────────────────────────────────────
 * Publishes status indicators through Dashboard under "Vision Health/" for the
 * Elastic "Vision Health" tab. Widget type, colors and placement live in the
 * Elastic layout file, not in code.
 * Drive team should check this tab before every match.
 * Green = good, Yellow = warning, Red = fault.
 *
 * ─── PRE-MATCH POSE CHECK ────────────────────────────────────────────────────
 * Call setExpectedStartPose() once the robot is placed on the field.
 * The monitor compares incoming vision poses against this expected pose
 * and flags if they disagree by more than POSE_PLAUSIBILITY_TOLERANCE_M.
 * Call clearExpectedStartPose() at match start so normal movement
 * doesn't trigger the plausibility check during play.
 */
public class VisionHealthMonitor {

    // ── Thresholds ────────────────────────────────────────────────────────────

    /** Frames of no tags before camera is flagged (~20ms per frame) */
    private static final int MISS_FRAMES_WARNING = 25;   // ~0.5 seconds
    private static final int MISS_FRAMES_FAULT   = 100;  // ~2 seconds

    /** Frames of elevated ambiguity before flagging */
    private static final int HIGH_AMBIGUITY_FRAMES = 15;

    /** How far (meters) front and rear pose estimates can disagree */
    private static final double INTER_CAMERA_TOLERANCE_M = 0.5;

    /**
     * How far (meters) a vision pose can be from the expected start pose
     * before flagging a plausibility warning.
     * Set generously — robot placement on field is not perfectly precise.
     */
    private static final double POSE_PLAUSIBILITY_TOLERANCE_M = 0.75;

    // ── State tracking ────────────────────────────────────────────────────────
    private int frontMissFrames     = 0;
    private int rearMissFrames      = 0;
    private int frontHighAmbFrames  = 0;
    private int rearHighAmbFrames   = 0;
    private int interCameraDisagree = 0;

    private boolean frontConnected  = false;
    private boolean rearConnected   = false;

    private Pose3d expectedStartPose = null;

    // ── Overall health ────────────────────────────────────────────────────────
    private HealthStatus overallStatus = HealthStatus.UNKNOWN;

    public enum HealthStatus { UNKNOWN, GOOD, WARNING, FAULT }

    // ── DataLog entries (post-match analysis) ─────────────────────────────────
    private final IntegerLogEntry logFrontMissFrames;
    private final IntegerLogEntry logRearMissFrames;
    private final IntegerLogEntry logFrontHighAmbFrames;
    private final IntegerLogEntry logRearHighAmbFrames;
    private final IntegerLogEntry logInterCameraDisagree;

    // ── Dashboard key prefixes (Vision Health/...) ─────────────────────────────
    private static final String DASH_PREFIX   = "Vision Health/";
    private static final String STATUS_PREFIX = DASH_PREFIX + "Camera Status/";
    private static final String AMB_PREFIX    = DASH_PREFIX + "Ambiguity/";
    private static final String CHECK_PREFIX  = DASH_PREFIX + "Checks/";
    private static final String POSE_PREFIX   = DASH_PREFIX + "Current Pose (ft)/";

    // ─────────────────────────────────────────────────────────────────────────
    // Constructor
    // ─────────────────────────────────────────────────────────────────────────

    public VisionHealthMonitor() {
        var log = DataLogManager.getLog();
        logFrontMissFrames     = new IntegerLogEntry(log, "/Vision/Debug/FrontMissFrames");
        logRearMissFrames      = new IntegerLogEntry(log, "/Vision/Debug/RearMissFrames");
        logFrontHighAmbFrames  = new IntegerLogEntry(log, "/Vision/Debug/FrontHighAmbFrames");
        logRearHighAmbFrames   = new IntegerLogEntry(log, "/Vision/Debug/RearHighAmbFrames");
        logInterCameraDisagree = new IntegerLogEntry(log, "/Vision/Debug/InterCameraDisagree");
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Public API
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Call from VisionSubsystem.periodic() every loop.
     * Processes latest estimates and updates health status.
     */
    public void update(RobotPoseEstimate frontEstimate, RobotPoseEstimate rearEstimate) {
        updateCameraHealth(frontEstimate, rearEstimate);
        updateInterCameraAgreement(frontEstimate, rearEstimate);
        updateOverallStatus();
        publishToDashboard(frontEstimate, rearEstimate);
        logDiagnosticCounters();
    }

    /**
     * Sets the pose the robot is expected to start at.
     * Call once the robot is placed on the field during pre-match setup.
     * The monitor flags if vision pose disagrees by more than
     * POSE_PLAUSIBILITY_TOLERANCE_M from this expected position.
     *
     * @param expectedPose  Expected starting Pose3d in field coordinates (meters)
     */
    public void setExpectedStartPose(Pose3d expectedPose) {
        this.expectedStartPose = expectedPose;
        Dashboard.log("Vision/Health/Expected Start",
            String.format("(%.2f ft, %.2f ft)",
                Units.metersToFeet(expectedPose.getX()),
                Units.metersToFeet(expectedPose.getY())));
    }

    /**
     * Clears the expected start pose.
     * Call at match start so normal robot movement doesn't trigger
     * the plausibility fault during play.
     */
    public void clearExpectedStartPose() {
        this.expectedStartPose = null;
        Dashboard.log("Vision/Health/Expected Start", "cleared");
    }

    public HealthStatus getOverallStatus()    { return overallStatus; }
    public boolean      isFrontCameraHealthy(){ return frontMissFrames < MISS_FRAMES_WARNING; }
    public boolean      isRearCameraHealthy() { return rearMissFrames  < MISS_FRAMES_WARNING; }
    public boolean      isOverallHealthy()    { return overallStatus == HealthStatus.GOOD; }

    // ─────────────────────────────────────────────────────────────────────────
    // Internal health logic
    // ─────────────────────────────────────────────────────────────────────────

    private void updateCameraHealth(RobotPoseEstimate front, RobotPoseEstimate rear) {
        // Front camera
        if (front.isValid) {
            frontMissFrames    = 0;
            frontHighAmbFrames = 0;
            frontConnected     = true;
        } else {
            frontMissFrames++;
            if (!front.cameraName.equals("none")) frontConnected = true;
        }

        // Rear camera
        if (rear.isValid) {
            rearMissFrames    = 0;
            rearHighAmbFrames = 0;
            rearConnected     = true;
        } else {
            rearMissFrames++;
            if (!rear.cameraName.equals("none")) rearConnected = true;
        }

        // Ambiguity tracking
        if (front.isValid && front.ambiguity > 0.15) frontHighAmbFrames++;
        else if (front.isValid) frontHighAmbFrames = 0;

        if (rear.isValid && rear.ambiguity > 0.15) rearHighAmbFrames++;
        else if (rear.isValid) rearHighAmbFrames = 0;
    }

    private void updateInterCameraAgreement(RobotPoseEstimate front, RobotPoseEstimate rear) {
        if (!front.isValid || !rear.isValid) {
            interCameraDisagree = 0;
            return;
        }

        double distance = front.pose.getTranslation()
            .getDistance(rear.pose.getTranslation());

        if (distance > INTER_CAMERA_TOLERANCE_M) {
            interCameraDisagree++;
        } else {
            interCameraDisagree = 0;
        }
    }

    private void updateOverallStatus() {
        boolean anyFault =
            frontMissFrames > MISS_FRAMES_FAULT   ||
            rearMissFrames  > MISS_FRAMES_FAULT   ||
            interCameraDisagree > 30;

        boolean anyWarning =
            frontMissFrames    > MISS_FRAMES_WARNING      ||
            rearMissFrames     > MISS_FRAMES_WARNING      ||
            frontHighAmbFrames > HIGH_AMBIGUITY_FRAMES    ||
            rearHighAmbFrames  > HIGH_AMBIGUITY_FRAMES    ||
            interCameraDisagree > 10;

        if (anyFault)              overallStatus = HealthStatus.FAULT;
        else if (anyWarning)       overallStatus = HealthStatus.WARNING;
        else if (frontConnected && rearConnected) overallStatus = HealthStatus.GOOD;
        else                       overallStatus = HealthStatus.UNKNOWN;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // DataLog publishing
    // ─────────────────────────────────────────────────────────────────────────

    private void logDiagnosticCounters() {
        logFrontMissFrames    .append(frontMissFrames);
        logRearMissFrames     .append(rearMissFrames);
        logFrontHighAmbFrames .append(frontHighAmbFrames);
        logRearHighAmbFrames  .append(rearHighAmbFrames);
        logInterCameraDisagree.append(interCameraDisagree);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Dashboard updates
    // ─────────────────────────────────────────────────────────────────────────

    private void publishToDashboard(RobotPoseEstimate front, RobotPoseEstimate rear) {
        Dashboard.log(STATUS_PREFIX + "Front connected", frontConnected);
        Dashboard.log(STATUS_PREFIX + "Rear connected", rearConnected);
        Dashboard.log(STATUS_PREFIX + "Front sees tags", front.isValid);
        Dashboard.log(STATUS_PREFIX + "Rear sees tags", rear.isValid);

        Dashboard.log(AMB_PREFIX + "Front ambiguity", front.isValid ? front.ambiguity : 1.0);
        Dashboard.log(AMB_PREFIX + "Rear ambiguity", rear.isValid   ? rear.ambiguity  : 1.0);

        Dashboard.log(CHECK_PREFIX + "Cameras agree", interCameraDisagree < 10);

        // Plausibility check — only meaningful when expected pose is set
        boolean plausible = true;
        if (expectedStartPose != null) {
            RobotPoseEstimate best = RobotPoseEstimate.best(front, rear);
            if (best.isValid) {
                double dist = expectedStartPose.getTranslation()
                    .getDistance(best.pose.getTranslation());
                plausible = dist < POSE_PLAUSIBILITY_TOLERANCE_M;
                Dashboard.log("Vision/Health/Start Pose Error (ft)",
                    Units.metersToFeet(dist));
            }
        }
        Dashboard.log(CHECK_PREFIX + "Pose plausible", plausible);

        Dashboard.log(DASH_PREFIX + "Overall Status", overallStatus.name());

        if (front.isValid) {
            Dashboard.log(POSE_PREFIX + "Front X (ft)", Units.metersToFeet(front.pose.getX()));
            Dashboard.log(POSE_PREFIX + "Front Y (ft)", Units.metersToFeet(front.pose.getY()));
        }
        if (rear.isValid) {
            Dashboard.log(POSE_PREFIX + "Rear X (ft)", Units.metersToFeet(rear.pose.getX()));
            Dashboard.log(POSE_PREFIX + "Rear Y (ft)", Units.metersToFeet(rear.pose.getY()));
        }
    }
}