package frc.robot.util;

import edu.wpi.first.util.datalog.*;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * RobotLogger.java
 * PATH: src/main/java/frc/robot/util/RobotLogger.java
 *
 * Static singleton that owns the WPILib DataLog instance and all log entry handles.
 * Call RobotLogger.init() once in Robot.robotInit() before any subsystem uses it.
 * Subsystems call the static log*() methods the same way they call SmartDashboard.
 *
 * ─── STORAGE ─────────────────────────────────────────────────────────────────
 * Primary:  /u/  — USB drive plugged into roboRIO (recommended)
 * Fallback: /home/lvuser/logs/ — roboRIO internal flash (limited write cycles)
 * Active path is posted to SmartDashboard: "Logger/Storage Path"
 *
 * USB drive requirements:
 *   Format: FAT32 (exFAT not supported by roboRIO)
 *   Speed:  Class 10 / UHS-I or better
 *   Size:   32GB+ recommended for a full season of logs
 *   Label:  "ROBOT_LOG" recommended for easy pit identification
 *
 * ─── FILE FORMAT ─────────────────────────────────────────────────────────────
 * WPILib DataLog writes .wpilog files. Open in AdvantageScope on driver station
 * laptop. Phoenix 6 SignalLogger writes .hoot files to the same USB path.
 * AdvantageScope can open both simultaneously on a shared timeline.
 *
 * ─── ENTRY NAMING CONVENTION ─────────────────────────────────────────────────
 * /Drive/...        — swerveDrive computed quantities
 * /Swerve/<FL|FR|BL|BR>/...  — per-module analog encoder (CAN signals via .hoot)
 * /Vision/...       — visionSubsystem estimates and health
 * /Pathfinding/...  — pathfindCommand state
 */
public final class RobotLogger {

    private static DataLog log;
    private static boolean initialized = false;

    // ── Drive — pose and speeds ───────────────────────────────────────────────
    public static DoubleLogEntry drivePoseX;
    public static DoubleLogEntry drivePoseY;
    public static DoubleLogEntry drivePoseHeadingDeg;

    public static DoubleLogEntry driveCommandedVx;
    public static DoubleLogEntry driveCommandedVy;
    public static DoubleLogEntry driveCommandedOmega;

    public static DoubleLogEntry driveActualVx;
    public static DoubleLogEntry driveActualVy;
    public static DoubleLogEntry driveActualOmega;

    // ── Drive — odometry buckets ──────────────────────────────────────────────
    public static DoubleLogEntry driveEncLinVel;
    public static DoubleLogEntry driveEncAngVel;
    public static DoubleLogEntry driveEncLinAccel;

    public static DoubleLogEntry driveImuLinVel;
    public static DoubleLogEntry driveImuAngVel;
    public static DoubleLogEntry driveImuLinAccel;

    public static DoubleLogEntry driveBlendedLinVel;
    public static DoubleLogEntry driveBlendedAngVel;
    public static DoubleLogEntry driveBlendedLinAccel;

    // ── Drive — per-module analog steer encoder ───────────────────────────────
    public static DoubleLogEntry[] moduleSteerVolts; // [FL, FR, BL, BR]

    // ── Vision — best pose ────────────────────────────────────────────────────
    public static DoubleLogEntry visionBestPoseX;
    public static DoubleLogEntry visionBestPoseY;
    public static DoubleLogEntry visionBestPoseHeadingDeg;
    public static DoubleLogEntry visionBestPoseZ;
    public static DoubleLogEntry visionBestPosePitchDeg;
    public static DoubleLogEntry visionBestPoseRollDeg;
    public static DoubleLogEntry visionBestTimestampSecs;
    public static DoubleLogEntry visionBestAmbiguity;
    public static IntegerLogEntry visionBestTagCount;
    public static StringLogEntry  visionBestCameraName;
    public static BooleanLogEntry visionHasValidPose;
    public static BooleanLogEntry visionBothCamerasValid;

    // ── Vision — per-camera pose ──────────────────────────────────────────────
    public static DoubleLogEntry visionFrontPoseX;
    public static DoubleLogEntry visionFrontPoseY;
    public static DoubleLogEntry visionFrontAmbiguity;
    public static IntegerLogEntry visionFrontTagCount;
    public static BooleanLogEntry visionFrontValid;

    public static DoubleLogEntry visionRearPoseX;
    public static DoubleLogEntry visionRearPoseY;
    public static DoubleLogEntry visionRearAmbiguity;
    public static IntegerLogEntry visionRearTagCount;
    public static BooleanLogEntry visionRearValid;

    // ── Vision — health ───────────────────────────────────────────────────────
    public static BooleanLogEntry visionHealthy;
    public static StringLogEntry  visionHealthStatus;

    // ── Pathfinding ───────────────────────────────────────────────────────────
    public static BooleanLogEntry pathfindActive;
    public static DoubleLogEntry  pathfindTargetX;
    public static DoubleLogEntry  pathfindTargetY;
    public static DoubleLogEntry  pathfindTargetHeadingDeg;
    public static DoubleLogEntry  pathfindPoseErrorM;
    public static BooleanLogEntry pathfindVisionStale;

    // ─────────────────────────────────────────────────────────────────────────

    private RobotLogger() {}

    /**
     * Initializes DataLogManager and all log entry handles.
     * Call once from Robot.robotInit(), before any subsystem logs anything.
     * Safe to call multiple times — subsequent calls are no-ops.
     */
    public static void init() {
        if (initialized) return;

        // Try USB first; fall back to roboRIO internal storage
        String path;
        java.io.File usb = new java.io.File("/u");
        if (usb.exists() && usb.canWrite()) {
            path = "/u/";
            SmartDashboard.putString("Logger/Storage Path", "USB (/u/)");
        } else {
            path = "/home/lvuser/logs/";
            new java.io.File(path).mkdirs();
            SmartDashboard.putString("Logger/Storage Path", "Internal (/home/lvuser/logs/) — INSERT USB");
        }

        DataLogManager.start(path);
        log = DataLogManager.getLog();

        // Also capture stdout/stderr to the log
        DataLogManager.logConsoleOutput(true);

        initDriveEntries();
        initVisionEntries();
        initPathfindingEntries();

        initialized = true;
        SmartDashboard.putBoolean("Logger/Initialized", true);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Entry initialization
    // ─────────────────────────────────────────────────────────────────────────

    private static void initDriveEntries() {
        drivePoseX            = new DoubleLogEntry(log, "/Drive/Pose/X_m");
        drivePoseY            = new DoubleLogEntry(log, "/Drive/Pose/Y_m");
        drivePoseHeadingDeg   = new DoubleLogEntry(log, "/Drive/Pose/Heading_deg");

        driveCommandedVx      = new DoubleLogEntry(log, "/Drive/Commanded/Vx_mps");
        driveCommandedVy      = new DoubleLogEntry(log, "/Drive/Commanded/Vy_mps");
        driveCommandedOmega   = new DoubleLogEntry(log, "/Drive/Commanded/Omega_radps");

        driveActualVx         = new DoubleLogEntry(log, "/Drive/Actual/Vx_mps");
        driveActualVy         = new DoubleLogEntry(log, "/Drive/Actual/Vy_mps");
        driveActualOmega      = new DoubleLogEntry(log, "/Drive/Actual/Omega_radps");

        driveEncLinVel        = new DoubleLogEntry(log, "/Drive/Encoder/LinVel_mps");
        driveEncAngVel        = new DoubleLogEntry(log, "/Drive/Encoder/AngVel_radps");
        driveEncLinAccel      = new DoubleLogEntry(log, "/Drive/Encoder/LinAccel_mps2");

        driveImuLinVel        = new DoubleLogEntry(log, "/Drive/IMU/LinVel_mps");
        driveImuAngVel        = new DoubleLogEntry(log, "/Drive/IMU/AngVel_radps");
        driveImuLinAccel      = new DoubleLogEntry(log, "/Drive/IMU/LinAccel_mps2");

        driveBlendedLinVel    = new DoubleLogEntry(log, "/Drive/Blended/LinVel_mps");
        driveBlendedAngVel    = new DoubleLogEntry(log, "/Drive/Blended/AngVel_radps");
        driveBlendedLinAccel  = new DoubleLogEntry(log, "/Drive/Blended/LinAccel_mps2");

        String[] names = {"FL", "FR", "BL", "BR"};
        moduleSteerVolts = new DoubleLogEntry[4];
        for (int i = 0; i < 4; i++) {
            moduleSteerVolts[i] = new DoubleLogEntry(log,
                "/Swerve/" + names[i] + "/SteerEncoder_volts");
        }
    }

    private static void initVisionEntries() {
        visionBestPoseX          = new DoubleLogEntry(log,  "/Vision/Best/X_m");
        visionBestPoseY          = new DoubleLogEntry(log,  "/Vision/Best/Y_m");
        visionBestPoseHeadingDeg = new DoubleLogEntry(log,  "/Vision/Best/Heading_deg");
        visionBestPoseZ          = new DoubleLogEntry(log,  "/Vision/Best/Z_m");
        visionBestPosePitchDeg   = new DoubleLogEntry(log,  "/Vision/Best/Pitch_deg");
        visionBestPoseRollDeg    = new DoubleLogEntry(log,  "/Vision/Best/Roll_deg");
        visionBestTimestampSecs  = new DoubleLogEntry(log,  "/Vision/Best/Timestamp_s");
        visionBestAmbiguity      = new DoubleLogEntry(log,  "/Vision/Best/Ambiguity");
        visionBestTagCount       = new IntegerLogEntry(log, "/Vision/Best/TagCount");
        visionBestCameraName     = new StringLogEntry(log,  "/Vision/Best/Camera");
        visionHasValidPose       = new BooleanLogEntry(log, "/Vision/HasValidPose");
        visionBothCamerasValid   = new BooleanLogEntry(log, "/Vision/BothCamerasValid");

        visionFrontPoseX         = new DoubleLogEntry(log,  "/Vision/Front/X_m");
        visionFrontPoseY         = new DoubleLogEntry(log,  "/Vision/Front/Y_m");
        visionFrontAmbiguity     = new DoubleLogEntry(log,  "/Vision/Front/Ambiguity");
        visionFrontTagCount      = new IntegerLogEntry(log, "/Vision/Front/TagCount");
        visionFrontValid         = new BooleanLogEntry(log, "/Vision/Front/Valid");

        visionRearPoseX          = new DoubleLogEntry(log,  "/Vision/Rear/X_m");
        visionRearPoseY          = new DoubleLogEntry(log,  "/Vision/Rear/Y_m");
        visionRearAmbiguity      = new DoubleLogEntry(log,  "/Vision/Rear/Ambiguity");
        visionRearTagCount       = new IntegerLogEntry(log, "/Vision/Rear/TagCount");
        visionRearValid          = new BooleanLogEntry(log, "/Vision/Rear/Valid");

        visionHealthy            = new BooleanLogEntry(log, "/Vision/Healthy");
        visionHealthStatus       = new StringLogEntry(log,  "/Vision/HealthStatus");
    }

    private static void initPathfindingEntries() {
        pathfindActive           = new BooleanLogEntry(log, "/Pathfinding/Active");
        pathfindTargetX          = new DoubleLogEntry(log,  "/Pathfinding/Target/X_m");
        pathfindTargetY          = new DoubleLogEntry(log,  "/Pathfinding/Target/Y_m");
        pathfindTargetHeadingDeg = new DoubleLogEntry(log,  "/Pathfinding/Target/Heading_deg");
        pathfindPoseErrorM       = new DoubleLogEntry(log,  "/Pathfinding/PoseError_m");
        pathfindVisionStale      = new BooleanLogEntry(log, "/Pathfinding/VisionStale");
    }
}
