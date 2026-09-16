package frc.robot.util;

import edu.wpi.first.util.datalog.*;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * RobotLogger.java — drivetrain-only branch (test/swerve)
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
 * ─── FILE FORMAT ─────────────────────────────────────────────────────────────
 * WPILib DataLog writes .wpilog files. Open in AdvantageScope on driver station
 * laptop. Phoenix 6 SignalLogger writes .hoot files to the same USB path.
 * AdvantageScope can open both simultaneously on a shared timeline.
 *
 * ─── ENTRY NAMING CONVENTION ─────────────────────────────────────────────────
 * /Drive/...       — swerveDrive computed quantities
 * /Swerve/<FL|FR|BL|BR>/... — per-module analog encoder (CAN signals via .hoot)
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

    // ─────────────────────────────────────────────────────────────────────────

    private RobotLogger() {}

    /**
     * Initializes DataLogManager and all log entry handles.
     * Call once from Robot.robotInit(), before any subsystem logs anything.
     * Safe to call multiple times — subsequent calls are no-ops.
     */
    public static void init() {
        if (initialized) return;

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
        DataLogManager.logConsoleOutput(true);

        initDriveEntries();

        initialized = true;
        SmartDashboard.putBoolean("Logger/Initialized", true);
    }

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
}
