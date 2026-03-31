package frc.robot;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.driveWithJoysticks;
import frc.robot.commands.xLockCommand;
import frc.robot.constants.swerveConstants;
import frc.robot.pathplanning.FieldTargets;
import frc.robot.pathplanning.pathfindCommand;
import frc.robot.subsystems.swerveDrive.swerveDrive;
import frc.robot.subsystems.swerveDrive.swerveModule;
import frc.robot.subsystems.vision.AprilTagFieldCalTab;
import frc.robot.subsystems.vision.robotPoseEstimate;
import frc.robot.subsystems.vision.visionSubsystem;

/**
 * RobotContainer.java
 * PATH: src/main/java/frc/robot/RobotContainer.java
 *
 * ─── SEASON CHANGE — ONE LINE ────────────────────────────────────────────────
 * Update GAME_YEAR_FIELD each season. Propagates to visionSubsystem →
 * AprilTagFieldCal → PhotonPoseEstimators → everywhere.
 *
 * ─── EVENT CHANGE — AprilTagIgnore.java ──────────────────────────────────────
 * To ignore a problem tag at a specific event, edit AprilTagIgnore.java
 * and redeploy. No other files need to change.
 */
public class RobotContainer {

    // ═════════════════════════════════════════════════════════════════════════
    // SEASON CONFIGURATION — update this one line each year
    // ═════════════════════════════════════════════════════════════════════════

    /**
     * AprilTag field layout for the current game year.
     * Propagates everywhere through the constructor chain.
     *
     * 2026 REBUILT:   AprilTagFields.kDefaultField (update when constant published)
     * 2025 Reefscape: AprilTagFields.k2025Reefscape
     * 2024 Crescendo: AprilTagFields.k2024Crescendo
     */
    private static final AprilTagFields GAME_YEAR_FIELD = AprilTagFields.kDefaultField;

    // ═════════════════════════════════════════════════════════════════════════
    // Robot geometry — all in INCHES
    // ═════════════════════════════════════════════════════════════════════════

    private static final double[] WHEEL_BASE_IN = { 22.0, 22.0 };
    private static final double[] FRAME_IN      = { 26.0, 26.0 };
    private static final double[] BUMPER_IN     = { 33.0, 33.0 };
    private static final double   WHEEL_DIAM_IN = 4.0;

    private static final double COR_MAX_X_IN = BUMPER_IN[0] / 2.0;
    private static final double COR_MAX_Y_IN = BUMPER_IN[1] / 2.0;

    // ═════════════════════════════════════════════════════════════════════════
    // Lock-to-target coordinates (field, in FEET)
    // ═════════════════════════════════════════════════════════════════════════

    private static final double TARGET_FEET_X     = 13.5;
    private static final double TARGET_FEET_Y     = 27.0;
    private static final double TARGET_OFFSET_DEG = 0.0;

    // ═════════════════════════════════════════════════════════════════════════
    // Heading PID
    // ═════════════════════════════════════════════════════════════════════════

    private static final double[] HEADING_PID = {
        swerveConstants.HEADING_PID_kP,
        swerveConstants.HEADING_PID_kI,
        swerveConstants.HEADING_PID_kD
    };

    // ═════════════════════════════════════════════════════════════════════════
    // Per-module PID defaults
    // drivePID: { kP, kI, kD, kS, kV, kA }
    // steerPID: { kP, kI, kD, kS, kV }
    // ═════════════════════════════════════════════════════════════════════════

    private static final double[] FL_DRIVE_PID = { 0.1, 0.0, 0.0, 0.15, 0.12, 0.01 };
    private static final double[] FL_STEER_PID = { 60.0, 0.0, 2.0, 0.25, 0.12 };
    private static final double[] FR_DRIVE_PID = { 0.1, 0.0, 0.0, 0.15, 0.12, 0.01 };
    private static final double[] FR_STEER_PID = { 60.0, 0.0, 2.0, 0.25, 0.12 };
    private static final double[] BL_DRIVE_PID = { 0.1, 0.0, 0.0, 0.15, 0.12, 0.01 };
    private static final double[] BL_STEER_PID = { 60.0, 0.0, 2.0, 0.25, 0.12 };
    private static final double[] BR_DRIVE_PID = { 0.1, 0.0, 0.0, 0.15, 0.12, 0.01 };
    private static final double[] BR_STEER_PID = { 60.0, 0.0, 2.0, 0.25, 0.12 };

    // ═════════════════════════════════════════════════════════════════════════
    // Controllers
    // ═════════════════════════════════════════════════════════════════════════

    private final Joystick leftStick  = new Joystick(0);
    private final Joystick rightStick = new Joystick(1);

    private final JoystickButton lockToTargetButton = new JoystickButton(rightStick, 2);
    private final JoystickButton xLockButton        = new JoystickButton(rightStick, 3);
    private final JoystickButton pathfindButton     = new JoystickButton(rightStick, 4);

    // ═════════════════════════════════════════════════════════════════════════
    // Subsystems, commands, calibration tab
    // ═════════════════════════════════════════════════════════════════════════

    private final swerveDrive        drive;
    private final visionSubsystem    vision;
    private final driveWithJoysticks driveCommand;
    private final xLockCommand       xLock;
    private final pathfindCommand    pathfind;
    private final AprilTagFieldCalTab calTab;

    // ─────────────────────────────────────────────────────────────────────────
    // Constructor
    // ─────────────────────────────────────────────────────────────────────────

    public RobotContainer() {

        swerveModule flModule = new swerveModule("FL",
            swerveConstants.FL_DRIVE_CAN_ID, swerveConstants.FL_STEER_CAN_ID,
            swerveConstants.FL_ANALOG_PORT,  swerveConstants.FL_STEER_OFFSET_VOLTS,
            true,  FL_DRIVE_PID, FL_STEER_PID);

        swerveModule frModule = new swerveModule("FR",
            swerveConstants.FR_DRIVE_CAN_ID, swerveConstants.FR_STEER_CAN_ID,
            swerveConstants.FR_ANALOG_PORT,  swerveConstants.FR_STEER_OFFSET_VOLTS,
            false, FR_DRIVE_PID, FR_STEER_PID);

        swerveModule blModule = new swerveModule("BL",
            swerveConstants.BL_DRIVE_CAN_ID, swerveConstants.BL_STEER_CAN_ID,
            swerveConstants.BL_ANALOG_PORT,  swerveConstants.BL_STEER_OFFSET_VOLTS,
            true,  BL_DRIVE_PID, BL_STEER_PID);

        swerveModule brModule = new swerveModule("BR",
            swerveConstants.BR_DRIVE_CAN_ID, swerveConstants.BR_STEER_CAN_ID,
            swerveConstants.BR_ANALOG_PORT,  swerveConstants.BR_STEER_OFFSET_VOLTS,
            false, BR_DRIVE_PID, BR_STEER_PID);

        drive = new swerveDrive(
            new swerveModule[]{ flModule, frModule, blModule, brModule },
            WHEEL_BASE_IN, FRAME_IN, BUMPER_IN, WHEEL_DIAM_IN, HEADING_PID
        );

        // GAME_YEAR_FIELD propagates from here into the entire vision stack
        vision = new visionSubsystem(GAME_YEAR_FIELD);

        // Calibration tab receives only visionSubsystem — single interface
        calTab = new AprilTagFieldCalTab(vision);

        driveCommand = new driveWithJoysticks(
            drive, leftStick, rightStick, COR_MAX_X_IN, COR_MAX_Y_IN
        );
        xLock = new xLockCommand(drive);

        drive.configureForAutoBuilder();

        pathfind = new pathfindCommand(
            drive, vision,
            () -> FieldTargets.get("speaker") // TODO: wire to button/selector for target choice
        );

        configureDefaultCommands();
        configureButtonBindings();
    }

    private void configureDefaultCommands() {
        drive.setDefaultCommand(driveCommand);
    }

    private void configureButtonBindings() {
        lockToTargetButton.whileTrue(Commands.startEnd(
            () -> driveCommand.enablePointAt(TARGET_FEET_X, TARGET_FEET_Y, TARGET_OFFSET_DEG),
            () -> driveCommand.disablePointAt()
        ));
        xLockButton.whileTrue(xLock);
        pathfindButton.whileTrue(pathfind);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Accessors
    // ─────────────────────────────────────────────────────────────────────────

    public swerveDrive     getDrive()  { return drive; }
    public visionSubsystem getVision() { return vision; }

    /** Called from Robot.java's robotPeriodic() every loop */
    public void updateCalibrationTab() { calTab.update(); }

    /**
     * Fuses the latest valid vision pose into the swerve drive Kalman filter.
     * Called from Robot.java's robotPeriodic() every loop, after the scheduler runs.
     * No-ops silently when vision has no valid pose — filter continues on odometry alone.
     */
    public void updatePoseEstimator() {
        if (!vision.hasValidPose()) return;
        robotPoseEstimate best = vision.getBestPose();
        drive.addVisionMeasurement(
            best.pose.toPose2d(),
            best.timestampSecs,
            best.tagCount
        );
    }

    public Command getAutonomousCommand() { return Commands.none(); }
}