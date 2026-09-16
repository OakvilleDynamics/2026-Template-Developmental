package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.constants.canIDs;
import frc.robot.commands.driveWithJoysticks;
import frc.robot.commands.xLockCommand;
import frc.robot.constants.swerveConstants;
import frc.robot.pathplanning.FieldTargets;
import frc.robot.pathplanning.pathfindCommand;
import frc.robot.subsystems.swerveDrive.swerveDrive;
import frc.robot.subsystems.swerveDrive.swerveModule;
import frc.robot.util.ConfigVerifier;

/**
 * RobotContainer.java — drivetrain-only branch (test/swerve)
 * PATH: src/main/java/frc/robot/RobotContainer.java
 */
public class RobotContainer {

    // ═════════════════════════════════════════════════════════════════════════
    // Robot geometry — all in INCHES
    // ═════════════════════════════════════════════════════════════════════════

    // Thrifty Narrow pivot center is 2.625" from outer frame edge on each side.
    // Wheel base = 27.0 - 2 × 2.625 = 21.75"
    // TODO: verify 2.625" offset against actual CAD/physical measurement.
    private static final double[] WHEEL_BASE_IN = { 21.75, 21.75 };
    private static final double[] FRAME_IN      = { 27.0, 27.0 };
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
    // Subsystems and commands
    // ═════════════════════════════════════════════════════════════════════════

    private final swerveDrive        drive;
    private final driveWithJoysticks driveCommand;
    private final xLockCommand       xLock;
    private final pathfindCommand    pathfind;

    // ─────────────────────────────────────────────────────────────────────────
    // Constructor
    // ─────────────────────────────────────────────────────────────────────────

    public RobotContainer() {

        // TODO: verify drive inversion — FL/BL assumed inverted, FR/BR not inverted.
        swerveModule flModule = new swerveModule("FL", 0,
            canIDs.FL_DRIVE,         canIDs.FL_STEER,
            canIDs.FL_ANALOG_PORT,   canIDs.FL_CANCODER,
            swerveConstants.FL_STEER_OFFSET_VOLTS, swerveConstants.FL_STEER_OFFSET_ROT,
            true,  FL_DRIVE_PID, FL_STEER_PID);

        swerveModule frModule = new swerveModule("FR", 1,
            canIDs.FR_DRIVE,         canIDs.FR_STEER,
            canIDs.FR_ANALOG_PORT,   canIDs.FR_CANCODER,
            swerveConstants.FR_STEER_OFFSET_VOLTS, swerveConstants.FR_STEER_OFFSET_ROT,
            false, FR_DRIVE_PID, FR_STEER_PID);

        swerveModule blModule = new swerveModule("BL", 2,
            canIDs.BL_DRIVE,         canIDs.BL_STEER,
            canIDs.BL_ANALOG_PORT,   canIDs.BL_CANCODER,
            swerveConstants.BL_STEER_OFFSET_VOLTS, swerveConstants.BL_STEER_OFFSET_ROT,
            true,  BL_DRIVE_PID, BL_STEER_PID);

        swerveModule brModule = new swerveModule("BR", 3,
            canIDs.BR_DRIVE,         canIDs.BR_STEER,
            canIDs.BR_ANALOG_PORT,   canIDs.BR_CANCODER,
            swerveConstants.BR_STEER_OFFSET_VOLTS, swerveConstants.BR_STEER_OFFSET_ROT,
            false, BR_DRIVE_PID, BR_STEER_PID);

        drive = new swerveDrive(
            new swerveModule[]{ flModule, frModule, blModule, brModule },
            WHEEL_BASE_IN, FRAME_IN, BUMPER_IN, WHEEL_DIAM_IN, HEADING_PID
        );

        // Config verification — checks motor controller config readback at startup.
        // Prints mismatches to DS console and DataLog.
        ConfigVerifier.register(flModule);
        ConfigVerifier.register(frModule);
        ConfigVerifier.register(blModule);
        ConfigVerifier.register(brModule);
        ConfigVerifier.runAll();

        driveCommand = new driveWithJoysticks(
            drive, leftStick, rightStick, COR_MAX_X_IN, COR_MAX_Y_IN
        );
        xLock = new xLockCommand(drive);

        drive.configureForAutoBuilder();

        pathfind = new pathfindCommand(
            drive,
            () -> FieldTargets.get("speaker")
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

    public swerveDrive getDrive() { return drive; }

    public Command getAutonomousCommand() { return Commands.none(); }
}
