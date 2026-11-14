package frc.robot;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.constants.CANIDs;
import frc.robot.constants.PortIDs;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.XLockCommand;
import frc.robot.constants.SwerveConstants;
import frc.robot.pathplanning.FieldTargets;
import frc.robot.pathplanning.GamePieceHuntCommand;
import frc.robot.pathplanning.GamePieceHuntCommand.HuntMode;
import frc.robot.pathplanning.PathfindCommand;
import frc.robot.subsystems.swerveDrive.SwerveDrive;
import frc.lib.hardware.encoder.AbsoluteEncoderIO;
import frc.lib.hardware.motor.MotorIO;
import frc.lib.swerve.SwerveHardware;
import frc.lib.swerve.SwerveModule;
import frc.lib.util.ConfigVerifier;
import frc.robot.subsystems.vision.AprilTagFieldCalTab;
import frc.robot.subsystems.vision.GamePieceVisionSubsystem;
import frc.robot.subsystems.vision.RobotPoseEstimate;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * RobotContainer.java
 * PATH: src/main/java/frc/robot/RobotContainer.java
 *
 * ─── SEASON CHANGE — ONE LINE ────────────────────────────────────────────────
 * Update GAME_YEAR_FIELD each season. Propagates to VisionSubsystem →
 * AprilTagFieldCal → PhotonPoseEstimators → everywhere.
 *
 * ─── EVENT CHANGE — AprilTagIgnore.java ──────────────────────────────────────
 * To ignore a problem tag at a specific event, edit AprilTagIgnore.java
 * and redeploy. No other files need to change.
 */
public class RobotContainer {

    //==========================================================================
    // SEASON CONFIGURATION — update this one line each year
    //==========================================================================


    /**
     * AprilTag field layout for the current game year.
     * Propagates everywhere through the constructor chain.
     *
     * 2026 REBUILT:   AprilTagFields.kDefaultField (update when constant published)
     * 2025 Reefscape: AprilTagFields.k2025Reefscape
     * 2024 Crescendo: AprilTagFields.k2024Crescendo
     */
    private static final AprilTagFields GAME_YEAR_FIELD = AprilTagFields.kDefaultField;

    //==========================================================================
    // Robot geometry — all in INCHES
    //==========================================================================


    // Thrifty Narrow pivot center is 2.5" from outer frame edge on each side (verified from CAD).
    // Wheel base = 27.0 - 2 × 2.5 = 22.0"
    private static final double[] WHEEL_BASE_IN = { 22.0, 22.0 };
    private static final double[] FRAME_IN      = { 27.0, 27.0 };
    private static final double[] BUMPER_IN     = { 33.0, 33.0 };
    private static final double   WHEEL_DIAM_IN = 4.0;

    private static final double COR_MAX_X_IN = BUMPER_IN[0] / 2.0;
    private static final double COR_MAX_Y_IN = BUMPER_IN[1] / 2.0;

    //==========================================================================
    // Lock-to-target coordinates (field, in FEET)
    //==========================================================================


    private static final double TARGET_FEET_X     = 13.5;
    private static final double TARGET_FEET_Y     = 27.0;
    private static final double TARGET_OFFSET_DEG = 0.0;

    //==========================================================================
    // Heading PID
    //==========================================================================

    private static final double[] HEADING_PID = {
        SwerveConstants.HEADING_PID_kP,
        SwerveConstants.HEADING_PID_kI,
        SwerveConstants.HEADING_PID_kD
    };

    //==========================================================================
    // Per-module PID defaults
    // drivePID: { kP, kI, kD, kS, kV, kA }
    // steerPID: { kP, kI, kD, kS, kV }
    //==========================================================================

    private static final double[] FL_DRIVE_PID = { 0.1, 0.0, 0.0, 0.15, 0.12, 0.01 };
    private static final double[] FL_STEER_PID = { 60.0, 0.0, 2.0, 0.25, 0.12 };
    private static final double[] FR_DRIVE_PID = { 0.1, 0.0, 0.0, 0.15, 0.12, 0.01 };
    private static final double[] FR_STEER_PID = { 60.0, 0.0, 2.0, 0.25, 0.12 };
    private static final double[] BL_DRIVE_PID = { 0.1, 0.0, 0.0, 0.15, 0.12, 0.01 };
    private static final double[] BL_STEER_PID = { 60.0, 0.0, 2.0, 0.25, 0.12 };
    private static final double[] BR_DRIVE_PID = { 0.1, 0.0, 0.0, 0.15, 0.12, 0.01 };
    private static final double[] BR_STEER_PID = { 60.0, 0.0, 2.0, 0.25, 0.12 };

    //==========================================================================
    // Controllers
    //==========================================================================

    private final Joystick leftStick  = new Joystick(0);
    private final Joystick rightStick = new Joystick(1);

    private final JoystickButton lockToTargetButton  = new JoystickButton(rightStick, 2);
    private final JoystickButton xLockButton         = new JoystickButton(rightStick, 3);
    private final JoystickButton pathfindButton      = new JoystickButton(rightStick, 4);

    // Game piece hunt — TODO: remap to final 2026 driver layout
    private final JoystickButton huntNearestButton   = new JoystickButton(rightStick, 5);
    private final JoystickButton huntSequentialButton = new JoystickButton(rightStick, 6);
    private final JoystickButton huntClusterButton   = new JoystickButton(rightStick, 7);
    private final JoystickButton huntSeqClusterButton = new JoystickButton(rightStick, 8);

    //==========================================================================
    // Subsystems, commands, calibration tab
    //==========================================================================

    private final SwerveDrive              drive;
    private final VisionSubsystem          vision;
    private final GamePieceVisionSubsystem gamePieceVision;
    private final DriveWithJoysticks       driveCommand;
    private final XLockCommand             xLock;
    private final PathfindCommand          pathfind;
    private final AprilTagFieldCalTab      calTab;
    private final GamePieceHuntCommand     huntNearest;
    private final GamePieceHuntCommand     huntSequential;
    private final GamePieceHuntCommand     huntCluster;
    private final GamePieceHuntCommand     huntSeqCluster;

    // ─────────────────────────────────────────────────────────────────────────
    // Constructor
    // ─────────────────────────────────────────────────────────────────────────

    public RobotContainer() {

        // TODO: verify drive inversion — FL/BL assumed inverted, FR/BR not inverted.
        SwerveModule flModule = buildModule("FL", 0,
            CANIDs.FL_DRIVE,                 CANIDs.FL_STEER,
            PortIDs.AI_FL_STEER_ENCODER,     CANIDs.FL_CANCODER,
            SwerveConstants.FL_STEER_OFFSET_VOLTS, SwerveConstants.FL_STEER_OFFSET_ROT,
            true,  FL_DRIVE_PID, FL_STEER_PID);

        SwerveModule frModule = buildModule("FR", 1,
            CANIDs.FR_DRIVE,                 CANIDs.FR_STEER,
            PortIDs.AI_FR_STEER_ENCODER,     CANIDs.FR_CANCODER,
            SwerveConstants.FR_STEER_OFFSET_VOLTS, SwerveConstants.FR_STEER_OFFSET_ROT,
            false, FR_DRIVE_PID, FR_STEER_PID);

        SwerveModule blModule = buildModule("BL", 2,
            CANIDs.BL_DRIVE,                 CANIDs.BL_STEER,
            PortIDs.AI_BL_STEER_ENCODER,     CANIDs.BL_CANCODER,
            SwerveConstants.BL_STEER_OFFSET_VOLTS, SwerveConstants.BL_STEER_OFFSET_ROT,
            true,  BL_DRIVE_PID, BL_STEER_PID);

        SwerveModule brModule = buildModule("BR", 3,
            CANIDs.BR_DRIVE,                 CANIDs.BR_STEER,
            PortIDs.AI_BR_STEER_ENCODER,     CANIDs.BR_CANCODER,
            SwerveConstants.BR_STEER_OFFSET_VOLTS, SwerveConstants.BR_STEER_OFFSET_ROT,
            false, BR_DRIVE_PID, BR_STEER_PID);

        drive = new SwerveDrive(
            new SwerveModule[]{ flModule, frModule, blModule, brModule },
            WHEEL_BASE_IN, FRAME_IN, BUMPER_IN, WHEEL_DIAM_IN, HEADING_PID
        );

        // ── Config verification — runs before CommandScheduler first executes ──
        // Add mechanisms here once they exist: ConfigVerifier.register(myShooter);
        ConfigVerifier.register(flModule);
        ConfigVerifier.register(frModule);
        ConfigVerifier.register(blModule);
        ConfigVerifier.register(brModule);

        // GAME_YEAR_FIELD propagates from here into the entire vision stack
        vision = new VisionSubsystem(GAME_YEAR_FIELD);

        // Game piece detection camera — must be constructed after drive (needs getPoseAtTime)
        gamePieceVision = new GamePieceVisionSubsystem(drive);

        // Calibration tab receives only VisionSubsystem — single interface
        calTab = new AprilTagFieldCalTab(vision);

        driveCommand = new DriveWithJoysticks(
            drive, leftStick, rightStick, COR_MAX_X_IN, COR_MAX_Y_IN
        );
        xLock = new XLockCommand(drive);

        drive.configureForAutoBuilder();

        pathfind = new PathfindCommand(
            drive, vision,
            () -> FieldTargets.get("speaker") // TODO: wire to button/selector for target choice
        );

        // TODO: replace stub lambdas with real intake subsystem calls once intake exists
        huntNearest    = new GamePieceHuntCommand(drive, gamePieceVision, HuntMode.NEAREST_PIECE,
                             () -> {}, () -> false);
        huntSequential = new GamePieceHuntCommand(drive, gamePieceVision, HuntMode.SEQUENTIAL_PIECES,
                             () -> {}, () -> false);
        huntCluster    = new GamePieceHuntCommand(drive, gamePieceVision, HuntMode.NEAREST_CLUSTER,
                             () -> {}, () -> false);
        huntSeqCluster = new GamePieceHuntCommand(drive, gamePieceVision, HuntMode.SEQUENTIAL_CLUSTERS,
                             () -> {}, () -> false);

        ConfigVerifier.runAll();

        configureDefaultCommands();
        configureButtonBindings();
    }

    /**
     * Build one swerve module from the hardware types selected in SwerveConstants.
     * Changing DRIVE_MOTOR_TYPE / STEER_MOTOR_TYPE / ABS_ENCODER_TYPE there is the
     * only change needed to swap module hardware.
     */
    private static SwerveModule buildModule(
            String name, int index,
            int driveCanId, int steerCanId, int analogPort, int cancoderCanId,
            double offsetVolts, double offsetRot,
            boolean driveInverted, double[] drivePID, double[] steerPID) {

        MotorIO driveMotor = MotorIO.create(SwerveHardware
            .driveMotorConfig(SwerveConstants.DRIVE_MOTOR_TYPE, driveCanId, driveInverted,
                              drivePID, SwerveConstants.DRIVE_GEAR_RATIO)
            .withCanSignals(SwerveConstants.SIGNAL_UPDATE_HZ, SwerveConstants.SIGNAL_UPDATE_HZ_TEMP,
                            SwerveConstants.OPTIMIZE_CAN_UTILIZATION)
            .build());

        MotorIO steerMotor = MotorIO.create(SwerveHardware
            .steerMotorConfig(SwerveConstants.STEER_MOTOR_TYPE, steerCanId,
                              steerPID, SwerveConstants.STEER_GEAR_RATIO)
            .withCanSignals(SwerveConstants.SIGNAL_UPDATE_HZ, SwerveConstants.SIGNAL_UPDATE_HZ_TEMP,
                            SwerveConstants.OPTIMIZE_CAN_UTILIZATION)
            .build());

        AbsoluteEncoderIO absEncoder = SwerveHardware.buildEncoder(
            SwerveConstants.ABS_ENCODER_TYPE, steerMotor,
            analogPort, cancoderCanId, offsetVolts, offsetRot);

        return new SwerveModule(name, index, driveMotor, steerMotor, absEncoder,
                                SwerveConstants.WHEEL_DIAMETER_INCHES);
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

        // Game piece hunt — TODO: remap to final 2026 driver layout
        huntNearestButton.whileTrue(huntNearest);
        huntSequentialButton.whileTrue(huntSequential);
        huntClusterButton.whileTrue(huntCluster);
        huntSeqClusterButton.whileTrue(huntSeqCluster);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Accessors
    // ─────────────────────────────────────────────────────────────────────────

    public SwerveDrive              getDrive()           { return drive; }
    public VisionSubsystem          getVision()          { return vision; }
    public GamePieceVisionSubsystem getGamePieceVision() { return gamePieceVision; }

    /** Called from Robot.java's robotPeriodic() every loop */
    public void updateCalibrationTab() { calTab.update(); }

    /**
     * Fuses the latest valid vision pose into the swerve drive Kalman filter.
     * Called from Robot.java's robotPeriodic() every loop, after the scheduler runs.
     * No-ops silently when vision has no valid pose — filter continues on odometry alone.
     */
    public void updatePoseEstimator() {
        if (!vision.hasValidPose()) return;
        RobotPoseEstimate best = vision.getBestPose();
        drive.addVisionMeasurement(
            best.pose.toPose2d(),
            best.timestampSecs,
            best.tagCount
        );
    }

    public Command getAutonomousCommand() { return Commands.none(); }
}