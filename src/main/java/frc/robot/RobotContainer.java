package frc.robot;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.constants.CANIDs;
import frc.robot.constants.PortIDs;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.XLockCommand;
import frc.robot.constants.SwerveConstants;
import frc.robot.controls.IDriverInputs;
import frc.robot.controls.FlightStickInputs;
import frc.robot.controls.XboxInputs;
import frc.robot.pathplanning.FieldTargets;
import frc.robot.pathplanning.GamePieceHuntCommand;
import frc.robot.pathplanning.GamePieceHuntCommand.HuntMode;
import frc.robot.pathplanning.PathfindCommand;
import frc.robot.subsystems.swerveDrive.SwerveDrive;
import frc.lib.hardware.encoder.IAbsoluteEncoderIO;
import frc.lib.hardware.gyro.CTREPigeon2Gyro;
import frc.lib.hardware.motor.IMotorIO;
import frc.lib.swerve.SwerveHardware;
import frc.lib.swerve.SwerveModule;
import frc.lib.util.ConfigVerifier;
import frc.robot.subsystems.vision.AprilTagFieldCalTab;
import frc.robot.subsystems.vision.GamePieceVisionSubsystem;
import frc.robot.subsystems.vision.RobotPoseEstimate;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * <h2>Season change — one line</h2>
 *
 * <p>Update GAME_YEAR_FIELD each season. Propagates to VisionSubsystem →
 * AprilTagFieldCal → PhotonPoseEstimators → everywhere.
 *
 * <h2>Event change — AprilTagIgnore.java</h2>
 *
 * <p>To ignore a problem tag at a specific event, edit AprilTagIgnore.java
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
     * <pre>{@code
     * 2026 REBUILT:   AprilTagFields.kDefaultField (update when constant published)
     * 2025 Reefscape: AprilTagFields.k2025Reefscape
     * 2024 Crescendo: AprilTagFields.k2024Crescendo
     * }</pre>
     */
    private static final AprilTagFields GAME_YEAR_FIELD = AprilTagFields.kDefaultField;

    //==========================================================================
    // Robot geometry — all in INCHES
    //==========================================================================


    // Thrifty Narrow pivot center is 2.5" from outer frame edge on each side (verified from CAD).
    // Wheel base = 27.0 - 2 × 2.5 = 22.0"
    private static final double[] WHEEL_BASE_IN = { 24.0, 24.0 };
    private static final double[] FRAME_IN      = { 27.0, 27.0 };
    private static final double[] BUMPER_IN     = { 33.0, 33.0 };
    private static final double   WHEEL_DIAM_IN = SwerveConstants.WHEEL_DIAMETER_INCHES;

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
    //
    // Drive gains run on the TalonFX in WHEEL units (SensorToMechanismRatio =
    // DRIVE_GEAR_RATIO), so kV is volts per wheel rot/s:
    //   Kraken X60 free speed ≈ 100 motor rot/s ÷ 5.79 ≈ 17.3 wheel rot/s
    //   kV ≈ 12 V ÷ 17.3 ≈ 0.7 (≈ 0.72 with FOC). Tune at the robot.
    //
    // Steer gains are for a Thrifty Nova (STEER_MOTOR_TYPE = NOVA), NOT a TalonFX:
    //   - Nova PID runs on MOTOR rotations (before STEER_GEAR_RATIO), so gains
    //     are much smaller than TalonFX/Phoenix 6 steer gains.
    //   - kS goes to Nova's setFF(), a feedforward on the setpoint, not a
    //     static-friction term. Keep it 0 for position control.
    //   - kV is ignored by the Nova.
    // Bring-up: start kP ≈ 0.1, raise until the wheels snap without oscillating,
    // then add kD. Tunable live under Swerve/<module>/PID/.
    //==========================================================================

    private static final double[] FL_DRIVE_PID = { 0.1, 0.0, 0.0, 0.15, 0.7, 0.01 };
    private static final double[] FL_STEER_PID = { 0.1, 0.0, 0.0, 0.0, 0.0 };
    private static final double[] FR_DRIVE_PID = { 0.1, 0.0, 0.0, 0.15, 0.7, 0.01 };
    private static final double[] FR_STEER_PID = { 0.1, 0.0, 0.0, 0.0, 0.0 };
    private static final double[] BL_DRIVE_PID = { 0.1, 0.0, 0.0, 0.15, 0.7, 0.01 };
    private static final double[] BL_STEER_PID = { 0.1, 0.0, 0.0, 0.0, 0.0 };
    private static final double[] BR_DRIVE_PID = { 0.1, 0.0, 0.0, 0.15, 0.7, 0.01 };
    private static final double[] BR_STEER_PID = { 0.1, 0.0, 0.0, 0.0, 0.0 };

    //==========================================================================
    // Controllers
    //==========================================================================

    /** Which controller layout the driver uses. */
    private enum DriverController {
        /** Two Logitech Extreme 3D Pro flight sticks on USB ports 0 and 1. */
        FLIGHT_STICKS,
        /**
         * One Xbox-style controller on USB port 0: left stick drives, right
         * stick X rotates. Driving only — no button bindings yet.
         */
        XBOX
    }

    /** Change this to switch the driver's controller layout. */
    private static final DriverController DRIVER_CONTROLLER = DriverController.FLIGHT_STICKS;

    // Driver Station USB ports
    private static final int LEFT_STICK_PORT  = 0;
    private static final int RIGHT_STICK_PORT = 1;
    private static final int XBOX_PORT        = 0;

    //==========================================================================
    // Subsystems, commands, calibration tab
    //==========================================================================

    private final SwerveDrive              drive;
    private final IDriverInputs             driverInputs;
    private final VisionSubsystem          vision;
    private final GamePieceVisionSubsystem gamePieceVision;
    private final TeleopDrive              driveCommand;
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

        SwerveModule flModule = buildModule("FL", 0,
            CANIDs.FL_DRIVE,                 CANIDs.FL_STEER,
            PortIDs.AI_FL_STEER_ENCODER,     CANIDs.FL_CANCODER_UNUSED,
            SwerveConstants.FL_STEER_OFFSET_ROT,
            SwerveConstants.FL_DRIVE_INVERTED, FL_DRIVE_PID, FL_STEER_PID);

        SwerveModule frModule = buildModule("FR", 1,
            CANIDs.FR_DRIVE,                 CANIDs.FR_STEER,
            PortIDs.AI_FR_STEER_ENCODER,     CANIDs.FR_CANCODER_UNUSED,
            SwerveConstants.FR_STEER_OFFSET_ROT,
            SwerveConstants.FR_DRIVE_INVERTED, FR_DRIVE_PID, FR_STEER_PID);

        SwerveModule blModule = buildModule("BL", 2,
            CANIDs.BL_DRIVE,                 CANIDs.BL_STEER,
            PortIDs.AI_BL_STEER_ENCODER,     CANIDs.BL_CANCODER_UNUSED,
            SwerveConstants.BL_STEER_OFFSET_ROT,
            SwerveConstants.BL_DRIVE_INVERTED, BL_DRIVE_PID, BL_STEER_PID);

        SwerveModule brModule = buildModule("BR", 3,
            CANIDs.BR_DRIVE,                 CANIDs.BR_STEER,
            PortIDs.AI_BR_STEER_ENCODER,     CANIDs.BR_CANCODER_UNUSED,
            SwerveConstants.BR_STEER_OFFSET_ROT,
            SwerveConstants.BR_DRIVE_INVERTED, BR_DRIVE_PID, BR_STEER_PID);

        drive = new SwerveDrive(
            new SwerveModule[]{ flModule, frModule, blModule, brModule },
            new CTREPigeon2Gyro(CANIDs.PIGEON2),
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

        driverInputs = switch (DRIVER_CONTROLLER) {
            case FLIGHT_STICKS -> new FlightStickInputs(
                new Joystick(LEFT_STICK_PORT), new Joystick(RIGHT_STICK_PORT));
            case XBOX -> new XboxInputs(new XboxController(XBOX_PORT));
        };
        driveCommand = new TeleopDrive(drive, driverInputs, COR_MAX_X_IN, COR_MAX_Y_IN);
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
            double offsetRot,
            boolean driveInverted, double[] drivePID, double[] steerPID) {

        IMotorIO driveMotor = IMotorIO.create(SwerveHardware
            .driveMotorConfig(SwerveConstants.DRIVE_MOTOR_TYPE, driveCanId, driveInverted,
                              drivePID, SwerveConstants.DRIVE_GEAR_RATIO)
            .withCanSignals(SwerveConstants.SIGNAL_UPDATE_HZ, SwerveConstants.SIGNAL_UPDATE_HZ_TEMP,
                            SwerveConstants.OPTIMIZE_CAN_UTILIZATION)
            .build());

        IMotorIO steerMotor = IMotorIO.create(SwerveHardware
            .steerMotorConfig(SwerveConstants.STEER_MOTOR_TYPE, steerCanId,
                              SwerveConstants.STEER_MOTOR_INVERTED,
                              steerPID, SwerveConstants.STEER_GEAR_RATIO)
            .withCanSignals(SwerveConstants.SIGNAL_UPDATE_HZ, SwerveConstants.SIGNAL_UPDATE_HZ_TEMP,
                            SwerveConstants.OPTIMIZE_CAN_UTILIZATION)
            .build());

        IAbsoluteEncoderIO absEncoder = SwerveHardware.buildEncoder(
            SwerveConstants.ABS_ENCODER_TYPE, steerMotor,
            analogPort, cancoderCanId, offsetRot,
            SwerveConstants.ABS_ENCODER_INVERTED);

        return new SwerveModule(name, index, driveMotor, steerMotor, absEncoder,
                                SwerveConstants.WHEEL_DIAMETER_INCHES);
    }

    private void configureDefaultCommands() {
        drive.setDefaultCommand(driveCommand);
    }

    private void configureButtonBindings() {
        // Zero the heading: point the robot away from the driver, then press.
        // Works while disabled. No subsystem requirement, so driving isn't interrupted.
        Command zeroHeading = Commands.runOnce(drive::zeroYaw).ignoringDisable(true);

        if (driverInputs instanceof XboxInputs xbox) {
            // TODO: add the rest of the Xbox button bindings.
            new JoystickButton(xbox.controller(), XboxController.Button.kStart.value)
                .onTrue(zeroHeading);
            return;
        }

        if (!(driverInputs instanceof FlightStickInputs sticks)) return;

        new JoystickButton(sticks.leftStick(), 2).onTrue(zeroHeading);

        Joystick rightStick = sticks.rightStick();

        JoystickButton lockToTargetButton   = new JoystickButton(rightStick, 2);
        JoystickButton xLockButton          = new JoystickButton(rightStick, 3);
        JoystickButton pathfindButton       = new JoystickButton(rightStick, 4);

        // Game piece hunt — TODO: remap to final 2026 driver layout
        JoystickButton huntNearestButton    = new JoystickButton(rightStick, 5);
        JoystickButton huntSequentialButton = new JoystickButton(rightStick, 6);
        JoystickButton huntClusterButton    = new JoystickButton(rightStick, 7);
        JoystickButton huntSeqClusterButton = new JoystickButton(rightStick, 8);

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