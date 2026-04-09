package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.vision.visionHealthMonitor.HealthStatus;
import frc.robot.util.RobotLogger;

/**
 * Robot.java
 * PATH: src/main/java/frc/robot/Robot.java
 *
 * Top-level robot lifecycle class. All periodic calls flow from here.
 *
 * ─── PERIODIC CALL ORDER (robotPeriodic, every 20ms) ─────────────────────────
 * 1. CommandScheduler.run()      — runs subsystem periodic() + all active commands
 * 2. updatePoseEstimator()       — fuses latest vision pose into Kalman filter
 * 3. updateCalibrationTab()      — updates field calibration Shuffleboard tab
 */
public class Robot extends TimedRobot {

    private RobotContainer robotContainer;
    private Command        autoCommand;

    @Override
    public void robotInit() {
        // Start loggers before constructing subsystems so no early signals are missed.
        // SignalLogger writes Phoenix 6 motor signals (.hoot) to USB if present.
        SignalLogger.setPath("/u/");
        SignalLogger.start();

        // WPILib DataLog — computed quantities, vision, pathfinding (.wpilog).
        // Falls back to roboRIO internal storage if no USB detected.
        RobotLogger.init();

        robotContainer = new RobotContainer();
        SmartDashboard.putString("Robot/Status", "Initializing");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        robotContainer.updatePoseEstimator();
        robotContainer.updateCalibrationTab();
    }

    @Override
    public void disabledInit() {
        SmartDashboard.putString("Robot/Status", "Disabled");
    }

    @Override
    public void disabledPeriodic() {
        HealthStatus health = robotContainer.getVision().getHealthStatus();
        SmartDashboard.putString("Robot/Vision Health", health.name());
        SmartDashboard.putBoolean("Robot/Vision Ready", health == HealthStatus.GOOD);
        robotContainer.getGamePieceVision().publishHealthStatus();
    }

    @Override
    public void autonomousInit() {
        robotContainer.getVision().clearExpectedStartPose();
        SmartDashboard.putString("Robot/Status", "Autonomous");
        autoCommand = robotContainer.getAutonomousCommand();
        if (autoCommand != null) CommandScheduler.getInstance().schedule(autoCommand);
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        robotContainer.getVision().clearExpectedStartPose();
        SmartDashboard.putString("Robot/Status", "Teleop");
        if (autoCommand != null) autoCommand.cancel();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        SmartDashboard.putString("Robot/Status", "Test");
    }

    @Override
    public void testPeriodic() {}
}
