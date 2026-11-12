package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.lib.util.Dashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.vision.visionHealthMonitor.HealthStatus;
import frc.lib.util.RobotLogger;

/**
 * Robot.java
 * PATH: src/main/java/frc/robot/Robot.java
 *
 * Top-level robot lifecycle class. All periodic calls flow from here.
 *
 * ─── PERIODIC CALL ORDER (robotPeriodic, every 20ms) ─────────────────────────
 * 1. CommandScheduler.run()      — runs subsystem periodic() + all active commands
 * 2. updatePoseEstimator()       — fuses latest vision pose into Kalman filter
 * 3. updateCalibrationTab()      — updates field calibration Elastic tab
 */
public class Robot extends TimedRobot {

    private RobotContainer robotContainer;
    private Command        autoCommand;

    /**
     * Robot-wide initialization. WPILib recommends the constructor over robotInit()
     * (robotInit() is removed in WPILib 2027).
     */
    public Robot() {
        // Start loggers before constructing subsystems so no early signals are missed.
        // SignalLogger writes Phoenix 6 motor signals (.hoot) to USB if present.
        SignalLogger.setPath("/u/");
        SignalLogger.start();

        // WPILib DataLog — computed quantities, vision, pathfinding (.wpilog).
        // Falls back to roboRIO internal storage if no USB detected.
        RobotLogger.init();

        robotContainer = new RobotContainer();
        Dashboard.log("Robot/Status", "Initializing");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        robotContainer.updatePoseEstimator();
        robotContainer.updateCalibrationTab();
    }

    @Override
    public void disabledInit() {
        Dashboard.log("Robot/Status", "Disabled");
    }

    @Override
    public void disabledPeriodic() {
        HealthStatus health = robotContainer.getVision().getHealthStatus();
        Dashboard.log("Robot/Vision Health", health.name());
        Dashboard.log("Robot/Vision Ready", health == HealthStatus.GOOD);
        robotContainer.getGamePieceVision().publishHealthStatus();
    }

    @Override
    public void autonomousInit() {
        robotContainer.getVision().clearExpectedStartPose();
        Dashboard.log("Robot/Status", "Autonomous");
        autoCommand = robotContainer.getAutonomousCommand();
        if (autoCommand != null) CommandScheduler.getInstance().schedule(autoCommand);
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        robotContainer.getVision().clearExpectedStartPose();
        Dashboard.log("Robot/Status", "Teleop");
        if (autoCommand != null) autoCommand.cancel();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        Dashboard.log("Robot/Status", "Test");
    }

    @Override
    public void testPeriodic() {}
}
