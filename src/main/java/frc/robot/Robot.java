package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.lib.util.Dashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.vision.VisionHealthMonitor.HealthStatus;
import frc.lib.util.RobotLogger;

/**
 * Top-level robot lifecycle class. All periodic calls flow from here.
 *
 * <h2>Periodic call order (robotPeriodic, every 20ms)</h2>
 *
 * <pre>{@code
 * 1. CommandScheduler.run()      — runs subsystem periodic() + all active commands
 * 2. updatePoseEstimator()       — fuses latest vision pose into Kalman filter
 * 3. updateCalibrationTab()      — updates field calibration Elastic tab
 * }</pre>
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

        // Serve the deploy folder over HTTP on port 5800 (allowed on FRC fields) so
        // Elastic on the drive laptop can pull elastic-layout.json with
        // File > Load Layout From Robot (Ctrl+D).
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

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
