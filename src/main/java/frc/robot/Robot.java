package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.RobotLogger;

/**
 * Robot.java
 * PATH: src/main/java/frc/robot/Robot.java
 *
 * Top-level robot lifecycle class. All periodic calls flow from here.
 *
 * ─── PERIODIC CALL ORDER (robotPeriodic, every 20ms) ─────────────────────────
 * 1. CommandScheduler.run()  — runs subsystem periodic() + all active commands
 */
public class Robot extends TimedRobot {

    private RobotContainer robotContainer;
    private Command        autoCommand;

    @Override
    public void robotInit() {
        SignalLogger.setPath("/u/");
        SignalLogger.start();
        RobotLogger.init();
        robotContainer = new RobotContainer();
        SmartDashboard.putString("Robot/Status", "Initializing");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        SmartDashboard.putString("Robot/Status", "Disabled");
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        SmartDashboard.putString("Robot/Status", "Autonomous");
        autoCommand = robotContainer.getAutonomousCommand();
        if (autoCommand != null) CommandScheduler.getInstance().schedule(autoCommand);
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
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
