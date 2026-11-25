package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerveDrive.SwerveDrive;

/**
 * Sets all four swerve modules to an X brace pattern and holds zero
 * drive speed. The robot resists being pushed from any direction.
 *
 * <p>Bound as whileTrue() — active only while the button is held.
 * Releasing cancels this command and the default drive command resumes.
 *
 * <p>Wheel angles (robot-relative):
 *
 * <pre>{@code
 * FL:  45°  FR: -45°
 *     \       /
 *      X     X
 *     /       \
 * BL: -45°  BR:  45°
 * }</pre>
 */
public class XLockCommand extends Command {

    private final SwerveDrive drive;

    public XLockCommand(SwerveDrive drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.lockWheelsX();
    }

    @Override
    public void execute() {
        // Re-command every loop — prevents any input from overriding the lock
        drive.lockWheelsX();
    }

    @Override
    public void end(boolean interrupted) {
        // Nothing to clean up — default drive command resumes automatically
    }

    @Override
    public boolean isFinished() {
        return false; // runs until button is released
    }
}