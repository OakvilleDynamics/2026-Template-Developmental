package frc.robot.pathplanning;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.constants.pathplannerConstants;
import frc.robot.subsystems.swerveDrive.swerveDrive;

import java.util.function.Supplier;

/**
 * pathfindCommand.java — drivetrain-only branch (test/swerve)
 * PATH: src/main/java/frc/robot/pathplanning/pathfindCommand.java
 *
 * On-the-fly pathfinding to a field target pose using PathPlanner's AD* algorithm.
 * Requires swerveDrive — interrupts driveWithJoysticks on schedule, restores it on end.
 *
 * ─── USAGE ───────────────────────────────────────────────────────────────────
 * Bind with .whileTrue() in RobotContainer. Button release ends the command and
 * the scheduler automatically resumes driveWithJoysticks as the default command.
 *
 * ─── TWO-PHASE NAVIGATION ────────────────────────────────────────────────────
 * Currently uses pathfindToPose() — AD* on-the-fly pathfinding only.
 * TODO: switch to pathfindThenFollowPath() once final-approach .path files are
 * authored in the PathPlanner GUI for each scoring position.
 */
public class pathfindCommand extends Command {

    private final swerveDrive      drive;
    private final Supplier<Pose2d> targetSupplier;

    private final PathConstraints constraints = new PathConstraints(
        pathplannerConstants.MAX_PATH_VEL_MPS,
        pathplannerConstants.MAX_PATH_ACCEL_MPS2,
        pathplannerConstants.MAX_PATH_ANG_VEL_RADPS,
        pathplannerConstants.MAX_PATH_ANG_ACCEL_RADPS2
    );

    private Command pathfinder = null;

    // ─────────────────────────────────────────────────────────────────────────
    // Constructor
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * @param drive          swerveDrive subsystem — declared as requirement
     * @param targetSupplier Supplies the target Pose2d at initialize() time
     */
    public pathfindCommand(swerveDrive drive, Supplier<Pose2d> targetSupplier) {
        this.drive          = drive;
        this.targetSupplier = targetSupplier;
        addRequirements(drive);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Command lifecycle
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void initialize() {
        Pose2d target = targetSupplier.get();
        // TODO: replace pathfindToPose with pathfindThenFollowPath once
        // final-approach .path files are authored in PathPlanner GUI.
        pathfinder = AutoBuilder.pathfindToPose(target, constraints);
        CommandScheduler.getInstance().schedule(pathfinder);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        if (pathfinder != null) {
            pathfinder.cancel();
            pathfinder = null;
        }
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return pathfinder != null && !pathfinder.isScheduled();
    }
}
