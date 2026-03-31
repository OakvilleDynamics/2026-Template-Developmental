package frc.robot.pathplanning;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.constants.pathplannerConstants;
import frc.robot.subsystems.swerveDrive.swerveDrive;
import frc.robot.subsystems.vision.visionSubsystem;

import java.util.function.Supplier;

/**
 * pathfindCommand.java
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
 * authored in the PathPlanner GUI for each scoring position. The path file name
 * should match the FieldTargets key (e.g. "speaker.path").
 *
 * ─── VISION STALENESS ────────────────────────────────────────────────────────
 * At initialize(), checks whether the most recent accepted vision pose is fresh.
 * If stale (older than VISION_STALENESS_THRESHOLD_S), logs a Shuffleboard warning.
 * The command still runs — odometry-only navigation is acceptable, just flagged.
 *
 * ─── ALLIANCE ZONE EXCLUSIONS ────────────────────────────────────────────────
 * Dynamic obstacle polygons from pathplannerConstants.ALLIANCE_ZONE_EXCLUSIONS
 * are injected at initialize(). Currently an empty list — populate in
 * pathplannerConstants.java once 2026 field zone geometry is published.
 */
public class pathfindCommand extends Command {

    private final swerveDrive     drive;
    private final visionSubsystem vision;
    private final Supplier<Pose2d> targetSupplier;

    private final PathConstraints constraints = new PathConstraints(
        pathplannerConstants.MAX_PATH_VEL_MPS,
        pathplannerConstants.MAX_PATH_ACCEL_MPS2,
        pathplannerConstants.MAX_PATH_ANG_VEL_RADPS,
        pathplannerConstants.MAX_PATH_ANG_ACCEL_RADPS2
    );

    private Command pathfinder = null;

    private final ShuffleboardTab tab = Shuffleboard.getTab("Pathfinding");

    // ─────────────────────────────────────────────────────────────────────────
    // Constructor
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * @param drive          swerveDrive subsystem — declared as requirement
     * @param vision         visionSubsystem — read-only, for staleness check
     * @param targetSupplier Supplies the target Pose2d at initialize() time
     */
    public pathfindCommand(swerveDrive drive, visionSubsystem vision,
                           Supplier<Pose2d> targetSupplier) {
        this.drive          = drive;
        this.vision         = vision;
        this.targetSupplier = targetSupplier;
        addRequirements(drive);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Command lifecycle
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void initialize() {
        checkVisionStaleness();

        Pose2d target = targetSupplier.get();

        // TODO: replace pathfindToPose with pathfindThenFollowPath once
        // final-approach .path files are authored in PathPlanner GUI.
        pathfinder = AutoBuilder.pathfindToPose(target, constraints);
        CommandScheduler.getInstance().schedule(pathfinder);
    }

    @Override
    public void execute() {
        // PathPlanner drives the robot via the AutoBuilder lambdas registered in
        // RobotContainer. Nothing to do here — isFinished() polls for completion.
    }

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

    // ─────────────────────────────────────────────────────────────────────────
    // Vision staleness check
    // ─────────────────────────────────────────────────────────────────────────

    private void checkVisionStaleness() {
        if (!vision.hasValidPose()) {
            tab.add("Vision Warning", "No valid pose — running on odometry only");
            return;
        }
        double age = Timer.getFPGATimestamp() - vision.getBestPose().timestampSecs;
        if (age > pathplannerConstants.VISION_STALENESS_THRESHOLD_S) {
            tab.add("Vision Warning",
                String.format("Stale pose: %.2fs old (threshold %.2fs) — odometry only",
                    age, pathplannerConstants.VISION_STALENESS_THRESHOLD_S));
        }
    }
}
