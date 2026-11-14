package frc.robot.pathplanning;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.constants.PathPlannerConstants;
import frc.robot.subsystems.swerveDrive.SwerveDrive;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.lib.util.RobotLogger;

import java.util.function.Supplier;

/**
 * PathfindCommand.java
 * PATH: src/main/java/frc/robot/pathplanning/PathfindCommand.java
 *
 * On-the-fly pathfinding to a field target pose using PathPlanner's AD* algorithm.
 * Requires SwerveDrive — interrupts DriveWithJoysticks on schedule, restores it on end.
 *
 * ─── USAGE ───────────────────────────────────────────────────────────────────
 * Bind with .whileTrue() in RobotContainer. Button release ends the command and
 * the scheduler automatically resumes DriveWithJoysticks as the default command.
 *
 * ─── TWO-PHASE NAVIGATION ────────────────────────────────────────────────────
 * Currently uses pathfindToPose() — AD* on-the-fly pathfinding only.
 * TODO: switch to pathfindThenFollowPath() once final-approach .path files are
 * authored in the PathPlanner GUI for each scoring position. The path file name
 * should match the FieldTargets key (e.g. "speaker.path").
 *
 * ─── VISION STALENESS ────────────────────────────────────────────────────────
 * At initialize(), checks whether the most recent accepted vision pose is fresh.
 * If stale (older than VISION_STALENESS_THRESHOLD_S), raises a dashboard Alert.
 * The command still runs — odometry-only navigation is acceptable, just flagged.
 *
 * ─── ALLIANCE ZONE EXCLUSIONS ────────────────────────────────────────────────
 * Dynamic obstacle polygons from PathPlannerConstants.ALLIANCE_ZONE_EXCLUSIONS
 * are injected at initialize(). Currently an empty list — populate in
 * PathPlannerConstants.java once 2026 field zone geometry is published.
 */
public class PathfindCommand extends Command {

    private final SwerveDrive     drive;
    private final VisionSubsystem vision;
    private final Supplier<Pose2d> targetSupplier;

    private final PathConstraints constraints = new PathConstraints(
        PathPlannerConstants.MAX_PATH_VEL_MPS,
        PathPlannerConstants.MAX_PATH_ACCEL_MPS2,
        PathPlannerConstants.MAX_PATH_ANG_VEL_RADPS,
        PathPlannerConstants.MAX_PATH_ANG_ACCEL_RADPS2
    );

    private Command pathfinder = null;
    private boolean unavailable = false;

    // Shown in the dashboard's alerts widget (Elastic: "Alerts" group).
    // Static so several pathfindCommands share one alert instead of stacking duplicates.
    private static final Alert visionAlert =
        new Alert("Pathfinding: vision stale", AlertType.kWarning);

    // ─────────────────────────────────────────────────────────────────────────
    // Constructor
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * @param drive          SwerveDrive subsystem — declared as requirement
     * @param vision         VisionSubsystem — read-only, for staleness check
     * @param targetSupplier Supplies the target Pose2d at initialize() time
     */
    public PathfindCommand(SwerveDrive drive, VisionSubsystem vision,
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
        // PathPlanner failed to load (see SwerveDrive.configureForAutoBuilder) — end immediately
        if (!drive.isPathPlannerAvailable()) {
            pathfinder = null;
            unavailable = true;
            return;
        }
        unavailable = false;

        boolean visionStale = checkVisionStaleness();
        RobotLogger.pathfindVisionStale.append(visionStale);
        RobotLogger.pathfindActive.append(true);

        Pose2d target = targetSupplier.get();
        RobotLogger.pathfindTargetX.append(target.getX());
        RobotLogger.pathfindTargetY.append(target.getY());
        RobotLogger.pathfindTargetHeadingDeg.append(target.getRotation().getDegrees());

        // TODO: replace pathfindToPose with pathfindThenFollowPath once
        // final-approach .path files are authored in PathPlanner GUI.
        pathfinder = AutoBuilder.pathfindToPose(target, constraints);
        CommandScheduler.getInstance().schedule(pathfinder);
    }

    @Override
    public void execute() {
        if (unavailable) return;
        // PathPlanner drives the robot via the AutoBuilder lambdas registered in
        // RobotContainer. Nothing to do here — isFinished() polls for completion.
        Pose2d current = drive.getPose();
        Pose2d target  = targetSupplier.get();
        RobotLogger.pathfindPoseErrorM.append(
            current.getTranslation().getDistance(target.getTranslation()));
    }

    @Override
    public void end(boolean interrupted) {
        RobotLogger.pathfindActive.append(false);
        RobotLogger.pathfindPoseErrorM.append(0.0);
        if (pathfinder != null) {
            pathfinder.cancel();
            pathfinder = null;
        }
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return unavailable || (pathfinder != null && !pathfinder.isScheduled());
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Vision staleness check
    // ─────────────────────────────────────────────────────────────────────────

    /** Returns true if vision is stale or unavailable. */
    private boolean checkVisionStaleness() {
        if (!vision.hasValidPose()) {
            visionAlert.setText("Pathfinding: no valid vision pose — running on odometry only");
            visionAlert.set(true);
            return true;
        }
        double age = Timer.getTimestamp() - vision.getBestPose().timestampSecs;
        if (age > PathPlannerConstants.VISION_STALENESS_THRESHOLD_S) {
            visionAlert.setText(
                String.format("Pathfinding: stale vision pose, %.2fs old (threshold %.2fs) — odometry only",
                    age, PathPlannerConstants.VISION_STALENESS_THRESHOLD_S));
            visionAlert.set(true);
            return true;
        }
        visionAlert.set(false);
        return false;
    }
}
