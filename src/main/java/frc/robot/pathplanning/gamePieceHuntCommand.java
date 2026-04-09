package frc.robot.pathplanning;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.constants.gamePieceConstants;
import frc.robot.constants.pathplannerConstants;
import frc.robot.subsystems.swerveDrive.swerveDrive;
import frc.robot.subsystems.vision.gamePieceVisionSubsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

/**
 * gamePieceHuntCommand.java
 * PATH: src/main/java/frc/robot/pathplanning/gamePieceHuntCommand.java
 *
 * Autonomously navigates to and collects game pieces detected by the intake camera.
 * Uses PathPlanner AD* pathfinding for navigation and a state machine to sequence
 * through the four hunt modes.
 *
 * ─── HUNT MODES ──────────────────────────────────────────────────────────────
 * NEAREST_PIECE       — drive to the nearest piece, trigger intake, stop.
 * SEQUENTIAL_PIECES   — intake nearest piece, then next nearest, repeat until none.
 * NEAREST_CLUSTER     — drive to the centroid of the nearest cluster, intake all, stop.
 * SEQUENTIAL_CLUSTERS — intake nearest cluster, then next, repeat until none.
 *
 * ─── STATE MACHINE ───────────────────────────────────────────────────────────
 *   FINDING      — poll gamePieceVision for a target.
 *                  No piece found within HUNT_NO_PIECE_TIMEOUT_SECS → DONE.
 *                  Piece found → compute arrival Pose2d, start pathfinder → PATHFINDING.
 *   PATHFINDING  — inner AutoBuilder command runs via CommandScheduler.
 *                  Pathfinder finishes → trigger intakeTrigger → INTAKING.
 *   INTAKING     — wait for intakeComplete or INTAKE_TIMEOUT_SECS.
 *                  NEAREST_PIECE / NEAREST_CLUSTER → DONE.
 *                  SEQUENTIAL_* → mark target consumed → FINDING.
 *   DONE         — isFinished() returns true.
 *
 * ─── ARRIVAL HEADING ─────────────────────────────────────────────────────────
 * The target Pose2d heading is computed so the robot's intake side faces the
 * game piece as it drives over it:
 *
 *   targetHeading = atan2(pieceY − robotY, pieceX − robotX) + APPROACH_HEADING_OFFSET_DEG
 *
 * APPROACH_HEADING_OFFSET_DEG in gamePieceConstants compensates for intakes
 * not exactly on the robot's forward axis.
 *
 * ─── INTAKE INTERFACE ────────────────────────────────────────────────────────
 * The command is decoupled from any specific intake subsystem. It takes:
 *   intakeTrigger  — Runnable called once when the robot arrives at the piece.
 *   intakeComplete — BooleanSupplier that returns true when the piece is collected.
 *
 * Wire these in RobotContainer once the intake subsystem exists.
 *
 * ─── REQUIREMENTS ────────────────────────────────────────────────────────────
 * Requires swerveDrive — preempts driveWithJoysticks while active.
 * gamePieceVisionSubsystem is read-only (not declared as a requirement).
 */
public class gamePieceHuntCommand extends Command {

    // ── Mode ──────────────────────────────────────────────────────────────────

    /** Determines how many pieces / clusters to collect before stopping. */
    public enum HuntMode {
        /** Navigate to the nearest single piece, intake it, then stop. */
        NEAREST_PIECE,
        /** Intake the nearest piece, then find and intake the next, repeat until none. */
        SEQUENTIAL_PIECES,
        /** Navigate to the centroid of the nearest cluster, intake all pieces, then stop. */
        NEAREST_CLUSTER,
        /** Intake the nearest cluster, then the next, repeat until none. */
        SEQUENTIAL_CLUSTERS
    }

    // ── State machine ─────────────────────────────────────────────────────────

    private enum State { FINDING, PATHFINDING, INTAKING, DONE }

    // ── Dependencies ──────────────────────────────────────────────────────────
    private final swerveDrive                drive;
    private final gamePieceVisionSubsystem   gamePieceVision;
    private final HuntMode                   mode;
    private final Runnable                   intakeTrigger;
    private final BooleanSupplier            intakeComplete;

    // ── PathPlanner ───────────────────────────────────────────────────────────
    private final PathConstraints constraints = new PathConstraints(
        pathplannerConstants.MAX_PATH_VEL_MPS,
        pathplannerConstants.MAX_PATH_ACCEL_MPS2,
        pathplannerConstants.MAX_PATH_ANG_VEL_RADPS,
        pathplannerConstants.MAX_PATH_ANG_ACCEL_RADPS2
    );

    // ── Runtime state ─────────────────────────────────────────────────────────
    private State   state          = State.FINDING;
    private Command pathfinder     = null;

    /** Translation of the current target piece/centroid (for consumption tracking). */
    private Translation2d currentTarget = null;

    /**
     * Positions consumed this run — excluded from future FINDING queries.
     * A consumed position is matched by proximity (CLUSTER_RADIUS_M / 3).
     */
    private final List<Translation2d> consumedTargets = new ArrayList<>();

    /** FPGA timestamp when the current state was entered — used for timeouts. */
    private double stateEnteredTimestamp = 0.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Constructor
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * @param drive           swerveDrive subsystem (declared as requirement)
     * @param gamePieceVision game piece detection subsystem (read-only)
     * @param mode            one of the four HuntMode values
     * @param intakeTrigger   called once when the robot arrives at a piece;
     *                        wire to intake.run() or similar in RobotContainer
     * @param intakeComplete  returns true when the current piece has been secured;
     *                        wire to intake.hasGamePiece() or similar
     */
    public gamePieceHuntCommand(swerveDrive drive,
                                 gamePieceVisionSubsystem gamePieceVision,
                                 HuntMode mode,
                                 Runnable intakeTrigger,
                                 BooleanSupplier intakeComplete) {
        this.drive          = drive;
        this.gamePieceVision = gamePieceVision;
        this.mode           = mode;
        this.intakeTrigger  = intakeTrigger;
        this.intakeComplete = intakeComplete;
        addRequirements(drive);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Command lifecycle
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void initialize() {
        state          = State.FINDING;
        currentTarget  = null;
        consumedTargets.clear();
        pathfinder     = null;
        stateEnteredTimestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        switch (state) {
            case FINDING    -> executeFinding();
            case PATHFINDING -> executePathfinding();
            case INTAKING   -> executeIntaking();
            case DONE       -> {}
        }
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
        return state == State.DONE;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // State handlers
    // ─────────────────────────────────────────────────────────────────────────

    private void executeFinding() {
        // Timeout — give up if no piece found within the configured window
        if (Timer.getFPGATimestamp() - stateEnteredTimestamp
                > gamePieceConstants.HUNT_NO_PIECE_TIMEOUT_SECS) {
            transitionTo(State.DONE);
            return;
        }

        Pose2d robotPose = drive.getPose();
        Optional<Translation2d> target = findTarget(robotPose);
        if (target.isEmpty()) return;  // keep polling

        currentTarget = target.get();

        // Compute arrival pose: robot heading so intake faces the piece
        double dx = currentTarget.getX() - robotPose.getX();
        double dy = currentTarget.getY() - robotPose.getY();
        double headingRad = Math.atan2(dy, dx)
            + Math.toRadians(gamePieceConstants.APPROACH_HEADING_OFFSET_DEG);

        Pose2d targetPose = new Pose2d(
            currentTarget.getX(),
            currentTarget.getY(),
            new Rotation2d(headingRad)
        );

        pathfinder = AutoBuilder.pathfindToPose(targetPose, constraints);
        CommandScheduler.getInstance().schedule(pathfinder);
        transitionTo(State.PATHFINDING);
    }

    private void executePathfinding() {
        // Wait for PathPlanner's inner command to finish
        if (pathfinder != null && !pathfinder.isScheduled()) {
            pathfinder = null;
            intakeTrigger.run();
            transitionTo(State.INTAKING);
        }
    }

    private void executeIntaking() {
        boolean complete = intakeComplete.getAsBoolean();
        boolean timedOut = Timer.getFPGATimestamp() - stateEnteredTimestamp
                           > gamePieceConstants.INTAKE_TIMEOUT_SECS;

        if (!complete && !timedOut) return;

        // Mark this target as consumed so FINDING skips it next time
        if (currentTarget != null) consumedTargets.add(currentTarget);

        switch (mode) {
            case NEAREST_PIECE, NEAREST_CLUSTER -> transitionTo(State.DONE);
            case SEQUENTIAL_PIECES, SEQUENTIAL_CLUSTERS -> transitionTo(State.FINDING);
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Target selection
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Returns the next target Translation2d based on the current hunt mode,
     * excluding previously consumed targets. Empty if nothing is available.
     */
    private Optional<Translation2d> findTarget(Pose2d robotPose) {
        return switch (mode) {
            case NEAREST_PIECE, SEQUENTIAL_PIECES -> findNearestPiece(robotPose);
            case NEAREST_CLUSTER, SEQUENTIAL_CLUSTERS -> findNearestClusterCentroid(robotPose);
        };
    }

    /** Returns the nearest unconsumned piece position. */
    private Optional<Translation2d> findNearestPiece(Pose2d robotPose) {
        return gamePieceVision.getFieldRelativePieces().stream()
            .map(p -> p.fieldPos)
            .filter(pos -> !isConsumed(pos))
            .min((a, b) -> Double.compare(
                a.getDistance(robotPose.getTranslation()),
                b.getDistance(robotPose.getTranslation())));
    }

    /** Returns the centroid of the nearest cluster whose centroid is not consumed. */
    private Optional<Translation2d> findNearestClusterCentroid(Pose2d robotPose) {
        List<List<Translation2d>> clusters = gamePieceVision.getClusters(robotPose);
        for (List<Translation2d> cluster : clusters) {
            Translation2d centroid = centroid(cluster);
            if (!isConsumed(centroid)) return Optional.of(centroid);
        }
        return Optional.empty();
    }

    /** True if the given position is close enough to a consumed target to be skipped. */
    private boolean isConsumed(Translation2d pos) {
        double mergeRadius = gamePieceConstants.CLUSTER_RADIUS_M / 3.0;
        for (Translation2d consumed : consumedTargets) {
            if (consumed.getDistance(pos) <= mergeRadius) return true;
        }
        return false;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Helpers
    // ─────────────────────────────────────────────────────────────────────────

    private void transitionTo(State next) {
        state = next;
        stateEnteredTimestamp = Timer.getFPGATimestamp();
    }

    private static Translation2d centroid(List<Translation2d> cluster) {
        double x = 0, y = 0;
        for (Translation2d p : cluster) { x += p.getX(); y += p.getY(); }
        return new Translation2d(x / cluster.size(), y / cluster.size());
    }
}
