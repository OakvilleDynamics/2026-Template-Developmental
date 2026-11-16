package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants;
import frc.robot.controls.IDriverInputs;
import frc.lib.swerve.DriveInput;
import frc.robot.subsystems.swerveDrive.SwerveDrive;
import frc.lib.mechanisms.shooter.IAimDriveModes;
import frc.lib.util.Dashboard;
import frc.lib.util.Units;

/**
 * TeleopDrive.java
 * PATH: src/main/java/frc/robot/commands/TeleopDrive.java
 *
 * Default drive command. Runs continuously while no other command requires
 * the SwerveDrive subsystem.
 *
 * ─── DRIVER INPUT ────────────────────────────────────────────────────────────
 * Reads the driver's axes through IDriverInputs, so this command does not know
 * which controller is attached. Controller mappings live in frc.robot.controls:
 *   FlightStickInputs — two Logitech Extreme 3D Pros (with CoR control)
 *   XboxInputs        — one Xbox-style controller (no CoR control)
 *
 *   forward() → vx    — forward/back, field-relative (ft/s)
 *   left()    → vy    — left/right strafe, field-relative (ft/s)
 *   rotate()  → omega — robot rotation rate (rad/s)
 *   corForward() / corLeft() → center of rotation (robot-relative)
 *
 * ─── CENTER OF ROTATION ──────────────────────────────────────────────────────
 * corForward()/corLeft() map linearly to CoR bounds (bumper half-dimensions).
 * Full deflection = CoR at bumper corner in that direction.
 * Stick within deadband = CoR at robot center (0,0) = pivot in place.
 * CoR is always robot-relative — independent of field heading.
 *
 * ─── DRIVE MODES ─────────────────────────────────────────────────────────────
 * NORMAL           — full driver control (default)
 * POINT_AT         — heading PID locks to a field target; translation free
 * HEADING_BOUND    — driver controls heading freely within [min, max] window;
 *                    PID-holds to nearest bound if heading exits the window
 * STATIC_SHOT_LOCK — wheels X-braced, no translation; for stationary shooting
 *
 * The ShooterAimController sets the active mode each loop via the enable/clear
 * methods. This is the external driver-input pipeline — driver inputs are read
 * every execute() but the output path is governed by the externally-set mode.
 *
 * ─── INPUT SHAPING ───────────────────────────────────────────────────────────
 * squareInput() squares magnitude while preserving sign.
 * Gives finer low-speed control without losing full power at full deflection.
 * Translation and rotation are then multiplied by the live-tunable teleop
 * scales ("Drive/Teleop Speed Scale", "Drive/Teleop Rotation Scale").
 */
public class TeleopDrive extends Command implements IAimDriveModes {

    // ── Dependencies ──────────────────────────────────────────────────────────
    private final SwerveDrive drive;

    private final IDriverInputs inputs;

    // ── CoR bounds (meters, converted from inches in constructor) ─────────────
    private final double corMaxX;
    private final double corMaxY;

    // ── Driver speed scaling (live-tunable, clamped to [0, 1]) ────────────────
    private final Dashboard.TunableNumber speedScale =
        Dashboard.tunable("Drive/Teleop Speed Scale", SwerveConstants.TELEOP_SPEED_SCALE);
    private final Dashboard.TunableNumber rotationScale =
        Dashboard.tunable("Drive/Teleop Rotation Scale", SwerveConstants.TELEOP_ROTATION_SCALE);

    // ── Drive mode ────────────────────────────────────────────────────────────

    private enum DriveMode {
        /** Full driver control — no heading override. */
        NORMAL,
        /** Heading PID holds robot facing a field target; translation free. */
        POINT_AT,
        /**
         * Driver controls heading freely within [headingBoundMinDeg, headingBoundMaxDeg].
         * PID-holds to the nearest bound when heading exits the window.
         */
        HEADING_BOUND,
        /** Wheels X-braced, no translation. Used for stationary shooting. */
        STATIC_SHOT_LOCK
    }

    private DriveMode driveMode = DriveMode.NORMAL;

    // ── POINT_AT state ────────────────────────────────────────────────────────
    private double targetFeetX    = 0.0;
    private double targetFeetY    = 0.0;
    private double targetOffsetDeg = 0.0;

    // ── HEADING_BOUND state ───────────────────────────────────────────────────
    private double headingBoundMinDeg = 0.0;
    private double headingBoundMaxDeg = 0.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Constructor
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * @param drive      SwerveDrive subsystem
     * @param inputs     Driver axes from the selected controller layout
     * @param corMaxXIn  CoR forward/back bound in INCHES (use bumper half-length)
     * @param corMaxYIn  CoR left/right bound in INCHES (use bumper half-width)
     */
    public TeleopDrive(
            SwerveDrive drive,
            IDriverInputs inputs,
            double corMaxXIn,
            double corMaxYIn) {

        this.drive   = drive;
        this.inputs  = inputs;
        this.corMaxX = Units.inchesToMeters(corMaxXIn);
        this.corMaxY = Units.inchesToMeters(corMaxYIn);

        addRequirements(drive);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Drive mode control — called by ShooterAimController each loop
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Activate POINT_AT mode: heading PID locks robot facing (targetFeetX, targetFeetY).
     * Translation remains under driver control.
     *
     * @param targetFeetX  Field X of target (feet).
     * @param targetFeetY  Field Y of target (feet).
     * @param offsetDeg    Heading offset in degrees (0 = robot front faces target).
     */
    public void enablePointAt(double targetFeetX, double targetFeetY, double offsetDeg) {
        this.driveMode      = DriveMode.POINT_AT;
        this.targetFeetX    = targetFeetX;
        this.targetFeetY    = targetFeetY;
        this.targetOffsetDeg = offsetDeg;
    }

    /**
     * Activate HEADING_BOUND mode: driver controls heading freely within the
     * provided window. PID-holds to the nearest bound if heading exits the window.
     * Translation remains under full driver control.
     *
     * Bounds come from ShooterOutput.headingBoundsDeg — the range of robot headings
     * within which the turret can reach the target without exceeding its soft stops.
     *
     * @param minAbsHeadingDeg  Minimum allowed absolute robot heading (degrees, field frame).
     * @param maxAbsHeadingDeg  Maximum allowed absolute robot heading (degrees, field frame).
     */
    public void enableHeadingBound(double minAbsHeadingDeg, double maxAbsHeadingDeg) {
        this.driveMode         = DriveMode.HEADING_BOUND;
        this.headingBoundMinDeg = minAbsHeadingDeg;
        this.headingBoundMaxDeg = maxAbsHeadingDeg;
    }

    /**
     * Activate STATIC_SHOT_LOCK mode: wheels X-braced, no translation.
     * The shooter fires while the robot is stationary.
     */
    public void enableStaticShotLock() {
        this.driveMode = DriveMode.STATIC_SHOT_LOCK;
    }

    /**
     * Restore NORMAL mode — full driver control, all heading overrides released.
     * Called by ShooterAimController.clearAimMode() on button release.
     */
    public void clearAimMode() {
        this.driveMode = DriveMode.NORMAL;
    }

    /**
     * Legacy: disable point-at and restore NORMAL mode.
     * Kept for backward compatibility with existing button bindings.
     */
    public void disablePointAt() {
        clearAimMode();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // execute — runs every 20ms
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void execute() {

        double linScale = MathUtil.clamp(speedScale.get(),    0.0, 1.0);
        double rotScale = MathUtil.clamp(rotationScale.get(), 0.0, 1.0);

        // ── Translation (field-relative) ──────────────────────────────────────
        double vxFtps = squareInput(applyDeadband(inputs.forward())) * linScale * SwerveConstants.MAX_DRIVE_SPEED_FPS;
        double vyFtps = squareInput(applyDeadband(inputs.left()))    * linScale * SwerveConstants.MAX_DRIVE_SPEED_FPS;

        // ── Rotation ──────────────────────────────────────────────────────────
        double omega = squareInput(applyDeadband(inputs.rotate())) * rotScale * SwerveConstants.MAX_ANGULAR_SPEED_RPS;

        // ── Center of rotation (linear, no squaring) ──────────────────────────
        // Inputs centered → CoR = (0,0) = robot center = pivot in place
        double corXm = MathUtil.clamp(applyDeadband(inputs.corForward()) * corMaxX, -corMaxX, corMaxX);
        double corYm = MathUtil.clamp(applyDeadband(inputs.corLeft())    * corMaxY, -corMaxY, corMaxY);

        Translation2d centerOfRotation = new Translation2d(corXm, corYm);

        // ── Build input and dispatch by drive mode ────────────────────────────
        DriveInput input = new DriveInput(vxFtps, vyFtps, omega, centerOfRotation);

        switch (driveMode) {
            case POINT_AT:
                drive.drivePointAt(input, targetFeetX, targetFeetY, targetOffsetDeg);
                break;

            case HEADING_BOUND:
                drive.driveWithHeadingBound(input, headingBoundMinDeg, headingBoundMaxDeg);
                break;

            case STATIC_SHOT_LOCK:
                // X-brace: ignore all driver input while locked for shot
                drive.lockWheelsX();
                break;

            case NORMAL:
            default:
                drive.drive(input);
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Helpers
    // ─────────────────────────────────────────────────────────────────────────

    private double applyDeadband(double value) {
        return MathUtil.applyDeadband(value, SwerveConstants.INPUT_DEADBAND);
    }

    /** Squares magnitude while preserving sign for improved low-speed feel. */
    private double squareInput(double value) {
        return Math.copySign(value * value, value);
    }
}
