package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.swerveConstants;
import frc.robot.subsystems.swerveDrive.driveInput;
import frc.robot.subsystems.swerveDrive.swerveDrive;
import frc.robot.util.units;

/**
 * driveWithJoysticks.java
 * PATH: src/main/java/frc/robot/commands/driveWithJoysticks.java
 *
 * Default drive command. Runs continuously while no other command requires
 * the swerveDrive subsystem.
 *
 * ─── CONTROL MAPPING (Logitech Extreme 3D Pro) ───────────────────────────────
 *
 * LEFT STICK:
 *   Axis 1 (Y, inverted) → vx  — forward/back, field-relative (ft/s)
 *   Axis 0 (X)           → vy  — left/right strafe, field-relative (ft/s)
 *
 * RIGHT STICK:
 *   Axis 2 (twist)       → omega — robot rotation rate (rad/s)
 *   Axis 1 (Y, inverted) → CoR X — center of rotation forward/back (robot-relative)
 *   Axis 0 (X)           → CoR Y — center of rotation left/right (robot-relative)
 *
 * ─── CENTER OF ROTATION ──────────────────────────────────────────────────────
 * Right stick X/Y maps linearly to CoR bounds (bumper half-dimensions).
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
 * The shooterAimController sets the active mode each loop via the enable/clear
 * methods. This is the external driver-input pipeline — joystick values are read
 * every execute() but the output path is governed by the externally-set mode.
 *
 * ─── INPUT SHAPING ───────────────────────────────────────────────────────────
 * squareInput() squares magnitude while preserving sign.
 * Gives finer low-speed control without losing full power at full deflection.
 */
public class driveWithJoysticks extends Command {

    // ── Logitech Extreme 3D Pro axis indices ──────────────────────────────────
    private static final int AXIS_X     = 0;  // stick left/right
    private static final int AXIS_Y     = 1;  // stick forward/back (inverted)
    private static final int AXIS_TWIST = 2;  // twist rotation

    // ── Dependencies ──────────────────────────────────────────────────────────
    private final swerveDrive drive;
    private final Joystick    leftStick;
    private final Joystick    rightStick;

    // ── CoR bounds (meters, converted from inches in constructor) ─────────────
    private final double corMaxX;
    private final double corMaxY;

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
     * @param drive       swerveDrive subsystem
     * @param leftStick   Left flight stick — translation input
     * @param rightStick  Right flight stick — rotation, CoR, and button input
     * @param corMaxXIn   CoR forward/back bound in INCHES (use bumper half-length)
     * @param corMaxYIn   CoR left/right bound in INCHES (use bumper half-width)
     */
    public driveWithJoysticks(
            swerveDrive drive,
            Joystick leftStick,
            Joystick rightStick,
            double corMaxXIn,
            double corMaxYIn) {

        this.drive      = drive;
        this.leftStick  = leftStick;
        this.rightStick = rightStick;
        this.corMaxX    = units.inches_m(corMaxXIn);
        this.corMaxY    = units.inches_m(corMaxYIn);

        addRequirements(drive);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Drive mode control — called by shooterAimController each loop
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
     * Called by shooterAimController.clearAimMode() on button release.
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

        // ── Left stick: translation (field-relative) ──────────────────────────
        double lY = applyDeadband(leftStick.getRawAxis(AXIS_Y));
        double lX = applyDeadband(leftStick.getRawAxis(AXIS_X));

        // Y axis inverted: push forward = negative raw → positive vx (field forward)
        double vxFtps =  squareInput(-lY) * swerveConstants.MAX_DRIVE_SPEED_FPS;
        double vyFtps = -squareInput( lX) * swerveConstants.MAX_DRIVE_SPEED_FPS;

        // ── Right stick twist: rotation ───────────────────────────────────────
        double twist = applyDeadband(rightStick.getRawAxis(AXIS_TWIST));
        double omega = -squareInput(twist) * swerveConstants.MAX_ANGULAR_SPEED_RPS;

        // ── Right stick X/Y: center of rotation ───────────────────────────────
        // Push stick forward  → CoR moves toward robot front (+X)
        // Push stick right    → CoR moves toward robot right (-Y)
        // Stick centered      → CoR = (0,0) = robot center = pivot in place
        double rY = applyDeadband(rightStick.getRawAxis(AXIS_Y));
        double rX = applyDeadband(rightStick.getRawAxis(AXIS_X));

        double corXm = MathUtil.clamp(-rY * corMaxX, -corMaxX, corMaxX);
        double corYm = MathUtil.clamp(-rX * corMaxY, -corMaxY, corMaxY);

        Translation2d centerOfRotation = new Translation2d(corXm, corYm);

        // ── Build input and dispatch by drive mode ────────────────────────────
        driveInput input = new driveInput(vxFtps, vyFtps, omega, centerOfRotation);

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
        return MathUtil.applyDeadband(value, swerveConstants.INPUT_DEADBAND);
    }

    /** Squares magnitude while preserving sign for improved low-speed feel. */
    private double squareInput(double value) {
        return Math.copySign(value * value, value);
    }
}
