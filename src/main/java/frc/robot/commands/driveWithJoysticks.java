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
 * ─── POINT-AT-TARGET ─────────────────────────────────────────────────────────
 * When enablePointAt() is called (by RobotContainer button binding):
 *   - Twist axis is ignored
 *   - Heading PID in swerveDrive takes over omega to face the target
 *   - Left stick translation still works normally
 * disablePointAt() restores normal twist-to-rotate behavior.
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

    // ── Point-at-target state ─────────────────────────────────────────────────
    private boolean pointAtActive  = false;
    private double  targetFeetX    = 0.0;
    private double  targetFeetY    = 0.0;
    private double  targetOffsetDeg = 0.0;

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
    // Point-at-target control (called by RobotContainer button binding)
    // ─────────────────────────────────────────────────────────────────────────

    public void enablePointAt(double targetFeetX, double targetFeetY, double offsetDeg) {
        this.pointAtActive   = true;
        this.targetFeetX     = targetFeetX;
        this.targetFeetY     = targetFeetY;
        this.targetOffsetDeg = offsetDeg;
    }

    public void disablePointAt() {
        this.pointAtActive = false;
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

        // ── Build input and command the subsystem ─────────────────────────────
        driveInput input = new driveInput(vxFtps, vyFtps, omega, centerOfRotation);

        if (pointAtActive) {
            // Heading PID takes over omega; translation passes through normally
            drive.drivePointAt(input, targetFeetX, targetFeetY, targetOffsetDeg);
        } else {
            drive.drive(input);
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

    /** Squares magnitude while preserving sign for improved low-speed feel */
    private double squareInput(double value) {
        return Math.copySign(value * value, value);
    }
}