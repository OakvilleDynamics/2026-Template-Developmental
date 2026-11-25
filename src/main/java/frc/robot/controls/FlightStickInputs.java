package frc.robot.controls;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Two Logitech Extreme 3D Pro flight sticks.
 *
 * <p>LEFT STICK:
 *
 * <pre>{@code
 * Axis 1 (Y)     → forward/back translation
 * Axis 0 (X)     → left/right strafe
 * }</pre>
 *
 * <p>RIGHT STICK:
 *
 * <pre>{@code
 * Axis 2 (twist) → rotation
 * Axis 1 (Y)     → center of rotation forward/back
 * Axis 0 (X)     → center of rotation left/right
 * }</pre>
 *
 * <p>Raw stick Y is negative when pushed forward and X/twist are positive to the
 * right, so every axis is negated to match the IDriverInputs convention.
 */
public class FlightStickInputs implements IDriverInputs {

    // ── Logitech Extreme 3D Pro axis indices ──────────────────────────────────
    private static final int AXIS_X     = 0;  // stick left/right
    private static final int AXIS_Y     = 1;  // stick forward/back (inverted)
    private static final int AXIS_TWIST = 2;  // twist rotation

    private final Joystick leftStick;
    private final Joystick rightStick;

    public FlightStickInputs(Joystick leftStick, Joystick rightStick) {
        this.leftStick  = leftStick;
        this.rightStick = rightStick;
    }

    @Override public double forward()    { return -leftStick.getRawAxis(AXIS_Y); }
    @Override public double left()       { return -leftStick.getRawAxis(AXIS_X); }
    @Override public double rotate()     { return -rightStick.getRawAxis(AXIS_TWIST); }
    @Override public double corForward() { return -rightStick.getRawAxis(AXIS_Y); }
    @Override public double corLeft()    { return -rightStick.getRawAxis(AXIS_X); }

    /** The left stick, for binding its buttons in RobotContainer. */
    public Joystick leftStick() { return leftStick; }

    /** The right stick, for binding its buttons in RobotContainer. */
    public Joystick rightStick() { return rightStick; }
}
