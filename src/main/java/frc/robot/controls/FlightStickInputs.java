package frc.robot.controls;

import edu.wpi.first.wpilibj.Joystick;

/**
 * FlightStickInputs.java
 * PATH: src/main/java/frc/robot/controls/FlightStickInputs.java
 *
 * Two Logitech Extreme 3D Pro flight sticks.
 *
 * LEFT STICK:
 *   Axis 1 (Y)     → forward/back translation
 *   Axis 0 (X)     → left/right strafe
 *
 * RIGHT STICK:
 *   Axis 2 (twist) → rotation
 *   Axis 1 (Y)     → center of rotation forward/back
 *   Axis 0 (X)     → center of rotation left/right
 *
 * Raw stick Y is negative when pushed forward and X/twist are positive to the
 * right, so every axis is negated to match the DriverInputs convention.
 */
public class FlightStickInputs implements DriverInputs {

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

    /** The right stick, for binding its buttons in RobotContainer. */
    public Joystick rightStick() { return rightStick; }
}
