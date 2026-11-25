package frc.robot.controls;

import edu.wpi.first.wpilibj.XboxController;

/**
 * One Xbox-style controller.
 *
 * <p>LEFT STICK:
 *
 * <ul>
 *   <li><b>Y</b> → forward/back translation</li>
 *   <li><b>X</b> → left/right strafe</li>
 * </ul>
 *
 * <p>RIGHT STICK:
 *
 * <ul>
 *   <li><b>X</b> → rotation</li>
 * </ul>
 *
 * <p>No center-of-rotation control — the robot always pivots about its center.
 *
 * <p>Raw stick Y is negative when pushed forward and X is positive to the right,
 * so every axis is negated to match the IDriverInputs convention.
 */
public class XboxInputs implements IDriverInputs {

    private final XboxController controller;

    public XboxInputs(XboxController controller) {
        this.controller = controller;
    }

    @Override public double forward() { return -controller.getLeftY(); }
    @Override public double left()    { return -controller.getLeftX(); }
    @Override public double rotate()  { return -controller.getRightX(); }

    /** The controller, for binding its buttons in RobotContainer. */
    public XboxController controller() { return controller; }
}
