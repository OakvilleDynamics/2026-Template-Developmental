package frc.robot.controls;

import edu.wpi.first.wpilibj.XboxController;

/**
 * XboxInputs.java
 * PATH: src/main/java/frc/robot/controls/XboxInputs.java
 *
 * One Xbox-style controller.
 *
 * LEFT STICK:
 *   Y → forward/back translation
 *   X → left/right strafe
 *
 * RIGHT STICK:
 *   X → rotation
 *
 * No center-of-rotation control — the robot always pivots about its center.
 *
 * Raw stick Y is negative when pushed forward and X is positive to the right,
 * so every axis is negated to match the DriverInputs convention.
 */
public class XboxInputs implements DriverInputs {

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
