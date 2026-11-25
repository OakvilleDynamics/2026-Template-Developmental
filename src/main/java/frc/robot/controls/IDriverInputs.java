package frc.robot.controls;

/**
 * The driver's drive axes, independent of which controller produces them.
 * TeleopDrive reads only this interface, so swapping controllers never touches
 * drive behavior. Add a new controller by implementing this interface.
 *
 * <h2>Conventions</h2>
 *
 * <p>Every axis is raw stick deflection in [−1, 1], before deadband or shaping.
 * Signs are already corrected for the controller, using WPILib's convention:
 *
 * <ul>
 *   <li>{@code forward()}: +1 = full forward</li>
 *   <li>{@code left()}: +1 = full left</li>
 *   <li>{@code rotate()}: +1 = full counter-clockwise</li>
 *   <li>{@code corForward()}: +1 = center of rotation at the front bumper</li>
 *   <li>{@code corLeft()}: +1 = center of rotation at the left bumper</li>
 * </ul>
 */
public interface IDriverInputs {

    /** Translation forward/back. +1 = full forward. */
    double forward();

    /** Translation strafe. +1 = full left. */
    double left();

    /** Rotation rate. +1 = full counter-clockwise. */
    double rotate();

    /** Center of rotation forward/back. Default 0 = pivot about robot center. */
    default double corForward() { return 0.0; }

    /** Center of rotation left/right. Default 0 = pivot about robot center. */
    default double corLeft() { return 0.0; }
}
