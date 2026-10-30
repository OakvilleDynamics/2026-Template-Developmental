package frc.lib.mechanisms.shooter;

/**
 * AimDriveModes.java
 * PATH: src/main/java/frc/lib/mechanisms/shooter/AimDriveModes.java
 *
 * The drive-mode controls shooterAimController needs from the robot's driver
 * drive command. Implemented by frc.robot.commands.driveWithJoysticks; any other
 * robot can implement it on its own drive command to reuse the aim controller.
 *
 * Angles in degrees (field-absolute), field coordinates in feet.
 */
public interface AimDriveModes {

    /** Driver translates; heading PID locks onto a field point. */
    void enablePointAt(double targetFeetX, double targetFeetY, double offsetDeg);

    /** Driver translates; heading is clamped into [minAbsHeadingDeg, maxAbsHeadingDeg]. */
    void enableHeadingBound(double minAbsHeadingDeg, double maxAbsHeadingDeg);

    /** Robot stops and X-braces for a static shot. */
    void enableStaticShotLock();

    /** Return to normal driver control. */
    void clearAimMode();
}
