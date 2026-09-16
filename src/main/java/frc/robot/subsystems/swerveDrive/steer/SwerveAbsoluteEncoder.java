package frc.robot.subsystems.swerveDrive.steer;

/**
 * SwerveAbsoluteEncoder.java
 * PATH: src/main/java/frc/robot/subsystems/swerveDrive/steer/SwerveAbsoluteEncoder.java
 *
 * Vendor-agnostic interface for a swerve module absolute steer encoder.
 *
 * The absolute encoder is read once at startup to seed the steer motor's
 * internal (high-resolution) encoder. It is not polled during normal operation.
 *
 * The mounting-angle zero offset is applied internally by each implementation —
 * callers always receive an offset-corrected reading.
 */
public interface SwerveAbsoluteEncoder {

    /**
     * @return absolute wheel azimuth angle in radians, range [0, 2π].
     *         Zero offset is applied; 0 rad corresponds to the module's
     *         configured "straight forward" direction.
     */
    double getAbsoluteAngleRad();
}
