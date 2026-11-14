package frc.lib.mechanisms.shooter;

import edu.wpi.first.math.geometry.Pose2d;

import frc.lib.swerve.DriveOdometryState;

/**
 * AimDriveState.java
 * PATH: src/main/java/frc/lib/mechanisms/shooter/AimDriveState.java
 *
 * The drivetrain state ShooterAimController reads each loop. Implemented by
 * frc.robot.subsystems.swerveDrive.SwerveDrive; tests implement it with fixed values.
 */
public interface AimDriveState {

    /** Fused field pose (meters, WPILib convention). */
    Pose2d getPose();

    /** Full motion snapshot (SI units). */
    DriveOdometryState getOdometryState();

    /** True when the heading PID is at its point-at setpoint. */
    boolean isPointedAtTarget();
}
