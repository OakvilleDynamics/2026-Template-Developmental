package frc.lib.mechanisms.shooter;

import edu.wpi.first.math.geometry.Pose2d;

import frc.lib.swerve.driveOdometryState;

/**
 * AimDriveState.java
 * PATH: src/main/java/frc/lib/mechanisms/shooter/AimDriveState.java
 *
 * The drivetrain state shooterAimController reads each loop. Implemented by
 * frc.robot.subsystems.swerveDrive.swerveDrive; tests implement it with fixed values.
 */
public interface AimDriveState {

    /** Fused field pose (meters, WPILib convention). */
    Pose2d getPose();

    /** Full motion snapshot (SI units). */
    driveOdometryState getOdometryState();

    /** True when the heading PID is at its point-at setpoint. */
    boolean isPointedAtTarget();
}
