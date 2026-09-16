package frc.robot.subsystems.swerveDrive.steer;

import frc.robot.util.motors.ConfigVerifyResult;

/**
 * SwerveSteerMotor.java
 * PATH: src/main/java/frc/robot/subsystems/swerveDrive/steer/SwerveSteerMotor.java
 *
 * Vendor-agnostic interface for a swerve steer (azimuth) motor.
 *
 * ─── UNIT CONTRACT ────────────────────────────────────────────────────────────
 * All positions are in MECHANISM SHAFT rotations — the output shaft of the steer
 * gearbox (i.e., the wheel azimuth direction). One rotation = 360° of wheel turn.
 * Gear ratio is handled internally by each implementation.
 *
 * ─── CONTINUOUS WRAP ─────────────────────────────────────────────────────────
 * Callers pass angles in [-0.5, 0.5] (equivalent to [-180°, 180°]).
 * Implementations are responsible for commanding the shortest path to any
 * target without crossing an encoder seam:
 *   CTRE  → ContinuousWrap = true (hardware handles it)
 *   REV   → positionWrappingEnabled(true) with range [-0.5, 0.5]
 *   Nova  → software shortest-path computed in setPositionRot()
 *
 * ─── ENCODER SEEDING ─────────────────────────────────────────────────────────
 * seedPosition() is called once at startup from the absolute encoder reading.
 * After seeding, the motor's internal (high-resolution) encoder is used for
 * closed-loop steering — not the absolute encoder.
 */
public interface SwerveSteerMotor {

    /**
     * Command a steer angle, taking the shortest path.
     * @param mechanismRotations  wheel azimuth in rotations, range [-0.5, 0.5]
     */
    void setPositionRot(double mechanismRotations);

    /**
     * @return current wheel azimuth in mechanism shaft rotations.
     *         Continuous — accumulates across multiple turns, not wrapped.
     */
    double getPositionRot();

    /** @return supply/bus current in amps, or closest vendor equivalent */
    double getSupplyCurrentAmps();

    /**
     * Seed the motor's internal encoder from the absolute encoder.
     * Called once at startup. Motor must not be moving when this is called.
     * @param mechanismRotations  absolute wheel azimuth in rotations [-0.5, 0.5]
     */
    void seedPosition(double mechanismRotations);

    /**
     * Apply updated PID + feed-forward gains.
     * kA is not used for position control and is ignored by all implementations.
     * Only called by swerveModule when values have changed from the previous loop.
     */
    void applyPIDGains(double kP, double kI, double kD, double kS, double kV);

    /**
     * Verify the motor controller accepted and retained its configuration.
     * @param label  human-readable label (e.g. "FL Steer")
     */
    ConfigVerifyResult verifyConfig(String label);
}
