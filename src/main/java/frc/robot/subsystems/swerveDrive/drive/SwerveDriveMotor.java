package frc.robot.subsystems.swerveDrive.drive;

import frc.robot.util.motors.ConfigVerifyResult;

/**
 * SwerveDriveMotor.java
 * PATH: src/main/java/frc/robot/subsystems/swerveDrive/drive/SwerveDriveMotor.java
 *
 * Vendor-agnostic interface for a swerve drive wheel motor.
 *
 * ─── UNIT CONTRACT ────────────────────────────────────────────────────────────
 * All velocities and positions are in WHEEL SHAFT units (rotations, rot/sec).
 * Gear ratio is handled internally by each implementation — callers never
 * multiply or divide by gear ratio.
 *
 * ─── PID LIVE TUNING ─────────────────────────────────────────────────────────
 * applyPIDGains() is safe to call on change each loop (swerveModule checks
 * for change before calling). Implementations push gains without full
 * motor re-initialization:
 *   CTRE  → Slot0Configs only (fast, non-disruptive)
 *   REV   → configure() with kNoPersistParameters (no flash write)
 *   Nova  → pid0 field update (live, no CAN frame needed)
 */
public interface SwerveDriveMotor {

    /**
     * Command a wheel velocity.
     * @param wheelRotPerSec  wheel shaft speed in rotations/sec (signed)
     */
    void setVelocityRotPerSec(double wheelRotPerSec);

    /** @return current wheel shaft speed in rotations/sec (signed) */
    double getVelocityRotPerSec();

    /** @return cumulative wheel shaft position in rotations (not wrapped, accumulates) */
    double getPositionRot();

    /** @return supply/bus current in amps, or closest vendor equivalent */
    double getSupplyCurrentAmps();

    /**
     * Apply updated PID + feed-forward gains.
     * Only called by swerveModule when values have changed from the previous loop.
     */
    void applyPIDGains(double kP, double kI, double kD, double kS, double kV, double kA);

    /**
     * Verify the motor controller accepted and retained its configuration.
     * @param label  human-readable label for DS/log output (e.g. "FL Drive")
     */
    ConfigVerifyResult verifyConfig(String label);
}
