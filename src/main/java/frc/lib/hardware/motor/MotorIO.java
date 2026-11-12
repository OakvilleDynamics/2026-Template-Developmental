package frc.lib.hardware.motor;

import frc.lib.util.ConfigVerifyResult;

/**
 * MotorIO.java
 * PATH: src/main/java/frc/lib/hardware/motor/MotorIO.java
 *
 * The single vendor-agnostic interface for ONE motor controller. Every motor on
 * the robot — swerve drive, swerve steer, and every mechanism motor — is a MotorIO.
 *
 *   MotorIO motor = MotorIO.create(MotorIOConfig.builder(12, MotorVendor.REV_SPARKMAX)
 *       .withGearRatio(25.0)
 *       .build());
 *
 * Swapping a motor to a different vendor is a one-value change: the MotorVendor
 * in its MotorIOConfig. Nothing above this interface imports a vendor library.
 *
 * ─── IMPLEMENTATIONS ─────────────────────────────────────────────────────────
 *   CTREMotor — TalonFX (Phoenix 6)
 *   REVMotor  — SparkMax / SparkFlex (REVLib)
 *   NovaMotor — ThriftyBot Nova (ThriftyLib)
 *   FakeMotorIO (src/test) — in-memory stand-in for unit tests, no HAL required
 *
 * ─── UNITS ───────────────────────────────────────────────────────────────────
 *   Mechanism shaft rotations and rotations/second (gear ratio applied inside).
 *   Feed-forward addends in volts.
 */
public interface MotorIO {

    /** Config apply retries used by every verifyConfig() implementation. */
    int VERIFY_RETRIES  = 5;
    /** Delay between config apply retries (ms). */
    int VERIFY_DELAY_MS = 50;

    /**
     * Create the vendor implementation selected by {@code config.vendor}.
     */
    static MotorIO create(MotorIOConfig config) {
        switch (config.vendor) {
            case CTRE_TALONFX:    return new CTREMotor(config);
            case REV_SPARKMAX:
            case REV_SPARKFLEX:   return new REVMotor(config);
            case THRIFTYBOT_NOVA: return new NovaMotor(config);
            default:
                throw new IllegalArgumentException("MotorIO: unknown vendor " + config.vendor);
        }
    }

    /** The config this motor was constructed from. */
    MotorIOConfig getConfig();

    // ── Commands ──────────────────────────────────────────────────────────────

    /** Open-loop output, -1.0 to 1.0. */
    void setDutyCycle(double duty);

    /**
     * Closed-loop velocity (slot 0).
     * @param velocityRps mechanism shaft rotations/second
     * @param ffVolts     Rio-computed feed-forward addend (volts)
     */
    void setVelocity(double velocityRps, double ffVolts);

    /**
     * Closed-loop position (slot 0). Uses the hardware motion profile when one
     * is configured and supported, and shortest-path routing when
     * continuousWrap is enabled.
     * @param positionRot mechanism shaft rotations
     * @param ffVolts     Rio-computed feed-forward addend (volts)
     */
    void setPosition(double positionRot, double ffVolts);

    /** Neutral output (brake or coast per config). */
    void stop();

    /**
     * Mirror another controller's output in hardware. The leader must be the
     * same vendor family.
     * @param leader       controller to follow
     * @param opposeLeader true = spin opposite to the leader
     */
    void follow(MotorIO leader, boolean opposeLeader);

    /** Overwrite the integrated encoder reading (mechanism shaft rotations). */
    void setEncoderPosition(double positionRot);

    /** Push new slot-0 gains to the controller (live PID tuning). */
    void applyGains(double kP, double kI, double kD, double kS, double kV, double kA);

    // ── Feedback ──────────────────────────────────────────────────────────────

    double getPositionRot();

    double getVelocityRps();

    double getSupplyCurrentAmps();

    /** Stator current in amps. 0 when the vendor does not expose it (REV). */
    double getStatorCurrentAmps();

    double getMotorVoltage();

    double getTemperatureC();

    boolean isForwardSoftLimitHit();

    boolean isReverseSoftLimitHit();

    /**
     * Current reading best suited for stall detection (homing, travel limits).
     * Stator current where available (proportional to torque); output current on REV.
     */
    double getStallDetectionCurrentAmps();

    // ── Capabilities ──────────────────────────────────────────────────────────

    /** True when setPosition() runs a hardware motion profile (MotionMagic / MAXMotion). */
    boolean hasHardwareMotionProfile();

    /** True when an open-loop ramp is applied on-controller. */
    default boolean hasHardwareDutyCycleRamp() {
        return getConfig().openLoopRampSecs > 0.0;
    }

    /**
     * True when this controller can act as an ENCODER_SYNC follower (independent
     * encoder readback while running its own output). False for Nova.
     */
    boolean supportsEncoderSync();

    // ── Startup verification ──────────────────────────────────────────────────

    /**
     * Re-apply this motor's full configuration (with retries), read it back, and
     * diff the key fields. Re-applies the FULL config, so running it never resets
     * gear ratio, gains, or soft limits to factory defaults.
     *
     * @param label device label for the DS report, e.g. "Elevator Leader"
     */
    ConfigVerifyResult verifyConfig(String label);
}
