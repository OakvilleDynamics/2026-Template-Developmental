package frc.lib.swerve;

import frc.lib.hardware.encoder.AbsoluteEncoderIO;
import frc.lib.hardware.encoder.CTRECANcoderEncoder;
import frc.lib.hardware.encoder.REVThroughBoreEncoder;
import frc.lib.hardware.encoder.ThriftyAnalogEncoder;
import frc.lib.hardware.motor.MotorIO;
import frc.lib.hardware.motor.MotorIOConfig;
import frc.lib.hardware.motor.MotorVendor;

/**
 * swerveHardware.java
 * PATH: src/main/java/frc/lib/swerve/swerveHardware.java
 *
 * Swerve hardware catalog: maps a motor/encoder TYPE to the MotorIOConfig and
 * AbsoluteEncoderIO that drive it. The per-type current limits, inversion
 * conventions, and ramp rates live here once, so swapping module hardware is a
 * one-value change to the type selected in the robot's swerveConstants.
 *
 * Returns MotorIOConfig.Builders so the caller can add robot-level settings
 * (e.g. CAN signal rates) before building.
 *
 *   MotorIO drive = MotorIO.create(swerveHardware
 *       .driveMotorConfig(DriveMotorType.KRAKEN_X60, 1, true, drivePID, 6.75)
 *       .withCanSignals(250, 4, false)
 *       .build());
 */
public final class swerveHardware {

    public enum DriveMotorType      { KRAKEN_X60, KRAKEN_X44, NEO_VORTEX, NEO, NOVA_PULSAR }
    public enum SteerMotorType      { KRAKEN_X60, KRAKEN_X44, MINION, NOVA, NEO_VORTEX, NEO, NEO_550 }
    public enum AbsoluteEncoderType { THRIFTY_ANALOG, CTRE_CANCODER, REV_THROUGH_BORE }

    private swerveHardware() {}

    /**
     * Drive motor config: velocity control in wheel-shaft units, brake mode.
     *
     * Current limits (supply / stator):
     *   Kraken X60 — 60 / 80 A     Kraken X44 — 40 / 60 A
     *   NEO Vortex — 60 A          NEO        — 50 A
     *   Nova/Pulsar — 60 A
     *
     * @param type      drive motor type
     * @param canId     controller CAN ID
     * @param inverted  invert drive output
     * @param pid       { kP, kI, kD, kS, kV, kA }
     * @param gearRatio drive motor rotations per wheel rotation
     */
    public static MotorIOConfig.Builder driveMotorConfig(
            DriveMotorType type, int canId, boolean inverted, double[] pid, double gearRatio) {

        MotorVendor vendor;
        double supplyAmps, statorAmps = 80.0;
        boolean ramps = true;
        switch (type) {
            case KRAKEN_X60:  vendor = MotorVendor.CTRE_TALONFX;    supplyAmps = 60.0; statorAmps = 80.0; break;
            case KRAKEN_X44:  vendor = MotorVendor.CTRE_TALONFX;    supplyAmps = 40.0; statorAmps = 60.0; break;
            case NEO_VORTEX:  vendor = MotorVendor.REV_SPARKFLEX;   supplyAmps = 60.0; break;
            case NEO:         vendor = MotorVendor.REV_SPARKMAX;    supplyAmps = 50.0; break;
            case NOVA_PULSAR: vendor = MotorVendor.THRIFTYBOT_NOVA; supplyAmps = 60.0; ramps = false; break;
            default: throw new IllegalArgumentException("swerveHardware: unsupported drive type " + type);
        }

        MotorIOConfig.Builder b = MotorIOConfig.builder(canId, vendor)
            .withInverted(inverted)
            .withBrakeOnNeutral(true)
            .withGearRatio(gearRatio)
            .withGains(pid)
            .withCurrentLimits(supplyAmps, statorAmps);
        if (ramps) b.withRampRates(0.1, 0.02);
        return b;
    }

    /**
     * Steer motor config: position control in module-azimuth rotations with
     * continuous wrap (shortest path across ±0.5 rot), brake mode.
     *
     * Current limits (supply / stator):
     *   Kraken X60 — 40 / 60 A     Kraken X44, Minion — 30 / 40 A
     *   Nova — 30 A   NEO Vortex — 30 A   NEO — 25 A   NEO 550 — 20 A
     *
     * Inversion: TalonFX steer runs Clockwise_Positive; REV and Nova run
     * non-inverted (matches the previously validated per-vendor classes).
     *
     * @param type      steer motor type
     * @param canId     controller CAN ID
     * @param pid       { kP, kI, kD, kS, kV } — kA unused for position control
     * @param gearRatio steer motor rotations per module azimuth rotation
     */
    public static MotorIOConfig.Builder steerMotorConfig(
            SteerMotorType type, int canId, double[] pid, double gearRatio) {

        MotorVendor vendor;
        double supplyAmps, statorAmps = 40.0;
        switch (type) {
            case KRAKEN_X60: vendor = MotorVendor.CTRE_TALONFX;    supplyAmps = 40.0; statorAmps = 60.0; break;
            case KRAKEN_X44:
            case MINION:     vendor = MotorVendor.CTRE_TALONFX;    supplyAmps = 30.0; statorAmps = 40.0; break;
            case NOVA:       vendor = MotorVendor.THRIFTYBOT_NOVA; supplyAmps = 30.0; break;
            case NEO_VORTEX: vendor = MotorVendor.REV_SPARKFLEX;   supplyAmps = 30.0; break;
            case NEO:        vendor = MotorVendor.REV_SPARKMAX;    supplyAmps = 25.0; break;
            case NEO_550:    vendor = MotorVendor.REV_SPARKMAX;    supplyAmps = 20.0; break;
            default: throw new IllegalArgumentException("swerveHardware: unsupported steer type " + type);
        }

        return MotorIOConfig.builder(canId, vendor)
            .withInverted(vendor == MotorVendor.CTRE_TALONFX)
            .withBrakeOnNeutral(true)
            .withGearRatio(gearRatio)
            .withGains(pid)
            .withCurrentLimits(supplyAmps, statorAmps)
            .withContinuousWrap(true);
    }

    /**
     * Build the module's absolute steer encoder.
     *
     * @param type          encoder type
     * @param steerMotor    the module's steer MotorIO (REV_THROUGH_BORE reads through it)
     * @param analogPort    roboRIO analog port (THRIFTY_ANALOG only)
     * @param cancoderCanId CANcoder CAN ID (CTRE_CANCODER only)
     * @param offsetVolts   Thrifty voltage when the wheel points forward (THRIFTY_ANALOG)
     * @param offsetRot     rotation offset when the wheel points forward (CANCODER, THROUGH_BORE)
     * @throws IllegalArgumentException REV_THROUGH_BORE with a non-REV steer motor
     */
    public static AbsoluteEncoderIO buildEncoder(
            AbsoluteEncoderType type, MotorIO steerMotor,
            int analogPort, int cancoderCanId, double offsetVolts, double offsetRot) {
        switch (type) {
            case THRIFTY_ANALOG:   return new ThriftyAnalogEncoder(analogPort, offsetVolts);
            case CTRE_CANCODER:    return new CTRECANcoderEncoder(cancoderCanId, offsetRot);
            case REV_THROUGH_BORE: return new REVThroughBoreEncoder(steerMotor, offsetRot);
            default: throw new IllegalArgumentException("swerveHardware: unsupported encoder type " + type);
        }
    }
}
