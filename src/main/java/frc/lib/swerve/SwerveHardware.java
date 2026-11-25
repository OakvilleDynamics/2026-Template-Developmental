package frc.lib.swerve;

import frc.lib.hardware.encoder.IAbsoluteEncoderIO;
import frc.lib.hardware.encoder.CTRECANcoderEncoder;
import frc.lib.hardware.encoder.REVThroughBoreEncoder;
import frc.lib.hardware.encoder.ThriftyAnalogEncoder;
import frc.lib.hardware.motor.IMotorIO;
import frc.lib.hardware.motor.MotorIOConfig;
import frc.lib.hardware.motor.MotorVendor;
import frc.lib.hardware.motor.MotorIOConfig.NovaMotorType;

/**
 * Swerve hardware catalog: maps a motor/encoder TYPE to the MotorIOConfig and
 * IAbsoluteEncoderIO that drive it. The per-type current limits, inversion
 * conventions, and ramp rates live here once, so swapping module hardware is a
 * one-value change to the type selected in the robot's SwerveConstants.
 *
 * <p>Returns MotorIOConfig.Builders so the caller can add robot-level settings
 * (e.g. CAN signal rates) before building.
 *
 * <pre>{@code
 * IMotorIO drive = IMotorIO.create(SwerveHardware
 *     .driveMotorConfig(DriveMotorType.KRAKEN_X60, 1, true, drivePID, 6.75)
 *     .withCanSignals(250, 4, false)
 *     .build());
 * }</pre>
 */
public final class SwerveHardware {

    public enum DriveMotorType      { KRAKEN_X60, KRAKEN_X44, NEO_VORTEX, NEO, NOVA_PULSAR }
    public enum SteerMotorType      { KRAKEN_X60, KRAKEN_X44, MINION, NOVA, NEO_VORTEX, NEO, NEO_550 }
    public enum AbsoluteEncoderType { THRIFTY_ANALOG, CTRE_CANCODER, REV_THROUGH_BORE }

    private SwerveHardware() {}

    /**
     * Drive motor config: velocity control in wheel-shaft units, brake mode.
     *
     * <p>Current limits (supply / stator):
     *
     * <ul>
     *   <li>Kraken X60 — 60 / 80 A</li>
     *   <li>Kraken X44 — 40 / 60 A</li>
     *   <li>NEO Vortex — 60 A</li>
     *   <li>NEO — 50 A</li>
     *   <li>Nova/Pulsar — 60 A</li>
     * </ul>
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
            default: throw new IllegalArgumentException("SwerveHardware: unsupported drive type " + type);
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
     * <p>Current limits (supply / stator):
     *
     * <ul>
     *   <li>Kraken X60 — 40 / 60 A</li>
     *   <li>Kraken X44, Minion — 30 / 40 A</li>
     *   <li>Nova — 30 A</li>
     *   <li>NEO Vortex — 30 A</li>
     *   <li>NEO — 25 A</li>
     *   <li>NEO 550 — 20 A</li>
     * </ul>
     *
     * <p>Inversion is chosen by the robot: positive output must turn the module
     * counter-clockwise viewed from above. Previously validated: TalonFX steer
     * inverted (Clockwise_Positive); REV and Nova non-inverted.
     *
     * @param type      steer motor type
     * @param canId     controller CAN ID
     * @param inverted  invert steer output
     * @param pid       { kP, kI, kD, kS, kV } — kA unused for position control
     * @param gearRatio steer motor rotations per module azimuth rotation
     */
    public static MotorIOConfig.Builder steerMotorConfig(
            SteerMotorType type, int canId, boolean inverted, double[] pid, double gearRatio) {

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
            default: throw new IllegalArgumentException("SwerveHardware: unsupported steer type " + type);
        }

        MotorIOConfig.Builder b = MotorIOConfig.builder(canId, vendor)
            .withInverted(inverted)
            .withBrakeOnNeutral(true)
            .withGearRatio(gearRatio)
            .withGains(pid)
            .withCurrentLimits(supplyAmps, statorAmps)
            .withContinuousWrap(true);
        // TODO: let the robot choose the motor a Nova steer drives (NEO or Minion)
        // instead of assuming Minion, e.g. split NOVA into NOVA_MINION / NOVA_NEO.
        if (type == SteerMotorType.NOVA) b.withNovaMotorType(NovaMotorType.MINION);
        return b;
    }

    /**
     * Build the module's absolute steer encoder.
     *
     * @param type          encoder type
     * @param steerMotor    the module's steer IMotorIO (REV_THROUGH_BORE reads through it)
     * @param analogPort    roboRIO analog port (THRIFTY_ANALOG only)
     * @param cancoderCanId CANcoder CAN ID (CTRE_CANCODER only)
     * @param offsetRot     raw reading in rotations when the wheel points forward
     * @param inverted      reverse the encoder direction (THRIFTY_ANALOG only)
     * @throws IllegalArgumentException REV_THROUGH_BORE with a non-REV steer motor,
     *                                  or inverted with an encoder that doesn't support it
     */
    public static IAbsoluteEncoderIO buildEncoder(
            AbsoluteEncoderType type, IMotorIO steerMotor,
            int analogPort, int cancoderCanId, double offsetRot,
            boolean inverted) {
        // TODO: support inversion for CTRE_CANCODER (SensorDirection) and REV_THROUGH_BORE.
        if (inverted && type != AbsoluteEncoderType.THRIFTY_ANALOG)
            throw new IllegalArgumentException(
                "SwerveHardware: encoder inversion is only supported for THRIFTY_ANALOG, got " + type);
        switch (type) {
            case THRIFTY_ANALOG:   return new ThriftyAnalogEncoder(analogPort, offsetRot, inverted);
            case CTRE_CANCODER:    return new CTRECANcoderEncoder(cancoderCanId, offsetRot);
            case REV_THROUGH_BORE: return new REVThroughBoreEncoder(steerMotor, offsetRot);
            default: throw new IllegalArgumentException("SwerveHardware: unsupported encoder type " + type);
        }
    }
}
