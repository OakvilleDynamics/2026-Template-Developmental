package frc.lib.hardware.encoder;

/**
 * Vendor-agnostic interface for an absolute angle encoder (swerve steer,
 * arm pivot, turret).
 *
 * <p>For swerve, the absolute encoder is read once at startup to seed the steer
 * motor's internal (high-resolution) encoder. It is not polled during normal
 * operation.
 *
 * <p>The mounting-angle zero offset is applied internally by each implementation —
 * callers always receive an offset-corrected reading.
 *
 * <p>Implementations: ThriftyAnalogEncoder, CTRECANcoderEncoder, REVThroughBoreEncoder.
 */
public interface IAbsoluteEncoderIO {

    /**
     * @return absolute angle in radians, range [0, 2π).
     *         Zero offset is applied; 0 rad corresponds to the configured
     *         zero direction (e.g. module "straight forward").
     */
    double getAbsoluteAngleRad();
}
