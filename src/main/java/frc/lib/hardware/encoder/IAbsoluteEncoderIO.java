package frc.lib.hardware.encoder;

/**
 * AbsoluteEncoderIO.java
 * PATH: src/main/java/frc/lib/hardware/encoder/AbsoluteEncoderIO.java
 *
 * Vendor-agnostic interface for an absolute angle encoder (swerve steer,
 * arm pivot, turret).
 *
 * For swerve, the absolute encoder is read once at startup to seed the steer
 * motor's internal (high-resolution) encoder. It is not polled during normal
 * operation.
 *
 * The mounting-angle zero offset is applied internally by each implementation —
 * callers always receive an offset-corrected reading.
 *
 * Implementations: ThriftyAnalogEncoder, CTRECANcoderEncoder, REVThroughBoreEncoder.
 */
public interface AbsoluteEncoderIO {

    /**
     * @return absolute angle in radians, range [0, 2π).
     *         Zero offset is applied; 0 rad corresponds to the configured
     *         zero direction (e.g. module "straight forward").
     */
    double getAbsoluteAngleRad();
}
