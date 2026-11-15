package frc.lib.util;

import java.util.List;

/**
 * IConfigVerifiable.java
 * PATH: src/main/java/frc/lib/util/IConfigVerifiable.java
 *
 * Marker interface for any object that can verify its motor controller
 * configurations were applied correctly. Implemented by MechanismUnit
 * subclasses, SwerveModule, and mechanism wrappers (ElevatorMechanism,
 * ShooterMechanism).
 *
 * Register any IConfigVerifiable with ConfigVerifier.register(), then call
 * ConfigVerifier.runAll() once at robot startup before enabling motors.
 */
public interface IConfigVerifiable {

    /**
     * Verify all motor controller configurations for this device or group.
     * Retries apply up to 5× per device, then reads back and diffs key fields.
     *
     * @return one ConfigVerifyResult per physical motor controller checked
     */
    List<ConfigVerifyResult> verifyConfig();
}
