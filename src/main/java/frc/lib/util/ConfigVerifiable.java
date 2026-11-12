package frc.robot.util.motors;

import java.util.List;

/**
 * ConfigVerifiable.java
 * PATH: src/main/java/frc/robot/util/motors/ConfigVerifiable.java
 *
 * Marker interface for any object that can verify its motor controller
 * configurations were applied correctly. Implemented by mechanismUnit
 * subclasses, swerveModule, and mechanism wrappers (elevatorMechanism,
 * shooterMechanism).
 *
 * Register any ConfigVerifiable with ConfigVerifier.register(), then call
 * ConfigVerifier.runAll() once in robotInit() before enabling motors.
 */
public interface ConfigVerifiable {

    /**
     * Verify all motor controller configurations for this device or group.
     * Retries apply up to 5× per device, then reads back and diffs key fields.
     *
     * @return one ConfigVerifyResult per physical motor controller checked
     */
    List<ConfigVerifyResult> verifyConfig();
}
