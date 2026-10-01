package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.telemetry.Telemetry;

import frc.robot.util.motors.ConfigVerifiable;
import frc.robot.util.motors.ConfigVerifyResult;

/**
 * ConfigVerifier.java
 * PATH: src/main/java/frc/robot/util/ConfigVerifier.java
 *
 * Startup motor config verification registry.
 *
 * ─── USAGE ───────────────────────────────────────────────────────────────────
 * In robotInit(), after constructing all subsystems and before enabling:
 *
 *   // Swerve — one call per module
 *   ConfigVerifier.register(frontLeftModule);
 *   ConfigVerifier.register(frontRightModule);
 *   ConfigVerifier.register(backLeftModule);
 *   ConfigVerifier.register(backRightModule);
 *
 *   // Mechanisms — same interface
 *   ConfigVerifier.register(myShooter);
 *   ConfigVerifier.register(myElevator);
 *
 *   ConfigVerifier.runAll();
 *
 * ─── WHAT IT CHECKS ──────────────────────────────────────────────────────────
 * Each registered device retries its config apply up to 5× with a 50ms delay,
 * then reads the config back from the device and diffs key fields:
 *
 *   CTRE TalonFX  : inversion, neutralMode, supplyCurrentLimit, statorCurrentLimit
 *   REV Spark     : inversion, idleMode, smartCurrentLimit
 *   ThriftyNova   : inversion, brakeMode, motorType (if withNovaMotorType() was set)
 *
 * ─── FAULT SURFACE ───────────────────────────────────────────────────────────
 * dashboard key  "ConfigVerifier/Status"     "OK" or "FAULTS DETECTED"
 * dashboard key  "ConfigVerifier/Faults"     blank-separated fault details
 * Driver Station console — DriverStationErrors.reportError() for any failure
 *
 * A per-device pass/fail is also pushed under "ConfigVerifier/<DeviceName>".
 */
public final class ConfigVerifier {

    private static final List<ConfigVerifiable> registry = new ArrayList<>();

    private ConfigVerifier() {}

    /**
     * Register a device or mechanism group for startup verification.
     * Call once per device after construction, before runAll().
     *
     * @param device any ConfigVerifiable — swerveModule, mechanismUnit subclass,
     *               shooterMechanism, elevatorMechanism, etc.
     */
    public static void register(ConfigVerifiable device) {
        registry.add(device);
    }

    /**
     * Run verification for all registered devices and surface results.
     * Call once in robotInit(), after all register() calls.
     * Blocks for up to (5 retries × 50ms × device count) — typically < 2s total.
     */
    public static void runAll() {
        if (registry.isEmpty()) {
            Telemetry.log("ConfigVerifier/Status", "OK (no devices registered)");
            Telemetry.log("ConfigVerifier/Faults", "");
            return;
        }

        List<ConfigVerifyResult> allResults = new ArrayList<>();
        for (ConfigVerifiable device : registry) {
            allResults.addAll(device.verifyConfig());
        }

        // Publish per-device pass/fail
        for (ConfigVerifyResult r : allResults) {
            Telemetry.log(
                "ConfigVerifier/" + r.deviceName,
                r.passed() ? "OK" : "FAIL");
        }

        // Aggregate failures
        List<ConfigVerifyResult> failures = new ArrayList<>();
        for (ConfigVerifyResult r : allResults) {
            if (!r.passed()) failures.add(r);
        }

        if (failures.isEmpty()) {
            Telemetry.log("ConfigVerifier/Status", "OK");
            Telemetry.log("ConfigVerifier/Faults", "");
            System.out.println("[ConfigVerifier] All " + allResults.size()
                               + " motor controller(s) verified OK.");
            return;
        }

        // Build fault string
        StringBuilder faultStr = new StringBuilder();
        for (ConfigVerifyResult r : failures) {
            if (faultStr.length() > 0) faultStr.append("\n");
            faultStr.append(r.toString());
        }

        Telemetry.log("ConfigVerifier/Status",
            failures.size() + " FAULT(S) DETECTED — CHECK DRIVER STATION");
        Telemetry.log("ConfigVerifier/Faults", faultStr.toString());

        // DS console — loud and visible before enabling
        DriverStationErrors.reportError(
            "[ConfigVerifier] *** MOTOR CONFIG FAULT(S) — "
            + failures.size() + " of " + allResults.size() + " device(s) failed ***\n"
            + faultStr,
            false);
    }
}
