package frc.lib.util;

import java.util.List;

/**
 * Immutable result of a single motor controller config verification pass.
 * One instance is produced per physical motor controller (leader or
 * ENCODER_SYNC follower) checked by ConfigVerifier.runAll().
 *
 * <h2>Fields</h2>
 *
 * <pre>{@code
 * deviceName      Human-readable label, e.g. "FL Drive", "Shooter Flywheel"
 * canId           CAN bus ID of the motor controller
 * vendor          Short vendor/model string, e.g. "CTRE TalonFX"
 * applySucceeded  False if every retry attempt failed — readback not attempted
 * fieldMismatches Each mismatch: "fieldName: expected X got Y"
 * }</pre>
 *
 * <h2>Result interpretation</h2>
 *
 * <pre>{@code
 * passed()        True only when apply succeeded AND no field mismatches
 * }</pre>
 */
public final class ConfigVerifyResult {

    public final String       deviceName;
    public final int          canId;
    public final String       vendor;
    public final boolean      applySucceeded;
    public final List<String> fieldMismatches;

    public ConfigVerifyResult(
            String deviceName,
            int canId,
            String vendor,
            boolean applySucceeded,
            List<String> fieldMismatches) {
        this.deviceName      = deviceName;
        this.canId           = canId;
        this.vendor          = vendor;
        this.applySucceeded  = applySucceeded;
        this.fieldMismatches = List.copyOf(fieldMismatches);
    }

    /** True only when config was applied successfully and all read-back fields matched. */
    public boolean passed() {
        return applySucceeded && fieldMismatches.isEmpty();
    }

    @Override
    public String toString() {
        if (passed()) {
            return deviceName + " (CAN " + canId + ") [" + vendor + "]: OK";
        }
        StringBuilder sb = new StringBuilder();
        sb.append(deviceName).append(" (CAN ").append(canId).append(") [").append(vendor).append("]: FAIL");
        if (!applySucceeded) {
            sb.append(" — config apply failed after all retries");
        }
        for (String m : fieldMismatches) {
            sb.append("\n    ").append(m);
        }
        return sb.toString();
    }
}
