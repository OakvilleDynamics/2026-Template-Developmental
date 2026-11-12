package frc.lib.hardware.motor;

/**
 * MotorVendor.java
 * PATH: src/main/java/frc/lib/hardware/motor/MotorVendor.java
 *
 * Motor controller vendor. Selects which MotorIO implementation
 * MotorIO.create() returns — changing this value is the only code change
 * needed to move a mechanism or swerve module to a different controller.
 *
 *   CTRE_TALONFX    — Kraken X60, Kraken X44, Minion (via TalonFX)
 *   REV_SPARKMAX    — NEO, NEO 550
 *   REV_SPARKFLEX   — NEO Vortex
 *   THRIFTYBOT_NOVA — Nova controller (NEO, Minion, Pulsar, brushed)
 */
public enum MotorVendor {
    CTRE_TALONFX("CTRE TalonFX"),
    REV_SPARKMAX("REV SparkMax"),
    REV_SPARKFLEX("REV SparkFlex"),
    THRIFTYBOT_NOVA("ThriftyBot Nova");

    /** Human-readable name used in startup banners and ConfigVerifier output. */
    public final String displayName;

    MotorVendor(String displayName) {
        this.displayName = displayName;
    }
}
