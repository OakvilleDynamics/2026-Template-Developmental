package frc.lib.hardware.motor;

/**
 * Motor controller vendor. Selects which IMotorIO implementation
 * IMotorIO.create() returns — changing this value is the only code change
 * needed to move a mechanism or swerve module to a different controller.
 *
 * <ul>
 *   <li>{@code CTRE_TALONFX} — Kraken X60, Kraken X44, Minion (via TalonFX)</li>
 *   <li>{@code REV_SPARKMAX} — NEO, NEO 550</li>
 *   <li>{@code REV_SPARKFLEX} — NEO Vortex</li>
 *   <li>{@code THRIFTYBOT_NOVA} — Nova controller (NEO, Minion, Pulsar, brushed)</li>
 * </ul>
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
