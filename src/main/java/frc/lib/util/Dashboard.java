package frc.lib.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Dashboard.java
 * PATH: src/main/java/frc/lib/util/Dashboard.java
 *
 * The one place robot and lib code talk to the dashboard. Every caller uses
 * Dashboard.log() for values to view and Dashboard.tunable()/toggle() for values
 * an operator edits live — never the WPILib dashboard API directly.
 *
 * This is the WPILib 2026 implementation, on SmartDashboard (NetworkTables
 * /SmartDashboard/<key>). The WPILib 2027 branch has the same API implemented
 * on Telemetry and Tunables, so moving between years changes only this file.
 * It is also the place to add policy later — e.g. limiting what is published
 * during a match.
 *
 * ─── USAGE ───────────────────────────────────────────────────────────────────
 *   Dashboard.log("Drive/Heading (deg)", headingDeg);        // view only
 *
 *   Dashboard.TunableNumber kP = Dashboard.tunable("Arm/PID/kP", 0.5);  // once
 *   double p = kP.get();                                      // every loop
 *
 * Create each tunable once (constructor / init), not every loop. Creating one
 * publishes its default, overwriting whatever the dashboard currently shows.
 */
public final class Dashboard {

    private Dashboard() {}

    // ─────────────────────────────────────────────────────────────────────────
    // View-only values
    // ─────────────────────────────────────────────────────────────────────────

    public static void log(String key, double value)  { SmartDashboard.putNumber(key, value); }
    public static void log(String key, boolean value) { SmartDashboard.putBoolean(key, value); }
    public static void log(String key, String value)  { SmartDashboard.putString(key, value); }

    // ─────────────────────────────────────────────────────────────────────────
    // Live-editable values
    // ─────────────────────────────────────────────────────────────────────────

    /** Publish a number the operator can edit live, starting at {@code defaultValue}. */
    public static TunableNumber tunable(String key, double defaultValue) {
        return new TunableNumber(key, defaultValue);
    }

    /** Publish a boolean the operator can toggle live (e.g. a dashboard button). */
    public static TunableToggle toggle(String key, boolean defaultValue) {
        return new TunableToggle(key, defaultValue);
    }

    /** A live-editable number. get() returns the dashboard's current value. */
    public static final class TunableNumber {
        private final String key;
        private final double defaultValue;

        private TunableNumber(String key, double defaultValue) {
            this.key          = key;
            this.defaultValue = defaultValue;
            SmartDashboard.putNumber(key, defaultValue);
        }

        public double get()           { return SmartDashboard.getNumber(key, defaultValue); }
        public void   set(double v)   { SmartDashboard.putNumber(key, v); }
    }

    /** A live-editable boolean. get() returns the dashboard's current value. */
    public static final class TunableToggle {
        private final String  key;
        private final boolean defaultValue;

        private TunableToggle(String key, boolean defaultValue) {
            this.key          = key;
            this.defaultValue = defaultValue;
            SmartDashboard.putBoolean(key, defaultValue);
        }

        public boolean get()          { return SmartDashboard.getBoolean(key, defaultValue); }
        public void    set(boolean v) { SmartDashboard.putBoolean(key, v); }
    }
}
