package frc.lib.util;

import org.wpilib.telemetry.Telemetry;
import org.wpilib.tunable.TunableBoolean;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;

/**
 * Dashboard.java
 * PATH: src/main/java/frc/lib/util/Dashboard.java
 *
 * The one place robot and lib code talk to the dashboard. Every caller uses
 * Dashboard.log() for values to view and Dashboard.tunable()/toggle() for values
 * an operator edits live — never the WPILib dashboard API directly.
 *
 * This is the WPILib 2027 implementation: view-only values go to Telemetry
 * (NetworkTables /Telemetry/<key>) and live-editable values to Tunables
 * (/Tunables/<key>). SmartDashboard is removed in 2027. The WPILib 2026 branch
 * has the same API implemented on SmartDashboard, so moving between years
 * changes only this file.
 * It is also the place to add policy later — e.g. limiting what is published
 * during a match.
 *
 * ─── USAGE ───────────────────────────────────────────────────────────────────
 *   Dashboard.log("Drive/Heading (deg)", headingDeg);        // view only
 *
 *   Dashboard.TunableNumber kP = Dashboard.tunable("Arm/PID/kP", 0.5);  // once
 *   double p = kP.get();                                      // every loop
 *
 * Create each tunable once (constructor / init), not every loop, and use each
 * key for only one tunable.
 */
public final class Dashboard {

    private Dashboard() {}

    // ─────────────────────────────────────────────────────────────────────────
    // View-only values
    // ─────────────────────────────────────────────────────────────────────────

    public static void log(String key, double value)  { Telemetry.log(key, value); }
    public static void log(String key, boolean value) { Telemetry.log(key, value); }
    public static void log(String key, String value)  { Telemetry.log(key, value); }

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
        private final TunableDouble tunable;

        private TunableNumber(String key, double defaultValue) {
            tunable = Tunables.addDouble(key, defaultValue);
        }

        public double get()           { return tunable.get(); }
        public void   set(double v)   { tunable.set(v); }
    }

    /** A live-editable boolean. get() returns the dashboard's current value. */
    public static final class TunableToggle {
        private final TunableBoolean tunable;

        private TunableToggle(String key, boolean defaultValue) {
            tunable = Tunables.addBoolean(key, defaultValue);
        }

        public boolean get()          { return tunable.get(); }
        public void    set(boolean v) { tunable.set(v); }
    }
}
