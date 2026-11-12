package frc.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * DashboardTest.java
 * PATH: src/test/java/frc/lib/util/DashboardTest.java
 *
 * Behaviour every Dashboard implementation (2026 SmartDashboard, 2027
 * Telemetry/Tunables) must share. Keys are unique per test.
 */
class DashboardTest {

    private static final double EPS = 1e-9;

    @Test
    void tunableStartsAtItsDefault() {
        Dashboard.TunableNumber t = Dashboard.tunable("DashboardTest/defaultNumber", 1.5);
        assertEquals(1.5, t.get(), EPS);
    }

    @Test
    void tunableReturnsTheLatestValue() {
        Dashboard.TunableNumber t = Dashboard.tunable("DashboardTest/setNumber", 1.0);
        t.set(4.0);
        assertEquals(4.0, t.get(), EPS);
    }

    @Test
    void toggleStartsAtItsDefaultAndCanBeReset() {
        Dashboard.TunableToggle t = Dashboard.toggle("DashboardTest/toggle", false);
        assertFalse(t.get());
        t.set(true);
        assertTrue(t.get());
        t.set(false);
        assertFalse(t.get());
    }

    @Test
    void logAcceptsEveryValueType() {
        Dashboard.log("DashboardTest/number", 2.0);
        Dashboard.log("DashboardTest/int", 3);
        Dashboard.log("DashboardTest/boolean", true);
        Dashboard.log("DashboardTest/string", "ok");
    }
}
