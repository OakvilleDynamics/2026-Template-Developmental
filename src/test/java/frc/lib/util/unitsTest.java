package frc.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

/**
 * unitsTest.java
 * PATH: src/test/java/frc/lib/util/unitsTest.java
 *
 * Every English ↔ SI conversion round-trips, and angle normalization lands in
 * (-180°, 180°].
 */
class unitsTest {

    private static final double EPS = 1e-9;

    @Test
    void lengthConversions() {
        assertEquals(0.0254, units.inches_m(1.0), EPS);
        assertEquals(0.3048, units.feet_m(1.0), EPS);
        assertEquals(12.0, units.m_inches(units.feet_m(1.0)), EPS);
        assertEquals(7.5, units.m_feet(units.feet_m(7.5)), EPS);
        assertEquals(units.feet_m(3.0), units.fieldFeet_m(3.0), EPS);
    }

    @Test
    void speedAndAccelRoundTrip() {
        assertEquals(14.0, units.mps_ftps(units.ftps_mps(14.0)), EPS);
        assertEquals(9.0, units.mps2_ftps2(units.ftps2_mps2(9.0)), EPS);
    }

    @Test
    void massInertiaTorque() {
        assertEquals(0.453592, units.lbs_kg(1.0), EPS);
        assertEquals(2.92639e-4, units.lbIn2_kgM2(1.0), EPS);
        assertEquals(0.112985, units.lbIn_Nm(1.0), EPS);
    }

    @Test
    void normalizeAngleDegRange() {
        assertEquals(-170.0, units.normalizeAngleDeg(190.0), EPS);
        assertEquals(180.0, units.normalizeAngleDeg(-180.0), EPS);
        assertEquals(180.0, units.normalizeAngleDeg(540.0), EPS);
        assertEquals(0.0, units.normalizeAngleDeg(720.0), EPS);
        assertEquals(-90.0, units.normalizeAngleDeg(270.0), EPS);
    }
}
