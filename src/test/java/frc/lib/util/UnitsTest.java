package frc.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

/**
 * UnitsTest.java
 * PATH: src/test/java/frc/lib/util/UnitsTest.java
 *
 * Every English ↔ SI conversion round-trips, and angle normalization lands in
 * (-180°, 180°].
 */
class UnitsTest {

    private static final double EPS = 1e-9;

    @Test
    void lengthConversions() {
        assertEquals(0.0254, Units.inchesToMeters(1.0), EPS);
        assertEquals(0.3048, Units.feetToMeters(1.0), EPS);
        assertEquals(12.0, Units.metersToInches(Units.feetToMeters(1.0)), EPS);
        assertEquals(7.5, Units.metersToFeet(Units.feetToMeters(7.5)), EPS);
        assertEquals(Units.feetToMeters(3.0), Units.fieldFeetToMeters(3.0), EPS);
    }

    @Test
    void speedAndAccelRoundTrip() {
        assertEquals(14.0, Units.mpsToFtps(Units.ftpsToMps(14.0)), EPS);
        assertEquals(9.0, Units.mps2ToFtps2(Units.ftps2ToMps2(9.0)), EPS);
    }

    @Test
    void massInertiaTorque() {
        assertEquals(0.453592, Units.lbsToKg(1.0), EPS);
        assertEquals(2.92639e-4, Units.lbIn2ToKgM2(1.0), EPS);
        assertEquals(0.112985, Units.lbInToNm(1.0), EPS);
    }

    @Test
    void normalizeAngleDegRange() {
        assertEquals(-170.0, Units.normalizeAngleDeg(190.0), EPS);
        assertEquals(180.0, Units.normalizeAngleDeg(-180.0), EPS);
        assertEquals(180.0, Units.normalizeAngleDeg(540.0), EPS);
        assertEquals(0.0, Units.normalizeAngleDeg(720.0), EPS);
        assertEquals(-90.0, Units.normalizeAngleDeg(270.0), EPS);
    }
}
