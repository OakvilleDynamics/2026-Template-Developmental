package frc.lib.control.ff;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.function.Supplier;

import org.junit.jupiter.api.Test;

import frc.lib.hardware.motor.MotorModels;
import frc.lib.swerve.DriveOdometryState;
import frc.lib.util.Units;

/**
 * FFTest.java
 * PATH: src/test/java/frc/lib/control/ff/FFTest.java
 *
 * Pure-math checks of the physics feed-forward factories. Expected values are
 * computed independently from V = τ × 12 / (gearRatio × stallTorqueNm).
 */
class FFTest {

    private static final double EPS = 1e-9;
    private static final double G   = 9.80665;
    private static final MotorModels.MotorModel NEO = MotorModels.NEO;

    private static double volts(double torqueNm, double gearRatio) {
        return torqueNm * 12.0 / (gearRatio * NEO.stallTorqueNm);
    }

    @SuppressWarnings("unchecked")
    private static Supplier<Integer>[] noPieces() { return new Supplier[0]; }

    // ── none ──────────────────────────────────────────────────────────────────

    @Test
    void noneIsAlwaysZero() {
        assertEquals(0.0, FF.none().compute(37.0, 4.0, 1.0), EPS);
    }

    // ── rotatingArm ───────────────────────────────────────────────────────────

    @Test
    void rotatingArmFullLoadAtHorizontal() {
        FFProvider ff = FF.rotatingArm(NEO, 100.0, 4.0, 10.0, null, null, null);
        double torque = Units.lbsToKg(4.0) * Units.inchesToMeters(10.0) * G;
        assertEquals(volts(torque, 100.0), ff.compute(0.0, 0.0, 0.0), EPS);
    }

    @Test
    void rotatingArmZeroAtVerticalAndNegativePastVertical() {
        FFProvider ff = FF.rotatingArm(NEO, 100.0, 4.0, 10.0, null, null, null);
        assertEquals(0.0, ff.compute(90.0, 0.0, 0.0), 1e-12);
        assertTrue(ff.compute(135.0, 0.0, 0.0) < 0.0, "past vertical, gravity pulls the other way");
        assertEquals(-ff.compute(0.0, 0, 0), ff.compute(180.0, 0, 0), EPS);
    }

    @Test
    @SuppressWarnings("unchecked")
    void rotatingArmAddsHeldGamePieces() {
        int[] count = { 0 };
        FFProvider ff = FF.rotatingArm(NEO, 50.0, 2.0, 8.0,
            new double[]{ 1.5 }, new double[]{ 20.0 }, new Supplier[]{ () -> count[0] });
        double empty = ff.compute(0.0, 0, 0);
        count[0] = 2;
        double loaded = ff.compute(0.0, 0, 0);
        double pieceTorque = 2 * Units.lbsToKg(1.5) * Units.inchesToMeters(20.0) * G;
        assertEquals(empty + volts(pieceTorque, 50.0), loaded, EPS);
    }

    @Test
    @SuppressWarnings("unchecked")
    void rotatingArmRejectsMismatchedArrays() {
        assertThrows(IllegalArgumentException.class, () ->
            FF.rotatingArm(NEO, 50.0, 2.0, 8.0,
                new double[]{ 1.0, 2.0 }, new double[]{ 5.0 }, new Supplier[]{ () -> 0 }));
    }

    // ── springTurret ──────────────────────────────────────────────────────────

    @Test
    void springTurretInterpolatesBetweenPoints() {
        FFProvider ff = FF.springTurret(NEO, 10.0, new double[][]{ { 0, 0 }, { 100, 20 } });
        assertEquals(volts(Units.lbInToNm(10.0), 10.0), ff.compute(50.0, 0.0, 0.0), EPS);
    }

    @Test
    void springTurretNegatesInReverseAndHoldsForwardInDeadband() {
        FFProvider ff = FF.springTurret(NEO, 10.0, new double[][]{ { 0, 10 }, { 100, 10 } });
        double fwd = ff.compute(50.0, 1.0, 0.0);
        assertEquals(-fwd, ff.compute(50.0, -1.0, 0.0), EPS);
        // Within ±FRICTION_DEADBAND_RPS → forward convention (no sign jitter while holding)
        assertEquals(fwd, ff.compute(50.0, -0.5 * FF.FRICTION_DEADBAND_RPS, 0.0), EPS);
    }

    @Test
    void springTurretZeroOutsideCalibratedRange() {
        FFProvider ff = FF.springTurret(NEO, 10.0, new double[][]{ { 0, 10 }, { 100, 10 } });
        assertEquals(0.0, ff.compute(-1.0, 0, 0), EPS);
        assertEquals(0.0, ff.compute(101.0, 0, 0), EPS);
    }

    @Test
    void springTurretRejectsBadCalibration() {
        assertThrows(IllegalArgumentException.class,
            () -> FF.springTurret(NEO, 10.0, new double[][]{ { 0, 1 } }));
        assertThrows(IllegalArgumentException.class,
            () -> FF.springTurret(NEO, 10.0, new double[][]{ { 50, 1 }, { 10, 2 } }));
    }

    // ── gyroscopicTurret ──────────────────────────────────────────────────────

    @Test
    void gyroscopicTurretScalesWithBothSpeedsAndFlipsSign() {
        double[] flywheelRps = { 50.0 };
        FFProvider ff = FF.gyroscopicTurret(NEO, 20.0, 10.0, () -> flywheelRps[0]);
        double expected = volts(Units.lbIn2ToKgM2(10.0) * (50.0 * 2 * Math.PI) * (0.5 * 2 * Math.PI), 20.0);
        assertEquals(expected, ff.compute(0.0, 0.5, 0.0), EPS);
        assertEquals(0.0, ff.compute(0.0, 0.0, 0.0), EPS);
        flywheelRps[0] = -50.0;
        assertEquals(-expected, ff.compute(0.0, 0.5, 0.0), EPS);
    }

    // ── multiStageElevator ────────────────────────────────────────────────────

    @Test
    void multiStageElevatorGravityAndFrictionDirection() {
        double[] angle = { 90.0 };
        FFProvider ff = FF.multiStageElevator(NEO, 12.0, 1.0, 10.0, new double[]{ 5.0 },
            0.3, -0.2, null, noPieces(), () -> angle[0]);
        double torque = Units.lbsToKg(15.0) * G * Units.inchesToMeters(1.0);
        assertEquals(volts(torque, 12.0) + 0.3, ff.compute(0, 0.0, 0), EPS);   // holding → up friction
        assertEquals(volts(torque, 12.0) - 0.2, ff.compute(0, -1.0, 0), EPS);  // moving down
        angle[0] = 0.0;                                                         // horizontal: no gravity
        assertEquals(0.3, ff.compute(0, 0.0, 0), EPS);
    }

    // ── pivotingElevator ──────────────────────────────────────────────────────

    @Test
    void pivotingElevatorAtRestIsPureGravity() {
        DriveOdometryState.Bucket zero = DriveOdometryState.Bucket.zero();
        DriveOdometryState still = new DriveOdometryState(zero, zero, zero, 0.0);
        FFProvider ff = FF.pivotingElevator(NEO, 80.0, 20.0, 0.0, 10.0, null, 0.0, 0.0,
            null, noPieces(), () -> still, () -> 0.0);
        double torque = Units.lbsToKg(10.0) * G * Units.inchesToMeters(20.0);
        assertEquals(volts(torque, 80.0), ff.compute(0.0, 0.0, 0.0), EPS);
        assertEquals(0.0, ff.compute(90.0, 0.0, 0.0), 1e-12);
    }
}
