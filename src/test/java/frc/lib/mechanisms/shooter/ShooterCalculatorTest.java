package frc.lib.mechanisms.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.lib.mechanisms.shooter.ShooterCalculator.ShooterOutput;
import frc.lib.mechanisms.shooter.ShooterCalculator.ShooterWarning;
import frc.lib.mechanisms.shooter.ShotProfile.LookupEntry;

/**
 * ShooterCalculatorTest.java
 * PATH: src/test/java/frc/lib/mechanisms/shooter/ShooterCalculatorTest.java
 *
 * Pure-math checks of the static-shot path, table interpolation, heading
 * bounds and turret limit warnings. Uses its own small table (pivot at robot
 * center) so results do not change when the robot's shot table is tuned.
 *
 * Replaces the old ShooterCalculator.main() smoke test.
 */
class ShooterCalculatorTest {

    private static final double EPS = 1e-9;

    /** Two hood-angle groups; rows deliberately unsorted. */
    private static final ShotProfile PROFILE = new ShotProfile(new LookupEntry[]{
        new LookupEntry("Hub", 3750, 58.0, 16, 0, 0.0),
        new LookupEntry("Hub", 2750, 58.0,  8, 0, 0.0),
        new LookupEntry("Hub", 3250, 58.0, 12, 0, 0.0),
        new LookupEntry("Hub", 3000, 62.0,  8, 0, 0.0),
        new LookupEntry("Hub", 2500, 62.0,  4, 0, 0.0),
    }, 0.0, 0.0, 10.0);

    /** Turret soft/hard limits ±90°. */
    private static ShooterCalculator calc() {
        return new ShooterCalculator(PROFILE, 50, 3, -90, 90, -90, 90);
    }

    /** Robot on the field +X axis at {@code distanceFt}, facing the target at the origin. */
    private static ShooterOutput shotFrom(double distanceFt) {
        return calc().calculateStaticShot("Hub", 0, 0, distanceFt, 0, 180);
    }

    @Test
    void exactDistanceUsesTableRowAndLowestHoodAngle() {
        ShooterOutput out = shotFrom(8.0);           // 8 ft exists in both groups
        assertEquals(2750, out.rpm, EPS);
        assertEquals(58.0, out.hoodAngle, EPS);
        assertFalse(out.hasWarnings());
    }

    @Test
    void interpolatesWithinBestFittingGroup() {
        ShooterOutput out = shotFrom(14.0);
        assertEquals(3500, out.rpm, EPS);
        assertEquals(58.0, out.hoodAngle, EPS);
        assertFalse(out.warnings.contains(ShooterWarning.DISTANCE_OUT_OF_TABLE_RANGE));
    }

    @Test
    void extrapolatesBeyondTableAndWarns() {
        ShooterOutput out = shotFrom(20.0);
        assertEquals(4250, out.rpm, EPS);            // 12→16 slope extended to 20
        assertTrue(out.warnings.contains(ShooterWarning.DISTANCE_OUT_OF_TABLE_RANGE));
    }

    @Test
    void headingsPointFromPivotToTarget() {
        ShooterOutput out = shotFrom(12.0);
        assertEquals(180.0, out.shooterHeadingFieldDeg, EPS);
        assertEquals(0.0, out.shooterHeadingRobotDeg, EPS);
        assertEquals(0.0, out.turretAngleRobotDeg, EPS);
    }

    @Test
    void unknownTargetReturnsEmptyOutputWithWarning() {
        ShooterOutput out = calc().calculateStaticShot("Speaker", 0, 0, 10, 0, 180);
        assertTrue(out.warnings.contains(ShooterWarning.UNKNOWN_TARGET_TYPE));
        assertEquals(0.0, out.rpm, EPS);
    }

    @Test
    void turretBeyondSoftLimitWarns() {
        // Facing away from the target → turret would need 180°
        ShooterOutput out = calc().calculateStaticShot("Hub", 0, 0, 12, 0, 0);
        assertTrue(out.warnings.contains(ShooterWarning.TURRET_ANGLE_EXCEEDS_LIMITS));
        assertFalse(out.isHeadingInBounds(0.0));
    }

    @Test
    void headingBoundsWrapAcross180() {
        // Aim at 180° with ±90° turret → valid robot headings [90°, -90°] through 180°
        ShooterOutput out = shotFrom(12.0);
        assertEquals(90.0, out.headingBoundsDeg[0], EPS);
        assertEquals(-90.0, out.headingBoundsDeg[1], EPS);
        assertTrue(out.isHeadingInBounds(180.0));
        assertTrue(out.isHeadingInBounds(-135.0));
        assertTrue(out.isHeadingInBounds(90.0));
        assertFalse(out.isHeadingInBounds(0.0));
    }

    @Test
    void isInArcHandlesBothWindowShapes() {
        assertTrue(ShooterOutput.isInArc(10, -20, 20));
        assertFalse(ShooterOutput.isInArc(30, -20, 20));
        assertTrue(ShooterOutput.isInArc(175, 150, -160));
        assertTrue(ShooterOutput.isInArc(-170, 150, -160));
        assertFalse(ShooterOutput.isInArc(0, 150, -160));
    }

    @Test
    void swerveAsTurretBoundsCollapseToAimHeading() {
        ShooterCalculator noTurret = new ShooterCalculator(PROFILE, 50, 3, 0, 0, 0, 0);
        ShooterOutput out = noTurret.calculateStaticShot("Hub", 0, 0, 0, 12, -90);
        assertEquals(-90.0, out.headingBoundsDeg[0], EPS);
        assertEquals(-90.0, out.headingBoundsDeg[1], EPS);
    }
}
