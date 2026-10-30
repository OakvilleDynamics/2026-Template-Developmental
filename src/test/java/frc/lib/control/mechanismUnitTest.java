package frc.lib.control;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.lib.hardware.motor.FakeMotorIO;
import frc.lib.hardware.motor.MotorVendor;
import frc.lib.util.ConfigVerifyResult;

/**
 * mechanismUnitTest.java
 * PATH: src/test/java/frc/lib/control/mechanismUnitTest.java
 *
 * mechanismUnit logic against FakeMotorIO — unit conversion, FF injection,
 * follower wiring, encoder sync, travel limits, homing, live PID tuning.
 * No motor hardware or vendor library is touched.
 *
 * Each test uses a unique mechanism name so SmartDashboard keys never collide.
 */
class mechanismUnitTest {

    private static final double EPS = 1e-9;
    private static int nameCounter = 0;

    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    private static String uniqueName() {
        return "TestMech" + (nameCounter++);
    }

    private static mechanismConfig.Builder single() {
        return new mechanismConfig.Builder(uniqueName(), new int[]{ 1 }, MotorVendor.CTRE_TALONFX);
    }

    // ── Commands reach the leader in rotations with FF attached ─────────────────

    @Test
    void positionIsSentInRotationsWithTier2PlusTier3FF() {
        FakeMotorIO leader = FakeMotorIO.withId(1);
        mechanismUnit unit = new mechanismUnit(single()
            .withTier2FF((p, v, a) -> 1.5)
            .withTier3FF((p, v, a) -> 0.25)
            .build(), leader);

        unit.setPosition(90.0);

        assertEquals(FakeMotorIO.Mode.POSITION, leader.lastMode);
        assertEquals(0.25, leader.lastPositionRot, EPS);
        assertEquals(1.75, leader.lastFFVolts, EPS);
    }

    @Test
    void ffProviderSeesPositionInDegrees() {
        FakeMotorIO leader = FakeMotorIO.withId(1);
        leader.positionRot = 0.5;
        mechanismUnit unit = new mechanismUnit(single().withTier2FF((p, v, a) -> p).build(), leader);
        unit.setVelocity(3.0);
        assertEquals(180.0, leader.lastFFVolts, EPS);
        assertEquals(3.0, leader.lastVelocityRps, EPS);
    }

    @Test
    void softwareMotionProfileOnlyWhenControllerLacksOne() {
        FakeMotorIO nova = FakeMotorIO.withId(1);
        nova.hardwareMotionProfile = false;
        mechanismUnit stepped = new mechanismUnit(single().withMotionProfile(10, 50, 0).build(), nova);
        stepped.setVelocity(10.0);
        assertEquals(50.0 * motorConstants.LOOP_PERIOD_SECS, nova.lastVelocityRps, EPS);
        stepped.setVelocity(10.0);
        assertEquals(2 * 50.0 * motorConstants.LOOP_PERIOD_SECS, nova.lastVelocityRps, EPS);

        FakeMotorIO ctre = FakeMotorIO.withId(2);
        mechanismUnit direct = new mechanismUnit(single().withMotionProfile(10, 50, 0).build(), ctre);
        direct.setVelocity(10.0);
        assertEquals(10.0, ctre.lastVelocityRps, EPS);
    }

    @Test
    void stopResetsSoftwarePositionRamp() {
        FakeMotorIO nova = FakeMotorIO.withId(1);
        nova.hardwareMotionProfile = false;
        mechanismUnit unit = new mechanismUnit(single().withMotionProfile(10, 50, 0).build(), nova);
        nova.positionRot = 2.0;
        unit.setPosition(0.0);                  // ramp seeds from current position (2.0 rot)
        assertEquals(2.0 - 50.0 * motorConstants.LOOP_PERIOD_SECS, nova.lastPositionRot, EPS);
        unit.stop();
        assertEquals(FakeMotorIO.Mode.STOPPED, nova.lastMode);
    }

    // ── Followers ──────────────────────────────────────────────────────────────

    @Test
    void mechanicalFollowersFollowTheirDesignatedLeader() {
        FakeMotorIO a = FakeMotorIO.withId(10), b = FakeMotorIO.withId(11), c = FakeMotorIO.withId(12);
        mechanismConfig cfg = new mechanismConfig.Builder(uniqueName(), new int[]{ 10, 11, 12 }, MotorVendor.CTRE_TALONFX)
            .withFollowerConfig(
                new motorConstants.FollowMode[]{ motorConstants.FollowMode.MECHANICAL,
                                                 motorConstants.FollowMode.MECHANICAL },
                new boolean[]{ false, true },
                new int[]{ 0, 1 })                 // c follows b (follower-of-follower)
            .build();

        new mechanismUnit(cfg, a, b, c);

        assertSame(a, b.followedLeader);
        assertFalse(b.followOpposed);
        assertSame(b, c.followedLeader);
        assertTrue(c.followOpposed);
        assertNull(a.followedLeader);
    }

    @Test
    void encoderSyncRejectedOnControllersThatCannotSync() {
        FakeMotorIO leader = FakeMotorIO.withId(1), follower = FakeMotorIO.withId(2);
        follower.encoderSyncSupported = false;      // e.g. Nova
        mechanismConfig cfg = new mechanismConfig.Builder(uniqueName(), new int[]{ 1, 2 }, MotorVendor.CTRE_TALONFX)
            .withFollowMode(motorConstants.FollowMode.ENCODER_SYNC, new boolean[]{ false })
            .build();
        assertThrows(IllegalArgumentException.class, () -> new mechanismUnit(cfg, leader, follower));
    }

    @Test
    void wrongFollowerCountIsRejected() {
        mechanismConfig cfg = new mechanismConfig.Builder(uniqueName(), new int[]{ 1, 2 }, MotorVendor.CTRE_TALONFX).build();
        assertThrows(IllegalArgumentException.class, () -> new mechanismUnit(cfg, FakeMotorIO.withId(1)));
    }

    @Test
    void encoderSyncCorrectsFollowerOutsideDeadband() {
        FakeMotorIO leader = FakeMotorIO.withId(1), follower = FakeMotorIO.withId(2);
        mechanismUnit unit = new mechanismUnit(
            new mechanismConfig.Builder(uniqueName(), new int[]{ 1, 2 }, MotorVendor.CTRE_TALONFX)
                .withFollowMode(motorConstants.FollowMode.ENCODER_SYNC, new boolean[]{ false })
                .withEncoderSync(0.05, 0.5)
                .build(),
            leader, follower);

        leader.positionRot = 1.0;
        follower.positionRot = 0.8;
        unit.setDutyCycle(0.4);
        assertEquals(0.5 * 0.2, follower.lastDutyCycle, EPS);

        follower.lastDutyCycle = Double.NaN;
        follower.positionRot = 0.98;                 // inside deadband → no correction
        unit.setDutyCycle(0.4);
        assertTrue(Double.isNaN(follower.lastDutyCycle));
    }

    // ── Travel limits ──────────────────────────────────────────────────────────

    @Test
    void travelLimitLatchesAfterStallCyclesThenClearsWhenReversed() {
        FakeMotorIO leader = FakeMotorIO.withId(1);
        mechanismUnit unit = new mechanismUnit(single()
            .withLeadTravelLimits(30.0, 0.1, Double.NaN, Double.NaN)
            .withStallDetectionCycles(3)
            .build(), leader);

        leader.stallCurrentAmps = 45.0;
        leader.velocityRps = 0.0;

        unit.setDutyCycle(-0.5);
        unit.setDutyCycle(-0.5);
        assertFalse(unit.isLowerLimitReached());
        assertEquals(-0.5, leader.lastDutyCycle, EPS);

        unit.setDutyCycle(-0.5);                     // third stalled cycle → latch
        assertTrue(unit.isLowerLimitReached());
        assertEquals(0.0, leader.lastDutyCycle, EPS);

        leader.stallCurrentAmps = 0.0;
        unit.setDutyCycle(0.3);                      // moving away clears the latch
        assertFalse(unit.isLowerLimitReached());
        assertEquals(0.3, leader.lastDutyCycle, EPS);
    }

    @Test
    void transientCurrentSpikeDoesNotLatch() {
        FakeMotorIO leader = FakeMotorIO.withId(1);
        mechanismUnit unit = new mechanismUnit(single()
            .withLeadTravelLimits(30.0, 0.1, Double.NaN, Double.NaN)
            .withStallDetectionCycles(3)
            .build(), leader);

        leader.stallCurrentAmps = 45.0;
        unit.setDutyCycle(-0.5);
        unit.setDutyCycle(-0.5);
        leader.stallCurrentAmps = 5.0;               // spike ends before 3 cycles
        unit.setDutyCycle(-0.5);
        leader.stallCurrentAmps = 45.0;
        unit.setDutyCycle(-0.5);
        assertFalse(unit.isLowerLimitReached());
    }

    // ── Homing ─────────────────────────────────────────────────────────────────

    @Test
    void homingDrivesUntilStallThenZeroesEncoder() {
        FakeMotorIO leader = FakeMotorIO.withId(1);
        mechanismUnit unit = new mechanismUnit(single()
            .withStallDetectionCycles(2)
            .withHomingZeroOffset(-5.0)
            .build(), leader);

        unit.startHoming(mechanismUnit.HomingDirection.NEGATIVE, 0.2, 25.0, 0.05);
        assertTrue(unit.isHoming());

        leader.velocityRps = -1.0;
        unit.updateHoming();
        assertEquals(-0.2, leader.lastDutyCycle, EPS);

        // set*() is ignored while homing
        int before = leader.commandCount;
        unit.setPosition(45.0);
        assertEquals(before, leader.commandCount);

        leader.velocityRps = 0.0;
        leader.stallCurrentAmps = 30.0;
        unit.updateHoming();
        assertTrue(unit.isHoming());
        unit.updateHoming();

        assertTrue(unit.isHomingComplete());
        assertFalse(unit.isHoming());
        assertEquals(0.0, leader.lastDutyCycle, EPS);
        assertEquals(-5.0 / 360.0, leader.lastEncoderReset, EPS);
    }

    @Test
    void cancelHomingStopsWithoutZeroing() {
        FakeMotorIO leader = FakeMotorIO.withId(1);
        mechanismUnit unit = new mechanismUnit(single().build(), leader);
        unit.startHoming(mechanismUnit.HomingDirection.POSITIVE, 0.3, 25.0, 0.05);
        leader.velocityRps = 2.0;
        unit.updateHoming();
        assertEquals(0.3, leader.lastDutyCycle, EPS);
        unit.cancelHoming();
        assertFalse(unit.isHoming());
        assertEquals(0.0, leader.lastDutyCycle, EPS);
        assertTrue(Double.isNaN(leader.lastEncoderReset));
    }

    // ── Setpoint tracking ──────────────────────────────────────────────────────

    @Test
    void isAtSetpointUsesDeadbandInCommandUnits() {
        FakeMotorIO leader = FakeMotorIO.withId(1);
        mechanismUnit unit = new mechanismUnit(single().withSetpointDeadband(2.0).build(), leader);
        unit.setPosition(90.0);
        leader.positionRot = 0.25;
        assertTrue(unit.isAtSetpoint());
        leader.positionRot = 0.3;                    // 108°
        assertFalse(unit.isAtSetpoint());

        unit.setVelocity(20.0);
        leader.velocityRps = 19.0;
        assertTrue(unit.isAtSetpoint());
    }

    @Test
    void isAtSetpointFalseWithoutDeadband() {
        FakeMotorIO leader = FakeMotorIO.withId(1);
        mechanismUnit unit = new mechanismUnit(single().build(), leader);
        unit.setPosition(0.0);
        assertFalse(unit.isAtSetpoint());
    }

    // ── Live PID + verification ────────────────────────────────────────────────

    @Test
    void dashboardGainChangeIsPushedToController() {
        FakeMotorIO leader = FakeMotorIO.withId(1);
        mechanismConfig cfg = single().withPID(new double[]{ 1, 0, 0, 0, 0, 0 }).build();
        mechanismUnit unit = new mechanismUnit(cfg, leader);

        unit.setPosition(0.0);
        assertNull(leader.lastGains, "unchanged gains must not be re-sent");

        SmartDashboard.putNumber(cfg.name + "/PID/kP", 4.0);
        unit.setPosition(0.0);
        assertEquals(4.0, leader.lastGains[0], EPS);
    }

    @Test
    void verifyCoversLeaderAndEncoderSyncFollowersOnly() {
        FakeMotorIO a = FakeMotorIO.withId(1), b = FakeMotorIO.withId(2), c = FakeMotorIO.withId(3);
        mechanismConfig cfg = new mechanismConfig.Builder(uniqueName(), new int[]{ 1, 2, 3 }, MotorVendor.CTRE_TALONFX)
            .withFollowerConfig(
                new motorConstants.FollowMode[]{ motorConstants.FollowMode.MECHANICAL,
                                                 motorConstants.FollowMode.ENCODER_SYNC },
                new boolean[]{ false, false }, new int[]{ 0, 0 })
            .build();
        List<ConfigVerifyResult> results = new mechanismUnit(cfg, a, b, c).verifyConfig();

        assertEquals(2, results.size());
        assertEquals(cfg.name + " Leader", results.get(0).deviceName);
        assertEquals(cfg.name + " Follower[2]", results.get(1).deviceName);
        assertEquals(0, b.verifyCount);
    }
}
