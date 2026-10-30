package frc.lib.control;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.lib.control.ff.ffProvider;
import frc.lib.hardware.motor.MotorIOConfig;
import frc.lib.hardware.motor.MotorVendor;

/**
 * mechanismConfigTest.java
 * PATH: src/test/java/frc/lib/control/mechanismConfigTest.java
 *
 * mechanismConfig → per-motor MotorIOConfig derivation, copy semantics, and
 * builder validation. Pure Java — no HAL.
 */
class mechanismConfigTest {

    private static final double EPS = 1e-12;

    private static mechanismConfig.Builder threeMotorElevator() {
        return new mechanismConfig.Builder("Elevator", new int[]{ 20, 21, 22 }, MotorVendor.REV_SPARKMAX)
            .withFollowerConfig(
                new motorConstants.FollowMode[]{ motorConstants.FollowMode.MECHANICAL,
                                                 motorConstants.FollowMode.ENCODER_SYNC },
                new boolean[]{ true, true },
                new int[]{ 0, 0 })
            .withGearRatio(12.0)
            .withInverted(true)
            .withPID(new double[]{ 1, 2, 3, 4, 5, 6 })
            .withSoftLimits(-36.0, 720.0)
            .withCurrentLimits(35, 55)
            .withMotionProfile(10, 20, 30);
    }

    @Test
    void leaderGetsEverySettingWithSoftLimitsInRotations() {
        MotorIOConfig leader = threeMotorElevator().build().motorConfig(0);
        assertEquals(20, leader.canId);
        assertEquals(MotorVendor.REV_SPARKMAX, leader.vendor);
        assertTrue(leader.inverted);
        assertEquals(12.0, leader.gearRatio, EPS);
        assertEquals(1.0, leader.kP, EPS);
        assertEquals(6.0, leader.kA, EPS);
        assertEquals(-0.1, leader.softLimitReverseRot, EPS);
        assertEquals(2.0, leader.softLimitForwardRot, EPS);
        assertEquals(35.0, leader.supplyCurrentLimitAmps, EPS);
        assertEquals(10.0, leader.motionCruiseVelocityRps, EPS);
    }

    @Test
    void followersGetHardwareSettingsButNoClosedLoopSettings() {
        mechanismConfig cfg = threeMotorElevator().build();
        MotorIOConfig mech = cfg.motorConfig(1);
        MotorIOConfig sync = cfg.motorConfig(2);

        assertEquals(21, mech.canId);
        assertEquals(12.0, mech.gearRatio, EPS);
        assertEquals(35.0, mech.supplyCurrentLimitAmps, EPS);
        assertEquals(0.0, mech.kP, EPS);
        assertTrue(Double.isNaN(mech.softLimitForwardRot));
        assertFalse(mech.hasMotionProfile());
        // MECHANICAL direction comes from follow(), not the device's own inversion
        assertFalse(mech.inverted);
        // ENCODER_SYNC runs its own output, so it carries its inversion
        assertTrue(sync.inverted);
    }

    @Test
    void unsetSoftLimitsStayDisabled() {
        MotorIOConfig leader = new mechanismConfig.Builder("Roller", new int[]{ 5 }, MotorVendor.THRIFTYBOT_NOVA)
            .build().motorConfig(0);
        assertTrue(Double.isNaN(leader.softLimitForwardRot));
        assertTrue(Double.isNaN(leader.softLimitReverseRot));
    }

    @Test
    void toBuilderPreservesEveryField() {
        // Regression: elevatorMechanism used to rebuild configs by hand and silently
        // dropped travel limits, homing offset, Nova motor type, and CAN signal rates.
        ffProvider t2 = (p, v, a) -> 1.0;
        mechanismConfig src = threeMotorElevator()
            .withTier2FF(t2)
            .withLeadTravelLimits(30, 0.1, 40, 0.2)
            .withFollowerTravelLimits(31, 0.11, 41, 0.21)
            .withStallDetectionCycles(7)
            .withHomingZeroOffset(-3.0)
            .withNovaMotorType(MotorIOConfig.NovaMotorType.MINION)
            .withCanSignals(100, 2, true)
            .withSetpointDeadband(0.5)
            .withEncoderSync(0.02, 0.9)
            .build();

        ffProvider t3 = (p, v, a) -> 2.0;
        mechanismConfig copy = src.toBuilder().withTier3FF(t3).build();

        assertSame(t2, copy.tier2FF);
        assertSame(t3, copy.tier3FF);
        assertEquals(30.0, copy.leadLowerCurrentAmps, EPS);
        assertEquals(0.21, copy.followerUpperMaxSpeedRps, EPS);
        assertEquals(7, copy.stallDetectionCycles);
        assertEquals(-3.0, copy.homingZeroOffsetDeg, EPS);
        assertEquals(MotorIOConfig.NovaMotorType.MINION, copy.novaMotorType);
        assertEquals(100.0, copy.signalUpdateHz, EPS);
        assertTrue(copy.optimizeCanUtilization);
        assertEquals(0.5, copy.setpointDeadband, EPS);
        assertEquals(0.9, copy.encoderSyncKp, EPS);
        assertEquals(motorConstants.FollowMode.ENCODER_SYNC, copy.followModeOf(2));
        assertEquals(-36.0, copy.softLimitReverseDeg, EPS);
    }

    @Test
    void mixedVendorsAreRejected() {
        assertThrows(IllegalArgumentException.class, () ->
            new mechanismConfig.Builder("Bad", new int[]{ 1, 2 },
                new MotorVendor[]{ MotorVendor.REV_SPARKMAX, MotorVendor.CTRE_TALONFX }));
    }

    @Test
    void singleVendorConstructorFillsEveryMotor() {
        mechanismConfig cfg = new mechanismConfig.Builder("Flywheel", new int[]{ 10, 11 }, MotorVendor.CTRE_TALONFX)
            .build();
        assertEquals(MotorVendor.CTRE_TALONFX, cfg.vendors[1]);
        assertEquals("2× CTRE TalonFX | CAN [10, 11]", cfg.toString());
    }

    @Test
    void followerConfigValidatesLeaderIndices() {
        assertThrows(IllegalArgumentException.class, () ->
            new mechanismConfig.Builder("Bad", new int[]{ 1, 2 }, MotorVendor.CTRE_TALONFX)
                .withFollowerConfig(new motorConstants.FollowMode[]{ motorConstants.FollowMode.MECHANICAL },
                                    new boolean[]{ false }, new int[]{ 1 }));   // follows itself
    }
}
