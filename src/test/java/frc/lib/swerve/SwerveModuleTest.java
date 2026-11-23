package frc.lib.swerve;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.hardware.encoder.FakeAbsoluteEncoderIO;
import frc.lib.hardware.motor.FakeMotorIO;
import frc.lib.hardware.motor.MotorIOConfig;
import frc.lib.hardware.motor.MotorVendor;
import frc.lib.util.ConfigVerifyResult;
import frc.lib.util.Dashboard;

/**
 * SwerveModuleTest.java
 * PATH: src/test/java/frc/lib/swerve/SwerveModuleTest.java
 *
 * SwerveModule logic against fakes — startup seeding, command and feedback unit
 * conversion, absolute-encoder resync, and live PID updates. No motor hardware
 * or vendor library is touched.
 *
 * Each test uses a unique module name so dashboard keys never collide.
 */
class SwerveModuleTest {

    private static final double EPS = 1e-9;
    private static final double WHEEL_DIAMETER_IN = 4.0;
    /** Written out from 4 in = 0.1016 m so it doesn't share the code under test's conversion. */
    private static final double WHEEL_CIRCUMFERENCE_M = Math.PI * 0.1016;

    private static final double[] DRIVE_GAINS = { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 };
    private static final double[] STEER_GAINS = { 7.0, 8.0, 9.0, 10.0, 11.0, 0.0 };

    private static int nameCounter = 0;

    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    /** A module on fakes, with the fakes exposed so tests can drive and inspect them. */
    private static final class Rig {
        final String name = "TestModule" + (nameCounter++);
        final FakeMotorIO drive = new FakeMotorIO(
            MotorIOConfig.builder(1, MotorVendor.CTRE_TALONFX).withGains(DRIVE_GAINS).build());
        final FakeMotorIO steer = new FakeMotorIO(
            MotorIOConfig.builder(2, MotorVendor.CTRE_TALONFX).withGains(STEER_GAINS).build());
        final FakeAbsoluteEncoderIO encoder;
        final SwerveModule module;

        Rig(double absoluteDeg) {
            encoder = new FakeAbsoluteEncoderIO(absoluteDeg);
            drive.positionRot = 12.0; // stale count from before this boot
            module = new SwerveModule(name, 0, drive, steer, encoder, WHEEL_DIAMETER_IN);
        }

        /** Simulates an operator editing a gain on the dashboard. */
        void editGain(String gain, double value) {
            Dashboard.tunable("Swerve/" + name + "/PID/" + gain, value);
        }
    }

    private static SwerveModuleState state(double speedMps, double angleDeg) {
        return new SwerveModuleState(speedMps, Rotation2d.fromDegrees(angleDeg));
    }

    /**
     * Rotation2d does not wrap its stored value (270° stays 0.75 rot), so compare
     * wheel direction: angles a whole number of turns apart are equal.
     */
    private static void assertSameDirection(double expectedRot, double actualRot) {
        assertEquals(0.0, MathUtil.inputModulus(actualRot - expectedRot, -0.5, 0.5), 1e-8,
            "expected direction " + expectedRot + " rot but was " + actualRot + " rot");
    }

    // ── Startup ─────────────────────────────────────────────────────────────────

    @Test
    void constructorZeroesTheDriveEncoder() {
        Rig rig = new Rig(0.0);
        assertEquals(0.0, rig.drive.lastEncoderReset, EPS);
        assertEquals(0.0, rig.module.getPosition().distanceMeters, EPS);
    }

    @ParameterizedTest
    @CsvSource({
        "0,     0.0",
        "45,    0.125",
        "90,    0.25",
        "179,   0.4972222222",
        "181,  -0.4972222222",
        "270,  -0.25",
        "359,  -0.0027777778",
    })
    void constructorSeedsSteerIntoHalfTurnRange(double absoluteDeg, double expectedRot) {
        Rig rig = new Rig(absoluteDeg);
        assertEquals(expectedRot, rig.steer.lastEncoderReset, 1e-8);
    }

    @Test
    void seedAtExactlyHalfTurnStillPointsBackward() {
        Rig rig = new Rig(180.0);
        assertEquals(0.5, Math.abs(rig.steer.lastEncoderReset), EPS);
        assertEquals(180.0, Math.abs(rig.module.getState().angle.getDegrees()), 1e-6);
    }

    @Test
    void constructorDoesNotCommandMotionOrRewriteGains() {
        Rig rig = new Rig(90.0);
        assertEquals(FakeMotorIO.Mode.NONE, rig.drive.lastMode);
        assertEquals(FakeMotorIO.Mode.NONE, rig.steer.lastMode);
        assertNull(rig.drive.lastGains);
        assertNull(rig.steer.lastGains);
    }

    // ── Commands ────────────────────────────────────────────────────────────────

    @ParameterizedTest
    @CsvSource({ "0.0", "1.0", "-1.0", "4.5", "-4.5" })
    void driveSpeedIsSentAsWheelRotationsPerSecond(double speedMps) {
        Rig rig = new Rig(0.0);
        rig.module.setDesiredState(state(speedMps, 0.0));

        assertEquals(FakeMotorIO.Mode.VELOCITY, rig.drive.lastMode);
        assertEquals(speedMps / WHEEL_CIRCUMFERENCE_M, rig.drive.lastVelocityRps, EPS);
        assertEquals(0.0, rig.drive.lastFFVolts, EPS);
    }

    @ParameterizedTest
    @CsvSource({
        "0,     0.0",
        "90,    0.25",
        "-90,  -0.25",
        "270,  -0.25",
        "135,   0.375",
        "-135, -0.375",
    })
    void steerAngleIsSentAsAzimuthRotations(double angleDeg, double expectedRot) {
        Rig rig = new Rig(0.0);
        rig.module.setDesiredState(state(0.0, angleDeg));

        assertEquals(FakeMotorIO.Mode.POSITION, rig.steer.lastMode);
        assertSameDirection(expectedRot, rig.steer.lastPositionRot);
        assertEquals(0.0, rig.steer.lastFFVolts, EPS);
    }

    // ── Feedback ────────────────────────────────────────────────────────────────

    @Test
    void stateReportsSpeedInMetersPerSecond() {
        Rig rig = new Rig(0.0);
        rig.drive.velocityRps = -2.5;
        rig.steer.positionRot = 0.25;

        SwerveModuleState measured = rig.module.getState();

        assertEquals(-2.5 * WHEEL_CIRCUMFERENCE_M, measured.speedMetersPerSecond, EPS);
        assertEquals(90.0, measured.angle.getDegrees(), 1e-6);
    }

    @Test
    void positionReportsDistanceInMeters() {
        Rig rig = new Rig(0.0);
        rig.drive.positionRot = 3.0;
        rig.steer.positionRot = -0.125;

        SwerveModulePosition measured = rig.module.getPosition();

        assertEquals(3.0 * WHEEL_CIRCUMFERENCE_M, measured.distanceMeters, EPS);
        assertEquals(-45.0, measured.angle.getDegrees(), 1e-6);
    }

    @Test
    void multiTurnSteerPositionPointsTheSameDirection() {
        Rig rig = new Rig(0.0);
        rig.steer.positionRot = 3.25;
        assertSameDirection(0.25, rig.module.getState().angle.getRotations());
        assertSameDirection(0.25, rig.module.getPosition().angle.getRotations());
    }

    // ── Absolute-encoder resync ─────────────────────────────────────────────────

    @ParameterizedTest
    @CsvSource({
        "90,    0.25,     within threshold",
        "91,    0.25,     within threshold",
        "89,    0.25,     within threshold",
        "359.5, 0.00139,  across the zero wrap",
        "0.5,  -0.00139,  across the zero wrap",
        "90,    3.25,     several turns ahead",
        "90,   -2.75,     several turns behind",
    })
    void syncLeavesAnAgreeingEncoderAlone(double absoluteDeg, double internalRot, String why) {
        Rig rig = new Rig(0.0);
        rig.encoder.setAngleDeg(absoluteDeg);
        rig.steer.positionRot = internalRot;
        rig.steer.lastEncoderReset = Double.NaN;

        rig.module.syncSteerToAbsolute();

        assertTrue(Double.isNaN(rig.steer.lastEncoderReset), "should not reseed: " + why);
        assertEquals(internalRot, rig.steer.positionRot, EPS);
    }

    @ParameterizedTest
    @CsvSource({
        "93,    0.25,      0.2583333333",
        "87,    0.25,      0.2416666667",
        "357,   0.00556,  -0.0083333333",
        "270,   0.25,     -0.25",
        "90,    3.5,       0.25",
    })
    void syncReseedsADisagreeingEncoder(double absoluteDeg, double internalRot, double expectedRot) {
        Rig rig = new Rig(0.0);
        rig.encoder.setAngleDeg(absoluteDeg);
        rig.steer.positionRot = internalRot;

        rig.module.syncSteerToAbsolute();

        assertEquals(expectedRot, rig.steer.lastEncoderReset, 1e-8);
        assertEquals(expectedRot, rig.steer.positionRot, 1e-8);
    }

    @Test
    void syncNeverCommandsMotionOrTouchesTheDriveEncoder() {
        Rig rig = new Rig(0.0);
        rig.drive.positionRot = 7.0;
        rig.encoder.setAngleDeg(120.0);

        rig.module.syncSteerToAbsolute();

        assertEquals(0, rig.drive.commandCount);
        assertEquals(0, rig.steer.commandCount);
        assertEquals(7.0, rig.drive.positionRot, EPS);
    }

    // ── Live PID updates ────────────────────────────────────────────────────────

    @Test
    void unchangedGainsAreNotReapplied() {
        Rig rig = new Rig(0.0);
        rig.module.setDesiredState(state(1.0, 0.0));
        rig.module.setDesiredState(state(1.0, 0.0));
        assertNull(rig.drive.lastGains);
        assertNull(rig.steer.lastGains);
    }

    @Test
    void driveGainEditReachesOnlyTheDriveMotor() {
        Rig rig = new Rig(0.0);
        rig.editGain("Drive kP", 20.0);

        rig.module.setDesiredState(state(1.0, 0.0));

        assertArrayEquals(new double[]{ 20.0, 2.0, 3.0, 4.0, 5.0, 6.0 }, rig.drive.lastGains, EPS);
        assertNull(rig.steer.lastGains);
    }

    @Test
    void steerGainEditReachesOnlyTheSteerMotor() {
        Rig rig = new Rig(0.0);
        rig.editGain("Steer kD", 30.0);

        rig.module.setDesiredState(state(1.0, 0.0));

        assertArrayEquals(new double[]{ 7.0, 8.0, 30.0, 10.0, 11.0, 0.0 }, rig.steer.lastGains, EPS);
        assertNull(rig.drive.lastGains);
    }

    @Test
    void aGainEditIsAppliedOnlyOnce() {
        Rig rig = new Rig(0.0);
        rig.editGain("Drive kV", 50.0);
        rig.module.setDesiredState(state(1.0, 0.0));
        rig.drive.lastGains = null;

        rig.module.setDesiredState(state(1.0, 0.0));

        assertNull(rig.drive.lastGains);
    }

    // ── Verification and telemetry ──────────────────────────────────────────────

    @Test
    void verifyConfigCoversBothMotors() {
        Rig rig = new Rig(0.0);

        List<ConfigVerifyResult> results = rig.module.verifyConfig();

        assertEquals(2, results.size());
        assertEquals(1, rig.drive.verifyCount);
        assertEquals(1, rig.steer.verifyCount);
    }

    @Test
    void telemetryDoesNotCommandTheMotors() {
        Rig rig = new Rig(45.0);
        rig.module.setDesiredState(state(1.0, 45.0));
        int driveCommands = rig.drive.commandCount;
        int steerCommands = rig.steer.commandCount;

        rig.module.publishTelemetry();

        assertEquals(driveCommands, rig.drive.commandCount);
        assertEquals(steerCommands, rig.steer.commandCount);
    }
}
