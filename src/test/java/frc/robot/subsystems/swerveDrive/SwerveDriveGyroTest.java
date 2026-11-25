package frc.robot.subsystems.swerveDrive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;

import frc.lib.hardware.encoder.FakeAbsoluteEncoderIO;
import frc.lib.hardware.gyro.FakeGyroIO;
import frc.lib.hardware.motor.FakeMotorIO;
import frc.lib.swerve.DriveInput;
import frc.lib.swerve.SwerveModule;

/**
 * SwerveDriveGyroTest.java
 * PATH: src/test/java/frc/robot/subsystems/swerveDrive/SwerveDriveGyroTest.java
 *
 * Proves SwerveDrive runs on an injected IGyroIO with no Pigeon or CAN bus:
 * yaw is read from the gyro, zeroed through it, and used for field-relative drive.
 */
class SwerveDriveGyroTest {

    private static final double EPS = 1e-6;
    private static final double[] SQUARE_IN = { 22.0, 22.0 };
    private static final double WHEEL_DIAMETER_IN = 4.0;
    private static final double WHEEL_CIRCUMFERENCE_M = Math.PI * 0.1016;
    private static final double ONE_FTPS_IN_MPS = 0.3048;

    private static int nameCounter = 0;

    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    private static final class Rig {
        final FakeGyroIO gyro = new FakeGyroIO();
        final FakeMotorIO[] driveMotors = new FakeMotorIO[4];
        final FakeMotorIO[] steerMotors = new FakeMotorIO[4];
        final SwerveDrive drive;

        Rig(double startYawDeg) {
            gyro.yawDeg = startYawDeg;
            SwerveModule[] modules = new SwerveModule[4];
            for (int i = 0; i < 4; i++) {
                driveMotors[i] = FakeMotorIO.withId(10 + i);
                steerMotors[i] = FakeMotorIO.withId(20 + i);
                modules[i] = new SwerveModule("GyroTest" + (nameCounter++), i,
                    driveMotors[i], steerMotors[i], new FakeAbsoluteEncoderIO(0.0), WHEEL_DIAMETER_IN);
            }
            drive = new SwerveDrive(modules, gyro, SQUARE_IN, SQUARE_IN, SQUARE_IN,
                                    WHEEL_DIAMETER_IN, new double[]{ 1.0, 0.0, 0.0 });
        }
    }

    @Test
    void yawComesFromTheGyro() {
        Rig rig = new Rig(37.0);
        assertEquals(37.0, rig.drive.getYaw().getDegrees(), EPS);
    }

    @Test
    void zeroYawGoesThroughTheGyro() {
        Rig rig = new Rig(37.0);

        rig.drive.zeroYaw();

        assertEquals(1, rig.gyro.setYawCount);
        assertEquals(0.0, rig.drive.getYaw().getDegrees(), EPS);
    }

    @Test
    void fieldForwardAtZeroYawDrivesWheelsStraight() {
        Rig rig = new Rig(0.0);

        rig.drive.drive(new DriveInput(1.0, 0.0, 0.0));

        for (int i = 0; i < 4; i++) {
            assertEquals(0.0, rig.steerMotors[i].lastPositionRot, EPS);
            assertEquals(ONE_FTPS_IN_MPS / WHEEL_CIRCUMFERENCE_M, rig.driveMotors[i].lastVelocityRps, EPS);
        }
    }

    @Test
    void fieldForwardWithRobotFacingLeftSteersWheelsToRobotRight() {
        Rig rig = new Rig(90.0);

        rig.drive.drive(new DriveInput(1.0, 0.0, 0.0));

        for (int i = 0; i < 4; i++) {
            assertEquals(-0.25, rig.steerMotors[i].lastPositionRot, EPS);
            assertEquals(ONE_FTPS_IN_MPS / WHEEL_CIRCUMFERENCE_M, rig.driveMotors[i].lastVelocityRps, EPS);
        }
    }
}
