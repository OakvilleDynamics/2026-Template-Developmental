package frc.robot.subsystems.swerveDrive;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.constants.swerveConstants;
import frc.robot.util.RobotLogger;
import frc.robot.util.motors.ConfigVerifiable;
import frc.robot.util.motors.ConfigVerifyResult;

import frc.robot.subsystems.swerveDrive.drive.SwerveDriveMotor;
import frc.robot.subsystems.swerveDrive.drive.CTREDriveMotor;
import frc.robot.subsystems.swerveDrive.drive.REVDriveMotor;
import frc.robot.subsystems.swerveDrive.drive.NovaDriveMotor;
import frc.robot.subsystems.swerveDrive.steer.SwerveSteerMotor;
import frc.robot.subsystems.swerveDrive.steer.SwerveAbsoluteEncoder;
import frc.robot.subsystems.swerveDrive.steer.CTRESteerMotor;
import frc.robot.subsystems.swerveDrive.steer.NovaSteerMotor;
import frc.robot.subsystems.swerveDrive.steer.REVSteerMotor;
import frc.robot.subsystems.swerveDrive.steer.ThriftyAnalogEncoder;
import frc.robot.subsystems.swerveDrive.steer.CTRECANcoderEncoder;
import frc.robot.subsystems.swerveDrive.steer.REVThroughBoreEncoder;

/**
 * swerveModule.java
 * PATH: src/main/java/frc/robot/subsystems/swerveDrive/swerveModule.java
 *
 * One Thrifty Narrow swerve module — vendor-agnostic.
 *
 * Drive and steer motor types are selected in swerveConstants via:
 *   DRIVE_MOTOR_TYPE  — Kraken X60, Kraken X44, NEO Vortex, NEO, Nova/Pulsar
 *   STEER_MOTOR_TYPE  — Kraken X60, Kraken X44, Minion, Nova, NEO Vortex, NEO, NEO 550
 *   ABS_ENCODER_TYPE  — Thrifty analog, CTRE CANcoder, REV Through Bore
 *
 * Changing those three constants is the only code change required to swap hardware.
 * This class contains no vendor-specific logic.
 *
 * ─── UNITS ────────────────────────────────────────────────────────────────────
 * SwerveDriveMotor interface: wheel shaft rotations / rot·s⁻¹ (gear ratio inside)
 * SwerveSteerMotor interface: mechanism shaft rotations [-0.5, 0.5]
 * Public API: meters, Rotation2d (WPILib standard)
 *
 * ─── ENCODER SEEDING ─────────────────────────────────────────────────────────
 * The absolute encoder is read once at startup to seed the steer motor's internal
 * high-resolution encoder. Closed-loop steering uses the internal encoder — not
 * the absolute encoder — to avoid noise at 20 ms update rates.
 *
 * ─── PID LIVE TUNING ─────────────────────────────────────────────────────────
 * All PID values are published to SmartDashboard at startup and re-read every
 * loop. When a value changes, applyDashboardPIDUpdates() delegates to the motor
 * implementation — no vendor-specific logic here.
 */
public class swerveModule implements ConfigVerifiable {

    // ── Vendor-agnostic hardware ──────────────────────────────────────────────
    private final SwerveDriveMotor      driveMotor;
    private final SwerveSteerMotor      steerMotor;
    private final SwerveAbsoluteEncoder absEncoder;

    // ── Identity ──────────────────────────────────────────────────────────────
    private final String name;
    private final int    moduleIndex;  // 0=FL, 1=FR, 2=BL, 3=BR — for RobotLogger indexing

    // ── Live PID values (read back from dashboard each loop) ──────────────────
    private double drive_kP, drive_kI, drive_kD, drive_kS, drive_kV, drive_kA;
    private double steer_kP, steer_kI, steer_kD, steer_kS, steer_kV;

    // ─────────────────────────────────────────────────────────────────────────
    // Constructor
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * @param name             Telemetry label: "FL", "FR", "BL", "BR"
     * @param moduleIndex      0=FL, 1=FR, 2=BL, 3=BR
     * @param driveCanId       Drive motor CAN ID
     * @param steerCanId       Steer motor CAN ID
     * @param analogPort       roboRIO analog port (THRIFTY_ANALOG only; ignored otherwise)
     * @param cancoderCanId    CANcoder CAN ID (CTRE_CANCODER only; ignored otherwise)
     * @param steerOffsetVolts Thrifty encoder voltage when wheel points forward
     * @param steerOffsetRot   Rotation offset when wheel points forward
     *                         (CTRE_CANCODER and REV_THROUGH_BORE; ignored for THRIFTY_ANALOG)
     * @param driveInverted    true if drive motor output should be inverted
     * @param drivePID         { kP, kI, kD, kS, kV, kA }
     * @param steerPID         { kP, kI, kD, kS, kV }
     */
    public swerveModule(
            String name,
            int moduleIndex,
            int driveCanId,
            int steerCanId,
            int analogPort,
            int cancoderCanId,
            double steerOffsetVolts,
            double steerOffsetRot,
            boolean driveInverted,
            double[] drivePID,
            double[] steerPID) {

        this.name        = name;
        this.moduleIndex = moduleIndex;

        drive_kP = drivePID[0]; drive_kI = drivePID[1]; drive_kD = drivePID[2];
        drive_kS = drivePID[3]; drive_kV = drivePID[4]; drive_kA = drivePID[5];
        steer_kP = steerPID[0]; steer_kI = steerPID[1]; steer_kD = steerPID[2];
        steer_kS = steerPID[3]; steer_kV = steerPID[4];

        publishPIDToDashboard();

        driveMotor = buildDriveMotor(driveCanId, driveInverted, drivePID);

        // Steer motor + encoder built together to handle the REV_THROUGH_BORE case
        // where the SparkBase must be shared between encoder and motor controller.
        Object[] steerAndEncoder = buildSteerAndEncoder(
            steerCanId, analogPort, cancoderCanId,
            steerOffsetVolts, steerOffsetRot, steerPID);
        steerMotor = (SwerveSteerMotor)     steerAndEncoder[0];
        absEncoder = (SwerveAbsoluteEncoder) steerAndEncoder[1];

        seedSteerFromAbsolute();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Factory methods — read swerveConstants, return vendor implementations
    // ─────────────────────────────────────────────────────────────────────────

    private static SwerveDriveMotor buildDriveMotor(
            int canId, boolean inverted, double[] pid) {
        double gr = swerveConstants.DRIVE_GEAR_RATIO;
        switch (swerveConstants.DRIVE_MOTOR_TYPE) {
            case KRAKEN_X60:
            case KRAKEN_X44:
                return new CTREDriveMotor(swerveConstants.DRIVE_MOTOR_TYPE, canId, inverted, pid, gr);
            case NEO_VORTEX:
            case NEO:
                return new REVDriveMotor(swerveConstants.DRIVE_MOTOR_TYPE, canId, inverted, pid, gr);
            case NOVA_PULSAR:
                return new NovaDriveMotor(canId, inverted, pid, gr);
            default:
                throw new IllegalArgumentException(
                    "swerveModule: unknown DRIVE_MOTOR_TYPE " + swerveConstants.DRIVE_MOTOR_TYPE);
        }
    }

    /**
     * Returns {SwerveSteerMotor, SwerveAbsoluteEncoder}.
     * Kept together so REV_THROUGH_BORE can share one SparkBase instance.
     */
    private static Object[] buildSteerAndEncoder(
            int steerCanId, int analogPort, int cancoderCanId,
            double offsetVolts, double offsetRot, double[] pid) {

        double gr = swerveConstants.STEER_GEAR_RATIO;
        SwerveSteerMotor    steer;
        SwerveAbsoluteEncoder encoder;

        if (swerveConstants.ABS_ENCODER_TYPE
                == swerveConstants.AbsoluteEncoderType.REV_THROUGH_BORE) {
            // SparkBase shared between encoder and motor — create once
            validateRevThroughBoreCompatibility();
            boolean isFlex = (swerveConstants.STEER_MOTOR_TYPE
                              == swerveConstants.SteerMotorType.NEO_VORTEX);
            SparkBase spark = isFlex
                ? new SparkFlex(steerCanId, MotorType.kBrushless)
                : new SparkMax(steerCanId, MotorType.kBrushless);
            encoder = new REVThroughBoreEncoder(spark, offsetRot);
            steer   = new REVSteerMotor(spark, isFlex, revSteerCurrentLimit(), pid, gr);
        } else {
            steer   = buildSteerMotor(steerCanId, pid, gr);
            encoder = buildEncoder(analogPort, cancoderCanId, offsetVolts, offsetRot);
        }

        return new Object[]{steer, encoder};
    }

    private static SwerveSteerMotor buildSteerMotor(int canId, double[] pid, double gr) {
        switch (swerveConstants.STEER_MOTOR_TYPE) {
            case KRAKEN_X60:
            case KRAKEN_X44:
            case MINION:
                return new CTRESteerMotor(swerveConstants.STEER_MOTOR_TYPE, canId, pid, gr);
            case NOVA:
                return new NovaSteerMotor(canId, pid, gr);
            case NEO_VORTEX:
            case NEO:
            case NEO_550:
                return new REVSteerMotor(swerveConstants.STEER_MOTOR_TYPE, canId, pid, gr);
            default:
                throw new IllegalArgumentException(
                    "swerveModule: unknown STEER_MOTOR_TYPE " + swerveConstants.STEER_MOTOR_TYPE);
        }
    }

    private static SwerveAbsoluteEncoder buildEncoder(
            int analogPort, int cancoderCanId, double offsetVolts, double offsetRot) {
        switch (swerveConstants.ABS_ENCODER_TYPE) {
            case THRIFTY_ANALOG:
                return new ThriftyAnalogEncoder(analogPort, offsetVolts);
            case CTRE_CANCODER:
                return new CTRECANcoderEncoder(cancoderCanId, offsetRot);
            default:
                throw new IllegalArgumentException(
                    "swerveModule: unknown ABS_ENCODER_TYPE " + swerveConstants.ABS_ENCODER_TYPE);
        }
    }

    private static void validateRevThroughBoreCompatibility() {
        switch (swerveConstants.STEER_MOTOR_TYPE) {
            case NEO_VORTEX:
            case NEO:
            case NEO_550:
                return;
            default:
                throw new IllegalArgumentException(
                    "swerveConstants: ABS_ENCODER_TYPE = REV_THROUGH_BORE requires a REV steer motor "
                    + "(NEO_VORTEX, NEO, or NEO_550). STEER_MOTOR_TYPE = "
                    + swerveConstants.STEER_MOTOR_TYPE + " is not compatible.");
        }
    }

    private static int revSteerCurrentLimit() {
        switch (swerveConstants.STEER_MOTOR_TYPE) {
            case NEO_VORTEX: return 30;
            case NEO:        return 25;
            case NEO_550:    return 20;
            default:         return 25;
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Public API
    // ─────────────────────────────────────────────────────────────────────────

    public void setDesiredState(SwerveModuleState desiredState) {
        applyDashboardPIDUpdates();

        // Drive: convert m/s → wheel rot/s; gear ratio handled inside driveMotor
        double wheelRotPerSec = desiredState.speedMetersPerSecond
            / (Math.PI * swerveConstants.WHEEL_DIAMETER_INCHES * 0.0254);
        driveMotor.setVelocityRotPerSec(wheelRotPerSec);

        // Steer: getRotations() returns [-0.5, 0.5]; wrap handled inside steerMotor
        steerMotor.setPositionRot(desiredState.angle.getRotations());
    }

    public SwerveModuleState getState() {
        double wheelRotPerSec = driveMotor.getVelocityRotPerSec();
        double speedMPS = wheelRotPerSec * Math.PI * swerveConstants.WHEEL_DIAMETER_INCHES * 0.0254;
        return new SwerveModuleState(speedMPS, getSteerAngle());
    }

    public SwerveModulePosition getPosition() {
        double wheelRot  = driveMotor.getPositionRot();
        double distanceM = wheelRot * Math.PI * swerveConstants.WHEEL_DIAMETER_INCHES * 0.0254;
        return new SwerveModulePosition(distanceM, getSteerAngle());
    }

    public void publishTelemetry() {
        SmartDashboard.putNumber("Swerve/" + name + "/Angle Deg",   getSteerAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/" + name + "/Speed MPS",   getState().speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve/" + name + "/Drive Amps",  driveMotor.getSupplyCurrentAmps());
        SmartDashboard.putNumber("Swerve/" + name + "/Steer Amps",  steerMotor.getSupplyCurrentAmps());

        // Publish abs encoder rotations for all encoder types — useful for offset calibration
        SmartDashboard.putNumber("Swerve/" + name + "/Abs Enc Rot",
            absEncoder.getAbsoluteAngleRad() / (2.0 * Math.PI));

        // Thrifty analog: also publish raw voltage and log to DataLog
        if (absEncoder instanceof ThriftyAnalogEncoder) {
            double rawVolts = ((ThriftyAnalogEncoder) absEncoder).getRawVolts();
            SmartDashboard.putNumber("Swerve/" + name + "/Raw Volts", rawVolts);
            RobotLogger.moduleSteerVolts[moduleIndex].append(rawVolts);
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Dashboard PID tuning
    // ─────────────────────────────────────────────────────────────────────────

    private void publishPIDToDashboard() {
        String p = "Swerve/" + name + "/PID/";
        SmartDashboard.putNumber(p + "Drive kP", drive_kP);
        SmartDashboard.putNumber(p + "Drive kI", drive_kI);
        SmartDashboard.putNumber(p + "Drive kD", drive_kD);
        SmartDashboard.putNumber(p + "Drive kS", drive_kS);
        SmartDashboard.putNumber(p + "Drive kV", drive_kV);
        SmartDashboard.putNumber(p + "Drive kA", drive_kA);
        SmartDashboard.putNumber(p + "Steer kP", steer_kP);
        SmartDashboard.putNumber(p + "Steer kI", steer_kI);
        SmartDashboard.putNumber(p + "Steer kD", steer_kD);
        SmartDashboard.putNumber(p + "Steer kS", steer_kS);
        SmartDashboard.putNumber(p + "Steer kV", steer_kV);
    }

    private void applyDashboardPIDUpdates() {
        String pfx = "Swerve/" + name + "/PID/";
        double ndkP = SmartDashboard.getNumber(pfx + "Drive kP", drive_kP);
        double ndkI = SmartDashboard.getNumber(pfx + "Drive kI", drive_kI);
        double ndkD = SmartDashboard.getNumber(pfx + "Drive kD", drive_kD);
        double ndkS = SmartDashboard.getNumber(pfx + "Drive kS", drive_kS);
        double ndkV = SmartDashboard.getNumber(pfx + "Drive kV", drive_kV);
        double ndkA = SmartDashboard.getNumber(pfx + "Drive kA", drive_kA);
        double nskP = SmartDashboard.getNumber(pfx + "Steer kP", steer_kP);
        double nskI = SmartDashboard.getNumber(pfx + "Steer kI", steer_kI);
        double nskD = SmartDashboard.getNumber(pfx + "Steer kD", steer_kD);
        double nskS = SmartDashboard.getNumber(pfx + "Steer kS", steer_kS);
        double nskV = SmartDashboard.getNumber(pfx + "Steer kV", steer_kV);

        boolean driveChanged = ndkP!=drive_kP||ndkI!=drive_kI||ndkD!=drive_kD
                            || ndkS!=drive_kS||ndkV!=drive_kV||ndkA!=drive_kA;
        boolean steerChanged = nskP!=steer_kP||nskI!=steer_kI||nskD!=steer_kD
                            || nskS!=steer_kS||nskV!=steer_kV;

        if (driveChanged) {
            drive_kP=ndkP; drive_kI=ndkI; drive_kD=ndkD;
            drive_kS=ndkS; drive_kV=ndkV; drive_kA=ndkA;
            driveMotor.applyPIDGains(drive_kP, drive_kI, drive_kD, drive_kS, drive_kV, drive_kA);
        }
        if (steerChanged) {
            steer_kP=nskP; steer_kI=nskI; steer_kD=nskD;
            steer_kS=nskS; steer_kV=nskV;
            steerMotor.applyPIDGains(steer_kP, steer_kI, steer_kD, steer_kS, steer_kV);
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // ConfigVerifiable implementation
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public List<ConfigVerifyResult> verifyConfig() {
        List<ConfigVerifyResult> results = new ArrayList<>();
        results.add(driveMotor.verifyConfig(name + " Drive"));
        results.add(steerMotor.verifyConfig(name + " Steer"));
        return results;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Private helpers
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Reads the absolute encoder, converts to mechanism shaft rotations [-0.5, 0.5),
     * and seeds the steer motor's internal encoder. Called once at construction.
     */
    private void seedSteerFromAbsolute() {
        double angleRad           = absEncoder.getAbsoluteAngleRad(); // [0, 2π]
        double mechanismRotations = angleRad / (2.0 * Math.PI);       // [0, 1)
        if (mechanismRotations > 0.5) mechanismRotations -= 1.0;      // → [-0.5, 0.5)
        steerMotor.seedPosition(mechanismRotations);
    }

    private Rotation2d getSteerAngle() {
        return Rotation2d.fromRotations(steerMotor.getPositionRot());
    }
}
