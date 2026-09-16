package frc.robot.subsystems.swerveDrive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.constants.swerveConstants;
import frc.robot.util.RobotLogger;
import frc.robot.util.motors.ConfigVerifiable;
import frc.robot.util.motors.ConfigVerifyResult;

/**
 * swerveModule.java
 * PATH: src/main/java/frc/robot/subsystems/swerveDrive/swerveModule.java
 *
 * One Thrifty Narrow swerve module:
 *   Drive  — Kraken X60 (TalonFX, Phoenix 6)
 *   Steer  — Minion (TalonFX, Phoenix 6)
 *   Angle  — Thrifty absolute encoder (0–3.3V analog)
 *
 * PID values come in via constructor — each module can have unique values.
 * All PID values are published to SmartDashboard and re-read every loop,
 * enabling live tuning from Shuffleboard without redeploying.
 *
 * ─── ENCODER SEEDING ─────────────────────────────────────────────────────────
 * Analog encoder is read once at startup to seed the Minion's internal encoder.
 * Closed-loop steering uses the Minion's high-resolution internal encoder
 * rather than the analog signal — avoiding noise at 20ms update rates.
 *
 * ─── DRIVE UNITS (Phoenix 6) ─────────────────────────────────────────────────
 * Native unit: rotations (position) and rotations/second (velocity).
 * Conversion to meters happens in getPosition() and getState().
 */
public class swerveModule implements ConfigVerifiable {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final TalonFX    driveMotor;
    private final TalonFX    steerMotor;
    private final AnalogInput steerEncoder;

    // ── Identity ──────────────────────────────────────────────────────────────
    private final String  name;
    private final int     moduleIndex;  // 0=FL, 1=FR, 2=BL, 3=BR — for RobotLogger array indexing
    private final double  steerOffsetVolts;
    private final boolean driveInverted;

    // ── Live PID values (read back from dashboard each loop) ──────────────────
    private double drive_kP, drive_kI, drive_kD, drive_kS, drive_kV, drive_kA;
    private double steer_kP, steer_kI, steer_kD, steer_kS, steer_kV;

    // ── Control requests (reused — avoids allocation each loop) ──────────────
    private final VelocityVoltage driveVelocityRequest =
        new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
    private final PositionVoltage steerPositionRequest =
        new PositionVoltage(0).withSlot(0).withEnableFOC(true);

    // ─────────────────────────────────────────────────────────────────────────
    // Constructor
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * @param name             Telemetry label: "FL", "FR", "BL", "BR"
     * @param driveCanId       Kraken X60 CAN ID
     * @param steerCanId       Minion CAN ID
     * @param analogPort       roboRIO analog port 0–3 for Thrifty encoder
     * @param steerOffsetVolts Analog voltage when wheel points straight forward
     * @param driveInverted    True if drive motor output should be inverted
     * @param drivePID         double[6] { kP, kI, kD, kS, kV, kA }
     * @param steerPID         double[5] { kP, kI, kD, kS, kV }
     */
    public swerveModule(
            String name,
            int moduleIndex,
            int driveCanId,
            int steerCanId,
            int analogPort,
            double steerOffsetVolts,
            boolean driveInverted,
            double[] drivePID,
            double[] steerPID) {

        this.name             = name;
        this.moduleIndex      = moduleIndex;
        this.steerOffsetVolts = steerOffsetVolts;
        this.driveInverted    = driveInverted;

        drive_kP = drivePID[0]; drive_kI = drivePID[1]; drive_kD = drivePID[2];
        drive_kS = drivePID[3]; drive_kV = drivePID[4]; drive_kA = drivePID[5];
        steer_kP = steerPID[0]; steer_kI = steerPID[1]; steer_kD = steerPID[2];
        steer_kS = steerPID[3]; steer_kV = steerPID[4];

        publishPIDToDashboard();

        steerEncoder = new AnalogInput(analogPort);
        driveMotor   = new TalonFX(driveCanId);
        steerMotor   = new TalonFX(steerCanId);

        configureDriveMotor();
        configureSteerMotor();
        seedSteerEncoder();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Motor configuration
    // ─────────────────────────────────────────────────────────────────────────

    private void configureDriveMotor() {
        driveMotor.getConfigurator().apply(buildDriveConfig());
        driveMotor.setPosition(0);

        // Set signal update rates for logging — high rate for transient capture
        BaseStatusSignal.setUpdateFrequencyForAll(swerveConstants.SIGNAL_UPDATE_HZ,
            driveMotor.getVelocity(),
            driveMotor.getPosition(),
            driveMotor.getSupplyCurrent(),
            driveMotor.getStatorCurrent(),
            driveMotor.getSupplyVoltage(),
            driveMotor.getMotorVoltage(),
            driveMotor.getDutyCycle()
        );
        BaseStatusSignal.setUpdateFrequencyForAll(swerveConstants.SIGNAL_UPDATE_HZ_TEMP,
            driveMotor.getDeviceTemp()
        );
        if (swerveConstants.OPTIMIZE_CAN_UTILIZATION) driveMotor.optimizeBusUtilization();
    }

    private void configureSteerMotor() {
        steerMotor.getConfigurator().apply(buildSteerConfig());

        BaseStatusSignal.setUpdateFrequencyForAll(swerveConstants.SIGNAL_UPDATE_HZ,
            steerMotor.getVelocity(),
            steerMotor.getPosition(),
            steerMotor.getSupplyCurrent(),
            steerMotor.getStatorCurrent(),
            steerMotor.getSupplyVoltage(),
            steerMotor.getMotorVoltage(),
            steerMotor.getDutyCycle()
        );
        BaseStatusSignal.setUpdateFrequencyForAll(swerveConstants.SIGNAL_UPDATE_HZ_TEMP,
            steerMotor.getDeviceTemp()
        );
        if (swerveConstants.OPTIMIZE_CAN_UTILIZATION) steerMotor.optimizeBusUtilization();
    }

    private void seedSteerEncoder() {
        double angleRad       = getAnalogAngleRad();
        double steerRotations = angleRad / (2.0 * Math.PI);
        double motorRotations = steerRotations * swerveConstants.STEER_GEAR_RATIO;
        steerMotor.setPosition(motorRotations);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Public API
    // ─────────────────────────────────────────────────────────────────────────

    public void setDesiredState(SwerveModuleState desiredState) {
        applyDashboardPIDUpdates();

        double wheelRotPerSec = desiredState.speedMetersPerSecond
            / (Math.PI * swerveConstants.WHEEL_DIAMETER_INCHES * 0.0254);
        double motorRotPerSec = wheelRotPerSec * swerveConstants.DRIVE_GEAR_RATIO;
        driveMotor.setControl(driveVelocityRequest.withVelocity(motorRotPerSec));

        double steerRotations = desiredState.angle.getRadians() / (2.0 * Math.PI);
        double motorRotations = steerRotations * swerveConstants.STEER_GEAR_RATIO;
        steerMotor.setControl(steerPositionRequest.withPosition(motorRotations));
    }

    public SwerveModuleState getState() {
        double motorRotPerSec = driveMotor.getVelocity().getValueAsDouble();
        double wheelRotPerSec = motorRotPerSec / swerveConstants.DRIVE_GEAR_RATIO;
        double speedMPS       = wheelRotPerSec * Math.PI * swerveConstants.WHEEL_DIAMETER_INCHES * 0.0254;
        return new SwerveModuleState(speedMPS, getSteerAngle());
    }

    public SwerveModulePosition getPosition() {
        double motorRot  = driveMotor.getPosition().getValueAsDouble();
        double wheelRot  = motorRot / swerveConstants.DRIVE_GEAR_RATIO;
        double distanceM = wheelRot * Math.PI * swerveConstants.WHEEL_DIAMETER_INCHES * 0.0254;
        return new SwerveModulePosition(distanceM, getSteerAngle());
    }

    public void publishTelemetry() {
        double steerVolts = steerEncoder.getVoltage();
        SmartDashboard.putNumber("Swerve/" + name + "/Raw Volts",  steerVolts);
        SmartDashboard.putNumber("Swerve/" + name + "/Angle Deg",  getSteerAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/" + name + "/Speed MPS",  getState().speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve/" + name + "/Drive Amps", driveMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Swerve/" + name + "/Steer Amps", steerMotor.getSupplyCurrent().getValueAsDouble());

        // Analog steer encoder — not on CAN, must be logged manually
        RobotLogger.moduleSteerVolts[moduleIndex].append(steerVolts);
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
            configureDriveMotor();
        }
        if (steerChanged) {
            steer_kP=nskP; steer_kI=nskI; steer_kD=nskD;
            steer_kS=nskS; steer_kV=nskV;
            configureSteerMotor();
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Config builders — shared by configure*() and verifyConfig()
    // ─────────────────────────────────────────────────────────────────────────

    private TalonFXConfiguration buildDriveConfig() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted    = driveInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.Slot0.kP = drive_kP; cfg.Slot0.kI = drive_kI; cfg.Slot0.kD = drive_kD;
        cfg.Slot0.kS = drive_kS; cfg.Slot0.kV = drive_kV; cfg.Slot0.kA = drive_kA;
        cfg.CurrentLimits.SupplyCurrentLimit       = 60;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit       = 80;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.OpenLoopRamps.VoltageOpenLoopRampPeriod     = 0.1;
        cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
        return cfg;
    }

    private TalonFXConfiguration buildSteerConfig() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted    = InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.Slot0.kP = steer_kP; cfg.Slot0.kI = steer_kI; cfg.Slot0.kD = steer_kD;
        cfg.Slot0.kS = steer_kS; cfg.Slot0.kV = steer_kV;
        cfg.CurrentLimits.SupplyCurrentLimit       = 30;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit       = 40;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.ClosedLoopGeneral.ContinuousWrap = true;
        return cfg;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // ConfigVerifiable implementation
    // ─────────────────────────────────────────────────────────────────────────

    private static final int VERIFY_RETRIES  = 5;
    private static final int VERIFY_DELAY_MS = 50;

    @Override
    public List<ConfigVerifyResult> verifyConfig() {
        List<ConfigVerifyResult> results = new ArrayList<>();
        results.add(verifyMotor(name + " Drive", driveMotor, buildDriveConfig(),
            driveInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive,
            NeutralModeValue.Brake, 60.0, 80.0));
        results.add(verifyMotor(name + " Steer", steerMotor, buildSteerConfig(),
            InvertedValue.Clockwise_Positive,
            NeutralModeValue.Brake, 30.0, 40.0));
        return results;
    }

    private static ConfigVerifyResult verifyMotor(
            String label, TalonFX motor, TalonFXConfiguration cfg,
            InvertedValue expectedInv, NeutralModeValue expectedNeutral,
            double expectedSupplyAmps, double expectedStatorAmps) {

        int    canId  = motor.getDeviceID();
        String vendor = "CTRE TalonFX";

        boolean applyOk = false;
        for (int attempt = 0; attempt < VERIFY_RETRIES; attempt++) {
            StatusCode sc = motor.getConfigurator().apply(cfg);
            if (sc.isOK()) { applyOk = true; break; }
            try { Thread.sleep(VERIFY_DELAY_MS); } catch (InterruptedException ignored) {}
        }

        if (!applyOk) {
            return new ConfigVerifyResult(label, canId, vendor, false, List.of());
        }

        TalonFXConfiguration readback = new TalonFXConfiguration();
        motor.getConfigurator().refresh(readback);

        List<String> mismatches = new ArrayList<>();

        if (readback.MotorOutput.Inverted != expectedInv) {
            mismatches.add("inversion: expected " + expectedInv
                           + " got " + readback.MotorOutput.Inverted);
        }
        if (readback.MotorOutput.NeutralMode != expectedNeutral) {
            mismatches.add("neutralMode: expected " + expectedNeutral
                           + " got " + readback.MotorOutput.NeutralMode);
        }
        if (Math.abs(readback.CurrentLimits.SupplyCurrentLimit - expectedSupplyAmps) > 0.5) {
            mismatches.add("supplyCurrentLimit: expected " + expectedSupplyAmps
                           + "A got " + readback.CurrentLimits.SupplyCurrentLimit + "A");
        }
        if (Math.abs(readback.CurrentLimits.StatorCurrentLimit - expectedStatorAmps) > 0.5) {
            mismatches.add("statorCurrentLimit: expected " + expectedStatorAmps
                           + "A got " + readback.CurrentLimits.StatorCurrentLimit + "A");
        }

        return new ConfigVerifyResult(label, canId, vendor, true, mismatches);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Private helpers
    // ─────────────────────────────────────────────────────────────────────────

    private double getAnalogAngleRad() {
        double rawVolts        = steerEncoder.getVoltage();
        double offsetVolts     = rawVolts - steerOffsetVolts;
        double normalizedVolts = offsetVolts % swerveConstants.ANALOG_FULL_SCALE_VOLTS;
        if (normalizedVolts < 0) normalizedVolts += swerveConstants.ANALOG_FULL_SCALE_VOLTS;
        return (normalizedVolts / swerveConstants.ANALOG_FULL_SCALE_VOLTS) * 2.0 * Math.PI;
    }

    private Rotation2d getSteerAngle() {
        double motorRotations = steerMotor.getPosition().getValueAsDouble();
        double steerRotations = motorRotations / swerveConstants.STEER_GEAR_RATIO;
        return Rotation2d.fromRotations(steerRotations);
    }
}