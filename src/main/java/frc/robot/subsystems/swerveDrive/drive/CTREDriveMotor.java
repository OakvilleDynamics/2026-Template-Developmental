package frc.robot.subsystems.swerveDrive.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.util.ArrayList;
import java.util.List;

import frc.robot.constants.swerveConstants;
import frc.robot.util.motors.ConfigVerifyResult;

/**
 * CTREDriveMotor.java
 * PATH: src/main/java/frc/robot/subsystems/swerveDrive/drive/CTREDriveMotor.java
 *
 * SwerveDriveMotor for CTRE TalonFX (Kraken X60, Kraken X44).
 *
 * SensorToMechanismRatio is set so Phoenix 6 position/velocity signals report
 * in wheel shaft units natively — no gear ratio math in callers.
 *
 * PID live updates push Slot0Configs only (no full config re-apply).
 *
 * Current limits by motor type:
 *   Kraken X60 — supply 60 A, stator 80 A
 *   Kraken X44 — supply 40 A, stator 60 A
 */
public class CTREDriveMotor implements SwerveDriveMotor {

    private final TalonFX motor;
    private final boolean inverted;
    private final double  supplyAmps;
    private final double  statorAmps;

    // Pre-allocated — never re-allocated in a loop
    private final VelocityVoltage velocityRequest =
        new VelocityVoltage(0).withSlot(0).withEnableFOC(true);

    /**
     * @param type      KRAKEN_X60 or KRAKEN_X44
     * @param canId     TalonFX CAN bus ID
     * @param inverted  true if motor output should be inverted
     * @param pid       { kP, kI, kD, kS, kV, kA }
     * @param gearRatio drive motor rotations per wheel rotation
     */
    public CTREDriveMotor(
            swerveConstants.DriveMotorType type,
            int canId,
            boolean inverted,
            double[] pid,
            double gearRatio) {

        this.inverted = inverted;

        switch (type) {
            case KRAKEN_X60: supplyAmps = 60.0; statorAmps = 80.0; break;
            case KRAKEN_X44: supplyAmps = 40.0; statorAmps = 60.0; break;
            default: throw new IllegalArgumentException("CTREDriveMotor: unsupported type " + type);
        }

        motor = new TalonFX(canId);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted    = inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // SensorToMechanismRatio makes all Phoenix 6 signals report in wheel shaft units
        cfg.Feedback.SensorToMechanismRatio             = gearRatio;
        cfg.Slot0.kP = pid[0]; cfg.Slot0.kI = pid[1]; cfg.Slot0.kD = pid[2];
        cfg.Slot0.kS = pid[3]; cfg.Slot0.kV = pid[4]; cfg.Slot0.kA = pid[5];
        cfg.CurrentLimits.SupplyCurrentLimit            = supplyAmps;
        cfg.CurrentLimits.SupplyCurrentLimitEnable      = true;
        cfg.CurrentLimits.StatorCurrentLimit            = statorAmps;
        cfg.CurrentLimits.StatorCurrentLimitEnable      = true;
        cfg.OpenLoopRamps.VoltageOpenLoopRampPeriod     = 0.1;
        cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
        motor.getConfigurator().apply(cfg);
        motor.setPosition(0);

        BaseStatusSignal.setUpdateFrequencyForAll(swerveConstants.SIGNAL_UPDATE_HZ,
            motor.getVelocity(), motor.getPosition(),
            motor.getSupplyCurrent(), motor.getStatorCurrent(),
            motor.getSupplyVoltage(), motor.getMotorVoltage(), motor.getDutyCycle());
        BaseStatusSignal.setUpdateFrequencyForAll(swerveConstants.SIGNAL_UPDATE_HZ_TEMP,
            motor.getDeviceTemp());
        if (swerveConstants.OPTIMIZE_CAN_UTILIZATION) motor.optimizeBusUtilization();
    }

    @Override
    public void setVelocityRotPerSec(double wheelRotPerSec) {
        motor.setControl(velocityRequest.withVelocity(wheelRotPerSec));
    }

    @Override
    public double getVelocityRotPerSec() {
        return motor.getVelocity().getValueAsDouble();
    }

    @Override
    public double getPositionRot() {
        return motor.getPosition().getValueAsDouble();
    }

    @Override
    public double getSupplyCurrentAmps() {
        return motor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void applyPIDGains(double kP, double kI, double kD, double kS, double kV, double kA) {
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = kP; slot0.kI = kI; slot0.kD = kD;
        slot0.kS = kS; slot0.kV = kV; slot0.kA = kA;
        motor.getConfigurator().apply(slot0);
    }

    @Override
    public ConfigVerifyResult verifyConfig(String label) {
        int    canId  = motor.getDeviceID();
        String vendor = "CTRE TalonFX";

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode                = NeutralModeValue.Brake;
        cfg.CurrentLimits.SupplyCurrentLimit       = supplyAmps;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit       = statorAmps;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        boolean applyOk = false;
        for (int i = 0; i < 5; i++) {
            StatusCode sc = motor.getConfigurator().apply(cfg);
            if (sc.isOK()) { applyOk = true; break; }
            try { Thread.sleep(50); } catch (InterruptedException ignored) {}
        }
        if (!applyOk) return new ConfigVerifyResult(label, canId, vendor, false, List.of());

        TalonFXConfiguration rb = new TalonFXConfiguration();
        motor.getConfigurator().refresh(rb);
        List<String> mismatches = new ArrayList<>();

        InvertedValue wantInv = inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        if (rb.MotorOutput.Inverted != wantInv)
            mismatches.add("inversion: expected " + wantInv + " got " + rb.MotorOutput.Inverted);
        if (rb.MotorOutput.NeutralMode != NeutralModeValue.Brake)
            mismatches.add("neutralMode: expected Brake got " + rb.MotorOutput.NeutralMode);
        if (Math.abs(rb.CurrentLimits.SupplyCurrentLimit - supplyAmps) > 0.5)
            mismatches.add("supplyCurrentLimit: expected " + supplyAmps
                           + "A got " + rb.CurrentLimits.SupplyCurrentLimit + "A");
        if (Math.abs(rb.CurrentLimits.StatorCurrentLimit - statorAmps) > 0.5)
            mismatches.add("statorCurrentLimit: expected " + statorAmps
                           + "A got " + rb.CurrentLimits.StatorCurrentLimit + "A");

        return new ConfigVerifyResult(label, canId, vendor, true, mismatches);
    }
}
