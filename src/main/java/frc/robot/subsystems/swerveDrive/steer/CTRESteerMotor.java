package frc.robot.subsystems.swerveDrive.steer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.util.ArrayList;
import java.util.List;

import frc.robot.constants.swerveConstants;
import frc.robot.util.motors.ConfigVerifyResult;

/**
 * CTRESteerMotor.java
 * PATH: src/main/java/frc/robot/subsystems/swerveDrive/steer/CTRESteerMotor.java
 *
 * SwerveSteerMotor for CTRE TalonFX (Kraken X60, Kraken X44, Minion).
 *
 * SensorToMechanismRatio is set so all Phoenix 6 position signals report
 * in mechanism shaft rotations natively. ContinuousWrap = true lets Phoenix 6
 * take the shortest path to any target angle — no software wrap logic needed.
 *
 * Current limits by motor type:
 *   Kraken X60 — supply 40 A, stator 60 A
 *   Kraken X44 — supply 30 A, stator 40 A
 *   Minion     — supply 30 A, stator 40 A
 */
public class CTRESteerMotor implements SwerveSteerMotor {

    private final TalonFX motor;
    private final double  supplyAmps;
    private final double  statorAmps;

    // Pre-allocated — never re-allocated in a loop
    private final PositionVoltage positionRequest =
        new PositionVoltage(0).withSlot(0).withEnableFOC(true);

    /**
     * @param type      KRAKEN_X60, KRAKEN_X44, or MINION
     * @param canId     TalonFX CAN bus ID
     * @param pid       { kP, kI, kD, kS, kV } — kA unused for position control
     * @param gearRatio steer motor rotations per module azimuth rotation
     */
    public CTRESteerMotor(
            swerveConstants.SteerMotorType type,
            int canId,
            double[] pid,
            double gearRatio) {

        switch (type) {
            case KRAKEN_X60: supplyAmps = 40.0; statorAmps = 60.0; break;
            case KRAKEN_X44: supplyAmps = 30.0; statorAmps = 40.0; break;
            case MINION:     supplyAmps = 30.0; statorAmps = 40.0; break;
            default: throw new IllegalArgumentException("CTRESteerMotor: unsupported type " + type);
        }

        motor = new TalonFX(canId);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted    = InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // SensorToMechanismRatio: all position signals report in mechanism shaft rotations
        cfg.Feedback.SensorToMechanismRatio          = gearRatio;
        cfg.Slot0.kP = pid[0]; cfg.Slot0.kI = pid[1]; cfg.Slot0.kD = pid[2];
        cfg.Slot0.kS = pid[3]; cfg.Slot0.kV = pid[4];
        cfg.CurrentLimits.SupplyCurrentLimit         = supplyAmps;
        cfg.CurrentLimits.SupplyCurrentLimitEnable   = true;
        cfg.CurrentLimits.StatorCurrentLimit         = statorAmps;
        cfg.CurrentLimits.StatorCurrentLimitEnable   = true;
        // ContinuousWrap: Phoenix 6 takes the shortest path automatically
        cfg.ClosedLoopGeneral.ContinuousWrap         = true;
        motor.getConfigurator().apply(cfg);

        BaseStatusSignal.setUpdateFrequencyForAll(swerveConstants.SIGNAL_UPDATE_HZ,
            motor.getVelocity(), motor.getPosition(),
            motor.getSupplyCurrent(), motor.getStatorCurrent(),
            motor.getSupplyVoltage(), motor.getMotorVoltage(), motor.getDutyCycle());
        BaseStatusSignal.setUpdateFrequencyForAll(swerveConstants.SIGNAL_UPDATE_HZ_TEMP,
            motor.getDeviceTemp());
        if (swerveConstants.OPTIMIZE_CAN_UTILIZATION) motor.optimizeBusUtilization();
    }

    @Override
    public void setPositionRot(double mechanismRotations) {
        // ContinuousWrap handles shortest-path routing automatically
        motor.setControl(positionRequest.withPosition(mechanismRotations));
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
    public void seedPosition(double mechanismRotations) {
        // With SensorToMechanismRatio set, setPosition() takes mechanism shaft rotations
        motor.setPosition(mechanismRotations);
    }

    @Override
    public void applyPIDGains(double kP, double kI, double kD, double kS, double kV) {
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = kP; slot0.kI = kI; slot0.kD = kD;
        slot0.kS = kS; slot0.kV = kV;
        motor.getConfigurator().apply(slot0);
    }

    @Override
    public ConfigVerifyResult verifyConfig(String label) {
        int    canId  = motor.getDeviceID();
        String vendor = "CTRE TalonFX";

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted               = InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.NeutralMode            = NeutralModeValue.Brake;
        cfg.CurrentLimits.SupplyCurrentLimit   = supplyAmps;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit   = statorAmps;
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

        if (rb.MotorOutput.Inverted != InvertedValue.Clockwise_Positive)
            mismatches.add("inversion: expected Clockwise_Positive got " + rb.MotorOutput.Inverted);
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
