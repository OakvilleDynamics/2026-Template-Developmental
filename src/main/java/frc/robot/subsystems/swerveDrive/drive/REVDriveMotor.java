package frc.robot.subsystems.swerveDrive.drive;

import com.revrobotics.REVLibError;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfigAccessor;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.util.ArrayList;
import java.util.List;

import frc.robot.constants.canIDs;
import frc.robot.constants.swerveConstants;
import frc.robot.util.motors.ConfigVerifyResult;

/**
 * REVDriveMotor.java
 * PATH: src/main/java/frc/robot/subsystems/swerveDrive/drive/REVDriveMotor.java
 *
 * SwerveDriveMotor for REV SparkMax (NEO) and SparkFlex (NEO Vortex).
 *
 * REVLib 2027 removed encoder conversion factors, so the controller works in
 * native units (motor rotations, motor RPM). This class converts at its
 * boundary: setpoints and readings are scaled by the gear ratio, and the
 * velocity gains (which callers give per wheel shaft RPS) are scaled to RPM.
 * Gear ratio is fully encapsulated — callers work in wheel shaft units only.
 *
 * PID live updates use kNoPersistParameters — no flash write during tuning.
 *
 * Current limits by motor type:
 *   NEO Vortex — smart 60 A
 *   NEO        — smart 50 A
 */
public class REVDriveMotor implements SwerveDriveMotor {

    private final SparkBase                 motor;
    private final SparkClosedLoopController controller;
    private final boolean                   isFlex;
    private final boolean                   inverted;
    private final int                       smartCurrentLimitAmps;
    /** Native motor RPM per wheel shaft RPS. */
    private final double                    rpmPerWheelRps;
    private final double                    gearRatio;

    /**
     * @param type      NEO_VORTEX (SparkFlex) or NEO (SparkMax)
     * @param canId     CAN bus ID
     * @param inverted  true if motor output should be inverted
     * @param pid       { kP, kI, kD, kS, kV, kA }
     * @param gearRatio drive motor rotations per wheel rotation
     */
    public REVDriveMotor(
            swerveConstants.DriveMotorType type,
            int canId,
            boolean inverted,
            double[] pid,
            double gearRatio) {

        this.inverted       = inverted;
        this.gearRatio      = gearRatio;
        this.rpmPerWheelRps = gearRatio * 60.0;

        switch (type) {
            case NEO_VORTEX: isFlex = true;  smartCurrentLimitAmps = 60; break;
            case NEO:        isFlex = false; smartCurrentLimitAmps = 50; break;
            default: throw new IllegalArgumentException("REVDriveMotor: unsupported type " + type);
        }

        motor = isFlex
            ? new SparkFlex(canIDs.CAN_PORT, canId, MotorType.kBrushless)
            : new SparkMax(canIDs.CAN_PORT, canId, MotorType.kBrushless);
        controller = motor.getClosedLoopController();

        SparkBaseConfig cfg = isFlex ? new SparkFlexConfig() : new SparkMaxConfig();
        cfg.inverted(inverted);
        cfg.idleMode(IdleMode.kBrake);
        cfg.smartCurrentLimit(smartCurrentLimitAmps);
        cfg.openLoopRampRate(0.1);
        cfg.closedLoopRampRate(0.02);
        // Gains are given per wheel shaft RPS; the controller runs in motor RPM
        cfg.closedLoop
            .pid(pid[0] / rpmPerWheelRps, pid[1] / rpmPerWheelRps, pid[2] / rpmPerWheelRps)
            .apply(new FeedForwardConfig().kV(pid[4] / rpmPerWheelRps));

        motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor.getEncoder().setPosition(0);
    }

    @Override
    public void setVelocityRotPerSec(double wheelRotPerSec) {
        controller.setSetpoint(wheelRotPerSec * rpmPerWheelRps, ControlType.kVelocity,
            ClosedLoopSlot.kSlot0, 0.0, ArbFFUnits.kVoltage);
    }

    @Override
    public double getVelocityRotPerSec() {
        return motor.getEncoder().getVelocity().get() / rpmPerWheelRps;
    }

    @Override
    public double getPositionRot() {
        return motor.getEncoder().getPosition().get() / gearRatio;
    }

    @Override
    public double getSupplyCurrentAmps() {
        return motor.getOutputCurrent().get();
    }

    @Override
    public void applyPIDGains(double kP, double kI, double kD, double kS, double kV, double kA) {
        // kNoPersistParameters — avoid flash write wear during live tuning sessions
        SparkBaseConfig cfg = isFlex ? new SparkFlexConfig() : new SparkMaxConfig();
        cfg.closedLoop
            .pid(kP / rpmPerWheelRps, kI / rpmPerWheelRps, kD / rpmPerWheelRps)
            .apply(new FeedForwardConfig().kV(kV / rpmPerWheelRps));
        motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public ConfigVerifyResult verifyConfig(String label) {
        int    canId  = motor.getDeviceId();
        String vendor = isFlex ? "REV SparkFlex" : "REV SparkMax";

        SparkBaseConfig cfg = isFlex ? new SparkFlexConfig() : new SparkMaxConfig();
        cfg.inverted(inverted);
        cfg.idleMode(IdleMode.kBrake);
        cfg.smartCurrentLimit(smartCurrentLimitAmps);

        boolean applyOk = false;
        for (int i = 0; i < 5; i++) {
            REVLibError err = motor.configure(cfg,
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            if (err == REVLibError.kOk) { applyOk = true; break; }
            try { Thread.sleep(50); } catch (InterruptedException ignored) {}
        }
        if (!applyOk) return new ConfigVerifyResult(label, canId, vendor, false, List.of());

        SparkBaseConfigAccessor accessor = motor instanceof SparkFlex
            ? ((SparkFlex) motor).configAccessor
            : ((SparkMax)  motor).configAccessor;
        List<String> mismatches = new ArrayList<>();

        if (accessor.getInverted() != inverted)
            mismatches.add("inversion: expected " + inverted + " got " + accessor.getInverted());
        if (accessor.getIdleMode() != IdleMode.kBrake)
            mismatches.add("idleMode: expected kBrake got " + accessor.getIdleMode());
        if (Math.abs(accessor.getSmartCurrentLimit() - smartCurrentLimitAmps) > 1)
            mismatches.add("smartCurrentLimit: expected " + smartCurrentLimitAmps
                           + "A got " + accessor.getSmartCurrentLimit() + "A");

        return new ConfigVerifyResult(label, canId, vendor, true, mismatches);
    }
}
