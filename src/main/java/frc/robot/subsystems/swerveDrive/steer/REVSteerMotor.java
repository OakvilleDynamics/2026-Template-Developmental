package frc.robot.subsystems.swerveDrive.steer;

import com.revrobotics.REVLibError;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
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

import frc.robot.constants.swerveConstants;
import frc.robot.util.motors.ConfigVerifyResult;

/**
 * REVSteerMotor.java
 * PATH: src/main/java/frc/robot/subsystems/swerveDrive/steer/REVSteerMotor.java
 *
 * SwerveSteerMotor for REV SparkMax (NEO, NEO 550) and SparkFlex (NEO Vortex).
 *
 * positionConversionFactor converts native motor rotations → mechanism shaft rotations.
 * positionWrappingEnabled(-0.5, 0.5) makes the controller take the shortest path
 * to any target — hardware continuous wrap, no software routing needed.
 *
 * Two constructors:
 *   public  — type + canId: motor created internally (standard case)
 *   package — SparkBase pre-built: used when REV_THROUGH_BORE encoder is selected,
 *             so the encoder and motor share a single controller instance
 *
 * PID live updates use kNoPersistParameters — no flash write during tuning.
 *
 * Current limits by motor type:
 *   NEO Vortex — smart 30 A
 *   NEO        — smart 25 A
 *   NEO 550    — smart 20 A
 */
public class REVSteerMotor implements SwerveSteerMotor {

    private final SparkBase                 motor;
    private final SparkClosedLoopController controller;
    private final boolean                   isFlex;
    private final int                       smartCurrentLimitAmps;

    /**
     * Standard constructor — creates the SparkBase internally.
     * Use when ABS_ENCODER_TYPE is not REV_THROUGH_BORE.
     */
    public REVSteerMotor(
            swerveConstants.SteerMotorType type,
            int canId,
            double[] pid,
            double gearRatio) {

        this(
            type == swerveConstants.SteerMotorType.NEO_VORTEX
                ? new SparkFlex(canId, MotorType.kBrushless)
                : new SparkMax(canId, MotorType.kBrushless),
            type == swerveConstants.SteerMotorType.NEO_VORTEX,
            currentLimitFor(type),
            pid,
            gearRatio);
    }

    /**
     * Pre-built SparkBase constructor — used when REV_THROUGH_BORE encoder is selected.
     * The swerveModule factory creates the SparkBase first (so the encoder and motor
     * share one controller instance), then passes it here.
     *
     * @param spark                pre-constructed SparkBase (SparkMax or SparkFlex)
     * @param isFlex               true if spark is a SparkFlex (NEO Vortex)
     * @param smartCurrentLimitAmps current limit to apply
     * @param pid                  { kP, kI, kD, kS, kV }
     * @param gearRatio            steer motor rotations per module azimuth rotation
     */
    public REVSteerMotor(SparkBase spark, boolean isFlex, int smartCurrentLimitAmps,
                  double[] pid, double gearRatio) {
        this.motor                 = spark;
        this.isFlex                = isFlex;
        this.smartCurrentLimitAmps = smartCurrentLimitAmps;
        this.controller            = motor.getClosedLoopController();
        configure(pid, gearRatio);
    }

    private void configure(double[] pid, double gearRatio) {
        SparkBaseConfig cfg = isFlex ? new SparkFlexConfig() : new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake);
        cfg.encoder
            .positionConversionFactor(1.0 / gearRatio)
            .velocityConversionFactor(1.0 / (gearRatio * 60.0));
        cfg.smartCurrentLimit(smartCurrentLimitAmps);
        cfg.closedLoop
            .pid(pid[0], pid[1], pid[2])
            .apply(new FeedForwardConfig().kV(pid[4]))
            // Hardware continuous wrap: SparkMAX/SparkFlex takes the shortest path
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(-0.5, 0.5);
        motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setPositionRot(double mechanismRotations) {
        // positionWrappingEnabled handles shortest-path routing automatically
        controller.setSetpoint(mechanismRotations, ControlType.kPosition,
            ClosedLoopSlot.kSlot0, 0.0);
    }

    @Override
    public double getPositionRot() {
        return motor.getEncoder().getPosition();
    }

    @Override
    public double getSupplyCurrentAmps() {
        return motor.getOutputCurrent();
    }

    @Override
    public void seedPosition(double mechanismRotations) {
        motor.getEncoder().setPosition(mechanismRotations);
    }

    @Override
    public void applyPIDGains(double kP, double kI, double kD, double kS, double kV) {
        SparkBaseConfig cfg = isFlex ? new SparkFlexConfig() : new SparkMaxConfig();
        cfg.closedLoop
            .pid(kP, kI, kD)
            .apply(new FeedForwardConfig().kV(kV));
        motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public ConfigVerifyResult verifyConfig(String label) {
        int    canId  = motor.getDeviceId();
        String vendor = isFlex ? "REV SparkFlex" : "REV SparkMax";

        SparkBaseConfig cfg = isFlex ? new SparkFlexConfig() : new SparkMaxConfig();
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

        if (accessor.getIdleMode() != IdleMode.kBrake)
            mismatches.add("idleMode: expected kBrake got " + accessor.getIdleMode());
        if (Math.abs(accessor.getSmartCurrentLimit() - smartCurrentLimitAmps) > 1)
            mismatches.add("smartCurrentLimit: expected " + smartCurrentLimitAmps
                           + "A got " + accessor.getSmartCurrentLimit() + "A");

        return new ConfigVerifyResult(label, canId, vendor, true, mismatches);
    }

    private static int currentLimitFor(swerveConstants.SteerMotorType type) {
        switch (type) {
            case NEO_VORTEX: return 30;
            case NEO:        return 25;
            case NEO_550:    return 20;
            default: throw new IllegalArgumentException("REVSteerMotor: unsupported type " + type);
        }
    }
}
