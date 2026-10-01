package frc.robot.subsystems.swerveDrive.steer;

import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.EncoderType;

import java.util.ArrayList;
import java.util.List;

import frc.robot.util.motors.ConfigVerifyResult;

/**
 * NovaSteerMotor.java
 * PATH: src/main/java/frc/robot/subsystems/swerveDrive/steer/NovaSteerMotor.java
 *
 * SwerveSteerMotor for ThriftyBot Nova controller (any output shaft — Minion or Pulsar).
 *
 * Nova has no hardware continuous-wrap equivalent, so shortest-path routing is
 * computed in software each loop in setPositionRot():
 *   1. Read current mechanism shaft position (motor / gearRatio)
 *   2. Compute where the current position sits within one rotation [-0.5, 0.5)
 *   3. Find the delta to the target; clamp to [-0.5, 0.5] to take shortest path
 *   4. Add the clamped delta to the continuous motor position and command that
 *
 * Gear ratio applied manually — Nova has no conversion factor API.
 * Current limit: supply 30 A.
 */
public class NovaSteerMotor implements SwerveSteerMotor {

    private final ThriftyNova motor;
    private final double      gearRatio;
    private final int         canId;
    private static final double SUPPLY_AMPS = 30.0;

    /**
     * @param canId     Nova CAN bus ID
     * @param pid       { kP, kI, kD, kS, kV } — kV mapped to FF on Nova
     * @param gearRatio steer motor rotations per module azimuth rotation
     */
    public NovaSteerMotor(int canId, double[] pid, double gearRatio) {
        this.canId     = canId;
        this.gearRatio = gearRatio;

        motor = new ThriftyNova(canId);
        motor.setInverted(false);
        motor.setBrakeMode(true);
        motor.useEncoderType(EncoderType.INTERNAL);
        motor.pid0.setP(pid[0]).setI(pid[1]).setD(pid[2]).setFF(pid[3]);
        motor.setMaxCurrent(ThriftyNova.CurrentType.SUPPLY, SUPPLY_AMPS);
    }

    @Override
    public void setPositionRot(double mechanismRotations) {
        // Software shortest-path continuous wrap
        double motorPos  = motor.getPosition();
        double mechPos   = motorPos / gearRatio;

        // Where is the current position within one full rotation? → [-0.5, 0.5)
        double mechMod = mechPos - Math.floor(mechPos);
        if (mechMod > 0.5) mechMod -= 1.0;

        // Shortest delta from current wrapped position to target
        double delta = mechanismRotations - mechMod;
        if (delta >  0.5) delta -= 1.0;
        if (delta < -0.5) delta += 1.0;

        // Command continuous motor position (accumulates across turns)
        double targetMech = mechPos + delta;
        motor.setPosition(targetMech * gearRatio, 0.0);
    }

    @Override
    public double getPositionRot() {
        return motor.getPosition() / gearRatio;
    }

    @Override
    public double getSupplyCurrentAmps() {
        return motor.getSupplyCurrent();
    }

    @Override
    public void seedPosition(double mechanismRotations) {
        motor.setEncoderPosition(mechanismRotations * gearRatio);
    }

    @Override
    public void applyPIDGains(double kP, double kI, double kD, double kS, double kV) {
        motor.pid0.setP(kP).setI(kI).setD(kD).setFF(kS);
    }

    @Override
    public ConfigVerifyResult verifyConfig(String label) {
        String vendor = "ThriftyBot Nova";

        motor.clearErrors();
        motor.setInverted(false);
        motor.setBrakeMode(true);
        motor.setMaxCurrent(ThriftyNova.CurrentType.SUPPLY, SUPPLY_AMPS);
        try { Thread.sleep(50); } catch (InterruptedException ignored) {}

        if (!motor.getErrors().isEmpty()) {
            return new ConfigVerifyResult(label, canId, vendor, false, List.of());
        }

        List<String> mismatches = new ArrayList<>();
        if (motor.getInverted())
            mismatches.add("inversion: expected false got true");
        if (!motor.getBrakeMode())
            mismatches.add("brakeMode: expected true got false");

        return new ConfigVerifyResult(label, canId, vendor, true, mismatches);
    }
}
