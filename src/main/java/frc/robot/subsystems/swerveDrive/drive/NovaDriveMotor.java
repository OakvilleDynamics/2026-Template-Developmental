package frc.robot.subsystems.swerveDrive.drive;

import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.EncoderType;

import java.util.ArrayList;
import java.util.List;

import frc.robot.util.motors.ConfigVerifyResult;

/**
 * NovaDriveMotor.java
 * PATH: src/main/java/frc/robot/subsystems/swerveDrive/drive/NovaDriveMotor.java
 *
 * SwerveDriveMotor for ThriftyBot Nova controller driving a Pulsar motor.
 *
 * Nova has no conversion factor API — gear ratio is applied manually:
 *   setVelocity(): wheelRotPerSec × gearRatio → motor native RPS
 *   getVelocity(): motor native RPS / gearRatio → wheel shaft RPS
 *   getPosition(): motor native rotations / gearRatio → wheel shaft rotations
 *
 * PID live updates write directly to pid0 fields — no CAN frame needed.
 * kA is not separately configurable on Nova; it is ignored.
 *
 * Current limit: supply 60 A.
 */
public class NovaDriveMotor implements SwerveDriveMotor {

    private final ThriftyNova motor;
    private final double      gearRatio;
    private final boolean     inverted;
    private final int         canId;
    private static final double SUPPLY_AMPS = 60.0;

    /**
     * @param canId     Nova CAN bus ID
     * @param inverted  true if motor output should be inverted
     * @param pid       { kP, kI, kD, kS, kV, kA } — kA ignored on Nova
     * @param gearRatio drive motor rotations per wheel rotation
     */
    public NovaDriveMotor(int canId, boolean inverted, double[] pid, double gearRatio) {
        this.canId     = canId;
        this.inverted  = inverted;
        this.gearRatio = gearRatio;

        motor = new ThriftyNova(canId);
        motor.setInverted(inverted);
        motor.setBrakeMode(true);
        motor.useEncoderType(EncoderType.INTERNAL);
        // kS maps to Nova's FF term (closest static FF equivalent)
        motor.pid0.setP(pid[0]).setI(pid[1]).setD(pid[2]).setFF(pid[3]);
        motor.setMaxCurrent(ThriftyNova.CurrentType.SUPPLY, SUPPLY_AMPS);
        motor.setEncoderPosition(0);
    }

    @Override
    public void setVelocityRotPerSec(double wheelRotPerSec) {
        motor.setVelocity(wheelRotPerSec * gearRatio, 0.0);
    }

    @Override
    public double getVelocityRotPerSec() {
        return motor.getVelocity() / gearRatio;
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
    public void applyPIDGains(double kP, double kI, double kD, double kS, double kV, double kA) {
        // kV and kA not separately configurable on Nova; kS covers static FF.
        motor.pid0.setP(kP).setI(kI).setD(kD).setFF(kS);
    }

    @Override
    public ConfigVerifyResult verifyConfig(String label) {
        String vendor = "ThriftyBot Nova";

        motor.clearErrors();
        motor.setInverted(inverted);
        motor.setBrakeMode(true);
        motor.setMaxCurrent(ThriftyNova.CurrentType.SUPPLY, SUPPLY_AMPS);
        try { Thread.sleep(50); } catch (InterruptedException ignored) {}

        if (!motor.getErrors().isEmpty()) {
            return new ConfigVerifyResult(label, canId, vendor, false, List.of());
        }

        List<String> mismatches = new ArrayList<>();
        if (motor.getInverted() != inverted)
            mismatches.add("inversion: expected " + inverted + " got " + motor.getInverted());
        if (!motor.getBrakeMode())
            mismatches.add("brakeMode: expected true got false");

        return new ConfigVerifyResult(label, canId, vendor, true, mismatches);
    }
}
