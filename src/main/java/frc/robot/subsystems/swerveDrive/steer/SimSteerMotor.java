package frc.robot.subsystems.swerveDrive.steer;

import java.util.List;

import frc.robot.util.motors.ConfigVerifyResult;

/**
 * SimSteerMotor.java
 * PATH: src/main/java/frc/robot/subsystems/swerveDrive/steer/SimSteerMotor.java
 *
 * Simulation-only SwerveSteerMotor with no hardware behind it. The module
 * azimuth reaches the commanded angle instantly, taking the shortest path.
 *
 * Used by swerveModule in simulation when the configured steer motor has no
 * library for this WPILib year (currently ThriftyBot Nova), so the rest of the
 * robot code can still run in the simulator. Never constructed on a real robot.
 */
public class SimSteerMotor implements SwerveSteerMotor {

    private double positionRot = 0.0;

    @Override
    public void setPositionRot(double mechanismRotations) {
        // Shortest path: move by the wrapped error, in [-0.5, 0.5] rotations
        double error = mechanismRotations - positionRot;
        error -= Math.rint(error);
        positionRot += error;
    }

    @Override
    public double getPositionRot() {
        return positionRot;
    }

    @Override
    public double getSupplyCurrentAmps() {
        return 0.0;
    }

    @Override
    public void seedPosition(double mechanismRotations) {
        positionRot = mechanismRotations;
    }

    @Override
    public void applyPIDGains(double kP, double kI, double kD, double kS, double kV) {}

    @Override
    public ConfigVerifyResult verifyConfig(String label) {
        return new ConfigVerifyResult(label, -1, "Simulated", true, List.of());
    }
}
