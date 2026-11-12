package frc.lib.swerve;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.util.Dashboard;

import frc.lib.hardware.encoder.AbsoluteEncoderIO;
import frc.lib.hardware.encoder.ThriftyAnalogEncoder;
import frc.lib.hardware.motor.MotorIO;
import frc.lib.hardware.motor.MotorIOConfig;
import frc.lib.util.ConfigVerifiable;
import frc.lib.util.ConfigVerifyResult;
import frc.lib.util.RobotLogger;
import frc.lib.util.units;

/**
 * swerveModule.java
 * PATH: src/main/java/frc/lib/swerve/swerveModule.java
 *
 * One swerve module — vendor-agnostic. Takes its drive motor, steer motor and
 * absolute encoder as already-built MotorIO / AbsoluteEncoderIO objects; use
 * swerveHardware to build them from a motor/encoder type. This class contains
 * no vendor-specific logic and reads no robot constants.
 *
 * ─── UNITS ────────────────────────────────────────────────────────────────────
 * Drive MotorIO: wheel shaft rotations / rot·s⁻¹ (gear ratio inside MotorIO)
 * Steer MotorIO: module azimuth rotations [-0.5, 0.5], continuous wrap
 * Public API: meters, Rotation2d (WPILib standard)
 *
 * ─── ENCODER SEEDING ─────────────────────────────────────────────────────────
 * The absolute encoder is read once at startup to seed the steer motor's internal
 * high-resolution encoder. Closed-loop steering uses the internal encoder — not
 * the absolute encoder — to avoid noise at 20 ms update rates.
 *
 * ─── PID LIVE TUNING ─────────────────────────────────────────────────────────
 * All PID values are published as dashboard tunables at startup and re-read every
 * loop. When a value changes, applyDashboardPIDUpdates() delegates to the
 * MotorIO — no vendor-specific logic here.
 */
public class swerveModule implements ConfigVerifiable {

    // ── Vendor-agnostic hardware ──────────────────────────────────────────────
    private final MotorIO           driveMotor;
    private final MotorIO           steerMotor;
    private final AbsoluteEncoderIO absEncoder;

    // ── Identity / geometry ───────────────────────────────────────────────────
    private final String name;
    private final int    moduleIndex;  // 0=FL, 1=FR, 2=BL, 3=BR — for RobotLogger indexing
    private final double wheelCircumferenceM;

    // ── Live PID values (read back from dashboard each loop) ──────────────────
    private double drive_kP, drive_kI, drive_kD, drive_kS, drive_kV, drive_kA;
    private double steer_kP, steer_kI, steer_kD, steer_kS, steer_kV;

    // ── Dashboard tunables (under Swerve/<name>/PID/) ─────────────────────────
    private Dashboard.TunableNumber tDrive_kP, tDrive_kI, tDrive_kD, tDrive_kS, tDrive_kV, tDrive_kA;
    private Dashboard.TunableNumber tSteer_kP, tSteer_kI, tSteer_kD, tSteer_kS, tSteer_kV;

    // ─────────────────────────────────────────────────────────────────────────
    // Constructor
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * @param name                Telemetry label: "FL", "FR", "BL", "BR"
     * @param moduleIndex         0=FL, 1=FR, 2=BL, 3=BR
     * @param driveMotor          drive MotorIO (velocity, wheel-shaft units)
     * @param steerMotor          steer MotorIO (position, azimuth units, continuous wrap)
     * @param absEncoder          absolute azimuth encoder, zero = wheel straight forward
     * @param wheelDiameterInches drive wheel diameter (inches)
     */
    public swerveModule(
            String name,
            int moduleIndex,
            MotorIO driveMotor,
            MotorIO steerMotor,
            AbsoluteEncoderIO absEncoder,
            double wheelDiameterInches) {

        this.name                = name;
        this.moduleIndex         = moduleIndex;
        this.driveMotor          = driveMotor;
        this.steerMotor          = steerMotor;
        this.absEncoder          = absEncoder;
        this.wheelCircumferenceM = Math.PI * units.inches_m(wheelDiameterInches);

        MotorIOConfig d = driveMotor.getConfig();
        MotorIOConfig s = steerMotor.getConfig();
        drive_kP = d.kP; drive_kI = d.kI; drive_kD = d.kD;
        drive_kS = d.kS; drive_kV = d.kV; drive_kA = d.kA;
        steer_kP = s.kP; steer_kI = s.kI; steer_kD = s.kD;
        steer_kS = s.kS; steer_kV = s.kV;

        publishPIDToDashboard();

        driveMotor.setEncoderPosition(0.0);
        seedSteerFromAbsolute();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Public API
    // ─────────────────────────────────────────────────────────────────────────

    public void setDesiredState(SwerveModuleState desiredState) {
        applyDashboardPIDUpdates();

        // Drive: convert m/s → wheel rot/s; gear ratio handled inside driveMotor
        driveMotor.setVelocity(desiredState.speedMetersPerSecond / wheelCircumferenceM, 0.0);

        // Steer: getRotations() returns [-0.5, 0.5]; wrap handled inside steerMotor
        steerMotor.setPosition(desiredState.angle.getRotations(), 0.0);
    }

    public SwerveModuleState getState() {
        double speedMPS = driveMotor.getVelocityRps() * wheelCircumferenceM;
        return new SwerveModuleState(speedMPS, getSteerAngle());
    }

    public SwerveModulePosition getPosition() {
        double distanceM = driveMotor.getPositionRot() * wheelCircumferenceM;
        return new SwerveModulePosition(distanceM, getSteerAngle());
    }

    public void publishTelemetry() {
        Dashboard.log("Swerve/" + name + "/Angle Deg",   getSteerAngle().getDegrees());
        Dashboard.log("Swerve/" + name + "/Speed MPS",   getState().speedMetersPerSecond);
        Dashboard.log("Swerve/" + name + "/Drive Amps",  driveMotor.getSupplyCurrentAmps());
        Dashboard.log("Swerve/" + name + "/Steer Amps",  steerMotor.getSupplyCurrentAmps());

        // Publish abs encoder rotations for all encoder types — useful for offset calibration
        Dashboard.log("Swerve/" + name + "/Abs Enc Rot",
            absEncoder.getAbsoluteAngleRad() / (2.0 * Math.PI));

        // Thrifty analog: also publish raw voltage and log to DataLog
        if (absEncoder instanceof ThriftyAnalogEncoder thrifty) {
            double rawVolts = thrifty.getRawVolts();
            Dashboard.log("Swerve/" + name + "/Raw Volts", rawVolts);
            RobotLogger.moduleSteerVolts[moduleIndex].append(rawVolts);
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Dashboard PID tuning
    // ─────────────────────────────────────────────────────────────────────────

    private void publishPIDToDashboard() {
        String p = "Swerve/" + name + "/PID/";
        tDrive_kP = Dashboard.tunable(p + "Drive kP", drive_kP);
        tDrive_kI = Dashboard.tunable(p + "Drive kI", drive_kI);
        tDrive_kD = Dashboard.tunable(p + "Drive kD", drive_kD);
        tDrive_kS = Dashboard.tunable(p + "Drive kS", drive_kS);
        tDrive_kV = Dashboard.tunable(p + "Drive kV", drive_kV);
        tDrive_kA = Dashboard.tunable(p + "Drive kA", drive_kA);
        tSteer_kP = Dashboard.tunable(p + "Steer kP", steer_kP);
        tSteer_kI = Dashboard.tunable(p + "Steer kI", steer_kI);
        tSteer_kD = Dashboard.tunable(p + "Steer kD", steer_kD);
        tSteer_kS = Dashboard.tunable(p + "Steer kS", steer_kS);
        tSteer_kV = Dashboard.tunable(p + "Steer kV", steer_kV);
    }

    private void applyDashboardPIDUpdates() {
        double ndkP = tDrive_kP.get();
        double ndkI = tDrive_kI.get();
        double ndkD = tDrive_kD.get();
        double ndkS = tDrive_kS.get();
        double ndkV = tDrive_kV.get();
        double ndkA = tDrive_kA.get();
        double nskP = tSteer_kP.get();
        double nskI = tSteer_kI.get();
        double nskD = tSteer_kD.get();
        double nskS = tSteer_kS.get();
        double nskV = tSteer_kV.get();

        boolean driveChanged = ndkP!=drive_kP||ndkI!=drive_kI||ndkD!=drive_kD
                            || ndkS!=drive_kS||ndkV!=drive_kV||ndkA!=drive_kA;
        boolean steerChanged = nskP!=steer_kP||nskI!=steer_kI||nskD!=steer_kD
                            || nskS!=steer_kS||nskV!=steer_kV;

        if (driveChanged) {
            drive_kP=ndkP; drive_kI=ndkI; drive_kD=ndkD;
            drive_kS=ndkS; drive_kV=ndkV; drive_kA=ndkA;
            driveMotor.applyGains(drive_kP, drive_kI, drive_kD, drive_kS, drive_kV, drive_kA);
        }
        if (steerChanged) {
            steer_kP=nskP; steer_kI=nskI; steer_kD=nskD;
            steer_kS=nskS; steer_kV=nskV;
            steerMotor.applyGains(steer_kP, steer_kI, steer_kD, steer_kS, steer_kV, 0.0);
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
        double angleRad           = absEncoder.getAbsoluteAngleRad(); // [0, 2π)
        double mechanismRotations = angleRad / (2.0 * Math.PI);       // [0, 1)
        if (mechanismRotations > 0.5) mechanismRotations -= 1.0;      // → [-0.5, 0.5)
        steerMotor.setEncoderPosition(mechanismRotations);
    }

    private Rotation2d getSteerAngle() {
        return Rotation2d.fromRotations(steerMotor.getPositionRot());
    }
}
