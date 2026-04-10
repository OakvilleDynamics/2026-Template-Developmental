package frc.robot.subsystems.swerveDrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.system.plant.DCMotor;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.pathplannerConstants;
import frc.robot.constants.swerveConstants;
import frc.robot.constants.visionConstants;
import frc.robot.util.RobotLogger;
import frc.robot.util.units;

/**
 * swerveDrive.java
 * PATH: src/main/java/frc/robot/subsystems/swerveDrive/swerveDrive.java
 *
 * Owns the four swerve modules and IMU. The only way to move the robot
 * is through drive(), drivePointAt(), lockWheelsX(), or stop().
 *
 * ─── CONSTRUCTOR GEOMETRY INPUTS (all in INCHES) ─────────────────────────────
 * wheelBaseInches    double[2] { x, y } — wheel steering axis center-to-center
 * frameInches        double[2] { x, y } — outer frame, no bumpers
 * bumperInches       double[2] { x, y } — outer dimensions with bumpers
 * wheelDiamInches    double    — drive wheel diameter
 * headingPID         double[3] { kP, kI, kD } — point-at-target heading controller
 *
 * All values converted to meters internally on entry.
 *
 * ─── GEOMETRY QUERY ──────────────────────────────────────────────────────────
 * getDimension(String key) — "wheel-base", "frame-perimeter", "bumper-perimeter"
 * Returns dimensionResult with .inches[] and .meters[] fields.
 *
 * ─── ODOMETRY STATE ──────────────────────────────────────────────────────────
 * getOdometryState() — returns driveOdometryState updated every loop.
 * Three buckets: encoderState, imuState, blendedState.
 *
 * ─── POINT-AT-TARGET ─────────────────────────────────────────────────────────
 * drivePointAt() — driver controls translation, heading PID controls omega.
 * Target in feet, offset in degrees. futureStatePoseM = null uses odometry.
 *
 * ─── X-LOCK ──────────────────────────────────────────────────────────────────
 * lockWheelsX() — sets all four modules to X brace pattern, zero drive speed.
 */
public class swerveDrive extends SubsystemBase {

    // ─────────────────────────────────────────────────────────────────────────
    // Geometry query result — carries both unit systems
    // ─────────────────────────────────────────────────────────────────────────

    public static class dimensionResult {
        public final double[] inches;
        public final double[] meters;
        public dimensionResult(double xMeters, double yMeters) {
            this.meters = new double[]{ xMeters, yMeters };
            this.inches = new double[]{ units.m_inches(xMeters), units.m_inches(yMeters) };
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Geometry (stored in meters)
    // ─────────────────────────────────────────────────────────────────────────
    private final double[] wheelBaseM;
    private final double[] framePerimeterM;
    private final double[] bumperPerimeterM;

    // ─────────────────────────────────────────────────────────────────────────
    // Hardware
    // ─────────────────────────────────────────────────────────────────────────
    private final swerveModule[] modules;
    private final Pigeon2 imu = new Pigeon2(swerveConstants.PIGEON2_CAN_ID);

    // ─────────────────────────────────────────────────────────────────────────
    // Kinematics and pose estimation
    // ─────────────────────────────────────────────────────────────────────────
    private final SwerveDriveKinematics    kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;

    // ─────────────────────────────────────────────────────────────────────────
    // Heading-lock PID
    // ─────────────────────────────────────────────────────────────────────────
    private final PIDController headingPID;
    private boolean headingLockActive = false;

    // ─────────────────────────────────────────────────────────────────────────
    // Odometry state (updated every loop, readable anytime)
    // ─────────────────────────────────────────────────────────────────────────
    private volatile driveOdometryState currentState = driveOdometryState.zero();

    // Derivative tracking
    private double prevLinVelMag = 0, prevLinVelX = 0, prevLinVelY = 0;
    private double prevAngVel    = 0, prevImuAngVel = 0;
    private double prevCorX = 0, prevCorY = 0;
    private double prevCorVelX = 0, prevCorVelY = 0;
    private double prevTimestamp = Timer.getFPGATimestamp();
    private Translation2d commandedCoR = new Translation2d();

    // Blend weight (encoder vs IMU)
    private double blendAlpha = swerveConstants.ODOMETRY_BLEND_ALPHA;

    // Last commanded speeds — captured in commandModules() for logging
    private ChassisSpeeds lastCommandedSpeeds = new ChassisSpeeds();

    // ─────────────────────────────────────────────────────────────────────────
    // Pose history ring buffer — for game piece vision latency compensation
    // ─────────────────────────────────────────────────────────────────────────
    // At 50 Hz, 4 entries covers ~80 ms — enough for typical PhotonVision
    // ML detection pipeline latency (20–60 ms).
    private static final int POSE_HISTORY_SIZE     = 4;
    private final Pose2d[]   poseHistory           = new Pose2d[POSE_HISTORY_SIZE];
    private final double[]   poseHistoryTimestamps = new double[POSE_HISTORY_SIZE];
    private int              poseHistoryHead       = 0;

    // ─────────────────────────────────────────────────────────────────────────
    // Constructor
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * @param modules          [FL, FR, BL, BR] — constructed in RobotContainer
     * @param wheelBaseInches  double[2] { x, y } — steering axis spacing in INCHES
     * @param frameInches      double[2] { x, y } — frame outer dimensions in INCHES
     * @param bumperInches     double[2] { x, y } — bumper outer dimensions in INCHES
     * @param wheelDiamInches  Drive wheel diameter in INCHES
     * @param headingPID       double[3] { kP, kI, kD } — heading lock controller
     */
    public swerveDrive(
            swerveModule[] modules,
            double[] wheelBaseInches,
            double[] frameInches,
            double[] bumperInches,
            double wheelDiamInches,
            double[] headingPID) {

        this.modules = modules;

        // Convert all geometry to meters on entry
        this.wheelBaseM       = new double[]{ units.inches_m(wheelBaseInches[0]),
                                              units.inches_m(wheelBaseInches[1]) };
        this.framePerimeterM  = new double[]{ units.inches_m(frameInches[0]),
                                              units.inches_m(frameInches[1]) };
        this.bumperPerimeterM = new double[]{ units.inches_m(bumperInches[0]),
                                              units.inches_m(bumperInches[1]) };

        // Module positions derived from wheel base (center = 0,0)
        Translation2d flPos = new Translation2d( wheelBaseM[0] / 2.0,  wheelBaseM[1] / 2.0);
        Translation2d frPos = new Translation2d( wheelBaseM[0] / 2.0, -wheelBaseM[1] / 2.0);
        Translation2d blPos = new Translation2d(-wheelBaseM[0] / 2.0,  wheelBaseM[1] / 2.0);
        Translation2d brPos = new Translation2d(-wheelBaseM[0] / 2.0, -wheelBaseM[1] / 2.0);

        kinematics = new SwerveDriveKinematics(flPos, frPos, blPos, brPos);
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            getYaw(),
            getModulePositions(),
            new Pose2d(),
            MatBuilder.fill(Nat.N3(), Nat.N1(),
                swerveConstants.ODOMETRY_STD_DEV_X,
                swerveConstants.ODOMETRY_STD_DEV_Y,
                swerveConstants.ODOMETRY_STD_DEV_THETA),
            MatBuilder.fill(Nat.N3(), Nat.N1(),
                visionConstants.VISION_STD_DEV_X,
                visionConstants.VISION_STD_DEV_Y,
                visionConstants.VISION_STD_DEV_THETA)
        );

        // Heading PID — continuous input handles 0/2π wrap cleanly
        this.headingPID = new PIDController(headingPID[0], headingPID[1], headingPID[2]);
        this.headingPID.enableContinuousInput(-Math.PI, Math.PI);
        this.headingPID.setTolerance(units.deg_rad(1.0));

        SmartDashboard.putNumber("Drive/HeadingPID/kP", headingPID[0]);
        SmartDashboard.putNumber("Drive/HeadingPID/kI", headingPID[1]);
        SmartDashboard.putNumber("Drive/HeadingPID/kD", headingPID[2]);

        // Pigeon 2 self-calibrates on power-up. Temperature calibration is performed
        // once in Phoenix Tuner X — no runtime calibrate() call needed.
        publishGeometry();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // periodic — runs every 20ms
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        poseEstimator.update(getYaw(), getModulePositions());

        // Write latest estimated pose into the ring buffer for getPoseAtTime()
        double now = Timer.getFPGATimestamp();
        poseHistory[poseHistoryHead]           = poseEstimator.getEstimatedPosition();
        poseHistoryTimestamps[poseHistoryHead] = now;
        poseHistoryHead = (poseHistoryHead + 1) % POSE_HISTORY_SIZE;
        double dt  = Math.max(now - prevTimestamp, 1e-4);
        prevTimestamp = now;

        currentState = computeOdometryState(dt);

        // Live-tune heading PID from dashboard
        double kP = SmartDashboard.getNumber("Drive/HeadingPID/kP", headingPID.getP());
        double kI = SmartDashboard.getNumber("Drive/HeadingPID/kI", headingPID.getI());
        double kD = SmartDashboard.getNumber("Drive/HeadingPID/kD", headingPID.getD());
        if (kP != headingPID.getP() || kI != headingPID.getI() || kD != headingPID.getD()) {
            headingPID.setPID(kP, kI, kD);
        }

        blendAlpha = SmartDashboard.getNumber("Drive/Blend Alpha", blendAlpha);

        SmartDashboard.putBoolean("Drive/HeadingLock Active", headingLockActive);
        headingLockActive = false; // reset; drivePointAt() sets it if called this loop

        publishTelemetry();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Drive API
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Normal driver-controlled drive.
     * Accepts a driveInput built with ft/s values by driveWithJoysticks.
     */
    public void drive(driveInput input) {
        headingLockActive = false;
        commandedCoR = input.getCenterOfRotation();
        ChassisSpeeds fieldRelative = ChassisSpeeds.fromFieldRelativeSpeeds(
            input.getVxMps(), input.getVyMps(), input.getOmegaRadps(), getYaw()
        );
        commandModules(fieldRelative, input.getCenterOfRotation());
    }

    /**
     * Point-at-target drive.
     * Driver controls translation; heading PID controls omega.
     * The omega field of driveInput is ignored.
     *
     * @param input           Driver translation (vx/vy used; omega ignored)
     * @param targetFeetX     Field target X in FEET
     * @param targetFeetY     Field target Y in FEET
     * @param offsetDegrees   Heading offset in DEGREES (0 = robot front faces target)
     */
    public void drivePointAt(driveInput input, double targetFeetX, double targetFeetY,
                              double offsetDegrees) {
        drivePointAt(input, targetFeetX, targetFeetY, offsetDegrees, null);
    }

    /**
     * Point-at-target drive with optional future-state pose for feedforward.
     *
     * @param futureStatePoseM  Expected robot pose at shot time (meters).
     *                          Pass null to use current odometry pose.
     *                          Wire in PhotonVision pose here when ready.
     */
    public void drivePointAt(driveInput input, double targetFeetX, double targetFeetY,
                              double offsetDegrees, Pose2d futureStatePoseM) {
        headingLockActive = true;
        commandedCoR = input.getCenterOfRotation();

        double targetXm = units.fieldFeet_m(targetFeetX);
        double targetYm = units.fieldFeet_m(targetFeetY);

        Pose2d referencePose = (futureStatePoseM != null) ? futureStatePoseM : getPose();

        double dx = targetXm - referencePose.getX();
        double dy = targetYm - referencePose.getY();
        double desiredHeadingRad = Math.atan2(dy, dx) + units.deg_rad(offsetDegrees);

        double omegaCorrection = headingPID.calculate(getYaw().getRadians(), desiredHeadingRad);
        omegaCorrection = MathUtil.clamp(omegaCorrection,
            -swerveConstants.HEADING_PID_MAX_OMEGA,
             swerveConstants.HEADING_PID_MAX_OMEGA);

        ChassisSpeeds fieldRelative = ChassisSpeeds.fromFieldRelativeSpeeds(
            input.getVxMps(), input.getVyMps(), omegaCorrection, getYaw()
        );
        commandModules(fieldRelative, input.getCenterOfRotation());

        SmartDashboard.putNumber("Drive/PointAt/Target X (ft)",     targetFeetX);
        SmartDashboard.putNumber("Drive/PointAt/Target Y (ft)",     targetFeetY);
        SmartDashboard.putNumber("Drive/PointAt/Desired Hdg (deg)", units.rad_deg(desiredHeadingRad));
        SmartDashboard.putNumber("Drive/PointAt/Hdg Error (deg)",   units.rad_deg(headingPID.getPositionError()));
        SmartDashboard.putBoolean("Drive/PointAt/At Heading",       headingPID.atSetpoint());
    }

    /**
     * Heading-bounded drive.
     *
     * Driver retains full translation and rotation control while the robot heading
     * stays within [minAbsHeadingDeg, maxAbsHeadingDeg] (field frame, degrees).
     *
     * Behavior:
     *   Heading within bounds  → passes driver omega through unchanged (full control)
     *   Heading below minDeg   → PID-holds to minAbsHeadingDeg (overrides driver omega)
     *   Heading above maxDeg   → PID-holds to maxAbsHeadingDeg (overrides driver omega)
     *
     * Reuses the same headingPID and HEADING_PID_MAX_OMEGA as drivePointAt().
     * Bounds come from ShooterOutput.headingBoundsDeg — the range of robot headings
     * within which the turret can reach the target without exceeding its soft stops.
     *
     * @param input              Driver translation + omega + CoR.
     * @param minAbsHeadingDeg   Lower heading bound (degrees, field frame).
     * @param maxAbsHeadingDeg   Upper heading bound (degrees, field frame).
     */
    public void driveWithHeadingBound(
            driveInput input,
            double minAbsHeadingDeg,
            double maxAbsHeadingDeg) {

        double currentHeadingDeg = units.rad_deg(getYaw().getRadians());

        if (currentHeadingDeg < minAbsHeadingDeg || currentHeadingDeg > maxAbsHeadingDeg) {
            // Outside bounds — PID-hold to the nearer bound
            headingLockActive = true;
            commandedCoR = input.getCenterOfRotation();

            double clampedDeg = currentHeadingDeg < minAbsHeadingDeg
                    ? minAbsHeadingDeg
                    : maxAbsHeadingDeg;
            double desiredRad = units.deg_rad(clampedDeg);

            double omegaCorrection = headingPID.calculate(getYaw().getRadians(), desiredRad);
            omegaCorrection = MathUtil.clamp(omegaCorrection,
                    -swerveConstants.HEADING_PID_MAX_OMEGA,
                     swerveConstants.HEADING_PID_MAX_OMEGA);

            ChassisSpeeds fieldRelative = ChassisSpeeds.fromFieldRelativeSpeeds(
                    input.getVxMps(), input.getVyMps(), omegaCorrection, getYaw());
            commandModules(fieldRelative, input.getCenterOfRotation());

            SmartDashboard.putBoolean("Drive/HeadingBound/InBounds", false);
        } else {
            // Within bounds — pass driver omega through unchanged
            headingLockActive = false;
            drive(input);

            SmartDashboard.putBoolean("Drive/HeadingBound/InBounds", true);
        }

        SmartDashboard.putNumber("Drive/HeadingBound/Min (deg)", minAbsHeadingDeg);
        SmartDashboard.putNumber("Drive/HeadingBound/Max (deg)", maxAbsHeadingDeg);
        SmartDashboard.putNumber("Drive/HeadingBound/Current (deg)", currentHeadingDeg);
    }

    /**
     * X-lock defense — sets all four modules to 45° X brace pattern.
     * No drive speed. Robot resists being pushed from any direction.
     * Called by xLockCommand every loop while defense button is held.
     *
     * Wheel angles (robot-relative, 0° = robot front):
     *   FL:  45°    FR: -45°
     *       \           /
     *        X         X
     *       /           \
     *   BL: -45°    BR:  45°
     */
    public void lockWheelsX() {
        modules[0].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees( 45.0))); // FL
        modules[1].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0))); // FR
        modules[2].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0))); // BL
        modules[3].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees( 45.0))); // BR
    }

    /** Stops all modules — holds current steer angle, sets drive speed to zero */
    public void stop() {
        drive(driveInput.stopped());
    }

    /**
     * Commands the drivetrain using robot-relative ChassisSpeeds.
     * Called exclusively by PathPlanner via the AutoBuilder lambda — not by driver commands.
     * Unlike drive(), no field-relative yaw rotation is applied; speeds go straight to modules.
     *
     * Note: this is an intentional exception to the driveInput-only command pattern.
     * PathPlanner outputs robot-relative speeds directly; routing through driveInput would
     * require a coordinate-frame roundtrip for no benefit. See CLAUDE.md.
     *
     * @param speeds Robot-relative ChassisSpeeds from PathPlanner
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        commandModules(speeds, new Translation2d());
    }

    /**
     * Returns current robot-relative ChassisSpeeds derived from wheel encoder velocities.
     * Called exclusively by PathPlanner via the AutoBuilder lambda for its feedback loop.
     *
     * @return Robot-relative vx (m/s), vy (m/s), omega (rad/s)
     */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Configures PathPlanner's AutoBuilder using this subsystem's own geometry and state.
     * Call once from RobotContainer after swerveDrive is constructed, before any path commands.
     *
     * Module positions are recomputed from wheelBaseM — same math as the constructor.
     * All PathPlanner dependencies are contained within swerveDrive; RobotContainer
     * remains unaware of PathPlanner internals.
     */
    public void configureForAutoBuilder() {
        Translation2d flPos = new Translation2d( wheelBaseM[0] / 2.0,  wheelBaseM[1] / 2.0);
        Translation2d frPos = new Translation2d( wheelBaseM[0] / 2.0, -wheelBaseM[1] / 2.0);
        Translation2d blPos = new Translation2d(-wheelBaseM[0] / 2.0,  wheelBaseM[1] / 2.0);
        Translation2d brPos = new Translation2d(-wheelBaseM[0] / 2.0, -wheelBaseM[1] / 2.0);

        ModuleConfig moduleConfig = new ModuleConfig(
            units.inches_m(swerveConstants.WHEEL_DIAMETER_INCHES / 2.0),
            pathplannerConstants.MAX_PATH_VEL_MPS,
            pathplannerConstants.WHEEL_COF,
            DCMotor.getKrakenX60(1),
            pathplannerConstants.DRIVE_CURRENT_LIMIT_AMPS,
            1
        );

        RobotConfig robotConfig = new RobotConfig(
            pathplannerConstants.ROBOT_MASS_KG,
            pathplannerConstants.ROBOT_MOI_KGM2,
            moduleConfig,
            flPos, frPos, blPos, brPos
        );

        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getRobotRelativeSpeeds,
            this::driveRobotRelative,
            new PPHolonomicDriveController(
                new PIDConstants(pathplannerConstants.TRANSLATION_PID_kP,
                                 pathplannerConstants.TRANSLATION_PID_kI,
                                 pathplannerConstants.TRANSLATION_PID_kD),
                new PIDConstants(pathplannerConstants.ROTATION_PID_kP,
                                 pathplannerConstants.ROTATION_PID_kI,
                                 pathplannerConstants.ROTATION_PID_kD)
            ),
            robotConfig,
            () -> DriverStation.getAlliance()
                    .filter(a -> a == DriverStation.Alliance.Red)
                    .isPresent(),
            this
        );
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Geometry query
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * @param key  "wheel-base" | "frame-perimeter" | "bumper-perimeter"
     * @return dimensionResult with .inches[]{x,y} and .meters[]{x,y}, or null.
     */
    public dimensionResult getDimension(String key) {
        switch (key) {
            case "wheel-base":       return new dimensionResult(wheelBaseM[0],       wheelBaseM[1]);
            case "frame-perimeter":  return new dimensionResult(framePerimeterM[0],  framePerimeterM[1]);
            case "bumper-perimeter": return new dimensionResult(bumperPerimeterM[0], bumperPerimeterM[1]);
            default:
                System.err.println("swerveDrive.getDimension: unknown key '" + key + "'");
                return null;
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Odometry and pose
    // ─────────────────────────────────────────────────────────────────────────

    /** Returns the full odometry state from last loop. Safe to call from anywhere. */
    public driveOdometryState getOdometryState() { return currentState; }

    public Pose2d    getPose()              { return poseEstimator.getEstimatedPosition(); }
    public Rotation2d getYaw() {
        // Pigeon 2 reports yaw in degrees, CCW positive (NWU).
        // Negated to match WPILib field convention — verify sign against physical mounting.
        return Rotation2d.fromDegrees(-imu.getYaw().getValueAsDouble());
    }

    /**
     * Returns the estimated robot pose at or nearest to the given FPGA timestamp.
     *
     * Linearly interpolates between the two ring-buffer entries that bracket the
     * requested timestamp. Falls back to the chronologically nearest stored entry
     * if the timestamp is outside the buffered window (~80 ms at 50 Hz).
     *
     * Used by gamePieceVisionSubsystem to back-project game piece detections to
     * field coordinates at the exact moment the camera frame was captured.
     *
     * @param timestampSecs  FPGA timestamp of the camera frame (seconds)
     * @return  Robot Pose2d at that time, or nearest available if out of window
     */
    public Pose2d getPoseAtTime(double timestampSecs) {
        // Find how many entries are populated (null check)
        int filled = 0;
        for (Pose2d p : poseHistory) { if (p != null) filled++; }
        if (filled == 0) return getPose();   // buffer not yet populated

        // Collect valid entries in chronological order
        // Ring buffer reads: start from the oldest entry = current head (next write slot)
        Pose2d  bestPose  = null;
        double  bestDelta = Double.MAX_VALUE;
        Pose2d  prevPose  = null;
        double  prevTs    = 0;

        for (int i = 0; i < POSE_HISTORY_SIZE; i++) {
            int idx = (poseHistoryHead + i) % POSE_HISTORY_SIZE;
            Pose2d p = poseHistory[idx];
            double t = poseHistoryTimestamps[idx];
            if (p == null) { prevPose = null; continue; }

            // Check if this entry and the previous bracket the requested timestamp
            if (prevPose != null && prevTs <= timestampSecs && t >= timestampSecs) {
                double span = t - prevTs;
                double frac = (span > 1e-6) ? (timestampSecs - prevTs) / span : 0.0;
                double x = prevPose.getX()                 + frac * (p.getX()                 - prevPose.getX());
                double y = prevPose.getY()                 + frac * (p.getY()                 - prevPose.getY());
                double h = prevPose.getRotation().getRadians()
                           + frac * (p.getRotation().getRadians() - prevPose.getRotation().getRadians());
                return new Pose2d(x, y, new Rotation2d(h));
            }

            // Track nearest entry in case we need to fall back
            double delta = Math.abs(t - timestampSecs);
            if (delta < bestDelta) { bestDelta = delta; bestPose = p; }

            prevPose = p;
            prevTs   = t;
        }

        // Timestamp outside window — return nearest stored entry
        return bestPose != null ? bestPose : getPose();
    }
    public boolean   isPointedAtTarget()   { return headingPID.atSetpoint(); }
    public boolean   isHeadingLockActive() { return headingLockActive; }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    /**
     * Injects a vision-derived pose measurement into the Kalman filter.
     * Called from RobotContainer.updatePoseEstimator() every loop when vision has a fresh pose.
     * Std devs are scaled by tag count — more tags = higher trust = lower std devs.
     *
     * @param pose            2D robot pose from PhotonVision (meters, field-relative)
     * @param timestampSecs   FPGA timestamp of the frame the estimate came from
     * @param tagCount        Number of AprilTags used to produce this estimate
     */
    public void addVisionMeasurement(Pose2d pose, double timestampSecs, int tagCount) {
        int idx = Math.min(tagCount - 1, visionConstants.VISION_STD_DEV_TAG_SCALE.length - 1);
        double scale = visionConstants.VISION_STD_DEV_TAG_SCALE[idx];
        poseEstimator.addVisionMeasurement(
            pose,
            timestampSecs,
            MatBuilder.fill(Nat.N3(), Nat.N1(),
                visionConstants.VISION_STD_DEV_X     * scale,
                visionConstants.VISION_STD_DEV_Y     * scale,
                visionConstants.VISION_STD_DEV_THETA * scale)
        );
    }

    public void zeroYaw() { imu.setYaw(0.0); }

    // ─────────────────────────────────────────────────────────────────────────
    // Internal helpers
    // ─────────────────────────────────────────────────────────────────────────

    private void commandModules(ChassisSpeeds speeds, Translation2d cor) {
        lastCommandedSpeeds = speeds;
        SwerveModuleState[] desired = kinematics.toSwerveModuleStates(speeds, cor);
        SwerveDriveKinematics.desaturateWheelSpeeds(desired, swerveConstants.MAX_DRIVE_SPEED_MPS);
        for (int i = 0; i < modules.length; i++) {
            desired[i].optimize(modules[i].getState().angle);
            modules[i].setDesiredState(desired[i]);
        }
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] p = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) p[i] = modules[i].getPosition();
        return p;
    }

    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] s = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) s[i] = modules[i].getState();
        return s;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Odometry state computation
    // ─────────────────────────────────────────────────────────────────────────

    private driveOdometryState computeOdometryState(double dt) {
        ChassisSpeeds enc    = kinematics.toChassisSpeeds(getModuleStates());
        double encVx         = enc.vxMetersPerSecond;
        double encVy         = enc.vyMetersPerSecond;
        double encLinVelMag  = Math.hypot(encVx, encVy);
        double encLinVelHdg  = Math.atan2(encVy, encVx);
        double encAccelVx    = (encVx - prevLinVelX) / dt;
        double encAccelVy    = (encVy - prevLinVelY) / dt;
        double encLinAccel   = (encLinVelMag - prevLinVelMag) / dt;
        double encLinAccelHdg = Math.atan2(encAccelVy, encAccelVx);
        double encAngVel     = enc.omegaRadiansPerSecond;
        double encAngAccel   = (encAngVel - prevAngVel) / dt;

        double corX   = commandedCoR.getX();
        double corY   = commandedCoR.getY();
        double corVx  = (corX - prevCorX) / dt;
        double corVy  = (corY - prevCorY) / dt;
        double corAx  = (corVx - prevCorVelX) / dt;
        double corAy  = (corVy - prevCorVelY) / dt;

        driveOdometryState.bucket encBucket = new driveOdometryState.bucket(
            encLinVelMag, encLinVelHdg, encLinAccel, encLinAccelHdg,
            encAngVel, encAngAccel,
            new double[]{corX, corY}, new double[]{corVx, corVy}, new double[]{corAx, corAy}
        );

        // Pigeon 2: getAngularVelocityZDevice() → deg/s, getAcceleration*() → g's
        double imuAngVelRad   = Math.toRadians(imu.getAngularVelocityZDevice().getValueAsDouble());
        double imuAngAccel    = (imuAngVelRad - prevImuAngVel) / dt;
        double imuAxMs2       = imu.getAccelerationX().getValueAsDouble() * 9.81;
        double imuAyMs2       = imu.getAccelerationY().getValueAsDouble() * 9.81;
        double imuLinAccelMag = Math.hypot(imuAxMs2, imuAyMs2);
        double imuLinAccelHdg = Math.atan2(imuAyMs2, imuAxMs2);
        double imuLinVelX     = prevLinVelX + imuAxMs2 * dt;
        double imuLinVelY     = prevLinVelY + imuAyMs2 * dt;
        double imuLinVelMag   = Math.hypot(imuLinVelX, imuLinVelY);
        double imuLinVelHdg   = Math.atan2(imuLinVelY, imuLinVelX);

        driveOdometryState.bucket imuBucket = new driveOdometryState.bucket(
            imuLinVelMag, imuLinVelHdg, imuLinAccelMag, imuLinAccelHdg,
            imuAngVelRad, imuAngAccel,
            new double[]{corX, corY}, new double[]{corVx, corVy}, new double[]{corAx, corAy}
        );

        double a = blendAlpha;
        driveOdometryState.bucket blended = new driveOdometryState.bucket(
            a * encLinVelMag  + (1-a) * imuLinVelMag,
            blendAngle(encLinVelHdg, imuLinVelHdg, a),
            a * encLinAccel   + (1-a) * imuLinAccelMag,
            blendAngle(encLinAccelHdg, imuLinAccelHdg, a),
            (1-a) * imuAngVelRad + a * encAngVel,
            (1-a) * imuAngAccel  + a * encAngAccel,
            new double[]{corX, corY}, new double[]{corVx, corVy}, new double[]{corAx, corAy}
        );

        prevLinVelMag = encLinVelMag; prevLinVelX = encVx; prevLinVelY = encVy;
        prevAngVel    = encAngVel;    prevImuAngVel = imuAngVelRad;
        prevCorX = corX; prevCorY = corY; prevCorVelX = corVx; prevCorVelY = corVy;

        return new driveOdometryState(encBucket, imuBucket, blended, Timer.getFPGATimestamp());
    }

    private double blendAngle(double a1, double a2, double alpha) {
        double diff = a2 - a1;
        while (diff >  Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        return a1 + alpha * diff;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Telemetry
    // ─────────────────────────────────────────────────────────────────────────

    private void publishGeometry() {
        SmartDashboard.putNumber("Drive/Geometry/WheelBase X (in)",  units.m_inches(wheelBaseM[0]));
        SmartDashboard.putNumber("Drive/Geometry/WheelBase Y (in)",  units.m_inches(wheelBaseM[1]));
        SmartDashboard.putNumber("Drive/Geometry/Frame X (in)",      units.m_inches(framePerimeterM[0]));
        SmartDashboard.putNumber("Drive/Geometry/Frame Y (in)",      units.m_inches(framePerimeterM[1]));
        SmartDashboard.putNumber("Drive/Geometry/Bumper X (in)",     units.m_inches(bumperPerimeterM[0]));
        SmartDashboard.putNumber("Drive/Geometry/Bumper Y (in)",     units.m_inches(bumperPerimeterM[1]));
        SmartDashboard.putNumber("Drive/Geometry/WheelBase X (m)",   wheelBaseM[0]);
        SmartDashboard.putNumber("Drive/Geometry/WheelBase Y (m)",   wheelBaseM[1]);
        SmartDashboard.putNumber("Drive/Blend Alpha",                 blendAlpha);
    }

    private void publishTelemetry() {
        Pose2d pose = getPose();
        SmartDashboard.putNumber("Drive/Pose X (ft)",   units.m_feet(pose.getX()));
        SmartDashboard.putNumber("Drive/Pose Y (ft)",   units.m_feet(pose.getY()));
        SmartDashboard.putNumber("Drive/Pose X (m)",    pose.getX());
        SmartDashboard.putNumber("Drive/Pose Y (m)",    pose.getY());
        SmartDashboard.putNumber("Drive/Heading (deg)", getYaw().getDegrees());

        driveOdometryState s = currentState;
        SmartDashboard.putNumber("Drive/Blended/LinVel (ft-s)",
            units.mps_ftps(s.blendedState.linearVelocityMagnitude));
        SmartDashboard.putNumber("Drive/Blended/AngVel (rad-s)", s.blendedState.angularVelocity);
        SmartDashboard.putNumber("Drive/Blended/CoR X (in)",
            units.m_inches(s.blendedState.centerOfRotation[0]));
        SmartDashboard.putNumber("Drive/Blended/CoR Y (in)",
            units.m_inches(s.blendedState.centerOfRotation[1]));

        for (swerveModule m : modules) m.publishTelemetry();

        // ── DataLog ───────────────────────────────────────────────────────────
        RobotLogger.drivePoseX.append(pose.getX());
        RobotLogger.drivePoseY.append(pose.getY());
        RobotLogger.drivePoseHeadingDeg.append(getYaw().getDegrees());

        RobotLogger.driveCommandedVx.append(lastCommandedSpeeds.vxMetersPerSecond);
        RobotLogger.driveCommandedVy.append(lastCommandedSpeeds.vyMetersPerSecond);
        RobotLogger.driveCommandedOmega.append(lastCommandedSpeeds.omegaRadiansPerSecond);

        ChassisSpeeds actual = getRobotRelativeSpeeds();
        RobotLogger.driveActualVx.append(actual.vxMetersPerSecond);
        RobotLogger.driveActualVy.append(actual.vyMetersPerSecond);
        RobotLogger.driveActualOmega.append(actual.omegaRadiansPerSecond);

        RobotLogger.driveEncLinVel.append(s.encoderState.linearVelocityMagnitude);
        RobotLogger.driveEncAngVel.append(s.encoderState.angularVelocity);
        RobotLogger.driveEncLinAccel.append(s.encoderState.linearAccelerationMagnitude);

        RobotLogger.driveImuLinVel.append(s.imuState.linearVelocityMagnitude);
        RobotLogger.driveImuAngVel.append(s.imuState.angularVelocity);
        RobotLogger.driveImuLinAccel.append(s.imuState.linearAccelerationMagnitude);

        RobotLogger.driveBlendedLinVel.append(s.blendedState.linearVelocityMagnitude);
        RobotLogger.driveBlendedAngVel.append(s.blendedState.angularVelocity);
        RobotLogger.driveBlendedLinAccel.append(s.blendedState.linearAccelerationMagnitude);
    }
}