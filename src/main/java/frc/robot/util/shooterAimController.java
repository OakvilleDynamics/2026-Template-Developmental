package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.driveWithJoysticks;
import frc.robot.subsystems.swerveDrive.driveOdometryState;
import frc.robot.subsystems.swerveDrive.swerveDrive;
import frc.robot.util.ShooterCalculator.ShooterOutput;
import frc.robot.util.mechanisms.shooterMechanism;

/**
 * shooterAimController.java
 * PATH: src/main/java/frc/robot/util/shooterAimController.java
 *
 * Orchestration middleware between the drivetrain, ShooterCalculator, and
 * shooterMechanism. Runs every loop as a SubsystemBase.
 *
 * ─── RESPONSIBILITIES ─────────────────────────────────────────────────────────
 * 1. Reads robot pose and odometry state from swerveDrive each loop.
 * 2. Calls ShooterCalculator with the live state to produce setpoints.
 * 3. Pushes RPM, hood angle, and (if PHYSICAL_TURRET) turret angle to shooterMechanism.
 * 4. Directs driveWithJoysticks into the appropriate drive mode:
 *      STATIC_SHOT    → X-lock (physical turret) or aim-then-lock (swerve turret)
 *      ON_THE_MOVE    → heading-bounded driver control
 *      IDLE           → full driver control (no override)
 *
 * ─── WIRING ──────────────────────────────────────────────────────────────────
 * Instantiate in RobotContainer. Register as a subsystem (periodic runs automatically).
 *
 * Button bindings (whileHeld pattern):
 *   button.whileTrue(Commands.runOnce(() -> aimController.setAimMode(AimMode.ON_THE_MOVE))
 *                   .andThen(Commands.idle())
 *                   .finallyDo(interrupted -> aimController.clearAimMode()));
 *
 * Target selection:
 *   aimController.setTarget("Hub", targetFeetX, targetFeetY);
 *
 * Ready gate (wire into shooterMechanism.Builder):
 *   .withReadyGate(() -> aimController.getLastResult().isHeadingInBounds())
 *
 * ─── TURRET MODES ─────────────────────────────────────────────────────────────
 * PHYSICAL_TURRET   — a physical turret motor rotates to aim; robot heading is
 *                     guided within the allowed window but not locked.
 *                     Static shot: wheels X-lock immediately while turret aims.
 *
 * SWERVE_AS_TURRET  — no physical turret; the robot heading IS the aim direction.
 *                     ShooterCalculator receives turretAngle = 0° always.
 *                     Static shot: two-phase — robot aims (POINT_AT) then X-locks.
 *
 * ─── UNIT CONVERSIONS ─────────────────────────────────────────────────────────
 * driveOdometryState is SI (m, m/s, m/s², rad, rad/s, rad/s²).
 * ShooterCalculator expects English (ft, ft/s, ft/s², deg, deg/s, deg/s²).
 * All conversions use units.java methods at the entry point here.
 */
public class shooterAimController extends SubsystemBase {

    // =========================================================================
    // Enums
    // =========================================================================

    public enum TurretMode {
        /** Robot has a physical turret motor — robot heading is guided, not locked. */
        PHYSICAL_TURRET,
        /**
         * No physical turret — swerve drive heading IS the turret.
         * Pass turretAngle = 0, turret vel/accel = 0 to ShooterCalculator.
         * Static shot performs aim-then-lock instead of immediate X-lock.
         */
        SWERVE_AS_TURRET
    }

    public enum AimMode {
        /** Robot is aimed at target then wheels are X-locked for a stationary shot. */
        STATIC_SHOT,
        /** Driver retains translation; robot heading is clamped within turret allow-window. */
        ON_THE_MOVE,
        /** No active aiming — full driver control, no shooter setpoints pushed. */
        IDLE
    }

    // =========================================================================
    // ShooterAimOutput — cached result published each loop
    // =========================================================================

    public static class ShooterAimOutput {

        /** Full ballistics result including headingBoundsDeg. Null if IDLE. */
        public final ShooterOutput shooterResult;

        /** True when the robot heading is currently outside headingBoundsDeg. */
        public final boolean headingChangeRequired;

        /** True when shooterMechanism.isReady() — all sub-mechanisms at setpoint. */
        public final boolean shooterReady;

        /** Which mode produced this output. */
        public final AimMode activeMode;

        ShooterAimOutput(
                ShooterOutput shooterResult,
                boolean headingChangeRequired,
                boolean shooterReady,
                AimMode activeMode) {
            this.shooterResult        = shooterResult;
            this.headingChangeRequired = headingChangeRequired;
            this.shooterReady         = shooterReady;
            this.activeMode           = activeMode;
        }

        /**
         * Convenience: returns true when the robot heading is within the bounds
         * output by ShooterCalculator. Use as a ready gate for shooterMechanism.
         * Returns true when IDLE or when no shooter result is available.
         */
        public boolean isHeadingInBounds() {
            return !headingChangeRequired;
        }

        /** Null-safe: returns true when IDLE (no shooter result). */
        public static ShooterAimOutput idle() {
            return new ShooterAimOutput(null, false, false, AimMode.IDLE);
        }
    }

    // =========================================================================
    // Dependencies
    // =========================================================================

    private final ShooterCalculator    calc;
    private final swerveDrive          drive;
    private final shooterMechanism     shooter;
    private final driveWithJoysticks   driveCmd;
    private final TurretMode           turretMode;

    // =========================================================================
    // State
    // =========================================================================

    private AimMode    activeMode       = AimMode.IDLE;
    private String     targetType       = "";
    private double     targetFeetX      = 0.0;
    private double     targetFeetY      = 0.0;

    /**
     * For SWERVE_AS_TURRET + STATIC_SHOT: two-phase state machine.
     * Phase 1 (false) = aiming with POINT_AT.
     * Phase 2 (true)  = X-locked after heading acquired.
     */
    private boolean    staticShotLocked = false;

    // ── Turret bound state (ON_THE_MOVE + PHYSICAL_TURRET only) ──────────────

    /**
     * Tracks where the turret is parked when robot heading has drifted outside
     * the valid window and the target is temporarily unreachable.
     *
     * IN_BOUNDS
     *   Heading within headingBoundsDeg. Turret aims normally. isReady() can fire.
     *
     * WAITING_AT_FORWARD_LIMIT
     *   Heading went CCW past headingBoundsDeg[0] — target would require the turret
     *   to exceed softMax. Turret parks at softMax and waits.
     *   → Back to IN_BOUNDS if heading returns inside the valid window.
     *   → Flips to WAITING_AT_REVERSE_LIMIT if heading drifts CCW past the flip
     *     threshold (X% of the dead zone from softMax toward softMin side).
     *     Turret traverses to softMin; once heading enters the valid window from
     *     that side the turret backs off softMin and aims normally.
     *
     * WAITING_AT_REVERSE_LIMIT
     *   Symmetric: heading went CW past headingBoundsDeg[1]. Turret parks at softMin.
     *   → Back to IN_BOUNDS if heading returns inside the valid window.
     *   → Flips to WAITING_AT_FORWARD_LIMIT if heading drifts CW past the flip threshold.
     */
    private enum TurretBoundState {
        IN_BOUNDS,
        WAITING_AT_FORWARD_LIMIT,
        WAITING_AT_REVERSE_LIMIT
    }

    private TurretBoundState turretBoundState = TurretBoundState.IN_BOUNDS;

    /**
     * Fraction of the dead zone width at which the turret flips from one parked
     * limit to the other. Measured from the soft limit the turret is currently
     * parked at, toward the opposite limit.
     *
     * 0.5 = flip at the midpoint of the dead zone (break-even point — neither
     *   rotation direction is shorter). Good default for symmetric turrets.
     * < 0.5 = flip sooner; > 0.5 = wait longer before committing.
     * Range: (0.0, 1.0).
     */
    private final double flipThresholdPct;

    private ShooterAimOutput lastResult = ShooterAimOutput.idle();

    // =========================================================================
    // Constructor
    // =========================================================================

    /**
     * @param calc              ShooterCalculator instance.
     * @param drive             swerveDrive subsystem.
     * @param shooter           shooterMechanism subsystem.
     * @param driveCmd          driveWithJoysticks default drive command.
     * @param turretMode        Whether a physical turret motor exists.
     * @param flipThresholdPct  Fraction of dead-zone width at which the turret
     *                          flips from one parked limit to the other during
     *                          ON_THE_MOVE. 0.5 = midpoint (recommended default).
     */
    public shooterAimController(
            ShooterCalculator  calc,
            swerveDrive        drive,
            shooterMechanism   shooter,
            driveWithJoysticks driveCmd,
            TurretMode         turretMode,
            double             flipThresholdPct) {

        this.calc              = calc;
        this.drive             = drive;
        this.shooter           = shooter;
        this.driveCmd          = driveCmd;
        this.turretMode        = turretMode;
        this.flipThresholdPct  = flipThresholdPct;
    }

    /**
     * Convenience constructor using the recommended 50% flip threshold.
     */
    public shooterAimController(
            ShooterCalculator  calc,
            swerveDrive        drive,
            shooterMechanism   shooter,
            driveWithJoysticks driveCmd,
            TurretMode         turretMode) {
        this(calc, drive, shooter, driveCmd, turretMode, 0.5);
    }

    // =========================================================================
    // Public API — target and mode control
    // =========================================================================

    /**
     * Set the field position of the target to aim at.
     * Call once from RobotContainer when the game piece / goal position is known,
     * or call each loop if the target is dynamic.
     *
     * @param targetType  String key matching shooterConstants.LOOKUP_TABLE rows.
     * @param targetFeetX Field X of target (feet).
     * @param targetFeetY Field Y of target (feet).
     */
    public void setTarget(String targetType, double targetFeetX, double targetFeetY) {
        this.targetType   = targetType;
        this.targetFeetX  = targetFeetX;
        this.targetFeetY  = targetFeetY;
    }

    /**
     * Activate a shot mode. Call from a button binding (whileHeld / whileTrue).
     * Pair with clearAimMode() on button release.
     *
     * @param mode STATIC_SHOT or ON_THE_MOVE.
     */
    public void setAimMode(AimMode mode) {
        if (mode != activeMode) {
            staticShotLocked  = false;
            turretBoundState  = TurretBoundState.IN_BOUNDS;
        }
        this.activeMode = mode;
    }

    /**
     * Restore IDLE mode and release all drive overrides.
     * Call on button release (finallyDo / onFalse).
     */
    public void clearAimMode() {
        activeMode       = AimMode.IDLE;
        staticShotLocked = false;
        turretBoundState = TurretBoundState.IN_BOUNDS;
        driveCmd.clearAimMode();
        lastResult = ShooterAimOutput.idle();
    }

    /**
     * Returns the most recent aim calculation result.
     * Use for telemetry and as the shooterMechanism ready gate:
     *   .withReadyGate(() -> aimController.getLastResult().isHeadingInBounds())
     */
    public ShooterAimOutput getLastResult() {
        return lastResult;
    }

    // =========================================================================
    // periodic — called automatically every loop by CommandScheduler
    // =========================================================================

    @Override
    public void periodic() {
        if (activeMode == AimMode.IDLE || targetType.isEmpty()) {
            driveCmd.clearAimMode();
            lastResult = ShooterAimOutput.idle();
            return;
        }

        // --- 1. Read drivetrain state ---
        edu.wpi.first.math.geometry.Pose2d pose = drive.getPose();
        driveOdometryState odo = drive.getOdometryState();
        driveOdometryState.bucket b = odo.blendedState;

        double robotX_ft  = units.m_feet(pose.getX());
        double robotY_ft  = units.m_feet(pose.getY());
        double robotHdgDeg = pose.getRotation().getDegrees();

        // --- 2. Decompose blended state for ShooterCalculator ---
        // Linear velocity: polar (magnitude + heading) → Cartesian field X/Y in ft/s
        double linVelX_fps = units.mps_ftps(
                b.linearVelocityMagnitude * Math.cos(b.linearVelocityHeading));
        double linVelY_fps = units.mps_ftps(
                b.linearVelocityMagnitude * Math.sin(b.linearVelocityHeading));

        // Linear acceleration: same decomposition
        double linAccX_fps2 = units.mps2_ftps2(
                b.linearAccelerationMagnitude * Math.cos(b.linearAccelerationHeading));
        double linAccY_fps2 = units.mps2_ftps2(
                b.linearAccelerationMagnitude * Math.sin(b.linearAccelerationHeading));

        // Angular velocity and acceleration: rad/s → deg/s
        double angVel_degs   = units.rad_deg(b.angularVelocity);
        double angAccel_degs2 = units.rad_deg(b.angularAcceleration);

        // Center of rotation: meters → feet
        double corX_ft = units.m_feet(b.centerOfRotation[0]);
        double corY_ft = units.m_feet(b.centerOfRotation[1]);

        // --- 3. Determine turret state ---
        double turretAngleDeg  = 0.0;
        double turretVelDegS   = 0.0;
        double turretAccDegS2  = 0.0;

        if (turretMode == TurretMode.PHYSICAL_TURRET) {
            turretAngleDeg = shooter.getTurretPositionDeg();
            // Convert RPS → deg/s (1 RPS = 360 deg/s)
            turretVelDegS  = shooter.getTurretVelocityRps() * 360.0;
            // Acceleration not currently exposed by shooterMechanism — use 0
            turretAccDegS2 = 0.0;
        }
        // SWERVE_AS_TURRET: all turret inputs stay at 0 — robot heading IS the aim direction

        // --- 4. Call ShooterCalculator ---
        ShooterOutput result;
        if (activeMode == AimMode.STATIC_SHOT) {
            result = calc.calculateStaticShot(
                    targetType,
                    targetFeetX, targetFeetY,
                    robotX_ft,   robotY_ft,
                    robotHdgDeg);
        } else {
            result = calc.calculateOnTheMove(
                    targetType,
                    targetFeetX,    targetFeetY,
                    robotX_ft,      robotY_ft,
                    robotHdgDeg,
                    linVelX_fps,    linVelY_fps,
                    linAccX_fps2,   linAccY_fps2,
                    angVel_degs,    angAccel_degs2,
                    corX_ft,        corY_ft,
                    turretAngleDeg,
                    turretVelDegS,  turretAccDegS2);
        }

        // --- 5. Push setpoints to shooterMechanism ---
        shooter.setFlywheelRpm(result.rpm);
        shooter.setHoodDeg(result.hoodAngle);
        if (turretMode == TurretMode.PHYSICAL_TURRET) {
            // For ON_THE_MOVE, route through the bound state machine so the turret
            // parks at a soft limit and waits (or flips) when heading is out of range.
            // For STATIC_SHOT, the heading correction phase ensures we're in bounds
            // before X-locking, so the raw aim angle is always reachable.
            double turretCmd = (activeMode == AimMode.ON_THE_MOVE)
                    ? computeTurretCommandOTM(result, robotHdgDeg)
                    : result.turretAngleRobotDeg;
            shooter.setTurretDeg(turretCmd);
        }

        // --- 6. Determine whether heading change is needed ---
        boolean headingInBounds = result.isHeadingInBounds(robotHdgDeg);

        // --- 7. Set drive mode ---
        applyDriveMode(result, headingInBounds);

        // --- 8. Cache result ---
        lastResult = new ShooterAimOutput(
                result,
                !headingInBounds,
                shooter.isReady(),
                activeMode);
    }

    // =========================================================================
    // Drive mode dispatch
    // =========================================================================

    private void applyDriveMode(ShooterOutput result, boolean headingInBounds) {
        switch (activeMode) {
            case ON_THE_MOVE:
                // Clamp driver heading within the turret's allowed window.
                driveCmd.enableHeadingBound(
                        result.headingBoundsDeg[0],
                        result.headingBoundsDeg[1]);
                break;

            case STATIC_SHOT:
                applyStaticShotDriveMode(result, headingInBounds);
                break;

            default:
                break;
        }
    }

    /**
     * Static shot drive mode.
     *
     * Both turret modes use a two-phase approach:
     *   Phase 1 — bring robot heading into the valid window (turret can reach target)
     *   Phase 2 — X-lock once aimed; robot stays braced while shot fires
     *
     * PHYSICAL_TURRET:
     *   Phase 1 uses HEADING_BOUND — rotates to the nearest valid heading edge.
     *   Once heading is in bounds the turret can reach the target, so X-lock.
     *   If the heading is already in bounds when the button is pressed, skip
     *   directly to Phase 2.
     *
     * SWERVE_AS_TURRET:
     *   Phase 1 uses POINT_AT — locks the whole robot to face the target exactly.
     *   Once isPointedAtTarget() is true, X-lock.
     *
     * In both cases staticShotLocked latches true once Phase 2 is entered so that
     * minor heading drift after the X-lock does not re-trigger Phase 1. The
     * readyGate on shooterMechanism handles the case where drift causes the turret
     * to fall out of range after locking — isReady() will return false and prevent
     * firing until the driver releases and re-engages the button.
     */
    private void applyStaticShotDriveMode(ShooterOutput result, boolean headingInBounds) {
        if (turretMode == TurretMode.PHYSICAL_TURRET) {
            if (staticShotLocked || headingInBounds) {
                // Phase 2: heading is (or was) in bounds — X-brace and hold
                staticShotLocked = true;
                driveCmd.enableStaticShotLock();
            } else {
                // Phase 1: heading outside turret's reach — rotate to nearest valid bound
                driveCmd.enableHeadingBound(
                        result.headingBoundsDeg[0],
                        result.headingBoundsDeg[1]);
            }
            return;
        }

        // SWERVE_AS_TURRET two-phase:
        if (!staticShotLocked) {
            // Phase 1: aim robot toward target
            driveCmd.enablePointAt(targetFeetX, targetFeetY, 0.0);
            if (drive.isPointedAtTarget()) {
                staticShotLocked = true;
            }
        } else {
            // Phase 2: locked on target — X-brace
            driveCmd.enableStaticShotLock();
        }
    }

    // =========================================================================
    // Turret bound state machine — ON_THE_MOVE + PHYSICAL_TURRET only
    // =========================================================================

    /**
     * Determines the actual turret position command to send to shooterMechanism
     * during ON_THE_MOVE mode when the robot heading may drift outside the valid
     * aiming window.
     *
     * When in bounds: returns result.turretAngleRobotDeg (normal aim).
     *
     * When out of bounds: parks the turret at the soft limit corresponding to
     * which side the heading exited from, and waits. isReady() stays false since
     * the turret is not at its aim setpoint. Three possible transitions:
     *   - Heading returns inside bounds → turretBoundState = IN_BOUNDS, aim resumes.
     *   - Heading drifts further past the flip threshold → turret traverses to the
     *     opposite soft limit (turretBoundState flips). Once heading returns inside
     *     bounds from that side, aim resumes from the new approach angle.
     *   - Heading reverses from the new waiting position → same logic in reverse.
     *
     * Threshold math:
     *   deadZoneDeg     = 360° − (boundsMax − boundsMin)  [valid window complement]
     *   flipThresholdDeg = deadZoneDeg × flipThresholdPct
     *   overshoot       = angular distance heading has traveled past the bound edge,
     *                     computed with normalizeAngleDeg() for wrap-correctness.
     */
    private double computeTurretCommandOTM(ShooterOutput result, double robotHdgDeg) {
        double boundsMin = result.headingBoundsDeg[0];
        double boundsMax = result.headingBoundsDeg[1];

        // Valid window width — wrap-aware
        double validWindowDeg = boundsMax - boundsMin;
        if (validWindowDeg < 0) validWindowDeg += 360.0;
        double deadZoneDeg       = 360.0 - validWindowDeg;
        double flipThresholdDeg  = deadZoneDeg * flipThresholdPct;

        boolean inBounds = ShooterOutput.isInArc(robotHdgDeg, boundsMin, boundsMax);

        switch (turretBoundState) {

            case IN_BOUNDS:
                if (!inBounds) {
                    // Determine which soft limit was crossed from the required turret angle.
                    // If the angle exceeds softMax the heading went CCW past boundsMin.
                    // If it is below softMin the heading went CW past boundsMax.
                    if (result.turretAngleRobotDeg > calc.getTurretSoftLimitMaxDeg()) {
                        turretBoundState = TurretBoundState.WAITING_AT_FORWARD_LIMIT;
                        return calc.getTurretSoftLimitMaxDeg();
                    } else {
                        turretBoundState = TurretBoundState.WAITING_AT_REVERSE_LIMIT;
                        return calc.getTurretSoftLimitMinDeg();
                    }
                }
                return result.turretAngleRobotDeg;

            case WAITING_AT_FORWARD_LIMIT:
                if (inBounds) {
                    // Heading returned inside the valid window — resume normal aim
                    turretBoundState = TurretBoundState.IN_BOUNDS;
                    return result.turretAngleRobotDeg;
                }
                // How far CCW past boundsMin has the heading traveled?
                // normalizeAngleDeg(boundsMin - robotHdgDeg) is positive when heading
                // is CCW of boundsMin, handles ±180° wrap correctly.
                double forwardOvershoot = units.normalizeAngleDeg(boundsMin - robotHdgDeg);
                if (forwardOvershoot > flipThresholdDeg) {
                    // Heading has crossed the midpoint of the dead zone — flip
                    turretBoundState = TurretBoundState.WAITING_AT_REVERSE_LIMIT;
                    return calc.getTurretSoftLimitMinDeg();
                }
                return calc.getTurretSoftLimitMaxDeg();

            case WAITING_AT_REVERSE_LIMIT:
                if (inBounds) {
                    turretBoundState = TurretBoundState.IN_BOUNDS;
                    return result.turretAngleRobotDeg;
                }
                // How far CW past boundsMax has the heading traveled?
                double reverseOvershoot = units.normalizeAngleDeg(robotHdgDeg - boundsMax);
                if (reverseOvershoot > flipThresholdDeg) {
                    turretBoundState = TurretBoundState.WAITING_AT_FORWARD_LIMIT;
                    return calc.getTurretSoftLimitMaxDeg();
                }
                return calc.getTurretSoftLimitMinDeg();

            default:
                return result.turretAngleRobotDeg;
        }
    }
}
