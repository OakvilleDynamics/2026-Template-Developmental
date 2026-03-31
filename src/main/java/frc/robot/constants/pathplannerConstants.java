package frc.robot.constants;

import frc.robot.util.units;

/**
 * pathplannerConstants.java
 * PATH: src/main/java/frc/robot/constants/pathplannerConstants.java
 *
 * Constants for PathPlanner integration.
 *
 * ─── UNIT CONVENTION ─────────────────────────────────────────────────────────
 * Physical robot properties follow the project pattern:
 *   English values defined first (lbs, lb·in²)
 *   SI values derived immediately below via units.*
 *   RobotContainer references the _KG and _KGM2 forms only.
 *
 * ─── WHAT LIVES HERE ─────────────────────────────────────────────────────────
 * Robot mass, moment of inertia, wheel COF, path constraints,
 * AutoBuilder translation/rotation PID defaults, vision staleness threshold.
 *
 * ─── WHAT DOES NOT LIVE HERE ─────────────────────────────────────────────────
 * Field target poses → FieldTargets.java
 * Alliance zone exclusions → Constants.java (PathPlanning inner class)
 * Module geometry → RobotContainer (derived from WHEEL_BASE_IN, same as swerveDrive)
 */
public final class pathplannerConstants {

    // ── Robot physical properties ─────────────────────────────────────────────

    /** Robot mass in pounds. TODO: weigh the robot with battery and bumpers */
    public static final double ROBOT_MASS_LBS  = 120.0;
    public static final double ROBOT_MASS_KG   = units.lbs_kg(ROBOT_MASS_LBS);

    /**
     * Robot moment of inertia about the vertical (Z) axis in lb·in².
     * TODO: calculate from CAD or measure via spin-down test.
     * Rough estimate: sum of (mass_i * r_i²) for major components.
     */
    public static final double ROBOT_MOI_LBIN2 = 2500.0;
    public static final double ROBOT_MOI_KGM2  = units.lbIn2_kgM2(ROBOT_MOI_LBIN2);

    /**
     * Wheel coefficient of friction on FRC carpet.
     * Typical range: 1.0–1.2 for billet wheels on competition carpet.
     * TODO: verify on actual field carpet.
     */
    public static final double WHEEL_COF = 1.2;

    // ── Drive current limit ───────────────────────────────────────────────────

    /**
     * Drive motor supply current limit passed to PathPlanner's ModuleConfig.
     * Must match the supply current limit set in swerveModule.configureDriveMotor().
     */
    public static final double DRIVE_CURRENT_LIMIT_AMPS = 60.0;

    // ── Path constraints ──────────────────────────────────────────────────────

    /** Maximum linear velocity during pathfinding (m/s). Matches drive speed cap. */
    public static final double MAX_PATH_VEL_MPS         = swerveConstants.MAX_DRIVE_SPEED_MPS;

    /** Maximum linear acceleration during pathfinding (m/s²). TODO: tune on carpet. */
    public static final double MAX_PATH_ACCEL_MPS2      = 3.0;

    /** Maximum angular velocity during pathfinding (rad/s). Matches drive angular cap. */
    public static final double MAX_PATH_ANG_VEL_RADPS   = swerveConstants.MAX_ANGULAR_SPEED_RPS;

    /** Maximum angular acceleration during pathfinding (rad/s²). TODO: tune on carpet. */
    public static final double MAX_PATH_ANG_ACCEL_RADPS2 = 2.0 * Math.PI;

    // ── AutoBuilder translation PID ───────────────────────────────────────────
    // Field X/Y position error → chassis velocity correction

    public static final double TRANSLATION_PID_kP = 5.0;  // TODO: tune
    public static final double TRANSLATION_PID_kI = 0.0;
    public static final double TRANSLATION_PID_kD = 0.0;

    // ── AutoBuilder rotation PID ──────────────────────────────────────────────
    // Heading error → omega correction (separate from drive headingPID)

    public static final double ROTATION_PID_kP = 5.0;  // TODO: tune
    public static final double ROTATION_PID_kI = 0.0;
    public static final double ROTATION_PID_kD = 0.0;

    // ── Vision staleness ──────────────────────────────────────────────────────

    /**
     * If the most recent accepted vision pose is older than this threshold (seconds),
     * pathfindCommand logs a Shuffleboard warning but still runs on odometry alone.
     */
    public static final double VISION_STALENESS_THRESHOLD_S = 0.5;

    // ── Alliance zone exclusions ──────────────────────────────────────────────

    /**
     * Dynamic obstacles injected at pathfindCommand initialize.
     * Each inner List<Translation2d> defines one convex polygon exclusion zone.
     * TODO: populate with 2026 field zone polygon vertices once field geometry is published.
     */
    public static final java.util.List<java.util.List<edu.wpi.first.math.geometry.Translation2d>>
        ALLIANCE_ZONE_EXCLUSIONS = java.util.Collections.emptyList();
}
