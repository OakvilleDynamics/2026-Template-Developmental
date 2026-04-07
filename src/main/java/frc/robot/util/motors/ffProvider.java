package frc.robot.util.motors;

/**
 * ffProvider.java
 * PATH: src/main/java/frc/robot/util/motors/ffProvider.java
 *
 * Functional interface for feed-forward voltage computations in mechanismUnit.
 * Used for both tier-2 (self-contained gravity/spring physics) and tier-3
 * (cross-system coupling that requires external subsystem state).
 *
 * ─── UNIT CONTRACT ───────────────────────────────────────────────────────────
 *   positionDeg : mechanism shaft position in degrees (after gear ratio)
 *   velocityRps : mechanism shaft velocity in rotations/second (signed)
 *   accelRpss   : mechanism shaft acceleration in rotations/second² (signed)
 *   return      : feed-forward voltage addend in volts
 *
 * ─── TIER-2 vs TIER-3 ────────────────────────────────────────────────────────
 * The type is the same for both tiers. The difference is what the lambda closes over:
 *
 *   Tier-2: closes over nothing external — uses only the params provided.
 *           Example: arm gravity, spring pull, multi-stage elevator weight steps.
 *
 *   Tier-3: closes over other subsystem references captured at construction time
 *           in RobotContainer (where all subsystems exist).
 *           Example: gyroscopic resistance from a flywheel, elevator tilt from
 *           a rotating base, game-piece-count-dependent weight.
 *
 * ─── EXAMPLE TIER-2 LAMBDAS ──────────────────────────────────────────────────
 *
 *   // Rotating arm: full gravity at horizontal (0°), zero at vertical (90°)
 *   (pos, vel, accel) -> KG * Math.cos(Math.toRadians(pos))
 *
 *   // Elevator: different constant FF for up vs down
 *   (pos, vel, accel) -> vel >= 0 ? KG_UP : KG_DOWN
 *
 *   // Multi-stage elevator: weight steps as stages extend
 *   (pos, vel, accel) -> {
 *       double kG = vel >= 0 ? KG_BASE_UP : KG_BASE_DOWN;
 *       if (pos > STAGE_2_EXTENDS_DEG) kG += vel >= 0 ? STAGE_2_UP : STAGE_2_DOWN;
 *       if (pos > STAGE_3_EXTENDS_DEG) kG += vel >= 0 ? STAGE_3_UP : STAGE_3_DOWN;
 *       return kG;
 *   }
 *
 *   // Non-linear spring (constant-force spring on energy chain puller)
 *   (pos, vel, accel) -> springForceTable.interpolate(pos)
 *
 * ─── EXAMPLE TIER-3 LAMBDAS ──────────────────────────────────────────────────
 *
 *   // Gyroscopic resistance: turret opposed by spinning flywheel
 *   // flywheel reference captured at RobotContainer construction time
 *   (pos, vel, accel) -> {
 *       double omegaTurretRadps   = vel * 2 * Math.PI;
 *       double omegaFlywheelRadps = flywheel.getVelocityRps() * 2 * Math.PI;
 *       return (FLYWHEEL_MOI_KGM2 * omegaFlywheelRadps * omegaTurretRadps) / MOTOR_TORQUE_K;
 *   }
 *
 *   // Tilting multi-stage elevator with game pieces: gravity + centrifugal + inertial
 *   // base and gamePieceTracker captured at construction time
 *   (pos, vel, accel) -> {
 *       double massKg = CARRIAGE_MASS_KG + gamePieceTracker.getCount() * PIECE_MASS_KG;
 *       if (pos > STAGE_2_DEG) massKg += STAGE_2_MASS_KG;
 *       double gravN     = massKg * 9.81 * Math.sin(Math.toRadians(base.getPositionDeg()));
 *       double radiusM   = pos / 360.0 * METERS_PER_ROT;
 *       double omegaBase = base.getVelocityRps() * 2 * Math.PI;
 *       double centrN    = massKg * omegaBase * omegaBase * radiusM;
 *       double inertN    = massKg * accel * METERS_PER_ROT;
 *       return (gravN - centrN + inertN) / NEWTONS_PER_VOLT;
 *   }
 *
 * ─── CONVENIENCE FACTORIES ───────────────────────────────────────────────────
 * See mechanismConfig.armFF(), mechanismConfig.elevatorFF(), mechanismConfig.noFF()
 * for pre-built lambdas covering the most common cases.
 *
 * ─── THREAD SAFETY ───────────────────────────────────────────────────────────
 * Both tier-2 and tier-3 lambdas are called exclusively from the robot periodic
 * thread (inside mechanismUnit.setVelocity() / setPosition()). No synchronization
 * is required as long as all callers remain on the same thread — which WPILib
 * CommandScheduler guarantees.
 */
@FunctionalInterface
public interface ffProvider {

    /**
     * Compute a feed-forward voltage addend for the current control cycle.
     *
     * @param positionDeg mechanism shaft position (degrees, after gear ratio applied)
     * @param velocityRps mechanism shaft velocity (rotations/second, signed:
     *                    positive = forward/up/out per mechanism convention)
     * @param accelRpss   mechanism shaft acceleration (rotations/second², signed)
     * @return            feed-forward voltage to add to the motor output command (volts)
     */
    double compute(double positionDeg, double velocityRps, double accelRpss);
}
