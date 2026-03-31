package frc.robot.subsystems.swerveDrive;

/**
 * driveOdometryState.java
 * PATH: src/main/java/frc/robot/subsystems/swerveDrive/driveOdometryState.java
 *
 * Full snapshot of drivetrain motion state, updated every 20ms loop.
 * Read via swerveDrive.getOdometryState() from any command or subsystem.
 *
 * ─── THREE BUCKETS ───────────────────────────────────────────────────────────
 *
 * encoderState  — derived from wheel encoders + kinematics only
 *                 Reliable for: short-term linear velocity, distance
 *                 Weakness: accumulates error from wheel slip over time
 *
 * imuState      — derived from ADIS16470 IMU only
 *                 Reliable for: angular velocity, angular acceleration, heading
 *                 Weakness: accelerometer drift on linear translation axes
 *
 * blendedState  — complementary filter combining both sources
 *                 Linear velocity/accel: encoder-dominant (alpha weight)
 *                 Angular velocity/accel: IMU-dominant (1-alpha weight)
 *                 Best general-purpose estimate for most use cases
 *
 * Blend weight alpha is tunable live from SmartDashboard: "Drive/Blend Alpha"
 * 0.0 = pure IMU, 1.0 = pure encoder. Default: 0.7 (encoder-dominant linear)
 *
 * ─── UNITS ───────────────────────────────────────────────────────────────────
 * All linear values in meters and m/s (SI internally).
 * All angular values in radians and rad/s.
 * All headings in radians (field-relative, 0 = field forward, CCW positive).
 * centerOfRotation x/y in meters, relative to robot center (0,0).
 */
public class driveOdometryState {

    // ─────────────────────────────────────────────────────────────────────────
    // Inner class: one bucket of motion state
    // ─────────────────────────────────────────────────────────────────────────

    public static class bucket {

        /** Linear speed magnitude (m/s) */
        public final double linearVelocityMagnitude;

        /** Direction of velocity vector (radians, field-relative) */
        public final double linearVelocityHeading;

        /** Linear acceleration magnitude (m/s²) */
        public final double linearAccelerationMagnitude;

        /** Direction of acceleration vector (radians, field-relative) */
        public final double linearAccelerationHeading;

        /** Angular rotation rate (rad/s, CCW positive) */
        public final double angularVelocity;

        /** Angular acceleration (rad/s²) */
        public final double angularAcceleration;

        /** Current center of rotation, robot-relative meters from robot center (0,0) */
        public final double[] centerOfRotation;           // [x, y]

        /** Rate of change of CoR position (m/s, robot-relative) */
        public final double[] centerOfRotationVelocity;   // [dx/dt, dy/dt]

        /** Rate of change of CoR velocity (m/s², robot-relative) */
        public final double[] centerOfRotationAcceleration; // [d²x/dt², d²y/dt²]

        public bucket(
                double linearVelocityMagnitude,
                double linearVelocityHeading,
                double linearAccelerationMagnitude,
                double linearAccelerationHeading,
                double angularVelocity,
                double angularAcceleration,
                double[] centerOfRotation,
                double[] centerOfRotationVelocity,
                double[] centerOfRotationAcceleration) {

            this.linearVelocityMagnitude        = linearVelocityMagnitude;
            this.linearVelocityHeading          = linearVelocityHeading;
            this.linearAccelerationMagnitude    = linearAccelerationMagnitude;
            this.linearAccelerationHeading      = linearAccelerationHeading;
            this.angularVelocity                = angularVelocity;
            this.angularAcceleration            = angularAcceleration;
            this.centerOfRotation               = centerOfRotation;
            this.centerOfRotationVelocity       = centerOfRotationVelocity;
            this.centerOfRotationAcceleration   = centerOfRotationAcceleration;
        }

        /** Safe zeroed bucket — returned before first loop runs */
        public static bucket zero() {
            return new bucket(0, 0, 0, 0, 0, 0,
                new double[]{0, 0}, new double[]{0, 0}, new double[]{0, 0});
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // The three buckets + timestamp
    // ─────────────────────────────────────────────────────────────────────────

    public final bucket encoderState;
    public final bucket imuState;
    public final bucket blendedState;

    /** FPGA timestamp of this snapshot (seconds) */
    public final double timestampSeconds;

    public driveOdometryState(
            bucket encoderState,
            bucket imuState,
            bucket blendedState,
            double timestampSeconds) {
        this.encoderState     = encoderState;
        this.imuState         = imuState;
        this.blendedState     = blendedState;
        this.timestampSeconds = timestampSeconds;
    }

    /** Safe zeroed state — returned before first loop completes */
    public static driveOdometryState zero() {
        return new driveOdometryState(bucket.zero(), bucket.zero(), bucket.zero(), 0.0);
    }
}