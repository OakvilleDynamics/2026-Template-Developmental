package frc.robot.util;

import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.telemetry.Telemetry;
import frc.robot.constants.AprilTagIgnore;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

/**
 * AprilTagFieldCal.java
 * PATH: src/main/java/frc/robot/util/AprilTagFieldCal.java
 *
 * Per-field AprilTag offset system and field geometry reference.
 * Wraps the WPILib AprilTagFieldLayout and applies per-tag corrections.
 *
 * ─── CONSTRUCTOR ─────────────────────────────────────────────────────────────
 * Receives a Fields enum value from RobotContainer via visionSubsystem.
 * This is the single point where game year is bound to the layout.
 *
 * ─── OFFSET SYSTEM ───────────────────────────────────────────────────────────
 * setTagOffset() registers a correction for a specific tag.
 * getCalibratedLayout() returns a layout with all corrections applied.
 * Call rebuildLayout() after changing offsets — visionSubsystem handles this.
 *
 * ─── TAG IGNORE INTEGRATION ──────────────────────────────────────────────────
 * getCalibratedLayout() automatically excludes tags that AprilTagIgnore marks
 * as ignored for the current alliance. The pose estimators never see
 * ignored tags, so they can't contribute to or corrupt pose estimates.
 *
 * ─── OFFSET UNITS ────────────────────────────────────────────────────────────
 * setTagOffset() accepts INCHES and DEGREES — units your team can measure.
 * Stored and applied internally in meters/radians.
 *
 * ─── IMPORTANT NOTE ──────────────────────────────────────────────────────────
 * Offsets should be small (≤ 2 inches, ≤ 2 degrees).
 * Larger corrections usually mean the camera transform is wrong, not the field.
 * Fix visionConstants camera transforms before applying large field offsets.
 */
public class AprilTagFieldCal {

    // ── Field layout ──────────────────────────────────────────────────────────
    private final Field baseLayout;
    private Field       calibratedLayout;

    // ── Per-tag offsets: tag ID → Transform3d correction ─────────────────────
    private final Map<Integer, Transform3d> tagOffsets = new HashMap<>();

    // ─────────────────────────────────────────────────────────────────────────
    // Constructor
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * @param gameYearField  Fields enum for the current game year.
     *                       Passed from RobotContainer via visionSubsystem.
     *                       Example: Fields.DEFAULT_FIELD (2026)
     *                                Fields.FRC_2025_REEFSCAPE_WELDED
     */
    public AprilTagFieldCal(Fields gameYearField) {
        baseLayout       = Field.loadField(gameYearField);
        calibratedLayout = baseLayout;

        Telemetry.log("Field/Length (ft)", getFieldLengthFt());
        Telemetry.log("Field/Width (ft)",  getFieldWidthFt());
        double[] far = getFarCornerFt();
        Telemetry.log("Field/Far Corner X (ft)", far[0]);
        Telemetry.log("Field/Far Corner Y (ft)", far[1]);
        Telemetry.log("Field/Tag Ignore Rules",  AprilTagIgnore.getSummary());
        Telemetry.log("Field/Total Tags",        baseLayout.getTags().size());
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Offset API
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Registers a position/orientation correction for a specific AprilTag.
     * Call rebuildLayout() after all offsets are set to apply them.
     *
     * @param tagId     AprilTag ID
     * @param xInches   X correction in INCHES (positive = toward far field wall)
     * @param yInches   Y correction in INCHES (positive = toward field left)
     * @param zInches   Z correction in INCHES (positive = upward)
     * @param rollDeg   Roll correction in DEGREES
     * @param pitchDeg  Pitch correction in DEGREES
     * @param yawDeg    Yaw correction in DEGREES
     */
    public void setTagOffset(int tagId,
                              double xInches, double yInches, double zInches,
                              double rollDeg,  double pitchDeg, double yawDeg) {
        tagOffsets.put(tagId, new Transform3d(
            new Translation3d(
                units.inches_m(xInches),
                units.inches_m(yInches),
                units.inches_m(zInches)
            ),
            new Rotation3d(
                units.deg_rad(rollDeg),
                units.deg_rad(pitchDeg),
                units.deg_rad(yawDeg)
            )
        ));
        Telemetry.log("FieldCal/Tag " + tagId + " Offset",
            String.format("x=%.2f\" y=%.2f\" z=%.2f\" yaw=%.1f°",
                xInches, yInches, zInches, yawDeg));
    }

    /** Removes the offset for a specific tag, reverting to WPILib baseline */
    public void clearTagOffset(int tagId) {
        tagOffsets.remove(tagId);
        Telemetry.log("FieldCal/Tag " + tagId + " Offset", "cleared");
    }

    /** Removes all offsets — full reset to WPILib baseline */
    public void clearAllOffsets() {
        tagOffsets.clear();
        Telemetry.log("FieldCal/Status", "All offsets cleared");
    }

    /**
     * Returns current offsets as an unmodifiable map.
     * Used by AprilTagFieldCalTab and the offset file generator.
     */
    public Map<Integer, Transform3d> getTagOffsets() {
        return Collections.unmodifiableMap(tagOffsets);
    }

    /**
     * Rebuilds the calibrated layout with current offsets and ignore rules.
     * Must be called after any setTagOffset() or clearTagOffset() call,
     * and after alliance assignment changes (AprilTagIgnore is dynamic).
     * visionSubsystem calls this automatically.
     */
    public void rebuildLayout() {
        if (tagOffsets.isEmpty() && AprilTagIgnore.getIgnoreMap().isEmpty()) {
            calibratedLayout = baseLayout;
            return;
        }

        List<FieldTag> correctedTags = new ArrayList<>();

        for (FieldTag tag : baseLayout.getTags()) {
            int tagId = tag.getID();

            if (AprilTagIgnore.shouldIgnore(tagId)) {
                Telemetry.log("FieldCal/Tag " + tagId + " Status", "IGNORED");
                continue;
            }

            if (tagOffsets.containsKey(tagId)) {
                Pose3d corrected = tag.getPose().transformBy(tagOffsets.get(tagId));
                correctedTags.add(new FieldTag(tagId, corrected));
            } else {
                correctedTags.add(tag);
            }
        }

        calibratedLayout = new Field(
            baseLayout.name, baseLayout.season, baseLayout.game, baseLayout.fieldImage,
            baseLayout.getFieldLength(),
            baseLayout.getFieldWidth(),
            baseLayout.program,
            correctedTags
        );

        Telemetry.log("FieldCal/Active Tags",     correctedTags.size());
        Telemetry.log("FieldCal/Total Tags",      baseLayout.getTags().size());
        Telemetry.log("FieldCal/Offsets Applied", tagOffsets.size());
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Layout access
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Returns the calibrated layout with all offsets and ignore rules applied.
     * This is the layout PhotonPoseEstimator uses — same data the robot acts on.
     */
    public Field getCalibratedLayout() { return calibratedLayout; }

    /**
     * Returns the raw WPILib baseline pose for a tag (no offsets applied).
     * Used by AprilTagFieldCalTab to compute deltas.
     */
    public Optional<Pose3d> getBaselineTagPose(int tagId) {
        return baseLayout.getTagPose(tagId);
    }

    /**
     * Returns all tags from the base layout including ignored ones.
     * Used by AprilTagFieldCalTab to build the full table.
     */
    public List<FieldTag> getAllBaseTags() {
        return Collections.unmodifiableList(baseLayout.getTags());
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Field geometry
    // ─────────────────────────────────────────────────────────────────────────

    public double getFieldLengthM()  { return baseLayout.getFieldLength(); }
    public double getFieldWidthM()   { return baseLayout.getFieldWidth(); }
    public double getFieldLengthFt() { return units.m_feet(baseLayout.getFieldLength()); }
    public double getFieldWidthFt()  { return units.m_feet(baseLayout.getFieldWidth()); }

    /**
     * Returns the far corner of the field from (0,0) in feet.
     * (0,0) = blue alliance wall, left corner (driver's perspective).
     *
     * @return double[2] { X far (ft), Y far (ft) }
     */
    public double[] getFarCornerFt() {
        return new double[]{
            units.m_feet(baseLayout.getFieldLength()),
            units.m_feet(baseLayout.getFieldWidth())
        };
    }
}