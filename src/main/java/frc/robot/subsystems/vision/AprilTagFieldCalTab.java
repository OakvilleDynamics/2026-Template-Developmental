package frc.robot.subsystems.vision;

import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.tunable.TunableBoolean;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;
import frc.robot.constants.AprilTagIgnore;
import frc.robot.constants.visionConstants;
import frc.robot.util.units;

import java.util.HashMap;
import java.util.Map;

/**
 * AprilTagFieldCalTab.java
 * PATH: src/main/java/frc/robot/subsystems/vision/AprilTagFieldCalTab.java
 *
 * Publishes the data behind the "Field Calibration" Elastic tab.
 * Only talks to visionSubsystem — no direct access to internal vision objects.
 *
 * Widget placement, sizes and types are defined in the Elastic layout file, not
 * in code. Read-only values are published under /Telemetry/Field Calibration/
 * and operator inputs under /Tunables/Field Calibration/.
 *
 * ─── TAB LAYOUT (build in Elastic) ───────────────────────────────────────────
 *
 * TOP HALF — Camera feeds
 *   Left:  Front camera MJPEG stream (PhotonVision native annotation)
 *   Right: Rear camera MJPEG stream  (PhotonVision native annotation)
 *   Add as Camera Stream widgets from the /CameraPublisher topics.
 *
 * MIDDLE — Tag calibration table (published to NetworkTables for Glass)
 *   Three rows per tag: Cam1, Cam2, Blend
 *   Tags not visible to a camera: read pose shown as N/A
 *   Offset input fields pre-populated with recommended values
 *
 * BOTTOM STRIP — Five equally-spaced groups:
 *   Blended POSE x/y/theta (deg) |
 *   Front cam POSE x/y/theta (deg) |
 *   Rear cam POSE x/y/theta (deg) |
 *   Far corner x/y (ft) |
 *   [Generate Offsets File] button + threshold slider
 *
 * ─── GLASS TABLE ─────────────────────────────────────────────────────────────
 * Tag table data published to NetworkTables under /FieldCalibration/Tags/
 * Open Glass (WPILib tool) on second monitor to view the full scrollable table.
 * Launch: C:\Users\Public\wpilib\2026\tools\glass.exe
 *
 * ─── USAGE ───────────────────────────────────────────────────────────────────
 * Construct once in RobotContainer.
 * Call update() from RobotContainer.updateCalibrationTab() every loop.
 */
public class AprilTagFieldCalTab {

    // ── Single reference to all vision data ───────────────────────────────────
    private final visionSubsystem vision;

    // ── Dashboard key prefix (/Telemetry/... and /Tunables/...) ───────────────
    private static final String DASH_PREFIX = "Field Calibration/";

    // ── Threshold input ───────────────────────────────────────────────────────
    private final TunableDouble toleranceTunable;
    private double              toleranceInches = visionConstants.CALIBRATION_TOLERANCE_INCHES;

    // ── Generate offsets file button ──────────────────────────────────────────
    private final TunableBoolean generateButton;

    // ── Per-tag offset inputs: { X (in), Y (in), Z (in), Yaw (deg) } ──────────
    private final Map<Integer, TunableDouble[]> offsetInputs = new HashMap<>();

    // ── NetworkTables for Glass tag table ─────────────────────────────────────
    private final NetworkTable tagTable;

    // ─────────────────────────────────────────────────────────────────────────
    // Constructor
    // ─────────────────────────────────────────────────────────────────────────

    public AprilTagFieldCalTab(visionSubsystem vision) {
        this.vision   = vision;
        this.tagTable = NetworkTableInstance.getDefault()
            .getTable("FieldCalibration").getSubTable("Tags");

        toleranceTunable = Tunables.addDouble(
            DASH_PREFIX + "Cal Tolerance (in)", visionConstants.CALIBRATION_TOLERANCE_INCHES);
        generateButton = Tunables.addBoolean(DASH_PREFIX + "Offsets File/Generate File", false);
        Telemetry.log(DASH_PREFIX + "Offsets File/Last saved", "never");

        double[] farCorner = vision.getFarCornerFt();
        Telemetry.log(DASH_PREFIX + "Field Far Corner/X (ft)", farCorner[0]);
        Telemetry.log(DASH_PREFIX + "Field Far Corner/Y (ft)", farCorner[1]);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // update — called every loop from RobotContainer.updateCalibrationTab()
    // ─────────────────────────────────────────────────────────────────────────

    public void update() {
        toleranceInches = toleranceTunable.get();
        updateBottomStrip();
        updateTagTable();
        checkGenerateButton();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Bottom strip updates
    // ─────────────────────────────────────────────────────────────────────────

    private void updateBottomStrip() {
        publishPose("Blended POSE/",   vision.getBestPose());
        publishPose("Front Cam POSE/", vision.getFrontPose());
        publishPose("Rear Cam POSE/",  vision.getRearPose());
    }

    private void publishPose(String group, robotPoseEstimate estimate) {
        String prefix = DASH_PREFIX + group;
        if (estimate.isValid) {
            Telemetry.log(prefix + "X (ft)",      units.m_feet(estimate.pose.getX()));
            Telemetry.log(prefix + "Y (ft)",      units.m_feet(estimate.pose.getY()));
            Telemetry.log(prefix + "Theta (deg)", units.rad_deg(estimate.pose.getRotation().getZ()));
        } else {
            Telemetry.log(prefix + "X (ft)",      0.0);
            Telemetry.log(prefix + "Y (ft)",      0.0);
            Telemetry.log(prefix + "Theta (deg)", 0.0);
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Tag table — published to NetworkTables for Glass
    // ─────────────────────────────────────────────────────────────────────────

    private void updateTagTable() {
        var readings = vision.getTagReadings();

        readings.forEach((tagId, byCamera) -> {
            var frontReading = byCamera.get(visionConstants.FRONT_CAMERA_NAME);
            var rearReading  = byCamera.get(visionConstants.REAR_CAMERA_NAME);
            if (frontReading == null || rearReading == null) return;

            String frontRec = computeRecommendation(frontReading, rearReading, "front");
            String rearRec  = computeRecommendation(rearReading,  frontReading, "rear");
            String blendRec = computeBlendRecommendation(
                frontReading, rearReading, frontRec, rearRec);

            boolean ignored    = AprilTagIgnore.shouldIgnore(tagId);
            String  ignoredStr = ignored
                ? "YES (" + AprilTagIgnore.getIgnoreMap().getOrDefault(tagId, "?") + ")"
                : "no";

            NetworkTable tagEntry = tagTable.getSubTable("Tag_" + tagId);

            writeTagRow(tagEntry.getSubTable("Cam1_Front"),
                frontReading, frontRec, ignoredStr, tagId);
            writeTagRow(tagEntry.getSubTable("Cam2_Rear"),
                rearReading, rearRec, ignoredStr, tagId);
            writeBlendRow(tagEntry.getSubTable("Blend"),
                frontReading, rearReading, blendRec, ignoredStr);

            updateOffsetInputs(tagId, frontRec, frontReading, rearReading);
        });
    }

    private void writeTagRow(NetworkTable row, visionSubsystem.tagReading reading,
                              String recommendation, String ignored, int tagId) {
        row.getEntry("TagID").setInteger(tagId);
        row.getEntry("Ignored").setString(ignored);
        row.getEntry("Camera").setString(reading.cameraName());
        row.getEntry("Visible").setBoolean(reading.visible());
        row.getEntry("Baseline X (ft)").setDouble(
            units.m_feet(reading.baselinePose().getX()));
        row.getEntry("Baseline Y (ft)").setDouble(
            units.m_feet(reading.baselinePose().getY()));
        row.getEntry("Recommendation").setString(recommendation);

        if (reading.visible()) {
            row.getEntry("Read X (ft)").setDouble(
                reading.readPose() != null ? units.m_feet(reading.readPose().getX()) : 0);
            row.getEntry("Read Y (ft)").setDouble(
                reading.readPose() != null ? units.m_feet(reading.readPose().getY()) : 0);
            row.getEntry("Delta X (in)").setDouble(reading.deltaXInches());
            row.getEntry("Delta Y (in)").setDouble(reading.deltaYInches());
            row.getEntry("Delta Z (in)").setDouble(reading.deltaZInches());
            row.getEntry("Delta Yaw (deg)").setDouble(reading.deltaYawDeg());
            row.getEntry("Ambiguity").setDouble(reading.ambiguity());
        } else {
            row.getEntry("Read X (ft)").setString("N/A");
            row.getEntry("Read Y (ft)").setString("N/A");
            row.getEntry("Delta X (in)").setString("N/A");
            row.getEntry("Delta Y (in)").setString("N/A");
            row.getEntry("Delta Z (in)").setString("N/A");
            row.getEntry("Delta Yaw (deg)").setString("N/A");
            row.getEntry("Ambiguity").setString("N/A");
        }
    }

    private void writeBlendRow(NetworkTable row,
                                visionSubsystem.tagReading front,
                                visionSubsystem.tagReading rear,
                                String recommendation, String ignored) {
        row.getEntry("Camera").setString("Blend");
        row.getEntry("Ignored").setString(ignored);
        row.getEntry("Recommendation").setString(recommendation);

        if (front.visible() && rear.visible()) {
            row.getEntry("Delta X (in)").setDouble(
                (front.deltaXInches() + rear.deltaXInches()) / 2.0);
            row.getEntry("Delta Y (in)").setDouble(
                (front.deltaYInches() + rear.deltaYInches()) / 2.0);
            row.getEntry("Ambiguity").setDouble(
                Math.min(front.ambiguity(), rear.ambiguity()));
        } else if (front.visible()) {
            row.getEntry("Delta X (in)").setDouble(front.deltaXInches());
            row.getEntry("Delta Y (in)").setDouble(front.deltaYInches());
            row.getEntry("Ambiguity").setDouble(front.ambiguity());
        } else if (rear.visible()) {
            row.getEntry("Delta X (in)").setDouble(rear.deltaXInches());
            row.getEntry("Delta Y (in)").setDouble(rear.deltaYInches());
            row.getEntry("Ambiguity").setDouble(rear.ambiguity());
        } else {
            row.getEntry("Delta X (in)").setString("N/A");
            row.getEntry("Delta Y (in)").setString("N/A");
            row.getEntry("Ambiguity").setString("N/A");
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Recommendation logic
    // ─────────────────────────────────────────────────────────────────────────

    private String computeRecommendation(
            visionSubsystem.tagReading thisCamera,
            visionSubsystem.tagReading otherCamera,
            String whichCam) {

        if (!thisCamera.visible()) return "Tag not visible";

        double thisMag  = magnitude(thisCamera.deltaXInches(), thisCamera.deltaYInches());
        double otherMag = otherCamera.visible()
            ? magnitude(otherCamera.deltaXInches(), otherCamera.deltaYInches()) : -1;

        if (thisMag <= toleranceInches) return "Within tolerance — no action";

        boolean bothVisible = otherMag >= 0;

        if (bothVisible) {
            double disagreement = Math.abs(thisMag - otherMag);
            if (disagreement > toleranceInches) {
                return String.format("Check %s cam transform (%.1f\" vs %.1f\")",
                    whichCam, thisMag, otherMag);
            } else {
                return String.format(
                    "Apply field offset (both cams off by ~%.1f\")", thisMag);
            }
        } else {
            if (thisMag > toleranceInches * 3) {
                return String.format(
                    "Large error (%.1f\") — check %s cam transform first",
                    thisMag, whichCam);
            } else {
                return String.format(
                    "Consider field offset (%.1f\") — verify with other cam", thisMag);
            }
        }
    }

    private String computeBlendRecommendation(
            visionSubsystem.tagReading front,
            visionSubsystem.tagReading rear,
            String frontRec, String rearRec) {

        if (!front.visible() && !rear.visible()) return "No cameras see this tag";
        if (!front.visible()) return rearRec;
        if (!rear.visible())  return frontRec;

        double frontMag = magnitude(front.deltaXInches(), front.deltaYInches());
        double rearMag  = magnitude(rear.deltaXInches(),  rear.deltaYInches());
        return frontMag >= rearMag ? frontRec : rearRec;
    }

    private double magnitude(double x, double y) { return Math.hypot(x, y); }

    // ─────────────────────────────────────────────────────────────────────────
    // Offset input fields
    // ─────────────────────────────────────────────────────────────────────────

    private void updateOffsetInputs(int tagId, String recommendation,
                                     visionSubsystem.tagReading front,
                                     visionSubsystem.tagReading rear) {
        if (!recommendation.contains("offset")) return;

        TunableDouble[] entries = offsetInputs.computeIfAbsent(tagId, id -> {
            String prefix = DASH_PREFIX + "Tag " + id + " Offset Input/";
            return new TunableDouble[] {
                Tunables.addDouble(prefix + "X offset (in)",    0.0),
                Tunables.addDouble(prefix + "Y offset (in)",    0.0),
                Tunables.addDouble(prefix + "Z offset (in)",    0.0),
                Tunables.addDouble(prefix + "Yaw offset (deg)", 0.0),
            };
        });

        // Pre-populate with negated average delta as suggested correction
        double suggestX = 0, suggestY = 0;
        if (front.visible() && rear.visible()) {
            suggestX = -(front.deltaXInches() + rear.deltaXInches()) / 2.0;
            suggestY = -(front.deltaYInches() + rear.deltaYInches()) / 2.0;
        } else if (front.visible()) {
            suggestX = -front.deltaXInches();
            suggestY = -front.deltaYInches();
        } else if (rear.visible()) {
            suggestX = -rear.deltaXInches();
            suggestY = -rear.deltaYInches();
        }

        entries[0].set(suggestX);
        entries[1].set(suggestY);

        // Apply if user has entered non-zero values
        double xIn  = entries[0].get();
        double yIn  = entries[1].get();
        double zIn  = entries[2].get();
        double yDeg = entries[3].get();

        if (Math.abs(xIn) > 0.01 || Math.abs(yIn) > 0.01 ||
            Math.abs(zIn) > 0.01 || Math.abs(yDeg) > 0.01) {
            vision.setTagOffset(tagId, xIn, yIn, zIn, 0, 0, yDeg);
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Generate offsets file button
    // ─────────────────────────────────────────────────────────────────────────

    private void checkGenerateButton() {
        if (generateButton.get()) {
            vision.generateOffsetsOutput();
            generateButton.set(false);
            Telemetry.log("FieldCal/Tab/GenerateStatus",
                "Generated — check Documents/FieldCalibration/");
        }
    }
}