package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.AprilTagIgnore;
import frc.robot.constants.visionConstants;
import frc.robot.util.units;

import java.util.HashMap;
import java.util.Map;

/**
 * AprilTagFieldCalTab.java
 * PATH: src/main/java/frc/robot/subsystems/vision/AprilTagFieldCalTab.java
 *
 * Builds and updates the "Field Calibration" Shuffleboard tab.
 * Only talks to visionSubsystem — no direct access to internal vision objects.
 *
 * ─── TAB LAYOUT ──────────────────────────────────────────────────────────────
 *
 * TOP HALF — Camera feeds
 *   Left:  Front camera MJPEG stream (PhotonVision native annotation)
 *   Right: Rear camera MJPEG stream  (PhotonVision native annotation)
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

    // ── Shuffleboard tab ──────────────────────────────────────────────────────
    private final ShuffleboardTab tab;

    // ── Threshold input ───────────────────────────────────────────────────────
    private GenericEntry toleranceEntry;
    private double       toleranceInches = visionConstants.CALIBRATION_TOLERANCE_INCHES;

    // ── Bottom strip entries ──────────────────────────────────────────────────
    private GenericEntry blendPoseX, blendPoseY, blendPoseTheta;
    private GenericEntry frontPoseX, frontPoseY, frontPoseTheta;
    private GenericEntry rearPoseX,  rearPoseY,  rearPoseTheta;
    private GenericEntry farCornerX, farCornerY;
    private GenericEntry generateButton;

    // ── Per-tag offset input entries ──────────────────────────────────────────
    private final Map<Integer, GenericEntry[]> offsetInputEntries = new HashMap<>();

    // ── NetworkTables for Glass tag table ─────────────────────────────────────
    private final NetworkTable tagTable;

    // ─────────────────────────────────────────────────────────────────────────
    // Constructor
    // ─────────────────────────────────────────────────────────────────────────

    public AprilTagFieldCalTab(visionSubsystem vision) {
        this.vision   = vision;
        this.tab      = Shuffleboard.getTab("Field Calibration");
        this.tagTable = NetworkTableInstance.getDefault()
            .getTable("FieldCalibration").getSubTable("Tags");

        buildTopSection();
        buildBottomStrip();
        buildToleranceInput();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // update — called every loop from RobotContainer.updateCalibrationTab()
    // ─────────────────────────────────────────────────────────────────────────

    public void update() {
        toleranceInches = toleranceEntry.getDouble(visionConstants.CALIBRATION_TOLERANCE_INCHES);
        updateBottomStrip();
        updateTagTable();
        checkGenerateButton();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Top section — camera feeds
    // ─────────────────────────────────────────────────────────────────────────

    private void buildTopSection() {
        tab.addCamera("Front Camera",
                visionConstants.FRONT_CAMERA_NAME,
                visionConstants.FRONT_CAMERA_STREAM_URL)
            .withWidget(BuiltInWidgets.kCameraStream)
            .withSize(4, 3)
            .withPosition(0, 0)
            .withProperties(Map.of("showCrosshair", true, "showControls", false));

        tab.addCamera("Rear Camera",
                visionConstants.REAR_CAMERA_NAME,
                visionConstants.REAR_CAMERA_STREAM_URL)
            .withWidget(BuiltInWidgets.kCameraStream)
            .withSize(4, 3)
            .withPosition(4, 0)
            .withProperties(Map.of("showCrosshair", true, "showControls", false));
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Bottom strip
    // ─────────────────────────────────────────────────────────────────────────

    private void buildBottomStrip() {
        ShuffleboardLayout blendLayout = tab
            .getLayout("Blended POSE", BuiltInLayouts.kList)
            .withSize(2, 3).withPosition(0, 6);
        blendPoseX     = blendLayout.add("X (ft)", 0.0).getEntry();
        blendPoseY     = blendLayout.add("Y (ft)", 0.0).getEntry();
        blendPoseTheta = blendLayout.add("Theta (deg)", 0.0).getEntry();

        ShuffleboardLayout frontLayout = tab
            .getLayout("Front Cam POSE", BuiltInLayouts.kList)
            .withSize(2, 3).withPosition(2, 6);
        frontPoseX     = frontLayout.add("X (ft)", 0.0).getEntry();
        frontPoseY     = frontLayout.add("Y (ft)", 0.0).getEntry();
        frontPoseTheta = frontLayout.add("Theta (deg)", 0.0).getEntry();

        ShuffleboardLayout rearLayout = tab
            .getLayout("Rear Cam POSE", BuiltInLayouts.kList)
            .withSize(2, 3).withPosition(4, 6);
        rearPoseX     = rearLayout.add("X (ft)", 0.0).getEntry();
        rearPoseY     = rearLayout.add("Y (ft)", 0.0).getEntry();
        rearPoseTheta = rearLayout.add("Theta (deg)", 0.0).getEntry();

        ShuffleboardLayout cornerLayout = tab
            .getLayout("Field Far Corner", BuiltInLayouts.kList)
            .withSize(2, 3).withPosition(6, 6);
        farCornerX = cornerLayout.add("X (ft)", 0.0).getEntry();
        farCornerY = cornerLayout.add("Y (ft)", 0.0).getEntry();

        double[] farCorner = vision.getFarCornerFt();
        farCornerX.setDouble(farCorner[0]);
        farCornerY.setDouble(farCorner[1]);

        ShuffleboardLayout genLayout = tab
            .getLayout("Offsets File", BuiltInLayouts.kList)
            .withSize(2, 3).withPosition(8, 6);
        generateButton = genLayout.add("Generate File", false)
            .withWidget(BuiltInWidgets.kToggleButton).getEntry();
        genLayout.add("Last saved", "never").withWidget(BuiltInWidgets.kTextView);
    }

    private void buildToleranceInput() {
        toleranceEntry = tab
            .add("Cal Tolerance (in)", visionConstants.CALIBRATION_TOLERANCE_INCHES)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withSize(2, 1).withPosition(8, 5)
            .withProperties(Map.of("min", 0.1, "max", 3.0))
            .getEntry();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Bottom strip updates
    // ─────────────────────────────────────────────────────────────────────────

    private void updateBottomStrip() {
        robotPoseEstimate best = vision.getBestPose();
        if (best.isValid) {
            blendPoseX.setDouble(units.m_feet(best.pose.getX()));
            blendPoseY.setDouble(units.m_feet(best.pose.getY()));
            blendPoseTheta.setDouble(units.rad_deg(best.pose.getRotation().getZ()));
        } else {
            blendPoseX.setDouble(0); blendPoseY.setDouble(0); blendPoseTheta.setDouble(0);
        }

        robotPoseEstimate front = vision.getFrontPose();
        if (front.isValid) {
            frontPoseX.setDouble(units.m_feet(front.pose.getX()));
            frontPoseY.setDouble(units.m_feet(front.pose.getY()));
            frontPoseTheta.setDouble(units.rad_deg(front.pose.getRotation().getZ()));
        } else {
            frontPoseX.setDouble(0); frontPoseY.setDouble(0); frontPoseTheta.setDouble(0);
        }

        robotPoseEstimate rear = vision.getRearPose();
        if (rear.isValid) {
            rearPoseX.setDouble(units.m_feet(rear.pose.getX()));
            rearPoseY.setDouble(units.m_feet(rear.pose.getY()));
            rearPoseTheta.setDouble(units.rad_deg(rear.pose.getRotation().getZ()));
        } else {
            rearPoseX.setDouble(0); rearPoseY.setDouble(0); rearPoseTheta.setDouble(0);
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

        if (!offsetInputEntries.containsKey(tagId)) {
            ShuffleboardLayout offsetLayout = tab
                .getLayout("Tag " + tagId + " Offset Input", BuiltInLayouts.kList)
                .withSize(2, 3)
                .withPosition(10, tagId % 3);

            GenericEntry[] entries = new GenericEntry[4];
            entries[0] = offsetLayout.add("X offset (in)", 0.0).getEntry();
            entries[1] = offsetLayout.add("Y offset (in)", 0.0).getEntry();
            entries[2] = offsetLayout.add("Z offset (in)", 0.0).getEntry();
            entries[3] = offsetLayout.add("Yaw offset (deg)", 0.0).getEntry();
            offsetInputEntries.put(tagId, entries);
        }

        GenericEntry[] entries = offsetInputEntries.get(tagId);

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

        entries[0].setDouble(suggestX);
        entries[1].setDouble(suggestY);

        // Apply if user has entered non-zero values
        double xIn  = entries[0].getDouble(0);
        double yIn  = entries[1].getDouble(0);
        double zIn  = entries[2].getDouble(0);
        double yDeg = entries[3].getDouble(0);

        if (Math.abs(xIn) > 0.01 || Math.abs(yIn) > 0.01 ||
            Math.abs(zIn) > 0.01 || Math.abs(yDeg) > 0.01) {
            vision.setTagOffset(tagId, xIn, yIn, zIn, 0, 0, yDeg);
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Generate offsets file button
    // ─────────────────────────────────────────────────────────────────────────

    private void checkGenerateButton() {
        if (generateButton.getBoolean(false)) {
            vision.generateOffsetsOutput();
            generateButton.setBoolean(false);
            SmartDashboard.putString("FieldCal/Tab/GenerateStatus",
                "Generated — check Documents/FieldCalibration/");
        }
    }
}