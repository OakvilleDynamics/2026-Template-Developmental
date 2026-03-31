package frc.robot.constants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

/**
 * AprilTagIgnore.java
 * PATH: src/main/java/frc/robot/constants/AprilTagIgnore.java
 *
 * Standalone file defining which AprilTags to ignore during pose estimation.
 *
 * ─── HOW TO USE ───────────────────────────────────────────────────────────────
 * Edit the IGNORED_TAGS map below and redeploy.
 * No other files need to change.
 *
 * This file is intentionally simple and self-contained so it can be
 * edited quickly at competition without risk of breaking anything else.
 *
 * ─── CONDITION VALUES ────────────────────────────────────────────────────────
 * "always" — ignore this tag regardless of alliance
 * "blue"   — ignore this tag when robot is on BLUE alliance
 * "red"    — ignore this tag when robot is on RED alliance
 *
 * ─── ALLIANCE DETECTION ──────────────────────────────────────────────────────
 * Alliance color is read from DriverStation.getAlliance() at runtime.
 * If the alliance is not yet assigned by FMS (Optional is empty — common
 * before a match is loaded), ALL tags are used. This is the safe default.
 *
 * ─── EXAMPLE ENTRIES ─────────────────────────────────────────────────────────
 * Tag 1 on the opposing alliance wall, always causing noise:
 *   IGNORED_TAGS.put(1, "always");
 *
 * Tags on the red alliance side (ignore when we are red, to avoid
 * self-referential pose confusion near our own structures):
 *   IGNORED_TAGS.put(6, "red");
 *   IGNORED_TAGS.put(7, "red");
 *
 * ─── 2026 REBUILT TAG LAYOUT REFERENCE ───────────────────────────────────────
 * Update this comment block each season with the tag ID → location mapping
 * so editors know what they're ignoring without needing to look it up.
 *
 * Tags 1–9:   Red alliance structures (coral station, reef, processor, barge)
 * Tags 10–22: Blue alliance structures (coral station, reef, processor, barge)
 * (Exact mapping TBD — update once 2026 field layout is finalized)
 *
 * TODO: Fill in tag descriptions once 2026 layout is published.
 */
public final class AprilTagIgnore {

    /**
     * Map of tag ID → ignore condition.
     *
     * ─── EDIT THIS SECTION AT COMPETITION ────────────────────────────────────
     * Add entries here for tags causing problems. Remove them when resolved.
     * Valid conditions: "always", "blue", "red"
     */
    private static final Map<Integer, String> IGNORED_TAGS = new HashMap<>() {{
        // Example — uncomment and edit as needed:
        // put(1, "always");   // tag 1: consistently bad readings on this field
        // put(6, "red");      // tag 6: ignore when on red alliance
        // put(13, "blue");    // tag 13: ignore when on blue alliance
    }};

    // ─────────────────────────────────────────────────────────────────────────
    // DO NOT EDIT BELOW THIS LINE
    // ─────────────────────────────────────────────────────────────────────────

    private AprilTagIgnore() {}

    /**
     * Returns true if the given tag ID should be ignored given the current
     * alliance assignment.
     *
     * When alliance is unknown (FMS not connected, pre-match),
     * returns false for all tags — use everything until we know better.
     *
     * @param tagId  AprilTag ID to check
     * @return       true if this tag should be excluded from pose estimation
     */
    public static boolean shouldIgnore(int tagId) {
        if (!IGNORED_TAGS.containsKey(tagId)) return false;

        String condition = IGNORED_TAGS.get(tagId);

        if (condition.equals("always")) return true;

        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return false;

        if (condition.equals("blue") && alliance.get() == Alliance.Blue) return true;
        if (condition.equals("red")  && alliance.get() == Alliance.Red)  return true;

        return false;
    }

    /**
     * Returns the full ignore map — used by AprilTagFieldCalTab to display
     * which tags are currently being filtered in the calibration table.
     */
    public static Map<Integer, String> getIgnoreMap() {
        return Collections.unmodifiableMap(IGNORED_TAGS);
    }

    /**
     * Returns a human-readable summary of current ignore rules.
     * Published to SmartDashboard at startup for visibility.
     */
    public static String getSummary() {
        if (IGNORED_TAGS.isEmpty()) return "No tags ignored";
        StringBuilder sb = new StringBuilder();
        IGNORED_TAGS.forEach((id, cond) ->
            sb.append("Tag ").append(id).append(": ").append(cond).append("  "));
        return sb.toString().trim();
    }
}