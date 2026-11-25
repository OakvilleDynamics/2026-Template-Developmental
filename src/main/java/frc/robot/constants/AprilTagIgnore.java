package frc.robot.constants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

/**
 * Standalone file defining which AprilTags to ignore during pose estimation.
 *
 * <h2>How to use</h2>
 *
 * <p>Edit the IGNORED_TAGS map below and redeploy.
 * No other files need to change.
 *
 * <p>This file is intentionally simple and self-contained so it can be
 * edited quickly at competition without risk of breaking anything else.
 *
 * <h2>Condition values</h2>
 *
 * <ul>
 *   <li><b>"always"</b> — ignore this tag regardless of alliance</li>
 *   <li><b>"blue"</b> — ignore this tag when robot is on BLUE alliance</li>
 *   <li><b>"red"</b> — ignore this tag when robot is on RED alliance</li>
 * </ul>
 *
 * <h2>Alliance detection</h2>
 *
 * <p>Alliance color is read from DriverStation.getAlliance() at runtime.
 * If the alliance is not yet assigned by FMS (Optional is empty — common
 * before a match is loaded), ALL tags are used. This is the safe default.
 *
 * <h2>Example entries</h2>
 *
 * <p>Tag 1 on the opposing alliance wall, always causing noise:
 *
 * <pre>{@code
 * IGNORED_TAGS.put(1, "always");
 * }</pre>
 *
 * <p>Tags on the red alliance side (ignore when we are red, to avoid
 * self-referential pose confusion near our own structures):
 *
 * <pre>{@code
 * IGNORED_TAGS.put(6, "red");
 * IGNORED_TAGS.put(7, "red");
 * }</pre>
 *
 * <h2>2026 REBUILT tag layout reference</h2>
 *
 * <p>Update this comment block each season with the tag ID → location mapping
 * so editors know what they're ignoring without needing to look it up.
 *
 * <ul>
 *   <li><b>Tags 1–9</b>: Red alliance structures (coral station, reef, processor, barge)</li>
 *   <li><b>Tags 10–22</b>: Blue alliance structures (coral station, reef, processor, barge)</li>
 * </ul>
 *
 * <p>(Exact mapping TBD — update once 2026 field layout is finalized)
 *
 * <p>TODO: Fill in tag descriptions once 2026 layout is published.
 */
public final class AprilTagIgnore {

    /**
     * Map of tag ID → ignore condition.
     *
     * <h4>Edit this section at competition</h4>
     *
     * <p>Add entries here for tags causing problems. Remove them when resolved.
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
     * <p>When alliance is unknown (FMS not connected, pre-match),
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
     * Published to the dashboard at startup for visibility.
     */
    public static String getSummary() {
        if (IGNORED_TAGS.isEmpty()) return "No tags ignored";
        StringBuilder sb = new StringBuilder();
        IGNORED_TAGS.forEach((id, cond) ->
            sb.append("Tag ").append(id).append(": ").append(cond).append("  "));
        return sb.toString().trim();
    }
}