package frc.lib;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Pattern;
import java.util.stream.Stream;

import org.junit.jupiter.api.Test;

/**
 * ArchitectureTest.java
 * PATH: src/test/java/frc/lib/ArchitectureTest.java
 *
 * Enforces the lib / robot boundary so the layout cannot quietly erode:
 *
 *   1. Nothing in frc.lib imports frc.robot — lib code must be copyable into
 *      any robot project without this robot's constants or subsystems.
 *   2. Vendor motor/sensor libraries (CTRE, REV, ThriftyBot) are imported only
 *      under frc.lib.hardware — everything above talks to MotorIO /
 *      AbsoluteEncoderIO, which is what makes a vendor swap a one-value change.
 *
 * Pure file scan — no HAL, runs in milliseconds.
 */
class ArchitectureTest {

    private static final Path LIB_ROOT      = Paths.get("src/main/java/frc/lib");
    private static final Path MAIN_ROOT     = Paths.get("src/main/java/frc");
    private static final Path HARDWARE_ROOT = LIB_ROOT.resolve("hardware");

    private static final Pattern ROBOT_IMPORT  =
        Pattern.compile("^\\s*import\\s+(static\\s+)?frc\\.robot\\.");
    private static final Pattern VENDOR_IMPORT =
        Pattern.compile("^\\s*import\\s+(static\\s+)?com\\.(ctre|revrobotics|thethriftybot)\\.");
    private static final Pattern VENDOR_FQN    =
        Pattern.compile("\\bcom\\.(ctre|revrobotics|thethriftybot)\\.");

    @Test
    void libNeverImportsRobot() throws IOException {
        List<String> violations = scan(LIB_ROOT, ROBOT_IMPORT, path -> true);
        assertTrue(violations.isEmpty(),
            "frc.lib must not depend on frc.robot:\n  " + String.join("\n  ", violations));
    }

    @Test
    void vendorLibrariesOnlyInHardwareLayer() throws IOException {
        // The robot's own drivetrain may still use vendor devices directly (e.g. the
        // Pigeon IMU in swerveDrive); the rule applies to everything in frc.lib.
        List<String> violations = scan(LIB_ROOT, VENDOR_FQN, path -> !path.startsWith(HARDWARE_ROOT));
        assertTrue(violations.isEmpty(),
            "Vendor APIs are only allowed under frc/lib/hardware:\n  " + String.join("\n  ", violations));
    }

    @Test
    void mechanismsUseNoVendorImports() throws IOException {
        // Mechanism-style robot code (anything outside the drivetrain's IMU and
        // Robot.java's SignalLogger) goes through MotorIO too.
        List<String> violations = scan(MAIN_ROOT, VENDOR_IMPORT, path ->
            !path.startsWith(HARDWARE_ROOT)
            && !path.toString().contains("subsystems/swerveDrive")
            && !path.getFileName().toString().equals("Robot.java"));
        assertTrue(violations.isEmpty(),
            "Use MotorIO instead of vendor imports:\n  " + String.join("\n  ", violations));
    }

    private interface PathFilter { boolean include(Path path); }

    private static List<String> scan(Path root, Pattern pattern, PathFilter filter) throws IOException {
        List<String> violations = new ArrayList<>();
        try (Stream<Path> files = Files.walk(root)) {
            for (Path file : (Iterable<Path>) files.filter(p -> p.toString().endsWith(".java"))::iterator) {
                if (!filter.include(file)) continue;
                List<String> lines = Files.readAllLines(file);
                for (int i = 0; i < lines.size(); i++) {
                    String line = lines.get(i);
                    if (line.trim().startsWith("*") || line.trim().startsWith("//")) continue;
                    if (pattern.matcher(line).find()) {
                        violations.add(file + ":" + (i + 1) + "  " + line.trim());
                    }
                }
            }
        }
        return violations;
    }
}
