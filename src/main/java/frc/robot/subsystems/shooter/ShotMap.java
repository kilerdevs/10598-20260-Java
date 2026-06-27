package frc.robot.subsystems.shooter;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Loads the shot map from {@code deploy/shotmap.json} and provides linear interpolation of RPM and
 * angle based on distance to the target.
 */
public class ShotMap {

    private final List<ShotEntry> entries = new ArrayList<>();

    public ShotMap() {
        load();
    }

    private void load() {
        File file = new File(Filesystem.getDeployDirectory(), "shotmap.json");
        if (!file.exists()) {
            DriverStation.reportError("[ShotMap] shotmap.json not found", false);
            return;
        }
        try {
            ObjectMapper mapper = new ObjectMapper();
            JsonNode root = mapper.readTree(file);
            for (JsonNode node : root) {
                entries.add(
                    new ShotEntry(
                        node.get("distance").asDouble(),
                        node.get("rpm").asDouble(),
                        node.get("angle").asDouble()));
            }
            entries.sort(Comparator.comparingDouble(e -> e.distance));
            System.out.println("[ShotMap] Loaded " + entries.size() + " entries from shotmap.json");
        } catch (Exception e) {
            DriverStation.reportError("[ShotMap] Failed to parse: " + e.getMessage(), false);
        }
    }

    /** Returns the interpolated RPM for the given distance to target (meters). */
    public double getRPM(double distance) {
        return interpolate(distance, true);
    }

    /** Returns the interpolated angle percent for the given distance to target (meters). */
    public double getAngle(double distance) {
        return interpolate(distance, false);
    }

    private double interpolate(double distance, boolean useRPM) {
        if (entries.isEmpty()) return useRPM ? 4500 : 20;

        if (distance <= entries.get(0).distance) {
            return useRPM ? entries.get(0).rpm : entries.get(0).angle;
        }
        if (distance >= entries.get(entries.size() - 1).distance) {
            return useRPM
                ? entries.get(entries.size() - 1).rpm
                : entries.get(entries.size() - 1).angle;
        }

        for (int i = 0; i < entries.size() - 1; i++) {
            ShotEntry low = entries.get(i);
            ShotEntry high = entries.get(i + 1);
            if (distance >= low.distance && distance <= high.distance) {
                double t = (distance - low.distance) / (high.distance - low.distance);
                return useRPM
                    ? low.rpm + t * (high.rpm - low.rpm)
                    : low.angle + t * (high.angle - low.angle);
            }
        }
        return useRPM ? 4500 : 20;
    }

    private static class ShotEntry {
        final double distance;
        final double rpm;
        final double angle;

        ShotEntry(double distance, double rpm, double angle) {
            this.distance = distance;
            this.rpm = rpm;
            this.angle = angle;
        }
    }
}
