package frc.robot.subsystems.zones;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Monitors the robot's field position and triggers registered commands when the robot enters or
 * exits predefined zones.
 *
 * <p>Zones are loaded from {@code deploy/zones.json} and can be alliance-relative (auto-mirrored
 * for red). Actions are string keys mapped to commands via {@link #registerEnterAction} and {@link
 * #registerExitAction}.
 */
public class ZoneActionSubsystem extends SubsystemBase {

  private final Supplier<Pose2d> poseSupplier;
  private final List<FieldZone> zones = new ArrayList<>();

  // Action key → command factory
  private final Map<String, Supplier<Command>> enterActions = new HashMap<>();
  private final Map<String, Supplier<Command>> exitActions = new HashMap<>();

  // Currently running enter-action commands, keyed by zone name
  private final Map<String, Command> runningEnterCommands = new HashMap<>();

  // Tracks which zones the robot is currently inside
  private final Set<String> activeZones = new HashSet<>();

  private double fieldLength = 16.54;

  public ZoneActionSubsystem(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
    loadZones();
  }

  // ── Zone loading ──────────────────────────────────────────────────────────

  private void loadZones() {
    File file = new File(Filesystem.getDeployDirectory(), "zones.json");

    if (!file.exists()) {
      DriverStation.reportError(
          "[ZoneAction] zones.json not found at: " + file.getAbsolutePath(), false);
      return;
    }

    try {
      ObjectMapper mapper = new ObjectMapper();
      JsonNode root = mapper.readTree(file);

      fieldLength = root.get("fieldLength").asDouble(16.54);

      JsonNode zonesArray = root.get("zones");
      if (zonesArray == null || !zonesArray.isArray()) {
        DriverStation.reportError("[ZoneAction] zones.json has no 'zones' array", false);
        return;
      }

      for (JsonNode zoneNode : zonesArray) {
        String name = zoneNode.get("name").asText();
        String action = zoneNode.get("action").asText();
        boolean allianceRelative =
            zoneNode.has("allianceRelative") && zoneNode.get("allianceRelative").asBoolean();

        JsonNode verts = zoneNode.get("vertices");
        Translation2d[] vertices = new Translation2d[verts.size()];
        for (int i = 0; i < verts.size(); i++) {
          JsonNode v = verts.get(i);
          vertices[i] = new Translation2d(v.get("x").asDouble(), v.get("y").asDouble());
        }

        zones.add(new FieldZone(name, action, allianceRelative, vertices, fieldLength));
      }

      System.out.println("[ZoneAction] Loaded " + zones.size() + " zones from zones.json");
    } catch (Exception e) {
      DriverStation.reportError(
          "[ZoneAction] Failed to parse zones.json: " + e.getMessage(), false);
    }
  }

  // ── Action registration ───────────────────────────────────────────────────

  /**
   * Register a command factory that runs when the robot enters a zone with the given action key.
   * The command is started on entry and cancelled on exit.
   */
  public void registerEnterAction(String actionKey, Supplier<Command> commandFactory) {
    enterActions.put(actionKey, commandFactory);
  }

  /**
   * Register a command factory that runs once when the robot exits a zone with the given action
   * key.
   */
  public void registerExitAction(String actionKey, Supplier<Command> commandFactory) {
    exitActions.put(actionKey, commandFactory);
  }

  // ── Periodic ──────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    Pose2d pose = poseSupplier.get();
    Translation2d position = pose.getTranslation();
    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

    Set<String> nowInside = new HashSet<>();

    for (FieldZone zone : zones) {
      boolean inside = zone.contains(position, isRed);
      String zoneName = zone.getName();

      if (inside) {
        nowInside.add(zoneName);
      }

      // ── Enter trigger ──────────────────────────────────────────────────
      if (inside && !activeZones.contains(zoneName)) {
        onZoneEnter(zone);
      }

      // ── Exit trigger ───────────────────────────────────────────────────
      if (!inside && activeZones.contains(zoneName)) {
        onZoneExit(zone);
      }
    }

    activeZones.clear();
    activeZones.addAll(nowInside);

    // ── Hub distance ──────────────────────────────────────────────────────
    double hubX = isRed ? fieldLength - 4.6 : 4.6;
    double hubY = 4.0;
    double hubDistance = Math.hypot(hubX - position.getX(), hubY - position.getY());
    Logger.recordOutput("ZoneAction/HubDistanceM", hubDistance);

    // ── Logging ────────────────────────────────────────────────────────────
    Logger.recordOutput("ZoneAction/RobotX", position.getX());
    Logger.recordOutput("ZoneAction/RobotY", position.getY());
    Logger.recordOutput("ZoneAction/ActiveZones", activeZones.toArray(new String[0]));
    Logger.recordOutput("ZoneAction/IsRedAlliance", isRed);

    for (int i = 0; i < zones.size(); i++) {
      FieldZone zone = zones.get(i);
      Translation2d[] verts =
          isRed && zone.isAllianceRelative() ? zone.getRedVertices() : zone.getVertices();
      double[] xs = new double[verts.length];
      double[] ys = new double[verts.length];
      // Build closed polygon as Pose2d[] for AdvantageScope field visualization
      Pose2d[] polygonPoses = new Pose2d[verts.length + 1];
      for (int j = 0; j < verts.length; j++) {
        xs[j] = verts[j].getX();
        ys[j] = verts[j].getY();
        polygonPoses[j] = new Pose2d(verts[j], new Rotation2d());
      }
      polygonPoses[verts.length] = new Pose2d(verts[0], new Rotation2d());

      Logger.recordOutput("ZoneAction/Zones/" + zone.getName(), polygonPoses);
      Logger.recordOutput("ZoneAction/Zone" + i + "/Name", zone.getName());
      Logger.recordOutput("ZoneAction/Zone" + i + "/Xs", xs);
      Logger.recordOutput("ZoneAction/Zone" + i + "/Ys", ys);
      Logger.recordOutput("ZoneAction/Zone" + i + "/Active", activeZones.contains(zone.getName()));
    }
  }

  private void onZoneEnter(FieldZone zone) {
    System.out.println(
        "[ZoneAction] ENTER zone: " + zone.getName() + " (action=" + zone.getAction() + ")");

    Supplier<Command> factory = enterActions.get(zone.getAction());
    if (factory != null) {
      Command cmd = factory.get();
      CommandScheduler.getInstance().schedule(cmd);
      runningEnterCommands.put(zone.getName(), cmd);
    }
  }

  private void onZoneExit(FieldZone zone) {
    System.out.println(
        "[ZoneAction] EXIT zone: " + zone.getName() + " (action=" + zone.getAction() + ")");

    // Cancel the enter command if it's still running
    Command running = runningEnterCommands.remove(zone.getName());
    if (running != null && running.isScheduled()) {
      running.cancel();
    }

    // Fire exit action
    Supplier<Command> factory = exitActions.get(zone.getAction());
    if (factory != null) {
      CommandScheduler.getInstance().schedule(factory.get());
    }
  }

  // ── Utility ───────────────────────────────────────────────────────────────

  /** Returns true if the robot is currently inside the named zone. */
  public boolean isInZone(String zoneName) {
    return activeZones.contains(zoneName);
  }

  /** Returns true if the robot is currently inside any zone with the given action key. */
  public boolean isInZoneWithAction(String action) {
    for (FieldZone zone : zones) {
      if (zone.getAction().equals(action) && activeZones.contains(zone.getName())) {
        return true;
      }
    }
    return false;
  }

  /** Returns the list of loaded zones. */
  public List<FieldZone> getZones() {
    return zones;
  }

  /** Returns the number of loaded zones. */
  public int getZoneCount() {
    return zones.size();
  }
}
