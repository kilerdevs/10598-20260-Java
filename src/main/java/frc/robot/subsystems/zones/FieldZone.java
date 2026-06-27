package frc.robot.subsystems.zones;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * A polygon zone on the field. The robot's position is tested against these vertices using a
 * point-in-polygon (ray casting) algorithm.
 */
public class FieldZone {
  private final String name;
  private final String action;
  private final boolean allianceRelative;
  private final Translation2d[] vertices;

  // Mirrored vertices for the red alliance (cached on first access)
  private Translation2d[] redVertices;

  private final double fieldLength;

  public FieldZone(
      String name,
      String action,
      boolean allianceRelative,
      Translation2d[] vertices,
      double fieldLength) {
    this.name = name;
    this.action = action;
    this.allianceRelative = allianceRelative;
    this.vertices = vertices;
    this.fieldLength = fieldLength;
  }

  public String getName() {
    return name;
  }

  public String getAction() {
    return action;
  }

  public boolean isAllianceRelative() {
    return allianceRelative;
  }

  public Translation2d[] getVertices() {
    return vertices;
  }

  /**
   * Returns the vertices mirrored for the red alliance (X flipped around field center). Only
   * applicable when {@link #isAllianceRelative()} is true.
   */
  public Translation2d[] getRedVertices() {
    if (redVertices == null) {
      redVertices = new Translation2d[vertices.length];
      for (int i = 0; i < vertices.length; i++) {
        redVertices[i] = new Translation2d(fieldLength - vertices[i].getX(), vertices[i].getY());
      }
    }
    return redVertices;
  }

  /**
   * Returns true if the given point is inside this zone's polygon using the ray-casting algorithm.
   *
   * @param point The point to test (field-relative, meters)
   * @param useRed If true and this zone is alliance-relative, use the mirrored red vertices
   */
  public boolean contains(Translation2d point, boolean useRed) {
    Translation2d[] verts = (useRed && allianceRelative) ? getRedVertices() : vertices;
    return pointInPolygon(point, verts);
  }

  /** Ray-casting point-in-polygon test. */
  private static boolean pointInPolygon(Translation2d point, Translation2d[] polygon) {
    boolean inside = false;
    int n = polygon.length;
    for (int i = 0, j = n - 1; i < n; j = i++) {
      double xi = polygon[i].getX(), yi = polygon[i].getY();
      double xj = polygon[j].getX(), yj = polygon[j].getY();

      boolean intersect =
          ((yi > point.getY()) != (yj > point.getY()))
              && (point.getX() < (xj - xi) * (point.getY() - yi) / (yj - yi) + xi);
      if (intersect) {
        inside = !inside;
      }
    }
    return inside;
  }
}
