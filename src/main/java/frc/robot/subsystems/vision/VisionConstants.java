// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "FR_Camera";
  public static String camera1Name = "BR_Camera";

  // ── Camera 0: FR_Camera — after 90° right rotation of robot front ────────
  // TODO: JAPIERDOLE ARHCIE TEZ TO MUSI WYCADOWAC
  // In new frame: old forward = new left, old left = new backward
  // Positive X = forward, positive Y = left, positive Z = up (meters)
  public static double camera0ForwardM = -0.27218; // was 27.2 cm left, now backward
  public static double camera0LeftM = -0.27218; // was 29 cm forward, now left
  public static double camera0UpM = 0.192; // up from floor to camera
  public static double camera0PitchRad = Math.toRadians(15.0); // 15° tilted up
  public static double camera0YawRad = Math.PI; // camera faces new left (old forward)

  public static Transform3d robotToCamera0 =
      new Transform3d(
          camera0ForwardM,
          camera0LeftM,
          camera0UpM,
          new Rotation3d(0.0, camera0PitchRad, camera0YawRad));

  // ── Camera 1: BL_Camera — back-left, facing RIGHT (90° clockwise) ──────
  // TODO: I TO TEZ
  public static double camera1ForwardM = 0.253; // forward from center (negative = back)
  public static double camera1LeftM = -0.2545; // left from center (positive = left)
  public static double camera1UpM = 0.192; // up from floor to camera
  public static double camera1PitchRad = Math.toRadians(15.0); // camera pitch (negative = down)

  public static Transform3d robotToCamera1 =
      new Transform3d(
          camera1ForwardM,
          camera1LeftM,
          camera1UpM,
          new Rotation3d(0.0, camera1PitchRad, -Math.PI / 2.0)); // yaw=-90° → facing right

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.2;
  public static double maxZError = 0.75;
  public static double maxTagDistance = 7.0; // Reject tags beyond this distance (meters)

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.04; // Meters
  public static double angularStdDevBaseline = 0.12; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
