// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final class DriveConstants {}

  public final class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "intake_camera_driver";
    public static String camera1Name = "shooter_camera";
    public static String camera2Name = "BL_camera";
    public static String camera3Name = "BR_camera";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)

    public static Transform3d robotToCamera0 =
        new Transform3d(Units.inchesToMeters(12), 0, 0.5, new Rotation3d(0.0, 0.0, 0.0));
    public static Transform3d robotToCamera1 =
        new Transform3d(
            Units.inchesToMeters(7),
            Units.inchesToMeters(12),
            0.1,
            new Rotation3d(0.0, -0.6, Math.PI / 2));
    public static Transform3d robotToCamera2 =
        new Transform3d(
            Units.inchesToMeters(7.5),
            Units.inchesToMeters(-12),
            0.1,
            new Rotation3d(0.0, -0.6, -Math.PI / 2));
    public static Transform3d robotToCamera3 =
        new Transform3d(
            Units.inchesToMeters(10),
            Units.inchesToMeters(-10),
            0.15,
            new Rotation3d(0.0, -0.6, -Math.PI * (3.0 / 4)));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
        new double[] {
          1.0, // Camera 0
          1.0, // Camera 1
          1.0, // Camera 2
          1.0 // Camera 3
        };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor =
        Double.POSITIVE_INFINITY; // No rotation data available
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
