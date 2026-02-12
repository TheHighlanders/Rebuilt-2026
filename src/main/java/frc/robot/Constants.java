// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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

  public static class ClimberConstants {
    public static final int CLIMBERID = 51;
    public static final double CLIMBER_SPEED = 1;
    public static final double UP_POSITION = 5; // Rotations. TODO
    public static final double POS_TOLERANCE = 0.1;

    public static final double RAISE_SPEED = 1;
    public static final double PULL_SPEED = -1;
    public static final double CLIMBER_SPEED_DOWN = -1;
  }

  public final class HopperConstants {

    public static final int HOPPERID = 21;
  }

  public static class IntakeConstants {
    public static final int SPINTAKEID = 41;
    public static final int DEPLOYID = 42;

    public static final double INTAKE_SPEED = 1;
    public static final double SPITAKE_SPEED = -1;
    public static final double DEPLOY_SPEED = 1;
    public static final double DEPLOY_POSITION = 3; // rotations
    public static final double DEPLOY_TOLERANCE = 0.1;

    public static final double kP1 = 0.1;
    public static final double kI1 = 0;
    public static final double kD1 = 0;

    public static final double kP2 = 0.0001;
    public static final double kI2 = 0;
    public static final double kD2 = 0;
  }

  public static class ShooterConstants {

    public static final int SHOOTERID = 31;
    public static final int KICKERID = 32;
    public static final Translation3d SHOOTER_RR_POS =
        new Translation3d(Meters.of(-0.1), Meters.of(0.3), Meters.of(0.27));
  }

  public static class FieldConstants {
    public static final Translation2d HUB_POSE_BLUE = new Translation2d(4.6, 4); // LIES FIX PLS
    public static final Translation2d HUB_POSE_RED = new Translation2d(12, 4); // LIES FIX PLS
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
