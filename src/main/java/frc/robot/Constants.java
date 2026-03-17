// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final class DriveConstants {
    public static final double SLOWMODE = 0.6;
    public static final double POINT_DEADBAND = 0.4;
    public static final Pose2d POSE_RESET =
        new Pose2d(Meters.of(3.61), Meters.of(3.87), Rotation2d.kZero);//CHANGED
    public static final int GYRO_ID = 0; // TODO
    public static final Angle ALIGN_SHOOTER_COMP =
        Radians.of(
            Math.PI
                / 2); // 1.67); // could make this a function of distance, but this works for now
    public static final Translation2d TO_CORNER_BUMPERS =
        new Translation2d(Units.inchesToMeters(33.25 / 2), Units.inchesToMeters(33.6 / 2));
  }

  public static final class VisionConstants {
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
            -0.132,
            -0.248,
            0.0513 + 0.115 + 0.0375,
            new Rotation3d(0.0, Units.degreesToRadians(-25), Math.PI / 2));
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
    public static final double UP_POSITION = 0; // Rotations. TODO
    public static final double DOWN_POSITION = 4;
    public static final double POS_TOLERANCE = 0.1;

    public static final double RAISE_SPEED = -0.3;
    public static final double PULL_SPEED = 1;
  }

  public final class HopperConstants {

    public static final int HOPPERID = 21;
    public static final int KICKERID = 31;
    public static final int KICKER_CURRENT_LIMIT = 50;
    public static final int HOPPER_CURRENT_LIMIT = 50;
    public static final boolean INVERT_KICKER = true;
    public static final boolean INVERT_HOPPER = false;
  }

  public static class IntakeConstants {
    public static final int SPINTAKEID = 42;
    public static final int SPINTAKEID_KRAKEN = 43;
    public static final int DEPLOYID = 41;

    public static final double INTAKE_SPEED = 0.5;
    public static final double SPITAKE_SPEED = -0.5;
    public static final double DEPLOY_SPEED = 1;
    public static final Angle DEPLOY_POSITION = Degrees.of(70);
    public static final Angle READY_POSITION = Degrees.of(40);
    public static final Angle UP_POSITION = Degrees.of(0);
    public static final Angle DEPLOY_TOLERANCE = Degrees.of(5);

    public static final double kP = 1; // 1.33;
    public static final double kI = 0;
    public static final double kD = 7.84;
    public static final double DEPLOY_RATIO = 25;

    public static double kS = 1.68;
    public static double kG = 3.05;
    public static double kV = 0.27;
  }

  public static class ShooterConstants {

    public static final int SHOOTERID = 32;

    public static final double kP = 0.11;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0.1;
    public static final double kV = 0.12;
    public static final double GEAR_RATIO = 2;

    public static final Translation3d SHOOTER_RR_POS =
        new Translation3d(Meters.of(-0.1), Meters.of(0.3), Meters.of(0.27)); // TODO find in cad
    public static final Angle SHOOTER_HOOD = Degrees.of(78);
    public static final double HOOD_SLOPE = 1 / Math.tan(Units.degreesToRadians(78));
    public static final Distance FLYWHEEL_RADIUS = Inches.of(2);

    public static final double GRAVITY = 9.80665;

    public static class LookupTable {
      // SORTED!!
      public static final double[] RPMS = {1, 2, 3, 5}; // TODO: fill with real data
      // DISTANCE OF FUEL LANDING FROM SHOOTER!!
      public static final double[] DISTS = {1, 2, 2.5, 5.5}; // TODO: fill with real data
    }
  }

  public static class FieldConstants {
    public static final Translation2d HUB_POSE_BLUE =
        new Translation2d(
            VisionConstants.aprilTagLayout.getTagPose(18).get().getX(),
            VisionConstants.aprilTagLayout.getTagPose(4).get().getY());
    public static final Translation2d HUB_POSE_RED =
        new Translation2d(
            VisionConstants.aprilTagLayout.getTagPose(5).get().getX(),
            VisionConstants.aprilTagLayout.getTagPose(4).get().getY());
    public static final double HUB_HEIGHT = 1.8288; // meters

    public static final Translation2d CENTER =
        new Translation2d(
            Units.inchesToMeters(325.61),
            Units.inchesToMeters(158.84)); // how you know none of this is vibe coded

    public static final Translation2d CLIMB_DEPOT_CORNER =
        new Translation2d(
            Units.inchesToMeters(40 + (3.51 / 2)),
            Units.inchesToMeters((158.32 - 11.46) + (32.25 / 2) + 1.5));
    public static final Translation2d CLIMB_OUTPOST_CORNER =
        new Translation2d(
            Units.inchesToMeters(40 + (3.51 / 2)),
            Units.inchesToMeters((158.32 - 11.46) - ((32.25 / 2) + 1.5)));
    public static final Translation2d OUTPOST_POSE_BLUE =
        new Translation2d(Units.inchesToMeters(33.25 / 2), Units.inchesToMeters(26.22));
    public static final Translation2d DEPOT_POSE_BLUE =
        new Translation2d(
            Units.inchesToMeters(27 / 2), Units.inchesToMeters(158.84 + 75.93)); // TODO: measure
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
