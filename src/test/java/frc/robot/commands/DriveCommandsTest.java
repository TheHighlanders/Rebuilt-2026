package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import java.util.Arrays;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

public class DriveCommandsTest {
  private static class GyroIOSim implements GyroIO {}

  private static Drive makeSimDrive() {
    GyroIO gyro = new GyroIOSim();
    ModuleIO fl = new ModuleIOSim(TunerConstants.FrontLeft);
    ModuleIO fr = new ModuleIOSim(TunerConstants.FrontRight);
    ModuleIO bl = new ModuleIOSim(TunerConstants.BackLeft);
    ModuleIO br = new ModuleIOSim(TunerConstants.BackRight);
    return new Drive(gyro, fl, fr, bl, br);
  }

  private enum TargetKind {
    HUB,
    DEPOT,
    OUTPOST
  }

  private static final double THRESHOLD_X =
      FieldConstants.HUB_POSE_BLUE.getX() + 0.597154; // Alliance-side boundary

  private static Stream<Arguments> fieldLocations() {
    double[] xs = {
      THRESHOLD_X - 1.0, // clearly inside alliance half
      THRESHOLD_X + 1.0, // just outside alliance half
      THRESHOLD_X + 3.0 // far into neutral/opposite half
    };
    double[] ys = {
      FieldConstants.CENTER.getY() - 1.0, // lower half
      FieldConstants.CENTER.getY() + 1.0 // upper half
    };

    return Stream.of(Alliance.Blue, Alliance.Red)
        .flatMap(
            alliance ->
                Arrays.stream(xs)
                    .boxed()
                    .flatMap(
                        x -> Arrays.stream(ys).boxed().map(y -> Arguments.of(x, y, alliance))));
  }

  @ParameterizedTest
  @MethodSource("fieldLocations")
  void chooseAlignTarget_matchesExpectedZone(double x, double y, Alliance alliance) {
    Pose2d pose = new Pose2d(x, y, Rotation2d.kZero);

    // Determine expected target zone in blue-normalized coordinates
    Pose2d normalizedPose =
        alliance == Alliance.Red
            ? pose.rotateAround(FieldConstants.CENTER, Rotation2d.k180deg)
            : pose;

    TargetKind expectedKind;
    if (normalizedPose.getX() < THRESHOLD_X) {
      expectedKind = TargetKind.HUB;
    } else if (normalizedPose.getY() > FieldConstants.CENTER.getY()) {
      expectedKind = TargetKind.DEPOT;
    } else {
      expectedKind = TargetKind.OUTPOST;
    }

    // Call helper under test
    Translation3d target = DriveCommands.chooseAlignTarget(pose, alliance);

    // Normalize target back to blue-alliance frame for comparison
    Translation3d normalizedTarget =
        alliance == Alliance.Red
            ? target.rotateAround(
                new Translation3d(FieldConstants.CENTER),
                new edu.wpi.first.math.geometry.Rotation3d(Rotation2d.k180deg))
            : target;

    Translation3d expectedTarget;
    switch (expectedKind) {
      case HUB:
        expectedTarget =
            new Translation3d(FieldConstants.HUB_POSE_BLUE.plus(new Translation2d(0.4, 0.0)))
                .plus(new Translation3d(0.0, 0.0, FieldConstants.HUB_HEIGHT));
        break;
      case DEPOT:
        expectedTarget = new Translation3d(FieldConstants.DEPOT_POSE_BLUE);
        break;
      case OUTPOST:
      default:
        expectedTarget = new Translation3d(FieldConstants.OUTPOST_POSE_BLUE);
        break;
    }

    double epsilon = 1e-3;
    assertEquals(expectedTarget.getX(), normalizedTarget.getX(), epsilon);
    assertEquals(expectedTarget.getY(), normalizedTarget.getY(), epsilon);
    assertEquals(expectedTarget.getZ(), normalizedTarget.getZ(), epsilon);
  }

  @Test
  void getAlignTarget_withZeroSpeeds_matchesChooseAlignTarget() {
    Drive drive = makeSimDrive();
    Pose2d pose =
        new Pose2d(
            FieldConstants.HUB_POSE_BLUE.getX() + 2.0,
            FieldConstants.CENTER.getY() - 1.0,
            Rotation2d.kZero);
    drive.setPose(pose);
    drive.runVelocity(new ChassisSpeeds());

    Translation3d helperTarget =
        DriveCommands.chooseAlignTarget(pose, DriverStation.getAlliance().orElse(Alliance.Blue));
    Translation3d fullTarget = invokeGetAlignTarget(drive);

    double epsilon = 1e-6;
    assertEquals(helperTarget.getX(), fullTarget.getX(), epsilon);
    assertEquals(helperTarget.getY(), fullTarget.getY(), epsilon);
    assertEquals(helperTarget.getZ(), fullTarget.getZ(), epsilon);
  }

  @Test
  void getAlignTarget_withMotion_includesLeadOffset() {
    Drive drive = makeSimDrive();
    Pose2d pose =
        new Pose2d(
            FieldConstants.HUB_POSE_BLUE.getX() + 2.0,
            FieldConstants.CENTER.getY() - 1.0,
            Rotation2d.kZero);
    drive.setPose(pose);
    drive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0));

    Translation3d baseTarget =
        DriveCommands.chooseAlignTarget(pose, DriverStation.getAlliance().orElse(Alliance.Blue));
    Translation3d fullTarget = invokeGetAlignTarget(drive);

    double distance = fullTarget.toTranslation2d().getDistance(baseTarget.toTranslation2d());
    assertTrue(distance > 0.0);
  }

  private static Translation3d invokeGetAlignTarget(Drive drive) {
    try {
      var method =
          DriveCommands.class.getDeclaredMethod(
              "getAlignTarget", frc.robot.subsystems.drive.Drive.class);
      method.setAccessible(true);
      return (Translation3d) method.invoke(null, drive);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }
}
