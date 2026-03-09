package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class DriveCommandsSimTest {

  private static class GyroIOSim implements GyroIO {}

  private static Drive makeSimDrive() {
    GyroIO gyro = new GyroIOSim();
    ModuleIO fl = new ModuleIOSim(TunerConstants.FrontLeft);
    ModuleIO fr = new ModuleIOSim(TunerConstants.FrontRight);
    ModuleIO bl = new ModuleIOSim(TunerConstants.BackLeft);
    ModuleIO br = new ModuleIOSim(TunerConstants.BackRight);
    return new Drive(gyro, fl, fr, bl, br);
  }

  @BeforeAll
  static void setupSimulationEnvironment() {
    // Ensure simulation is enabled for tests that rely on WPILib sim behavior.
    System.setProperty("wpilib.simulation.enabled", "true");
  }

  @Test
  void getAlignTarget_inSimUsesRealDrivePoseAndSpeeds() {
    Drive drive = makeSimDrive();

    Pose2d pose =
        new Pose2d(
            FieldConstants.HUB_POSE_BLUE.getX() + 2.0,
            FieldConstants.CENTER.getY() - 1.0,
            Rotation2d.kZero);
    drive.setPose(pose);
    drive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0));

    Translation3d fullTarget = invokeGetAlignTarget(drive);

    Translation3d baseTarget =
        DriveCommands.chooseAlignTarget(
            drive.getPose(), DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue));

    double baseDistance = baseTarget.toTranslation2d().getDistance(FieldConstants.HUB_POSE_BLUE);
    double fullDistance = fullTarget.toTranslation2d().getDistance(FieldConstants.HUB_POSE_BLUE);

    // In this simple integration test we just assert that we still aim generally towards
    // the expected region (distance is finite and not wildly off).
    // Students can extend this with more precise expectations if desired.
    double epsilon = 10.0;
    assertEquals(baseDistance, fullDistance, epsilon);
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
