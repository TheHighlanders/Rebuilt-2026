package frc.robot;

import choreo.auto.AutoFactory;
import frc.robot.subsystems.drive.Drive;

public class Autos {
    AutoFactory autoFactory;
    Drive drivesubsystem;

  public Autos(
    Drive drivesubsystem) {
        autoFactory = new AutoFactory(
        drivesubsystem::getPose, // A function that returns the current robot pose
        drivesubsystem::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
        drivesubsystem::followTrajectory, // The drive subsystem trajectory follower 
true, // If alliance flipping should be enabled 
        drivesubsystem, // The drive subsystem
    );
    this.drivesubsystem = drivesubsystem;
  }
}
