package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;

public class Autos {
  AutoFactory autoFactory;
  AutoChooser autoChooser;
  Drive drive;

  public Autos(Drive drive) {
    autoFactory =
        new AutoFactory(
            drive::getPose, // A function that returns the current robot pose
            drive::setPose, // A function that resets the current robot pose to the provided Pose2d
            drive::followTrajectory, // The drive subsystem trajectory follower
            true, // If alliance flipping should be enabled
            drive // The drive subsystem
            );
    this.drive = drive;
  }

  public AutoRoutine shootTwiceRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("ShootTwice");

    AutoTrajectory align = routine.trajectory("align");
    AutoTrajectory collect = routine.trajectory("collect");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                align.resetOdometry(),
                align.cmd(),
                Commands.runOnce(() -> drive.stop(), drive),
                Commands.waitSeconds(3),
                collect.cmd()));

    return routine;
  }

  public AutoRoutine theBestAuto() {
    return null;
  }

  public AutoRoutine theSecondBestAuto() {
    return null;
  }
}
