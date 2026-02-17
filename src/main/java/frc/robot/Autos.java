package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Deploy;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Hopper;
import frc.robot.subsystems.shooter.Shooter;

public class Autos {
  AutoFactory autoFactory;
  AutoChooser autoChooser;
  Drive drive;
  Deploy deploy;
  Intake intake;
  Hopper hopper;
  Shooter shooter;
  Climber climber;

  public Autos(
      Drive drive, Deploy deploy, Intake intake, Hopper hopper, Shooter shooter, Climber climber) {
    autoFactory =
        new AutoFactory(
            drive::getPose, // A function that returns the current robot pose
            drive::setPose, // A function that resets the current robot pose to the provided Pose2d
            drive::followTrajectory, // The drive subsystem trajectory follower
            true, // If alliance flipping should be enabled
            drive // The drive subsystem
            );
    this.drive = drive;
    this.deploy = deploy;
    this.intake = intake;
    this.hopper = hopper;
    this.shooter = shooter;
    this.climber = climber;
  }

  private Command sendState(String state) {
    return Commands.runOnce(() -> SmartDashboard.putString("Auto/Auto State", state));
  }

  public AutoRoutine depotAndClimb() {
    AutoRoutine routine = autoFactory.newRoutine("DepotAndClimb");

    AutoTrajectory collect = routine.trajectory("collect");
    AutoTrajectory climb = routine.trajectory("climb");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                collect.resetOdometry(),
                sendState("Auto Started!"),
                Commands.parallel(collect.cmd(), deploy.deployCMD())));

    collect.atTime("intake").onTrue(Commands.sequence(intake.intakeCMD(), sendState("Intaking!")));

    collect
        .done()
        .onTrue(
            Commands.sequence(
                intake.stoptakeCMD(),
                sendState("Shooting!"),
                DriveCommands.joystickAlignDrive(drive, shooter, () -> 0, () -> 0, () -> true)));

    collect.doneDelayed(1).onTrue(hopper.shootCMD());

    collect
        .doneDelayed(8)
        .onTrue(
            Commands.parallel(
                sendState("Aligning!"),
                climb.cmd(), 
                climber.raiseCMD(), 
                hopper.stopCMD(), 
                shooter.stopCMD()));

    climb.atTime("aligning").onTrue(Commands.sequence(
      Commands.waitSeconds(1),
      sendState("Pulling!"),
      climber.pullCMD()));

    return routine;
  }

  public AutoRoutine humanAndClimb() {
    return null;
  }

  public AutoRoutine midClimbLeft() {
    return null;
  }

  public AutoRoutine midClimbRight() {
    return null;
  }
}
