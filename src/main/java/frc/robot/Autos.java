package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

  public AutoRoutine badLaptopTestAuto() {
    AutoRoutine routine = autoFactory.newRoutine("Test");

    routine.active().onTrue(Commands.none());

    return routine;
  }

  public AutoRoutine testAuto() {
    AutoRoutine routine = autoFactory.newRoutine("Test");

    AutoTrajectory square = routine.trajectory("TestSquare");
    AutoTrajectory turn = routine.trajectory("TestSquareTurn");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                square.resetOdometry(),
                square.cmd(),
                Commands.runOnce(
                    () -> {
                      SmartDashboard.putBoolean("Auto/Trigger Tests/Active Trigger", false);
                      SmartDashboard.putBoolean("Auto/Trigger Tests/Time Trigger", false);
                      SmartDashboard.putBoolean("Auto/Trigger Tests/Marker Trigger", false);
                      SmartDashboard.putBoolean("Auto/Trigger Tests/Position Trigger", false);
                      SmartDashboard.putBoolean(
                          "Auto/Trigger Tests/Time Before End Trigger", false);
                      SmartDashboard.putBoolean("Auto/Trigger Tests/Done Trigger", false);
                    })));

    square
        .active()
        .onTrue(
            Commands.runOnce(
                () -> SmartDashboard.putBoolean("Auto/Trigger Tests/Active Trigger", true)));
    square
        .atTime(1)
        .onTrue(
            Commands.runOnce(
                () -> SmartDashboard.putBoolean("Auto/Trigger Tests/Time Trigger", true)));
    square
        .atTime("halfway!")
        .onTrue(
            Commands.runOnce(
                () -> SmartDashboard.putBoolean("Auto/Trigger Tests/Marker Trigger", true)));
    square
        .atPose(new Pose2d(0, 1, Rotation2d.kZero), 0.05, 1)
        .onTrue(
            Commands.runOnce(
                () -> SmartDashboard.putBoolean("Auto/Trigger Tests/Position Trigger", true)));
    square
        .atTimeBeforeEnd(1)
        .onTrue(
            Commands.runOnce(
                () ->
                    SmartDashboard.putBoolean("Auto/Trigger Tests/Time Before End Trigger", true)));
    square
        .done()
        .onTrue(
            Commands.runOnce(
                () -> SmartDashboard.putBoolean("Auto/Trigger Tests/Done Trigger", true)));

    // .onTrue(Commands.runOnce(() -> SmartDashboard.putBoolean("Auto/Trigger Tests/Marker Trigger",
    // true)));

    square
        .done()
        .onTrue(Commands.sequence(Commands.waitSeconds(2), turn.resetOdometry(), turn.cmd()));

    return routine;
  }

  public AutoRoutine simpleShoot() {
    AutoRoutine routine = autoFactory.newRoutine("");

    routine
        .active()
        .onTrue(
            Commands.parallel(
                DriveCommands.joystickAlignDrive(drive, shooter, () -> 0, () -> 0, () -> true),
                Commands.sequence(
                    Commands.waitSeconds(1),
                    Commands.waitUntil(
                        () -> {
                          return shooter.atSpeed() && DriveCommands.aligned().getAsBoolean();
                        }),
                    // Commands.runOnce(drive::stopWithX),//maybe
                    hopper.shootCMD())));

    return routine;
  }

  public AutoRoutine simplerShoot() {
    AutoRoutine routine = autoFactory.newRoutine("");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                Commands.deadline(
                    Commands.waitSeconds(1.5),
                    DriveCommands.joystickDrive(drive, () -> 0, () -> 0.4, () -> 0, () -> true)),
                Commands.deadline(
                    Commands.waitSeconds(12.5),
                    Commands.parallel(
                        shooter.rawFlywheelCMD(() -> 0.25),
                        Commands.sequence(Commands.waitSeconds(1), hopper.shootCMD())),
                    Commands.deadline(
                        Commands.waitSeconds(2),
                        Commands.parallel(climber.pullCMD(), hopper.stopCMD(), shooter.stopCMD())),
                    Commands.deadline(Commands.waitSeconds(2), climber.raiseCMD()),
                    Commands.deadline(Commands.waitSeconds(2), climber.pullCMD()))));

    return routine;
  }

  public AutoRoutine depotAndClimb(boolean addClimb) {
    AutoRoutine routine = autoFactory.newRoutine("DepotAndClimb");

    AutoTrajectory collect = routine.trajectory("collectDepot");
    AutoTrajectory climb = routine.trajectory("climbFromDepot");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                sendState("Auto Started!"),
                DriveCommands.autoAlign(drive, collect.getInitialPose().orElse(drive.getPose())),
                collect.resetOdometry(),
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

    if (addClimb) {
      collect
          .doneDelayed(10)
          .onTrue(
              Commands.parallel(
                  sendState("Aligning!"),
                  DriveCommands.autoClimb(drive, climber),
                  hopper.stopCMD(),
                  shooter.stopCMD(),
                  deploy.undeployCMD()));

      climb
          .atTime("aligning")
          .onTrue(
              Commands.sequence(Commands.waitSeconds(1), sendState("Pulling!"), climber.pullCMD()));
    } else {
      collect.doneDelayed(12).onTrue(Commands.parallel(hopper.stopCMD(), shooter.stopCMD()));
    }

    return routine;
  }

  public AutoRoutine outpostAndClimb(boolean addClimb) {
    AutoRoutine routine = autoFactory.newRoutine("OutpostAndClimb");

    AutoTrajectory collect = routine.trajectory("collectOutpost");
    AutoTrajectory shoot = routine.trajectory("shootFromOutpost");
    AutoTrajectory climb = routine.trajectory("climbFromOutpost");

    routine
        .active()
        .onTrue(
            Commands.sequence(collect.resetOdometry(), sendState("Auto started!"), collect.cmd()));

    collect
        .doneDelayed(4)
        .onTrue(
            Commands.sequence(
                shoot.cmd(),
                DriveCommands.joystickAlignDrive(drive, shooter, () -> 0, () -> 0, () -> true)));

    shoot.doneDelayed(1).onTrue(hopper.shootCMD());

    if (addClimb) {
      shoot
          .doneDelayed(8)
          .onTrue(
              Commands.sequence(
                  shooter.stopCMD(),
                  hopper.stopCMD(),
                  Commands.parallel(climber.raiseCMD(), climb.cmd())));

      climb.atTimeBeforeEnd(0.1).onTrue(climber.pullCMD());
    } else {
      shoot.doneDelayed(12).onTrue(Commands.sequence(shooter.stopCMD(), hopper.stopCMD()));
    }

    return routine;
  }

  public AutoRoutine midClimbLeft() {
    return null;
  }

  public AutoRoutine midClimbRight() {
    return null;
  }
}
