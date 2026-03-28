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
import frc.robot.Constants.FieldConstants;
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
  private static final double boost = 0.3;

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
    // AutoTrajectory turn = routine.trajectory("TestSquareTurn");

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
        .onTrue(Commands.sequence(Commands.waitSeconds(2), Commands.runOnce(() -> drive.stop())));

    return routine;
  }

  public AutoRoutine simpleShoot() {
    AutoRoutine routine = autoFactory.newRoutine("DepotAndClimb");

    AutoTrajectory collect = routine.trajectory("midToDepotShoot");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                collect.resetOdometry(),
                Commands.deadline(
                    Commands.waitSeconds(0),
                    DriveCommands.joystickDrive(drive, () -> 0, () -> 0, () -> 0, () -> true)),
                Commands.deadline(
                    Commands.waitSeconds(12.5),
                    Commands.parallel(
                        shooter.rawFlywheelCMD(() -> 0.25),
                        Commands.sequence(Commands.waitSeconds(1), hopper.shootCMD()))),
                Commands.deadline(
                    Commands.waitSeconds(2),
                    Commands.parallel(hopper.stopCMD(), shooter.stopCMD()))));

    return routine;
  }

  public AutoRoutine shootandClimb() {
    AutoRoutine routine = autoFactory.newRoutine("ShootandClimb");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                Commands.deadline(
                    Commands.waitSeconds(0),
                    DriveCommands.joystickDrive(drive, () -> 0, () -> 0, () -> 0, () -> true)),
                Commands.deadline(
                    Commands.waitSeconds(12.5),
                    Commands.parallel(
                        shooter.rawFlywheelCMD(() -> 0.25),
                        Commands.sequence(Commands.waitSeconds(1), hopper.shootCMD()))),
                Commands.parallel(hopper.stopCMD(), shooter.stopCMD()),
                DriveCommands.autoClimb(drive, climber)));

    return routine;
  }

  public AutoRoutine middle() {
    AutoRoutine routine = autoFactory.newRoutine("middle");

    AutoTrajectory outpost = routine.trajectory("midToOutpost");
    AutoTrajectory shoot = routine.trajectory("outpostToShoot");
    AutoTrajectory depot = routine.trajectory("shootToDepot");

    routine
        .active()
        .onTrue(
            Commands.sequence(sendState("mid -> outpost"), outpost.resetOdometry(), outpost.cmd()));

    outpost
        .doneDelayed(2)
        .onTrue(
            Commands.sequence(
                sendState("outpost -> shooting"), shoot.resetOdometry(), shoot.cmd()));

    shoot
        .atTimeBeforeEnd(1)
        .onTrue(
            shooter.flywheelHubCMD(
                () ->
                    Math.min(
                            drive
                                .getPose()
                                .getTranslation()
                                .getDistance(FieldConstants.HUB_POSE_BLUE),
                            drive
                                .getPose()
                                .getTranslation()
                                .getDistance(FieldConstants.HUB_POSE_RED))
                        + boost));

    shoot.done().onTrue(hopper.shootCMD().andThen(sendState("shooting!")));

    shoot.doneDelayed(3.9).onTrue(Commands.sequence(hopper.stopCMD(), shooter.stopCMD()));

    shoot
        .doneDelayed(4)
        .onTrue(
            Commands.sequence(sendState("shooting -> depot"), depot.resetOdometry(), depot.cmd()));

    depot.atTime("intake").onTrue(Commands.parallel(deploy.deployCMD(), intake.intakeCMD()));

    depot
        .atTime("retract")
        .onTrue(
            Commands.sequence(deploy.readyCMD(), Commands.waitSeconds(0.5), intake.stoptakeCMD()));

    depot
        .atTimeBeforeEnd(1)
        .onTrue(
            shooter.flywheelHubCMD(
                () ->
                    Math.min(
                            drive
                                .getPose()
                                .getTranslation()
                                .getDistance(FieldConstants.HUB_POSE_BLUE),
                            drive
                                .getPose()
                                .getTranslation()
                                .getDistance(FieldConstants.HUB_POSE_RED))
                        + boost));

    depot
        .done()
        .onTrue(
            Commands.sequence(
                sendState("final shots!"),
                hopper.shootCMD(),
                Commands.waitSeconds(5),
                hopper.stopCMD(),
                shooter.stopCMD()));

    return routine;
  }

  public AutoRoutine middleDepot() {
    AutoRoutine routine = autoFactory.newRoutine("DepotAndClimb");

    AutoTrajectory collect = routine.trajectory("midToDepotShoot");

    routine
        .active()
        .onTrue(
            Commands.sequence(sendState("Auto Started!"), collect.resetOdometry(), collect.cmd()));

    collect
        .atTime("intake")
        .onTrue(Commands.sequence(deploy.deployCMD(), intake.intakeCMD(), sendState("Intaking!")));

    collect
        .done()
        .onTrue(
            Commands.sequence(
                intake.stoptakeCMD(),
                sendState("Shooting!"),
                DriveCommands.joystickAlignDrive(drive, shooter, () -> 0, () -> 0, () -> true)));

    collect.doneDelayed(1).onTrue(hopper.shootCMD());

    collect
        .atTimeBeforeEnd(1)
        .onTrue(
            shooter.flywheelHubCMD(
                () ->
                    Math.min(
                            drive
                                .getPose()
                                .getTranslation()
                                .getDistance(FieldConstants.HUB_POSE_BLUE),
                            drive
                                .getPose()
                                .getTranslation()
                                .getDistance(FieldConstants.HUB_POSE_RED))
                        + boost));

    collect.done().onTrue(hopper.shootCMD().andThen(sendState("shooting!")));

    return routine;
  }

  public AutoRoutine leftMid(boolean aggressive) {
    AutoRoutine routine = autoFactory.newRoutine("");

    AutoTrajectory midInitial =
        aggressive
            ? routine.trajectory("leftToMidAggressive")
            : routine.trajectory("leftToMidChill");
    AutoTrajectory midSecondary = routine.trajectory("leftToMidAgain");

    routine.active().onTrue(Commands.sequence(midInitial.resetOdometry(), midInitial.cmd()));

    midInitial.atTime("intake").onTrue(Commands.parallel(deploy.deployCMD(), intake.intakeCMD()));

    midInitial.atTime("retract").onTrue(Commands.sequence(deploy.readyCMD(), intake.stoptakeCMD()));

    midInitial
        .atTimeBeforeEnd(1)
        .onTrue(
            shooter.flywheelHubCMD(
                () ->
                    Math.min(
                            drive
                                .getPose()
                                .getTranslation()
                                .getDistance(FieldConstants.HUB_POSE_BLUE),
                            drive
                                .getPose()
                                .getTranslation()
                                .getDistance(FieldConstants.HUB_POSE_RED))
                        + boost));

    midInitial.done().onTrue(hopper.shootCMD().andThen(sendState("shooting!")));

    midInitial.doneDelayed(4.9).onTrue(hopper.stopCMD().andThen(shooter.stopCMD()));

    midInitial.doneDelayed(5).onTrue(midSecondary.cmd());

    midSecondary.atTime("intake").onTrue(Commands.parallel(deploy.deployCMD(), intake.intakeCMD()));

    midSecondary
        .atTime("retract")
        .onTrue(Commands.sequence(deploy.readyCMD(), intake.stoptakeCMD()));

    midSecondary
        .atTimeBeforeEnd(1)
        .onTrue(
            shooter.flywheelHubCMD(
                () ->
                    Math.min(
                            drive
                                .getPose()
                                .getTranslation()
                                .getDistance(FieldConstants.HUB_POSE_BLUE),
                            drive
                                .getPose()
                                .getTranslation()
                                .getDistance(FieldConstants.HUB_POSE_RED))
                        + boost));

    midSecondary.done().onTrue(hopper.shootCMD().andThen(sendState("shooting!")));

    return routine;
  }

  public AutoRoutine rightMid(boolean aggressive) {
    AutoRoutine routine = autoFactory.newRoutine("");

    AutoTrajectory midInitial =
        aggressive
            ? routine.trajectory("rightToMidAggressive")
            : routine.trajectory("rightToMidChill");
    AutoTrajectory midSecondary = routine.trajectory("rightToMidAgain");

    routine.active().onTrue(Commands.sequence(midInitial.resetOdometry(), midInitial.cmd()));

    midInitial.atTime("intake").onTrue(Commands.sequence(deploy.deployCMD(), intake.intakeCMD()));

    midInitial.atTime("retract").onTrue(Commands.sequence(deploy.readyCMD(), intake.stoptakeCMD()));

    midInitial
        .atTimeBeforeEnd(1)
        .onTrue(
            shooter.flywheelHubCMD(
                () ->
                    Math.min(
                            drive
                                .getPose()
                                .getTranslation()
                                .getDistance(FieldConstants.HUB_POSE_BLUE),
                            drive
                                .getPose()
                                .getTranslation()
                                .getDistance(FieldConstants.HUB_POSE_RED))
                        + boost));

    midInitial.done().onTrue(hopper.shootCMD().andThen(sendState("shooting!")));

    midInitial.doneDelayed(4.9).onTrue(hopper.stopCMD().andThen(shooter.stopCMD()));

    midInitial.doneDelayed(5).onTrue(midSecondary.cmd());

    midSecondary.atTime("intake").onTrue(Commands.parallel(deploy.deployCMD(), intake.intakeCMD()));

    midSecondary
        .atTime("retract")
        .onTrue(Commands.sequence(deploy.readyCMD(), intake.stoptakeCMD()));

    midSecondary
        .atTimeBeforeEnd(1)
        .onTrue(
            shooter.flywheelHubCMD(
                () ->
                    Math.min(
                            drive
                                .getPose()
                                .getTranslation()
                                .getDistance(FieldConstants.HUB_POSE_BLUE),
                            drive
                                .getPose()
                                .getTranslation()
                                .getDistance(FieldConstants.HUB_POSE_RED))
                        + boost));

    midSecondary.done().onTrue(hopper.shootCMD().andThen(sendState("shooting!")));

    return routine;
  }
}
