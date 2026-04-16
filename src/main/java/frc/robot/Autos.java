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
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Deploy;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

public class Autos {
  AutoFactory autoFactory;
  AutoChooser autoChooser;
  Drive drive;
  Vision vision;
  Deploy deploy;
  Intake intake;
  Hopper hopper;
  Shooter shooter;
  Climber climber;

  public Autos(
      Drive drive, Vision vision, Deploy deploy, Intake intake, Hopper hopper, Shooter shooter, Climber climber) {
    autoFactory =
        new AutoFactory(
            drive::getPose, // A function that returns the current robot pose
            drive::setPose, // A function that resets the current robot pose to the provided Pose2d
            drive::followTrajectory, // The drive subsystem trajectory follower
            true, // If alliance flipping should be enabled
            drive // The drive subsystem
            );
    this.drive = drive;
    this.vision = vision;
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

    AutoTrajectory pose = routine.trajectory("midStayStill");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                pose.resetOdometry(),
                Commands.deadline(
                    Commands.waitSeconds(2.5),
                    DriveCommands.joystickDrive(drive, () -> 0, () -> 0.35, () -> 0, () -> true)),
                Commands.deadline(
                    Commands.waitSeconds(0.1),
                    DriveCommands.joystickDrive(drive, () -> 0, () -> 0, () -> 0, () -> true)),
                Commands.deadline(
                    Commands.waitSeconds(12.5),
                    Commands.parallel(
                        shooter.flywheelHubCMD(() -> 1.5),
                        Commands.sequence(Commands.waitSeconds(3), hopper.shootCMD()))),
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
                        drive.getPose().getTranslation().getDistance(FieldConstants.HUB_POSE_BLUE),
                        drive
                            .getPose()
                            .getTranslation()
                            .getDistance(FieldConstants.HUB_POSE_RED))));

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
                        drive.getPose().getTranslation().getDistance(FieldConstants.HUB_POSE_BLUE),
                        drive
                            .getPose()
                            .getTranslation()
                            .getDistance(FieldConstants.HUB_POSE_RED))));

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

  public AutoRoutine middleDepot(boolean side) {
    AutoRoutine routine = autoFactory.newRoutine("DepotAndClimb");

    AutoTrajectory collect = routine.trajectory(side ? "midToDepotShoot" : "midToDepotShoot2");

    routine
        .active()
        .onTrue(
            Commands.sequence(sendState("Auto Started!"), collect.resetOdometry(), collect.cmd()));

    collect
        .atTime("intake")
        .onTrue(
            Commands.sequence(
                Commands.waitSeconds(0.5),
                deploy.deployCMD(),
                intake.intakeCMD(),
                sendState("Intaking!")));

    collect
        .done()
        .onTrue(
            Commands.sequence(
                intake.stoptakeCMD(),
                sendState("Shooting!"),
                DriveCommands.joystickAlignDrive(drive, shooter, () -> 0, () -> 0, () -> true)));

    collect
        .doneDelayed(1)
        .onTrue(
            Commands.sequence(
                Commands.waitUntil(DriveCommands.aligned()::getAsBoolean), hopper.shootCMD()));

    collect
        .atTimeBeforeEnd(1)
        .onTrue(
            shooter.flywheelHubCMD(
                () ->
                    Math.min(
                        drive.getPose().getTranslation().getDistance(FieldConstants.HUB_POSE_BLUE),
                        drive
                            .getPose()
                            .getTranslation()
                            .getDistance(FieldConstants.HUB_POSE_RED))));

    collect.done().onTrue(hopper.shootCMD().andThen(sendState("shooting!")));

    return routine;
  }

  public AutoRoutine side(boolean aggressive, boolean left) {
    AutoRoutine routine = autoFactory.newRoutine("");

    AutoTrajectory midInitial =
        aggressive
            ? routine.trajectory(left ? "leftToMidAggressive" : "rightToMidAggressive")
            : routine.trajectory(left ? "leftToMidChill" : "rightToMidChill");
    AutoTrajectory midSecondary = routine.trajectory(left ? "leftToMidAgain" : "rightToMidAgain");
    AutoTrajectory backup = routine.trajectory(left ? "leftSaveMe" : "rightSaveMe");

    routine.active().onTrue(Commands.sequence(midInitial.resetOdometry(), midInitial.cmd()));

    midInitial.atTime("intake").onTrue(Commands.parallel(deploy.deployCMD(), intake.intakeCMD()));

    midInitial
        .atTime("retract")
        .onTrue(
            Commands.sequence(deploy.readyCMD(), Commands.waitSeconds(0.5), intake.stoptakeCMD()));

    midInitial
        .atTimeBeforeEnd(1)
        .onTrue(
            shooter.flywheelHubCMD(
                () ->
                    Math.min(
                        drive.getPose().getTranslation().getDistance(FieldConstants.HUB_POSE_BLUE),
                        drive
                            .getPose()
                            .getTranslation()
                            .getDistance(FieldConstants.HUB_POSE_RED))));

    midInitial
        .done()
        .onTrue(DriveCommands.joystickAlignDriveHub(drive, shooter, () -> 0, () -> 0, () -> false));

    midInitial.done().onTrue(
        Commands.sequence(
            Commands.either(
                DriveCommands.autoAlign(drive, backup.getInitialPose().orElse(drive.getPose())), 
                Commands.none(), 
                () -> drive.getPose().getTranslation().getDistance(midSecondary.getInitialPose().orElse(drive.getPose()).getTranslation()) < 1),
        hopper.shootCMD().andThen(sendState("shooting!"))
    ));

    midInitial.doneDelayed(4.9).onTrue(hopper.stopCMD().andThen(shooter.stopCMD()));

    midInitial
        .doneDelayed(5)
        .onTrue(
            Commands.sequence(
                Commands.either(
                    DriveCommands.autoAlign(
                        drive, midSecondary.getInitialPose().orElse(drive.getPose())),
                    Commands.none(),
                    () -> vision.hasTarget() && drive.getPose().getTranslation().getDistance(midSecondary.getInitialPose().orElse(drive.getPose()).getTranslation()) < 1),
                midSecondary.cmd()));

    midSecondary.atTime("intake").onTrue(Commands.parallel(deploy.deployCMD(), intake.intakeCMD()));

    midSecondary
        .atTime("retract")
        .onTrue(
            Commands.sequence(deploy.readyCMD(), Commands.waitSeconds(0.5), intake.stoptakeCMD()));

    midSecondary
        .atTimeBeforeEnd(1)
        .onTrue(
            shooter.flywheelHubCMD(
                () ->
                    Math.min(
                        drive.getPose().getTranslation().getDistance(FieldConstants.HUB_POSE_BLUE),
                        drive
                            .getPose()
                            .getTranslation()
                            .getDistance(FieldConstants.HUB_POSE_RED))));

    midSecondary
        .done()
        .onTrue(DriveCommands.joystickAlignDriveHub(drive, shooter, () -> 0, () -> 0, () -> false));

    midSecondary.done().onTrue(
        Commands.sequence(
            Commands.either(
                DriveCommands.autoAlign(drive, backup.getInitialPose().orElse(drive.getPose())), 
                Commands.none(), 
                () -> drive.getPose().getTranslation().getDistance(midSecondary.getInitialPose().orElse(drive.getPose()).getTranslation()) < 1),
        hopper.shootCMD().andThen(sendState("shooting!"))
    ));

    return routine;
  }

  public AutoRoutine sideDefend(boolean aggressive, boolean left) {
    AutoRoutine routine = autoFactory.newRoutine("");

    AutoTrajectory midInitial =
        aggressive
            ? routine.trajectory(left ? "leftToMidAggressive" : "rightToMidAggressive")
            : routine.trajectory(left ? "leftToMidChill" : "rightToMidChill");
    AutoTrajectory midSecondary = routine.trajectory(left ? "leftToMidDef" : "rightToMidDef");

    routine.active().onTrue(Commands.sequence(midInitial.resetOdometry(), midInitial.cmd()));

    midInitial.atTime("intake").onTrue(Commands.parallel(deploy.deployCMD(), intake.intakeCMD()));

    midInitial
        .atTime("retract")
        .onTrue(
            Commands.sequence(deploy.readyCMD(), Commands.waitSeconds(0.5), intake.stoptakeCMD()));

    midInitial
        .atTimeBeforeEnd(1)
        .onTrue(
            shooter.flywheelHubCMD(
                () ->
                    Math.min(
                        drive.getPose().getTranslation().getDistance(FieldConstants.HUB_POSE_BLUE),
                        drive
                            .getPose()
                            .getTranslation()
                            .getDistance(FieldConstants.HUB_POSE_RED))));

    midInitial.done().onTrue(hopper.shootCMD().andThen(sendState("shooting!")));

    midInitial.doneDelayed(4.9).onTrue(hopper.stopCMD().andThen(shooter.stopCMD()));

    midInitial.doneDelayed(5).onTrue(midSecondary.cmd());

    midSecondary.atTime("intake").onTrue(Commands.parallel(deploy.deployCMD(), intake.intakeCMD()));

    midSecondary
        .atTime("retract")
        .onTrue(
            Commands.sequence(deploy.readyCMD(), Commands.waitSeconds(0.5), intake.stoptakeCMD()));

    return routine;
  }

public AutoRoutine simpleShootSneak(boolean left) {
    AutoRoutine routine = autoFactory.newRoutine("DepotAndClimb");

    AutoTrajectory sneak =
        left ? routine.trajectory("midSneakLeft") : routine.trajectory("midSneakRight");

    double routineWaitTimer =
        (left ? DriveConstants.SNEAK_WAIT_TIME_LEFT : DriveConstants.SNEAK_WAIT_TIME_RIGHT) - 2;

    routine
        .active()
        .onTrue(
            Commands.parallel(
                Commands.sequence(
                    Commands.deadline(
                        Commands.waitSeconds(routineWaitTimer),
                        Commands.parallel(
                            shooter.flywheelHubCMD(() -> 1),
                            Commands.sequence(Commands.waitSeconds(3), hopper.shootCMD()))),
                    Commands.deadline(
                        Commands.waitSeconds(2),
                        Commands.parallel(hopper.stopCMD(), shooter.stopCMD()))),
                Commands.sequence(
                    sneak.resetOdometry(), Commands.waitSeconds(routineWaitTimer), sneak.cmd())));

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

    midInitial
        .atTime("retract")
        .onTrue(
            Commands.sequence(deploy.readyCMD(), Commands.waitSeconds(0.5), intake.stoptakeCMD()));

    midInitial
        .atTimeBeforeEnd(1)
        .onTrue(
            shooter.flywheelHubCMD(
                () ->
                    Math.min(
                        drive.getPose().getTranslation().getDistance(FieldConstants.HUB_POSE_BLUE),
                        drive
                            .getPose()
                            .getTranslation()
                            .getDistance(FieldConstants.HUB_POSE_RED))));

    midInitial
        .done()
        .onTrue(DriveCommands.joystickAlignDriveHub(drive, shooter, () -> 0, () -> 0, () -> false));

    midInitial.done().onTrue(hopper.shootCMD().andThen(sendState("shooting!")));

    midInitial.doneDelayed(4.9).onTrue(hopper.stopCMD().andThen(shooter.stopCMD()));

    midInitial
        .doneDelayed(5)
        .onTrue(
            Commands.sequence(
                DriveCommands.autoAlign(
                    drive, midSecondary.getInitialPose().orElse(drive.getPose())),
                midSecondary.cmd()));

    midSecondary.atTime("intake").onTrue(Commands.parallel(deploy.deployCMD(), intake.intakeCMD()));

    midSecondary
        .atTime("retract")
        .onTrue(
            Commands.sequence(deploy.readyCMD(), Commands.waitSeconds(0.5), intake.stoptakeCMD()));

    midSecondary
        .atTimeBeforeEnd(1)
        .onTrue(
            shooter.flywheelHubCMD(
                () ->
                    Math.min(
                        drive.getPose().getTranslation().getDistance(FieldConstants.HUB_POSE_BLUE),
                        drive
                            .getPose()
                            .getTranslation()
                            .getDistance(FieldConstants.HUB_POSE_RED))));

    midSecondary
        .done()
        .onTrue(DriveCommands.joystickAlignDriveHub(drive, shooter, () -> 0, () -> 0, () -> false));

    midSecondary.done().onTrue(hopper.shootCMD().andThen(sendState("shooting!")));

    return routine;
  }

  public AutoRoutine rightMidDefend(boolean aggressive) {
    AutoRoutine routine = autoFactory.newRoutine("");

    AutoTrajectory midInitial =
        aggressive
            ? routine.trajectory("rightToMidAggressive")
            : routine.trajectory("rightToMidChill");
    AutoTrajectory midSecondary = routine.trajectory("rightToMidDef");

    routine.active().onTrue(Commands.sequence(midInitial.resetOdometry(), midInitial.cmd()));

    midInitial.atTime("intake").onTrue(Commands.sequence(deploy.deployCMD(), intake.intakeCMD()));

    midInitial
        .atTime("retract")
        .onTrue(
            Commands.sequence(deploy.readyCMD(), Commands.waitSeconds(0.5), intake.stoptakeCMD()));

    midInitial
        .atTimeBeforeEnd(1)
        .onTrue(
            shooter.flywheelHubCMD(
                () ->
                    Math.min(
                        drive.getPose().getTranslation().getDistance(FieldConstants.HUB_POSE_BLUE),
                        drive
                            .getPose()
                            .getTranslation()
                            .getDistance(FieldConstants.HUB_POSE_RED))));

    midInitial.done().onTrue(hopper.shootCMD().andThen(sendState("shooting!")));

    midInitial.doneDelayed(4.9).onTrue(hopper.stopCMD().andThen(shooter.stopCMD()));

    midInitial.doneDelayed(5).onTrue(midSecondary.cmd());

    midSecondary.atTime("intake").onTrue(Commands.parallel(deploy.deployCMD(), intake.intakeCMD()));

    midSecondary
        .atTime("retract")
        .onTrue(
            Commands.sequence(deploy.readyCMD(), Commands.waitSeconds(0.5), intake.stoptakeCMD()));

    return routine;
  }
}
