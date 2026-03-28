// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.*;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOBoron;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Deploy;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Hopper;
import frc.robot.subsystems.shooter.HopperSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Autos autos;
  private final Intake intake;
  private final Deploy deploy;
  private final Hopper hopper;
  private final Climber climber;
  private final Shooter shooter;
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  private boolean robotRelative;
  private double speed;
  private double mannualShotLength;
  double testDistance = 1;

  // Dashboard inputs
  //  private final LoggedDashboardChooser<Command> autoChooser;
  private final AutoChooser autoChooser;

  public FuelSim fuelSim = new FuelSim("fuelsim"); // creates a new fuelSim of FuelSim

  @SuppressWarnings("unused")
  private Command testVisionSim;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Define Climber
    climber = new Climber();
    // Define Intake
    intake = new Intake();
    // Define Intake Deployer
    deploy = new Deploy();

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            // new Drive(
            //     new GyroIO() {},
            //     new ModuleIO() {},
            //     new ModuleIO() {},
            //     new ModuleIO() {},
            //     new ModuleIO() {});
            new Drive(
                new GyroIOBoron(), // new GyroIOBoron(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        // new VisionIOPhotonVision(
        //     VisionConstants.camera0Name, VisionConstants.robotToCamera0),
        // new VisionIOPhotonVision(
        //     VisionConstants.camera1Name, VisionConstants.robotToCamera1));
        // new VisionIOPhotonVision(
        //     VisionConstants.camera2Name, VisionConstants.robotToCamera2),
        // new VisionIOPhotonVision(
        //     VisionConstants.camera3Name, VisionConstants.robotToCamera3));
        shooter = new Shooter();
        hopper = new Hopper();
        configureButtonBindings();

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIONavX(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        // new VisionIOPhotonVisionSim(
        //     VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
        // new VisionIOPhotonVisionSim(
        //     VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose),
        // new VisionIOPhotonVisionSim(
        //     VisionConstants.camera2Name, VisionConstants.robotToCamera2, drive::getPose),
        // new VisionIOPhotonVisionSim(
        //     VisionConstants.camera3Name, VisionConstants.robotToCamera3, drive::getPose));
        ShooterSim temp = new ShooterSim(fuelSim); // how do i destruct this
        shooter = temp;
        hopper = new HopperSim(temp);

        fuelSim.spawnStartingFuel(false); // Armaan, disable center fuel for performance

        // Register a robot for collision with fuel
        fuelSim.registerRobot(
            0.6858, // from left to right in meters
            0.6858, // from front to back in meters
            0.1, // from floor to top of bumpers in meters
            drive::getPose, // Supplier<Pose2d> of robot pose
            () ->
                ChassisSpeeds.fromRobotRelativeSpeeds(
                    drive.getSpeeds().vxMetersPerSecond,
                    drive.getSpeeds().vyMetersPerSecond,
                    drive.getSpeeds().omegaRadiansPerSecond,
                    drive.getRotation())); // Supplier<ChassisSpeeds> of field-centric chassis
        // speeds.

        // Register an intake to remove fuel from the field as a rectangular bounding box
        fuelSim.registerIntake(
            0.35,
            0.45,
            -0.35,
            0.35,
            () -> HopperSim.intake()); // robot-centric coordinates for bounding box in meters

        fuelSim.setSubticks(
            3); // sets the number of physics iterations to perform per 20ms loop. Default = 5

        fuelSim.enableAirResistance(); // an additional drag force will be applied to fuel in
        // physics
        // update step

        fuelSim
            .start(); // enables the simulation to run (updateSim must still be called periodically)

        configureButtonBindings();

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        shooter = new Shooter();
        hopper = new Hopper();
        configureButtonBindings();

        break;
    }

    testVisionSim =
        Commands.runOnce(() -> SmartDashboard.putNumber("failed tests", 0))
            .andThen(
                Commands.run(
                    () -> {
                      if (drive.getPose().getMeasureX().in(Meters) > 10) {

                      } else if (drive.getPose().getMeasureY().in(Meters) > 8.5) {
                        drive.setPose(
                            new Pose2d(drive.getPose().getX() + 0.5, 0, Rotation2d.fromDegrees(0)));

                      } else if (drive.getPose().getRotation().getDegrees() < -15
                          && drive.getPose().getRotation().getDegrees() > -45) {
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getX(),
                                drive.getPose().getY() + 0.5,
                                Rotation2d.fromDegrees(0)));

                      } else
                        drive.setPose(
                            drive
                                .getPose()
                                .plus(new Transform2d(0, 0, Rotation2d.fromDegrees(36))));
                      if (!vision.hasTarget()) {
                        SmartDashboard.putNumber(
                            "failed tests", SmartDashboard.getNumber("failed tests", 0) + 1);
                      }
                    },
                    drive,
                    vision));

    // Set up auto routines
    autos = new Autos(drive, deploy, intake, hopper, shooter, climber);
    //  autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoFactory.buildAutoChooser());
    autoChooser = new AutoChooser();
    // autoChooser.addRoutine("Subsystem Test", () -> autos.badLaptopTestAuto());
    autoChooser.addRoutine("Test Square", () -> autos.testAuto());
    autoChooser.addRoutine("middle -> shoot", () -> autos.simpleShoot());
    autoChooser.addRoutine("middle -> outpost -> depot", () -> autos.middle());
    autoChooser.addRoutine("middle -> depot", () -> autos.middleDepot());
    autoChooser.addRoutine("left -> neutral", () -> autos.leftMid(true));
    autoChooser.addRoutine("left -> neutral (chill)", () -> autos.leftMid(false));
    autoChooser.addRoutine("right -> neutral", () -> autos.rightMid(true));
    autoChooser.addRoutine("right -> neutral (chill)", () -> autos.rightMid(false));

    SmartDashboard.putData("Auto Chooser", autoChooser);
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    robotRelative = false;
    speed = 1;
    mannualShotLength = 1;

    /* DRIVE COMMANDS */
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY() * speed,
            () -> -controller.getLeftX() * speed,
            () -> controller.getRightX() * speed,
            () -> robotRelative));

    // toggles between robot- and field-relative drive
    controller
        .leftStick()
        .onTrue(
            Commands.runOnce(
                () -> {
                  robotRelative = !robotRelative;
                  SmartDashboard.putBoolean("Robot Relative Drive", robotRelative);
                }));

    controller
        .rightStick()
        .toggleOnTrue(
            Commands.sequence(
                Commands.waitUntil(() -> !controller.rightStick().getAsBoolean()),
                DriveCommands.joystickPointDrive(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> -controller.getRightY(),
                        () -> -controller.getRightX(),
                        () -> robotRelative)
                    .until(() -> controller.rightStick().getAsBoolean())));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // snaps intake forward
    controller
        .b()
        .onTrue(
            DriveCommands.joystickPointDrive(
                    drive,
                    () -> -controller.getLeftY() * speed,
                    () -> -controller.getLeftX() * speed,
                    () -> 1,
                    () -> 0,
                    () -> robotRelative)
                .until(DriveCommands.aligned()));

    // auto align
    controller
        .rightBumper()
        .onTrue(
            DriveCommands.joystickAlignDrive(
                    drive,
                    shooter,
                    () -> -controller.getLeftY() * 0.7,
                    () -> -controller.getLeftX() * 0.7,
                    () -> robotRelative)
                .until(() -> !controller.rightBumper().getAsBoolean()));

    // slow mode
    operator
        .povDown()
        .onTrue(
            Commands.runOnce(
                () -> {
                  speed -= 0.1;
                  if (speed < DriveConstants.SLOWMODE) speed = 1;
                  SmartDashboard.putNumber("Drive/Speed", speed);
                }));

    // reset drive commands
    operator
        .povUp()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(
                    () -> {
                      speed = 1;
                      robotRelative = false;
                    }),
                DriveCommands.joystickDrive(
                    drive,
                    () -> -controller.getLeftY() * speed,
                    () -> -controller.getLeftX() * speed,
                    () -> controller.getRightX() * speed,
                    () -> robotRelative)));

    // fancy gyro reset
    operator
        .leftStick()
        .onTrue(
            DriveCommands.joystickGyroOverride(
                    drive,
                    () -> -controller.getLeftX() * speed,
                    () -> -controller.getLeftY() * speed,
                    () -> controller.getRightX() * speed,
                    () -> -operator.getLeftY(),
                    () -> -operator.getLeftX(),
                    () -> robotRelative)
                .until(() -> !operator.leftStick().getAsBoolean()));

    // reset gyro
    controller.povDown().onTrue(Commands.runOnce(() -> drive.setPose(DriveConstants.POSE_RESET)));

    operator
        .leftStick()
        .onTrue(
            DriveCommands.joystickGyroOverride(
                    drive,
                    () -> -controller.getLeftX() * speed,
                    () -> -controller.getLeftY() * speed,
                    () -> controller.getRightX() * speed,
                    () -> -operator.getLeftY(),
                    () -> -operator.getLeftX(),
                    () -> robotRelative)
                .until(() -> !operator.leftStick().getAsBoolean()));

    /* INTAKE COMMANDS. */
    // intake and deploy
    controller
        .leftBumper()
        .onTrue(
            Commands.parallel(
                intake.intakeCMD(),
                Commands.either(Commands.none(), deploy.deployCMD(), operator.b()::getAsBoolean)));
    controller
        .leftBumper()
        .onFalse(Commands.parallel(intake.stoptakeCMD())); // , deploy.readyCMD()));
    // outtake
    operator.a().onTrue(intake.spitakeCMD());
    operator
        .a()
        .onFalse(
            Commands.either(
                intake.intakeCMD(), intake.stoptakeCMD(), controller.leftBumper()::getAsBoolean));
    // retract intake
    operator.b().onTrue(deploy.readyCMD());
    operator.b().onFalse(deploy.deployCMD());

    operator
        .rightStick()
        .onTrue(
            deploy
                .manualCMD(operator::getRightY)
                .until(() -> !operator.rightStick().getAsBoolean()));

    /* HOPPER COMMANDS */

    // shoot
    controller
        .a()
        .onTrue(
            Commands.parallel(
                // Commands.runOnce(drive::stopWithX, drive), //uncomment for stopping while
                // shooting
                hopper.shootCMD()));
    // Commands.waitUntil(
    //     () -> shooter.atSpeed() || DriveCommands.aligned().getAsBoolean()),
    // spun up trigger

    controller.a().onFalse(hopper.stopCMD());

    // clear hopper
    operator.x().onTrue(hopper.backdriveCMD());
    operator.x().onFalse(hopper.stopCMD());

    controller.povUp().onTrue(DriveCommands.autoAlign(drive, DriveConstants.POSE_RESET));
    // drive
    //     .getPose()
    //     .plus(
    //         new Transform2d(
    //             new Translation2d(0.2, drive.getRotation()), drive.getRotation()))));

    /* SHOOTER COMMANDS */

    // This trigger probably goes off way too much - maybe make shooter.atSpeed() lock this at true?
    DriveCommands.aligned()
        .and(() -> shooter.atSpeed() && false) // Armaan, turn off rumble
        .onTrue(
            Commands.sequence(
                Commands.run(
                    () -> {
                      controller.getHID().setRumble(RumbleType.kLeftRumble, 1);
                      controller.getHID().setRumble(RumbleType.kRightRumble, 1);
                      SmartDashboard.putString("Rumble?", "Yes");
                    }),
                Commands.waitSeconds(1),
                Commands.run(
                    () -> {
                      controller.getHID().setRumble(RumbleType.kBothRumble, 0);
                      SmartDashboard.putString("Rumble?", "No");
                    })));

    // backup mannual flywheel spinup
    controller
        .rightTrigger(0.05)
        .onTrue(shooter.rawFlywheelCMD(() -> controller.getRightTriggerAxis() / 4));

    controller.povRight().onTrue(shooter.tuneCMD());

    controller.rightTrigger(0.05).onFalse(shooter.stopCMD());
    controller.rightBumper().onFalse(shooter.stopCMD());

    // increment backup shot length
    operator
        .povRight()
        .onTrue(
            Commands.runOnce(
                () -> {
                  mannualShotLength += 0.5;
                  if (mannualShotLength > 6) mannualShotLength = 1;
                  SmartDashboard.putNumber("Shooter/Mannual shot length", mannualShotLength);
                }));

    // flywheel pre-spin-up (not precise)
    operator.y().onTrue(shooter.flywheelGndCMD(() -> 6));

    /* CLIMBER COMMANDS */

    // hold left and right triggers for 0.5 seconds to auto-climb
    operator
        .leftBumper()
        .and(operator.rightBumper())
        .onTrue(
            Commands.sequence(
                Commands.waitSeconds(0.5),
                Commands.either(
                    DriveCommands.autoClimb(drive, climber),
                    Commands.none(),
                    operator.leftBumper().and(operator.rightBumper())::getAsBoolean)));

    // backup---raise and lower climber with trigger
    operator.leftTrigger(0.95).onTrue(climber.raiseCMD());
    operator.leftTrigger(0.1).onFalse(Commands.parallel(climber.pullCMD(), deploy.undeployCMD()));
    // operator.rightStick().onTrue(climber.manualCMD(() -> operator.getRightY())); // TODO
  }

  @SuppressWarnings("unused")
  private void configureShooterTestBindings() {
    testDistance = 1;
    Command shootCommand = shooter.flywheelGndCMD(() -> testDistance);

    controller
        .povUp()
        .onTrue(
            Commands.runOnce(
                () -> {
                  testDistance += 0.5;
                  if (testDistance > 5.5) testDistance = 1;
                }));

    controller.rightBumper().onTrue(shootCommand);

    controller
        .rightTrigger(0.05)
        .onTrue(shooter.rawFlywheelCMD(() -> controller.getRightTriggerAxis()));

    controller.a().onTrue(hopper.shootCMD());

    controller
        .b()
        .onTrue(
            Commands.sequence(
                Commands.waitUntil(
                    () -> {
                      return shooter.atSpeed() && shooter.getCurrentCommand() == shootCommand;
                    }),
                hopper.shootCMD()));

    controller.x().onTrue(hopper.backdriveCMD());

    controller.rightTrigger(0.05).onFalse(shooter.stopCMD());

    controller.a().onFalse(hopper.stopCMD());

    controller.b().onFalse(hopper.stopCMD());

    controller.rightBumper().onFalse(shooter.stopCMD());

    controller.x().onFalse(hopper.stopCMD());
  }

  public void teleopInit() {
    CommandScheduler.getInstance().schedule(Commands.sequence(hopper.stopCMD(), shooter.stopCMD()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }
}
