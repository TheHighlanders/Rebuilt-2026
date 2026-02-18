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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.*;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
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
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

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
            new Drive(
                new GyroIONavX(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                new VisionIOPhotonVision(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1),
                new VisionIOPhotonVision(
                    VisionConstants.camera2Name, VisionConstants.robotToCamera2),
                new VisionIOPhotonVision(
                    VisionConstants.camera3Name, VisionConstants.robotToCamera3));
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
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera2Name, VisionConstants.robotToCamera2, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera3Name, VisionConstants.robotToCamera3, drive::getPose));
        ShooterSim temp = new ShooterSim(fuelSim); // how do i destruct this
        shooter = temp;
        hopper = new HopperSim(temp);

        fuelSim.spawnStartingFuel(false);

        // Register a robot for collision with fuel
        fuelSim.registerRobot(
            0.6858, // from left to right in meters
            0.6858, // from front to back in meters
            0.1, // from floor to top of bumpers in meters
            drive::getPose, // Supplier<Pose2d> of robot pose
            drive::getSpeeds); // Supplier<ChassisSpeeds> of field-centric chassis speeds

        // Register an intake to remove fuel from the field as a rectangular bounding box
        fuelSim.registerIntake(
            0.35,
            0.45,
            -0.35,
            0.35,
            () -> HopperSim.intake()); // robot-centric coordinates for bounding box in meters

        fuelSim.setSubticks(
            3); // sets the number of physics iterations to perform per 20ms loop. Default = 5

        fuelSim
            .enableAirResistance(); // an additional drag force will be applied to fuel in physics
        // update step

        fuelSim
            .start(); // enables the simulation to run (updateSim must still be called periodically)

        configureAutoButtonBindings();

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
    autoChooser.addRoutine("Depot + Climb", () -> autos.depotAndClimb(true));
    autoChooser.addRoutine("Depot", () -> autos.depotAndClimb(false));
    autoChooser.addRoutine("Outpost + Climb", () -> autos.outpostAndClimb(true));
    autoChooser.addRoutine("Outpost", () -> autos.outpostAndClimb(false));

    SmartDashboard.putData("CHOREO", autoChooser);
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

    // Configure the button bindings
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    robotRelative = false;
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX(),
            () -> robotRelative));

    // Lock to 0° when Y button is held - why?
    controller
        .y()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero,
                () -> robotRelative));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // align to shoot on right bumper
    controller
        .rightBumper()
        .onTrue(
            Commands.parallel(
                    DriveCommands.joystickAlignDrive(
                        drive,
                        shooter,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> robotRelative), // / get position of april tag on blue basket,
                    Commands.runOnce(
                        () -> {
                          SmartDashboard.putString(
                              "shooter/aim",
                              DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                                  ? "RED"
                                  : "BLUE");
                        }))
                .until(() -> !controller.rightBumper().getAsBoolean()));

    // runs kicker and hopper if flywheel is at speed. If flywheel is not being spun up, spits out
    // fuel.
    controller
        .a()
        .onTrue(
            Commands.either(
                Commands.either(hopper.shootCMD(), Commands.none(), shooter::atSpeed),
                Commands.sequence(shooter.flywheelCMD(() -> 10), hopper.shootCMD()),
                controller.rightBumper()::getAsBoolean));

    controller.a().onFalse(Commands.sequence(hopper.stopCMD(), shooter.stopCMD()));
    /**
     * DriveCommands.joystickOrbitDrive( drive, () -> -controller.getLeftY(), () ->
     * -controller.getLeftX(), aprilTagLayout.getTagPose(28).get().toPose2d()));// Pose2d(5, 5,
     * Rotation2d.kZero)));
     */

    /* operator controlls port 1 */

    // runs intake
    operator.b().onTrue(intake.intakeCMD());
    operator.b().onFalse(intake.stoptakeCMD());

    // runs intake backwards
    operator.y().onTrue(intake.spitakeCMD());
    operator.y().onFalse(intake.stoptakeCMD());

    // deploys intake
    operator.povRight().toggleOnTrue(deploy.deployCMD());
    operator.povRight().toggleOnFalse(deploy.undeployCMD());

    // climbs
    operator.povUp().onTrue(climber.raiseCMD());
    operator.povDown().onTrue(climber.pullCMD());

    // toggles between robot- and field-relative drive
    operator
        .povDown()
        .onTrue(
            Commands.runOnce(
                () -> {
                  robotRelative = !robotRelative;
                  SmartDashboard.putBoolean("Robot Relative Drive", robotRelative);
                }));

    // slow mode
    controller
        .povLeft()
        .onTrue(
            DriveCommands.joystickDrive(
                    drive,
                    () -> {
                      return controller.getLeftY() * DriveConstants.SLOWMODE;
                    },
                    () -> {
                      return controller.getLeftX() * DriveConstants.SLOWMODE;
                    },
                    () -> {
                      return controller.getRightX() * DriveConstants.SLOWMODE;
                    },
                    () -> robotRelative)
                .until(() -> !controller.rightBumper().getAsBoolean()));
    // activates the shooter without the hopper, meant for unclogging the shooter or if something
    // goes wrong.
    // activates the shooter and hopper, meant for shooting fuel.
    operator.rightBumper().onTrue(hopper.clearCMD().andThen(shooter.flywheelCMD(() -> 10)));
    operator.rightBumper().onFalse(hopper.stopCMD());
  }

  public void configureAutoButtonBindings() {
    robotRelative = false;
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX(),
            () -> robotRelative));

    // Lock to 0° when Y button is held - why?
    controller
        .y()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero,
                () -> robotRelative));

    // align to shoot on right bumper
    controller
        .rightBumper()
        .onTrue(
            Commands.parallel(
                    DriveCommands.joystickAlignDrive(
                        drive,
                        shooter,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> robotRelative), // / get position of april tag on blue basket,
                    Commands.runOnce(
                        () -> {
                          SmartDashboard.putString(
                              "shooter/aim",
                              DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                                  ? "RED"
                                  : "BLUE");
                        }))
                .until(() -> !controller.rightBumper().getAsBoolean()));

    // runs kicker and hopper if flywheel is at speed. If flywheel is not being spun up, spits out
    // fuel.
    controller
        .b()
        .onTrue(
            Commands.either(
                Commands.either(hopper.shootCMD(), Commands.none(), shooter::atSpeed),
                Commands.sequence(shooter.flywheelCMD(() -> 10), hopper.shootCMD()),
                controller.rightBumper()::getAsBoolean));

    controller.b().onFalse(Commands.sequence(hopper.stopCMD(), shooter.stopCMD()));

    // toggles between robot- and field-relative drive
    controller
        .povDown()
        .onTrue(
            Commands.runOnce(
                () -> {
                  robotRelative = !robotRelative;
                  SmartDashboard.putBoolean("Robot Relative Drive", robotRelative);
                }));
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
