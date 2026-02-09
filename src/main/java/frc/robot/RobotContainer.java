// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.VisionConstants.*;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.deploy.Deploy;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
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
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Dashboard inputs
  //  private final LoggedDashboardChooser<Command> autoChooser;
  private final AutoChooser autoChooser;

  private boolean robotRelative;

  private final Shooter m_Shooter;

  private Command testVisionSim;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Define Shooter
    m_Shooter = new Shooter();
    // Define Hopper
    hopper = new Hopper();
    // Define Climber
    climber = new Climber();

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
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1),
                new VisionIOPhotonVision(camera2Name, robotToCamera2),
                new VisionIOPhotonVision(camera3Name, robotToCamera3));

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
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose),
                new VisionIOPhotonVisionSim(camera2Name, robotToCamera2, drive::getPose),
                new VisionIOPhotonVisionSim(camera3Name, robotToCamera3, drive::getPose)
                );
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

        break;
    }
    intake = new Intake(); // Fits outside because it's the same in both Real and Sim.
    deploy = new Deploy();

    testVisionSim = Commands.runOnce(
                        () -> SmartDashboard.putNumber("failed tests", 0)
                    )
                    .andThen(Commands.run(
                        () -> {
                            if (drive.getPose().getMeasureX().in(Meters) > 10) {
                                
                            }
                            else if (drive.getPose().getMeasureY().in(Meters) > 8.5) {
                                drive.setPose(new Pose2d(drive.getPose().getX() + 0.5, 0, Rotation2d.fromDegrees(0)));
                                
                            }
                            else if (drive.getPose().getRotation().getDegrees() < -15 && drive.getPose().getRotation().getDegrees() > -45) {
                                drive.setPose(new Pose2d(drive.getPose().getX(), drive.getPose().getY() + 0.5, Rotation2d.fromDegrees(0)));
                                
                            }
                            else drive.setPose(drive.getPose().plus(new Transform2d(0, 0, Rotation2d.fromDegrees(36))));
                            if (!vision.hasTarget()) {
                                SmartDashboard.putNumber("failed tests", SmartDashboard.getNumber("failed tests", 0) + 1);
                            }
                    }, drive, vision));

    // Set up auto routines
    autos = new Autos(drive);
    //  autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoFactory.buildAutoChooser());
    autoChooser = new AutoChooser();
    autoChooser.addRoutine(
        "THE WORKING AUTO", () -> autos.shootTwiceRoutine()); // Set up SysId routines

    SmartDashboard.putData("CHOREO", autoChooser);
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

    // Configure the button bindings
    configureButtonBindings();
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

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero,
                () -> robotRelative));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
    controller
        .x()
        .whileTrue(
            DriveCommands.joystickOrbitDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                aprilTagLayout.getTagPose(25).get().toPose2d(),
                () -> robotRelative) // / get position of april tag on blue basket
            );

    controller
        .povDown()
        .onTrue(
            Commands.runOnce(
                () -> {
                  robotRelative = !robotRelative;
                  SmartDashboard.putBoolean("Robot Relative Drive", robotRelative);
                }));
    /**
     * DriveCommands.joystickOrbitDrive( drive, () -> -controller.getLeftY(), () ->
     * -controller.getLeftX(), aprilTagLayout.getTagPose(28).get().toPose2d()));// Pose2d(5, 5,
     * Rotation2d.kZero)));
     */

    /* operator controlls port 1 */

    operator.b().onTrue(intake.intakeCMD());
    operator.b().onFalse(intake.stoptakeCMD());

    operator.y().onTrue(intake.spitakeCMD());
    operator.y().onFalse(intake.stoptakeCMD());

    operator.a().onTrue(deploy.deployCMD());
    operator.a().onFalse(deploy.stopDeployCMD());

    operator.rightStick().onTrue(climber.downCMD());
    operator.rightStick().onFalse(climber.stopClimbCMD());

    // Hold down the button to climb.
    operator.x().onTrue(climber.climberCMD());
    operator.x().onFalse(climber.stopClimbCMD());
    // activates the shooter without the hopper, meant for unclogging the shooter or if something
    // goes wrong.
    controller.leftBumper().onFalse(m_Shooter.PIDCMD(500));
    controller.leftBumper().onTrue(m_Shooter.PIDCMD(0));
    // activates the shooter and hopper, meant for shooting fuel.
    controller.rightBumper().onFalse(Commands.parallel(hopper.StopCMD(), m_Shooter.PIDCMD(0)));
    controller.rightBumper().onTrue(Commands.parallel(hopper.SpinCMD(), m_Shooter.PIDCMD(500)));
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
