// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 30.0;
  private static final double ANGLE_KI = 0;
  private static final double ANGLE_KD = 0;
  private static final double POS_KP = 10.0; // TODO
  private static final double POS_KI = 0;
  private static final double POS_KD = 0;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
  private static final double ALIGN_ANGLE_SPEED_TOLERANCE = 0.1;
  private static final double ALIGN_POS_SPEED_TOLERANCE = 0.03;

  private static final ProfiledPIDController angleController =
      new ProfiledPIDController(
          ANGLE_KP,
          ANGLE_KI,
          ANGLE_KD,
          new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
  private static final PIDController xController = new PIDController(POS_KP, POS_KI, POS_KD);
  private static final PIDController yController = new PIDController(POS_KP, POS_KI, POS_KD);

  private static Rotation2d pointAngle = Rotation2d.kZero;

  // @AutoLogOutput(key = "Auto/Target")
  private static Translation3d getAlignTarget(Drive drive) {
    Pose2d testPose = drive.getPose();
    Translation3d target;
    double fieldAlignX = 1;

    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      testPose = testPose.rotateAround(FieldConstants.CENTER, Rotation2d.k180deg);
      fieldAlignX = (FieldConstants.CENTER.getX() * 2) - fieldAlignX;
    }

    // returns the hub if the robot is inside the alliance side
    if (testPose.getMeasureX().in(Meters) < FieldConstants.HUB_POSE_BLUE.getX() + 0.597154) {
      target =
          new Translation3d(FieldConstants.HUB_POSE_BLUE); // .plus(new Translation2d(0.4, 0)));
      target = target.plus(new Translation3d(0, 0, FieldConstants.HUB_HEIGHT));
    }

    // otherwise, shoots towards outpost or depot, depending on robot position
    else if (testPose.getY() > FieldConstants.CENTER.getY()) {
      target = new Translation3d(FieldConstants.DEPOT_POSE_BLUE); // blue depot
    } else {
      target = new Translation3d(FieldConstants.OUTPOST_POSE_BLUE); // blue outpost
    }

    // account for alliance side
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      target =
          target.rotateAround(
              new Translation3d(FieldConstants.CENTER), new Rotation3d(Rotation2d.k180deg));
    }

    /* MOVEMENT COMP */
    // https://www.desmos.com/calculator/2jxmstl1qs

    // robot-relative target pose
    Translation2d movingTarget = target.toTranslation2d().minus(drive.getPose().getTranslation());

    // robot linear velocity
    Translation2d robotLinVel =
        new Translation2d(drive.getSpeeds().vxMetersPerSecond, drive.getSpeeds().vyMetersPerSecond)
            .rotateBy(drive.getRotation());

    // delta distance
    double dd =
        -((movingTarget.getX() * robotLinVel.getX()) + (movingTarget.getY() * robotLinVel.getY()))
            / movingTarget.getNorm();

    // airtime
    double airtime =
        (dd
                + Math.sqrt(
                    (dd * dd)
                        - (2
                            * ShooterConstants.GRAVITY
                            * ShooterConstants.HOOD_SLOPE
                            * ((target.getZ() * ShooterConstants.HOOD_SLOPE)
                                - movingTarget.getNorm()))))
            / (2 * ShooterConstants.GRAVITY * ShooterConstants.HOOD_SLOPE);

    Translation3d movementComp = new Translation3d(robotLinVel.times(airtime * 2));

    SmartDashboard.putNumber("Auto Align/robot linear velocity", robotLinVel.getNorm());
    SmartDashboard.putNumber("Auto Align/dd over dt", dd);
    SmartDashboard.putNumber("Auto Align/distance", movingTarget.getNorm());
    SmartDashboard.putNumber("Auto Align/airtime", airtime);

    Logger.recordOutput("robot speeds", new Pose2d(robotLinVel, Rotation2d.kZero));

    return target.minus(movementComp);
  }

  public static Trigger aligned() {
    return new Trigger(angleController::atSetpoint);
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  private static Rotation2d getAngleFromJoysticks(double angleXSupplier, double angleYSupplier) {
    if (Math.hypot(angleXSupplier, angleYSupplier) > DriveConstants.POINT_DEADBAND) {
      pointAngle = Rotation2d.fromRadians(Math.atan2(angleYSupplier, angleXSupplier));
    }
    return pointAngle;
  }

  private static Pose2d[] getAutoClimbAlignSequence(boolean red, boolean outpost) {
    Pose2d[] alignSeq = {null, null};

    // if the robot is totally on the outpost side of the tower, align to the outpost side.
    // otherwise, depot side.
    if (outpost) {
      alignSeq[0] =
          new Pose2d(
              FieldConstants.CLIMB_OUTPOST_CORNER
                  .minus(DriveConstants.TO_CORNER_BUMPERS)
                  .plus(new Translation2d(0, -0.1)),
              Rotation2d.kCW_90deg);
      alignSeq[1] =
          new Pose2d(
              FieldConstants.CLIMB_OUTPOST_CORNER
                  .minus(DriveConstants.TO_CORNER_BUMPERS)
                  .plus(new Translation2d(0.1, 0)),
              Rotation2d.kCW_90deg);
    } else {
      alignSeq[0] =
          new Pose2d(
              FieldConstants.CLIMB_DEPOT_CORNER
                  .plus(DriveConstants.TO_CORNER_BUMPERS)
                  .plus(new Translation2d(0, 0.1)),
              Rotation2d.kCCW_90deg);
      alignSeq[1] =
          new Pose2d(
              FieldConstants.CLIMB_DEPOT_CORNER
                  .plus(DriveConstants.TO_CORNER_BUMPERS)
                  .plus(new Translation2d(-0.1, 0)),
              Rotation2d.kCCW_90deg);
    }

    // reflect the align positions back if we're on the red alliance side
    if (red) {
      alignSeq[0] = alignSeq[0].rotateAround(FieldConstants.CENTER, Rotation2d.k180deg);
      alignSeq[1] = alignSeq[1].rotateAround(FieldConstants.CENTER, Rotation2d.k180deg);
    }

    if (alignSeq[1] == null) {
      SmartDashboard.putString("Drive/AutoAlign", "FAIL");
    } else {
      SmartDashboard.putString("Drive/AutoAlign", "SUCCESS");
    }

    return alignSeq;
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier robotRelative) {
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Apply rotation deadband
              double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

              // Square rotation value for more precise control
              omega = Math.copySign(omega * omega, omega);

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega * drive.getMaxAngularSpeedRadPerSec());

              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;

              drive.runVelocity(
                  robotRelative.getAsBoolean()
                      ? ChassisSpeeds.fromRobotRelativeSpeeds(speeds, Rotation2d.fromDegrees(0))
                      : ChassisSpeeds.fromFieldRelativeSpeeds(
                          speeds,
                          isFlipped
                              ? drive.getRotation().plus(new Rotation2d(Math.PI))
                              : drive.getRotation()));
            },
            drive)
        .withName("Joystic Drive");
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier,
      BooleanSupplier robotRelative) {
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    angleController.setTolerance(Units.degreesToRadians(5));

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);

              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;

              drive.runVelocity(
                  robotRelative.getAsBoolean()
                      ? ChassisSpeeds.fromRobotRelativeSpeeds(speeds, Rotation2d.fromDegrees(0))
                      : ChassisSpeeds.fromFieldRelativeSpeeds(
                          speeds,
                          isFlipped
                              ? drive.getRotation().plus(new Rotation2d(Math.PI))
                              : drive.getRotation()));

              Logger.recordOutput(
                  "Drive/Align/drive angle error",
                  rotationSupplier.get().minus(drive.getRotation()));
              Logger.recordOutput("Drive/Align/drive angle target", rotationSupplier.get());
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()))
        .withName("Joystic Drive At Angle");
  }

  /** Aligns shooter and robot towards color hub */
  public static Command joystickAlignDrive(
      Drive drive,
      Shooter shooter,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      BooleanSupplier robotRelative) {

    Supplier<Translation3d> targetFR = () -> getAlignTarget(drive);

    return Commands.parallel(
            joystickDriveAtAngle(
                drive,
                xSupplier,
                ySupplier,
                () ->
                    Rotation2d.fromRadians(
                        Math.atan2(
                                targetFR.get().getY() - drive.getPose().getY(),
                                targetFR.get().getX() - drive.getPose().getX())
                            + DriveConstants.ALIGN_SHOOTER_COMP.in(Radians)),
                robotRelative),
            shooter.flywheelCMD(
                () ->
                    new Translation2d(
                        drive
                            .getPose()
                            .getTranslation()
                            .getDistance(targetFR.get().toTranslation2d()),
                        targetFR.get().getZ())),
            // shooter.flywheelCMD(
            //     () -> {
            //       return new Translation2d(
            //           drive.getPose().getTranslation().getDistance(targetFR.get()),
            //           targetFR.get().equals(FieldConstants.HUB_POSE_BLUE)
            //                   || targetFR.get().equals(FieldConstants.HUB_POSE_RED)
            //               ? FieldConstants.HUB_HEIGHT
            //               : 0);
            //     }),
            Commands.run(
                () ->
                    Logger.recordOutput(
                        "Auto/Align Target",
                        new Pose2d(targetFR.get().toTranslation2d(), Rotation2d.kZero))))
        .withName("Joystic Align Drive");
  }

  /** Aligns shooter and robot towards color hub */
  public static Command joystickAlignDriveHub(
      Drive drive,
      Shooter shooter,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      BooleanSupplier robotRelative) {

    Supplier<Translation2d> target =
        () -> {
          Translation2d movementComp = Translation2d.kZero;
          // new Translation2d(drive.getSpeeds().vxMetersPerSecond,
          // drive.getSpeeds().vyMetersPerSecond)
          //     .rotateBy(drive.getPose().getRotation())
          //     .div(3);
          return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
              ? FieldConstants.HUB_POSE_RED.minus(movementComp)
              : FieldConstants.HUB_POSE_BLUE.minus(movementComp);
        };

    return Commands.parallel(
            joystickDriveAtAngle(
                drive,
                xSupplier,
                ySupplier,
                () ->
                    Rotation2d.fromRadians(
                        Math.atan2(
                                target.get().getY() - drive.getPose().getY(),
                                target.get().getX() - drive.getPose().getX())
                            + DriveConstants.ALIGN_SHOOTER_COMP.in(Radians)),
                robotRelative),
            shooter.flywheelHubCMD(
                () -> {
                  return drive.getPose().getTranslation().getDistance(target.get());
                }),
            Commands.run(
                () ->
                    Logger.recordOutput(
                        "Auto/Align Target", new Pose2d(target.get(), Rotation2d.kZero))))
        .withName("Joystic Align Drive");
  }

  /**
   * Field relative drive command where joystick controlls linear velocity and robot points in the
   * direction of the rotation joystick.
   */
  public static Command joystickPointDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier angleXSupplier,
      DoubleSupplier angleYSupplier,
      BooleanSupplier robotRelative) {
    pointAngle = drive.getRotation();

    return Commands.parallel(
            joystickDriveAtAngle(
                drive,
                xSupplier,
                ySupplier,
                () ->
                    getAngleFromJoysticks(
                        angleXSupplier.getAsDouble(), angleYSupplier.getAsDouble()),
                robotRelative),
            Commands.run(
                () -> {
                  Logger.recordOutput(
                      "Drive/point angle",
                      getAngleFromJoysticks(
                          angleXSupplier.getAsDouble(), angleYSupplier.getAsDouble()));
                }))
        .withName("Joystic Point Drive");
  }

  public static Command autoAlign(Drive drive, Pose2d pose) { // TODO: Tune PID

    xController.setTolerance(0.05);
    yController.setTolerance(0.05);

    return Commands.run(
            () -> {
              // Get linear velocity
              double xSpeed = xController.calculate(drive.getPose().getX());

              double ySpeed = yController.calculate(drive.getPose().getY());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), pose.getRotation().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, omega); // ySpeed, omega);

              // boolean isFlipped =
              //     DriverStation.getAlliance().isPresent()
              //         && DriverStation.getAlliance().get() == Alliance.Red;

              drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));

              Logger.recordOutput("Drive/Align/Target", pose);
              Logger.recordOutput("Drive/Align/Error", pose.minus(drive.getPose()));
            },
            drive)
        .until(
            () -> {
              return xController.atSetpoint()
                  && yController.atSetpoint()
                  && angleController.atSetpoint()
                  && drive.getSpeeds().omegaRadiansPerSecond < ALIGN_ANGLE_SPEED_TOLERANCE
                  && Math.atan2(
                          drive.getSpeeds().vyMetersPerSecond, drive.getSpeeds().vxMetersPerSecond)
                      < ALIGN_POS_SPEED_TOLERANCE;
            })
        .andThen(Commands.runOnce(() -> drive.stop()))

        // Reset PID controller when command starts
        .beforeStarting(
            () -> {
              angleController.reset(drive.getRotation().getRadians());
              xController.setSetpoint(pose.getX());
              yController.setSetpoint(pose.getY());
            })
        .withName("Auto-Align");
  }

  private static Command autoAlign(Drive drive, Pose2d[] poses) {
    Command alignCMD = Commands.waitSeconds(0);
    for (Pose2d pose : poses) {
      alignCMD = alignCMD.andThen(autoAlign(drive, pose));
    }
    return alignCMD;
  }

  public static Command autoClimb(Drive drive, Climber climber) {
    return Commands.either(
        Commands.either(
            autoClimb(drive, climber, getAutoClimbAlignSequence(true, true))
                .beforeStarting(
                    Commands.runOnce(
                        () -> {
                          SmartDashboard.putString("Drive/AutoClimb", "Red Outpost");
                        })),
            autoClimb(drive, climber, getAutoClimbAlignSequence(true, false))
                .beforeStarting(
                    Commands.runOnce(
                        () -> {
                          SmartDashboard.putString("Drive/AutoClimb", "Red Depot");
                        })),
            () ->
                drive
                    .getPose()
                    .rotateAround(FieldConstants.CENTER, Rotation2d.k180deg)
                    .getMeasureY()
                    .lt(
                        FieldConstants.CLIMB_OUTPOST_CORNER
                            .minus(DriveConstants.TO_CORNER_BUMPERS)
                            .getMeasureY())),
        Commands.either(
            autoClimb(drive, climber, getAutoClimbAlignSequence(false, true))
                .beforeStarting(
                    Commands.runOnce(
                        () -> {
                          SmartDashboard.putString("Drive/AutoClimb", "Blue Outpost");
                        })),
            autoClimb(drive, climber, getAutoClimbAlignSequence(false, false))
                .beforeStarting(
                    Commands.runOnce(
                        () -> {
                          SmartDashboard.putString("Drive/AutoClimb", "Blue Depot");
                        })),
            () ->
                drive
                    .getPose()
                    .getMeasureY()
                    .lt(
                        FieldConstants.CLIMB_OUTPOST_CORNER
                            .minus(DriveConstants.TO_CORNER_BUMPERS)
                            .getMeasureY())),
        () -> {
          return drive.getPose().getMeasureX().in(Meters) > FieldConstants.CENTER.getX();
        });
  }

  // auto climb tool that aligns to the tower given a sequence of Pose2ds, then pulls the robot up.
  private static Command autoClimb(Drive drive, Climber climber, Pose2d[] autoClimbSequence) {

    return Commands.either(
        Commands.sequence(
                Commands.deadline( // parallel
                    autoAlign(drive, autoClimbSequence), climber.raiseCMD()),
                Commands.deadline(
                    Commands.sequence(Commands.waitSeconds(0.5), climber.pullCMD()),
                    joystickDriveAtAngle(
                        drive, () -> 0, () -> 0.3, () -> drive.getRotation(), () -> true)))
            .beforeStarting(
                Commands.runOnce(
                    () -> {
                      SmartDashboard.putString("Drive/AutoClimbOff", "No---Climbing");
                    })),
        Commands.none()
            .beforeStarting(
                Commands.runOnce(
                    () -> {
                      SmartDashboard.putString("Drive/AutoClimbOff", "Yes---FAIL");
                    })),
        () -> {
          Pose2d testPose = drive.getPose();
          if (drive.getPose().getMeasureX().in(Meters) > FieldConstants.CENTER.getX()) {
            testPose = testPose.rotateAround(FieldConstants.CENTER, Rotation2d.k180deg);
          }
          // returns if the robot is inside the alliance side
          return testPose.getMeasureX().in(Meters) < FieldConstants.HUB_POSE_BLUE.getX() - 0.597154;
        });
  }

  public static Command joystickGyroOverride(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      DoubleSupplier gyroXSupplier,
      DoubleSupplier gyroYSupplier,
      BooleanSupplier robotRelative) {
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Apply rotation deadband
              double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

              // Square rotation value for more precise control
              omega = Math.copySign(omega * omega, omega);

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega * drive.getMaxAngularSpeedRadPerSec());

              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;

              // reset gyro
              drive.setPose(
                  new Pose2d(
                      drive.getPose().getTranslation(),
                      getAngleFromJoysticks(
                          gyroXSupplier.getAsDouble(), gyroYSupplier.getAsDouble())));

              drive.runVelocity(
                  robotRelative.getAsBoolean()
                      ? ChassisSpeeds.fromRobotRelativeSpeeds(speeds, Rotation2d.fromDegrees(0))
                      : ChassisSpeeds.fromFieldRelativeSpeeds(
                          speeds,
                          isFlipped
                              ? drive.getRotation().plus(new Rotation2d(Math.PI))
                              : drive.getRotation()));
            },
            drive)
        .withName("Joystic Drive");
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}
