// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.Constants.VisionConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  SparkMax flywheel = new SparkMax(Constants.ShooterConstants.SHOOTERID, MotorType.kBrushless);

  SparkMax kicker = new SparkMax(Constants.ShooterConstants.KICKERID, MotorType.kBrushless);

  // pid
  SparkClosedLoopController shootController = flywheel.getClosedLoopController();

  // Encoder: A sensor that measures the amount of rotations
  RelativeEncoder flywheelEncoder;

  // PID
  double kP = 0.1;
  double kI = 0.0;
  double kD = 3.0;
  double targetRPM = 0.0;

  // pid config
  SparkMaxConfig ShooterConfig = new SparkMaxConfig();

  public Shooter() {

    // initialize encoder
    flywheelEncoder = flywheel.getEncoder();

    // Set PID gains
    ShooterConfig.closedLoop.p(kP).i(kI).d(kD);
    // dropper config
    flywheel.configure(
        ShooterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    SmartDashboard.putNumber("PID/Shooter/kP", kP);
    SmartDashboard.putNumber("PID/Shooter/kI", kI);
    SmartDashboard.putNumber("PID/Shooter/kD", kD);
    SmartDashboard.putNumber("PID/Shooter/Target RPM", 0.0);
    SmartDashboard.putNumber("PID/KickWheel/Target RPM", 0.0);
  }

  public void intake() {}

  protected double calculate(double distanceMeters) {
    // TODO: wait until shooter is finalized
    double linearVelocity = 1.4 * distanceMeters + 6.1;
    return linearVelocity * 193; // trust that makes it angular i did the (?) math:
    // https://www.desmos.com/calculator/zroouacb64
  }

  public boolean atSpeed() {
    return shootController.isAtSetpoint();
  }

  public Command flywheelCMD(DoubleSupplier distance) {

    return Commands.run(
        () -> {
          targetRPM = calculate(distance.getAsDouble());
          SmartDashboard.putNumber("PID/Shooter/Target RPM", targetRPM);
          shootController.setSetpoint(targetRPM, ControlType.kVelocity);
        },
        this);
  }

  public Command kickerCMD() {
    return runOnce(() -> kicker.set(1));
  }

  public Command stopCMD() {
    return Commands.runOnce(
        () -> {
          kicker.set(0);
          targetRPM = 0;
        },
        this);
  }

  @SuppressWarnings("unused")
  @Override
  public void periodic() {

    // if within 5 ft of AprilTag 25, then set targetRPM to low number. else shoot far-ish;
    // Translation2d robot = getPose.get().getTranslation();
    // Translation2d hub = aprilTagLayout.getTagPose(25).get().toPose2d().getTranslation();

    // double distFromHub = hub.getDistance(robot);

    // SmartDashboard.putNumber("Shooter/distanceFromHub", distFromHub);
    // if (distFromHub < 1.524) {
    //   targetRPM = 500;
    // } else {
    //   targetRPM = 2000;
    // }

    shootController.setSetpoint(targetRPM, ControlType.kVelocity);

    SmartDashboard.putNumber("Shooter/targetRPM", targetRPM);
    // For Elastic and Advtange Scope
    double newP = SmartDashboard.getNumber("PID/Shooter/kP", kP);
    double newI = SmartDashboard.getNumber("PID/Shooter/kI", kI);
    double newD = SmartDashboard.getNumber("PID/Shooter/kD", kD);
    // double newTTargetRPM = SmartDashboard.getNumber("PID/Shooter/Target RPM", targetRPM);

    SmartDashboard.putNumber(
        "PID/Shooter/Dropper Output", flywheelEncoder.getPosition()); // Setpoint

    SmartDashboard.putNumber("PID/Shooter/Target RPM", shootController.getSetpoint()); //
    // Setpoint
    SmartDashboard.putNumber(
        "PID/Shooter/Dropper Velocity", flywheelEncoder.getVelocity()); // Actual velocity

    // SmartDashboard.updateValues();
  }
}
