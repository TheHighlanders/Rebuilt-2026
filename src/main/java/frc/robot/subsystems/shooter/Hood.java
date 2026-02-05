// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {
  /** Creates a new Hood. */
  SparkMax hoodMax = new SparkMax(4, MotorType.kBrushless);

  // To use a SparkMax, we create a SparkMax object. To use PID, we have to use the PID Object.
  // REVLib have their own PID Object called SparkClosedLoopController.
  SparkClosedLoopController hoodController = hoodMax.getClosedLoopController();

  // Encoder: A sensor that measures the amount of rotations
  RelativeEncoder hoodEncoder;

  // PID
  double kP = 0.1;
  double kI = 0.0;
  double kD = 3.0;
  double targetRPM = 0.0;

  // pid config
  SparkMaxConfig hoodConfig = new SparkMaxConfig();

  public Hood() {
    // initialize encoder
    hoodEncoder = hoodMax.getEncoder();

    // Set PID gains
    hoodConfig.closedLoop.p(kP).i(kI).d(kD);

    // dropper config
    hoodMax.configure(
        hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    // For Elastic and Advantage

    SmartDashboard.putNumber("PID/Hood/kP", kP);
    SmartDashboard.putNumber("PID/Hood/kI", kI);
    SmartDashboard.putNumber("PID/Hood/kD", kD);
    SmartDashboard.putNumber("PID/Hood/Target RPM", targetRPM);
  }

  /*
   * Hint: New Commands and Methods go here
   */
  public Command PIDCMD(double newTargetRPM) {

    // advantage scope (?)
    targetRPM = newTargetRPM;

    return runOnce(
        () -> {
          hoodController.setSetpoint(newTargetRPM, ControlType.kVelocity);
        });
  }

  @SuppressWarnings("unused")
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // For Elastic and Advtange Scope
    double newP = SmartDashboard.getNumber("PID/Hood/kP", kP);
    double newI = SmartDashboard.getNumber("PID/Hood/kI", kI);
    double newD = SmartDashboard.getNumber("PID/Hood/kD", kD);
    double newTargetRPM = SmartDashboard.getNumber("Target RPM", targetRPM);

    SmartDashboard.putNumber("PID/Hood/Dropper Setpoint", targetRPM); // Setpoint
    SmartDashboard.putNumber(
        "PID/Hood/Dropper Velocity", hoodEncoder.getVelocity()); // Actual velocity
  }
}
