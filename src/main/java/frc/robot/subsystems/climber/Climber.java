// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  SparkMax climbMotor = new SparkMax(ClimberConstants.CLIMBERID, MotorType.kBrushless);
  RelativeEncoder climbEncoder = climbMotor.getEncoder();
  double climberspeed = 0.6;
  /** Creates a new Climber. */
  public Climber() {
    climbEncoder.setPosition(0);
  }

  private Command runCMD(double speed) {
    return run(
        () -> {
          climbMotor.set(speed);
        });
  }

  public Command raiseCMD() {
    // Deploys fuel
    return Commands.deadline(
            Commands.waitUntil(
                () ->
                    climbEncoder.getPosition()
                        >= ClimberConstants.UP_POSITION - ClimberConstants.POS_TOLERANCE),
            runCMD(ClimberConstants.RAISE_SPEED))
        .andThen(runCMD(0));
  }

  public Command pullCMD() {
    // Deploys fuel
    return Commands.deadline(
            Commands.waitUntil(() -> climbEncoder.getPosition() <= ClimberConstants.POS_TOLERANCE),
            runCMD(ClimberConstants.PULL_SPEED))
        .andThen(runCMD(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
