// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Deploy extends SubsystemBase {
  /** Creates a new Deploy. */
  SparkMax deployMotor = new SparkMax(IntakeConstants.DEPLOYID, MotorType.kBrushless);
  RelativeEncoder deployEncoder = deployMotor.getEncoder();

  SparkMaxConfig config = new SparkMaxConfig();

  public Deploy() {
    config.smartCurrentLimit(50).idleMode(IdleMode.kBrake);

    deployMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    deployEncoder.setPosition(0);
  }
  // Adds start and stop for deploying
  public Command deployCMD() {
    // Deploys fuel
    return Commands.deadline(
            Commands.waitUntil(() -> deployEncoder.getPosition() == IntakeConstants.DEPLOY_POSITION),
              run(
                () -> {
                  deployMotor.set(IntakeConstants.DEPLOY_SPEED);
                })
            )
            .andThen(
            run(
              () -> {
                deployMotor.set(0);
              }
            ));
  }
  // Stops motor
  public Command undeployCMD() {
    return Commands.deadline(
            Commands.waitUntil(() -> deployEncoder.getPosition() == 0),
              run(
                () -> {
                  deployMotor.set(0-IntakeConstants.DEPLOY_SPEED);
                })
            )
            .andThen(
            run(
              () -> {
                deployMotor.set(0);
              }
            ));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
