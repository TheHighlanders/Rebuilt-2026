// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Deploy extends SubsystemBase {
  /** Creates a new Deploy. */
  SparkMax deployMotor = new SparkMax(Constants.IntakeConstants.DEPLOYID, MotorType.kBrushless);

  SparkMaxConfig config = new SparkMaxConfig();

  public Deploy() {
    config.smartCurrentLimit(50).idleMode(IdleMode.kBrake);

    deployMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  // Adds start and stop for deploying
  public Command deployCMD() {
    // Deploys fuel
    return runOnce(
        () -> {
          deployMotor.set(Constants.IntakeConstants.DEPLOY_SPEED);
        });
  }
  // Stops motor
  public Command stopDeployCMD() {
    return runOnce(
        () -> {
          deployMotor.set(0);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
