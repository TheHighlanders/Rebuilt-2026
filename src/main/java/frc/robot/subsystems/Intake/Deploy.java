// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Deploy extends SubsystemBase {
  /** Creates a new Deploy. */
  SparkMax deployMotor = new SparkMax(1, MotorType.kBrushless);

  public Deploy() {}

  public void InandOut() {
    deployMotor.set(Constants.IntakeConstants.DEPLOYID);
  }
  public void stopDeploy() {
    deployMotor.set(0);
  }
  public Command deployCMD() {
    return runOnce(
        () -> {
          InandOut();
        });
  }
    public Command stopDeployCMD() {
    return runOnce(
        () -> {
          stopDeploy();
        });
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
