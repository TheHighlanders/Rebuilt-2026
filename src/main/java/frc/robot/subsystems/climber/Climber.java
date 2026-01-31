// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  SparkMax climbMotor = new SparkMax( ClimberConstants.CLIMBERID, MotorType.kBrushless);
  /** Creates a new Climber. */
  public Climber() {}

public Command  climberCMD() {
  return runOnce(
() -> {
          climbMotor.set(ClimberConstants.CLIMBERID);
        });
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
