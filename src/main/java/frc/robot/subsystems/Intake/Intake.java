// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  SparkMax intakeMotor = new SparkMax(IntakeConstants.SPINTAKEID, MotorType.kBrushless);
  SparkClosedLoopController controller = intakeMotor.getClosedLoopController();

  public Intake() {

    intakeMotor.configure(getConfig(IdleMode.kCoast), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private SparkMaxConfig getConfig(IdleMode idleMode) {
    SparkMaxConfig config =  new SparkMaxConfig();
    config
      .smartCurrentLimit(50)
      .idleMode(idleMode);

    return config;
  }
  // Intake commands to take in, spit out, and not move
  public Command intakeCMD() {
    // Takes in
    return runOnce(
        () -> {
          intakeMotor.configure(getConfig(IdleMode.kCoast), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);          
          intakeMotor.set(IntakeConstants.INTAKE_SPEED);
        });
  }
  // Spits out
  public Command spitakeCMD() {
    return runOnce(
        () -> {
          intakeMotor.set(IntakeConstants.SPITAKE_SPEED);
        });
  }

  public Command stoptakeCMD() {
    // Stops motor
    return runOnce(
        () -> {
          intakeMotor.set(0);
        });
  }

  public Command killCMD() {
    return runOnce(
      () -> {
        intakeMotor.configure(getConfig(IdleMode.kBrake), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      }
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
