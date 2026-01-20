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

public class Intake extends SubsystemBase {
  SparkMax intakeMotor1 =
      new SparkMax(Constants.IntakeConstants.INTAKEID, MotorType.kBrushless); // 10 and
  SparkMax intakeMotor2 =
      new SparkMax(Constants.IntakeConstants.INTAKEID2, MotorType.kBrushless); // 11
  SparkMaxConfig config = new SparkMaxConfig();

  public Intake() {
    config.smartCurrentLimit(50).idleMode(IdleMode.kBrake);

    intakeMotor1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotor2.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void intake() {
    //  SmartDashboard.putNumber(("intake"), 1);
    intakeMotor1.set(Constants.IntakeConstants.INTAKE_SPEED);
    intakeMotor2.set(-Constants.IntakeConstants.INTAKE_SPEED);
  }

  public void spitake() {
    intakeMotor1.set(-Constants.IntakeConstants.SPITAKE_SPEED);
    intakeMotor2.set(Constants.IntakeConstants.SPITAKE_SPEED);
  }

  public void stoptake() {
    intakeMotor1.set(0);
    intakeMotor2.set(0);
  }

  public Command intakeCMD() {
    return runOnce(
        () -> {
          intake();
        });
  }

  public Command spitakeCMD() {
    return runOnce(
        () -> {
          spitake();
        });
  }

  public Command stoptakeCMD() {
    return runOnce(
        () -> {
          stoptake();
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
