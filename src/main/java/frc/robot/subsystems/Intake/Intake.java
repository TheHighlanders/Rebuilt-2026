// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  TalonFX intakeMotor = new TalonFX(IntakeConstants.SPINTAKEID);
  TalonFXConfiguration config = new TalonFXConfiguration();
  CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();

  VelocityVoltage controller = new VelocityVoltage(0).withSlot(0);

  double inSpeed = IntakeConstants.INTAKE_SPEED;
  double outSpeed = IntakeConstants.SPITAKE_SPEED;

  public Intake() {
    config.Feedback.SensorToMechanismRatio = IntakeConstants.INTAKE_RATIO;

    config.Slot0.kP = 1;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;

    currentLimits.StatorCurrentLimit = 80;
    currentLimits.StatorCurrentLimitEnable = true;

    tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(config));
    tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(currentLimits));

    SmartDashboard.putNumber("Intake/in speed", inSpeed);
    SmartDashboard.putNumber("Intake/out speed", outSpeed);
    SmartDashboard.putString("Intake/State", "NONE");
  }
  // Intake commands to take in, spit out, and not move
  public Command intakeCMD() {
    // Takes in
    return Commands.runOnce(
        () -> {
          SmartDashboard.putString("Intake/State", "INTAKING");
          intakeMotor.setControl(controller.withVelocity(inSpeed));
        },
        this);
  }
  // Spits out
  public Command spitakeCMD() {
    return Commands.runOnce(
        () -> {
          SmartDashboard.putString("Intake/State", "SPITTING");
          intakeMotor.setControl(controller.withVelocity(outSpeed));
        },
        this);
  }

  public Command stoptakeCMD() {
    // Stops motor
    return Commands.runOnce(
        () -> {
          SmartDashboard.putString("Intake/State", "NONE");
          intakeMotor.setControl(controller.withVelocity(0));
        },
        this);
  }

  public Command killCMD() {
    return Commands.runOnce(
        () -> {
          SmartDashboard.putString("Intake/State", "BRAKED");
          intakeMotor.setControl(new StaticBrake());
        },
        this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    inSpeed = SmartDashboard.getNumber("Intake/in speed", IntakeConstants.INTAKE_SPEED);
    outSpeed = SmartDashboard.getNumber("Intake/out speed", IntakeConstants.SPITAKE_SPEED);
    SmartDashboard.putNumber("Intake/Velocity", intakeMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Current", intakeMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Voltage", intakeMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Temp", intakeMotor.getDeviceTemp().getValueAsDouble());
  }
}
