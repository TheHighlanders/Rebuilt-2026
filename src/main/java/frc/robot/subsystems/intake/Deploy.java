// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class Deploy extends SubsystemBase {
  /** Creates a new Deploy. */
  TalonFX deployMotor = new TalonFX(IntakeConstants.DEPLOYID);
  TalonFXConfiguration config = new TalonFXConfiguration();
  CurrentLimitsConfigs smartCurrentLimit = new CurrentLimitsConfigs();

  PositionVoltage controller2 = new PositionVoltage(0).withSlot(0);
  PIDController controller =
      new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);
      
  ArmFeedforward feedforward =
      new ArmFeedforward(IntakeConstants.kS, IntakeConstants.kG, IntakeConstants.kV);
  
  boolean raised = true;

  public Deploy() {
    smartCurrentLimit.StatorCurrentLimit = 50;
    deployMotor.setNeutralMode(NeutralModeValue.Brake);

    config.Slot0.kP = IntakeConstants.kP;
    config.Slot0.kI = IntakeConstants.kI;
    config.Slot0.kD = IntakeConstants.kD;
    config.Slot0.kS = IntakeConstants.kS;
    config.Slot0.kG = IntakeConstants.kG;
    config.Slot0.kV = IntakeConstants.kV;
    // SmartDashboard.putNumber("INTAKE/Deploy Encoder", deployMotor.getPosition());
    
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Feedback.SensorToMechanismRatio = IntakeConstants.DEPLOY_RATIO;

    deployMotor.setPosition(0);
  }

  public Command deployCMD() {
    // Deploys fuel
    return Commands.runOnce(
            () -> {
              controller.setSetpoint(
                  IntakeConstants.DEPLOY_POSITION.in(Radians));
            })
        .withName("Deployed");
  }

  public Command readyCMD() {
    return Commands.runOnce(
            () -> {
              controller.setSetpoint(
                  IntakeConstants.READY_POSITION.in(Radians));
            })
        .withName("Ready");
  }

  public Command undeployCMD() {
    return Commands.run(
            () -> {
              controller.setSetpoint(
                  IntakeConstants.UP_POSITION.in(Radians));
            })
        .withName("Retracted");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake/Deploy/Motor Current", deployMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Deploy/Motor Voltage", deployMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Deploy/Motor Temp", deployMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Deploy/Motor Position", deployMotor.getPosition().getValueAsDouble());
  }

  public Command swapCMD() {
    DriverStation.reportWarning("SwapCMD", false);
    return Commands.either(deployCMD(), undeployCMD(), () -> raised);
  }
}
