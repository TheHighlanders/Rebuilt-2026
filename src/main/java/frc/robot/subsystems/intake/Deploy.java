// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Deploy extends SubsystemBase {
  /** Creates a new Deploy. */
  TalonFX deployMotor = new TalonFX(IntakeConstants.DEPLOYID);
  TalonFXConfiguration config = new TalonFXConfiguration();
  CurrentLimitsConfigs smartCurrentLimit = new CurrentLimitsConfigs();

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
    // .kCosRatio(IntakeConstants.DEPLOY_RATIO);
    // SmartDashboard.putNumber("INTAKE/Deploy Encoder", deployMotor.getPosition());

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
    // return Commands.deadline(
    //         Commands.waitUntil(
    //             () ->
    //                 deployEncoder.getPosition()
    //                     >= IntakeConstants.DEPLOY_POSITION - IntakeConstants.DEPLOY_TOLERANCE),
    //         runCMD(IntakeConstants.DEPLOY_SPEED))
    //     .andThen(runCMD(0));
  }

  public Command readyCMD() {
    return Commands.runOnce(
            () -> {
              controller.setSetpoint(
                  IntakeConstants.READY_POSITION.in(Radians));
            })
        .withName("Ready");
    // return Commands.deadline(
    //         Commands.waitUntil(
    //             () ->
    //                 deployEncoder.getPosition()
    //                     >= IntakeConstants.READY_POSITION - IntakeConstants.DEPLOY_TOLERANCE),
    //         runCMD(IntakeConstants.DEPLOY_SPEED))
    //     .andThen(runCMD(0));
  }

  public Command undeployCMD() {
    return Commands.run(
            () -> {
              controller.setSetpoint(
                  IntakeConstants.UP_POSITION.in(Radians));
            })
        .withName("Retracted");
    // return Commands.deadline(
    //         Commands.waitUntil(
    //             () -> deployEncoder.getPosition() <= IntakeConstants.DEPLOY_TOLERANCE),
    //         runCMD(0 - IntakeConstants.DEPLOY_SPEED))
    //     .andThen(runCMD(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("INTAKE/Deploy/Deploy Encoder", deployMotor.getPosition());
    // SmartDashboard.putNumber("INTAKE/Deploy/Deploy Encoder Velocity", deployMotor.getVelocity());
    // SmartDashboard.putNumber("INTAKE/Deploy/Deploy Current", deployMotor.getStatorCurrent());

    SmartDashboard.putString(
        "INTAKE/Deploy State",
        getCurrentCommand() == null ? "NONE" : getCurrentCommand().getName());
  }

  public Command swapCMD() {
    DriverStation.reportWarning("SwapCMD", false);
    return Commands.either(deployCMD(), undeployCMD(), () -> raised);
  }
}
