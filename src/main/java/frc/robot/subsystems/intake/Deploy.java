// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import static edu.wpi.first.units.Units.Radians;

import java.util.function.DoubleSupplier;

public class Deploy extends SubsystemBase {
  /** Creates a new Deploy. */
  SparkMax deployMotor = new SparkMax(IntakeConstants.DEPLOYID, MotorType.kBrushless);

  SparkClosedLoopController closedLoopController = deployMotor.getClosedLoopController();
  RelativeEncoder deployEncoder = deployMotor.getEncoder();
  ProfiledPIDController controller;
  boolean raised = true;

  SparkMaxConfig config = new SparkMaxConfig();
  ArmFeedforward feedforward =
      new ArmFeedforward(IntakeConstants.kS, IntakeConstants.kG, IntakeConstants.kV);

  public Deploy() {
    config.smartCurrentLimit(50).idleMode(IdleMode.kBrake);
    config.encoder.positionConversionFactor(IntakeConstants.DEPLOY_RATIO);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(IntakeConstants.kP)
        .i(IntakeConstants.kI)
        .d(IntakeConstants.kD)
        .feedForward
        .kS(IntakeConstants.kS)
        .kG(IntakeConstants.kG)
        .kV(IntakeConstants.kV);
    // .kCosRatio(IntakeConstants.DEPLOY_RATIO);
    deployMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    SmartDashboard.putNumber("INTAKE/Deploy Encoder", deployEncoder.getPosition());

    deployEncoder.setPosition(0);
  }

  public Command deployCMD() {
    // Deploys fuel
    return Commands.runOnce(
            () -> {
              closedLoopController.setSetpoint(
                  IntakeConstants.DEPLOY_POSITION.in(Radians), ControlType.kPosition);
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
              closedLoopController.setSetpoint(
                  IntakeConstants.READY_POSITION.in(Radians), ControlType.kPosition);
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
              closedLoopController.setSetpoint(
                  IntakeConstants.UP_POSITION.in(Radians), ControlType.kPosition);
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
    SmartDashboard.putNumber("INTAKE/Deploy/Deploy Encoder", deployEncoder.getPosition());
    SmartDashboard.putNumber("INTAKE/Deploy/Deploy Encoder Velocity", deployEncoder.getVelocity());
    SmartDashboard.putNumber("INTAKE/Deploy/Deploy Current", deployMotor.getOutputCurrent());

    SmartDashboard.putString(
        "INTAKE/Deploy State",
        getCurrentCommand() == null ? "NONE" : getCurrentCommand().getName());
  }

  public Command swapCMD() {
    DriverStation.reportWarning("SwapCMD", false);
    return Commands.either(deployCMD(), undeployCMD(), () -> raised);
  }
}
