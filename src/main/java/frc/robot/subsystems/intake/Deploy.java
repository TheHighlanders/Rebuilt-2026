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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import java.util.function.DoubleSupplier;

public class Deploy extends SubsystemBase {
  /** Creates a new Deploy. */
  SparkMax deployMotor = new SparkMax(IntakeConstants.DEPLOYID, MotorType.kBrushless);

  SparkClosedLoopController closedLoopController = deployMotor.getClosedLoopController();
  RelativeEncoder deployEncoder = deployMotor.getEncoder();
  boolean raised = true;

  double inPos = IntakeConstants.UP_POSITION;
  double readyPos = IntakeConstants.READY_POSITION;
  double downPos = IntakeConstants.DEPLOY_POSITION;

  SparkMaxConfig config = new SparkMaxConfig();
  ArmFeedforward feedforward =
      new ArmFeedforward(IntakeConstants.kS, IntakeConstants.kG, IntakeConstants.kV);

  public Deploy() {
    config.smartCurrentLimit(40).idleMode(IdleMode.kBrake);
    config.encoder.positionConversionFactor(IntakeConstants.DEPLOY_RATIO);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(IntakeConstants.kP)
        .i(IntakeConstants.kI)
        .d(IntakeConstants.kD)
        .feedForward
        .kS(IntakeConstants.kS)
        .kCos(IntakeConstants.kG)
        .kV(IntakeConstants.kV);
    // .kCosRatio(IntakeConstants.DEPLOY_RATIO);
    deployMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    SmartDashboard.putNumber("Intake/Deploy/Deploy Encoder", deployEncoder.getPosition());

    SmartDashboard.putNumber("Intake/Intake Up Setpoint", inPos);
    SmartDashboard.putNumber("Intake/Intake Ready Setpoint", readyPos);
    SmartDashboard.putNumber("Intake/Intake Down Setpoint", downPos);

    deployEncoder.setPosition(0);
  }

  public Command deployCMD() {
    // Deploys fuel
    return Commands.runOnce(
            () -> {
              closedLoopController.setSetpoint(downPos, ControlType.kPosition);
              raised = false;
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
              closedLoopController.setSetpoint(readyPos, ControlType.kPosition);
              raised = false;
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
              closedLoopController.setSetpoint(inPos, ControlType.kPosition);
              raised = true;
            })
        .withName("Retracted");
    // return Commands.deadline(
    //         Commands.waitUntil(
    //             () -> deployEncoder.getPosition() <= IntakeConstants.DEPLOY_TOLERANCE),
    //         runCMD(0 - IntakeConstants.DEPLOY_SPEED))
    //     .andThen(runCMD(0));
  }

  public Command manualCMD(DoubleSupplier speed) {
    return Commands.run(() -> deployMotor.set(speed.getAsDouble()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    inPos = SmartDashboard.getNumber("Intake/Intake Up Setpoint", inPos);
    readyPos = SmartDashboard.getNumber("Intake/Intake Ready Setpoint", readyPos);
    downPos = SmartDashboard.getNumber("Intake/Intake Down Setpoint", downPos);

    SmartDashboard.putNumber("Intake/Deploy/Deploy Encoder", deployEncoder.getPosition());
    SmartDashboard.putNumber("Intake/Deploy/Deploy Encoder Velocity", deployEncoder.getVelocity());
    SmartDashboard.putNumber("Intake/Deploy/Deploy Current", deployMotor.getOutputCurrent());

    SmartDashboard.putString(
        "Intake/Deploy State",
        getCurrentCommand() == null ? "NONE" : getCurrentCommand().getName());

    if (Math.abs(closedLoopController.getSetpoint() - deployEncoder.getPosition()) < 10) {
      deployMotor.stopMotor();
    }
  }

  public Command swapCMD() {
    DriverStation.reportWarning("SwapCMD", false);
    return Commands.either(deployCMD(), undeployCMD(), () -> raised);
  }
}
