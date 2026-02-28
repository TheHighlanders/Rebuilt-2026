// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Deploy extends SubsystemBase {
  /** Creates a new Deploy. */
  SparkMax deployMotor = new SparkMax(IntakeConstants.DEPLOYID, MotorType.kBrushless);

  SparkClosedLoopController closedLoopController = deployMotor.getClosedLoopController();
  RelativeEncoder deployEncoder = deployMotor.getEncoder();

  SparkMaxConfig config = new SparkMaxConfig();

  public Deploy() {
    config.smartCurrentLimit(50).idleMode(IdleMode.kBrake);
    config.encoder.positionConversionFactor(1).velocityConversionFactor(1);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(IntakeConstants.kP1)
        .i(IntakeConstants.kI1)
        .d(IntakeConstants.kD1)
        .outputRange(-1, 1)
        .p(IntakeConstants.kP2, ClosedLoopSlot.kSlot1)
        .i(IntakeConstants.kI2, ClosedLoopSlot.kSlot1)
        .d(IntakeConstants.kD2, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);
    deployMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    deployEncoder.setPosition(0);
  }

  private Command runCMD(double speed) {
    return run(
        () -> {
          deployMotor.set(speed);
        });
  }
  // Adds start and stop for deploying
  public Command deployCMD() {
    // Deploys fuel
    return Commands.deadline(
            Commands.waitUntil(
                () ->
                    deployEncoder.getPosition()
                        >= IntakeConstants.DEPLOY_POSITION - IntakeConstants.DEPLOY_TOLERANCE),
            runCMD(IntakeConstants.DEPLOY_SPEED))
        .andThen(runCMD(0));
  }
  // Stops motor
  public Command undeployCMD() {
    return Commands.deadline(
            Commands.waitUntil(
                () -> deployEncoder.getPosition() <= IntakeConstants.DEPLOY_TOLERANCE),
            runCMD(0 - IntakeConstants.DEPLOY_SPEED))
        .andThen(runCMD(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
