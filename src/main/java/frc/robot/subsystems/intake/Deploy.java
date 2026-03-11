// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;

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

  SparkMaxConfig config = new SparkMaxConfig();
  double p = IntakeConstants.kP;
  double i = IntakeConstants.kI;
  double d = IntakeConstants.kD;
  double s = IntakeConstants.kS;
  double g = IntakeConstants.kG;
  double v = IntakeConstants.kV;
  double rest = 0;

  public Deploy() {
    config.smartCurrentLimit(50).idleMode(IdleMode.kBrake);
    config.encoder.positionConversionFactor(IntakeConstants.DEPLOY_RATIO);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(p)
        .i(i)
        .d(d)
        .feedForward
        .kS(s)
        .kG(g)
        .kV(v);
    // .kCosRatio(IntakeConstants.DEPLOY_RATIO);
    deployMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    SmartDashboard.putNumber("INTAKE/Deploy Encoder", deployEncoder.getPosition());

    deployEncoder.setPosition(0);
    SmartDashboard.putNumber("INTAKE/kS", IntakeConstants.kS);
    SmartDashboard.putNumber("INTAKE/kG", IntakeConstants.kG);
    SmartDashboard.putNumber("INTAKE/kV", IntakeConstants.kV);
    SmartDashboard.putNumber("INTAKE/kP", IntakeConstants.kP);
    SmartDashboard.putNumber("INTAKE/kI", IntakeConstants.kI);
    SmartDashboard.putNumber("INTAKE/kD", IntakeConstants.kD);
    SmartDashboard.putNumber("INTAKE/rest speed", rest);
  }

  // private Command runCMD(double voltage) {
  //   return run(
  //       () -> {
  //         deployMotor.set(speed);
  //       });
  // }
  // Adds start and stop for deploying
  public Command deployCMD() {
    // Deploys fuel
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  closedLoopController.setSetpoint(
                      IntakeConstants.DEPLOY_POSITION.in(Radians), ControlType.kPosition);
                }),
            Commands.waitUntil(closedLoopController::isAtSetpoint),
            Commands.runOnce(() -> deployMotor.set(rest)))
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

  public Command mannualCMD(DoubleSupplier speed) {
    return run(() -> deployMotor.set(-speed.getAsDouble()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    s = SmartDashboard.getNumber("INTAKE/kS", IntakeConstants.kS);
    g = SmartDashboard.getNumber("INTAKE/kG", IntakeConstants.kG);
    v = SmartDashboard.getNumber("INTAKE/kV", IntakeConstants.kV);
    p = SmartDashboard.getNumber("INTAKE/kP", IntakeConstants.kP);
    i = SmartDashboard.getNumber("INTAKE/kI", IntakeConstants.kI);
    d = SmartDashboard.getNumber("INTAKE/kD", IntakeConstants.kD);
    rest = SmartDashboard.getNumber("INTAKE/rest speed", rest);
    SmartDashboard.putNumber("INTAKE/Deploy Encoder", deployEncoder.getPosition());

    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(p)
        .i(i)
        .d(d)
        .feedForward
        .kS(s)
        .kG(g)
        .kV(v);
    // .kCosRatio(IntakeConstants.DEPLOY_RATIO);
    deployMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
