// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
  ProfiledPIDController controller;
  boolean raised = true;

  SparkMaxConfig config = new SparkMaxConfig();
  double rest = 0;

  public Deploy() {
    config.smartCurrentLimit(60).idleMode(IdleMode.kBrake);
    config.encoder.positionConversionFactor(IntakeConstants.DEPLOY_RATIO);

    deployMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    SmartDashboard.putNumber("INTAKE/Deploy Encoder", deployEncoder.getPosition());

    deployEncoder.setPosition(0);
  }

  public Command deployCMD() {
    return Commands.deadline(
            Commands.waitSeconds(0.5),
            Commands.runOnce(
                () -> {
                  deployMotor.set(-0.2);
                }))
        .andThen(
            Commands.runOnce(
                () -> {
                  deployMotor.set(0);
                  raised = false;
                }));
  }

  public Command undeployCMD() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              deployMotor.set(0.3);
            }),
        Commands.waitSeconds(2),
        Commands.runOnce(
            () -> {
              deployMotor.set(0.02);
              raised = true;
            }));
  }

  public Command mannualCMD(DoubleSupplier speed) {
    return run(() -> deployMotor.set(-0.3 * speed.getAsDouble()));
  }

  public boolean isRaised() {
    return raised;
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
