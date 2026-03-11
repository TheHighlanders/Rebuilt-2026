// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  SparkMax climbMotor = new SparkMax(ClimberConstants.CLIMBERID, MotorType.kBrushless);
  RelativeEncoder climbEncoder = climbMotor.getEncoder();
  SparkMaxConfig config = new SparkMaxConfig();
  boolean initialized = false;

  /** Creates a new Climber. */
  public Climber() {
    config.idleMode(IdleMode.kBrake);
    climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    climbEncoder.setPosition(0);
  }

  private Command runCMD(double speed) {
    return run(
        () -> {
          climbMotor.set(speed);
        });
  }

  public Command initializeCMD() {
    // runs the climber down until it jumps above a certain current
    // to find bottom of climber
    // capped at 2ish seconds for safety
    return Commands.deadline(
              Commands.deadline(
                Commands.waitSeconds(2/ClimberConstants.CURRENT_DETECT_SPEED),
                Commands.waitUntil(() -> 
                  climbMotor.getOutputCurrent() > ClimberConstants.CURRENT_DETECT_LEVEL)), 
              runCMD(ClimberConstants.CURRENT_DETECT_SPEED))
          //then, resets climb encoder.
              .andThen(Commands.runOnce(() -> {
                initialized = true;
                climbEncoder.setPosition(0);
              }));
  }

  public Command raiseCMD() {
    return Commands.deadline(
            Commands.waitUntil(
                () ->
                    climbEncoder.getPosition()
                        <=  ClimberConstants.UP_POS_OFFSET),
            runCMD(ClimberConstants.RAISE_SPEED))
        .andThen(runCMD(0));
  }

  public Command pullCMD() {
    return Commands.either(
          //normal raise command  
            Commands.deadline(
              Commands.waitUntil(
                      () ->
                          climbEncoder.getPosition()
                              >= 0),
                  runCMD(ClimberConstants.PULL_SPEED))
              .andThen(runCMD(ClimberConstants.HOLD_SPEED)),
            this.initializeCMD(),
            () -> initialized);
  }

  public DoubleSupplier currentDraw() {//use this for better autoClimb?
    return climbMotor::getOutputCurrent;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
