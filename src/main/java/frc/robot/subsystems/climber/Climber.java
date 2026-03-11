// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  SparkMax climbMotor = new SparkMax(ClimberConstants.CLIMBERID, MotorType.kBrushless);
  RelativeEncoder climbEncoder = climbMotor.getEncoder();
  SparkMaxConfig config = new SparkMaxConfig();
  double testSpeed = 1;
  double downPos = ClimberConstants.DOWN_POSITION;
  double downSpd = ClimberConstants.PULL_SPEED;
  double tuckSpd = ClimberConstants.TUCK_SPEED;
  double upPos = 0;
  double upSpd = ClimberConstants.RAISE_SPEED;
  double tolerance = ClimberConstants.POS_TOLERANCE;

  /** Creates a new Climber. */
  public Climber() {
    config.idleMode(IdleMode.kBrake);
    climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    climbEncoder.setPosition(0);

    SmartDashboard.putNumber("CLIMBER/down position", downPos);
    SmartDashboard.putNumber("CLIMBER/pull speed", downSpd);
    SmartDashboard.putNumber("CLIMBER/tuck speed", tuckSpd);
    SmartDashboard.putNumber("CLIMBER/up position", upPos);
    SmartDashboard.putNumber("CLIMBER/up speed", upSpd);
    SmartDashboard.putNumber("CLIMBER/tolerance", tolerance);
  }

  private Command runCMD(double speed) {
    return run(
        () -> {
          climbMotor.set(speed);
        });
  }

  public Command runUpCMD() {
    return run(
        () -> {
          climbMotor.set(upSpd);
        });
  }

  public Command runDownCMD() {
    return run(
        () -> {
          climbMotor.set(downSpd);
        });
  }

  public Command stopCMD() {
    return run(
        () -> {
          climbMotor.set(0);
        });
  }

  public Command raiseCMD() {
    return Commands.deadline(
            Commands.waitUntil(() -> climbEncoder.getPosition() <= upPos + tolerance),
            runCMD(upSpd))
        .andThen(runCMD(0));
  }

  public Command pullCMD() {
    return Commands.deadline(
            Commands.waitUntil(() -> climbEncoder.getPosition() >= downPos - tolerance),
            runCMD(downSpd))
        .andThen(runCMD(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("CLIMBER/Position", climbEncoder.getPosition());
    SmartDashboard.putNumber("CLIMBER/Motor/Voltage", climbMotor.getBusVoltage());
    SmartDashboard.putNumber("CLIMBER/Motor/Current", climbMotor.getOutputCurrent());
    SmartDashboard.putNumber("CLIMBER/Motor/Temp", climbMotor.getMotorTemperature());

    // SmartDashboard.putNumber("CLIMBER/down position", downPos);
    // SmartDashboard.putNumber("CLIMBER/pull speed", downSpd);
    // SmartDashboard.putNumber("CLIMBER/tuck speed", tuckSpd);
    // SmartDashboard.putNumber("CLIMBER/up position", upPos);
    // SmartDashboard.putNumber("CLIMBER/up speed", upSpd);
    // SmartDashboard.putNumber("CLIMBER/tolerance", tolerance);

    double newDownPos = SmartDashboard.getNumber("CLIMBER/down position", downPos);
    double newdownSpd = SmartDashboard.getNumber("CLIMBER/pull speed", downSpd);
    double newtuckSpd = SmartDashboard.getNumber("CLIMBER/tuck speed", tuckSpd);
    double newupPos = SmartDashboard.getNumber("CLIMBER/up position", upPos);
    double newupSpd = SmartDashboard.getNumber("CLIMBER/up speed", upSpd);
    double newtolerance = SmartDashboard.getNumber("CLIMBER/tolerance", tolerance);

    downPos = newDownPos;
    downSpd = newdownSpd;
    tuckSpd = newtuckSpd;
    upPos = newupPos;
    upSpd = newupSpd;
    tolerance = newtolerance;
  }
}
