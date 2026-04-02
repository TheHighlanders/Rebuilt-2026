// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
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
import frc.robot.Constants.HopperConstants;
import java.util.function.DoubleSupplier;

public class Hopper extends SubsystemBase {
  /** Creates a new Hopper. */
  SparkMax hopper = new SparkMax(HopperConstants.HOPPERID, MotorType.kBrushless);

  SparkMax kicker = new SparkMax(HopperConstants.KICKERID, MotorType.kBrushless);

  SparkClosedLoopController kickLoopController = kicker.getClosedLoopController();

  double kP = 6 * 0.0001;
  double kI = 0.0;
  double kD = 2;

  SparkMaxConfig kickConfig = new SparkMaxConfig();

  DoubleSupplier kickerSpeed;

  public Hopper(DoubleSupplier shooterSpeed) {
    SparkMaxConfig hopperConfig = new SparkMaxConfig();
    hopperConfig.smartCurrentLimit(HopperConstants.HOPPER_CURRENT_LIMIT).idleMode(IdleMode.kCoast);
    hopperConfig.inverted(HopperConstants.INVERT_HOPPER);

    kickConfig.closedLoop.p(kP).i(kI).d(kD);
    kickConfig.smartCurrentLimit(HopperConstants.KICKER_CURRENT_LIMIT).idleMode(IdleMode.kCoast);
    kickConfig.inverted(HopperConstants.INVERT_KICKER);

    // Persist parameters to retain configuration in the event of a power cycle
    hopper.configure(hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    kicker.configure(kickConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putNumber("Shooter/Hopper Current", hopper.getOutputCurrent());
    SmartDashboard.putNumber("Shooter/Hopper Current", kicker.getOutputCurrent());

    SmartDashboard.putNumber("Shooter/Kicker/P", kP);
    SmartDashboard.putNumber("Shooter/Kicker/I", kI);
    SmartDashboard.putNumber("Shooter/Kicker/D", kD);
    SmartDashboard.putNumber("Shooter/Kicker/Reset?", 0);

    kickerSpeed =
        () -> {
          return shooterSpeed.getAsDouble() == 0 ? 600 : shooterSpeed.getAsDouble() * 60;
        };
  }
  // spins the motor inside the hopper
  public Command shootCMD() {
    return Commands.runOnce(
        () -> {
          kickLoopController.setSetpoint(kickerSpeed.getAsDouble(), ControlType.kVelocity);
          // hopper.set(0.4);
          // speed can be changed
          SmartDashboard.putString("Shooter/Hopper State", "Shooting");
        },
        this);
  }

  public Command doubleCMD() {
    return Commands.runOnce(
        () -> {
          kickLoopController.setSetpoint(kickerSpeed.getAsDouble(), ControlType.kVelocity);
          hopper.set(0.8);
          // speed can be changed
          SmartDashboard.putString("Shooter/Hopper State", "Shooting");
        },
        this);
  }

  // stops the hopper
  public Command stopCMD() {
    return Commands.runOnce(
        () -> {
          kickLoopController.setSetpoint(0, ControlType.kVelocity);
          hopper.set(0);
          SmartDashboard.putString("Shooter/Hopper State", "Stopped");
        },
        this);
  }

  // backdrives the hopper
  public Command backdriveCMD() {
    return Commands.runOnce(
        () -> {
          kickLoopController.setSetpoint(-7, ControlType.kVelocity);
          hopper.set(-0.5);
          SmartDashboard.putString("Shooter/Hopper State", "Backdriving");
        },
        this);
  }

  // clears the hopper
  public Command clearCMD() {
    return Commands.runOnce(
        () -> {
          kickLoopController.setSetpoint(kickerSpeed.getAsDouble(), ControlType.kVelocity);
          hopper.set(-1);
          SmartDashboard.putString("Shooter/Hopper State", "Clearing");
        },
        this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter/Hopper Current", hopper.getOutputCurrent());
    SmartDashboard.putNumber("Shooter/Hopper Current", kicker.getOutputCurrent());
    SmartDashboard.putNumber("Shooter/Kicker RPS", kicker.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter/Kicker Target RPS", kickerSpeed.getAsDouble());

    kP = SmartDashboard.getNumber("Shooter/Kicker/P", kP);
    kI = SmartDashboard.getNumber("Shooter/Kicker/I", kI);
    kD = SmartDashboard.getNumber("Shooter/Kicker/D", kD);
    kickConfig.closedLoop.p(kP).i(kI).d(kD);
    if (SmartDashboard.getNumber("Shooter/Kicker/Reset?", 0) != 0)
      kicker.configure(
          kickConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
