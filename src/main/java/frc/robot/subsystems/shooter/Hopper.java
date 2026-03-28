// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;

public class Hopper extends SubsystemBase {
  /** Creates a new Hopper. */
  SparkMax hopper = new SparkMax(HopperConstants.HOPPERID, MotorType.kBrushless);

  SparkMax kicker = new SparkMax(HopperConstants.KICKERID, MotorType.kBrushless);

  double kP = 0.1;
  double kI = 0.0;
  double kD = 3.0;
  double kickTargetRPM = 2000.0;

  SparkMaxConfig kickConfig = new SparkMaxConfig();

  public Hopper() {
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
  }
  // spins the motor inside the hopper
  public Command shootCMD() {
    return Commands.runOnce(
        () -> {
          kicker.set(0.8);
          hopper.set(0.4);
          // speed can be changed
          SmartDashboard.putString("Shooter/Hopper State", "Shooting");
        },
        this);
  }
  // stops the hopper
  public Command stopCMD() {
    return Commands.runOnce(
        () -> {
          kicker.set(0);
          hopper.set(0);
          SmartDashboard.putString("Shooter/Hopper State", "Stopped");
        },
        this);
  }

  // backdrives the hopper
  public Command backdriveCMD() {
    return Commands.runOnce(
        () -> {
          kicker.set(-0.5);
          hopper.set(-0.5);
          SmartDashboard.putString("Shooter/Hopper State", "Backdriving");
        },
        this);
  }

  // clears the hopper
  public Command clearCMD() {
    return Commands.runOnce(
        () -> {
          kicker.set(1);
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

  }
}
