// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
  /** Creates a new Hopper. */
  SparkMax Hopper = new SparkMax(Constants.HopperConstants.HOPPERID, MotorType.kBrushless);

  public Hopper() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(50).idleMode(IdleMode.kCoast);

    // Persist parameters to retain configuration in the event of a power cycle
    Hopper.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  // spins the motor inside the hopper
  public Command SpinCMD() {

    return runOnce(
        () -> {
          Hopper.set(1);
          // speed can be changed
          DriverStation.reportWarning("StartHopper", false);
        });
  }
  // stops the hopper
  public Command StopCMD() {

    return runOnce(
        () -> {
          Hopper.set(0);
          DriverStation.reportWarning("StopHopper", false);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
