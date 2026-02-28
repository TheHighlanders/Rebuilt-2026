// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  TalonFX flywheel = new TalonFX(ShooterConstants.SHOOTERID);

  // Config for PID
  TalonFXConfiguration config = new TalonFXConfiguration();

  VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
  NeutralOut brake = new NeutralOut();

  CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();

  double targetRPS;

  public Shooter() {

    config.Slot0.kS = ShooterConstants.kS;
    config.Slot0.kV = ShooterConstants.kV;
    config.Slot0.kP = ShooterConstants.kP;
    config.Slot0.kI = ShooterConstants.kI;
    config.Slot0.kD = ShooterConstants.kD;

    config.Voltage.withPeakForwardVoltage(8).withPeakReverseVoltage(8);

    currentLimits.StatorCurrentLimit = 80;
    currentLimits.SupplyCurrentLowerLimit = -80;
    currentLimits.StatorCurrentLimitEnable = true;

    tryUntilOk(5, () -> flywheel.getConfigurator().apply(config));
    tryUntilOk(5, () -> flywheel.getConfigurator().apply(currentLimits));

    SmartDashboard.putNumber("Shooter/Target RPS", 0.0);
  }

  public void intake() {}

  protected double calculate(double distanceMeters) {
    // TODO: wait until shooter is finalized
    double linearVelocity = 1.4 * distanceMeters + 6.1;
    return linearVelocity * 193 / 60; // trust that makes it angular i did the (?) math:
    // https://www.desmos.com/calculator/zroouacb64
  }

  public boolean atSpeed() {
    return Math.abs(flywheel.getVelocity().getValueAsDouble() - targetRPS) < 1;
  }

  public Command flywheelCMD(DoubleSupplier distance) {
    return Commands.run(
        () -> {
          targetRPS = calculate(distance.getAsDouble());
          flywheel.setControl(velocityVoltage.withVelocity(targetRPS));
          SmartDashboard.putNumber(
              "Shooter/Flywheel/Voltage", flywheel.getMotorVoltage().getValueAsDouble());
          SmartDashboard.putNumber(
              "Shooter/Flywheel/Current", flywheel.getStatorCurrent().getValueAsDouble());
          SmartDashboard.putNumber("Shooter/Target RPS", targetRPS);
          SmartDashboard.putNumber(
              "Shooter/Flywheel RPS", flywheel.getVelocity().getValueAsDouble());
        },
        this);
  }

  public Command stopCMD() {
    return Commands.runOnce(
        () -> {
          flywheel.setControl(brake);
        },
        this);
  }

  @Override
  public void simulationPeriodic() {
    SmartDashboard.putNumber(
        "Shooter/Flywheel/Voltage", flywheel.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber(
        "Shooter/Flywheel/Current", flywheel.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Target RPS", targetRPS);
    SmartDashboard.putNumber("Shooter/Flywheel RPS", flywheel.getVelocity().getValueAsDouble());
  }
}
