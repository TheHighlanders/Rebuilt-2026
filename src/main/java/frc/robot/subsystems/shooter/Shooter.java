// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  TalonFX flywheel = new TalonFX(ShooterConstants.SHOOTERID);

  // Config for PID
  TalonFXConfiguration config = new TalonFXConfiguration();

  VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
  // VelocityTorqueCurrentFOC velocityTorque = new VelocityTorqueCurrentFOC(0).withSlot(1);
  DutyCycleOut dutyCycle = new DutyCycleOut(0);
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

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Feedback.SensorToMechanismRatio = ShooterConstants.GEAR_RATIO;

    currentLimits.StatorCurrentLimit = 80;
    currentLimits.SupplyCurrentLowerLimit = -80;
    currentLimits.StatorCurrentLimitEnable = true;

    tryUntilOk(5, () -> flywheel.getConfigurator().apply(config));
    tryUntilOk(5, () -> flywheel.getConfigurator().apply(currentLimits));

    SmartDashboard.putNumber("Shooter/Target RPS", 0.0);
  }

  public void intake() {} // for sim

  protected double calculate(Translation2d trajectory) {
    // TODO: wait until shooter is finalized, and tune.
    double linearVelocity =
        Math.sqrt(
            ShooterConstants.GRAVITY
                * Math.pow(trajectory.getX(), 2)
                / (2
                    * Math.pow(Math.cos(ShooterConstants.SHOOTER_HOOD.in(Radians)), 2)
                    * (trajectory.getX() * Math.tan(ShooterConstants.SHOOTER_HOOD.in(Radians))
                        - trajectory.getY())));

    return linearVelocity / (ShooterConstants.FLYWHEEL_RADIUS.in(Meters) * 2 * Math.PI);
  }

  protected double calculateRR(Translation2d trajectory) {
    Translation3d robotToHub = new Translation3d(trajectory.getX(), 0, trajectory.getY());
    Translation3d shooterToHub =
        robotToHub.minus(
            ShooterConstants.SHOOTER_RR_POS.rotateBy(new Rotation3d(0, 0, Math.PI / 2)));
    Translation2d newTrajectory =
        new Translation2d(
            Math.hypot(shooterToHub.getX(), shooterToHub.getY()), shooterToHub.getZ());
    return calculate(newTrajectory);
  }

  protected double calculateHub(double distance) {
    return calculateRR(new Translation2d(distance, FieldConstants.HUB_HEIGHT));
  }

  protected double calculateGround(double distance) {
    return calculateRR(new Translation2d(distance, 0));
  }

  public boolean atSpeed() {
    return Math.abs(flywheel.getVelocity().getValueAsDouble() - targetRPS) < 0.4;
  }

  public Command flywheelCMD(DoubleSupplier distance) {
    return Commands.run(
        () -> {
          targetRPS = calculateHub(distance.getAsDouble());
          flywheel.setControl(velocityVoltage.withVelocity(targetRPS));
          SmartDashboard.putNumber(
              "Shooter/Flywheel/Voltage", flywheel.getMotorVoltage().getValueAsDouble());
          SmartDashboard.putNumber(
              "Shooter/Flywheel/Current", flywheel.getStatorCurrent().getValueAsDouble());
          SmartDashboard.putNumber("Shooter/Target RPS", targetRPS);
          SmartDashboard.putNumber(
              "Shooter/Flywheel RPS", flywheel.getVelocity().getValueAsDouble());
          SmartDashboard.putNumber("Shooter/Distance", distance.getAsDouble());
        },
        this);
  }
  
  public Command flywheelGndCMD(DoubleSupplier distance) {
    return Commands.run(
        () -> {
          targetRPS = calculateGround(distance.getAsDouble());
          flywheel.setControl(velocityVoltage.withVelocity(targetRPS));
          SmartDashboard.putNumber(
              "Shooter/Flywheel/Voltage", flywheel.getMotorVoltage().getValueAsDouble());
          SmartDashboard.putNumber(
              "Shooter/Flywheel/Current", flywheel.getStatorCurrent().getValueAsDouble());
          SmartDashboard.putNumber("Shooter/Target RPS", targetRPS);
          SmartDashboard.putNumber(
              "Shooter/Flywheel RPS", flywheel.getVelocity().getValueAsDouble());
          SmartDashboard.putNumber("Shooter/Distance", distance.getAsDouble());
        },
        this);
  }

  public Command rawFlywheelCMD(DoubleSupplier drive) {
    return Commands.run(
        () -> {
          flywheel.setControl(dutyCycle.withOutput(drive.getAsDouble()));
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
