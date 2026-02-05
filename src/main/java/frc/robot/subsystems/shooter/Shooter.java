// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  SparkMax flywheel = new SparkMax(Constants.ShooterConstants.SHOOTERID, MotorType.kBrushless);
  SparkMax kicker = new SparkMax(Constants.ShooterConstants.SHOOTERID, MotorType.kBrushless);

  SparkMaxSim flywheelSim = new SparkMaxSim(flywheel, DCMotor.getNEO(1));
  SparkMaxSim kickerSim = new SparkMaxSim(kicker, DCMotor.getNEO(1));

  // pid
  SparkClosedLoopController shootController = flywheel.getClosedLoopController();

  // Encoder: A sensor that measures the amount of rotations
  RelativeEncoder flywheelEncoder;

  // PID
  double kP = 0.1;
  double kI = 0.0;
  double kD = 3.0;
  double targetRPM = 0.0;

  // pid config
  SparkMaxConfig ShooterConfig = new SparkMaxConfig();

  public Shooter() {
    // initialize encoder
    flywheelEncoder = flywheel.getEncoder();

    // Set PID gains
    ShooterConfig.closedLoop.p(kP).i(kI).d(kD);

    // dropper config
    flywheel.configure(
        ShooterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    // For Elastic and Advantage
    SmartDashboard.putNumber("PID/Shooter/kP", kP);
    SmartDashboard.putNumber("PID/Shooter/kI", kI);
    SmartDashboard.putNumber("PID/Shooter/kD", kD);
    SmartDashboard.putNumber("PID/Shooter/Target RPM", 0.0);
  }

  private double calculate(double asDouble) {
    // TODO Auto-generated method stub

    return asDouble;
  }

  public boolean atSpeed() {
    return shootController.isAtSetpoint();
  }

  public Command flywheelCMD(DoubleSupplier distance) {

    return Commands.run(
        () -> {

          targetRPM = calculate(distance.getAsDouble());
          SmartDashboard.putNumber("PID/Shooter/Target RPM", targetRPM);

          shootController.setSetpoint(targetRPM, ControlType.kVelocity);

        }, this);
  }

  public Command kickerCMD() {

    return Commands.runOnce(
      () -> kicker.set(1)
      , this);
    
  }
          
          
  public Command stopCMD() {

    targetRPM = 0;
    //SmartDashboard.putNumber("PID/Shooter/Target RPM", newTargetRPM);

    return Commands.runOnce(
        () -> {
          shootController.setSetpoint(targetRPM, ControlType.kVelocity);
          kicker.set(0);
        }, this);
  }

  @SuppressWarnings("unused")
  @Override
  public void periodic() {
    // For Elastic and Advtange Scope
    double newP = SmartDashboard.getNumber("PID/Shooter/kP", kP);
    double newI = SmartDashboard.getNumber("PID/Shooter/kI", kI);
    double newD = SmartDashboard.getNumber("PID/Shooter/kD", kD);
    // double newTTargetRPM = SmartDashboard.getNumber("PID/Shooter/Target RPM", targetRPM);

    SmartDashboard.putNumber(
        "PID/Shooter/Dropper Output", flywheelEncoder.getPosition()); // Setpoint

    SmartDashboard.putNumber("PID/Shooter/Target RPM", shootController.getSetpoint()); //
    // Setpoint
    SmartDashboard.putNumber(
        "PID/Shooter/Dropper Velocity", flywheelEncoder.getVelocity()); // Actual velocity

    // SmartDashboard.updateValues();
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)

    // Next, we update it. The standard loop time is 20ms.

    // Now, we update the Spark MAX
    flywheelSim.iterate(
        flywheelSim.getSetpoint(),
        12, // Simulated battery voltage, in Volts
        0.02); // Time interval, in Seconds

    // SimBattery estimates loaded battery voltages
    // This should include all motors being simulated

  }
}
