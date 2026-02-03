// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  SparkMax Flywheel = new SparkMax(Constants.ShooterConstants.SHOOTERID, MotorType.kBrushless);

  SparkMax Kickerwheel = new SparkMax(Constants.ShooterConstants.KICKERID, MotorType.kBrushless);
  SparkMaxSim FlywheelSim = new SparkMaxSim(Flywheel, DCMotor.getNEO(1));

  // pid
  SparkClosedLoopController shootController = Flywheel.getClosedLoopController();
  SparkClosedLoopController kickController = Kickerwheel.getClosedLoopController();

  // Encoder: A sensor that measures the amount of rotations
  RelativeEncoder flywheelEncoder;
  RelativeEncoder kickEncoder;

  // PID
  double kP = 0.1;
  double kI = 0.0;
  double kD = 3.0;
  double targetRPM = 0.0;

  double kickTargetRPM = 0.0;

  // pid config
  SparkMaxConfig ShooterConfig = new SparkMaxConfig();
  SparkMaxConfig KickConfig = new SparkMaxConfig();

  public Shooter() {
    // initialize encoder
    flywheelEncoder = Flywheel.getEncoder();
    kickEncoder = Kickerwheel.getEncoder();

    // Set PID gains
    ShooterConfig.closedLoop.p(kP).i(kI).d(kD);
    KickConfig.closedLoop.p(kP).i(kI).d(kD);
    // dropper config
    Flywheel.configure(
        ShooterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    // For Elastic and Advantage
    SmartDashboard.putNumber("PID/Shooter/kP", kP);
    SmartDashboard.putNumber("PID/Shooter/kI", kI);
    SmartDashboard.putNumber("PID/Shooter/kD", kD);
    SmartDashboard.putNumber("PID/Shooter/Target RPM", 0.0);
    SmartDashboard.putNumber("PID/KickWheel/Target RPM", 0.0);
  }

  public Command shootCMD(double newTargetRPM) {
    /**
     * spin the flywheel untill it reaches speed once it reaches speed, activate the kicker wheel.
     * then it stops both
     */
    return Commands.none();
  }

  public Command PIDCMD(double newTargetRPM) {

    targetRPM = newTargetRPM;
    // SmartDashboard.putNumber("PID/Shooter/Target RPM", newTargetRPM);

    return runOnce(
        () -> {
          DriverStation.reportWarning("Shooter", false);
          shootController.setSetpoint(newTargetRPM, ControlType.kVelocity);
        });
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
    FlywheelSim.iterate(
        FlywheelSim.getSetpoint(),
        12, // Simulated battery voltage, in Volts
        0.02); // Time interval, in Seconds

    // SimBattery estimates loaded battery voltages
    // This should include all motors being simulated

  }
}
