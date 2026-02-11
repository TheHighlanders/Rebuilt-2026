// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
  /** Creates a new Hopper. */
  SparkMax Hopper = new SparkMax(Constants.HopperConstants.HOPPERID, MotorType.kBrushless);

  SparkMax Kickerwheel = new SparkMax(Constants.ShooterConstants.KICKERID, MotorType.kBrushless);

  SparkMaxSim KickerwheelSim = new SparkMaxSim(Kickerwheel, DCMotor.getNEO(1));

  RelativeEncoder kickEncoder;
  SparkClosedLoopController kickController = Kickerwheel.getClosedLoopController();

  double kP = 0.1;
  double kI = 0.0;
  double kD = 3.0;
  double kickTargetRPM = 2000.0;

  SparkMaxConfig KickConfig = new SparkMaxConfig();

  public Hopper() {
    SparkMaxConfig config = new SparkMaxConfig();
    kickEncoder = Kickerwheel.getEncoder();
    config.smartCurrentLimit(50).idleMode(IdleMode.kCoast);

    KickConfig.closedLoop.p(kP).i(kI).d(kD);

    // Persist parameters to retain configuration in the event of a power cycle
    Kickerwheel.configure(
        KickConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    Hopper.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  // spins the motor inside the hopper
  public Command SpinCMD() {
    return runOnce(
        () -> {
          Hopper.set(1);
          kickController.setSetpoint(kickTargetRPM, ControlType.kVelocity);
          // speed can be changed
          DriverStation.reportWarning("StartHopper", false);
        });
  }
  // stops the hopper
  public Command StopCMD() {
    return runOnce(
        () -> {
          Hopper.set(0);
          kickController.setSetpoint(0, ControlType.kVelocity);
          DriverStation.reportWarning("StopHopper", false);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)

    // Next, we update it. The standard loop time is 20ms.

    // Now, we update the Spark MAX
    KickerwheelSim.iterate(
        KickerwheelSim.getSetpoint(),
        12, // Simulated battery voltage, in Volts
        0.02); // Time interval, in Seconds

    // SimBattery estimates loaded battery voltages
    // This should include all motors being simulated

  }
}
