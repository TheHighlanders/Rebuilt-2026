// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeTalon extends SubsystemBase implements Intake {
  TalonFX intake = new TalonFX(IntakeConstants.SPINTAKEID_KRAKEN);

  double inSpeed = IntakeConstants.INTAKE_SPEED;
  double outSpeed = IntakeConstants.SPITAKE_SPEED;

  TalonFXConfiguration config = new TalonFXConfiguration();

  StaticBrake brake = new StaticBrake();
  CoastOut coast = new CoastOut();
  VoltageOut forward = new VoltageOut(2.5);
  VoltageOut back = new VoltageOut(-3);

  public IntakeTalon() {

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    tryUntilOk(5, () -> intake.getConfigurator().apply(config));

    SmartDashboard.putNumber("INTAKE/in speed", inSpeed);
    SmartDashboard.putNumber("INTAKE/out speed", outSpeed);
    SmartDashboard.putString("INTAKE/State", "NONE");
  }
  // Intake commands to take in, spit out, and not move
  public Command intakeCMD() {
    // Takes in
    return Commands.run(
        () -> {
          DriverStation.reportWarning("intakeCMD", false);

          // intakeMotor.configure(
          //     getConfig(IdleMode.kCoast),
          //     ResetMode.kResetSafeParameters,
          //     PersistMode.kNoPersistParameters);
          SmartDashboard.putString("INTAKE/State", "INTAKING");
          intake.setVoltage(2.5);
        },
        this);
  }
  // Spits out
  public Command spitakeCMD() {
    return Commands.run(
        () -> {
          DriverStation.reportWarning("spitakeCMD", false);

          SmartDashboard.putString("INTAKE/State", "OUTTAKING");
          intake.setVoltage(-3);
        },
        this);
  }

  public Command stoptakeCMD() {
    // Stops motor

    return Commands.runOnce(
        () -> {
          DriverStation.reportWarning("stoptakeCMD", false);

          SmartDashboard.putString("INTAKE/State", "STOPPED");
          intake.setControl(coast);
        },
        this);
  }

  public Command killCMD() {
    return runOnce(
        () -> {
          intake.setControl(brake);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // disabled bc not sure what htis is doing //inSpeed = SmartDashboard.getNumber("INTAKE/in
    // speed", IntakeConstants.INTAKE_SPEED);
    outSpeed = SmartDashboard.getNumber("INTAKE/out speed", IntakeConstants.SPITAKE_SPEED);
    SmartDashboard.putNumber("INTAKE/Encoder Velocity", intake.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("INTAKE/Current", intake.getStatorCurrent().getValueAsDouble());
  }
}
