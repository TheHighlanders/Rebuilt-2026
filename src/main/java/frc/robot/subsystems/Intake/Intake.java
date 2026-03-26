// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  TalonFX intakeMotor = new TalonFX(IntakeConstants.SPINTAKEID);
  //SparkClosedLoopController controller = intakeMotor.getClosedLoopController();

  double inSpeed = IntakeConstants.INTAKE_SPEED;
  double outSpeed = IntakeConstants.SPITAKE_SPEED;

  public Intake() {

    intakeMotor.configure(
        getConfig(IdleMode.kCoast), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putNumber("INTAKE/in speed", inSpeed);
    SmartDashboard.putNumber("INTAKE/out speed", outSpeed);
    SmartDashboard.putString("INTAKE/State", "NONE");
  }

  private SparkMaxConfig getConfig(IdleMode idleMode) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(50).idleMode(idleMode);

    return config;
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
          intakeMotor.set(inSpeed);
        },
        this);
  }
  // Spits out
  public Command spitakeCMD() {
    return Commands.run(
        () -> {
          DriverStation.reportWarning("spitakeCMD", false);

          SmartDashboard.putString("INTAKE/State", "OUTTAKING");
          intakeMotor.set(outSpeed);
        },
        this);
  }

  public Command stoptakeCMD() {
    // Stops motor

    return Commands.runOnce(
        () -> {
          DriverStation.reportWarning("stoptakeCMD", false);

          SmartDashboard.putString("INTAKE/State", "STOPPED");
          intakeMotor.set(0);
        },
        this);
  }

  public Command killCMD() {
    return runOnce(
        () -> {
          intakeMotor.configure(
              getConfig(IdleMode.kBrake),
              ResetMode.kResetSafeParameters,
              PersistMode.kNoPersistParameters);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // disabled bc not sure what htis is doing //inSpeed = SmartDashboard.getNumber("INTAKE/in
    // speed", IntakeConstants.INTAKE_SPEED);
    outSpeed = SmartDashboard.getNumber("INTAKE/out speed", IntakeConstants.SPITAKE_SPEED);
    SmartDashboard.putNumber("INTAKE/Encoder Velocity", intakeMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("INTAKE/Encoder Pose", intakeMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("INTAKE/Current", intakeMotor.getOutputCurrent());
  }
}
