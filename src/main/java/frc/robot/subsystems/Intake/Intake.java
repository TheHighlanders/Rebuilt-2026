// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase {
  SparkMax intakeMotor = new SparkMax(IntakeConstants.SPINTAKEID, MotorType.kBrushless);
  SparkMaxConfig config = new SparkMaxConfig();
  SparkClosedLoopController controller = intakeMotor.getClosedLoopController();

  double inSpeed = IntakeConstants.INTAKE_SPEED;
  double outSpeed = IntakeConstants.SPITAKE_SPEED;

  public Intake() {

    config.smartCurrentLimit(50).idleMode(IdleMode.kCoast);

    intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putNumber("INTAKE/in speed", inSpeed);
    SmartDashboard.putNumber("INTAKE/out speed", outSpeed);
  }

  // Intake commands to take in, spit out, and not move
  public Command intakeCMD() {
    // Takes in
    return runOnce(
        () -> {
          intakeMotor.set(IntakeConstants.INTAKE_SPEED);
        });
  }
  // Spits out
  public Command spitakeCMD() {
    return runOnce(
        () -> {
          intakeMotor.set(IntakeConstants.SPITAKE_SPEED);
        });
  }

  public Command stoptakeCMD() {
    // Stops motor
    return runOnce(
        () -> {
          intakeMotor.set(0);
        });
  }

  public Command mannualCMD(DoubleSupplier speed) {
    return run(() -> intakeMotor.set(speed.getAsDouble()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    inSpeed = SmartDashboard.getNumber("INTAKE/in speed", IntakeConstants.INTAKE_SPEED);
    outSpeed = SmartDashboard.getNumber("INTAKE/out speed", IntakeConstants.SPITAKE_SPEED);
  }
}
