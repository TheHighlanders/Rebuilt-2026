// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import java.util.function.DoubleSupplier;

public class Deploy extends SubsystemBase {
  /** Creates a new Deploy. */
  SparkMax deployMotor = new SparkMax(IntakeConstants.DEPLOYID, MotorType.kBrushless);

  SparkClosedLoopController closedLoopController = deployMotor.getClosedLoopController();
  RelativeEncoder deployEncoder = deployMotor.getEncoder();
  ProfiledPIDController controller;

  SparkMaxConfig config = new SparkMaxConfig();
  double p = IntakeConstants.kP;
  double i = IntakeConstants.kI;
  double d = IntakeConstants.kD;
  double s = IntakeConstants.kS;
  double g = IntakeConstants.kG;
  double v = IntakeConstants.kV;
  double rest = 0;

  public Deploy() {
    config.smartCurrentLimit(50).idleMode(IdleMode.kBrake);
    config.encoder.positionConversionFactor(IntakeConstants.DEPLOY_RATIO);

    controller = new ProfiledPIDController(p, i, d, new TrapezoidProfile.Constraints(1.75, 0.75));

    // .kCosRatio(IntakeConstants.DEPLOY_RATIO);
    deployMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    SmartDashboard.putNumber("INTAKE/Deploy Encoder", deployEncoder.getPosition());

    deployEncoder.setPosition(1);
    SmartDashboard.putNumber("INTAKE/rest speed", rest);
  }

  public Command deployCMD() {
    return run(
        () -> {
          deployMotor.set(0.2);
        });
  }

  public Command undeployCMD() {
    return Commands.sequence(
      Commands.runOnce(
        () -> {
          deployMotor.set(-0.3);
        }),
      Commands.waitSeconds(0.5),
      Commands.runOnce(() -> {
        deployMotor.set(-0.02);
      }));

  }

  public Command mannualCMD(DoubleSupplier speed) {
    return run(() -> deployMotor.set(-0.3 * speed.getAsDouble()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    s = SmartDashboard.getNumber("INTAKE/kS", IntakeConstants.kS);
    g = SmartDashboard.getNumber("INTAKE/kG", IntakeConstants.kG);
    v = SmartDashboard.getNumber("INTAKE/kV", IntakeConstants.kV);
    p = SmartDashboard.getNumber("INTAKE/kP", IntakeConstants.kP);
    i = SmartDashboard.getNumber("INTAKE/kI", IntakeConstants.kI);
    d = SmartDashboard.getNumber("INTAKE/kD", IntakeConstants.kD);
    rest = SmartDashboard.getNumber("INTAKE/rest speed", rest);
    SmartDashboard.putNumber("INTAKE/Deploy/Deploy Encoder", deployEncoder.getPosition());
    SmartDashboard.putNumber("INTAKE/Deploy/Deploy Encoder Velocity", deployEncoder.getVelocity());
    SmartDashboard.putNumber("INTAKE/Deploy/Deploy Current", deployMotor.getOutputCurrent());

    SmartDashboard.putString(
        "INTAKE/Deploy State",
        getCurrentCommand() == null ? "NONE" : getCurrentCommand().getName());
  }
}
