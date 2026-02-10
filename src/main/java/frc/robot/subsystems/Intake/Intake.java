// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  SparkMax intakeMotor =
      new SparkMax(IntakeConstants.SPINTAKEID, MotorType.kBrushless); // 10, for now
  RelativeEncoder encoder;
  SparkMaxConfig config = new SparkMaxConfig();
  SparkClosedLoopController controller = intakeMotor.getClosedLoopController();

  public Intake() {

    encoder = intakeMotor.getEncoder();

    config.encoder.positionConversionFactor(1).velocityConversionFactor(1);

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(IntakeConstants.kP1)
        .i(IntakeConstants.kI1)
        .d(IntakeConstants.kD1)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(IntakeConstants.kP2, ClosedLoopSlot.kSlot1)
        .i(IntakeConstants.kI2, ClosedLoopSlot.kSlot1)
        .d(IntakeConstants.kD2, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
        .feedForward
        // kV is now in Volts, so we multiply by the nominal voltage (12V)
        .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);

    /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    config.smartCurrentLimit(50).idleMode(IdleMode.kBrake);

    intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
