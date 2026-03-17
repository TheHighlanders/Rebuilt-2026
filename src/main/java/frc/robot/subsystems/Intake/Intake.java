// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;

public interface Intake {

  // Intake commands to take in, spit out, and not move
  public Command intakeCMD();
  // Spits out
  public Command spitakeCMD();

  public Command stoptakeCMD();

  public Command killCMD();

  public void periodic();
}
