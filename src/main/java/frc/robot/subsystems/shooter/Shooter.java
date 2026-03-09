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
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.LookupTable;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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

  /*
   * compensates for the shooter's position on the robot based on a flat robot-relative trajectory to the target.
   */
  private static Translation2d robotToShooterTraj(Translation2d trajectory) {
    Translation3d robotToHub = new Translation3d(trajectory.getX(), 0, trajectory.getY());
    Translation3d shooterToHub =
        robotToHub.minus(
            ShooterConstants.SHOOTER_RR_POS.rotateBy(
                new Rotation3d(0, 0, DriveConstants.ALIGN_SHOOTER_COMP.in(Radians))));
    return new Translation2d(
        Math.hypot(shooterToHub.getX(), shooterToHub.getY()), shooterToHub.getZ());
  }

  /*
   * finds desired angular velocity of the shooter to target a certain shooter-relative point
   * using a physics simulation.
   */
  protected static double calculate(Translation2d trajectory) {
    // TODO: tune.
    double linearVelocity =
        Math.sqrt(
            ShooterConstants.GRAVITY
                * Math.pow(trajectory.getX(), 2)
                / (2
                    * Math.pow(Math.cos(ShooterConstants.SHOOTER_HOOD.in(Radians)), 2)
                    * (trajectory.getX() * Math.tan(ShooterConstants.SHOOTER_HOOD.in(Radians))
                        - trajectory.getY())))
         + (trajectory.getX() / 25); // air resistance fudge factor works way too well

    return linearVelocity / (ShooterConstants.FLYWHEEL_RADIUS.in(Meters) * 2 * Math.PI);
  }

  /*
   * finds desired angular velocity of the shooter to target a certain robot-relative point
   * using a physics simulation.
   */
  protected static double calculateRR(Translation2d trajectory) {
    return calculate(robotToShooterTraj(trajectory));
  }

  /*
   * finds desired angular velocity of the shooter to reach the hub from a certain distance away
   * using a physics simulation.
   */
  protected double calculateRRHub(double distance) {
    return calculateRR(new Translation2d(distance, FieldConstants.HUB_HEIGHT));
  }

  /*
   * finds desired angular velocity of the shooter to reach the ground a certain distance away
   * using a physics simulation.
   */
  protected double calculateRRGround(double distance) {
    return calculateRR(new Translation2d(distance, 0));
  }

  /*
   * finds desired angular velocity of the shooter to reach the ground a certain distance away
   * using a lookup table of values taken from the real robot.
   */
  protected double calculateGroundLookup(double distance) {
    // find position of data at around desired distance on lookup table
    int index = 0;
    for (double test : LookupTable.DISTS) {
      if (distance > test) index++;
      else break;
    }
    int range = (int) Math.sqrt(LookupTable.DISTS.length / 4);

    // data limits
    if (index + range > LookupTable.DISTS.length) index = LookupTable.DISTS.length - range;

    if (index < Math.sqrt(LookupTable.DISTS.length / 4)) index = range;

    // find neigboring data
    double[] neighborsX = Arrays.copyOfRange(LookupTable.DISTS, index - range, index + range);

    double[] neighborsY = Arrays.copyOfRange(LookupTable.RPMS, index - range, index + range);

    /* FIND REGRESSION FOR NEIGHBORING DATA */
    // x mean
    double xMean = 0;
    for (double d : neighborsX) xMean += d;
    xMean /= neighborsX.length;

    // x standard deviation
    double xStdDev = 0;
    for (double d : neighborsX) xStdDev += Math.pow(d - xMean, 2);
    xStdDev = Math.sqrt(xStdDev / neighborsX.length);

    // y mean
    double yMean = 0;
    for (double d : neighborsY) yMean += d;
    yMean /= neighborsY.length;

    // y standard deviation
    double yStdDev = 0;
    for (double d : neighborsY) yStdDev += Math.pow(d - yMean, 2);
    yStdDev = Math.sqrt(yStdDev / neighborsY.length);

    // slope
    double r = 0;
    for (int i = 0; i < neighborsX.length; i++) {
      r += (neighborsX[i] - xMean) / xStdDev * (neighborsY[i] - yMean) / yStdDev;
    }
    r /= neighborsX.length - 1;
    double slope = r * (yStdDev / xStdDev);

    // regression
    return yMean + (slope * (distance - xMean));
  }

  /*
   * finds desired angular velocity of the shooter to target a certain shooter-relative point
   * using a lookup table of values taken from the real robot.
   */
  protected double calculateLookup(Translation2d trajectory) {
    double fallrate =
        (trajectory.getY()
                - Math.tan(ShooterConstants.SHOOTER_HOOD.in(Radians)) * trajectory.getX())
            / (trajectory.getX() * trajectory.getX());
    return calculateGroundLookup(
        (-Math.tan(ShooterConstants.SHOOTER_HOOD.in(Radians))
                * Math.sqrt(
                    Math.pow(Math.tan(ShooterConstants.SHOOTER_HOOD.in(Radians)), 2)
                        - (4 * ShooterConstants.SHOOTER_RR_POS.getZ() * fallrate)))
            / (2 * fallrate));
  }

  /*
   * finds desired angular velocity of the shooter to target a certain robot-relative point
   * using a lookup table of values taken from the real robot.
   */
  protected double calculateRRLookup(Translation2d trajectory) {
    return calculateLookup(robotToShooterTraj(trajectory));
  }

  /*
   * finds desired angular velocity of the shooter to reach the hub from a certain distance away
   * using a physics simulation.
   */
  protected double calculateHubRRLookup(double distance) {
    return calculateRRLookup(new Translation2d(distance, FieldConstants.HUB_HEIGHT));
  }

  public boolean atSpeed() {
    return Math.abs(flywheel.getVelocity().getValueAsDouble() - targetRPS) < 0.4;
  }

  public Command flywheelCMD(Supplier<Translation2d> distance) {
    return Commands.run(
        () -> {
          targetRPS = calculateRR(distance.get());
          flywheel.setControl(velocityVoltage.withVelocity(targetRPS));
          SmartDashboard.putNumber(
              "Shooter/Flywheel/Voltage", flywheel.getMotorVoltage().getValueAsDouble());
          SmartDashboard.putNumber(
              "Shooter/Flywheel/Current", flywheel.getStatorCurrent().getValueAsDouble());
          SmartDashboard.putNumber("Shooter/Target RPS", targetRPS);
          SmartDashboard.putNumber(
              "Shooter/Flywheel RPS", flywheel.getVelocity().getValueAsDouble());
          SmartDashboard.putNumber("Shooter/Distance", distance.get().getX());
        },
        this);
  }

  public Command flywheelGndCMD(DoubleSupplier distance) {
    return Commands.run(
        () -> {
          targetRPS = calculateRRGround(distance.getAsDouble());
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

  public Command flywheelHubCMD(DoubleSupplier distance) {
    return Commands.run(
        () -> {
          targetRPS = calculateRRHub(distance.getAsDouble());
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

  public Command flywheelHubCMD(Object object) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'flywheelHubCMD'");
  }
}
