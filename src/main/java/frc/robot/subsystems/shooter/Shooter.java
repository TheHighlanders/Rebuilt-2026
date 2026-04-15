// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.controller.PIDController;
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
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  TalonFX flywheel = new TalonFX(ShooterConstants.SHOOTERID);

  // Config for PID
  TalonFXConfiguration config = new TalonFXConfiguration();

  NeutralOut brake = new NeutralOut();
  PIDController controller =
      new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
  VelocityVoltage velocity = new VelocityVoltage(0).withSlot(0);

  CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();

  double targetRPS;

  public Shooter() {

    SmartDashboard.putNumber("Shooter/Mannual Target RPS", 0);
    SmartDashboard.putNumber("Shooter/Distance Tune", 1);

    config.Slot0.kP = ShooterConstants.kP;
    config.Slot0.kI = ShooterConstants.kI;
    config.Slot0.kD = ShooterConstants.kD;
    config.Slot0.kS = ShooterConstants.kS;
    config.Slot0.kV = ShooterConstants.kV;

    config.Voltage.withPeakForwardVoltage(12).withPeakReverseVoltage(-12);

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Feedback.SensorToMechanismRatio = ShooterConstants.GEAR_RATIO;

    currentLimits.StatorCurrentLimit = 80;
    currentLimits.SupplyCurrentLowerLimit = 80; // sus
    currentLimits.StatorCurrentLimitEnable = true;

    tryUntilOk(5, () -> flywheel.getConfigurator().apply(config));
    tryUntilOk(5, () -> flywheel.getConfigurator().apply(currentLimits));

    SmartDashboard.putNumber("Shooter/ Flywheel Target RPS", 0.0);
    SmartDashboard.putNumber(
        "Shooter/Flywheel Voltage", flywheel.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber(
        "Shooter/Flywheel Current", flywheel.getStatorCurrent().getValueAsDouble());
  }

  public double getTargetRPS() {
    return targetRPS;
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
    double linearVelocity =
        Math.sqrt(
                ShooterConstants.GRAVITY
                    * Math.pow(trajectory.getX(), 2)
                    / (2
                        * Math.pow(Math.cos(ShooterConstants.SHOOTER_HOOD.in(Radians)), 2)
                        * (trajectory.getX() * Math.tan(ShooterConstants.SHOOTER_HOOD.in(Radians))
                            - trajectory.getY())))
            + (trajectory.getX() / 25); // air resistance fudge factor works way too well

    double rotationalVelocity =
        linearVelocity / (ShooterConstants.FLYWHEEL_RADIUS.in(Meters) * 4 * Math.PI);
    return rotationalVelocity;
  }

  /*
   * finds desired angular velocity of the shooter to target a certain robot-relative point
   * using a physics simulation.
   */
  protected static double calculateRR(Translation2d trajectory) {
    return calculate(robotToShooterTraj(trajectory));
  }

  /*
   * finds desired angular velocity of the shooter to reach the ground a certain distance away
   * using a lookup table of values taken from the real robot.
   */
  protected double calculateGroundLookup(double distance) {
    // find position of data at around desired distance on lookup table
    SmartDashboard.putNumber("Shooter/Ground Calc'd Distance", distance);
    // if (distance > 500) return 18;

    // int index = 0;
    // for (double test : LookupTable.DISTS) {
    //   if (distance > test) index++;
    //   else break;
    // }
    // if (index >= LookupTable.DISTS.length) index = LookupTable.DISTS.length - 1;
    // if (index <= 0) index = 1;

    // double lowRPS = LookupTable.RPMS[index - 1];
    // double highRPS = LookupTable.RPMS[index];

    // double iterateProportion =
    //     (distance - LookupTable.DISTS[index - 1])
    //         / (LookupTable.DISTS[index] - LookupTable.DISTS[index - 1]);

    // double targetRPSLookup = lowRPS + ((highRPS - lowRPS) * iterateProportion);

    return LookupTable.rpmFromRegression(distance);
  }

  /*
   * finds desired angular velocity of the shooter to target a certain shooter-relative point
   * using a lookup table of values taken from the real robot.
   */
  protected double calculateLookup(Translation2d trajectory) {
    // detecting a shot too steep.
    // We multiply by two since sometimes the trajectory hits the target pose while the ball is
    // still going up.
    if (2 * trajectory.getY() / trajectory.getX() > Math.tan(1.36135682))
      return calculateLookup(
          new Translation2d(2 * trajectory.getY() / Math.tan(1.36135682), trajectory.getY()));

    // https://www.desmos.com/calculator/jceelcasyl

    double fallrate =
        (trajectory.getY()
                - (Math.tan(ShooterConstants.SHOOTER_HOOD.in(Radians)) * trajectory.getX()))
            / (trajectory.getX() * trajectory.getX());

    return calculateGroundLookup(
        (-Math.tan(ShooterConstants.SHOOTER_HOOD.in(Radians))
                - Math.sqrt(
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

  public boolean atSpeed() {
    return Math.abs(flywheel.getVelocity().getValueAsDouble() - targetRPS) < 0.4;
  }

  public Command flywheelCMD(Supplier<Translation2d> shotPoint) {
    return Commands.run(
        () -> {
          targetRPS = calculateRRLookup(shotPoint.get());
          flywheel.setControl(velocity.withVelocity(targetRPS));
          SmartDashboard.putNumber("Shooter/Shot Distance", shotPoint.get().getX());
        },
        this);
  }

  public Command flywheelGndCMD(DoubleSupplier distance) {
    return flywheelCMD(() -> new Translation2d(distance.getAsDouble(), 0));
  }

  public Command flywheelHubCMD(DoubleSupplier distance) {
    return flywheelCMD(() -> new Translation2d(distance.getAsDouble(), FieldConstants.HUB_HEIGHT));
  }

  public Command rawFlywheelCMD(DoubleSupplier drive) {
    return Commands.run(
            () -> {
              flywheel.setVoltage(drive.getAsDouble() * 12);
            },
            this)
        .withName("Raw Flywheel");
  }

  public Command tuneCMD() {
    return Commands.run(
            () -> {
              targetRPS = SmartDashboard.getNumber("Shooter/Mannual Target RPS", 0);
              flywheel.setControl(velocity.withVelocity(targetRPS)); // .setVoltage(voltage);
            },
            this)
        .withName("Tuning");
  }

  public Command stopCMD() {
    return Commands.runOnce(
            () -> {
              flywheel.setControl(brake);
              targetRPS = 0;
            },
            this)
        .withName("Stop");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
        "Shooter/Flywheel Voltage", flywheel.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber(
        "Shooter/Flywheel Current", flywheel.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Flywheel Target RPS", targetRPS);
    SmartDashboard.putNumber("Shooter/Flywheel RPS", flywheel.getVelocity().getValueAsDouble());
    SmartDashboard.putString(
        "Shooter/Current Shooter Command",
        this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
  }
}
